/*
 * QEMU PowerPC sPAPR IRQ interface
 *
 * Copyright (c) 2018, IBM Corporation.
 *
 * This code is licensed under the GPL version 2 or later. See the
 * COPYING file in the top-level directory.
 */

#include "qemu/osdep.h"
#include "qemu/log.h"
#include "qemu/error-report.h"
#include "qapi/error.h"
#include "hw/ppc/spapr.h"
#include "hw/ppc/spapr_xive.h"
#include "hw/ppc/xics.h"
#include "sysemu/kvm.h"

#include "trace.h"

void spapr_irq_msi_init(sPAPRMachineState *spapr, uint32_t nr_msis)
{
    spapr->irq_map_nr = nr_msis;
    spapr->irq_map = bitmap_new(spapr->irq_map_nr);
}

int spapr_irq_msi_alloc(sPAPRMachineState *spapr, uint32_t num, bool align,
                        Error **errp)
{
    int irq;

    /*
     * The 'align_mask' parameter of bitmap_find_next_zero_area()
     * should be one less than a power of 2; 0 means no
     * alignment. Adapt the 'align' value of the former allocator
     * to fit the requirements of bitmap_find_next_zero_area()
     */
    align -= 1;

    irq = bitmap_find_next_zero_area(spapr->irq_map, spapr->irq_map_nr, 0, num,
                                     align);
    if (irq == spapr->irq_map_nr) {
        error_setg(errp, "can't find a free %d-IRQ block", num);
        return -1;
    }

    bitmap_set(spapr->irq_map, irq, num);

    return irq + SPAPR_IRQ_MSI;
}

void spapr_irq_msi_free(sPAPRMachineState *spapr, int irq, uint32_t num)
{
    bitmap_clear(spapr->irq_map, irq - SPAPR_IRQ_MSI, num);
}

void spapr_irq_msi_reset(sPAPRMachineState *spapr)
{
    bitmap_clear(spapr->irq_map, 0, spapr->irq_map_nr);
}


/*
 * XICS IRQ backend.
 */

static ICSState *spapr_ics_create(sPAPRMachineState *spapr,
                                  const char *type_ics,
                                  int nr_irqs, Error **errp)
{
    Error *local_err = NULL;
    Object *obj;

    obj = object_new(type_ics);
    object_property_add_child(OBJECT(spapr), "ics", obj, &error_abort);
    object_property_add_const_link(obj, ICS_PROP_XICS, OBJECT(spapr),
                                   &error_abort);
    object_property_set_int(obj, nr_irqs, "nr-irqs", &local_err);
    if (local_err) {
        goto error;
    }
    object_property_set_bool(obj, true, "realized", &local_err);
    if (local_err) {
        goto error;
    }

    return ICS_BASE(obj);

error:
    error_propagate(errp, local_err);
    return NULL;
}

static void spapr_irq_init_xics(sPAPRMachineState *spapr, uint32_t nr_servers,
                                Error **errp)
{
    MachineState *machine = MACHINE(spapr);
    sPAPRMachineClass *smc = SPAPR_MACHINE_GET_CLASS(spapr);
    int nr_irqs = smc->irq->nr_irqs;
    Error *local_err = NULL;

    /* Initialize the MSI IRQ allocator. */
    if (!SPAPR_MACHINE_GET_CLASS(spapr)->legacy_irq_allocation) {
        spapr_irq_msi_init(spapr, smc->irq->nr_msis);
    }

    if (kvm_enabled()) {
        if (machine_kernel_irqchip_allowed(machine) &&
            !xics_kvm_init(spapr, &local_err)) {
            spapr->icp_type = TYPE_KVM_ICP;
            spapr->ics = spapr_ics_create(spapr, TYPE_ICS_KVM, nr_irqs,
                                          &local_err);
        }
        if (machine_kernel_irqchip_required(machine) && !spapr->ics) {
            error_prepend(&local_err,
                          "kernel_irqchip requested but unavailable: ");
            goto error;
        }
        error_free(local_err);
        local_err = NULL;
    }

    if (!spapr->ics) {
        xics_spapr_init(spapr);
        spapr->icp_type = TYPE_ICP;
        spapr->ics = spapr_ics_create(spapr, TYPE_ICS_SIMPLE, nr_irqs,
                                      &local_err);
    }

error:
    error_propagate(errp, local_err);
}

#define ICS_IRQ_FREE(ics, srcno)   \
    (!((ics)->irqs[(srcno)].flags & (XICS_FLAGS_IRQ_MASK)))

static int spapr_irq_claim_xics(sPAPRMachineState *spapr, int irq, bool lsi,
                                Error **errp)
{
    ICSState *ics = spapr->ics;

    assert(ics);

    if (!ics_valid_irq(ics, irq)) {
        error_setg(errp, "IRQ %d is invalid", irq);
        return -1;
    }

    if (!ICS_IRQ_FREE(ics, irq - ics->offset)) {
        error_setg(errp, "IRQ %d is not free", irq);
        return -1;
    }

    ics_set_irq_type(ics, irq - ics->offset, lsi);
    return 0;
}

static void spapr_irq_free_xics(sPAPRMachineState *spapr, int irq, int num)
{
    ICSState *ics = spapr->ics;
    uint32_t srcno = irq - ics->offset;
    int i;

    if (ics_valid_irq(ics, irq)) {
        trace_spapr_irq_free(0, irq, num);
        for (i = srcno; i < srcno + num; ++i) {
            if (ICS_IRQ_FREE(ics, i)) {
                trace_spapr_irq_free_warn(0, i);
            }
            memset(&ics->irqs[i], 0, sizeof(ICSIRQState));
        }
    }
}

static qemu_irq spapr_qirq_xics(sPAPRMachineState *spapr, int irq)
{
    ICSState *ics = spapr->ics;
    uint32_t srcno = irq - ics->offset;

    if (ics_valid_irq(ics, irq)) {
        return ics->qirqs[srcno];
    }

    return NULL;
}

static void spapr_irq_print_info_xics(sPAPRMachineState *spapr, Monitor *mon)
{
    CPUState *cs;

    CPU_FOREACH(cs) {
        PowerPCCPU *cpu = POWERPC_CPU(cs);

        icp_pic_print_info(ICP(cpu->intc), mon);
    }

    ics_pic_print_info(spapr->ics, mon);
}

static void spapr_irq_dt_populate_xics(sPAPRMachineState *spapr,
                                       uint32_t nr_servers, void *fdt,
                                       uint32_t phandle)
{
    spapr_dt_xics(nr_servers, fdt, phandle);
}

static Object *spapr_irq_cpu_intc_create_xics(sPAPRMachineState *spapr,
                                              Object *cpu, Error **errp)
{
    return icp_create(cpu, spapr->icp_type, XICS_FABRIC(spapr), errp);
}

#define SPAPR_IRQ_XICS_NR_IRQS     0x1000
#define SPAPR_IRQ_XICS_NR_MSIS     \
    (XICS_IRQ_BASE + SPAPR_IRQ_XICS_NR_IRQS - SPAPR_IRQ_MSI)

sPAPRIrq spapr_irq_xics = {
    .nr_irqs     = SPAPR_IRQ_XICS_NR_IRQS,
    .nr_msis     = SPAPR_IRQ_XICS_NR_MSIS,
    .ov5         = 0x0, /* XICS only */

    .init        = spapr_irq_init_xics,
    .claim       = spapr_irq_claim_xics,
    .free        = spapr_irq_free_xics,
    .qirq        = spapr_qirq_xics,
    .print_info  = spapr_irq_print_info_xics,
    .dt_populate = spapr_irq_dt_populate_xics,
    .cpu_intc_create = spapr_irq_cpu_intc_create_xics,
};

/*
 * XIVE IRQ backend.
 */
static sPAPRXive *spapr_xive_create(sPAPRMachineState *spapr,
                                    const char *type_xive, int nr_irqs,
                                    int nr_servers, Error **errp)
{
    sPAPRXive *xive;
    Error *local_err = NULL;
    Object *obj;
    uint32_t nr_ends = nr_servers << 3; /* 8 priority ENDs per CPU */
    int i;

    obj = object_new(type_xive);
    object_property_set_int(obj, nr_irqs, "nr-irqs", &error_abort);
    object_property_set_int(obj, nr_ends, "nr-ends", &error_abort);
    object_property_set_bool(obj, true, "realized", &local_err);
    if (local_err) {
        error_propagate(errp, local_err);
        return NULL;
    }
    qdev_set_parent_bus(DEVICE(obj), sysbus_get_default());
    xive = SPAPR_XIVE(obj);

    /* Enable the CPU IPIs */
    for (i = 0; i < nr_servers; ++i) {
        spapr_xive_irq_enable(xive, SPAPR_IRQ_IPI + i, false);
    }

    return xive;
}

static void spapr_irq_init_xive(sPAPRMachineState *spapr, uint32_t nr_servers,
                                Error **errp)
{
    MachineState *machine = MACHINE(spapr);
    sPAPRMachineClass *smc = SPAPR_MACHINE_GET_CLASS(machine);
    int nr_irqs = smc->irq->nr_irqs;
    Error *local_err = NULL;

    /* Initialize the MSI IRQ allocator */
    spapr_irq_msi_init(spapr, smc->irq->nr_msis);

    /* KVM XIVE support */
    if (kvm_enabled()) {
        if (machine_kernel_irqchip_required(machine)) {
            error_setg(errp, "kernel_irqchip requested. no XIVE support");
            return;
        }
    }

    /* QEMU XIVE support */
    spapr->xive = spapr_xive_create(spapr, TYPE_SPAPR_XIVE, nr_irqs, nr_servers,
                                    &local_err);
    if (local_err) {
        error_propagate(errp, local_err);
        return;
    }

    spapr->xive_tctx_type = TYPE_XIVE_TCTX;
    spapr_xive_hcall_init(spapr);
}

static int spapr_irq_claim_xive(sPAPRMachineState *spapr, int irq, bool lsi,
                                Error **errp)
{
    if (!spapr_xive_irq_enable(spapr->xive, irq, lsi)) {
        error_setg(errp, "IRQ %d is invalid", irq);
        return -1;
    }
    return 0;
}

static void spapr_irq_free_xive(sPAPRMachineState *spapr, int irq, int num)
{
    int i;

    for (i = irq; i < irq + num; ++i) {
        spapr_xive_irq_disable(spapr->xive, i);
    }
}

static qemu_irq spapr_qirq_xive(sPAPRMachineState *spapr, int irq)
{
    return spapr_xive_qirq(spapr->xive, irq);
}

static void spapr_irq_print_info_xive(sPAPRMachineState *spapr,
                                      Monitor *mon)
{
    CPUState *cs;

    CPU_FOREACH(cs) {
        PowerPCCPU *cpu = POWERPC_CPU(cs);

        xive_tctx_pic_print_info(XIVE_TCTX(cpu->intc), mon);
    }

    spapr_xive_pic_print_info(spapr->xive, mon);
}

static void spapr_irq_dt_populate_xive(sPAPRMachineState *spapr,
                                       uint32_t nr_servers, void *fdt,
                                       uint32_t phandle)
{
    spapr_dt_xive(spapr->xive, nr_servers, fdt, phandle);
}

static Object *spapr_irq_cpu_intc_create_xive(sPAPRMachineState *spapr,
                                              Object *cpu, Error **errp)
{
    return xive_tctx_create(cpu, spapr->xive_tctx_type,
                            XIVE_ROUTER(spapr->xive), errp);
}

/*
 * XIVE uses the full IRQ number space. Set it to 8K to be compatible
 * with XICS.
 */

#define SPAPR_IRQ_XIVE_NR_IRQS     0x2000
#define SPAPR_IRQ_XIVE_NR_MSIS     (SPAPR_IRQ_XIVE_NR_IRQS - SPAPR_IRQ_MSI)

sPAPRIrq spapr_irq_xive = {
    .nr_irqs     = SPAPR_IRQ_XIVE_NR_IRQS,
    .nr_msis     = SPAPR_IRQ_XIVE_NR_MSIS,
    .ov5         = 0x40, /* XIVE exploitation mode only */

    .init        = spapr_irq_init_xive,
    .claim       = spapr_irq_claim_xive,
    .free        = spapr_irq_free_xive,
    .qirq        = spapr_qirq_xive,
    .print_info  = spapr_irq_print_info_xive,
    .dt_populate = spapr_irq_dt_populate_xive,
    .cpu_intc_create = spapr_irq_cpu_intc_create_xive,
};

/*
 * sPAPR IRQ frontend routines for devices
 */

int spapr_irq_claim(sPAPRMachineState *spapr, int irq, bool lsi, Error **errp)
{
    sPAPRMachineClass *smc = SPAPR_MACHINE_GET_CLASS(spapr);

    return smc->irq->claim(spapr, irq, lsi, errp);
}

void spapr_irq_free(sPAPRMachineState *spapr, int irq, int num)
{
    sPAPRMachineClass *smc = SPAPR_MACHINE_GET_CLASS(spapr);

    smc->irq->free(spapr, irq, num);
}

qemu_irq spapr_qirq(sPAPRMachineState *spapr, int irq)
{
    sPAPRMachineClass *smc = SPAPR_MACHINE_GET_CLASS(spapr);

    return smc->irq->qirq(spapr, irq);
}

/*
 * XICS legacy routines - to deprecate one day
 */

static int ics_find_free_block(ICSState *ics, int num, int alignnum)
{
    int first, i;

    for (first = 0; first < ics->nr_irqs; first += alignnum) {
        if (num > (ics->nr_irqs - first)) {
            return -1;
        }
        for (i = first; i < first + num; ++i) {
            if (!ICS_IRQ_FREE(ics, i)) {
                break;
            }
        }
        if (i == (first + num)) {
            return first;
        }
    }

    return -1;
}

int spapr_irq_find(sPAPRMachineState *spapr, int num, bool align, Error **errp)
{
    ICSState *ics = spapr->ics;
    int first = -1;

    assert(ics);

    /*
     * MSIMesage::data is used for storing VIRQ so
     * it has to be aligned to num to support multiple
     * MSI vectors. MSI-X is not affected by this.
     * The hint is used for the first IRQ, the rest should
     * be allocated continuously.
     */
    if (align) {
        assert((num == 1) || (num == 2) || (num == 4) ||
               (num == 8) || (num == 16) || (num == 32));
        first = ics_find_free_block(ics, num, num);
    } else {
        first = ics_find_free_block(ics, num, 1);
    }

    if (first < 0) {
        error_setg(errp, "can't find a free %d-IRQ block", num);
        return -1;
    }

    return first + ics->offset;
}

#define SPAPR_IRQ_XICS_LEGACY_NR_IRQS     0x400

sPAPRIrq spapr_irq_xics_legacy = {
    .nr_irqs     = SPAPR_IRQ_XICS_LEGACY_NR_IRQS,
    .nr_msis     = SPAPR_IRQ_XICS_LEGACY_NR_IRQS,
    .ov5         = 0x0, /* XICS only */

    .init        = spapr_irq_init_xics,
    .claim       = spapr_irq_claim_xics,
    .free        = spapr_irq_free_xics,
    .qirq        = spapr_qirq_xics,
    .print_info  = spapr_irq_print_info_xics,
    .dt_populate = spapr_irq_dt_populate_xics,
    .cpu_intc_create = spapr_irq_cpu_intc_create_xics,
};
