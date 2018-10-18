/*
 * QEMU PowerPC sPAPR XIVE interrupt controller model
 *
 * Copyright (c) 2017-2018, IBM Corporation.
 *
 * This code is licensed under the GPL version 2 or later. See the
 * COPYING file in the top-level directory.
 */

#include "qemu/osdep.h"
#include "qemu/log.h"
#include "qemu/error-report.h"
#include "qapi/error.h"
#include "target/ppc/cpu.h"
#include "sysemu/cpus.h"
#include "sysemu/kvm.h"
#include "hw/ppc/spapr.h"
#include "hw/ppc/spapr_xive.h"
#include "hw/ppc/xive.h"
#include "kvm_ppc.h"

#include <sys/ioctl.h>

#define OPAL_STATE_VERSION	0x00010001
#define OPAL_STATE_COMPAT(v)	((v) & 0xffff)

/*
 * Helpers for CPU hotplug
 */
typedef struct KVMEnabledCPU {
    unsigned long vcpu_id;
    QLIST_ENTRY(KVMEnabledCPU) node;
} KVMEnabledCPU;

static QLIST_HEAD(, KVMEnabledCPU)
    kvm_enabled_cpus = QLIST_HEAD_INITIALIZER(&kvm_enabled_cpus);

static bool kvm_cpu_is_enabled(CPUState *cs)
{
    KVMEnabledCPU *enabled_cpu;
    unsigned long vcpu_id = kvm_arch_vcpu_id(cs);

    QLIST_FOREACH(enabled_cpu, &kvm_enabled_cpus, node) {
        if (enabled_cpu->vcpu_id == vcpu_id) {
            return true;
        }
    }
    return false;
}

static void kvm_cpu_enable(CPUState *cs)
{
    KVMEnabledCPU *enabled_cpu;
    unsigned long vcpu_id = kvm_arch_vcpu_id(cs);

    enabled_cpu = g_malloc(sizeof(*enabled_cpu));
    enabled_cpu->vcpu_id = vcpu_id;
    QLIST_INSERT_HEAD(&kvm_enabled_cpus, enabled_cpu, node);
}

/*
 * XIVE Thread Interrupt Management context (KVM)
 */
static int xive_tctx_kvm_get_state(XiveTCTX *tctx)
{
    uint64_t state[4] = { 0 };
    int ret;

    ret = kvm_get_one_reg(tctx->cs, KVM_REG_PPC_NVT_STATE, state);
    if (ret != 0) {
        error_report("Unable to retrieve KVM XIVE interrupt controller state"
                     " for CPU %ld: %s", kvm_arch_vcpu_id(tctx->cs),
                     strerror(errno));
        return ret;
    }

    /* First quad is word0 and word1 of the OS ring. */
    *((uint64_t *) &tctx->regs[TM_QW1_OS]) = state[0];

    /*
     * KVM also returns word2 containing the VP CAM line value which
     * is interesting to print out the VP identifer in the QEMU
     * monitor but we don't need to restore it.
     */
    *((uint64_t *) &tctx->regs[TM_QW1_OS + TM_WORD2]) = state[1];

    /* Last quads are the OPAL internal state which holds word4 of the
     * NVT structure. We are only interested by the IPB in there but
     * we should consider it as opaque.
     */
    tctx->opal[0] = state[2];
    tctx->opal[1] = state[3];
    return 0;
}

static void xive_tctx_kvm_do_synchronize_state(CPUState *cpu,
                                              run_on_cpu_data arg)
{
    xive_tctx_kvm_get_state(arg.host_ptr);
}

static void xive_tctx_kvm_synchronize_state(XiveTCTX *tctx)
{
    run_on_cpu(tctx->cs, xive_tctx_kvm_do_synchronize_state,
               RUN_ON_CPU_HOST_PTR(tctx));
}

static int xive_tctx_kvm_post_load(XiveTCTX *tctx, int version_id)
{
    uint64_t state[4];
    int ret;

    /* word0 and word1 of the OS ring. */
    state[0] = *((uint64_t *) &tctx->regs[TM_QW1_OS]);

    /* VP identifier. Only for debug */
    state[1] = *((uint64_t *) &tctx->regs[TM_QW1_OS + TM_WORD2]);

    /* OPAL internal state */
    state[2] = tctx->opal[0];
    state[3] = tctx->opal[1];

    ret = kvm_set_one_reg(tctx->cs, KVM_REG_PPC_NVT_STATE, state);
    if (ret != 0) {
        error_report("Unable to restore KVM XIVE interrupt controller state"
                     " for CPU %ld: %s", kvm_arch_vcpu_id(tctx->cs),
                     strerror(errno));
    }
    return ret;
}

static void xive_tctx_kvm_realize(DeviceState *dev, Error **errp)
{
    XiveTCTX *tctx = XIVE_TCTX_KVM(dev);
    XiveTCTXClass *xtc = XIVE_TCTX_BASE_GET_CLASS(dev);
    sPAPRXive *xive;
    unsigned long vcpu_id;
    int ret;
    Error *local_err = NULL;

    xtc->parent_realize(dev, &local_err);
    if (local_err) {
        error_propagate(errp, local_err);
        return;
    }

    vcpu_id = kvm_arch_vcpu_id(tctx->cs);
    xive = SPAPR_XIVE_KVM(tctx->xrtr);

    /* Check if CPU was hot unplugged and replugged. */
    if (kvm_cpu_is_enabled(tctx->cs)) {
        return;
    }

    ret = kvm_vcpu_enable_cap(tctx->cs, KVM_CAP_PPC_IRQ_XIVE, 0, xive->fd,
                              vcpu_id, 0);
    if (ret < 0) {
        error_setg(errp, "Unable to connect CPU%ld to KVM XIVE device: %s",
                   vcpu_id, strerror(errno));
        return;
    }

    kvm_cpu_enable(tctx->cs);
}

static void xive_tctx_kvm_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    XiveTCTXClass *xtc = XIVE_TCTX_BASE_CLASS(klass);

    dc->desc = "sPAPR XIVE KVM Interrupt Thread Context";

    device_class_set_parent_realize(dc, xive_tctx_kvm_realize,
                                    &xtc->parent_realize);

    xtc->synchronize_state = xive_tctx_kvm_synchronize_state;
    xtc->post_load = xive_tctx_kvm_post_load;
}

static const TypeInfo xive_tctx_kvm_info = {
    .name          = TYPE_XIVE_TCTX_KVM,
    .parent        = TYPE_XIVE_TCTX_BASE,
    .instance_size = sizeof(XiveTCTX),
    .class_init    = xive_tctx_kvm_class_init,
    .class_size    = sizeof(XiveTCTXClass),
};

/*
 * XIVE Interrupt Source (KVM)
 */

static void xive_source_kvm_reset(DeviceState *dev)
{
    XiveSource *xsrc = XIVE_SOURCE_KVM(dev);
    sPAPRXive *xive = SPAPR_XIVE_KVM(xsrc->xive);
    XiveSourceClass *xsc = XIVE_SOURCE_BASE_GET_CLASS(dev);
    int i;

    xsc->parent_reset(dev);

    /*
     * At reset, interrupt sources are simply created and MASKED. We
     * only need to inform the KVM device about their type: LSI or
     * MSI.
     */
    for (i = 0; i < xsrc->nr_irqs; i++) {
        Error *err = NULL;
        uint64_t state = 0;

        if (xive_source_irq_is_lsi(xsrc, i)) {
            state |= KVM_XIVE_LEVEL_SENSITIVE;
            if (xsrc->status[i] & XIVE_STATUS_ASSERTED) {
                state |= KVM_XIVE_LEVEL_ASSERTED;
            }
        }

        kvm_device_access(xive->fd, KVM_DEV_XIVE_GRP_SOURCES, i, &state,
                          true, &err);
        if (err) {
            error_report_err(err);
            return;
        }
    }
}

/*
 * This is used to perform the magic loads from an ESB described in
 * xive.h.
 */
static uint8_t xive_esb_read(XiveSource *xsrc, int srcno, uint32_t offset)
{
    unsigned long addr = (unsigned long) xsrc->esb_mmap +
        xive_source_esb_mgmt(xsrc, srcno) + offset;

    return *((uint8_t *) addr);
}

static void xive_source_kvm_get_state(XiveSource *xsrc)
{
    int i;

    for (i = 0; i < xsrc->nr_irqs; i++) {
        /* Perform a load without side effect to retrieve the PQ bits */
        uint8_t pq = xive_esb_read(xsrc, i, XIVE_ESB_GET);

        /* and save PQ locally */
        xive_source_esb_set(xsrc, i, pq);
    }
}

static void xive_source_kvm_synchronize_state(XiveSource *xsrc)
{
    xive_source_kvm_get_state(xsrc);
}

static int xive_source_kvm_post_load(XiveSource *xsrc, int version_id)
{
    int i;
    int unused = 0;

    for (i = 0; i < xsrc->nr_irqs; i++) {
        uint8_t pq = xive_source_esb_get(xsrc, i);

        /* TODO: prevent the compiler from optimizing away the load */
        unused |= xive_esb_read(xsrc, i, XIVE_ESB_SET_PQ_00 + (pq << 8));
    }

    return unused;
}

static void xive_source_kvm_set_irq(void *opaque, int srcno, int val)
{
    XiveSource *xsrc = opaque;
    struct kvm_irq_level args;
    int rc;

    args.irq = srcno;
    if (!xive_source_irq_is_lsi(xsrc, srcno)) {
        if (!val) {
            return;
        }
        args.level = KVM_INTERRUPT_SET;
    } else {
        args.level = val ? KVM_INTERRUPT_SET_LEVEL : KVM_INTERRUPT_UNSET;
    }
    rc = kvm_vm_ioctl(kvm_state, KVM_IRQ_LINE, &args);
    if (rc < 0) {
        error_report("kvm_irq_line() failed : %s", strerror(errno));
    }
}

static void *spapr_xive_kvm_mmap(sPAPRXive *xive, int ctrl, size_t len,
                                 Error **errp)
{
    Error *local_err = NULL;
    void *addr;
    int fd;

    kvm_device_access(xive->fd, KVM_DEV_XIVE_GRP_CTRL, ctrl, &fd, false,
                      &local_err);
    if (local_err) {
        error_propagate(errp, local_err);
        return NULL;
    }

    addr = mmap(NULL, len, PROT_WRITE | PROT_READ, MAP_SHARED, fd, 0);
    close(fd);
    if (addr == MAP_FAILED) {
        error_setg_errno(errp, errno, "Unable to set XIVE mmaping");
        return NULL;
    }

    return addr;
}

/*
 * The sPAPRXive KVM model should have initialized the KVM device
 * before initializing the source
 */
static void xive_source_kvm_realize(DeviceState *dev, Error **errp)
{
    XiveSource *xsrc = XIVE_SOURCE_KVM(dev);
    XiveSourceClass *xsc = XIVE_SOURCE_BASE_GET_CLASS(dev);
    sPAPRXive *xive = NULL;
    Error *local_err = NULL;
    size_t esb_len;

    xsc->parent_realize(dev, &local_err);
    if (local_err) {
        error_propagate(errp, local_err);
        return;
    }

    xsrc->qirqs = qemu_allocate_irqs(xive_source_kvm_set_irq, xsrc,
                                     xsrc->nr_irqs);

    xive = SPAPR_XIVE_KVM(xsrc->xive);

    /* Map the source ESB pages */
    esb_len = (1ull << xsrc->esb_shift) * xsrc->nr_irqs;
    xsrc->esb_mmap = spapr_xive_kvm_mmap(xive, KVM_DEV_XIVE_GET_ESB_FD,
                                         esb_len, &local_err);
    if (local_err) {
        error_propagate(errp, local_err);
        return;
    }

    memory_region_init_ram_device_ptr(&xsrc->esb_mmio, OBJECT(xsrc),
                                      "xive.esb", esb_len, xsrc->esb_mmap);
    sysbus_init_mmio(SYS_BUS_DEVICE(dev), &xsrc->esb_mmio);
}

static void xive_source_kvm_unrealize(DeviceState *dev, Error **errp)
{
    XiveSource *xsrc = XIVE_SOURCE_KVM(dev);
    size_t esb_len = (1ull << xsrc->esb_shift) * xsrc->nr_irqs;

    munmap(xsrc->esb_mmap, esb_len);
}

static void xive_source_kvm_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    XiveSourceClass *xsc = XIVE_SOURCE_BASE_CLASS(klass);

    device_class_set_parent_realize(dc, xive_source_kvm_realize,
                                    &xsc->parent_realize);
    device_class_set_parent_reset(dc, xive_source_kvm_reset,
                                  &xsc->parent_reset);

    dc->desc = "sPAPR XIVE KVM Interrupt Source";
    dc->unrealize = xive_source_kvm_unrealize;

    xsc->synchronize_state = xive_source_kvm_synchronize_state;
    xsc->post_load = xive_source_kvm_post_load;
}

static const TypeInfo xive_source_kvm_info = {
    .name = TYPE_XIVE_SOURCE_KVM,
    .parent = TYPE_XIVE_SOURCE_BASE,
    .instance_size = sizeof(XiveSource),
    .class_init    = xive_source_kvm_class_init,
    .class_size    = sizeof(XiveSourceClass),
};

/*
 * sPAPR XIVE Router (KVM)
 */
static int spapr_xive_kvm_set_eq_state(sPAPRXive *xive, CPUState *cs,
                                        Error **errp)
{
    XiveRouter *xrtr = XIVE_ROUTER(xive);
    int ret;
    int i;

    for (i = 0; i < XIVE_PRIORITY_MAX + 1; i++) {
        Error *local_err = NULL;
        XiveEND end;
        struct kvm_ppc_xive_eq kvm_eq = { 0 };
        uint8_t end_blk;
        uint32_t end_idx;

        if (!spapr_xive_priority_is_valid(i)) {
            continue;
        }

        spapr_xive_cpu_to_end(xive, POWERPC_CPU(cs), i, &end_blk, &end_idx);

        ret = xive_router_get_end(xrtr, end_blk, end_idx, &end);
        if (ret) {
            error_setg(errp, "XIVE: No END for CPU %ld priority %d",
                       kvm_arch_vcpu_id(cs), i);
            return ret;
        }

        if (!(end.w0 & END_W0_VALID)) {
            continue;
        }

        kvm_eq.flags = KVM_XIVE_EQ_FLAG_ALWAYS_NOTIFY;
        kvm_eq.qsize = GETFIELD(END_W0_QSIZE, end.w0) + 12;
        kvm_eq.qpage = (((uint64_t)(end.w2 & 0x0fffffff)) << 32) | end.w3;

        /* The model only supports OPAL state version 1 */
        kvm_eq.version = OPAL_STATE_VERSION;
        kvm_eq.opaque = end.w1;

        /* KVM and QEMU have the same END indexing */
        ret = kvm_device_access(xive->fd, KVM_DEV_XIVE_GRP_EQ, end_idx,
                                &kvm_eq, true, &local_err);
        if (local_err) {
            error_propagate(errp, local_err);
            return ret;
        }
    }

    return 0;
}

static int spapr_xive_kvm_get_eq_state(sPAPRXive *xive, CPUState *cs,
                                        Error **errp)
{
    XiveRouter *xrtr = XIVE_ROUTER(xive);
    int ret;
    int i;

    for (i = 0; i < XIVE_PRIORITY_MAX + 1; i++) {
        Error *local_err = NULL;
        struct kvm_ppc_xive_eq kvm_eq = { 0 };
        XiveEND end = { 0 };
        uint8_t end_blk;
        uint32_t end_idx;

        /* Skip priorities reserved for the hypervisor */
        if (!spapr_xive_priority_is_valid(i)) {
            continue;
        }

        spapr_xive_cpu_to_end(xive, POWERPC_CPU(cs), i, &end_blk, &end_idx);

        /* KVM and QEMU have the same END indexing */
        ret = kvm_device_access(xive->fd, KVM_DEV_XIVE_GRP_EQ, end_idx,
                                &kvm_eq, false, &local_err);
        if (local_err) {
            error_propagate(errp, local_err);
            return ret;
        }

        /*
         * The model only supports OPAL state version 1. The OPAL
         * state is interpreted as the word1 value of the XIVE END
         * internal structure.
         */
	if (OPAL_STATE_COMPAT(kvm_eq.version) > 1) {
            error_setg(errp, "XIVE: invalid OPAL state version %08x\n",
                       kvm_eq.version);
            return -EIO;
	}

        if (!(kvm_eq.flags & KVM_XIVE_EQ_FLAG_ENABLED)) {
            continue;
        }

        /* Update the local END structure with KVM input */
	if (kvm_eq.flags & KVM_XIVE_EQ_FLAG_ENABLED)
		end.w0 |= END_W0_VALID|END_W0_ENQUEUE;
	if (kvm_eq.flags & KVM_XIVE_EQ_FLAG_ALWAYS_NOTIFY)
		end.w0 |= END_W0_UCOND_NOTIFY;
	if (kvm_eq.flags & KVM_XIVE_EQ_FLAG_ESCALATE)
		end.w0 |= END_W0_ESCALATE_CTL;
	end.w0 |= SETFIELD(END_W0_QSIZE, 0ul, kvm_eq.qsize - 12);

	end.w1 = kvm_eq.opaque & 0xffffffff;
	end.w2 = (kvm_eq.qpage >> 32) & 0x0fffffff;
	end.w3 = kvm_eq.qpage & 0xffffffff;
	end.w4 = 0;
	end.w5 = 0;
	end.w6 = SETFIELD(END_W6_NVT_BLOCK, 0ul, 0ul) |
            SETFIELD(END_W6_NVT_INDEX, 0ul, kvm_arch_vcpu_id(cs));
        end.w7 = SETFIELD(END_W7_F0_PRIORITY, 0ul, i);

        ret = xive_router_set_end(xrtr, end_blk, end_idx, &end);
        if (ret) {
            error_setg(errp, "XIVE: No END for CPU %ld priority %d",
                       kvm_arch_vcpu_id(cs), i);
            return ret;
        }
    }

    return 0;
}

/*
 * TODO: move under spapr_xive
 */
#define SPAPR_XIVE_END_INDEX(server, prio)  (((server) << 3) | ((prio) & 0x7))
#define SPAPR_XIVE_END_SERVER(end_idx)      ((end_idx) >> 3)
#define SPAPR_XIVE_END_PRIO(end_idx)        ((end_idx) & 0x7)

static void spapr_xive_kvm_set_eas_state(sPAPRXive *xive, Error **errp)
{
    XiveSource *xsrc = &xive->source;
    int i;

    for (i = 0; i < xsrc->nr_irqs; i++) {
        XiveEAS *eas = &xive->eat[i];
        uint32_t end_idx;
        uint64_t kvm_eas;
        Error *local_err = NULL;

        if (!(eas->w & EAS_VALID) || eas->w & EAS_MASKED) {
            continue;
        }

        end_idx = GETFIELD(EAS_END_INDEX, eas->w);

        /* TODO: handle EAS_MASKED ? */

        kvm_eas = SPAPR_XIVE_END_PRIO(end_idx) << KVM_XIVE_EAS_PRIORITY_SHIFT &
            KVM_XIVE_EAS_PRIORITY_MASK;
        kvm_eas |= SPAPR_XIVE_END_SERVER(end_idx) << KVM_XIVE_EAS_SERVER_SHIFT &
            KVM_XIVE_EAS_SERVER_MASK;
        kvm_eas |= GETFIELD(EAS_END_DATA, eas->w) << KVM_XIVE_EAS_EISN_SHIFT &
            KVM_XIVE_EAS_EISN_SHIFT;

        kvm_device_access(xive->fd, KVM_DEV_XIVE_GRP_EAS, i, &kvm_eas, true,
                          &local_err);
        if (local_err) {
            error_propagate(errp, local_err);
            return;
        }
    }
}

static void spapr_xive_kvm_get_eas_state(sPAPRXive *xive, Error **errp)
{
    XiveSource *xsrc = &xive->source;
    int i;

    for (i = 0; i < xsrc->nr_irqs; i++) {
        XiveEAS *eas = &xive->eat[i];
        XiveEAS new_eas;
        uint64_t kvm_eas;
        uint8_t priority;
        uint32_t server;
        uint32_t eisn;
        Error *local_err = NULL;

        if (!(eas->w & EAS_VALID)) {
            continue;
        }

        kvm_device_access(xive->fd, KVM_DEV_XIVE_GRP_EAS, i, &kvm_eas, false,
                          &local_err);
        if (local_err) {
            error_propagate(errp, local_err);
            return;
        }

        priority = (kvm_eas & KVM_XIVE_EAS_PRIORITY_MASK) >>
            KVM_XIVE_EAS_PRIORITY_SHIFT;
        server = (kvm_eas & KVM_XIVE_EAS_SERVER_MASK) >>
            KVM_XIVE_EAS_SERVER_SHIFT;
        eisn = (kvm_eas & KVM_XIVE_EAS_EISN_MASK) >> KVM_XIVE_EAS_EISN_SHIFT;

        new_eas.w = EAS_VALID;
        if (kvm_eas & KVM_XIVE_EAS_MASK_MASK) {
            new_eas.w |= EAS_MASKED;
        }

        new_eas.w = SETFIELD(EAS_END_INDEX, new_eas.w,
                             SPAPR_XIVE_END_INDEX(server, priority));
        new_eas.w = SETFIELD(EAS_END_DATA, new_eas.w, eisn);

        *eas = new_eas;
    }
}

static void spapr_xive_kvm_sync_all(sPAPRXive *xive, Error **errp)
{
    XiveSource *xsrc = &xive->source;
    Error *local_err = NULL;
    int i;

    /* Quiesce the sources */
    for (i = 0; i < xsrc->nr_irqs; i++) {
        XiveEAS *eas = &xive->eat[i];

        if (!(eas->w & EAS_VALID)) {
            continue;
        }

        /* Sync the source now in KVM */
        kvm_device_access(xive->fd, KVM_DEV_XIVE_GRP_SYNC, i, NULL, true,
                          &local_err);
        if (local_err) {
            error_propagate(errp, local_err);
            return;
        }
    }
}

/*
 * XIVE save
 *
 * Migration needs to follow a specific sequence to make sure the
 * different internal states are captured correctly. The sPAPRXive KVM
 * model migration priority is higher to make sure its pre_save
 * handler runs before the other XIVE models' pre_save.
 *
 *   1. mask all the sources by setting PQ=01, which returns the
 *      previous value and save it.
 *   2. XIVE sync to stabilize the queues
 *   3. Dump the ENDs
 *   4. Dump the thread context (IPB)
 *
 *  Rollback to restore the current configuration of the sources
 */
static int spapr_xive_kvm_pre_save(sPAPRXive *xive)
{
    XiveSource *xsrc = &xive->source;
    Error *local_err = NULL;
    CPUState *cs;
    int i;
    int ret = 0;

    /* Quiesce the sources */
    for (i = 0; i < xsrc->nr_irqs; i++) {
        uint8_t pq;

        /*
         * Mask and save the ESB PQs locally in the XiveSource
         * object. The state will be collected by the XiveSource
         * vmstate handler afterwards
         */
        pq = xive_esb_read(xsrc, i, XIVE_ESB_SET_PQ_01);
        xive_source_esb_set(xsrc, i, pq);
    }

    /* Sync the sources in KVM */
    spapr_xive_kvm_sync_all(xive, &local_err);
    if (local_err) {
        error_report_err(local_err);
        goto out;
    }

    /* Grab the EAT (could be done earlier ?) */
    spapr_xive_kvm_get_eas_state(xive, &local_err);
    if (local_err) {
        error_report_err(local_err);
        goto out;
    }

    /*
     * Grab the ENDs. The EQ index and the toggle bit are what we want
     * to capture
     */
    CPU_FOREACH(cs) {
        spapr_xive_kvm_get_eq_state(xive, cs, &local_err);
        if (local_err) {
            error_report_err(local_err);
            goto out;
        }
    }

    /* Get the thread contexts, for the IPB register */
    CPU_FOREACH(cs) {
        PowerPCCPU *cpu = POWERPC_CPU(cs);
        XiveTCTX *tctx = XIVE_TCTX_KVM(cpu->intc);

        /* TODO: Check if we need to use under run_on_cpu() ? */
        ret = xive_tctx_kvm_get_state(tctx);
        if (ret) {
            goto out;
        }
    }

    /* We should be done now */

out:
    /* Restore the sources to their initial state */
    for (i = 0; i < xsrc->nr_irqs; i++) {
        uint8_t pq = xive_source_esb_get(xsrc, i);
        if (xive_esb_read(xsrc, i, XIVE_ESB_SET_PQ_00 + (pq << 8)) != 0x1) {
            error_report("XIVE: IRQ %d has an invalid state", i);
        }
    }

    return ret;
}

/*
 * XIVE restore
 *
 * post_load is simpler and only needs to restore the different states
 * in the correct order. sPAPRXive model has the highest priority and
 * handles first the XIVE routing internal tables: ENDT and EAT.
 * Restored next are the source ESB PQ bits by the KVM XiveSource and
 * the thread interrupt context registers by the KVM XiveTCTX model.
 *
 * All should be in place when the CPUs resume execution.
 */
static int spapr_xive_kvm_post_load(sPAPRXive *xive, int version_id)
{
    XiveSource *xsrc = &xive->source;
    Error *local_err = NULL;
    CPUState *cs;

    /* Set the ENDs first. The targetting depends on it. */
    CPU_FOREACH(cs) {
        spapr_xive_kvm_set_eq_state(xive, cs, &local_err);
        if (local_err) {
            error_report_err(local_err);
            return -1;
        }
    }

    /*
     * Create the interrupt sources from a KVM perspective. This is
     * needed for targetting which is done next
     */
    xive_source_kvm_reset(DEVICE(xsrc));

    /* Restore the targetting, if any */
    spapr_xive_kvm_set_eas_state(xive, &local_err);
    if (local_err) {
        error_report_err(local_err);
        return -1;
    }

    return 0;
}

static void spapr_xive_kvm_synchronize_state(sPAPRXive *xive)
{
    CPUState *cs;

    spapr_xive_kvm_get_eas_state(xive, &error_fatal);

    CPU_FOREACH(cs) {
        spapr_xive_kvm_get_eq_state(xive, cs, &error_fatal);
    }
}

static void spapr_xive_kvm_instance_init(Object *obj)
{
    sPAPRXive *xive = SPAPR_XIVE_KVM(obj);

    /* We need a KVM flavored source */
    object_initialize(&xive->source, sizeof(xive->source),
                      TYPE_XIVE_SOURCE_KVM);
    object_property_add_child(obj, "source", OBJECT(&xive->source), NULL);

    /* No KVM support for END ESBs. OPAL doesn't either */
    object_initialize(&xive->end_source, sizeof(xive->end_source),
                      TYPE_XIVE_END_SOURCE);
    object_property_add_child(obj, "end_source", OBJECT(&xive->end_source),
                              NULL);
}

static void spapr_xive_kvm_realize(DeviceState *dev, Error **errp)
{
    sPAPRXive *xive = SPAPR_XIVE_KVM(dev);
    sPAPRXiveClass *sxc = SPAPR_XIVE_BASE_GET_CLASS(dev);
    Error *local_err = NULL;
    size_t tima_len;

    if (!kvm_enabled() || !kvmppc_has_cap_xive()) {
        error_setg(errp,
                   "IRQ_XIVE capability must be present for KVM XIVE device");
        return;
    }

    /* First, try to create the KVM XIVE device */
    xive->fd = kvm_create_device(kvm_state, KVM_DEV_TYPE_XIVE, false);
    if (xive->fd < 0) {
        error_setg_errno(errp, -xive->fd, "error creating KVM XIVE device");
        return;
    }

    /*
     * Inform KVM where we will map the ESB pages. This is needed by
     * the H_INT_GET_SOURCE_INFO hcall which returns the source
     * characteristics, among which the ESB page address.
     *
     * We could also inform KVM where the TIMA will be mapped but as
     * this is a fixed MMIO address for the system it does not seem
     * necessary to provide a KVM ioctl to change it
     */
    kvm_device_access(xive->fd, KVM_DEV_XIVE_GRP_CTRL, KVM_DEV_XIVE_VC_BASE,
                      &xive->vc_base, true, &local_err);
    if (local_err) {
        error_propagate(errp, local_err);
        return;
    }


    /* Initialize the source and the local routing tables */
    sxc->parent_realize(dev, &local_err);
    if (local_err) {
        error_propagate(errp, local_err);
        return;
    }

    /* Map the TIMA pages */
    tima_len = 4ull << TM_SHIFT;
    xive->tm_mmap = spapr_xive_kvm_mmap(xive, KVM_DEV_XIVE_GET_TIMA_FD,
                                        tima_len, &local_err);
    if (local_err) {
        error_propagate(errp, local_err);
        return;
    }
    memory_region_init_ram_device_ptr(&xive->tm_mmio, OBJECT(xive),
                                      "xive.tima", tima_len, xive->tm_mmap);
    sysbus_init_mmio(SYS_BUS_DEVICE(dev), &xive->tm_mmio);

    kvm_kernel_irqchip = true;
    kvm_msi_via_irqfd_allowed = true;
    kvm_gsi_direct_mapping = true;
}

static void spapr_xive_kvm_unrealize(DeviceState *dev, Error **errp)
{
    sPAPRXive *xive = SPAPR_XIVE_KVM(dev);

    close(xive->fd);
    xive->fd = -1;

    munmap(xive->tm_mmap, 4ull << TM_SHIFT);
}

static void spapr_xive_kvm_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    sPAPRXiveClass *sxc = SPAPR_XIVE_BASE_CLASS(klass);

    device_class_set_parent_realize(dc, spapr_xive_kvm_realize,
                                    &sxc->parent_realize);

    dc->desc = "sPAPR XIVE KVM Interrupt Controller";
    dc->unrealize = spapr_xive_kvm_unrealize;

    sxc->synchronize_state = spapr_xive_kvm_synchronize_state;
    sxc->pre_save = spapr_xive_kvm_pre_save;
    sxc->post_load = spapr_xive_kvm_post_load;
}

static const TypeInfo spapr_xive_kvm_info = {
    .name = TYPE_SPAPR_XIVE_KVM,
    .parent = TYPE_SPAPR_XIVE_BASE,
    .instance_init = spapr_xive_kvm_instance_init,
    .instance_size = sizeof(sPAPRXive),
    .class_init = spapr_xive_kvm_class_init,
    .class_size = sizeof(sPAPRXiveClass),
};

static void xive_kvm_register_types(void)
{
    type_register_static(&spapr_xive_kvm_info);
    type_register_static(&xive_source_kvm_info);
    type_register_static(&xive_tctx_kvm_info);
}

type_init(xive_kvm_register_types)
