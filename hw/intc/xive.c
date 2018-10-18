/*
 * QEMU PowerPC XIVE interrupt controller model
 *
 * Copyright (c) 2017-2018, IBM Corporation.
 *
 * This code is licensed under the GPL version 2 or later. See the
 * COPYING file in the top-level directory.
 */

#include "qemu/osdep.h"
#include "qemu/log.h"
#include "qapi/error.h"
#include "target/ppc/cpu.h"
#include "sysemu/cpus.h"
#include "sysemu/dma.h"
#include "monitor/monitor.h"
#include "hw/ppc/xive.h"

/*
 * XIVE ESB helpers
 */

static uint8_t xive_esb_set(uint8_t *pq, uint8_t value)
{
    uint8_t old_pq = *pq & 0x3;

    *pq &= ~0x3;
    *pq |= value & 0x3;

    return old_pq;
}

static bool xive_esb_trigger(uint8_t *pq)
{
    uint8_t old_pq = *pq & 0x3;

    switch (old_pq) {
    case XIVE_ESB_RESET:
        xive_esb_set(pq, XIVE_ESB_PENDING);
        return true;
    case XIVE_ESB_PENDING:
    case XIVE_ESB_QUEUED:
        xive_esb_set(pq, XIVE_ESB_QUEUED);
        return false;
    case XIVE_ESB_OFF:
        xive_esb_set(pq, XIVE_ESB_OFF);
        return false;
    default:
         g_assert_not_reached();
    }
}

static bool xive_esb_eoi(uint8_t *pq)
{
    uint8_t old_pq = *pq & 0x3;

    switch (old_pq) {
    case XIVE_ESB_RESET:
    case XIVE_ESB_PENDING:
        xive_esb_set(pq, XIVE_ESB_RESET);
        return false;
    case XIVE_ESB_QUEUED:
        xive_esb_set(pq, XIVE_ESB_PENDING);
        return true;
    case XIVE_ESB_OFF:
        xive_esb_set(pq, XIVE_ESB_OFF);
        return false;
    default:
         g_assert_not_reached();
    }
}

/*
 * XIVE Interrupt Source (or IVSE)
 */

uint8_t xive_source_esb_get(XiveSource *xsrc, uint32_t srcno)
{
    assert(srcno < xsrc->nr_irqs);

    return xsrc->status[srcno] & 0x3;
}

uint8_t xive_source_esb_set(XiveSource *xsrc, uint32_t srcno, uint8_t pq)
{
    assert(srcno < xsrc->nr_irqs);

    return xive_esb_set(&xsrc->status[srcno], pq);
}

/*
 * Returns whether the event notification should be forwarded.
 */
static bool xive_source_lsi_trigger(XiveSource *xsrc, uint32_t srcno)
{
    uint8_t old_pq = xive_source_esb_get(xsrc, srcno);

    switch (old_pq) {
    case XIVE_ESB_RESET:
        xive_source_esb_set(xsrc, srcno, XIVE_ESB_PENDING);
        return true;
    default:
        return false;
    }
}

/*
 * Returns whether the event notification should be forwarded.
 */
static bool xive_source_esb_trigger(XiveSource *xsrc, uint32_t srcno)
{
    bool ret;

    assert(srcno < xsrc->nr_irqs);

    ret = xive_esb_trigger(&xsrc->status[srcno]);

    if (xive_source_irq_is_lsi(xsrc, srcno) &&
        xive_source_esb_get(xsrc, srcno) == XIVE_ESB_QUEUED) {
        qemu_log_mask(LOG_GUEST_ERROR,
                      "XIVE: queued an event on LSI IRQ %d\n", srcno);
    }

    return ret;
}

/*
 * Returns whether the event notification should be forwarded.
 */
static bool xive_source_esb_eoi(XiveSource *xsrc, uint32_t srcno)
{
    bool ret;

    assert(srcno < xsrc->nr_irqs);

    ret = xive_esb_eoi(&xsrc->status[srcno]);

    /* LSI sources do not set the Q bit but they can still be
     * asserted, in which case we should forward a new event
     * notification
     */
    if (xive_source_irq_is_lsi(xsrc, srcno) &&
        xsrc->status[srcno] & XIVE_STATUS_ASSERTED) {
        ret = xive_source_lsi_trigger(xsrc, srcno);
    }

    return ret;
}

/*
 * Forward the source event notification to the Router
 */
static void xive_source_notify(XiveSource *xsrc, int srcno)
{
    XiveFabricClass *xfc = XIVE_FABRIC_GET_CLASS(xsrc->xive);

    if (xfc->notify) {
        xfc->notify(xsrc->xive, srcno);
    }
}

/*
 * In a two pages ESB MMIO setting, even page is the trigger page, odd
 * page is for management
 */
static inline bool addr_is_even(hwaddr addr, uint32_t shift)
{
    return !((addr >> shift) & 1);
}

static inline bool xive_source_is_trigger_page(XiveSource *xsrc, hwaddr addr)
{
    return xive_source_esb_has_2page(xsrc) &&
        addr_is_even(addr, xsrc->esb_shift - 1);
}

/*
 * ESB MMIO loads
 *                      Trigger page    Management/EOI page
 * 2 pages setting      even            odd
 *
 * 0x000 .. 0x3FF       -1              EOI and return 0|1
 * 0x400 .. 0x7FF       -1              EOI and return 0|1
 * 0x800 .. 0xBFF       -1              return PQ
 * 0xC00 .. 0xCFF       -1              return PQ and atomically PQ=0
 * 0xD00 .. 0xDFF       -1              return PQ and atomically PQ=0
 * 0xE00 .. 0xDFF       -1              return PQ and atomically PQ=1
 * 0xF00 .. 0xDFF       -1              return PQ and atomically PQ=1
 */
static uint64_t xive_source_esb_read(void *opaque, hwaddr addr, unsigned size)
{
    XiveSource *xsrc = XIVE_SOURCE(opaque);
    uint32_t offset = addr & 0xFFF;
    uint32_t srcno = addr >> xsrc->esb_shift;
    uint64_t ret = -1;

    /* In a two pages ESB MMIO setting, trigger page should not be read */
    if (xive_source_is_trigger_page(xsrc, addr)) {
        qemu_log_mask(LOG_GUEST_ERROR,
                      "XIVE: invalid load on IRQ %d trigger page at "
                      "0x%"HWADDR_PRIx"\n", srcno, addr);
        return -1;
    }

    switch (offset) {
    case XIVE_ESB_LOAD_EOI ... XIVE_ESB_LOAD_EOI + 0x7FF:
        ret = xive_source_esb_eoi(xsrc, srcno);

        /* Forward the source event notification for routing */
        if (ret) {
            xive_source_notify(xsrc, srcno);
        }
        break;

    case XIVE_ESB_GET ... XIVE_ESB_GET + 0x3FF:
        ret = xive_source_esb_get(xsrc, srcno);
        break;

    case XIVE_ESB_SET_PQ_00 ... XIVE_ESB_SET_PQ_00 + 0x0FF:
    case XIVE_ESB_SET_PQ_01 ... XIVE_ESB_SET_PQ_01 + 0x0FF:
    case XIVE_ESB_SET_PQ_10 ... XIVE_ESB_SET_PQ_10 + 0x0FF:
    case XIVE_ESB_SET_PQ_11 ... XIVE_ESB_SET_PQ_11 + 0x0FF:
        ret = xive_source_esb_set(xsrc, srcno, (offset >> 8) & 0x3);
        break;
    default:
        qemu_log_mask(LOG_GUEST_ERROR, "XIVE: invalid ESB load addr %x\n",
                      offset);
    }

    return ret;
}

/*
 * ESB MMIO stores
 *                      Trigger page    Management/EOI page
 * 2 pages setting      even            odd
 *
 * 0x000 .. 0x3FF       Trigger         Trigger
 * 0x400 .. 0x7FF       Trigger         EOI
 * 0x800 .. 0xBFF       Trigger         undefined
 * 0xC00 .. 0xCFF       Trigger         PQ=00
 * 0xD00 .. 0xDFF       Trigger         PQ=01
 * 0xE00 .. 0xDFF       Trigger         PQ=10
 * 0xF00 .. 0xDFF       Trigger         PQ=11
 */
static void xive_source_esb_write(void *opaque, hwaddr addr,
                                  uint64_t value, unsigned size)
{
    XiveSource *xsrc = XIVE_SOURCE(opaque);
    uint32_t offset = addr & 0xFFF;
    uint32_t srcno = addr >> xsrc->esb_shift;
    bool notify = false;

    /* In a two pages ESB MMIO setting, trigger page only triggers */
    if (xive_source_is_trigger_page(xsrc, addr)) {
        notify = xive_source_esb_trigger(xsrc, srcno);
        goto out;
    }

    switch (offset) {
    case 0 ... 0x3FF:
        notify = xive_source_esb_trigger(xsrc, srcno);
        break;

    case XIVE_ESB_STORE_EOI ... XIVE_ESB_STORE_EOI + 0x3FF:
        if (!(xsrc->esb_flags & XIVE_SRC_STORE_EOI)) {
            qemu_log_mask(LOG_GUEST_ERROR,
                          "XIVE: invalid Store EOI for IRQ %d\n", srcno);
            return;
        }

        notify = xive_source_esb_eoi(xsrc, srcno);
        break;

    case XIVE_ESB_SET_PQ_00 ... XIVE_ESB_SET_PQ_00 + 0x0FF:
    case XIVE_ESB_SET_PQ_01 ... XIVE_ESB_SET_PQ_01 + 0x0FF:
    case XIVE_ESB_SET_PQ_10 ... XIVE_ESB_SET_PQ_10 + 0x0FF:
    case XIVE_ESB_SET_PQ_11 ... XIVE_ESB_SET_PQ_11 + 0x0FF:
        xive_source_esb_set(xsrc, srcno, (offset >> 8) & 0x3);
        break;

    default:
        qemu_log_mask(LOG_GUEST_ERROR, "XIVE: invalid ESB write addr %x\n",
                      offset);
        return;
    }

out:
    /* Forward the source event notification for routing */
    if (notify) {
        xive_source_notify(xsrc, srcno);
    }
}

static const MemoryRegionOps xive_source_esb_ops = {
    .read = xive_source_esb_read,
    .write = xive_source_esb_write,
    .endianness = DEVICE_BIG_ENDIAN,
    .valid = {
        .min_access_size = 8,
        .max_access_size = 8,
    },
    .impl = {
        .min_access_size = 8,
        .max_access_size = 8,
    },
};

static void xive_source_set_irq(void *opaque, int srcno, int val)
{
    XiveSource *xsrc = XIVE_SOURCE(opaque);
    bool notify = false;

    if (xive_source_irq_is_lsi(xsrc, srcno)) {
        if (val) {
            xsrc->status[srcno] |= XIVE_STATUS_ASSERTED;
            notify = xive_source_lsi_trigger(xsrc, srcno);
        } else {
            xsrc->status[srcno] &= ~XIVE_STATUS_ASSERTED;
        }
    } else {
        if (val) {
            notify = xive_source_esb_trigger(xsrc, srcno);
        }
    }

    /* Forward the source event notification for routing */
    if (notify) {
        xive_source_notify(xsrc, srcno);
    }
}

void xive_source_pic_print_info(XiveSource *xsrc, uint32_t offset, Monitor *mon)
{
    int i;

    monitor_printf(mon, "XIVE Source %08x .. %08x\n",
                   offset, offset + xsrc->nr_irqs - 1);
    for (i = 0; i < xsrc->nr_irqs; i++) {
        uint8_t pq = xive_source_esb_get(xsrc, i);

        if (pq == XIVE_ESB_OFF) {
            continue;
        }

        monitor_printf(mon, "  %08x %s %c%c%c\n", i + offset,
                       xive_source_irq_is_lsi(xsrc, i) ? "LSI" : "MSI",
                       pq & XIVE_ESB_VAL_P ? 'P' : '-',
                       pq & XIVE_ESB_VAL_Q ? 'Q' : '-',
                       xsrc->status[i] & XIVE_STATUS_ASSERTED ? 'A' : ' ');
    }
}

static void xive_source_reset(DeviceState *dev)
{
    XiveSource *xsrc = XIVE_SOURCE(dev);

    /* Do not clear the LSI bitmap */

    /* PQs are initialized to 0b01 which corresponds to "ints off" */
    memset(xsrc->status, 0x1, xsrc->nr_irqs);
}

static void xive_source_realize(DeviceState *dev, Error **errp)
{
    XiveSource *xsrc = XIVE_SOURCE(dev);
    Object *obj;
    Error *local_err = NULL;

    obj = object_property_get_link(OBJECT(dev), "xive", &local_err);
    if (!obj) {
        error_propagate(errp, local_err);
        error_prepend(errp, "required link 'xive' not found: ");
        return;
    }

    xsrc->xive = XIVE_FABRIC(obj);

    if (!xsrc->nr_irqs) {
        error_setg(errp, "Number of interrupt needs to be greater than 0");
        return;
    }

    if (xsrc->esb_shift != XIVE_ESB_4K &&
        xsrc->esb_shift != XIVE_ESB_4K_2PAGE &&
        xsrc->esb_shift != XIVE_ESB_64K &&
        xsrc->esb_shift != XIVE_ESB_64K_2PAGE) {
        error_setg(errp, "Invalid ESB shift setting");
        return;
    }

    xsrc->qirqs = qemu_allocate_irqs(xive_source_set_irq, xsrc,
                                     xsrc->nr_irqs);

    xsrc->status = g_malloc0(xsrc->nr_irqs);

    xsrc->lsi_map = bitmap_new(xsrc->nr_irqs);
    xsrc->lsi_map_size = xsrc->nr_irqs;

    memory_region_init_io(&xsrc->esb_mmio, OBJECT(xsrc),
                          &xive_source_esb_ops, xsrc, "xive.esb",
                          (1ull << xsrc->esb_shift) * xsrc->nr_irqs);
    sysbus_init_mmio(SYS_BUS_DEVICE(dev), &xsrc->esb_mmio);
}

static const VMStateDescription vmstate_xive_source = {
    .name = TYPE_XIVE_SOURCE,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (VMStateField[]) {
        VMSTATE_UINT32_EQUAL(nr_irqs, XiveSource, NULL),
        VMSTATE_VBUFFER_UINT32(status, XiveSource, 1, NULL, nr_irqs),
        VMSTATE_BITMAP(lsi_map, XiveSource, 1, lsi_map_size),
        VMSTATE_END_OF_LIST()
    },
};

/*
 * The default XIVE interrupt source setting for the ESB MMIOs is two
 * 64k pages without Store EOI, to be in sync with KVM.
 */
static Property xive_source_properties[] = {
    DEFINE_PROP_UINT64("flags", XiveSource, esb_flags, 0),
    DEFINE_PROP_UINT32("nr-irqs", XiveSource, nr_irqs, 0),
    DEFINE_PROP_UINT32("shift", XiveSource, esb_shift, XIVE_ESB_64K_2PAGE),
    DEFINE_PROP_END_OF_LIST(),
};

static void xive_source_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->desc    = "XIVE Interrupt Source";
    dc->props   = xive_source_properties;
    dc->realize = xive_source_realize;
    dc->reset   = xive_source_reset;
    dc->vmsd    = &vmstate_xive_source;
}

static const TypeInfo xive_source_info = {
    .name          = TYPE_XIVE_SOURCE,
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(XiveSource),
    .class_init    = xive_source_class_init,
};

/*
 * XIVE Router (aka. Virtualization Controller or IVRE)
 */

int xive_router_get_eas(XiveRouter *xrtr, uint32_t lisn, XiveEAS *eas)
{
    XiveRouterClass *xrc = XIVE_ROUTER_GET_CLASS(xrtr);

    return xrc->get_eas(xrtr, lisn, eas);
}

int xive_router_set_eas(XiveRouter *xrtr, uint32_t lisn, XiveEAS *eas)
{
    XiveRouterClass *xrc = XIVE_ROUTER_GET_CLASS(xrtr);

    return xrc->set_eas(xrtr, lisn, eas);
}

static void xive_router_notify(XiveFabric *xf, uint32_t lisn)
{
    XiveRouter *xrtr = XIVE_ROUTER(xf);
    XiveEAS eas;

    /* EAS cache lookup */
    if (xive_router_get_eas(xrtr, lisn, &eas)) {
        qemu_log_mask(LOG_GUEST_ERROR, "XIVE: Unknown LISN %x\n", lisn);
        return;
    }

    /* The IVRE has a State Bit Cache for its internal sources which
     * is also involed at this point. We skip the SBC lookup because
     * the state bits of the sources are modeled internally in QEMU.
     */

    if (!(eas.w & EAS_VALID)) {
        qemu_log_mask(LOG_GUEST_ERROR, "XIVE: invalid LISN %x\n", lisn);
        return;
    }

    if (eas.w & EAS_MASKED) {
        /* Notification completed */
        return;
    }
}

static Property xive_router_properties[] = {
    DEFINE_PROP_UINT32("chip-id", XiveRouter, chip_id, 0),
    DEFINE_PROP_END_OF_LIST(),
};

static void xive_router_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    XiveFabricClass *xfc = XIVE_FABRIC_CLASS(klass);

    dc->desc    = "XIVE Router Engine";
    dc->props   = xive_router_properties;
    xfc->notify = xive_router_notify;
}

static const TypeInfo xive_router_info = {
    .name          = TYPE_XIVE_ROUTER,
    .parent        = TYPE_SYS_BUS_DEVICE,
    .abstract      = true,
    .class_size    = sizeof(XiveRouterClass),
    .class_init    = xive_router_class_init,
    .interfaces    = (InterfaceInfo[]) {
        { TYPE_XIVE_FABRIC },
        { }
    }
};

void xive_router_print_eas(XiveRouter *xrtr, uint32_t lisn, XiveEAS *eas,
                           Monitor *mon)
{
    if (!(eas->w & EAS_VALID)) {
        return;
    }

    monitor_printf(mon, "  %08x %s\n", lisn, eas->w & EAS_MASKED ? "M" : " ");
}

/*
 * XIVE Fabric
 */
static const TypeInfo xive_fabric_info = {
    .name = TYPE_XIVE_FABRIC,
    .parent = TYPE_INTERFACE,
    .class_size = sizeof(XiveFabricClass),
};

static void xive_register_types(void)
{
    type_register_static(&xive_source_info);
    type_register_static(&xive_fabric_info);
    type_register_static(&xive_router_info);
}

type_init(xive_register_types)
