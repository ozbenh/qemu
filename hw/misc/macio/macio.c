/*
 * PowerMac MacIO device emulation
 *
 * Copyright (c) 2005-2007 Fabrice Bellard
 * Copyright (c) 2007 Jocelyn Mayer
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
 * THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */
#include "qemu/osdep.h"
#include "qapi/error.h"
#include "hw/hw.h"
#include "hw/ppc/mac.h"
#include "hw/pci/pci.h"
#include "hw/ppc/mac_dbdma.h"
#include "hw/char/escc.h"

#define TYPE_MACIO "macio"
#define MACIO(obj) OBJECT_CHECK(MacIOState, (obj), TYPE_MACIO)

typedef struct MacIOState
{
    /*< private >*/
    PCIDevice parent;
    /*< public >*/

    MemoryRegion bar;
    CUDAState cuda;
    DeviceState *pmu;
    void *dbdma;
    MemoryRegion *pic_mem;
    MemoryRegion *escc_mem;
    uint64_t frequency;
    bool has_pmu;
} MacIOState;

#define OLDWORLD_MACIO(obj) \
    OBJECT_CHECK(OldWorldMacIOState, (obj), TYPE_OLDWORLD_MACIO)

typedef struct OldWorldMacIOState {
    /*< private >*/
    MacIOState parent_obj;
    /*< public >*/

    qemu_irq irqs[5];

    MacIONVRAMState nvram;
    MACIOIDEState ide[2];
} OldWorldMacIOState;

#define NEWWORLD_MACIO(obj) \
    OBJECT_CHECK(NewWorldMacIOState, (obj), TYPE_NEWWORLD_MACIO)

typedef struct NewWorldMacIOState {
    /*< private >*/
    MacIOState parent_obj;
    /*< public >*/
    qemu_irq irqs[6];
    int gpio1_irq;
    MACIOIDEState ide[2];
    uint8_t gpio_levels[8];
    uint8_t gpio_regs[36]; /* XXX Check count */
} NewWorldMacIOState;

/*
 * The mac-io has two interfaces to the ESCC. One is called "escc-legacy",
 * while the other one is the normal, current ESCC interface.
 *
 * The magic below creates memory aliases to spawn the escc-legacy device
 * purely by rerouting the respective registers to our escc region. This
 * works because the only difference between the two memory regions is the
 * register layout, not their semantics.
 *
 * Reference: ftp://ftp.software.ibm.com/rs6000/technology/spec/chrp/inwork/CHRP_IORef_1.0.pdf
 */
static void macio_escc_legacy_setup(MacIOState *macio_state)
{
    MemoryRegion *escc_legacy = g_new(MemoryRegion, 1);
    MemoryRegion *bar = &macio_state->bar;
    int i;
    static const int maps[] = {
        0x00, 0x00, /* Command B */
        0x02, 0x20, /* Command A */
        0x04, 0x10, /* Data B */
        0x06, 0x30, /* Data A */
        0x08, 0x40, /* Enhancement B */
        0x0A, 0x50, /* Enhancement A */
        0x80, 0x80, /* Recovery count */
        0x90, 0x90, /* Start A */
        0xa0, 0xa0, /* Start B */
        0xb0, 0xb0, /* Detect AB */
    };

    memory_region_init(escc_legacy, OBJECT(macio_state), "escc-legacy", 256);
    for (i = 0; i < ARRAY_SIZE(maps); i += 2) {
        MemoryRegion *port = g_new(MemoryRegion, 1);
        memory_region_init_alias(port, OBJECT(macio_state), "escc-legacy-port",
                                 macio_state->escc_mem, maps[i+1], 0x2);
        memory_region_add_subregion(escc_legacy, maps[i], port);
    }

    memory_region_add_subregion(bar, 0x12000, escc_legacy);
}

static void macio_bar_setup(MacIOState *macio_state)
{
    MemoryRegion *bar = &macio_state->bar;

    if (macio_state->escc_mem) {
        memory_region_add_subregion(bar, 0x13000, macio_state->escc_mem);
        macio_escc_legacy_setup(macio_state);
    }
}

static void macio_common_realize(PCIDevice *d, Error **errp)
{
    MacIOState *s = MACIO(d);
    SysBusDevice *sysbus_dev;
    Error *err = NULL;
    MemoryRegion *dbdma_mem;

    s->dbdma = DBDMA_init(&dbdma_mem);
    memory_region_add_subregion(&s->bar, 0x08000, dbdma_mem);

    if (s->has_pmu) {
        qdev_init_nofail(s->pmu);
        sysbus_dev = SYS_BUS_DEVICE(s->pmu);
    } else {
        object_property_set_bool(OBJECT(&s->cuda), true, "realized", &err);
        if (err) {
            error_propagate(errp, err);
            return;
        }
        sysbus_dev = SYS_BUS_DEVICE(&s->cuda);
    }
    memory_region_add_subregion(&s->bar, 0x16000,
                                sysbus_mmio_get_region(sysbus_dev, 0));

    macio_bar_setup(s);
    pci_register_bar(d, 0, PCI_BASE_ADDRESS_SPACE_MEMORY, &s->bar);
}

static void macio_realize_ide(MacIOState *s, MACIOIDEState *ide,
                              qemu_irq irq0, qemu_irq irq1, int dmaid,
                              Error **errp)
{
    SysBusDevice *sysbus_dev;

    sysbus_dev = SYS_BUS_DEVICE(ide);
    sysbus_connect_irq(sysbus_dev, 0, irq0);
    sysbus_connect_irq(sysbus_dev, 1, irq1);
    macio_ide_register_dma(ide, s->dbdma, dmaid);
    object_property_set_bool(OBJECT(ide), true, "realized", errp);
}

static void macio_oldworld_realize(PCIDevice *d, Error **errp)
{
    MacIOState *s = MACIO(d);
    OldWorldMacIOState *os = OLDWORLD_MACIO(d);
    Error *err = NULL;
    SysBusDevice *sysbus_dev;
    int i;
    int cur_irq = 0;

    macio_common_realize(d, &err);
    if (err) {
        error_propagate(errp, err);
        return;
    }

    if (s->has_pmu) {
        sysbus_dev = SYS_BUS_DEVICE(s->pmu);
    } else {
        sysbus_dev = SYS_BUS_DEVICE(&s->cuda);
    }
    sysbus_connect_irq(sysbus_dev, 0, os->irqs[cur_irq++]);

    object_property_set_bool(OBJECT(&os->nvram), true, "realized", &err);
    if (err) {
        error_propagate(errp, err);
        return;
    }
    sysbus_dev = SYS_BUS_DEVICE(&os->nvram);
    memory_region_add_subregion(&s->bar, 0x60000,
                                sysbus_mmio_get_region(sysbus_dev, 0));
    pmac_format_nvram_partition(&os->nvram, os->nvram.size);

    if (s->pic_mem) {
        /* Heathrow PIC */
        memory_region_add_subregion(&s->bar, 0x00000, s->pic_mem);
    }

    /* IDE buses */
    for (i = 0; i < ARRAY_SIZE(os->ide); i++) {
        qemu_irq irq0 = os->irqs[cur_irq++];
        qemu_irq irq1 = os->irqs[cur_irq++];

        macio_realize_ide(s, &os->ide[i], irq0, irq1, 0x16 + (i * 4), &err);
        if (err) {
            error_propagate(errp, err);
            return;
        }
    }
}

static void macio_init_ide(MacIOState *s, MACIOIDEState *ide, size_t ide_size,
                           int index)
{
    gchar *name;

    object_initialize(ide, ide_size, TYPE_MACIO_IDE);
    qdev_set_parent_bus(DEVICE(ide), sysbus_get_default());
    memory_region_add_subregion(&s->bar, 0x1f000 + ((index + 1) * 0x1000),
                                &ide->mem);
    name = g_strdup_printf("ide[%i]", index);
    object_property_add_child(OBJECT(s), name, OBJECT(ide), NULL);
    g_free(name);
}

static void macio_oldworld_init(Object *obj)
{
    MacIOState *s = MACIO(obj);
    OldWorldMacIOState *os = OLDWORLD_MACIO(obj);
    DeviceState *dev;
    int i;

    qdev_init_gpio_out(DEVICE(obj), os->irqs, ARRAY_SIZE(os->irqs));

    object_initialize(&os->nvram, sizeof(os->nvram), TYPE_MACIO_NVRAM);
    dev = DEVICE(&os->nvram);
    qdev_prop_set_uint32(dev, "size", 0x2000);
    qdev_prop_set_uint32(dev, "it_shift", 4);

    for (i = 0; i < 2; i++) {
        macio_init_ide(s, &os->ide[i], sizeof(os->ide[i]), i);
    }
}

static void timer_write(void *opaque, hwaddr addr, uint64_t value,
                       unsigned size)
{
}

static uint64_t timer_read(void *opaque, hwaddr addr, unsigned size)
{
    uint32_t value = 0;
    uint64_t systime = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
    uint64_t kltime;

    kltime = muldiv64(systime, 4194300, NANOSECONDS_PER_SECOND * 4);
    kltime = muldiv64(kltime, 18432000, 1048575);

    switch (addr) {
    case 0x38:
        value = kltime;
        break;
    case 0x3c:
        value = kltime >> 32;
        break;
    }

    return value;
}

static const MemoryRegionOps timer_ops = {
    .read = timer_read,
    .write = timer_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .impl = {
        .min_access_size = 4,
        .max_access_size = 4,
    },
};

static void gpio_write(void *opaque, hwaddr addr, uint64_t value,
                       unsigned size)
{
    NewWorldMacIOState *ns = opaque;

    printf("MACIO: Write GPIO %lx/%d = %lx\n",
           (unsigned long)addr, size, (unsigned long)value);

    /* Levels regs are read-only */
    if (addr < 8) {
        return;
    }
    addr -= 8;
    if (addr < 36) {
        uint8_t ibit;

        value &= ~2;
        if (value & 4) {
            ibit = (value & 1) << 1;
        } else {
            ibit = ns->gpio_regs[addr] & 2;
        }
        ns->gpio_regs[addr] = value | ibit;

        // XXX Update the levels summary
        if (addr < 32) {
        } else {
        }
    }
}

void MacIOSetGPIO(DeviceState *dev, uint32_t gpio, bool state)
{
    NewWorldMacIOState *ns = NEWWORLD_MACIO(dev);

    if (ns->gpio_regs[gpio] & 4) {
        printf("MACIO: Setting GPIO %d while it's an output\n", gpio);
    }
    ns->gpio_regs[gpio] = ns->gpio_regs[gpio] & 2;
    if (state) {
        ns->gpio_regs[gpio] |= 2;
    }
    // XXX Update the levels summary
    // XXX Update the interrupts
    if (gpio == 9) {
        if (state) {
            qemu_irq_raise(ns->irqs[ns->gpio1_irq]);
        } else {
            qemu_irq_lower(ns->irqs[ns->gpio1_irq]);
        }
    }
}

static uint64_t gpio_read(void *opaque, hwaddr addr, unsigned size)
{
    NewWorldMacIOState *ns = opaque;

    printf("MACIO: Read GPIO %lx/%d\n", (unsigned long)addr, size);

    /* Levels regs */
    if (addr < 8) {
        return ns->gpio_levels[addr];
    }
    addr -= 8;
    if (addr < 36) {
        return ns->gpio_regs[addr];
    }
    return 0;
}

static const MemoryRegionOps gpio_ops = {
    .read = gpio_read,
    .write = gpio_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .impl = {
        .min_access_size = 1, // XXX Check these
        .max_access_size = 1,
    },
};

static void macio_newworld_realize(PCIDevice *d, Error **errp)
{
    MacIOState *s = MACIO(d);
    NewWorldMacIOState *ns = NEWWORLD_MACIO(d);
    Error *err = NULL;
    SysBusDevice *sysbus_dev;
    MemoryRegion *timer_memory = NULL;
    MemoryRegion *gpio_memory = NULL;
    int i;
    int cur_irq = 0;

    macio_common_realize(d, &err);
    if (err) {
        error_propagate(errp, err);
        return;
    }


    if (s->has_pmu) {
        sysbus_dev = SYS_BUS_DEVICE(s->pmu);
    } else {
        sysbus_dev = SYS_BUS_DEVICE(&s->cuda);
    }
    sysbus_connect_irq(sysbus_dev, 0, ns->irqs[cur_irq++]);

    if (s->pic_mem) {
        /* OpenPIC */
        memory_region_add_subregion(&s->bar, 0x40000, s->pic_mem);
    }

    /* IDE buses */
    for (i = 0; i < ARRAY_SIZE(ns->ide); i++) {
        qemu_irq irq0 = ns->irqs[cur_irq++];
        qemu_irq irq1 = ns->irqs[cur_irq++];

        macio_realize_ide(s, &ns->ide[i], irq0, irq1, 0x16 + (i * 4), &err);
        if (err) {
            error_propagate(errp, err);
            return;
        }
    }

    /* Timer */
    timer_memory = g_new(MemoryRegion, 1);
    memory_region_init_io(timer_memory, OBJECT(s), &timer_ops, NULL, "timer",
                          0x1000);
    memory_region_add_subregion(&s->bar, 0x15000, timer_memory);

    /* GPIO registers */
    ns->gpio1_irq = cur_irq++;
    gpio_memory = g_new(MemoryRegion, 1);
    memory_region_init_io(gpio_memory, OBJECT(s), &gpio_ops, ns, "gpio", 0x30);
    memory_region_add_subregion(&s->bar, 0x50, gpio_memory);
}

static void macio_newworld_init(Object *obj)
{
    MacIOState *s = MACIO(obj);
    NewWorldMacIOState *ns = NEWWORLD_MACIO(obj);
    int i;

    qdev_init_gpio_out(DEVICE(obj), ns->irqs, ARRAY_SIZE(ns->irqs));

    for (i = 0; i < 2; i++) {
        macio_init_ide(s, &ns->ide[i], sizeof(ns->ide[i]), i);
    }
}

static void macio_instance_init(Object *obj)
{
    MacIOState *s = MACIO(obj);

    memory_region_init(&s->bar, obj, "macio", 0x80000);
}

static const VMStateDescription vmstate_macio_oldworld = {
    .name = "macio-oldworld",
    .version_id = 0,
    .minimum_version_id = 0,
    .fields = (VMStateField[]) {
        VMSTATE_PCI_DEVICE(parent_obj.parent, OldWorldMacIOState),
        VMSTATE_END_OF_LIST()
    }
};

static void macio_oldworld_class_init(ObjectClass *oc, void *data)
{
    PCIDeviceClass *pdc = PCI_DEVICE_CLASS(oc);
    DeviceClass *dc = DEVICE_CLASS(oc);

    pdc->realize = macio_oldworld_realize;
    pdc->device_id = PCI_DEVICE_ID_APPLE_343S1201;
    dc->vmsd = &vmstate_macio_oldworld;
}

static const VMStateDescription vmstate_macio_newworld = {
    .name = "macio-newworld",
    .version_id = 0,
    .minimum_version_id = 0,
    .fields = (VMStateField[]) {
        VMSTATE_PCI_DEVICE(parent_obj.parent, NewWorldMacIOState),
        VMSTATE_END_OF_LIST()
    }
};

static void macio_newworld_class_init(ObjectClass *oc, void *data)
{
    PCIDeviceClass *pdc = PCI_DEVICE_CLASS(oc);
    DeviceClass *dc = DEVICE_CLASS(oc);

    pdc->realize = macio_newworld_realize;
    pdc->device_id = PCI_DEVICE_ID_APPLE_UNI_N_KEYL;
    dc->vmsd = &vmstate_macio_newworld;
}

static Property macio_properties[] = {
    DEFINE_PROP_UINT64("frequency", MacIOState, frequency, 0),
    DEFINE_PROP_BOOL("has_pmu", MacIOState, has_pmu, false),
    DEFINE_PROP_END_OF_LIST()
};

static void macio_class_init(ObjectClass *klass, void *data)
{
    PCIDeviceClass *k = PCI_DEVICE_CLASS(klass);
    DeviceClass *dc = DEVICE_CLASS(klass);

    k->vendor_id = PCI_VENDOR_ID_APPLE;
    k->class_id = PCI_CLASS_OTHERS << 8;
    dc->props = macio_properties;
    set_bit(DEVICE_CATEGORY_BRIDGE, dc->categories);
}

static const TypeInfo macio_oldworld_type_info = {
    .name          = TYPE_OLDWORLD_MACIO,
    .parent        = TYPE_MACIO,
    .instance_size = sizeof(OldWorldMacIOState),
    .instance_init = macio_oldworld_init,
    .class_init    = macio_oldworld_class_init,
};

static const TypeInfo macio_newworld_type_info = {
    .name          = TYPE_NEWWORLD_MACIO,
    .parent        = TYPE_MACIO,
    .instance_size = sizeof(NewWorldMacIOState),
    .instance_init = macio_newworld_init,
    .class_init    = macio_newworld_class_init,
};

static const TypeInfo macio_type_info = {
    .name          = TYPE_MACIO,
    .parent        = TYPE_PCI_DEVICE,
    .instance_size = sizeof(MacIOState),
    .instance_init = macio_instance_init,
    .abstract      = true,
    .class_init    = macio_class_init,
};

static void macio_register_types(void)
{
    type_register_static(&macio_type_info);
    type_register_static(&macio_oldworld_type_info);
    type_register_static(&macio_newworld_type_info);
}

type_init(macio_register_types)

void macio_init(PCIDevice *d,
                MemoryRegion *pic_mem,
                MemoryRegion *escc_mem)
{
    MacIOState *s = MACIO(d);

    s->pic_mem = pic_mem;
    s->escc_mem = escc_mem;

    if (s->has_pmu) {
        printf("MACIO: Creating PMU\n");
        s->pmu = qdev_create(NULL, "via-pmu");
        object_property_add_child(OBJECT(s), "pmu", OBJECT(s->pmu), NULL);
        qdev_prop_set_uint64(s->pmu, "frequency", s->frequency);
        qdev_prop_set_ptr(s->pmu, "macio", s);
   } else {
        printf("MACIO: Creating CUDA\n");
        object_initialize(&s->cuda, sizeof(s->cuda), TYPE_CUDA);
        qdev_set_parent_bus(DEVICE(&s->cuda), sysbus_get_default());
        object_property_add_child(OBJECT(s), "cuda", OBJECT(&s->cuda), NULL);
        qdev_prop_set_uint64(DEVICE(&s->cuda), "frequency",
                             s->frequency);
    }

    qdev_init_nofail(DEVICE(d));
}
