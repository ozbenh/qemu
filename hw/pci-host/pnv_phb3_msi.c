/*
 * QEMU PowerPC PowerNV PHB3 model
 *
 * Copyright (c) 2014-2017, IBM Corporation.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License, version 2, as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, see <http://www.gnu.org/licenses/>.
 */
#include "qemu/osdep.h"
#include "qemu/log.h"
#include "qapi/error.h"
#include "qemu-common.h"
#include "hw/pci-host/pnv_phb3.h"
#include "hw/pci/msi.h"

static uint64_t phb3_msi_ive_addr(PnvPhb3State *phb, int srcno)
{
    uint64_t ivtbar = phb->regs[PHB_IVT_BAR >> 3];
    uint64_t phbctl = phb->regs[PHB_CONTROL >> 3];

    if (!(ivtbar & PHB_IVT_BAR_ENABLE)) {
        qemu_log_mask(LOG_GUEST_ERROR, "Failed access to disable IVT BAR !");
        return 0;
    }

    if (srcno >= (ivtbar & PHB_IVT_LENGTH_MASK)) {
        qemu_log_mask(LOG_GUEST_ERROR, "MSI out of bounds (%d vs  0x%"PRIx64")",
                      srcno, ivtbar & PHB_IVT_LENGTH_MASK);
        return 0;
    }

    ivtbar &= PHB_IVT_BASE_ADDRESS_MASK;

    if (phbctl & PHB_CTRL_IVE_128_BYTES) {
        return ivtbar + 128 * srcno;
    } else {
        return ivtbar + 16 * srcno;
    }
}

static bool phb3_msi_read_ive(PnvPhb3State *phb, int srcno, uint64_t *out_ive)
{
    uint64_t ive_addr, ive;

    ive_addr = phb3_msi_ive_addr(phb, srcno);
    if (!ive_addr) {
        return false;
    }

    if (dma_memory_read(&address_space_memory, ive_addr, &ive, sizeof(ive))) {
        qemu_log_mask(LOG_GUEST_ERROR, "Failed to read IVE at 0x%" PRIx64,
                      ive_addr);
        return false;
    }
    *out_ive = be64_to_cpu(ive);

    return true;
}

static void phb3_msi_set_p(Phb3MsiState *msis, int srcno, uint8_t gen)
{
    uint64_t ive_addr;
    uint8_t p = 0x01 | (gen << 1);

    ive_addr = phb3_msi_ive_addr(msis->phb, srcno);
    if (!ive_addr) {
        return;
    }

    if (dma_memory_write(&address_space_memory, ive_addr + 4, &p, 1)) {
        qemu_log_mask(LOG_GUEST_ERROR,
                      "Failed to write IVE (set P) at 0x%" PRIx64, ive_addr);
    }
}

static void phb3_msi_set_q(Phb3MsiState *msis, int srcno)
{
    uint64_t ive_addr;
    uint8_t q = 0x01;

    ive_addr = phb3_msi_ive_addr(msis->phb, srcno);
    if (!ive_addr) {
        return;
    }

    if (dma_memory_write(&address_space_memory, ive_addr + 5, &q, 1)) {
        qemu_log_mask(LOG_GUEST_ERROR,
                      "Failed to write IVE (set Q) at 0x%" PRIx64, ive_addr);
    }
}

static void phb3_msi_try_send(Phb3MsiState *msis, int srcno, bool ignore_p)
{
    ICSState *ics = &msis->ics;
    uint64_t ive;
    uint64_t server, prio, pq, gen;

    if (!phb3_msi_read_ive(msis->phb, srcno, &ive)) {
        return;
    }

    server = GETFIELD(IODA2_IVT_SERVER, ive);
    prio = GETFIELD(IODA2_IVT_PRIORITY, ive);
    pq = GETFIELD(IODA2_IVT_Q, ive);
    if (!ignore_p) {
        pq |= GETFIELD(IODA2_IVT_P, ive) << 1;
    }
    gen = GETFIELD(IODA2_IVT_GEN, ive);

    /*
     * The low order 2 bits are the link pointer (Type II interrupts).
     * Shift back to get a valid IRQ server.
     */
    server >>= 2;

    switch (pq) {
    case 0: /* 00 */
        if (prio == 0xff) {
            /* Masked, set Q */
            phb3_msi_set_q(msis, srcno);
        } else {
            /* Enabled, set P and send */
            phb3_msi_set_p(msis, srcno, gen);
            icp_irq(ics, server, srcno + ics->offset, prio);
        }
        break;
    case 2: /* 10 */
        /* Already pending, set Q */
        phb3_msi_set_q(msis, srcno);
        break;
    case 1: /* 01 */
    case 3: /* 11 */
    default:
        /* Just drop stuff if Q already set */
        break;
    }
}

static void phb3_msi_set_irq(void *opaque, int srcno, int val)
{
    Phb3MsiState *msis = opaque;

    if (val) {
        phb3_msi_try_send(msis, srcno, false);
    }
}


void pnv_phb3_msi_send(Phb3MsiState *msis, uint64_t addr, uint16_t data,
                       int32_t dev_pe)
{
    uint64_t ive;
    uint16_t pe;
    uint32_t src = ((addr >> 4) & 0xffff) | (data & 0x1f);

    if (src >= msis->ics.nr_irqs) {
        qemu_log_mask(LOG_GUEST_ERROR, "MSI %d out of bounds", src);
        return;
    }
    if (dev_pe >= 0) {
        if (!phb3_msi_read_ive(msis->phb, src, &ive)) {
            return;
        }
        pe = GETFIELD(IODA2_IVT_PE, ive);
        if (pe != dev_pe) {
            qemu_log_mask(LOG_GUEST_ERROR,
                          "MSI %d send by PE#%d but assigned to PE#%d",
                          src, dev_pe, pe);
            return;
        }
    }
    qemu_irq_pulse(msis->ics.qirqs[src]);
}

void pnv_phb3_msi_ffi(Phb3MsiState *msis, uint64_t val)
{
    /* Emit interrupt */
    pnv_phb3_msi_send(msis, val, 0, -1);

    /* Clear FFI lock */
    msis->phb->regs[PHB_FFI_LOCK >> 3] = 0;
}

static void phb3_msi_reject(ICSState *ics, uint32_t nr)
{
    Phb3MsiState *msis = PHB3_MSI(ics);
    unsigned int srcno = nr - ics->offset;
    unsigned int idx = srcno >> 6;
    unsigned int bit = 1ull << (srcno & 0x3f);

    assert(srcno < PHB3_MAX_MSI);

    msis->rba[idx] |= bit;
    msis->rba_sum |= (1u << idx);
}

static void phb3_msi_resend(ICSState *ics)
{
    Phb3MsiState *msis = PHB3_MSI(ics);
    unsigned int i, j;

    if (msis->rba_sum == 0) {
        return;
    }

    for (i = 0; i < 32; i++) {
        if ((msis->rba_sum & (1u << i)) == 0) {
            continue;
        }
        msis->rba_sum &= ~(1u << i);
        for (j = 0; j < 64; j++) {
            if ((msis->rba[i] & (1ull << j)) == 0) {
                continue;
            }
            msis->rba[i] &= ~(1u << j);
            phb3_msi_try_send(msis, i * 64 + j, true);
        }
    }
}

static void phb3_msi_reset(void *dev)
{
    Phb3MsiState *msis = PHB3_MSI(dev);

    memset(msis->rba, 0, sizeof(msis->rba));
    msis->rba_sum = 0;
}

void pnv_phb3_msi_update_config(Phb3MsiState *msis, uint32_t base,
                                uint32_t count)
{
    if (count > PHB3_MAX_MSI) {
        count = PHB3_MAX_MSI;
    }
    msis->ics.nr_irqs = count;
    msis->ics.offset = base;
}

static void phb3_msi_initfn(Object *obj)
{
    Phb3MsiState *msis = PHB3_MSI(obj);

    /* Will be overriden later */
    msis->ics.offset = 0;

    /* Hard wire 2048, we ignore the fact that 8 of them can be
     * taken over by LSIs at this point
     */
    msis->ics.nr_irqs = PHB3_MAX_MSI;
}

static void phb3_msi_realize(ICSState *dev, Error **errp)
{
    Phb3MsiState *msis = PHB3_MSI(dev);
    Object *obj;
    Error *err = NULL;

    obj = object_property_get_link(OBJECT(dev), "phb", &err);
    if (!obj) {
        error_setg(errp, "%s: required link 'phb' not found: %s",
                   __func__, error_get_pretty(err));
        return;
    }
    msis->phb = PNV_PHB3(obj);

    msis->ics.irqs = NULL;
    msis->ics.qirqs = qemu_allocate_irqs(phb3_msi_set_irq, msis, PHB3_MAX_MSI);

    qemu_register_reset(phb3_msi_reset, dev);
}

static void phb3_msi_class_init(ObjectClass *klass, void *data)
{
    ICSStateClass *isc = ICS_BASE_CLASS(klass);

    isc->realize = phb3_msi_realize;
    isc->reject = phb3_msi_reject;
    isc->resend = phb3_msi_resend;
}

static const TypeInfo phb3_msi_info = {
    .name = TYPE_PHB3_MSI,
    .parent = TYPE_ICS_BASE,
    .instance_size = sizeof(Phb3MsiState),
    .class_init = phb3_msi_class_init,
    .class_size = sizeof(ICSStateClass),
    .instance_init = phb3_msi_initfn,
};

static void pnv_phb3_msi_register_types(void)
{
    type_register_static(&phb3_msi_info);
}

type_init(pnv_phb3_msi_register_types)
