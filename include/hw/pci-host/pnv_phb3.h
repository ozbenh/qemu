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

#ifndef PCI_HOST_PNV_PHB3_H
#define PCI_HOST_PNV_PHB3_H

#include "hw/pci/pci_host.h"
#include "hw/ppc/xics.h"
#include "hw/pci-host/pnv_phb3_regs.h"

#define PNV_PHB3_NUM_M64     16
#define PNV_PHB3_NUM_REGS    (0x1000 >> 3)
#define PNV_PHB3_NUM_LSI     8
#define PNV_PHB3_NUM_PE      256

#define PCI_MMIO_TOTAL_SIZE     (0x1ull << 60)

#define IODA2_PCI_BUS_MAX 256

typedef struct PnvPBCQState PnvPBCQState;
typedef struct PnvPhb3State PnvPhb3State;
typedef struct PnvPhb3DMASpace PnvPhb3DMASpace;

/* Similarily with pnv_phb3_msi */
typedef struct Phb3MsiState Phb3MsiState;

/* We have one such address space wrapper per possible device
 * under the PHB since they need to be assigned statically at
 * qemu device creation time. The relationship to a PE is done
 * later dynamically. This means we can potentially create a lot
 * of these guys. Q35 stores them as some kind of radix tree but
 * we never really need to do fast lookups so instead we simply
 * keep a QLIST of them for now, we can add the radix if needed
 * later on.
 *
 * We do cache the PE number to speed things up a bit though.
 */
struct PnvPhb3DMASpace {
    PCIBus *bus;
    uint8_t devfn;
    int pe_num;         /* Cached PE number */
#define PHB_INVALID_PE (-1)
    PnvPhb3State *phb;
    AddressSpace dma_as;
    IOMMUMemoryRegion dma_mr;
    MemoryRegion msi32_mr;
    MemoryRegion msi64_mr;
    bool msi32_mapped;
    bool msi64_mapped;
    QLIST_ENTRY(PnvPhb3DMASpace) list;
};

struct PnvPhb3State {
    PCIHostState parent_obj;

    uint32_t chip_id;
    uint32_t phb_id;
    MemoryRegion mr_m32;
    MemoryRegion mr_m64[PNV_PHB3_NUM_M64];
    MemoryRegion mr_regs;
    bool regs_mapped;
    bool m32_mapped;
    bool m64_mapped[PNV_PHB3_NUM_M64];
    MemoryRegion pci_mmio;
    MemoryRegion pci_io;
    uint64_t regs[PNV_PHB3_NUM_REGS];
    PnvPBCQState *pbcq;
    uint64_t ioda_LIST[8];
    uint64_t ioda_LXIVT[8];
    uint64_t ioda_TVT[512];
    uint64_t ioda_M64BT[16];
    uint64_t ioda_MDT[256];
    uint64_t ioda_PEEV[4];
    uint32_t total_irq;
    ICSState *lsi_ics;
    Phb3MsiState *msis;
    QLIST_HEAD(, PnvPhb3DMASpace) dma_spaces;
};

struct PnvPBCQState {
    DeviceState parent;

    uint32_t chip_id;
    uint32_t phb_id;
    uint32_t nest_xbase;
    uint32_t spci_xbase;
    uint32_t pci_xbase;
    uint64_t nest_regs[PBCQ_NEST_REGS_COUNT];
    uint64_t spci_regs[PBCQ_SPCI_REGS_COUNT];
    uint64_t pci_regs[PBCQ_PCI_REGS_COUNT];
    MemoryRegion mmbar0;
    MemoryRegion mmbar1;
    MemoryRegion phbbar;
    bool mmio0_mapped;
    bool mmio1_mapped;
    bool phb_mapped;
    uint64_t mmio0_base;
    uint64_t mmio0_size;
    uint64_t mmio1_base;
    uint64_t mmio1_size;
    PnvPhb3State *phb;

    MemoryRegion xscom_nest_regs;
    MemoryRegion xscom_pci_regs;
    MemoryRegion xscom_spci_regs;
};

#define TYPE_PNV_PBCQ "pnv-pbcq"
#define PNV_PBCQ(obj) \
     OBJECT_CHECK(PnvPBCQState, (obj), TYPE_PNV_PBCQ)


#define TYPE_PNV_PHB3 "pnv-phb3"
#define PNV_PHB3(obj) \
     OBJECT_CHECK(PnvPhb3State, (obj), TYPE_PNV_PHB3)

#define TYPE_PNV_PHB3_RC "pnv-phb3-rc"

#define PHB3_MAX_MSI     2048

typedef struct Phb3MsiState {
    ICSState ics;
    PnvPhb3State *phb;
    uint64_t rba[PHB3_MAX_MSI / 64];
    uint32_t rba_sum;
} Phb3MsiState;

#define TYPE_PHB3_MSI "phb3-msi"
#define PHB3_MSI(obj) OBJECT_CHECK(Phb3MsiState, (obj), TYPE_PHB3_MSI)

uint64_t pnv_phb3_reg_read(void *opaque, hwaddr off, unsigned size);
void pnv_phb3_reg_write(void *opaque, hwaddr off, uint64_t val, unsigned size);
void pnv_phb3_update_regions(PnvPhb3State *phb);
void pnv_phb3_remap_irqs(PnvPhb3State *phb);
void pnv_phb3_msi_update_config(Phb3MsiState *msis, uint32_t base,
                                uint32_t count);
void pnv_phb3_msi_send(Phb3MsiState *msis, uint64_t addr, uint16_t data,
                       int32_t dev_pe);
void pnv_phb3_msi_ffi(Phb3MsiState *msis, uint64_t val);

#endif /* PCI_HOST_PNV_PHB3_H */
