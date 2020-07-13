//
// Created by palulukan on 6/11/20.
//

#include "mem.h"

#include "log.h"

#include <assert.h>
#include <string.h>

#define MMU_MMR0_MMU_EN         (1 << 0)    // Enable Relocation
#define MMU_MMR0_PAGE_NO_MASK   0x000E      // Page Number
#define MMU_MMR0_SET_PAGE_NO(mmr, page) ((mmr) = ((mmr) & ~MMU_MMR0_PAGE_NO_MASK) | (((page) << 1) & MMU_MMR0_PAGE_NO_MASK))
#define MMU_MMR0_PAGE_SPACE     (1 << 4)    // Page Address Space
#define MMU_MMR0_CPU_MODE_MASK  0x0060      // Processor Mode
#define MMU_MMR0_SET_CPU_MODE_NO(mmr, mode) ((mmr) = ((mmr) & ~MMU_MMR0_CPU_MODE_MASK) | (((mode) << 5) & MMU_MMR0_CPU_MODE_MASK))
#define MMU_MMR0_INST_COMPLETED (1 << 7)    // Instruction Completed
#define MMU_MMR0_DST_MODE       (1 << 8)    // Maintenance/Destination Mode
#define MMU_MMR0_TRAP_EN        (1 << 9)    // Enable Memory Management Traps
#define MMU_MMR0_TRAP           (1 << 12)   // Trap - set whenever a Memory Management trap condition occurs
#define MMU_MMR0_ERR_ABRT_RO    (1 << 13)   // Abort - Read Only
#define MMU_MMR0_ERR_ABRT_PL    (1 << 14)   // Abort - Page Length
#define MMU_MMR0_ERR_ABRT_NR    (1 << 15)   // Abort - Non-Resident
#define MMU_MMR0_ERR_MASK   (MMU_MMR0_ERR_ABRT_RO | MMU_MMR0_ERR_ABRT_PL | MMU_MMR0_ERR_ABRT_NR)
#define MMU_MMR0_WR_MASK    (MMU_MMR0_MMU_EN | MMU_MMR0_PAGE_NO_MASK | MMU_MMR0_PAGE_SPACE \
    | MMU_MMR0_CPU_MODE_MASK | MMU_MMR0_DST_MODE | MMU_MMR0_TRAP_EN | MMU_MMR0_TRAP | MMU_MMR0_ERR_MASK)

#define MMU_MMR3_USR_D_EN   (1 << 0)    // Enable User D Space
#define MMU_MMR3_SVR_D_EN   (1 << 1)    // Enable Supervisor D Space
#define MMU_MMR3_KRN_D_EN   (1 << 2)    // Enable Kernel D Space
#define MMU_MMR3_22BIT_MAP  (1 << 4)    // Enable 22-bit mapping
#define MMU_MMR3_UB_MAP_REL (1 << 5)    // Enable UNIBUS Map relocation
#define MMU_MMR3_WR_MASK (MMU_MMR3_USR_D_EN | MMU_MMR3_SVR_D_EN | MMU_MMR3_KRN_D_EN | MMU_MMR3_22BIT_MAP | MMU_MMR3_UB_MAP_REL)

#define PDR_ACF_MASK        0x0007
#define PDR_GET_ACF(pdr)    ((pdr) & PDR_ACF_MASK)
#define PDR_ED              (1 << 3)
#define PDR_W               (1 << 6)
#define PDR_A               (1 << 7)
#define PDR_PLF_MASK        0x7F00
#define PDR_GET_PLF(pdr)    (((pdr) >> 8) & 0x7F)
#define PDR_WR_MASK         (PDR_PLF_MASK | PDR_ED | PDR_ACF_MASK)

struct _periph_io
{
    io_rd_cb read;
    io_wr_cb write;
    void* arg;
};

static struct
{
    cpu_word physicalMemory[MEM_SIZE_WORDS];
    struct _periph_io peripheralPageMap[MEM_PERIPH_PAGE_SIZE_WORDS];

    struct
    {
        cpu_word MMR[4];
        cpu_word PAR[_cpu_mode_max][_cpu_space_max][8];
        cpu_word PDR[_cpu_mode_max][_cpu_space_max][8];
    } mmu;
} mem;

static struct _mmuRegGroupInfo
{
    un_addr ioStart;
    un_addr ioEnd;
    cpu_mode m;
    cpu_space s;
} mmuGegGroupsTable[2][6] = {
    {
        // PDR
        { 0777600, 0777616, cpu_mode_User, cpu_space_I },       // UISDR[0-7]
        { 0777620, 0777636, cpu_mode_User, cpu_space_D },       // UDSDR[0-7]

        { 0772200, 0772216, cpu_mode_Supervisor, cpu_space_I }, // SISDR[0-7]
        { 0772220, 0772236, cpu_mode_Supervisor, cpu_space_D }, // SDSDR[0-7]

        { 0772300, 0772316, cpu_mode_Kernel, cpu_space_I },     // KISDR[0-7]
        { 0772320, 0772336, cpu_mode_Kernel, cpu_space_D },     // KDSDR[0-7]
    },
    {
        // PAR
        { 0777640, 0777656, cpu_mode_User, cpu_space_I },       // UISAR[0-7]
        { 0777660, 0777676, cpu_mode_User, cpu_space_D },       // UDSAR[0-7]

        { 0772240, 0772256, cpu_mode_Supervisor, cpu_space_I }, // SISAR[0-7]
        { 0772260, 0772276, cpu_mode_Supervisor, cpu_space_D }, // SDSAR[0-7]

        { 0772340, 0772356, cpu_mode_Kernel, cpu_space_I },     // KISAR[0-7]
        { 0772360, 0772376, cpu_mode_Kernel, cpu_space_D },     // KDSAR[0-7]
    }
};

static cpu_word _mmuRegReadPDR(un_addr addr, void* arg)
{
    struct _mmuRegGroupInfo *pRG = (struct _mmuRegGroupInfo *)arg;

    assert(addr >= pRG->ioStart && addr <= pRG->ioEnd);
    assert(!(addr & 1));

    int nReg = (addr - pRG->ioStart) >> 1;

    //DEBUG("MMU: Reg RD PDR%d", nReg);

    return mem.mmu.PDR[pRG->m][pRG->s][nReg];
}

static cpu_word _mmuRegReadPAR(un_addr addr, void* arg)
{
    struct _mmuRegGroupInfo *pRG = (struct _mmuRegGroupInfo *)arg;

    assert(addr >= pRG->ioStart && addr <= pRG->ioEnd);
    assert(!(addr & 1));

    int nReg = (addr - pRG->ioStart) >> 1;

    //DEBUG("MMU: Reg RD PAR%d", nReg);

    return mem.mmu.PAR[pRG->m][pRG->s][nReg];
}

static void _mmuRegWritePAR(un_addr addr, cpu_word data, void* arg)
{
    struct _mmuRegGroupInfo *pRG = (struct _mmuRegGroupInfo *)arg;

    assert(addr >= pRG->ioStart && addr <= pRG->ioEnd);
    assert(!(addr & 1));

    int nReg = (addr - pRG->ioStart) >> 1;

    //DEBUG("MMU: Reg WR PAR%d", nReg);

    mem.mmu.PAR[pRG->m][pRG->s][nReg] = data;
    mem.mmu.PDR[pRG->m][pRG->s][nReg] &= ~(PDR_A | PDR_W);
}

static void _mmuRegWritePDR(un_addr addr, cpu_word data, void* arg)
{
    struct _mmuRegGroupInfo *pRG = (struct _mmuRegGroupInfo *)arg;

    assert(addr >= pRG->ioStart && addr <= pRG->ioEnd);
    assert(!(addr & 1));

    int nReg = (addr - pRG->ioStart) >> 1;

    //DEBUG("MMU: Reg WR PDR%d", nReg);

    mem.mmu.PDR[pRG->m][pRG->s][nReg] = data & PDR_WR_MASK;
}

static cpu_word _regRead(un_addr addr, void* arg)
{
    (void)arg;

    switch(addr)
    {
        case 0777572:   // MMR0
            return mem.mmu.MMR[0];

        case 0777574:   // MMR1
            return mem.mmu.MMR[1];

        case 0777576:   // MMR2
            return mem.mmu.MMR[2];

        case 0772516:   // MMR3
            return mem.mmu.MMR[3];

//        case 0777760:   // Lower Size Register
//            return (MEM_SIZE_BYTES >> 6) - 1;
//
//        case 0777762:   // Upper Size Register
//            return 0;
    }

    assert(false);
    return 0;
}

static void _regWrite(un_addr addr, cpu_word data, void* arg)
{
    (void)arg;

    switch(addr)
    {
        default:
            assert(false);
            break;

        case 0777572:   // MMR0
            assert(!(data & MMU_MMR0_DST_MODE));    // TODO: Add support for maintenance mode
            mem.mmu.MMR[0] = data & MMU_MMR0_WR_MASK;
            break;

        case 0777574:   // MMR1
            mem.mmu.MMR[1] = data;
            break;

        case 0777576:   // MMR2 is read-only
            break;

        case 0772516:   // MMR3
            assert(!(data & MMU_MMR3_22BIT_MAP));   // TODO: Implement
            assert(!(data & MMU_MMR3_UB_MAP_REL));  // TODO: Implement
            mem.mmu.MMR[3] = data & MMU_MMR3_WR_MASK;
            break;

//        case 0777760:   // Lower Size Register
//        case 0777762:   // Upper Size Register
//            break;
    }
}

static struct _periph_io* _getPeriphIO(un_addr addr)
{
    assert((addr & 1) == 0);
    assert(addr <= MEM_UNIBUS_ADDR_MAX);

    int idx = (addr - MEM_UNIBUS_PERIPH_PAGE_ADDR) >> 1;
    return mem.peripheralPageMap + idx;
}

static uint32_t _mmuMap(cpu_addr addr, cpu_space s, cpu_mode m, bool bWR)
{
    assert(s < _cpu_space_max);
    assert(m < _cpu_mode_max);

    if(!(mem.mmu.MMR[0] & MMU_MMR0_MMU_EN))
    {
        // Memory Management Unit is inoperative and addresses are not
        // relocated or protected.

        ph_addr PA = addr;
        if(addr >= MEM_16BIT_PERIPH_PAGE_ADDR)
        {
            PA -= MEM_16BIT_PERIPH_PAGE_ADDR;
            PA += MEM_UNIBUS_PERIPH_PAGE_ADDR + MEM_22BIT_UNIBUS_ADDR;
        }

        return PA;
    }

    if(m == cpu_mode_Invalid)
    {
        // TODO: Trap
        assert(false);
    }

    if(   (m == cpu_mode_Kernel && !(mem.mmu.MMR[3] & MMU_MMR3_KRN_D_EN))
       || (m == cpu_mode_Supervisor && !(mem.mmu.MMR[3] & MMU_MMR3_SVR_D_EN))
       || (m == cpu_mode_User && !(mem.mmu.MMR[3] & MMU_MMR3_USR_D_EN)))
    {
        // When D space is disabled, all references use the I space registers
        s = cpu_space_I;
    }

    // TODO: Check MMU mode

    // Select Page Address Register (PAR) and Page Descriptor Register (PDR).
    // Each CPU mode (Kernel, Supervisor and User) has it's own set
    // of PARs and PDRs for each space (Instruction and Data).
    // PAR for current space and mode is selected by
    // Active Page Field (APF) which is 3 most significant
    // bits of the virtual address. The rest 13 bits of the Virtual
    // Address (VA) are called Displacement Field (DF).
    // It consists of Block Number (BN) - higher 7 bits and
    // Displacement In Block (DIB) - lower 6 bits.
    cpu_word PAR = mem.mmu.PAR[m][s][(addr >> 13) & 7];
    cpu_word *pPDR = mem.mmu.PDR[m][s] + ((addr >> 13) & 7);

    cpu_word BN = (addr >> 6) & 0x007F;

    if(((*pPDR & PDR_ED) && (BN < PDR_GET_PLF(*pPDR))) || (!(*pPDR & PDR_ED) && (BN > PDR_GET_PLF(*pPDR))))
    {
        // TODO: Page length violation
        assert(false);
    }

    switch(*pPDR & PDR_ACF_MASK)
    {
        case 0: // non-resident: abort all accesses
        case 3: // unused: abort all accesses - reserved for future use
        case 7: // unused: abort all accesses - reserved for future use
            assert(false);  // TODO: Implement

        case 1: // read-only: abort on write attempt, memory management trap on read
            assert(false);  // TODO: Implement

        case 2: // read-only: abort on write attempt
        {
            *pPDR |= PDR_A;

            if(bWR)
                assert(false);  // TODO: Implement

            break;
        }

        case 4: // read/write: memory management trap upon completion of a read or write
            assert(false);  // TODO: Implement

        case 5: // read/write: memory management trap upon completion of a write
            assert(false);  // TODO: Implement

        case 6: // read/write: no system trap/abort action
            *pPDR |= bWR ? (PDR_A | PDR_W) : PDR_A;
            break;
    }

    // Page Address Field (PAF) of selected PAR is
    // 12 bits on 11/34A and 11/60 and 16 bits on 11/44 and 11/70.
    // It holds the starting address of the page.
    ph_addr PA = PAR;

    // The Physical Block Number (PBN) is obtained by
    // adding the PAF from PAR to BN from virtual address.
    // PBN will contain our final Physical Address (PA).
    PA += BN;

    // Get final PA by joining 6 bit DIB from VA to PBN.
    PA = (PA << 6) | (addr & 0x003F);

//    if(*pAddr != addr)
//        DEBUG("MAP!");

//    if(PA >= 0x3E000)
//        DEBUG("dbg");

    if(!(mem.mmu.MMR[3] & MMU_MMR3_22BIT_MAP))
    {
        if(PA >= MEM_18BIT_PERIPH_PAGE_ADDR)
        {
            PA -= MEM_18BIT_PERIPH_PAGE_ADDR;
            PA += MEM_UNIBUS_PERIPH_PAGE_ADDR + MEM_22BIT_UNIBUS_ADDR;
        }
    }
    else
        assert(false);  // TODO: Implement 22-bit mapping

    return PA;
}

void mem_init(ph_addr base, const uint8_t* buf, ph_size size)
{
    memset(&mem, 0, sizeof(mem));

    // Memory management registers
    mem_registerUnibusIO(0777572, 0777576, &_regRead, &_regWrite, NULL);  // MMR0 - MMR2
    mem_registerUnibusIO(0772516, 0772516, &_regRead, &_regWrite, NULL);  // MMR3
//    mem_registerUnibusIO(0777760, 0777762, &_regRead, &_regWrite, NULL);  // Lower and Upper Size Register

    for(size_t i = 0; i < sizeof(mmuGegGroupsTable[0]) / sizeof(mmuGegGroupsTable[0][0]); ++i)
    {
        struct _mmuRegGroupInfo *pRG = mmuGegGroupsTable[0] + i;
        mem_registerUnibusIO(pRG->ioStart, pRG->ioEnd, &_mmuRegReadPDR, &_mmuRegWritePDR, pRG);

        pRG = mmuGegGroupsTable[1] + i;
        mem_registerUnibusIO(pRG->ioStart, pRG->ioEnd, &_mmuRegReadPAR, &_mmuRegWritePAR, pRG);
    }

    mem_mmu_reset();

    if(buf)
    {
        assert((base + size) < MEM_SIZE_BYTES);
        memcpy((uint8_t *)mem.physicalMemory + base, buf, size);
    }
}

void mem_registerUnibusIO(un_addr ioStart, un_addr ioEnd, io_rd_cb rd, io_wr_cb wr, void* arg)
{
    assert(ioStart <= ioEnd);
    assert((ioStart & 1) == 0);
    assert((ioEnd & 1) == 0);
    assert(ioStart >= MEM_UNIBUS_PERIPH_PAGE_ADDR);
    assert(ioEnd >= MEM_UNIBUS_PERIPH_PAGE_ADDR);
    assert(ioStart <= MEM_UNIBUS_ADDR_MAX);
    assert(ioEnd <= MEM_UNIBUS_ADDR_MAX);
    assert(rd);
    assert(wr);

    ioStart = (ioStart - MEM_UNIBUS_PERIPH_PAGE_ADDR) >> 1;
    ioEnd = (ioEnd - MEM_UNIBUS_PERIPH_PAGE_ADDR) >> 1;

    for(un_addr i = ioStart; i <= ioEnd; ++i)
    {
        assert(!mem.peripheralPageMap[i].read);
        assert(!mem.peripheralPageMap[i].write);

        mem.peripheralPageMap[i].read = rd;
        mem.peripheralPageMap[i].write = wr;
        mem.peripheralPageMap[i].arg = arg;
    }
}

void mem_deregisterUnibusIO(un_addr ioStart, un_addr ioEnd)
{
    assert(ioStart <= ioEnd);
    assert((ioStart & 1) == 0);
    assert((ioEnd & 1) == 0);
    assert(ioStart >= MEM_UNIBUS_PERIPH_PAGE_ADDR);
    assert(ioEnd >= MEM_UNIBUS_PERIPH_PAGE_ADDR);
    assert(ioStart <= MEM_UNIBUS_ADDR_MAX);
    assert(ioEnd <= MEM_UNIBUS_ADDR_MAX);

    ioStart = (ioStart - MEM_UNIBUS_PERIPH_PAGE_ADDR) >> 1;
    ioEnd = (ioEnd - MEM_UNIBUS_PERIPH_PAGE_ADDR) >> 1;

    for(un_addr i = ioStart; i < ioEnd; ++i)
    {
        mem.peripheralPageMap[i].read = NULL;
        mem.peripheralPageMap[i].write = NULL;
        mem.peripheralPageMap[i].arg = NULL;
    }
}

static uint32_t _readRAM(ph_addr addr)
{
    assert((addr & 1) == 0);

    if(addr >= MEM_SIZE_BYTES)
        return MEM_ERR(MEM_ERR_NX_MEM);

//    if(addr >= 070310 && addr < 071310)
//        DEBUG("!");

    return mem.physicalMemory[addr >> 1];
}

static uint32_t _writeRAM(ph_addr addr, bool bByte, cpu_word data)
{
    assert(bByte || (addr & 1) == 0);

    if(addr >= MEM_SIZE_BYTES)
        return MEM_ERR(MEM_ERR_NX_MEM);

//    if(addr >= 070310 && addr < 071310)
//        DEBUG("!");

    if(bByte)
    {
        int sh = (8 * (addr & 1U));
        cpu_word mask = 0xFF << sh;

        cpu_word w = mem.physicalMemory[addr >> 1];
        data = (((data & 0xFF) << sh) & mask) | (w & ~mask);
    }

    mem.physicalMemory[addr >> 1] = data;

    return 0;
}

uint32_t mem_readUnibus(un_addr addr)
{
    assert((addr & 1) == 0);

    assert(addr <= MEM_UNIBUS_ADDR_MAX);    // TODO: Debug
    addr &= MEM_UNIBUS_ADDR_MAX;

    if(addr < MEM_UNIBUS_PERIPH_PAGE_ADDR)
        return _readRAM(addr);

    //DEBUG("UNIBUS: I/O read: 0%06o", addr);

    struct _periph_io* pio = _getPeriphIO(addr);
    if(!pio->read)
    {
        DEBUG("UNIBUS: Reading unknown periphery page address: 0%08o", addr);
        return MEM_ERR(MEM_ERR_UNB_TIMEOUT);
    }

    return pio->read(addr, pio->arg);
}

uint32_t mem_writeUnibus(un_addr addr, bool bByte, cpu_word data)
{
    assert(bByte || (addr & 1) == 0);

    assert(addr <= MEM_UNIBUS_ADDR_MAX);    // TODO: Debug
    addr &= MEM_UNIBUS_ADDR_MAX;

    if(addr < MEM_UNIBUS_PERIPH_PAGE_ADDR)
        return _writeRAM(addr, bByte, data);

    // TODO: Trap?
    assert(!bByte);

    //DEBUG("UNIBUS: I/O write: 0%06o -> addr 0%06o", data, addr);

    struct _periph_io* pio = _getPeriphIO(addr);
    if(!pio->write)
    {
        DEBUG("UNIBUS: Writing unknown periphery page address: 0%08o, data: 0%06o", addr, data);
        return MEM_ERR(MEM_ERR_UNB_TIMEOUT);
    }

    pio->write(addr, data, pio->arg);

    return 0;
}

uint32_t mem_read(cpu_addr addr, cpu_space s, cpu_mode m)
{
    ph_addr pa = _mmuMap(addr, s, m, false);
    if(pa & MEM_HAS_ERR)
        return pa;

    if(pa >= MEM_22BIT_UNIBUS_ADDR)
        return mem_readUnibus(pa - MEM_22BIT_UNIBUS_ADDR);

    return _readRAM(pa);
}

uint32_t mem_write(cpu_addr addr, cpu_space s, cpu_mode m, bool bByte, cpu_word data)
{
    ph_addr pa = _mmuMap(addr, s, m, true);
    if(pa & MEM_HAS_ERR)
        return false;

    if(pa >= MEM_22BIT_UNIBUS_ADDR)
        return mem_writeUnibus(pa - MEM_22BIT_UNIBUS_ADDR, bByte, data);

    return _writeRAM(pa, bByte, data);
}

void mem_mmu_reset(void)
{
    memset(&mem.mmu, 0, sizeof(mem.mmu));
    mem.mmu.MMR[0] |= MMU_MMR0_INST_COMPLETED | MMU_MMR0_TRAP_EN;
}

void mem_updateMMR0(bool bInstCompleted)
{
    mem.mmu.MMR[0] = (mem.mmu.MMR[0] & ~MMU_MMR0_INST_COMPLETED) | (bInstCompleted ? MMU_MMR0_INST_COMPLETED : 0);
}
void mem_resetMMR1(void)
{
    if(!(mem.mmu.MMR[0] & MMU_MMR0_ERR_MASK))
        mem.mmu.MMR[1] = 0;
}

void mem_updateMMR1(int reg, int diff)
{
    assert(reg >= 0);
    assert(reg < 8);
    assert(diff >= -16);
    assert(diff < 16);

    if(!(mem.mmu.MMR[0] & MMU_MMR0_ERR_MASK))
    {
        mem.mmu.MMR[1] <<= 8;
        mem.mmu.MMR[1] = ((diff << 3) & 0x00F8) | (reg & 7);
    }
}

void mem_updateMMR2(cpu_word val)
{
    if(!(mem.mmu.MMR[0] & MMU_MMR0_ERR_MASK))
        mem.mmu.MMR[2] = val;
}
