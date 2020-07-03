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

enum _mmuGegGroup
{
    _mmu_rgrp_UISDR = 0,    // PDR[cpu_mode_User][cpu_space_I]          -> 0777600 - 0777616
    _mmu_rgrp_UDSDR,        // PDR[cpu_mode_User][cpu_space_D]          -> 0777620 - 0777636
    _mmu_rgrp_UISAR,        // PAR[cpu_mode_User][cpu_space_I]          -> 0777640 - 0777656
    _mmu_rgrp_UDSAR,        // PAR[cpu_mode_User][cpu_space_D]          -> 0777660 - 0777676

    _mmu_rgrp_SISDR,        // PDR[cpu_mode_Supervisor][cpu_space_I]    -> 0772200 - 0772216
    _mmu_rgrp_SDSDR,        // PDR[cpu_mode_Supervisor][cpu_space_D]    -> 0772220 - 0772236
    _mmu_rgrp_SISAR,        // PAR[cpu_mode_Supervisor][cpu_space_I]    -> 0772240 - 0772256
    _mmu_rgrp_SDSAR,        // PAR[cpu_mode_Supervisor][cpu_space_D]    -> 0772260 - 0772276

    _mmu_rgrp_IISDR,        // PDR[cpu_mode_Invalid][cpu_space_I]       -> x - x
    _mmu_rgrp_IDSDR,        // PDR[cpu_mode_Invalid][cpu_space_D]       -> x - x
    _mmu_rgrp_IISAR,        // PAR[cpu_mode_Invalid][cpu_space_I]       -> x - x
    _mmu_rgrp_IDSAR,        // PAR[cpu_mode_Invalid][cpu_space_D]       -> x - x

    _mmu_rgrp_KISDR,        // PDR[cpu_mode_Kernel][cpu_space_I]        -> 0772300 - 0772316
    _mmu_rgrp_KDSDR,        // PDR[cpu_mode_Kernel][cpu_space_D]        -> 0772320 - 0772336
    _mmu_rgrp_KISAR,        // PAR[cpu_mode_Kernel][cpu_space_I]        -> 0772340 - 0772356
    _mmu_rgrp_KDSAR,        // PAR[cpu_mode_Kernel][cpu_space_D]        -> 0772360 - 0772376

    _mmuGegGroup_max
};

static struct _mmuGegGroupInfo
{
    un_addr ioStart;
    un_addr ioEnd;
    cpu_word *base;
} mmuGegGroupsTable[_mmuGegGroup_max] = {
    { 0777600, 0777616, mem.mmu.PDR[cpu_mode_User][cpu_space_I] },          // _mmu_rgrp_UISDR
    { 0777620, 0777636, mem.mmu.PDR[cpu_mode_User][cpu_space_D] },          // _mmu_rgrp_UDSDR
    { 0777640, 0777656, mem.mmu.PAR[cpu_mode_User][cpu_space_I] },          // _mmu_rgrp_UISAR
    { 0777660, 0777676, mem.mmu.PAR[cpu_mode_User][cpu_space_D] },          // _mmu_rgrp_UDSAR

    { 0772200, 0772216, mem.mmu.PDR[cpu_mode_Supervisor][cpu_space_I] },    // _mmu_rgrp_SISDR
    { 0772220, 0772236, mem.mmu.PDR[cpu_mode_Supervisor][cpu_space_D] },    // _mmu_rgrp_SDSDR
    { 0772240, 0772256, mem.mmu.PAR[cpu_mode_Supervisor][cpu_space_I] },    // _mmu_rgrp_SISAR
    { 0772260, 0772276, mem.mmu.PAR[cpu_mode_Supervisor][cpu_space_D] },    // _mmu_rgrp_SDSAR

    //{ 0000000, 0000000, mem.mmu.PDR[cpu_mode_Invalid][cpu_space_I] },       // _mmu_rgrp_IISDR
    //{ 0000000, 0000000, mem.mmu.PDR[cpu_mode_Invalid][cpu_space_D] },       // _mmu_rgrp_IDSDR
    //{ 0000000, 0000000, mem.mmu.PAR[cpu_mode_Invalid][cpu_space_I] },       // _mmu_rgrp_IISAR
    //{ 0000000, 0000000, mem.mmu.PAR[cpu_mode_Invalid][cpu_space_D] },       // _mmu_rgrp_IDSAR

    { 0772300, 0772316, mem.mmu.PDR[cpu_mode_Kernel][cpu_space_I] },        // _mmu_rgrp_KISDR
    { 0772320, 0772336, mem.mmu.PDR[cpu_mode_Kernel][cpu_space_D] },        // _mmu_rgrp_KDSDR
    { 0772340, 0772356, mem.mmu.PAR[cpu_mode_Kernel][cpu_space_I] },        // _mmu_rgrp_KISAR
    { 0772360, 0772376, mem.mmu.PAR[cpu_mode_Kernel][cpu_space_D] },        // _mmu_rgrp_KDSAR
};

static cpu_word _ioDummyRead(un_addr addr, void* arg)
{
    (void)arg;

    // TODO: Trap
    DEBUG("UNIBUS: Reading unknown periphery page address: 0%08o", addr);
    assert(false);
    return 0;
}

static void _ioDummyWrite(un_addr addr, cpu_word data, void* arg)
{
    (void)arg;

    // TODO: Trap
    DEBUG("UNIBUS: Writing unknown periphery page address: 0%08o, data: 0%06o", addr, data);
    assert(false);
}

static cpu_word _mmuRegRead(un_addr addr, void* arg)
{
    struct _mmuGegGroupInfo *pRG = (struct _mmuGegGroupInfo *)arg;

    assert(addr >= pRG->ioStart && addr <= pRG->ioEnd);
    assert(!(addr & 1));

    return pRG->base[(addr - pRG->ioStart) >> 1];
}

static void _mmuRegWrite(un_addr addr, cpu_word data, void* arg)
{
    struct _mmuGegGroupInfo *pRG = (struct _mmuGegGroupInfo *)arg;

    assert(addr >= pRG->ioStart && addr <= pRG->ioEnd);
    assert(!(addr & 1));

    pRG->base[(addr - pRG->ioStart) >> 1] = data;
}

static cpu_word _mmuRead(un_addr addr, void* arg)
{
    (void)arg;

    cpu_word res = 0;

    switch(addr)
    {
        default:
            // TODO: Trap
            assert(false);
            break;

        case 0777572:   // MMR0
            return mem.mmu.MMR[0];

        case 0777574:   // MMR1
            return mem.mmu.MMR[1];

        case 0777576:   // MMR2
            return mem.mmu.MMR[2];

        case 0772516:   // MMR3
            return mem.mmu.MMR[3];
    }

    return 0;
}

static void _mmuWrite(un_addr addr, cpu_word data, void* arg)
{
    (void)arg;

    switch(addr)
    {
        default:
            // TODO: Trap
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
            mem.mmu.MMR[3] = data & MMU_MMR3_WR_MASK;
            break;
    }
}

static struct _periph_io* _getPeriphIO(un_addr addr)
{
    assert((addr & 1) == 0);
    assert(addr <= MEM_UNIBUS_ADDR_MAX);

    int idx = (addr - MEM_UNIBUS_PERIPH_PAGE_ADDR) >> 1;
    return mem.peripheralPageMap + idx;
}

static cpu_word _readIOPage(un_addr addr)
{
    //DEBUG("UNIBUS: I/O read: 0%06o", addr);

    struct _periph_io* pio = _getPeriphIO(addr);
    return pio->read(addr, pio->arg);
}

static void _writeIOPage(un_addr addr, cpu_word data)
{
    //DEBUG("UNIBUS: I/O write: 0%06o -> addr 0%06o", data, addr);

    struct _periph_io* pio = _getPeriphIO(addr);
    pio->write(addr, data, pio->arg);
}

static ph_addr _mmuMap(cpu_addr addr, cpu_space s, cpu_mode m)
{
    assert(s < _cpu_space_max);
    assert(m < _cpu_mode_max);

    if(!(mem.mmu.MMR[0] & MMU_MMR0_MMU_EN))
    {
        // Memory Management Unit is inoperative and addresses are not
        // relocated or protected.
        return addr;
    }

    // TODO: Check if MMU is enabled
    // TODO: Check MMU mode

    // TODO: Read from MMR3 weather D space is enabled or not

    // Select Page Address Register (PAR).
    // Each CPU mode (Kernel, Supervisor and User) has it's own set
    // of PARs and PDRs (Page Descriptor Registers)
    // for each space (Instruction and Data).
    // PAR for current space and mode is selected by
    // Active Page Field (APF) which is 3 most significant
    // bits of the virtual address. The rest 13 bits of the Virtual
    // Address (VA) are called Displacement Field (DF).
    // It consists of Block Number (BN) - higher 7 bits and
    // Displacement In Block (DIB) - lower 6 bits.
    cpu_word PAR = mem.mmu.PAR[m][s][(addr & 0xE000) >> 12];

    // Page Address Field (PAF) of selected PAR is
    // 12 bits on 11/34A and 11/60 and 16 bits on 11/44 and 11/70.
    // It holds the starting address of the page.
    ph_addr physicalAddr = PAR;

    // The Physical Block Number (PBN) is obtained by
    // adding the PAF from PAR to BN from virtual address.
    // PBN will contain our final Physical Address (PA).
    physicalAddr += (addr >> 6) & 0x007F;

    // Get final PA by joining 6 bit DIB from VA to PBN.
    physicalAddr = (physicalAddr << 6) | (addr & 0x003F);

    return physicalAddr;
}

void mem_init(ph_addr base, const uint8_t* buf, ph_size size)
{
    memset(&mem, 0, sizeof(mem));

    for(size_t i = 0; i < sizeof(mem.peripheralPageMap) / sizeof(mem.peripheralPageMap[0]); ++i)
    {
        mem.peripheralPageMap[i].read = &_ioDummyRead;
        mem.peripheralPageMap[i].write = &_ioDummyWrite;
    }

    // Memory management registers
    mem_registerUnibusIO(0777572, 0777576, &_mmuRead, &_mmuWrite, NULL);  // MMR0 - MMR2
    mem_registerUnibusIO(0772516, 0772516, &_mmuRead, &_mmuWrite, NULL);  // MMR3

    for(size_t i = 0; i < sizeof(mmuGegGroupsTable) / sizeof(mmuGegGroupsTable[0]); ++i)
    {
        struct _mmuGegGroupInfo *pRG = mmuGegGroupsTable + i;
        if(pRG->ioStart)
            mem_registerUnibusIO(pRG->ioStart, pRG->ioEnd, &_mmuRegRead, &_mmuRegWrite, pRG);
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
        assert(mem.peripheralPageMap[i].read == &_ioDummyRead);
        assert(mem.peripheralPageMap[i].write == &_ioDummyWrite);

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
        mem.peripheralPageMap[i].read = &_ioDummyRead;
        mem.peripheralPageMap[i].write = &_ioDummyWrite;
        mem.peripheralPageMap[i].arg = NULL;
    }
}

bool mem_readUnibus(un_addr addr, cpu_word* data)
{
    assert((addr & 1) == 0);

    if(addr > MEM_UNIBUS_ADDR_MAX)
        return false;

    if(addr >= MEM_16BIT_PERIPH_PAGE_ADDR)
    {
        *data = _readIOPage(MEM_UNIBUS_PERIPH_PAGE_ADDR + (addr - MEM_16BIT_PERIPH_PAGE_ADDR));
        return true;
    }

    if(addr >= MEM_SIZE_BYTES)
        return false;

    *data = mem.physicalMemory[addr >> 1];

    return true;
}

bool mem_writeUnibus(un_addr addr, bool bByte, cpu_word data)
{
    assert(bByte || (addr & 1) == 0);

    if(addr > MEM_UNIBUS_ADDR_MAX)
        return false;

    if(addr >= MEM_16BIT_PERIPH_PAGE_ADDR)
    {
        // TODO: Trap?
        assert(!bByte);

        _writeIOPage(MEM_UNIBUS_PERIPH_PAGE_ADDR + (addr - MEM_16BIT_PERIPH_PAGE_ADDR), data);
        return true;
    }

    if(addr >= MEM_SIZE_BYTES)
        return false;

    if(bByte)
    {
        cpu_word w = 0;
        if(!mem_readUnibus(addr & ~1U, &w))
            return false;

        int sh = (8 * (addr & 1U));
        cpu_word mask = 0xFF << sh;

        data = (((data & 0xFF) << sh) & mask) | (w & ~mask);
    }

    mem.physicalMemory[addr >> 1] = data;

    return true;
}

bool mem_read(cpu_addr addr, cpu_space s, cpu_mode m, cpu_word* data)
{
    _mmuMap(addr, s, m);

    return mem_readUnibus(addr, data);
}

bool mem_write(cpu_addr addr, cpu_space s, cpu_mode m, bool bByte, cpu_word data)
{
    return mem_writeUnibus(addr, bByte, data);
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
