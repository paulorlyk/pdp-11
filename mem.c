//
// Created by palulukan on 6/11/20.
//

#include "mem.h"

#include "log.h"

#include "mmu.h"

#include <assert.h>
#include <string.h>

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
} mem;

static struct _periph_io* _getPeriphIO(un_addr addr)
{
    assert((addr & 1) == 0);
    assert(addr <= MEM_UNIBUS_ADDR_MAX);

    int idx = (addr - MEM_UNIBUS_PERIPH_PAGE_ADDR) >> 1;
    return mem.peripheralPageMap + idx;
}

void mem_init(ph_addr base, const uint8_t* buf, ph_size size)
{
    memset(&mem, 0, sizeof(mem));

    if(buf)
    {
        assert((base + size) < MEM_SIZE_BYTES);
        memcpy((uint8_t *)mem.physicalMemory + base, buf, size);
    }

    mmu_init();
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

    return mem.physicalMemory[addr >> 1];
}

static uint32_t _writeRAM(ph_addr addr, bool bByte, cpu_word data)
{
    assert(bByte || (addr & 1) == 0);

    if(addr >= MEM_SIZE_BYTES)
        return MEM_ERR(MEM_ERR_NX_MEM);

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
    ph_addr pa = mmu_map(addr, s, m, false);
    if(pa & MEM_HAS_ERR)
        return pa;

    if(pa >= MEM_22BIT_UNIBUS_ADDR)
        return mem_readUnibus(pa - MEM_22BIT_UNIBUS_ADDR);

    return _readRAM(pa);
}

uint32_t mem_write(cpu_addr addr, cpu_space s, cpu_mode m, bool bByte, cpu_word data)
{
    ph_addr pa = mmu_map(addr, s, m, true);
    if(pa & MEM_HAS_ERR)
        return false;

    if(pa >= MEM_22BIT_UNIBUS_ADDR)
        return mem_writeUnibus(pa - MEM_22BIT_UNIBUS_ADDR, bByte, data);

    return _writeRAM(pa, bByte, data);
}
