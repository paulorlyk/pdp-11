//
// Created by palulukan on 6/11/20.
//

#include "mem.h"

#include "log.h"

#include <assert.h>
#include <string.h>

struct _periph_io
{
    io_rd_cb read;
    io_wr_cb write;
};

static struct
{
    cpu_word physicalMemory[MEM_SIZE_WORDS];
    struct _periph_io peripheralPageMap[MEM_PERIPH_PAGE_SIZE_WORDS];
} mem;

static cpu_word _ioDummyRead(ph_addr addr)
{
    // TODO: Trap
    DEBUG("UNIBUS: Reading unknown periphery page address: 0%08o", addr);
    return 0;
}

static void _ioDummyWrite(ph_addr addr, cpu_word data)
{
    // TODO: Trap
    DEBUG("UNIBUS: Writing unknown periphery page address: 0%08o, data: 0%06o", addr, data);
}

static struct _periph_io* _getPeriphIO(ph_addr addr)
{
    assert((addr & 1) == 0);
    assert(addr <= MEM_UNIBUS_ADDR_MAX);

    int idx = (addr - MEM_UNIBUS_PERIPH_PAGE_ADDR) >> 1;
    return mem.peripheralPageMap + idx;
}

static cpu_word _readIOPage(ph_addr addr)
{
    return _getPeriphIO(addr)->read(addr);
}

static void _writeIOPage(ph_addr addr, cpu_word data)
{
    _getPeriphIO(addr)->write(addr, data);
}

void mem_init(ph_addr base, const uint8_t* buf, ph_size size)
{
    memset(&mem, 0, sizeof(mem));

    for(int i = 0; i < sizeof(mem.peripheralPageMap) / sizeof(mem.peripheralPageMap[0]); ++i)
    {
        mem.peripheralPageMap[i].read = &_ioDummyRead;
        mem.peripheralPageMap[i].write = &_ioDummyWrite;
    }

    if(buf)
    {
        assert((base + size) < MEM_SIZE_BYTES);
        memcpy((uint8_t *)mem.physicalMemory + base, buf, size);
    }
}

void mem_register_io(ph_addr ioStart, ph_addr ioEnd, io_rd_cb rd, io_wr_cb wr)
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

    for(ph_addr i = ioStart; i < ioEnd; ++i)
    {
        assert(mem.peripheralPageMap[i].read == &_ioDummyRead);
        assert(mem.peripheralPageMap[i].write == &_ioDummyWrite);

        mem.peripheralPageMap[i].read = rd;
        mem.peripheralPageMap[i].write = wr;
    }
}

void mem_deregister_io(ph_addr ioStart, ph_addr ioEnd)
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

    for(ph_addr i = ioStart; i < ioEnd; ++i)
    {
        mem.peripheralPageMap[i].read = &_ioDummyRead;
        mem.peripheralPageMap[i].write = &_ioDummyWrite;
    }
}

bool mem_read_physical(ph_addr addr, cpu_word* data)
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

bool mem_write_physical(ph_addr addr, bool bByte, cpu_word data)
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
        if(!mem_read_physical(addr & ~1U, &w))
            return false;

        int sh = (8 * (addr & 1U));
        cpu_word mask = 0xFF << sh;

        data = (((data & 0xFF) << sh) & mask) | (w & ~mask);
    }

    mem.physicalMemory[addr >> 1] = data;

    return true;
}
