//
// Created by palulukan on 6/11/20.
//

#include "mem.h"

#include <assert.h>
#include <string.h>

static cpu_word physicalMemory[MEM_SIZE_WORDS];

void mem_init(ph_addr base, const uint8_t* buf, ph_size size)
{
    if(buf)
    {
        assert((base + size) < MEM_SIZE_BYTES);
        memcpy((uint8_t *)physicalMemory + base, buf, size);
    }
}

cpu_word mem_read_physical(ph_addr addr)
{
    assert(addr < MEM_SIZE_BYTES);
    assert((addr & 1) == 0);

    return physicalMemory[addr >> 1];
}

void mem_write_physical(ph_addr addr, bool byte, cpu_word data)
{
    assert(addr < MEM_SIZE_BYTES);
    assert(byte || (addr & 1) == 0);

    if(byte)
    {
        cpu_word w = mem_read_physical(addr & ~1U);

        int sh = (8 * (addr & 1U));
        cpu_word mask = 0xFF << sh;

        data = (((data & 0xFF) << sh) & mask) | (w & ~mask);
    }

    physicalMemory[addr >> 1] = data;
}
