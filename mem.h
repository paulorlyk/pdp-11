//
// Created by palulukan on 6/11/20.
//

#ifndef MEM_H_FA19B89AB9CD44F9B591AA96AC60CD93
#define MEM_H_FA19B89AB9CD44F9B591AA96AC60CD93

#include <stdint.h>
#include <stdbool.h>

#define MEM_UNIBUS_PERIPH_PAGE_ADDR 0x3E000
#define MEM_UNIBUS_ADDR_MAX         0x3FFFF

#define MEM_WORD_SIZE   2

#define MEM_PERIPH_PAGE_SIZE_WORDS ((MEM_UNIBUS_ADDR_MAX - MEM_UNIBUS_PERIPH_PAGE_ADDR + 1) / MEM_WORD_SIZE)

#define MEM_16BIT_PERIPH_PAGE_ADDR     0xE000

#define MEM_SIZE_WORDS  (MEM_16BIT_PERIPH_PAGE_ADDR / MEM_WORD_SIZE)
#define MEM_SIZE_BYTES  (MEM_SIZE_WORDS * MEM_WORD_SIZE)

typedef uint32_t ph_size;
typedef uint32_t ph_addr;

typedef uint16_t cpu_word;
typedef uint16_t cpu_addr;

typedef cpu_word (* io_rd_cb)(ph_addr, void*);
typedef void (* io_wr_cb)(ph_addr, cpu_word, void*);

void mem_init(ph_addr base, const uint8_t* buf, ph_size size);

void mem_register_io(ph_addr ioStart, ph_addr ioEnd, io_rd_cb rd, io_wr_cb wr, void* arg);

void mem_deregister_io(ph_addr ioStart, ph_addr ioEnd);

bool mem_read_physical(ph_addr addr, cpu_word* data);
bool mem_write_physical(ph_addr addr, bool bByte, cpu_word data);

#endif //MEM_H_FA19B89AB9CD44F9B591AA96AC60CD93
