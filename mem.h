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
#define MEM_18BIT_PERIPH_PAGE_ADDR     MEM_UNIBUS_PERIPH_PAGE_ADDR
#define MEM_22BIT_UNIBUS_ADDR          0x3C0000

#define MEM_SIZE_WORDS  (MEM_22BIT_UNIBUS_ADDR / MEM_WORD_SIZE)
#define MEM_SIZE_BYTES  (MEM_SIZE_WORDS * MEM_WORD_SIZE)

// 22 bit physical address
typedef uint32_t ph_size;
typedef uint32_t ph_addr;

// 18 bit Unibus address
typedef uint32_t un_size;
typedef uint32_t un_addr;

// 16 bit CPU virtual address
typedef uint16_t cpu_word;
typedef uint16_t cpu_addr;

typedef enum
{
    cpu_space_I = 0,
    cpu_space_D = 1,

    _cpu_space_max
} cpu_space;

typedef enum
{
    cpu_mode_Kernel = 0,
    cpu_mode_Supervisor = 1,
    cpu_mode_Invalid = 2,
    cpu_mode_User = 3,

    _cpu_mode_max
} cpu_mode;

#define MEM_HAS_ERR         (1 << 31)
#define MEM_ERR_ILL_HLT     (1 << 7)
#define MEM_ERR_ODD_ADDR    (1 << 6)
#define MEM_ERR_NX_MEM      (1 << 5)
#define MEM_ERR_UNB_TIMEOUT (1 << 4)
#define MEM_ERR_YZ_STACK    (1 << 3)
#define MEM_ERR_RZ_STACK    (1 << 2)
#define MEM_ERR(err)        (MEM_HAS_ERR | (err))

typedef cpu_word (* io_rd_cb)(un_addr, void*);
typedef void (* io_wr_cb)(un_addr, cpu_word, void*);

void mem_init(ph_addr base, const uint8_t* buf, ph_size size);

void mem_registerUnibusIO(un_addr ioStart, un_addr ioEnd, io_rd_cb rd, io_wr_cb wr, void* arg);
void mem_deregisterUnibusIO(un_addr ioStart, un_addr ioEnd);

uint32_t mem_readUnibus(un_addr addr);
uint32_t mem_writeUnibus(un_addr addr, bool bByte, cpu_word data);

uint32_t mem_read(cpu_addr addr, cpu_space s, cpu_mode m);
uint32_t mem_write(cpu_addr addr, cpu_space s, cpu_mode m, bool bByte, cpu_word data);

void mem_mmu_reset(void);
// TODO: Set to false when in T bit, Parity, Odd Address, and Time Out traps and interrupts
//  Note that EMT, TRAP, BPT, and lOT do not set this to true.
void mem_updateMMR0(bool bInstCompleted);
void mem_resetMMR1(void);
void mem_updateMMR1(int reg, int diff);
void mem_updateMMR2(cpu_word val);

#endif //MEM_H_FA19B89AB9CD44F9B591AA96AC60CD93
