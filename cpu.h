//
// Created by palulukan on 6/11/20.
//

#ifndef CPU_H_63FF543D3CE340CEA2683539FDDB2A0E
#define CPU_H_63FF543D3CE340CEA2683539FDDB2A0E

#include "mem.h"

#include <stdint.h>

#define GET_BIT_FIELD(word, mask, offset) (((word) & (mask)) >> (offset))
#define SET_BIT_FIELD(word, val, mask, offset) ((word) = ((word) & ~(mask)) | (((val) << (offset)) & (mask)))

// Current processor mode
// Allowed values: 0, 1, 3
#define PSW_CUR_MODE_MASK 0xC000
#define PSW_CUR_MODE_OFFSET 14
#define PSW_GET_CUR_MODE(psw) GET_BIT_FIELD((psw), PSW_CUR_MODE_MASK, PSW_CUR_MODE_OFFSET)
#define PSW_SET_CUR_MODE(psw, val) SET_BIT_FIELD((psw), (val), PSW_CUR_MODE_MASK, PSW_CUR_MODE_OFFSET)

// Previous processor mode
// Allowed values: 0, 1, 3
#define PSW_PREV_MODE_MASK 0x3000
#define PSW_PREV_MODE_OFFSET 12
#define PSW_GET_PREV_MODE(psw) GET_BIT_FIELD((psw), PSW_PREV_MODE_MASK, PSW_PREV_MODE_OFFSET)
#define PSW_SET_PREV_MODE(psw, val) SET_BIT_FIELD((psw), (val), PSW_PREV_MODE_MASK, PSW_PREV_MODE_OFFSET)

#define CPU_MODE_KERNEL     0
#define CPU_MODE_SUPERVISOR 1
#define CPU_MODE_USER       3

// Processor general register set
// Allowed values: 0 - 1
#define PSW_REG_SET_MASK 0x0800
#define PSW_REG_SET_OFFSET 11
#define PSW_GET_REG_SET(psw) GET_BIT_FIELD((psw), PSW_REG_SET_MASK, PSW_REG_SET_OFFSET)
#define PSW_SET_REG_SET(psw, val) SET_BIT_FIELD((psw), (val), PSW_REG_SET_MASK, PSW_REG_SET_OFFSET)

// Processor interrupt priority
// Allowed values: 0 - 7
#define PSW_PRIORITY_MASK 0x00E0
#define PSW_PRIORITY_OFFSET 5
#define PSW_GET_PRIORITY(psw) GET_BIT_FIELD((psw), PSW_PRIORITY_MASK, PSW_PRIORITY_OFFSET)
#define PSW_SET_PRIORITY(psw, val) SET_BIT_FIELD((psw), (val), PSW_PRIORITY_MASK, PSW_PRIORITY_OFFSET)

// Trap flag
// Allowed values: 0, 1
#define PSW_TRAP_MASK 0x0010
#define PSW_TRAP_OFFSET 4
#define PSW_GET_TRAP(psw) GET_BIT_FIELD((psw), PSW_TRAP_MASK, PSW_TRAP_OFFSET)
#define PSW_SET_TRAP(psw, val) SET_BIT_FIELD((psw), (val), PSW_TRAP_MASK, PSW_TRAP_OFFSET)

// Negative flag
// Allowed values: 0, 1
#define PSW_N_MASK 0x0008
#define PSW_N_OFFSET 3
#define PSW_GET_N(psw) GET_BIT_FIELD((psw), PSW_N_MASK, PSW_N_OFFSET)
#define PSW_SET_N(psw, val) SET_BIT_FIELD((psw), (val), PSW_N_MASK, PSW_N_OFFSET)

// Zero flag
// Allowed values: 0, 1
#define PSW_Z_MASK 0x0004
#define PSW_Z_OFFSET 2
#define PSW_GET_Z(psw) GET_BIT_FIELD((psw), PSW_Z_MASK, PSW_Z_OFFSET)
#define PSW_SET_Z(psw, val) SET_BIT_FIELD((psw), (val), PSW_Z_MASK, PSW_Z_OFFSET)

// Overflow flag
// Allowed values: 0, 1
#define PSW_V_MASK 0x0002
#define PSW_V_OFFSET 1
#define PSW_GET_V(psw) GET_BIT_FIELD((psw), PSW_V_MASK, PSW_V_OFFSET)
#define PSW_SET_V(psw, val) SET_BIT_FIELD((psw), (val), PSW_V_MASK, PSW_V_OFFSET)

// Carry flag
// Allowed values: 0, 1
#define PSW_C_MASK 0x0001
#define PSW_C_OFFSET 0
#define PSW_GET_C(psw) GET_BIT_FIELD((psw), PSW_C_MASK, PSW_C_OFFSET)
#define PSW_SET_C(psw, val) SET_BIT_FIELD((psw), (val), PSW_C_MASK, PSW_C_OFFSET)

void cpu_init(cpu_word R7);

void cpu_run(void);

#endif //CPU_H_63FF543D3CE340CEA2683539FDDB2A0E
