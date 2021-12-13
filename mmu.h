//
// Created by palulukan on 12/11/21.
//

#ifndef MMU_H_029A64D49B5B4802B292BF482C6E04DA
#define MMU_H_029A64D49B5B4802B292BF482C6E04DA

#include "mem.h"

void mmu_init(void);
void mmu_reset(void);
// TODO: Set to false when in T bit, Parity, Odd Address, and Time Out traps and interrupts
//  Note that EMT, TRAP, BPT, and lOT do not set this to true.
void mmu_updateMMR0(bool bInstCompleted);
void mmu_resetMMR1(void);
void mmu_updateMMR1(int reg, int diff);
void mmu_updateMMR2(cpu_word val);

ph_addr mmu_map(cpu_addr addr, cpu_space s, cpu_mode m, bool bWR);

#endif //MMU_H_029A64D49B5B4802B292BF482C6E04DA
