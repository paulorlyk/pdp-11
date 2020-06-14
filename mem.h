//
// Created by palulukan on 6/11/20.
//

#ifndef MEM_H_FA19B89AB9CD44F9B591AA96AC60CD93
#define MEM_H_FA19B89AB9CD44F9B591AA96AC60CD93

#include <stdint.h>
#include <stdbool.h>

#define MEM_SIZE_WORDS  32768
#define MEM_SIZE_BYTES  (MEM_SIZE_WORDS * 2)

typedef uint16_t ph_size;
typedef uint16_t ph_addr;

typedef uint16_t cpu_word;
typedef uint16_t cpu_addr;

void mem_init(ph_addr base, const uint8_t* buf, ph_size size);

cpu_word mem_read_physical(ph_addr addr);
void mem_write_physical(ph_addr addr, bool byte, cpu_word data);

#endif //MEM_H_FA19B89AB9CD44F9B591AA96AC60CD93
