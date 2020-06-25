//
// Created by palulukan on 6/14/20.
//

#ifndef RK11_H_FF5871D654BD40E3A9FB94A7EF2C6069
#define RK11_H_FF5871D654BD40E3A9FB94A7EF2C6069

#include "device.h"

#include <stdbool.h>

#define RK05_DISKS_MAX  8
#define RK05_SIZE_WORDS 1247232
#define RK05_SIZE       (RK05_SIZE_WORDS * MEM_WORD_SIZE)

void rk11_init(void);

void rk11_destroy(void);

device_handle rk11_getHandle(void);

// Connect or disconnect the disk without loading
bool rk11_connectDisk(int nDisk, bool connected);

// Connect the disk if not yet connected and load the image
bool rk11_loadDisk(const char* szImagePath, int nDisk);

#endif //RK11_H_FF5871D654BD40E3A9FB94A7EF2C6069
