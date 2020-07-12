//
// Created by palulukan on 6/24/20.
//

#ifndef DEVICE_H_E15B380E25784BBFBED27E08B1D75F34
#define DEVICE_H_E15B380E25784BBFBED27E08B1D75F34

#include "mem.h"

#define IRQ_PRIORITY_MAX 7

typedef void* device_handle;

typedef cpu_word (*irq_ack_cb)(void *);
typedef void (*dev_reset_cb)(void *);

typedef struct
{
    un_addr ioStart;    // First address of I/O region
    un_addr ioEnd;      // Last address of I/O region
    io_rd_cb rd;
    io_wr_cb wr;
    void* arg;
} dev_io_info;

void dev_init(void);

// name - pointer to statically allocated null-terminated string. Used for debugging purposes. May be NULL.
// ioMap - array of dev_io_info structures terminated by element with NULL value in .rd and/or .wr fields.
//      May be NULL if device does not have any memory mapped IO.
//      Fields of dev_io_info must comply to the following requirements:
//          - ioStart <= ioEnd
//          - (ioStart & 1) == 0
//          - (ioEnd & 1) == 0
//          - ioStart > MEM_UNIBUS_PERIPH_PAGE_ADDR
//          - ioEnd >= MEM_UNIBUS_PERIPH_PAGE_ADDR
//          - ioStart <= MEM_UNIBUS_ADDR_MAX
//          - ioEnd <= MEM_UNIBUS_ADDR_MAX
//          - rd != NULL (except of the last element in the ioMap array)
//          - wr != NULL (except of the last element in the ioMap array)
// irqPriority - must be positive and less then IRQ_PRIORITY_MAX.
// irqACK - IRQ acknowledgment callback, may be NULL if device does not
//          require any interrupts.
// devReset - callback for UNIBUS INIT signal. May be NULL.
// arg - argument with which irqACK and devReset callbacks will be invoked.
device_handle dev_initDevice(const char* name,
                             const dev_io_info *ioMap,
                             int irqPriority,
                             irq_ack_cb irqACK,
                             dev_reset_cb devReset,
                             void* arg);

void dev_destroyDevice(device_handle device);

void dev_registerDevice(device_handle device);

void dev_deregisterDevice(device_handle device);

void dev_setIRQ(device_handle device);

void dev_clearIRQ(device_handle device);

const char* dev_getName(device_handle device);

void dev_reset(void);

device_handle dev_getIRQ(int minPriority);

cpu_word dev_ackIRQ(device_handle device);

#endif //DEVICE_H_E15B380E25784BBFBED27E08B1D75F34
