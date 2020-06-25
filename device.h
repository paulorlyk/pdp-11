//
// Created by palulukan on 6/24/20.
//

#ifndef DEVICE_H_E15B380E25784BBFBED27E08B1D75F34
#define DEVICE_H_E15B380E25784BBFBED27E08B1D75F34

#include "mem.h"

#define IRQ_PRIORITY_MAX 7

typedef void* device_handle;

typedef void (*irq_ack_cb)(void);

void dev_init(void);

device_handle dev_initDevice(ph_addr ioStart, ph_addr ioEnd, io_rd_cb rd, io_wr_cb wr, cpu_word irq, int irqPriority, irq_ack_cb irqACK);

void dev_destroyDevice(device_handle device);

void dev_registerDevice(device_handle device);

void dev_deregisterDevice(device_handle device);

void dev_setIRQ(device_handle device);

void dev_clearIRQ(device_handle device);

device_handle dev_getIRQ(int minPriority);

cpu_word dev_ackIRQ(device_handle device);

#endif //DEVICE_H_E15B380E25784BBFBED27E08B1D75F34
