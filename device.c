//
// Created by palulukan on 6/24/20.
//

#include "device.h"

#include <string.h>
#include <stdlib.h>
#include <stdbool.h>
#include <assert.h>

struct _device
{
    ph_addr ioStart;
    ph_addr ioEnd;
    io_rd_cb rd;
    io_wr_cb wr;
    irq_ack_cb irqACK;
    void* arg;
    int irqPriority;
    bool bIRQ;
    bool registered;
    struct _device *next;
    struct _device *prev;
};

struct
{
    struct _device *curDevIRQ;
    struct _device *head;
    struct _device *tail;
} devices;

void _updateIRQ(void)
{
    devices.curDevIRQ = NULL;

    for(struct _device *it = devices.head; it; it = it->next)
    {
        if(it->bIRQ && (!devices.curDevIRQ || devices.curDevIRQ->irqPriority < it->irqPriority))
        {
            devices.curDevIRQ = it;

            if(devices.curDevIRQ->irqPriority == IRQ_PRIORITY_MAX)
                break;
        }
    }
}

void dev_init(void)
{
    memset(&devices, 0, sizeof(devices));
}

device_handle dev_initDevice(ph_addr ioStart, ph_addr ioEnd, io_rd_cb rd, io_wr_cb wr, int irqPriority, irq_ack_cb irqACK, void* arg)
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
    assert(irqPriority >= 0 && irqPriority <= IRQ_PRIORITY_MAX);
    assert(irqACK);

    struct _device *pDev = calloc(1, sizeof(struct _device));
    if(!pDev)
        return NULL;

    pDev->ioStart = ioStart;
    pDev->ioEnd = ioEnd;
    pDev->rd = rd;
    pDev->wr = wr;
    pDev->irqPriority = irqPriority;
    pDev->irqACK = irqACK;
    pDev->arg = arg;

    return pDev;
}

void dev_destroyDevice(device_handle device)
{
    if(!device)
        return;

    dev_deregisterDevice(device);

    free(device);
}

void dev_registerDevice(device_handle device)
{
    struct _device *pDev = (struct _device *)device;

    if(!pDev || pDev->registered)
        return;

    pDev->registered = true;
    pDev->next = NULL;
    pDev->prev = devices.tail;

    if(!devices.head)
        devices.head = pDev;
    else
        devices.tail->next = pDev;

    devices.tail = pDev;

    if(pDev->bIRQ && (!devices.curDevIRQ || devices.curDevIRQ->irqPriority <= pDev->irqPriority))
        _updateIRQ();

    mem_register_io(pDev->ioStart, pDev->ioEnd, pDev->rd, pDev->wr, pDev->arg);
}

void dev_deregisterDevice(device_handle device)
{
    struct _device *pDev = (struct _device *)device;

    if(!pDev || !pDev->registered)
        return;

    if(pDev->prev)
        pDev->prev->next = pDev->next;
    if(pDev->next)
        pDev->next->prev = pDev->prev;

    pDev->prev = NULL;
    pDev->next = NULL;
    pDev->registered = false;

    mem_deregister_io(pDev->ioStart, pDev->ioEnd);
}

void dev_setIRQ(device_handle device)
{
    struct _device *pDev = (struct _device *)device;

    if(!pDev)
        return;

    pDev->bIRQ = true;

    if(pDev->registered && (!devices.curDevIRQ || devices.curDevIRQ->irqPriority <= pDev->irqPriority))
        _updateIRQ();
}

void dev_clearIRQ(device_handle device)
{
    struct _device *pDev = (struct _device *)device;

    if(!pDev)
        return;

    pDev->bIRQ = false;

    if(devices.curDevIRQ == pDev)
        _updateIRQ();
}

device_handle dev_getIRQ(int minPriority)
{
    if(devices.curDevIRQ && devices.curDevIRQ->irqPriority >= minPriority)
        return devices.curDevIRQ;

    return NULL;
}

cpu_word dev_ackIRQ(device_handle device)
{
    struct _device *pDev = (struct _device *)device;

    if(!pDev)
        return 0;

    return pDev->irqACK(pDev->arg);
}
