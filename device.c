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
    dev_io_info *ioMap;
    int irqPriority;
    irq_ack_cb irqACK;
    dev_reset_cb devReset;
    void *arg;
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

device_handle dev_initDevice(const dev_io_info *ioMap,
                             int irqPriority,
                             irq_ack_cb irqACK,
                             dev_reset_cb devReset,
                             void* arg)
{
    if(irqACK && (irqPriority < 0 || irqPriority > IRQ_PRIORITY_MAX))
        return NULL;

    int ioMapLen = 0;
    for(const dev_io_info *cio = ioMap; cio && cio->wr && cio->rd; ++cio, ++ioMapLen)
    {
        bool ok = cio->ioStart <= cio->ioEnd;
        ok |= !(cio->ioStart & 1);
        ok |= !(cio->ioEnd & 1);
        ok |= cio->ioStart >= MEM_UNIBUS_PERIPH_PAGE_ADDR;
        ok |= cio->ioEnd >= MEM_UNIBUS_PERIPH_PAGE_ADDR;
        ok |= cio->ioStart <= MEM_UNIBUS_ADDR_MAX;
        ok |= cio->ioEnd <= MEM_UNIBUS_ADDR_MAX;
        ok |= cio->rd && cio->wr;

        if(!ok)
            return NULL;
    }

    struct _device *pDev = calloc(1, sizeof(struct _device));
    if(!pDev)
        return NULL;

    if(ioMap)
    {
        pDev->ioMap = calloc(ioMapLen + 1, sizeof(dev_io_info));
        if(!pDev->ioMap)
        {
            free(pDev);
            return NULL;
        }

        // Because pDev->ioMap is already zeroed out, there is no need
        // to copy last element of the input array
        memcpy(pDev->ioMap, ioMap, sizeof(dev_io_info) * ioMapLen);
    }

    pDev->irqPriority = irqPriority;
    pDev->irqACK = irqACK;
    pDev->devReset = devReset;
    pDev->arg = arg;

    return pDev;
}

void dev_destroyDevice(device_handle device)
{
    if(!device)
        return;

    dev_deregisterDevice(device);

    struct _device *pDev = (struct _device *)device;

    if(pDev->ioMap)
        free(pDev->ioMap);

    free(pDev);
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

    if(pDev->irqACK && pDev->bIRQ && (!devices.curDevIRQ || devices.curDevIRQ->irqPriority <= pDev->irqPriority))
        _updateIRQ();

    for(const dev_io_info *cio = pDev->ioMap; cio && cio->wr && cio->rd; ++cio)
        mem_registerUnibusIO(cio->ioStart, cio->ioEnd, cio->rd, cio->wr, cio->arg);
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

    _updateIRQ();

    for(const dev_io_info *cio = pDev->ioMap; cio && cio->wr && cio->rd; ++cio)
        mem_deregisterUnibusIO(cio->ioStart, cio->ioEnd);
}

void dev_setIRQ(device_handle device)
{
    struct _device *pDev = (struct _device *)device;

    if(!pDev || !pDev->irqACK)
        return;

    pDev->bIRQ = true;

    if(pDev->registered && (!devices.curDevIRQ || devices.curDevIRQ->irqPriority <= pDev->irqPriority))
        _updateIRQ();
}

void dev_clearIRQ(device_handle device)
{
    struct _device *pDev = (struct _device *)device;

    if(!pDev || !pDev->irqACK)
        return;

    pDev->bIRQ = false;

    if(devices.curDevIRQ == pDev)
        _updateIRQ();
}

void dev_reset(void)
{
    for(struct _device *it = devices.head; it; it = it->next)
    {
        if(it->devReset)
            it->devReset(it->arg);
    }
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

    if(!pDev || !pDev->irqACK)
        return 0;

    return pDev->irqACK(pDev->arg);
}
