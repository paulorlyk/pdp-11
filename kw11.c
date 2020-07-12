//
// Created by palulukan on 7/8/20.
//

#include "kw11.h"
#include "task_scheduler.h"

#include <string.h>
#include <assert.h>

#define KW11_ADDR 0777546

#define KW11_IRQ 0100

#define KW11_IRQ_PRIORITY 6

#define KW11_SR_IE      (1 << 6)    // Interrupt enable
#define KW11_SR_IM      (1 << 7)    // Interrupt monitor
#define KW11_SR_MASK    (KW11_SR_IE | KW11_SR_IM)

static struct
{
    device_handle device;
    ts_handle timerTask;
    cpu_word SR;
} kw11;

static void _timeout(void *arg)
{
    (void)arg;

    kw11.SR |= KW11_SR_IM;

    if(kw11.SR & KW11_SR_IE)
        dev_setIRQ(kw11.device);
}

static cpu_word _read(un_addr addr, void* arg)
{
    (void)arg;

    assert(addr == KW11_ADDR);
    assert((addr & 1) == 0);

    return kw11.SR;
}

static void _write(un_addr addr, cpu_word data, void* arg)
{
    (void)arg;

    assert(addr == KW11_ADDR);
    assert((addr & 1) == 0);

    kw11.SR = data & KW11_SR_MASK;

    if(!(kw11.SR & KW11_SR_IE))
        dev_clearIRQ(kw11.device);
}

static cpu_word _irqACK(void* arg)
{
    (void)arg;

    dev_clearIRQ(kw11.device);

    return KW11_IRQ;
}

static void _devReset(void* arg)
{
    (void)arg;

    kw11.SR = KW11_SR_IM;

    dev_clearIRQ(kw11.device);
}

bool kw11_init(void)
{
    memset(&kw11, 0, sizeof(kw11));

    dev_io_info ioMap[] = {
        { KW11_ADDR, KW11_ADDR, &_read, &_write, NULL },
        { 0 }
    };
    if(!(kw11.device = dev_initDevice("KW11", ioMap, KW11_IRQ_PRIORITY, &_irqACK, &_devReset, NULL)))
        return false;

    if(!(kw11.timerTask = ts_createTask(&_timeout, NULL)))
    {
        dev_destroyDevice(kw11.device);
        kw11.device = NULL;
        return false;
    }

#if CONFIG_KW11_HZ
    if(!ts_schedulePeriodic(kw11.timerTask, (1.0 / CONFIG_KW11_HZ) * TS_SECONDS))
    {
        dev_destroyDevice(kw11.device);
        kw11.device = NULL;

        ts_destroyTask(kw11.timerTask);
        kw11.timerTask = NULL;

        return false;
    }
#endif

    _devReset(NULL);

    return true;
}

void kw11_destroy(void)
{
    if(kw11.device)
    {
        dev_deregisterDevice(kw11.device);
        dev_destroyDevice(kw11.device);
        kw11.device = NULL;
    }

    if(kw11.timerTask)
    {
        ts_cancel(kw11.timerTask);
        ts_destroyTask(kw11.timerTask);
        kw11.timerTask = NULL;
    }
}

device_handle kw11_getHandle(void)
{
    return kw11.device;
}
