//
// Created by palulukan on 6/26/20.
//

#include "dl11.h"

#include "log.h"
#include "task_scheduler.h"

#include <string.h>
#include <assert.h>
#include <stdlib.h>

#define DL11_IRQ_PRIORITY 4

#define DL11_XMIT_TIME_MS 1

// RCSR - Receiver Status Register
// Bit  Designation
// 0    RDR ENB (Reader Enable)
// 1    DTR (Data Terminal Ready)
// 2    REQ TO SEND (Request to Send)
// 3    SEC XMIT (Secondary Transmit or Supervisory Transmitted Data)
// 4    Unused
// 5    DATASET INT ENB (Dataset Interrupt Enable)
// 6    RCVR INT ENB (Receiver Interrupt Enable)
// 7    RCVR DONE (Receiver Done)
// 8    Unused
// 9    Unused
// 10   SEC REC (Secondary Receive or Supervisory Received Data)
// 11   RCVR_ACT (Receiver Active)
// 12   CAR DET (Carrier Detect)
// 13   CLR TO SEND (Clear to Send)
// 14   RING
// 14   DATASET INT (Dataset Interrupt)
#define DL11_RCSR                   0
#define DL11_RCSR_RDR_ENB           (1 << 0)
#define DL11_RCSR_DTR               (1 << 1)
#define DL11_RCSR_REQ_TO_SEND       (1 << 2)
#define DL11_RCSR_SEC_XMIT          (1 << 3)
#define DL11_RCSR_DATASET_INT_ENB   (1 << 5)
#define DL11_RCSR_RCVR_INT_ENB      (1 << 6)
#define DL11_RCSR_RCVR_DONE         (1 << 7)
//#define DL11_RCSR_SEC_REC           (1 << 10)
#define DL11_RCSR_RCVR_ACT          (1 << 11)
//#define DL11_RCSR_CAR_DET           (1 << 12)
//#define DL11_RCSR_CLR_TO_SEND       (1 << 13)
//#define DL11_RCSR_RING              (1 << 14)
#define DL11_RCSR_DATASET_INT       (1 << 15)
#define DL11_RCSR_WR_MASK           (DL11_RCSR_DTR | DL11_RCSR_REQ_TO_SEND | DL11_RCSR_SEC_XMIT | DL11_RCSR_DATASET_INT_ENB)

// RBUF - Receiver Buffer Register
// Bit      Designation
// 0 - 7    RECEIVED DATA BITS
// 8 - 11   Unused
// 12        P ERR (Parity Error)
// 13        FR ERR (Framing Error)
// 14        OR ERR (Overrun Error)
// 15        ERROR
#define DL11_RBUF                       1
#define DL11_RBUF_SET_DATA(rbuf, data)  ((rbuf) = ((rbuf) & ~0xFF) | ((data) & 0xFF))
//#define DL11_RBUF_P_ERR                 (1 << 12)
//#define DL11_RBUF_FR_ERR                (1 << 13)
#define DL11_RBUF_OR_ERR                (1 << 14)
#define DL11_RBUF_ERROR                 (1 << 15)

// XCSR - Transmitter Status Register
// Bit  Designation
// 0    BREAK
// 1    Unused
// 2    MAINT (Maintenance)
// 3    Unused
// 4    Unused
// 5    Unused
// 6    XMIT INT ENB (Transmitter Interrupt Enable)
// 7    XMIT RDY (Transmitter Ready)
#define DL11_XCSR               2
#define DL11_XCSR_BREAK         (1 << 0)
#define DL11_XCSR_MAINT         (1 << 2)
#define DL11_XCSR_XMIT_INT_ENB  (1 << 6)
#define DL11_XCSR_XMIT_RDY      (1 << 7)
#define DL11_XCSR_WR_MASK       (DL11_XCSR_BREAK | DL11_XCSR_MAINT | DL11_XCSR_XMIT_INT_ENB)

// XBUF - Transmitter Buffer Register
// Bit      Designation
// 0 - 7    TRANSMITTER DATA BUFFER
// 8 - 15   Unused
#define DL11_XBUF                       3
#define DL11_XBUF_SET_DATA(xbuf, data)  ((xbuf) = ((xbuf) & ~0xFF) | ((data) & 0xFF))
#define DL11_XBUF_GET_DATA(xbuf)        ((xbuf) & 0xFF)

struct _dl11
{
    un_addr baseAddr;
    cpu_word baseVector;
    dl11_tx_cb txCb;
    device_handle device;
    cpu_word regs[4];
    ts_handle txTask;
    ts_handle rxTask;
    char chRx;  // Equivalent to receiver shift register
    bool bTxPending;
    char chTx;  // Equivalent to transmitter shift register
};

static void _updateInterrupts(struct _dl11 *pDev)
{
    if(    ((pDev->regs[DL11_RCSR] & (DL11_RCSR_DATASET_INT_ENB | DL11_RCSR_DATASET_INT)) == (DL11_RCSR_DATASET_INT_ENB | DL11_RCSR_DATASET_INT))
        || ((pDev->regs[DL11_RCSR] & (DL11_RCSR_RCVR_INT_ENB | DL11_RCSR_RCVR_DONE)) == (DL11_RCSR_RCVR_INT_ENB | DL11_RCSR_RCVR_DONE))
        || ((pDev->regs[DL11_XCSR] & (DL11_XCSR_XMIT_INT_ENB | DL11_XCSR_XMIT_RDY)) == (DL11_XCSR_XMIT_INT_ENB | DL11_XCSR_XMIT_RDY)))
    {
        dev_setIRQ(pDev->device);
    }
    else
        dev_clearIRQ(pDev->device);

}

static void _runReceiver(struct _dl11 *pDev, char ch)
{
    pDev->chRx = ch;
    pDev->regs[DL11_RCSR] |= DL11_RCSR_RCVR_ACT;
    ts_schedule(pDev->rxTask, DL11_XMIT_TIME_MS * TS_MILLISECONDS);
}

static void _runTransmitter(struct _dl11 *pDev)
{
    if(!(pDev->regs[DL11_XCSR] & DL11_XCSR_XMIT_RDY))
    {
        pDev->bTxPending = true;
        return;
    }

    pDev->bTxPending = false;
    pDev->regs[DL11_XCSR] &= ~DL11_XCSR_XMIT_RDY;

    pDev->chTx = DL11_XBUF_GET_DATA(pDev->regs[DL11_XBUF]);
    ts_schedule(pDev->txTask, DL11_XMIT_TIME_MS * TS_MILLISECONDS);

    if(pDev->regs[DL11_XCSR] & DL11_XCSR_MAINT)
        _runReceiver(pDev, pDev->chTx);

    _updateInterrupts(pDev);
}

static cpu_word _read(un_addr addr, void* arg)
{
    assert(arg);
    struct _dl11 *pDev = (struct _dl11 *)arg;

    assert(addr >= pDev->baseAddr);
    assert(addr < pDev->baseAddr + 4 * MEM_WORD_SIZE);
    assert((addr & 1) == 0);

    int reg = (addr - pDev->baseAddr) >> 1;

    cpu_word res = pDev->regs[reg];

    if(reg == DL11_RCSR)
    {
        // DL11_RCSR_DATASET_INT is read-onee bit so clear it
        // when register is read.
        pDev->regs[DL11_RCSR] &= ~DL11_RCSR_DATASET_INT;
        _updateInterrupts(pDev);
    }
    else if(reg == DL11_RBUF)
    {
        // DL11_RCSR_RCVR_DONE should be cleared when RBUF
        // register is addressed.
        pDev->regs[DL11_RCSR] &= ~DL11_RCSR_RCVR_DONE;
        _updateInterrupts(pDev);
    }

    return res;
}

static void _write(un_addr addr, cpu_word data, void* arg)
{
    assert(arg);
    struct _dl11 *pDev = (struct _dl11 *)arg;

    assert(addr >= pDev->baseAddr);
    assert(addr < pDev->baseAddr + 4 * MEM_WORD_SIZE);
    assert((addr & 1) == 0);

    int reg = (addr - pDev->baseAddr) >> 1;

    switch(reg)
    {
        case DL11_RCSR:
            //DEBUG("DL11: Writing RCSR: 0%06o", data);

            assert(!(data & DL11_RCSR_DATASET_INT_ENB));
            assert(!(data & DL11_RCSR_RCVR_INT_ENB));

            pDev->regs[DL11_RCSR] = data & DL11_RCSR_WR_MASK;

            if(data & DL11_RCSR_RDR_ENB)
                pDev->regs[DL11_RCSR] &= ~DL11_RCSR_RCVR_DONE;

            _updateInterrupts(pDev);
            break;

        case DL11_RBUF:
            //DEBUG("DL11: Writing RBUF: 0%06o", data);
            // DL11_RCSR_RCVR_DONE should be cleared when RBUF
            // register is addressed.
            // No actual write is performed since RBUF is read-only
            pDev->regs[DL11_RCSR] &= ~DL11_RCSR_RCVR_DONE;
            _updateInterrupts(pDev);
            break;

        case DL11_XCSR:
            //DEBUG("DL11: Writing XCSR: 0%06o", data);

            assert(!(data & DL11_XCSR_XMIT_INT_ENB));

            pDev->regs[DL11_XCSR] = data & DL11_XCSR_WR_MASK;

            if(data & DL11_XCSR_MAINT)
                ts_cancel(pDev->rxTask);

            _updateInterrupts(pDev);
            break;

        case DL11_XBUF:
            //DEBUG("DL11: Writing XBUF: 0%06o", data);
            DL11_XBUF_SET_DATA(pDev->regs[DL11_XBUF], data);
            _runTransmitter(pDev);
            break;
    }
}

static cpu_word _irqACK(void* arg)
{
    assert(arg);
    struct _dl11 *pDev = (struct _dl11 *)arg;

    // TODO: Implement
    assert(false);

    // pDev->baseVector - RX,  higher priority than TX
    // pDev->baseVector + 4 - TX

    return pDev->baseVector;
}

static void _devReset(void* arg)
{
    assert(arg);
    struct _dl11 *pDev = (struct _dl11 *)arg;

    memset(pDev->regs, 0, sizeof(pDev->regs));

    pDev->regs[DL11_XCSR] = DL11_XCSR_XMIT_RDY;

    ts_cancel(pDev->txTask);
    ts_cancel(pDev->rxTask);

    pDev->bTxPending = false;

    _updateInterrupts(pDev);
}

static void _txDone(void* arg)
{
    assert(arg);
    struct _dl11 *pDev = (struct _dl11 *)arg;

    if(!(pDev->regs[DL11_XCSR] & DL11_XCSR_MAINT))
        pDev->txCb(pDev->chTx);

    pDev->regs[DL11_XCSR] |= DL11_XCSR_XMIT_RDY;
    if(pDev->bTxPending)
        _runTransmitter(pDev);
    else
        _updateInterrupts(pDev);
}

static void _rxDone(void* arg)
{
    assert(arg);
    struct _dl11 *pDev = (struct _dl11 *)arg;

    // When new character received all error bits are cleared.
    // We might as well clear data bits too since they will
    // be overwritten shortly.
    pDev->regs[DL11_RBUF] = 0;

    // Set Overflow error flag if previous character was not processed yet.
    // Also make sure to set DL11_RBUF_ERROR because it represents  logical "or"
    // of all other error bits in RBUF register.
    if(pDev->regs[DL11_RCSR] & DL11_RCSR_RCVR_DONE)
        pDev->regs[DL11_RBUF] |= DL11_RBUF_OR_ERR | DL11_RBUF_ERROR;

    DL11_RBUF_SET_DATA(pDev->regs[DL11_RBUF], pDev->chRx);

    pDev->regs[DL11_RCSR] &= ~DL11_RCSR_RCVR_ACT;
    pDev->regs[DL11_RCSR] |= DL11_RCSR_RCVR_DONE;

    _updateInterrupts(pDev);
}

DL11 dl11_init(un_addr baseAddr, cpu_word baseVector, dl11_tx_cb tx)
{
    if((baseAddr & 1) || (baseVector & 7) || !tx)
        return NULL;

    struct _dl11 *pDev = calloc(1, sizeof(struct _dl11));

    pDev->baseAddr = baseAddr;
    pDev->baseVector = baseVector;
    pDev->txCb = tx;

    dev_io_info ioMap[] = {
        { baseAddr, baseAddr + 6, &_read, &_write, pDev },
        { 0 }
    };
    if(!(pDev->device = dev_initDevice("DL11", ioMap, DL11_IRQ_PRIORITY, &_irqACK, &_devReset, pDev)))
    {
        free(pDev);
        return NULL;
    }

    if(!(pDev->txTask = ts_createTask(_txDone, pDev)))
    {
        dev_destroyDevice(pDev->device);
        free(pDev);
        return NULL;
    }

    if(!(pDev->rxTask = ts_createTask(_rxDone, pDev)))
    {
        dev_destroyDevice(pDev->device);
        ts_destroyTask(pDev->txTask);
        free(pDev);
        return NULL;
    }

    _devReset(pDev);

    return pDev;
}

void dl11_destroy(DL11 dev)
{
    struct _dl11 *pDev = (struct _dl11 *)dev;
    if(!pDev)
        return;

    dev_deregisterDevice(pDev->device);
    dev_destroyDevice(pDev->device);
    pDev->device = NULL;

    ts_cancel(pDev->txTask);
    ts_destroyTask(pDev->txTask);
    pDev->txTask = NULL;

    ts_cancel(pDev->rxTask);
    ts_destroyTask(pDev->rxTask);
    pDev->rxTask = NULL;

    free(pDev);
}

device_handle dl11_getHandle(DL11 dev)
{
    struct _dl11 *pDev = (struct _dl11 *)dev;
    if(!pDev)
        return NULL;

    return pDev->device;
}

bool dl11_rx(DL11 dev, char ch)
{
    struct _dl11 *pDev = (struct _dl11 *)dev;
    if(!pDev)
        return false;

    // Is receiver busy processing previous character?
    if(ts_isScheduled(pDev->rxTask))
        return false;

    // Ignore receiver input while in maintenance mode
    if(pDev->regs[DL11_XCSR] & DL11_XCSR_MAINT)
        return true;

    _runReceiver(pDev, ch);
    return true;
}
