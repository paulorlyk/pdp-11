//
// Created by palulukan on 6/14/20.
//

#include "rk11.h"

#include "log.h"
#include "task_scheduler.h"

#include <stdlib.h>
#include <assert.h>
#include <string.h>
#include <stdio.h>

#define RK11_REG_RKDS   0777400
//#define RK11_REG_RKER   0777402
#define RK11_REG_RKCS   0777404
#define RK11_REG_RKWC   0777406
#define RK11_REG_RKBA   0777410
#define RK11_REG_RKDA   0777412
//#define RK11_REG_RKMR   0777414
#define RK11_REG_RKDB   0777416

#define RK11_PERIPH_START   RK11_REG_RKDS
#define RK11_PERIPH_END     RK11_REG_RKDB

#define RK11_IRQ            0220
#define RK11_IRQ_PRIORITY   5

#define RK05_SECTORS    12

// RKDS - Device Status Register
// Bit  Designation
// 0    Sector counter [0]
// 1    Sector counter [1]
// 2    Sector counter [2]
// 3    Sector counter [3]
// 4    SC=SA
// 5    Write Protect Status (WPS)
// 6    Read/Write/Seek Ready (R/W/S ROY)
// 7    Drive Ready (DRY)
// 8    Sector Counter OK(SOK)
// 9    Seek Incomplete (SIN)
// 10   Drive Unsafe (DRU)
// 11   RK05 Disk on Line (RK05)
// 12   Drive Power Low (DPL)
// 13   Identification of Drive (ID) [0]
// 14   Identification of Drive (ID) [1]
// 15   Identification of Drive (ID) [2]
#define RK11_RKDS                   0
#define RK11_RKDS_GET_SECTOR(rkds)  ((rkds) & 0x000F)
#define RK11_RKDS_SCSA              (1 << 4)
#define RK11_RKDS_WPS               (1 << 5)
#define RK11_RKDS_ROY               (1 << 6)
#define RK11_RKDS_DRY               (1 << 7)
#define RK11_RKDS_SOK               (1 << 8)
#define RK11_RKDS_RK05              (1 << 11)
#define RK11_RKDS_DRIVE_MASK        0xE000
#define RK11_RKDS_SET_DRIVE(drv)   ((drv) & 7) << 13)

// RKER - Error Register
// Bit  Designation
// 0    Write Check Error (WCE)
// 1    Checksum Error (CSE)
// 2    Unused
// 3    Unused
// 4    Unused
// 5    Nonexistent Sector (NXS)
// 6    Nonexistent Cylinder (NXC)
// 7    Nonexistent Disk (NXD)
// 8    Timing Error (TE)
// 9    Data Late (DLT)
// 10   Nonexistent Memory (NXM)
// 11   Programming Error (PGE)
// 12   Seek Error (SKE)
// 13   Write Lockout Violation (WLO)
// 14   Overrun (OVR)
// 15   Drive Error (DRE)
#define RK11_RKER   1
#define RK11_RKER_NXS (1 << 5)
#define RK11_RKER_NXC (1 << 6)
#define RK11_RKER_NXD (1 << 7)
#define RK11_RKER_NXM (1 << 10)
#define RK11_RKER_PGE (1 << 11)
#define RK11_RKER_WLO (1 << 13)
#define RK11_RKER_OVR (1 << 14)
#define RK11_RKER_DRE (1 << 15)
#define RK11_RKER_HARD_MASK 0xFFE0

// RKCS - Control Sstatus Register
// Bit  Designation
// 0    GO (Write Only)
// 1    Function (Read/Write) [0]
// 2    Function (Read/Write) [1]
// 3    Function (Read/Write) [2]
// 4    Memory Extension (MEX) (Read/Write) [0]
// 5    Memory Extension (MEX) (Read/Write) [1]
// 6    Interrupt on Done Enable (IDE) (Read/Write)
// 7    Control Ready (RDY) (Read Only)
// 8    Stop on Soft Error (SSE) (Read/Write)
// 9    Extra Bit (EXB)
// 10   Format (FMT) (Read/Write)
// 11   Inhibit Incrementing the RKBA (IBA) (Read/Write)
// 12   Unused
// 13   Search Complete (SCP) (Read Only)
// 14   Hard Error (HE)
// 15   Error (ERR) (Read Only)
#define RK11_RKCS   2
#define RK11_RKCS_GO  (1 << 0)
#define RK11_RKCS_IDE (1 << 6)
#define RK11_RKCS_RDY (1 << 7)
#define RK11_RKCS_EXB (1 << 9)
#define RK11_RKCS_FMT (1 << 10)
#define RK11_RKCS_IBA (1 << 11)
#define RK11_RKCS_UN  (1 << 12)
#define RK11_RKCS_SCP (1 << 13)
#define RK11_RKCS_HE  (1 << 14)
#define RK11_RKCS_ERR (1 << 15)

#define RK11_RKCS_GET_MEX(rkcs)         (((rkcs) >> 4) & 3)
#define RK11_RKCS_SET_MEX(rkcs, mex)    ((rkcs) = ((rkcs) & ~(3 << 4)) | (((mex) & 3) << 4))

#define RK11_RKCS_GET_FUNC(rkcs)        (((rkcs) >> 1) & 7)
#define RK11_RKCS_FUNC_CONTROL_RESET    0
#define RK11_RKCS_FUNC_WRITE            1
#define RK11_RKCS_FUNC_READ             2
#define RK11_RKCS_FUNC_WRITE_CHECK      3
#define RK11_RKCS_FUNC_SEEK             4
#define RK11_RKCS_FUNC_READ_CHECK       5
#define RK11_RKCS_FUNC_DRIVE_RESET      6
#define RK11_RKCS_FUNC_WRITE_LOCK       7
#define RK11_RKCS_FUNC_IDLE             -1              // For internal purposes only
#define RK11_RKCS_FUNC_IS_IDLE(func)    ((func) < 0)    // For internal purposes only

#define RK11_RKWC   3

#define RK11_RKBA   4

#define RK11_RKDA   5
#define RK11_RKDA_GET_SECTOR(rkda)          ((rkda) & 0x000F)
#define RK11_RKDA_SET_SECTOR(rkda, sec)     ((rkda) = ((rkda) & ~0x000F) | ((sec) & 0x000F))
#define RK11_RKDA_SECTORS                   12
#define RK11_RKDA_GET_SURFACE(rkda)         (((rkda) >> 4) & 0x0001)
#define RK11_RKDA_SET_SURFACE(rkda, surf)   ((rkda) = ((rkda) & ~(1 << 4)) | (((surf) & 1) << 4))
#define RK11_RKDA_SURFACES                  2
#define RK11_RKDA_GET_CYLINDER(rkda)        (((rkda) >> 5) & 0x00FF)
#define RK11_RKDA_SET_CYLINDER(rkda, cyl)   ((rkda) = ((rkda) & ~(0x00FF << 5)) | (((cyl) & 0x00FF) << 5))
#define RK11_RKDA_CYLINDERS                 0313
#define RK11_RKDA_GET_DRIVE(rkda)           (((rkda) >> 13) & 7)

//#define RK11_RKMR   6
//#define RK11_RKDB   7

#define RK11_CYL_SEEK_TIME_US   250

#define RK11_SECTOR_SIZE_WORDS    256

static struct
{
    struct _rk05
    {
        uint8_t *img;
        bool bConnected;
        bool bINT;
        bool writeProtect;
        ts_handle task;
        int nSector;
        int nSurface;
        int nCylinder;
        int func;
    } disks[RK05_DISKS_MAX];
    cpu_word regs[8];
    device_handle device;
    bool bINT;
    int currentDrive;
} rk11;

static void _updateInterrupts(void)
{
    if(!(rk11.regs[RK11_RKCS] & RK11_RKCS_IDE))
    {
        rk11.bINT = false;

        for(int i = 0; i < RK05_DISKS_MAX; ++i)
            rk11.disks[i].bINT = false;

        dev_clearIRQ(rk11.device);

        return;
    }

    if(!(rk11.regs[RK11_RKCS] & RK11_RKCS_RDY))
        return;

    if(rk11.bINT)
    {
        dev_setIRQ(rk11.device);
        return;
    }

    for(int i = 0; i < RK05_DISKS_MAX; ++i)
    {
        if(rk11.disks[i].bINT)
        {
            dev_setIRQ(rk11.device);
            return;
        }
    }
    dev_clearIRQ(rk11.device);
}

static void _controlReset(void)
{
    for(int i = 0; i < RK05_DISKS_MAX; ++i)
    {
        ts_cancel(rk11.disks[i].task);

        rk11.disks[i].bINT = false;
        rk11.disks[i].func = RK11_RKCS_FUNC_IDLE;
    }

    memset(rk11.regs, 0, sizeof(rk11.regs));
    rk11.regs[RK11_RKCS] = RK11_RKCS_RDY;

    _updateInterrupts();
}

static void _finishFunction(void)
{
    rk11.regs[RK11_RKCS] |= RK11_RKCS_RDY;
    rk11.bINT = true;

    rk11.regs[RK11_RKCS] &= ~(RK11_RKCS_GO | RK11_RKCS_EXB | RK11_RKCS_UN | RK11_RKCS_HE | RK11_RKCS_ERR);
    rk11.regs[RK11_RKCS] |=
          ((rk11.regs[RK11_RKER] & RK11_RKER_HARD_MASK) ? RK11_RKCS_HE : 0)
        | ((rk11.regs[RK11_RKER] & ~RK11_RKER_HARD_MASK) ? RK11_RKCS_ERR : 0);

    _updateInterrupts();
}

static void _runFunction(void)
{
    // Must be cleared at the initiation of any new function
    rk11.regs[RK11_RKCS] &= ~(RK11_RKCS_SCP | RK11_RKCS_RDY);
    rk11.regs[RK11_RKER] &= RK11_RKER_HARD_MASK;    // Clear all soft errors

    //rk11.bINT = false;
    //_updateInterrupts();

    rk11.currentDrive = RK11_RKDA_GET_DRIVE(rk11.regs[RK11_RKDA]);

    int func = RK11_RKCS_GET_FUNC(rk11.regs[RK11_RKCS]);

    // Control Reset should always work
    if(func == RK11_RKCS_FUNC_CONTROL_RESET)
    {
        _controlReset();
        _finishFunction();
        return;
    }

    // TODO: Implement format
    if(rk11.regs[RK11_RKCS] & RK11_RKCS_FMT)
    {
        DEBUG("RK11: Format bit is set in RKCS");
        assert(false);
    }

    struct _rk05 *disk = rk11.disks + rk11.currentDrive;

    if(!disk->bConnected)
    {
        rk11.regs[RK11_RKER] |= RK11_RKER_NXD;
        _finishFunction();
        return;
    }

    if(!disk->img || disk->func >= 0)
    {
        rk11.regs[RK11_RKER] |= RK11_RKER_DRE;
        _finishFunction();
        return;
    }

    if(func == RK11_RKCS_FUNC_WRITE_LOCK)
    {
        disk->writeProtect = true;
        _finishFunction();
        return;
    }

    if((rk11.regs[RK11_RKCS] & RK11_RKCS_FMT) && func != RK11_RKCS_FUNC_READ && func != RK11_RKCS_FUNC_WRITE)
    {
        rk11.regs[RK11_RKER] |= RK11_RKER_PGE;
        _finishFunction();
        return;
    }

    int nSector = 0;
    int nSurface = 0;
    int nCylinder = 0;
    if(func != RK11_RKCS_FUNC_DRIVE_RESET)
    {
        nSector = RK11_RKDA_GET_SECTOR(rk11.regs[RK11_RKDA]);
        if(nSector >= RK11_RKDA_SECTORS)
        {
            rk11.regs[RK11_RKER] |= RK11_RKER_NXS;
            _finishFunction();
            return;
        }

        nSurface = RK11_RKDA_GET_SURFACE(rk11.regs[RK11_RKDA]);

        nCylinder = RK11_RKDA_GET_CYLINDER(rk11.regs[RK11_RKDA]);
        if(nCylinder >= RK11_RKDA_CYLINDERS)
        {
            rk11.regs[RK11_RKER] |= RK11_RKER_NXC;
            _finishFunction();
            return;
        }
    }
    else
    {
        disk->writeProtect = false;
        func = RK11_RKCS_FUNC_SEEK;
    }

    if(func == RK11_RKCS_FUNC_WRITE && disk->writeProtect)
    {
        rk11.regs[RK11_RKER] |= RK11_RKER_WLO;
        _finishFunction();
        return;
    }

#if !CONFIG_RK11_NO_DELAYS
    unsigned long int ulOpTime = RK11_CYL_SEEK_TIME_US * (abs(disk->nCylinder - nCylinder) + 1);
#else
    unsigned long int ulOpTime = 0;
#endif

    disk->func = func;
    disk->nSector = nSector;
    disk->nSurface = nSurface;
    disk->nCylinder = nCylinder;

    ts_schedule(disk->task, ulOpTime * TS_MICROSECONDS);

    if(func == RK11_RKCS_FUNC_SEEK)
        _finishFunction();
}

static void _diskTaskCb(void *arg)
{
    assert(arg);

    struct _rk05 *disk = (struct _rk05 *)arg;

    int func = disk->func;

    disk->func = RK11_RKCS_FUNC_IDLE;

    if(func == RK11_RKCS_FUNC_SEEK)
    {
        rk11.regs[RK11_RKCS] |= RK11_RKCS_SCP;
        disk->bINT = true;
        _updateInterrupts();
        return;
    }

    // Is the drive still connected and loaded?
    if(!disk->bConnected || !disk->img)
    {
        rk11.regs[RK11_RKER] |= RK11_RKER_DRE;
        _finishFunction();
        return;
    }

    size_t nDiskWord = ((disk->nCylinder * RK11_RKDA_SURFACES + disk->nSurface) * RK11_RKDA_SECTORS + disk->nSector) * RK11_SECTOR_SIZE_WORDS;

    // Use direct value of RKWC here because apparently it is allowed
    // to change words count of the function while it is running
    size_t nWordsCount = (size_t)(0x10000 - rk11.regs[RK11_RKWC]);

    if((nDiskWord + nWordsCount) > RK05_SIZE_WORDS)
    {
        nWordsCount = RK05_SIZE_WORDS - nDiskWord;
        rk11.regs[RK11_RKER] |= RK11_RKER_OVR;
    }

    un_addr addr = rk11.regs[RK11_RKBA] | (RK11_RKCS_GET_MEX(rk11.regs[RK11_RKCS]) << 16);
    cpu_word *data = ((cpu_word *)disk->img) + nDiskWord;
    size_t nWordsDone = 0;

    switch(func)
    {
        case RK11_RKCS_FUNC_WRITE:
        {
            // TODO: Not implemented
            assert(false);
            break;
        }

        case RK11_RKCS_FUNC_READ:
        {
            // TODO: Implement format
            if(rk11.regs[RK11_RKCS] & RK11_RKCS_FMT)
            {
                DEBUG("RK11: Format bit is set in RKCS");
                assert(false);
            }

            DEBUG("RK11: Reading %lu words from disk %d, cyl %d surf %d sect %d [img 0x%lX] to memory location 0%06o",
                  nWordsCount, rk11.currentDrive, disk->nCylinder, disk->nSurface, disk->nSector, nDiskWord * 2, addr);

            for(nWordsDone = 0; nWordsDone < nWordsCount; ++nWordsDone)
            {
                if(mem_writeUnibus(addr, false, *data++) & MEM_HAS_ERR)
                {
                    rk11.regs[RK11_RKER] |= RK11_RKER_NXM;
                    break;
                }

                if(!(rk11.regs[RK11_RKCS] & RK11_RKCS_IBA))
                    addr += MEM_WORD_SIZE;
            }

            break;
        }

        case RK11_RKCS_FUNC_WRITE_CHECK:
        {
            // TODO: Not implemented
            assert(false);
            break;
        }

        case RK11_RKCS_FUNC_READ_CHECK:
        {
            // TODO: Not implemented
            assert(false);
            break;
        }
    }

    rk11.regs[RK11_RKWC] += nWordsDone;

    rk11.regs[RK11_RKBA] = addr & 0xFFFF;
    RK11_RKCS_SET_MEX(rk11.regs[RK11_RKCS], addr >> 16);

    nDiskWord = (nDiskWord + nWordsDone) / RK11_SECTOR_SIZE_WORDS;

    disk->nSector = nDiskWord % RK11_RKDA_SECTORS;
    disk->nSurface = (nDiskWord / RK11_RKDA_SECTORS) & 1;
    disk->nCylinder = (nDiskWord / (RK11_RKDA_SECTORS << 1));

    RK11_RKDA_SET_SECTOR(rk11.regs[RK11_RKDA], disk->nSector);
    RK11_RKDA_SET_SURFACE(rk11.regs[RK11_RKDA], disk->nSurface);
    RK11_RKDA_SET_CYLINDER(rk11.regs[RK11_RKDA], disk->nCylinder);

    _finishFunction();
}

static void _unloadDisk(int n)
{
    ts_cancel(rk11.disks[n].task);

    if(rk11.disks[n].img)
    {
        free(rk11.disks[n].img);
        rk11.disks[n].img = NULL;
    }

    rk11.disks[n].bINT = false;
    rk11.disks[n].writeProtect = false;
    rk11.disks[n].func = RK11_RKCS_FUNC_IDLE;
    rk11.disks[n].nSector = 0;
    rk11.disks[n].nSurface = 0;
    rk11.disks[n].nCylinder = 0;

    _updateInterrupts();
}

static cpu_word _read(un_addr addr, void* arg)
{
    assert(addr >= RK11_PERIPH_START);
    assert(addr < RK11_PERIPH_END);
    assert((addr & 1) == 0);

    (void)arg;

    int reg = (addr - RK11_PERIPH_START) >> 1;

    if(reg == RK11_RKDS)
    {
        struct _rk05 *disk = rk11.disks + RK11_RKDA_GET_DRIVE(rk11.regs[RK11_RKDS]);

        rk11.regs[RK11_RKDS] &= RK11_RKDS_DRIVE_MASK;
        if(disk->bConnected)
        {
            // "Rotate" disk to the random sector if it is loaded
            if(disk->img)
                rk11.regs[RK11_RKDS] |= rand() % RK05_SECTORS;

            rk11.regs[RK11_RKDS] |=
                  RK11_RKDS_RK05
                | RK11_RKDS_SOK
                | (disk->img ? RK11_RKDS_DRY : 0)
                | (RK11_RKCS_FUNC_IS_IDLE(disk->func) ? RK11_RKDS_ROY : 0)
                | (disk->writeProtect ? RK11_RKDS_WPS : 0)
                | (RK11_RKDS_GET_SECTOR(rk11.regs[RK11_RKDS]) == RK11_RKDA_GET_SECTOR(rk11.regs[RK11_RKDA]) ? RK11_RKDS_SCSA : 0);
        }
    }
//    else if(reg == RK11_RKCS)
//    {
//        rk11.regs[RK11_RKCS] &= ~(RK11_RKCS_GO | RK11_RKCS_EXB | RK11_RKCS_UN | RK11_RKCS_HE | RK11_RKCS_ERR);
//        rk11.regs[RK11_RKCS] |=
//              ((rk11.regs[RK11_RKER] & RK11_RKER_HARD_MASK) ? RK11_RKCS_HE : 0)
//            | ((rk11.regs[RK11_RKER] & ~RK11_RKER_HARD_MASK) ? RK11_RKCS_ERR : 0);
//    }

    return rk11.regs[reg];
}

static void _write(un_addr addr, cpu_word data, void* arg)
{
    assert(addr >= RK11_PERIPH_START);
    assert(addr < RK11_PERIPH_END);
    assert((addr & 1) == 0);

    (void)arg;

    switch(addr)
    {
        case RK11_REG_RKCS:
        {
            DEBUG("RK11: Writing RKCS: 0%06o", data);

            bool bIDE = (rk11.regs[RK11_RKCS] & RK11_RKCS_IDE) != 0;

            // Preserve read-only bits
            const cpu_word roMask =
                  RK11_RKCS_GO
                | RK11_RKCS_RDY
                | RK11_RKCS_EXB
                | RK11_RKCS_SCP
                | RK11_RKCS_UN
                | RK11_RKCS_HE
                | RK11_RKCS_ERR;
            rk11.regs[RK11_RKCS] &= roMask;
            rk11.regs[RK11_RKCS] |= data & ~roMask;

            if(bIDE != ((data & RK11_RKCS_IDE) != 0))
            {
                // TODO: Clear all pending interrupt flags or just disable interrupt delivery?
                _updateInterrupts();
            }

            if((rk11.regs[RK11_RKCS] & RK11_RKCS_RDY) && (data & RK11_RKCS_GO))
                _runFunction();

            break;
        }

        case RK11_REG_RKWC:
            DEBUG("RK11: Writing RKWC: 0%06o", data);
            rk11.regs[RK11_RKWC] = data;
            break;

        case RK11_REG_RKBA:
            DEBUG("RK11: Writing RKBA: 0%06o", data);
            rk11.regs[RK11_RKBA] = data & 0xFFFE;
            break;

        case RK11_REG_RKDA:
        {
            DEBUG("RK11: Writing RKDA: 0%06o", data);

            if(rk11.regs[RK11_RKCS] & RK11_RKCS_RDY)
                rk11.regs[RK11_RKDA] = data;

            break;
        }
    }
}

static cpu_word _irqACK(void* arg)
{
    (void)arg;

    // TODO: Implement
    assert(false);

    return RK11_IRQ;
}

static void _devReset(void* arg)
{
    (void)arg;

    for(int i = 0; i < RK05_DISKS_MAX; ++i)
        rk11.disks[i].writeProtect = false;

    _controlReset();
}

bool rk11_init(void)
{
    memset(&rk11, 0, sizeof(rk11));

    dev_io_info ioMap[] = {
        { RK11_PERIPH_START, RK11_PERIPH_END, &_read, &_write, NULL },
        { 0 }
    };
    if(!(rk11.device = dev_initDevice(ioMap, RK11_IRQ_PRIORITY, &_irqACK, &_devReset, NULL)))
        return false;

    for(int i = 0; i < RK05_DISKS_MAX; ++i)
    {
        if(!(rk11.disks[i].task = ts_createTask(&_diskTaskCb, rk11.disks + i)))
        {
            dev_destroyDevice(rk11.device);
            rk11.device = NULL;

            for(int j = 0; j < i; ++j)
            {
                ts_destroyTask(rk11.disks[j].task);
                rk11.disks[j].task = NULL;
            }

            return false;
        }
        rk11.disks[i].func = RK11_RKCS_FUNC_IDLE;
    }

    _devReset(NULL);

    return true;
}

void rk11_destroy(void)
{
    for(int i = 0; i < RK05_DISKS_MAX; ++i)
    {
        _unloadDisk(i);

        ts_destroyTask(rk11.disks[i].task);
        rk11.disks[i].task = NULL;
    }

    dev_deregisterDevice(rk11.device);
    dev_destroyDevice(rk11.device);
    rk11.device = NULL;
}

device_handle rk11_getHandle(void)
{
    return rk11.device;
}

bool rk11_connectDisk(int nDisk, bool connected)
{
    if(nDisk < 0 || nDisk >= RK05_DISKS_MAX)
    {
        DEBUG("RK11: Attempting to connect disk with invalid number %d", nDisk);
        return false;
    }

    if(!connected && rk11.disks[nDisk].bConnected)
    {
        _unloadDisk(nDisk);
        rk11.disks[nDisk].bConnected = false;

        DEBUG("RK11: Disk %d disconnected", nDisk);
    }

    if(connected && !rk11.disks[nDisk].bConnected)
    {
        rk11.disks[nDisk].bConnected = true;

        DEBUG("RK11: Disk %d connected", nDisk);
    }

    return true;
}

bool rk11_loadDisk(const char* szImagePath, int nDisk)
{
    if(!szImagePath)
    {
        DEBUG("RK11: Need disk image file name");
        return false;
    }

    if(nDisk < 0 || nDisk >= RK05_DISKS_MAX)
    {
        DEBUG("RK11: Attempting to load disk with invalid number %d", nDisk);
        return false;
    }

    _unloadDisk(nDisk);
    if(!rk11_connectDisk(nDisk, true))
        return false;

    FILE *pfImage = fopen(szImagePath, "rb");
    if(!pfImage)
    {
        DEBUG("RK11: Failed to open image file `%s`", szImagePath);
        return false;
    }

    rk11.disks[nDisk].img = calloc(1, RK05_SIZE);
    if(!rk11.disks[nDisk].img)
    {
        DEBUG("RK11: Out of memory");
        return false;
    }

    size_t nRead = 0;
    while(nRead < RK05_SIZE)
    {
        nRead += fread(rk11.disks[nDisk].img + nRead, 1, RK05_SIZE - nRead, pfImage);

        if(feof(pfImage))
            break;

        if(ferror(pfImage))
        {
            DEBUG("RK11: Error reading the disk image file `%s`", szImagePath);

            fclose(pfImage);
            free(rk11.disks[nDisk].img);
            rk11.disks[nDisk].img = NULL;
            return false;
        }
    }

    fclose(pfImage);

    DEBUG("RK11: Loaded %lu bytes from image file `%s` to disk %d", nRead, szImagePath, nDisk);

    return true;
}
