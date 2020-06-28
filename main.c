#include <stdlib.h>

#include "task_scheduler.h"
#include "cpu.h"
#include "mem.h"
#include "rk11.h"
#include "dl11.h"

#include <stdio.h>
#include <string.h>
#include <unistd.h>

/*
RK02/03/05 Disk Unit 0

Loc.	Cont.	Instruction	Comment
=======================================
001000	012700	mov #rkwc, r0	controller address
001002	177406
001004	012710	mov #-256,(r0)	set the word count
001006	177400
001010	012740	mov #5,-(r0)	read command
001012	000005
001014	105710	tstb (r0)	wait for ready
001016	100376  bpl .-2
001020	005007	clr pc		start loaded bootstrap with jump to 0
*/
uint16_t bootstrap[] = {
    0012700,    // mov #rkwc, r0
    0177406,
    0012710,    // mov #-256,(r0)
    0177400,
    0012740,    // mov #5,-(r0)
    0000005,
    0105710,    // tstb (r0)
    0100376,    // bpl .-2
    0005007     // clr pc
};
ph_addr bootstrapBase = 0001000;

void terminalRx(char ch)
{
    putchar(ch);
    fflush(stdout);
}

void kbdTaskCb(void *arg)
{
    DL11 dl11 = (DL11)arg;

    static const char *msg = "unix\n";
    static int pos = 0;

    if(pos >= strlen(msg))
        return;

    if(dl11_rx(dl11, msg[pos]))
        ++pos;
}

int main(void)
{
    ts_init();
    mem_init(bootstrapBase, (uint8_t *)bootstrap, sizeof(bootstrap));
    dev_init();

    DL11 dl11 = dl11_init(0777560, 060, &terminalRx);
    if(!dl11)
        return EXIT_FAILURE;
    dev_registerDevice(dl11_getHandle(dl11));

    rk11_init();
    if(!rk11_loadDisk("img/unix_v5_rk/unix_v5_rk.dsk", 0))
        return EXIT_FAILURE;
    dev_registerDevice(rk11_getHandle());

    cpu_init(bootstrapBase);

    ts_handle kbdTask = ts_createTask(&kbdTaskCb, dl11);

    for(int i = 0;;)
    {
        if(!ts_isScheduled(kbdTask) && (i++ > 100))
        {
            ts_schedule(kbdTask, 500 * TS_MILLISECONDS);
            i = 0;
        }

        cpu_run();

        const long int nSleepMax = 1 * TS_MICROSECONDS;

        long int nSleep = ts_run();
        if(nSleep == 0 || nSleep > nSleepMax)
            nSleep = nSleepMax;

        //usleep(nSleepMax / TS_MICROSECONDS);
    }

    dl11_destroy(dl11);
    rk11_destroy();

    ts_destroyTask(kbdTask);

    return EXIT_SUCCESS;
}
