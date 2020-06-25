#include <stdlib.h>

#include "task_scheduler.h"
#include "cpu.h"
#include "mem.h"
#include "rk11.h"

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
001020	000000	halt		(or 005007 to auto start)
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
    0000000,    // halt
};
ph_addr bootstrapBase = 0001000;

int main(void)
{
    ts_init();
    mem_init(bootstrapBase, (uint8_t *)bootstrap, sizeof(bootstrap));
    dev_init();

    rk11_init();
    if(!rk11_loadDisk("img/unix_v5_rk/unix_v5_rk.dsk", 0))
        return EXIT_FAILURE;
    dev_registerDevice(rk11_getHandle());

    cpu_init(bootstrapBase);

    for(;;)
    {
        cpu_run();

        const long int nSleepMax = 250 * TS_MILLISECONDS;

        long int nSleep = ts_run();
        if(nSleep == 0 || nSleep > nSleepMax)
            nSleep = nSleepMax;

        usleep(nSleepMax / TS_MICROSECONDS);
    }

    rk11_destroy();

    return EXIT_SUCCESS;
}
