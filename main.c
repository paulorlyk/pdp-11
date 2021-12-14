#include <stdlib.h>

#include "task_scheduler.h"
#include "cpu.h"
#include "mem.h"
#include "rk11.h"
#include "dl11.h"
#include "kw11.h"

#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <sys/select.h>

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
cpu_addr bootstrapBase = 0001000;

void terminalRx(char ch)
{
    ch = ch & 0x7F;

    putchar(ch);
    fflush(stdout);
}

bool hasInput(void)
{
    struct timeval tv = {0};

    fd_set fds;
    FD_ZERO(&fds);
    FD_SET(STDIN_FILENO, &fds);

    select(STDIN_FILENO + 1, &fds, NULL, NULL, &tv);

    return FD_ISSET(STDIN_FILENO, &fds);
}

void kbdTaskCb(void *arg)
{
    DL11 dl11 = (DL11)arg;

    static const char msg[] = "unix\n";
//    static const char msg[] = "";
    static const size_t msgLen = sizeof(msg) - 1;
    static size_t nProcessed = 0;

    char ch;
    if(nProcessed >= msgLen)
    {
        if(!hasInput())
            return;

        ssize_t res = read(STDIN_FILENO, &ch, sizeof(ch));
        if(res <= 0)
            return;
    }
    else
        ch = msg[nProcessed];

    if(dl11_rx(dl11, ch))
        ++nProcessed;
}

int main(void)
{
    ts_init();
    dev_init();
    mem_init(bootstrapBase, (uint8_t *)bootstrap, sizeof(bootstrap));

    if(!kw11_init())
        return EXIT_FAILURE;
    dev_registerDevice(kw11_getHandle());

    DL11 dl11 = dl11_init(0777560, 060, &terminalRx);
    if(!dl11)
        return EXIT_FAILURE;
    dev_registerDevice(dl11_getHandle(dl11));

    if(!rk11_init())
        return EXIT_FAILURE;
    if(!rk11_loadDisk("img/unix_v5_rk/unix_v5_rk.dsk", 0))
        return EXIT_FAILURE;
    dev_registerDevice(rk11_getHandle());

    if(!cpu_init(bootstrapBase))
        return EXIT_FAILURE;
    dev_registerDevice(cpu_getHandle());

    ts_handle kbdTask = ts_createTask(&kbdTaskCb, dl11);
    ts_schedulePeriodic(kbdTask, 10 * TS_MILLISECONDS);

    for(;;)
    {
        long int nSleep = ts_run();
        if(!cpu_run())
        {
            const long int nSleepMax = 10 * TS_MILLISECONDS;

            if(nSleep == 0 || nSleep > nSleepMax)
                nSleep = nSleepMax;

            usleep(nSleepMax / TS_MICROSECONDS);
        }
    }

    cpu_destroy();
    dl11_destroy(dl11);
    rk11_destroy();
    kw11_destroy();

    ts_destroyTask(kbdTask);

    return EXIT_SUCCESS;
}
