//
// Created by palulukan on 6/23/20.
//

#include "task_scheduler.h"

#include <assert.h>
#include <string.h>
#include <stdlib.h>
#include <time.h>
#include <stdbool.h>

struct _task
{
    ts_cb cb;
    void *arg;
    unsigned long int ts;
    bool scheduled;
    struct _task *next;
    struct _task *prev;
};

static struct
{
    struct _task *head;
} task_sched;

static unsigned long int _getMonotonicTS(void)
{
    struct timespec ts = {0};
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (unsigned long int)ts.tv_sec * 1000000000UL + ts.tv_nsec;
}

void ts_init(void)
{
    memset(&task_sched, 0, sizeof(task_sched));
}

ts_handle ts_createTask(ts_cb cb, void* arg)
{
    if(!cb)
        return NULL;

    struct _task *pTask = calloc(1, sizeof(struct _task));
    if(!pTask)
        return NULL;

    pTask->cb = cb;
    pTask->arg = arg;

    return pTask;
}

void ts_destroyTask(ts_handle task)
{
    if(!task)
        return;

    ts_cancel(task);

    free(task);
}

bool ts_isScheduled(ts_handle task)
{
    struct _task *pTask = (struct _task *)task;

    return pTask && pTask->scheduled;
}

bool ts_schedule(ts_handle task, unsigned long int timeout_ns)
{
    struct _task *pTask = (struct _task *)task;

    if(!pTask || pTask->scheduled)
        return false;

    pTask->ts = _getMonotonicTS() + timeout_ns;
    pTask->scheduled = true;

    struct _task **it = &task_sched.head;
    while(*it && (*it)->ts <= pTask->ts)
        it = &(*it)->next;

    pTask->next = *it;
    *it = pTask;

    return true;
}

void ts_cancel(ts_handle task)
{
    struct _task *pTask = (struct _task *)task;
    if(!pTask || !pTask->scheduled)
        return;

    if(pTask->prev)
        pTask->prev->next = pTask->next;
    if(pTask->next)
        pTask->next->prev = pTask->prev;

    pTask->prev = NULL;
    pTask->next = NULL;
    pTask->scheduled = false;
}

long int ts_run(void)
{
    if(!task_sched.head)
        return 0;

    unsigned long int tsNow = _getMonotonicTS();
    while(task_sched.head && task_sched.head->ts <= tsNow)
    {
        struct _task *pTask = task_sched.head;
        task_sched.head = pTask->next;

        pTask->next = NULL;
        pTask->scheduled = false;

        pTask->cb(pTask->arg);
    }

    return task_sched.head ? task_sched.head->ts - tsNow : 0;
}
