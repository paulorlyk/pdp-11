//
// Created by palulukan on 6/23/20.
//

#ifndef TASK_SCHEDULER_H_B71DBF1E154240A18533E510CAF62AAF
#define TASK_SCHEDULER_H_B71DBF1E154240A18533E510CAF62AAF

#include <stdbool.h>

typedef void (*ts_cb)(void*);

typedef void* ts_handle;

#define TS_NANOSECONDS  (TS_MICROSECONDS / 1000UL)
#define TS_MICROSECONDS (TS_MILLISECONDS / 1000UL)
#define TS_MILLISECONDS (TS_SECONDS / 1000UL)
#define TS_SECONDS      (1000000000UL)

void ts_init(void);

ts_handle ts_createTask(ts_cb cb, void* arg);

void ts_destroyTask(ts_handle task);

bool ts_isScheduled(ts_handle task);

bool ts_schedule(ts_handle task, unsigned long int timeout_ns);

void ts_cancel(ts_handle task);

// Return values:
//  < 0 - error
//  = 0 - task queue is empty
//  > 0 - time to the next event in nanoseconds
long int ts_run(void);

#endif //TASK_SCHEDULER_H_B71DBF1E154240A18533E510CAF62AAF
