#include "async.h"
#include <stdint.h>
#include <string.h>
#include <stdio.h>

static async_ctx_t async_queue[ASYNC_TASKS_MAX];

void async_init()
{
    for (async_ctx_t *t = async_queue; t < async_queue + ASYNC_TASKS_MAX; t++)
    {
        t->fn = NULL;
    }
}

/**
 * Counts the number of active tasks
 */
static size_t tasks_count()
{
    async_ctx_t *t;
    for (t = async_queue; t < async_queue + ASYNC_TASKS_MAX && t->fn; t++)
    {
    }
    return t - async_queue;
}

void async_exec_tasks()
{
    for (async_ctx_t *t = async_queue; t < async_queue + ASYNC_TASKS_MAX && t->fn; t++)
    {
        t->fn(t);
        if (t->resume_at == ASYNC_COMPLETE_MARKER)
        {
            // printf("Task at index %d completed\n", t - async_queue);
            // Task completed.
            // Find out how many tasks there are
            size_t left_in_loop = tasks_count() - (async_queue - t) - 1;
            t->fn = NULL;
            // Shift the remaining tasks down
            if (left_in_loop > 0)
            {
                // printf("copying down %d running tasks\n", left_in_loop);
                memcpy(t, t + 1, left_in_loop * sizeof(async_ctx_t));
            }
            // decrement t so that when it gets incremented again afterwards, we'll still be in the same place
            t--;
        }
    }
}

static async_ctx_t *async_start_task_common(async_handler_fn_t fn)
{
    int count = tasks_count();
    if (count == ASYNC_TASKS_MAX)
    {
        printf("Too many tasks already running\n");
        return NULL; // This is bad!!
    }
    async_ctx_t *t = async_queue + count;
    t->fn = fn;
    t->resume_at = 0;
    return t;
}

void async_start_task(async_handler_fn_t fn, int data)
{
    async_ctx_t *t = async_start_task_common(fn);
    if (t)
    {
        t->idata = data;
    }
}

void async_start_taskb(async_handler_fn_t fn, uint8_t *data, size_t sz)
{
    async_ctx_t *t = async_start_task_common(fn);
    if (t)
    {
        memcpy(t->bdata, data, sz);
    }
}
