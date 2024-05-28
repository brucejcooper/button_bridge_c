#ifndef _LWT_H
#define _LWT_H

#include <stdint.h>
#include <stddef.h>

struct async_ctx_t;

typedef void (*async_handler_fn_t)(struct async_ctx_t *ctx);

typedef struct async_ctx_t
{
    unsigned int resume_at; // The duff's machine target that it will resume at
    async_handler_fn_t fn;  // The handler function

    union
    {
        int idata;
        uint8_t bdata[4];
    };
    int scratch; // Additional scratch data
} async_ctx_t;

#define ASYNC_TASKS_MAX 8

#define ASYNC_COMPLETE_MARKER 0xFFFF

#define async_begin()       \
    switch (ctx->resume_at) \
    {                       \
    case 0:

#define async_end()                             \
    default:                                    \
        ctx->resume_at = ASYNC_COMPLETE_MARKER; \
        return;                                 \
        }

#define await(condition)       \
    ctx->resume_at = __LINE__; \
    case __LINE__:             \
        if (!(condition))      \
        {                      \
            return;            \
        }

#define async_yield()          \
    ctx->resume_at = __LINE__; \
    return;                    \
    case __LINE__:

#define async_abort()                       \
    ctx->resume_at = ASYNC_COMPLETE_MARKER; \
    return;

void async_init();
void async_start_task(async_handler_fn_t fn, int idata);
void async_start_taskb(async_handler_fn_t fn, uint8_t *data, size_t sz);
void async_exec_tasks();

#endif
