

#include "queue.h"
#include <string.h>
#include <stdio.h>

void queue_init(queue_t *q, void *data, uint item_sz, uint depth)
{
    q->data = data;
    q->depth = depth;
    q->item_sz = item_sz;
    q->len = 0;
    q->head = 0;
}

void dump(uint8_t *ptr, uint sz)
{
    while (--sz)
    {
        printf("%02x", *ptr++);
    }
}

bool queue_add(queue_t *q, void *d)
{
    // are we full?
    if (q->len == q->depth)
    {
        return false;
    }
    // Copy data into tail of queue, incrementing len as we go
    memcpy(q->data + ((q->head + q->len) % q->depth) * q->item_sz, d, q->item_sz);
    q->len++;
    return true;
}

bool queue_get(queue_t *q, void *d)
{
    // Are we empty?
    if (q->len == 0)
    {
        return false;
    }
    // Copy head value into output
    memcpy(d, q->data + q->head * q->item_sz, q->item_sz);
    // Increment head
    q->head = (q->head + 1) % q->depth;
    // Decrement length;
    q->len--;
    return true;
}

void *queue_peek(queue_t *q)
{
    if (q->len == 0)
    {
        return NULL;
    }
    return q->data + (q->len * q->item_sz);
}

int queue_get_length(queue_t *q)
{
    return q->len;
}
