#ifndef _QUEUE_H
#define _QUEUE_H
#include <pico/stdlib.h>

typedef struct
{
    uint8_t *data;
    uint item_sz;
    uint depth;
    uint head;
    uint len;
} queue_t;

void queue_init(queue_t *q, void *data, uint item_sz, uint depth);
bool queue_add(queue_t *q, void *d);
bool queue_get(queue_t *q, void *d);
void *queue_peek(queue_t *q);
int queue_get_length(queue_t *q);
#endif
