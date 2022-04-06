#include <stdint.h>

struct queue
{
    void **buff;
    uint8_t head;
    uint8_t tail;
    uint8_t size;
};

void queue_push(struct queue *q, void *addr)
{
    q->buff[q->head] = addr;
    q->head++;
    q->head &= (1u << q->size);
}

void *queue_pop(struct queue *q)
{
    q->tail++;
    q->tail &= (1u << q->size);

    return q->buff[q->tail];
}
