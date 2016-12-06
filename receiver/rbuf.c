#include <string.h>
#include <avr/interrupt.h>
#include "rbuf.h"

void rbuf_init(struct rbuf *rb)
{
  memset(rb->buffer, 0, RBUF_SIZE);
  rb->head = 0;
  rb->tail = 0;
}

int rbuf_push(struct rbuf *rb, char data)
{
  
  int next_head = (rb->head + 1) % RBUF_SIZE;
    
  if (next_head != rb->tail) {
    rb->buffer[rb->head] = data;
    rb->head = next_head;
    return 1;
  } else {
    return 0;
  }
}

int rbuf_pop(struct rbuf *rb, char *data)
{
  if (rb->head != rb->tail) {
    *data = rb->buffer[rb->tail];
    rb->tail = (rb->tail + 1) % RBUF_SIZE;
    return 1;
  } else {
    data = NULL;
    return 0;
  }
}

