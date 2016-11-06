#ifndef __RBUF_H
#define __RBUF_H

#define RBUF_SIZE 64 

struct rbuf {
  char buffer[RBUF_SIZE];
  volatile int head;
  volatile int tail;
};

void rbuf_init(struct rbuf *rb);
int rbuf_pop(struct rbuf *rb, char *data);
int rbuf_push(struct rbuf *rb, char data);

#endif
