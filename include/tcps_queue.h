
#ifndef _TCPS_QUEUE_H_
#define _TCPS_QUEUE_H_

#include "tcps_fifo.h"

int tcps_queue_alloc(struct __kfifo *fifo, unsigned int size);
void tcps_queue_free(struct __kfifo *fifo);
int tcps_queue_put(struct __kfifo *fifo, void *buf, int size);
unsigned int tcps_queue_get(struct __kfifo *fifo, void * buf, int size, int msec);

#endif