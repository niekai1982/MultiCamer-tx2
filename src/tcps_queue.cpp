#include <stdio.h>
#include <string.h>
#include "tcps_queue.h"

int tcps_queue_alloc(struct __kfifo *fifo, unsigned int size)
{
	/*
	 * round down to the next power of 2, since our 'let the indices
	 * wrap' technique works only in this case.
	 */
	fifo->in = 0;
	fifo->out = 0;
	//fifo->esize = esize;

	fifo->data = (char *)malloc(size);

	if (!fifo->data) {
		return -ENOMEM;
	}
    fifo->size = size;
    //DEBUG("%s(%d): len = %u , esize = %u, fifo->size = %llu \n", __func__, __LINE__, len, esize, fifo->size);
	pthread_condattr_init(&fifo->condAttr);
	pthread_mutex_init(&(fifo->lock), NULL);
	pthread_condattr_setclock(&(fifo->condAttr), CLOCK_MONOTONIC);
    pthread_cond_init(&(fifo->rcond), &fifo->condAttr);

	return 0;
}

void tcps_queue_free(struct __kfifo *fifo)
{
	if (NULL != fifo->data)
    free(fifo->data);
    
    pthread_mutex_destroy(&(fifo->lock));
    pthread_cond_destroy(&(fifo->rcond));
	pthread_condattr_destroy(&(fifo->condAttr));
	fifo->in    = 0;
	fifo->out   = 0;
	//fifo->esize = 0;
	fifo->data  = NULL;
    fifo->size  = 0;
}

int tcps_queue_put(struct __kfifo *fifo, void *buf, int size)
{
	if (NULL == fifo || NULL == buf)
    {
        return k_failure;
    }

    unsigned int len = 0;
    unsigned int l   = 0;
    
    pthread_mutex_lock(&(fifo->lock));

    len = size;
    len = tcpsmin(len, fifo->size - (fifo->in - fifo->out));
    smp_mb();
    l = tcpsmin(len, fifo->size - (fifo->in & (fifo->size - 1)));

    memcpy(fifo->data + (fifo->in & (fifo->size -1)), (char *)buf, l);
    memcpy(fifo->data, (char *)buf + l, len -l);
    smp_wmb();

    //step = (0 == len) ? 0 : fifo->esize;
    fifo->in += len;
    if (len > 0)
    {
        pthread_cond_signal(&(fifo->rcond));
    }
//DEBUG("====>%s(%d): put fifo======>, in = %u, out = %u, cha = %d, size-cha = %u, len = %u, size = %llu\n", __func__, __LINE__, fifo->in, fifo->out, (fifo->in-fifo->out),
	//fifo->size - (fifo->in - fifo->out), len, fifo->size);

    pthread_mutex_unlock(&(fifo->lock));
    return len;
}

unsigned int tcps_queue_get(struct __kfifo *fifo, void * buf, int size, int msec)
{
    if (NULL == fifo || NULL == buf || 0 == size)
    {
        return 0;
    }
	int          ret    = 0;
    unsigned int l      = 0;
    unsigned int len    = 0;
    unsigned int bytes  = 0;
	char        *pdata  = NULL;
    struct timespec sttime;
	memset(&sttime, 0, sizeof(struct timespec));
    pthread_mutex_lock(&(fifo->lock));
    maketimeout(&sttime, (long long int)msec);
    while(1)
    {   
        bytes = size;
		pdata = (char *)(fifo->data + (fifo->out & (fifo->size - 1)));
        len = tcpsmin(bytes, fifo->in - fifo->out);
		//DEBUG("====>%s(%d): get fifo======>, in = %u, out = %u, cha = %d, size-cha = %u, len = %u\n", __func__, __LINE__, fifo->in, fifo->out, (fifo->in-fifo->out),
	//fifo->size - (fifo->in - fifo->out), len);
  
        if (0 == len)
        {  
			ret = pthread_cond_timedwait(&(fifo->rcond), &(fifo->lock), &sttime);
            if (0 == ret)
            {
                continue;
            }   
            else
            {
                pthread_mutex_unlock(&(fifo->lock));
                break;
            }       
        }
        pthread_mutex_unlock(&(fifo->lock));
        smp_rmb();
        l = tcpsmin(len, fifo->size - (fifo->out &(fifo->size - 1)));
        memcpy((char *)buf, (char *)pdata, l);
        memcpy((char*)buf + l, fifo->data, len -l);
        smp_mb();

        fifo->out += len;
//DEBUG("====>%s(%d): get fifo======>, in = %u, out = %u, cha = %d, size-cha = %u, len = %u\n", __func__, __LINE__, fifo->in, fifo->out, (fifo->in-fifo->out),
	//fifo->size - (fifo->in - fifo->out), len);

        break;
    }
   
    return len;
}

unsigned int tcps_queue_quary(struct __kfifo *fifo, void * buf, int size, int msec)
{
	if (NULL == fifo || NULL == buf)
    {
        return 0;
    }
	int          ret   = 0;
    unsigned int l     = 0;
    unsigned int len   = 0;
    unsigned int bytes = 0;
    struct timespec sttime;
	char        *pdata  = NULL;
	memset(&sttime, 0, sizeof(struct timespec));
    pthread_mutex_lock(&(fifo->lock));
    maketimeout(&sttime, (long long int)msec);
    while(1)
    {   
        bytes = size;
		pdata = (char *)(fifo->data + (fifo->out & (fifo->size - 1)));
        len = tcpsmin(bytes, fifo->in - fifo->out);
		//DEBUG("====>%s(%d): get fifo======>, in = %u, out = %u, cha = %d, size-cha = %u, len = %u\n", __func__, __LINE__, fifo->in, fifo->out, (fifo->in-fifo->out),
	//fifo->size - (fifo->in - fifo->out), len);
  
        if (0 == len)
        {  
			ret = pthread_cond_timedwait(&(fifo->rcond), &(fifo->lock), &sttime);
            if (0 == ret)
            {
                continue;
            }   
            else
            {
                pthread_mutex_unlock(&(fifo->lock));
                break;
            }       
        }
        pthread_mutex_unlock(&(fifo->lock));
        smp_rmb();
        l = tcpsmin(len, fifo->size - (fifo->out &(fifo->size - 1)));
        memcpy((char *)buf, (char *)pdata, l);
        memcpy((char*)buf + l, fifo->data, len -l);
        smp_mb();

        //fifo->out += len;
//DEBUG("====>%s(%d): get fifo======>, in = %u, out = %u, cha = %d, size-cha = %u, len = %u\n", __func__, __LINE__, fifo->in, fifo->out, (fifo->in-fifo->out),
	//fifo->size - (fifo->in - fifo->out), len);

        break;
    }
   
    return len;
}
