

#include <stdio.h>
#include <string.h>
#include "tcps_fifo.h"
#include <unistd.h>
#include <sys/time.h>
#include <time.h>
#include "tcps_common.h"

void maketimeout(struct timespec *tsp, long long int msec)
{
	clock_gettime(CLOCK_MONOTONIC, tsp);
	tsp->tv_sec += (tsp->tv_nsec + (msec * 1000000)) / 1000000000;
	tsp->tv_nsec = (tsp->tv_nsec + (msec * 1000000)) % 1000000000;
	//tsp->tv_sec += sec;
}

int kfifo_alloc(struct __kfifo *fifo, unsigned int len, size_t esize)
{
	/*
	 * round down to the next power of 2, since our 'let the indices
	 * wrap' technique works only in this case.
	 */
	 //DEBUG("%s(%d): len = %u , esize = %u\n", __func__, __LINE__, len, esize);
	//len = is_power_of_2(len);
	//DEBUG("%s(%d): len = %u , esize = %u\n", __func__, __LINE__, len, esize);
	fifo->in = 0;
	fifo->out = 0;
	fifo->esize = esize;

	fifo->data = (char *)malloc(len * esize);

	if (!fifo->data) {
		return -ENOMEM;
	}
    fifo->size = len * esize;
    //DEBUG("%s(%d): len = %u , esize = %u, fifo->size = %llu \n", __func__, __LINE__, len, esize, fifo->size);
	pthread_condattr_init(&fifo->condAttr);
	pthread_mutex_init(&(fifo->lock), NULL);
	pthread_condattr_setclock(&(fifo->condAttr), CLOCK_MONOTONIC);
    pthread_cond_init(&(fifo->rcond), &fifo->condAttr);
	return 0;
}

void kfifo_free(struct __kfifo *fifo)
{
    if (NULL != fifo->data)
    free(fifo->data);
    
    pthread_mutex_destroy(&(fifo->lock));
    pthread_cond_destroy(&(fifo->rcond));
	pthread_condattr_destroy(&(fifo->condAttr));
	fifo->in    = 0;
	fifo->out   = 0;
	fifo->esize = 0;
	fifo->data  = NULL;
    fifo->size  = 0;
}

//需要确保esize >= sizeof(struct __block)
unsigned int kfifo_put(struct __kfifo *fifo, struct __block *block)//const void *buf, unsigned int len)
{
    if (NULL == fifo || NULL == block)
    {
        return k_failure;
    }

    unsigned int step = 0;
    unsigned int len = 0;
    unsigned int l   = 0;
    
    pthread_mutex_lock(&(fifo->lock));

    len = sizeof(struct __block) + block->m_datasize;
    len = tcpsmin(len, fifo->esize);
    len = tcpsmin(len, fifo->size - (fifo->in - fifo->out));
//DEBUG("====>%s(%d): put fifo======>, in = %u, out = %u, cha = %d, size-cha = %u, len = %u, size = %llu\n", __func__, __LINE__, fifo->in, fifo->out, (fifo->in-fifo->out),
	//fifo->size - (fifo->in - fifo->out), len, fifo->size);
    block->m_datasize = len - sizeof(struct __block);

    smp_mb();
    l = tcpsmin(len, fifo->size - (fifo->in & (fifo->size - 1)));

    memcpy(fifo->data + (fifo->in & (fifo->size -1)), (char *)block, l);
    memcpy(fifo->data, (char *)block + l, len -l);
    smp_wmb();

    step = (0 == len) ? 0 : fifo->esize;
    fifo->in += step;
    if (len > 0)
    {
        pthread_cond_signal(&(fifo->rcond));
    }
//DEBUG("====>%s(%d): put fifo======>, in = %u, out = %u, cha = %d, size-cha = %u, len = %u, size = %llu\n", __func__, __LINE__, fifo->in, fifo->out, (fifo->in-fifo->out),
	//fifo->size - (fifo->in - fifo->out), len, fifo->size);

    pthread_mutex_unlock(&(fifo->lock));
    return len;
}

//需要确保esize >= sizeof(struct __block)
unsigned int kfifo_get(struct __kfifo *fifo, struct __block *block, int msec)
{
    if (NULL == fifo || NULL == block)
    {
        return -1;
    }

    int          ret   = 0;
    struct __block *pdata = NULL;
    unsigned int l     = 0;
    unsigned int len   = 0;
    unsigned int bytes = 0;
    unsigned int step  = 0;
    struct timespec sttime;
	memset(&sttime, 0, sizeof(struct timespec));
    pthread_mutex_lock(&(fifo->lock));
    maketimeout(&sttime, (long long int)msec);
    while(1)
    {   
        pdata = (struct __block *)(fifo->data + (fifo->out & (fifo->size - 1)));
        bytes = pdata->m_datasize + sizeof(struct __block);
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
        memcpy((char *)block, (char *)pdata, l);
        memcpy((char*)block + l, fifo->data, len -l);
        smp_mb();

        step = (0 == len) ? 0 : fifo->esize;
        fifo->out += step;
//DEBUG("====>%s(%d): get fifo======>, in = %u, out = %u, cha = %d, size-cha = %u, len = %u\n", __func__, __LINE__, fifo->in, fifo->out, (fifo->in-fifo->out),
	//fifo->size - (fifo->in - fifo->out), len);

        ret = 0;
        break;
    }
   
    return ret;
}

unsigned int kfifo_query(struct __kfifo *fifo, struct __block *block, int msec)
{
    if (NULL == fifo || NULL == block)
    {
        return -1;
    }

    int          ret   = 0;
    struct __block *pdata = NULL;
    unsigned int l     = 0;
    unsigned int len   = 0;
    unsigned int bytes = 0;
    //unsigned int step  = 0;
    struct timespec sttime;
	memset(&sttime, 0, sizeof(struct timespec));
    pthread_mutex_lock(&(fifo->lock));
    maketimeout(&sttime, (long long int)msec);
    while(1)
    {   
        pdata = (struct __block *)(fifo->data + (fifo->out & (fifo->size - 1)));
        bytes = pdata->m_datasize + sizeof(struct __block);
        len = tcpsmin(bytes, fifo->in - fifo->out);
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
        memcpy((char *)block, (char *)pdata, l);
        memcpy((char*)block + l, fifo->data, len -l);
        smp_mb();

        //step = (0 == len) ? 0 : fifo->esize;
        //fifo->out += step;
        ret = 0;
        break;
    }
    
    return ret;
}

struct __block *block_alloc(unsigned int size)
{
    if (size <=  sizeof(struct __block))
    {
        return NULL;
    }

    struct __block *p_block = NULL;

    p_block = (struct __block *)malloc(size);
    
    return p_block;
}

void block_free(struct __block *block)
{
    if (block)
    free(block);
    return;
}
