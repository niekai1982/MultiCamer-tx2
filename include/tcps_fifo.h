
#ifndef _KFIFO_H
#define _KFIFO_H
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <pthread.h>
#include <stdint.h>
#include <errno.h>

#ifdef __cplusplus
extern "C" {
#endif

#define k_success   (0)
#define k_failure   (-1)

#define barrier() asm volatile("" ::: "memory")
#define mb() __sync_synchronize()

#define smp_mb()	mb()
#define smp_rmb()	barrier()
#define smp_wmb()	barrier()


//判断x是否是2的次方
//#define is_power_of_2(x) ((x) != 0 && (((x) & ((x) - 1)) == 0))
//取a和b中最小值
#define tcpsmin(a, b) (((a) < (b)) ? (a) : (b))

//结构体成员的偏移量
#define offset(struct_type, member) ((size_t) &((struct_type *) 0)->member)

struct __kfifo {
	unsigned int	in;
	unsigned int	out;
	unsigned int	esize;
    unsigned long long int    size;
	char		   *data;
    pthread_condattr_t condAttr;
    pthread_cond_t  rcond;
    //pthread_cond_t  wcond;
    pthread_mutex_t lock;
};


struct __block{
    unsigned int   m_datasize;   //数据大小
    uint8_t        data[];
};




int kfifo_alloc(struct __kfifo *fifo, unsigned int len, size_t esize);
void kfifo_free(struct __kfifo *fifo);
unsigned int kfifo_put(struct __kfifo *fifo, struct __block *block);

unsigned int kfifo_get(struct __kfifo *fifo, struct __block *block, int msec);
unsigned int kfifo_query(struct __kfifo *fifo, struct __block *block, int msec);
struct __block *block_alloc(unsigned int size);
void block_free(struct __block *block);

void maketimeout(struct timespec *tsp, long long int msec);

#ifdef __cplusplus
}
#endif
#endif

