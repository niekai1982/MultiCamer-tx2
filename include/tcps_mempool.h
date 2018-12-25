

#ifndef _TCPS_MEMPOOL_H_
#define _TCPS_MEMPOOL_H_

#include <stdio.h>
#include <string.h>
#include <time.h>
#include "tcps_fifo.h"
#include "list.h"

#ifdef __cplusplus
extern "C" {
#endif

#ifndef NULL
#define NULL 0
#endif

#define     tsuccess          (0)
#define     tfailure          (-1)
#define     ELEMENT_NUM       (128)
#define     ELEMENT_SIZE      (1024 * 1024 * 5)
#define MEM_NODE_ALIGN __alignof__(mem_node)
typedef struct
{
    uint32_t         m_len;                //���ݳ���
    struct list_head m_node;               //֡�ڵ�
    uint8_t          m_data[];             //֡����
}__attribute__((aligned(4))) mem_node;

typedef struct 
{
    pthread_mutex_t    m_lock;
    void*              m_pbuf;           //framebufָ��
    //struct list_head   m_busylist;       //���ڱ�ռ�õĽڵ�
    struct list_head   m_freelist;       //���нڵ�
    int                m_freenum;        //���нڵ���
}membuf_obj;

typedef struct
{
    struct __kfifo m_fifo;
    int            m_len;
    int            m_esize;
}memfifo_obj;

void  *tcps_memnode_alloc(void *memhandle, int *len);
int tcps_memnode_release(void *memhandle, void *pmem);
void *tcps_mempool_init(unsigned int num, unsigned int size);
int tcps_mempool_exit(void * memhandle);
int tcps_memfifo_create(memfifo_obj *fbufobj, unsigned int len);
int tcps_memfifo_destroy(memfifo_obj *fbufobj);
int tcps_memfifo_put(memfifo_obj *fbufobj, char *pnode);
int tcps_memfifo_get(memfifo_obj *fbufobj, char **pnode, int msec);
int tcps_memfifo_query(memfifo_obj *fbufobj, char **pnode, int msec);

#ifdef __cplusplus
}
#endif
#endif



