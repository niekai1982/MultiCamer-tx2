
#include <stdio.h>
#include <string.h>
#include <time.h>
#include "tcps_fifo.h"
#include "tcps_mempool.h"


//申请内存
void  *tcps_memnode_alloc(void *memhandle, int* len)
{
    //if (len > ELEMENT_SIZE || len <= 0)
    //{
    //    return NULL; 
    //}
    
    mem_node         *pnode = NULL;
    struct list_head *pos   = NULL;
    membuf_obj *pmem    = (membuf_obj *)memhandle;
	
    pthread_mutex_lock(&pmem->m_lock);
    //获取空闲节点
    if (NULL == pmem->m_pbuf || pmem->m_freenum <= 0)
    {
        goto exit;
    }

    list_for_each(pos, &(pmem->m_freelist))
    {     
        pnode = list_entry(pos, mem_node, m_node);
        if (NULL == pnode)
        {
            goto exit;
        }
        //从空闲节点链表中删除
        *len = pnode->m_len;
        list_del(&(pnode->m_node));
        pmem->m_freenum--;
        break;
    }

exit:
    pthread_mutex_unlock(&pmem->m_lock);
    return (pnode != NULL) ? pnode->m_data : NULL;
}

//释放内存
int tcps_memnode_release(void *memhandle, void *pmem)
{
    if (NULL == pmem || NULL == memhandle)
    {
        return tfailure;
    }

    membuf_obj *pmem_buf    = (membuf_obj *)memhandle;
    mem_node *pnode = NULL;
    pthread_mutex_lock(&(pmem_buf->m_lock));

    pnode = (mem_node *)((char *)pmem - sizeof(mem_node));

    //添加空闲节点
    pnode->m_len = 0;
    list_add_tail(&(pnode->m_node), &(pmem_buf->m_freelist));
    pmem_buf->m_freenum++;

    pthread_mutex_unlock(&(pmem_buf->m_lock));
    return tsuccess;
}

void *tcps_mempool_init(unsigned int num, unsigned int size)
{
    //int       ret   = ffailure;
    unsigned int       i     = 0;
    mem_node *pnode = NULL;
    size_t    padlen;
	
	membuf_obj *framehandle =(membuf_obj *)malloc(sizeof(membuf_obj));
	
	int esize = sizeof(mem_node)+ size;
	padlen = (-esize) & (MEM_NODE_ALIGN - 1);
	esize += padlen;
    //int       esize = sizeof(mem_node) + size;
    //DEBUG("sizeof(frame_node)=%d\n", sizeof(frame_node));
    framehandle->m_pbuf = (void *)malloc(num * esize);

    //初始化互斥锁
    pthread_mutex_init(&(framehandle->m_lock), NULL);

    //初始化链表
    //INIT_LIST_HEAD(&(g_framebuf.m_busylist));
    INIT_LIST_HEAD(&(framehandle->m_freelist));

    //添加空节点
    for (i = 0 ; i < num; i++)
    {
        pnode = (mem_node *)((char *)(framehandle->m_pbuf) + i * esize);
		pnode->m_len = size;
        list_add_tail(&(pnode->m_node), &(framehandle->m_freelist) );
    }
  
    framehandle->m_freenum = num;

    return (void *)framehandle;
}

int tcps_mempool_exit(void * memhandle)
{
    membuf_obj *g_framebuf = (membuf_obj *)memhandle;
    pthread_mutex_lock(&g_framebuf->m_lock);
    free(g_framebuf->m_pbuf);
    pthread_mutex_unlock(&g_framebuf->m_lock);
    return 0;
}

int tcps_memfifo_create(memfifo_obj *fbufobj, unsigned int len)
{
    if (NULL == fbufobj || len <= 0)
    {
        return tfailure;
    }

    int ret = tfailure;
    unsigned int esize = 256;
   //DEBUG("frame_node = sizeof(%d)\n", sizeof(frame_node));
    ret = kfifo_alloc(&(fbufobj->m_fifo), len, esize);

    return ret;
}


int tcps_memfifo_destroy(memfifo_obj *fbufobj)
{
    if (NULL == fbufobj)
    {
        return tfailure;
    }
    
    int ret = tsuccess;
    kfifo_free(&(fbufobj->m_fifo));

    return ret;
}

int tcps_memfifo_put(memfifo_obj *fbufobj, char *pnode)
{
    if (NULL == pnode || NULL == fbufobj)
    {
        return tfailure;
    }

    int ret = tfailure;
    char block[128];
    long long int addr = (long long int)pnode;
    long long int *p = &addr;
    struct __block *pblock = (struct __block *)block;

    pblock->m_datasize = sizeof(pnode);

    //pblock->data = pnode;
    //frame_node *pnode1 = ()pblock->data;
    //pnode1 = pnode;
    memcpy(pblock->data, p, sizeof(long long int));
    ret = kfifo_put(&(fbufobj->m_fifo), pblock);

    return ret;
}

int tcps_memfifo_get(memfifo_obj *fbufobj, char **pnode, int msec)
{
    if (NULL == fbufobj || NULL == pnode)
    {
        return tfailure;
    }
    
    int ret = tfailure;

    char block[128];
    struct __block *pblock = (struct __block *)block;
    *pnode = NULL;
    ret = kfifo_get(&(fbufobj->m_fifo), pblock, msec);
    if (0 == ret)
    {
        *pnode = (char *)(*((long long int*)pblock->data));
    }
    
    return ret;
}

int tcps_memfifo_query(memfifo_obj *fbufobj, char **pnode, int msec)
{
    if (NULL == fbufobj || NULL == pnode)
    {
        return tfailure;
    }
    
    int ret = tfailure;

    char block[128];
    struct __block *pblock = (struct __block *)block;
    *pnode = NULL;
    ret = kfifo_query(&(fbufobj->m_fifo), pblock, msec);
    if (0 == ret)
    {
        *pnode = (char *)(*((long long int*)pblock->data));
    }
    
    return ret;
}

