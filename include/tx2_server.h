#ifndef _TX2_SERVER_H_
#define _TX2_SERVER_H_

#ifdef __cplusplus
extern "C" {
#endif

#define LENGTH      (65536)
//#define CLIENT_SUM  (4)

#include <netinet/in.h>
#include <sys/wait.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <sys/select.h>
#include <sys/time.h>
#include <sys/types.h>
#include <unistd.h>

#include "tcps_mempool.h"
#include "tcps_common.h"
#include "lib_display.h"
#define OUTNUM_PER  (2)
#define save_datalen        (1024 * 1024 * 15)
//#define FIFO_NUM    (3)

typedef enum
{
    type_dec = 0,
    type_save,
    type_num
}tx2_frame_type;

typedef struct
{
    int                 m_id;       //客户ID
    int                 m_run;
    int   				m_socket;       
	struct sockaddr_in  m_client;
    memfifo_obj         m_fifo[OUTNUM_PER];
    struct __kfifo      m_queue;
    void*               m_pool[OUTNUM_PER];
}client_item;

typedef struct
{   
    int                m_num;
    client_item        m_client[CLIENT_SUM];
    memfifo_obj        m_syncfifo;
    void *             p_syncpool;
    unsigned int       m_sync;
    pthread_condattr_t condAttr;
    pthread_cond_t     m_rcond;
    pthread_mutex_t    m_lock;
}tx2_client_list;

typedef enum
{
    func_keliu0 = 0,
    func_keliu1,
    func_dynamic,
    func_num
}tx2_virtdev_func;

typedef struct
{
    int                m_init;              //初始化标志
    struct __kfifo     m_kfifo[FIFO_NUM];
    memfifo_obj        m_fifo[FIFO_NUM];
    //void*              p_frame;
}tx2_virtdev_obj;

typedef struct
{
    char *p_path;
    int   m_wrgblen;
    char *p_rgbdata;
    int   m_wdepthlen;
    char *p_depthdata;
}tx2_save_obj;

typedef struct
{
    int         m_init;
    LibDisplay *p_display;
}tx2_display_obj;

#ifdef __cplusplus
}
#endif
#endif
