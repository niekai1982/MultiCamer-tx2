#ifndef _TX2_DEC_H_
#define _TX2_DEC_H_

#include "tcps_common.h"
#include "tcps_mempool.h"
#include "tcps_fifo.h"
#include "lib_decode.h"
#include "MoveSenseDataProcessing.h"

using namespace movesense;

/***************************************************************************
                            shelly add
***************************************************************************/

#define DEVS_DEC_MAX	(10)
#define CHNS_DEC_MAX	(10)

#define dec_depthwidth            (640)
#define dec_depthheight           (800)
#define img_depthwidth            (640)
#define img_depthheight           (400)

#define dec_rgbwidth              (640)
#define dec_rgbheight             (400)
#define img_rgbwidth              (640)
#define img_rgbheight             (400)

#define DEC_SIZE_64k		 (1024 * 640 * 3)//(1024 * 64)
#define DEC_SIZE_400x1p5	 (640 * 400 * 3 / 2 + 1024)
#define DEC_SIZE_400x2		 (640 * 400 * 2 + 1024)
#define DEC_SIZE_400x3		 (640 * 400 * 3 + 1024)
#define DEC_SIZE_800x3		 (640 * 800 * 3 + 1024)
#define DEC_SIZE_800x1p5	 (640 * 800 * 3 / 2 + 1024)

typedef enum
{
	size_400x2 = 0,
	size_400x3,
	size_800x3,
	size_64k,
	size_800x1p5,
	size_num
}tx2_dec_poolsize;

typedef enum
{
	dectype_rgb = 0,
	dectype_depth = 1,
	dectype_mix,
	dectype_num
}tx2_dec_type;

typedef struct
{
	Dec_Param_T 	m_decParam;     //解码器参数
	DecBuf_Param_T  m_decBuffParam;	//缓冲器参数
}tx2_dec_attr;

typedef struct
{
	int 			m_decid;						//解码器id
	tx2_dec_type    m_dectype;                      //解码器类型
	int             m_decchns;						//解码路数
	tx2_dec_attr    m_decattr;						//解码器属性
	LibDec*			p_decobj[CHNS_DEC_MAX];		    //解码实例
	MoveSenseDataProcessing *m_decdepth;            //深度转换
	memfifo_obj     m_decfifo[CHNS_DEC_MAX];	    //解码器fifo (int / out)
	void*			p_decpool[CHNS_DEC_MAX + 1];	//解码池
	memfifo_obj     m_outfifo;					    //输出fifo
	struct __block* p_reqblock;                     //临时buffer
    void*           p_dectmp[size_num];             //临时解码buffer
    int             m_startflag;                    //同步线程启动标志
}tx2_dec_obj;

typedef struct
{
	int             m_chnid;		//通道标号
	tx2_dec_type    m_dectype;		//解码器类型
	tx2_dec_attr    m_decrgbattr;   //rgb
	tx2_dec_attr    m_decdepthattr; //depth
}tx2_dec_params;

typedef struct
{
    tx2_dec_type  dectype;
    long long int timestamp;
    int           size;
	void*		  pdata;
}tx2_dec_frame;

typedef struct
{
    int                  m_chns;                //监控的通道数
    int                  m_chnid[CHNS_DEC_MAX]; //监控的通道id
    struct __block*      p_resblock;
    struct __kfifo       m_fifo;
}tx2_decs_admin;

int tx2_decsdev_create(int chnid, tx2_dec_type dectype, tx2_dec_params *param);
int tx2_decsdev_destroy(int chnid, tcps_frames *pframe);
int tx2_decsdev_putfrm(int chnid, tcps_frames *pframe);
int tx2_decsdev_getfrm(int chnid, tcps_frames **pframe, int msec);
int tx2_decsdev_release(int chnid, tcps_frames *pframe);
int tx2_decsadmin_init(int chns, int *chnid);
int tx2_decsuser_request(int chnid);
int tx2_decsadmin_respond(int msec);

#endif
