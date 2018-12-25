
#ifndef _TCPS_HUMANPLUS_COMMON_H_
#define _TCPS_HUMANPLUS_COMMON_H_

#include <sys/syscall.h> 
#define gettid() syscall(__NR_gettid)

//add 20180726
//#define  __DEBUG
#ifdef __DEBUG

#define DEBUG    printf

#else

#define DEBUG

#endif

#ifdef TCPS_DEBUG
#define tk_printf(level, format, ...)   \
do {    \
    char *color = (level == 0) ? COLOR_YELLOW : \
                (level == 0xff0000) ? COLOR_RED : \
                (level == 0x12345678) ? COLOR_CYAN : COLOR_GREEN;  \
    printf("%s"format"%s\n", color, ##__VA_ARGS__, COLOR_OFF);    \
} while (0)
#else
#define tk_printf(level, format, ...)
#endif

#define tk_person_number    (128)
#define ALGO_FIFO_NUM			(3)
#define algo_input_num		(2)

#define rgb_width 		(640)
#define rgb_height 		(400)
#define depth_width 		(640)
#define depth_height 		(400)
#define depth_decwidth      (640)
#define depth_decheight     (800)
#define tk_inputcamera_number	(4)
#define FIFO_NUM		(6)
#define CLIENT_SUM  (4)
#define tcps_block_size			(1024*128)
#define sortnet_block_size			(1024*512)
//#define tsuccess    (0)
//#define tfailure    (-1)

#define TRUE     (1)
#define FALSE    (0)

/**************************************************
    �����ʽ
    
***************************************************/
typedef enum
{
    tcps_enc_null = 0,
    tcps_enc_h264,
    tcps_enc_num
}tcps_encformat;

/**************************************************
    fifo type
    
***************************************************/
/*
typedef enum
{
	tcps_fifo_cap = 0,
	tcps_fifo_dec,
	tcps_fifo_save,
	tcps_fifo_sync,
	tcps_fifo_display
}tcps_fifo_format;
*/

/**************************************************
    fifo type
    
***************************************************/
typedef enum
{
	tcps_fifo_put = 0,
	tcps_fifo_rgb2depth,
	tcps_fifo_get,
	//tcps_fifo_sync,
	tcps_fifo_display
}tcps_fifo_format;

/**************************************************
    ͼƬ֡����

***************************************************/
typedef struct
{
    unsigned int     m_frmid;        //֡ID
    long long int    m_timestamp;    //ʱ���
    unsigned int     m_imgwidth;     //ͼ����
    unsigned int     m_imgheight;    //ͼ��߶�
    unsigned int     m_pixelwidth;   //���ؿ��
    int              m_framelen;     //֡��
}tcps_imgframe_para;

/**************************************************
    ͼƬ֡

***************************************************/
typedef struct
{
    unsigned int     m_frmid;        //֡ID
    long long int    m_timestamp;    //ʱ���
    unsigned int     m_imgwidth;     //ͼ����
    unsigned int     m_imgheight;    //ͼ��߶�
    unsigned int     m_pixelwidth;   //���ؿ��
    int              m_framelen;     //֡��
    void*            p_data;         //֡����
}tcps_imgframe;

/**************************************************
    ͼƬ֡���㷨Ԥ����

***************************************************/
typedef struct
{
	float x;
	float y;
	float z;
	float width;
	float height;
	float l;
}RECT_S;

typedef struct
{
	float x;
	float y;
}TCPS_POINT;

typedef struct _KELIU_DETPERSON_TAG
{
	unsigned int id;
	RECT_S rec;
	float value;
	TCPS_POINT point;//det_point
}KELIU_DETPERSON_T;

typedef struct _DET_RESULT_TAG
{
	float depth_x;
	float depth_y;
	float depth_z;
	//float size_x;
	//float size_y;
	//float size_z;
	RECT_S det_rec;
	int camera_id;
	
}DET_RESULT_T;

typedef struct _SORT_RESULT_TAG
{
	unsigned int id;	
	RECT_S sort_rec[tk_person_number];
	float pro;
	
}SORT_RESULT_T;

typedef struct _POP_RESULT_TAG
{
	unsigned int pop_id;	
	unsigned int pop_size;
	unsigned int reserved = 1;
	
}POP_RESULT_T;

typedef struct TrackingBox2
{
	int frame;
	int id;
	RECT_S box;
	float pro;
	int camera_id;
}TrackingBox2;


typedef struct _KELIU_SAVERESULT_TAG
{
	unsigned int 		id;	//sort ID
	long long int 		frameid;	//֡ID
	long long int		devid;
	long long int		time;
	unsigned int 		det_size;	
	DET_RESULT_T 	det_res[tk_person_number];
	unsigned int 		match_size;
	TCPS_POINT		match_point[tk_person_number];
	unsigned int 		sort_size;
	TrackingBox2 		sort_res[tk_person_number];
	unsigned int 		pop_size;
	POP_RESULT_T 	pop_res[tk_person_number];
	int 				up_num;
	int 				down_num;
	
}KELIU_SAVERESULT_T;

/*
typedef struct
{
	unsigned int dev_id;
	long long int iFrameId;
	long long int time;
	unsigned int total_number;
	KELIU_DETPERSON_T person[tk_person_number];
}tcps_imgpred;
*/
typedef struct
{
	KELIU_SAVERESULT_T keliuresult;
}tcps_imgpred;


/**************************************************
    ��Ƶ֡

***************************************************/
typedef struct
{
    unsigned int  m_devid;
 	tcps_imgpred  m_algopred;
 	tcps_imgframe m_rgbimg;
 	tcps_imgframe m_depthimg;
}tcps_frames;

/**************************************************
    ͬ��֡����

***************************************************/
typedef struct
{
    int           m_clientid;
    long long int m_timestamp;
    void         *m_pframe;
}tx2_syncframe_input;

/**************************************************
    ͬ��֡���

***************************************************/
typedef struct
{
    int         syncnum;                 //ͬ��·��
    void       *syncframe[CLIENT_SUM];   //ͬ��֡
}tx2_syncframe_out;



/*����ʱ���ǩ�ṹ��*/
typedef struct DATEINFO_S
{
	unsigned char   year;                   /*��*/
	unsigned char   month;                  /*��*/
	unsigned char   day;                    /*��*/
	unsigned char   hour;                   /*ʱ*/
	unsigned char   minute;                 /*��*/
	unsigned char   second;                 /*��*/
	unsigned int   msecond;
	unsigned char   week;					/*����*/
	unsigned char	isdst;					/*��ʱ��*/
	unsigned char   reserver[2];
}T_IPC_Date;

#endif



