#ifndef _TCPS_ALGO_INTERFACE_H_
#define _TCPS_ALGO_INTERFACE_H_

#include "tcps_common.h"
#include <opencv2/core/core.hpp>  
#include <opencv2/highgui/highgui.hpp>  
#include <opencv2/imgproc/imgproc.hpp>
#include "sort.h"


#define		WIN32_PLATFORM			0				//WIN32
#define		LINUX_PLATFORM			1				//ARM/LINUX
#define		ALGORITHM_PLATFORM		LINUX_PLATFORM	//**算法运行操作系统，需根据应用选择，必须选择**
#define     tk_points_num			(64)

#if (ALGORITHM_PLATFORM == WIN32_PLATFORM)
#define TCPS_Export		__declspec(dllexport)

#else
#define TCPS_Export		extern

#endif

typedef void *HANDLE;

//控制码命令
typedef enum
{
	//跟踪控制:
	CMD_ResetTrack	= 1,		//重置跟踪
	//日志控制：
	CMD_SaveLog		= 1 << 1,	//开启存储日志
	CMD_CloseLog	= 1 << 2,	//关闭存储日志
	CMD_PrintLog	= 1 << 3,	//开启打印日志
	CMD_HiddenLog	= 1 << 4,	//关闭打印日志
	//显示控制：
	CMD_ShowAmpImg	= 1 << 5,	//显示置信度图
	CMD_ShowRGB		= 1 << 6,	//显示原图
	CMD_ShowRGBAmp	= 1 << 7,	//显示置信度叠加在原图
	CMD_CloseImg	= 1 << 8,	//关闭图像显示
	CMD_ShowDetect	= 1 << 9,	//显示检测框
	CMD_HidDetect	= 1 << 10,	//关闭检测框
	CMD_ShowTrack	= 1 << 11,	//显示跟踪轨迹
	CMD_HidTrack	= 1 << 12,	//关闭跟踪轨迹
	CMD_ShowAmp		= 1 << 13,	//显示置信度叠加
	CMD_HidAmp		= 1 << 14,	//关闭置信度叠加
}ALGO_CRTL_CMD;



//算法运行输入参数
typedef struct
{
	int 				iSyncNum;		//同步帧路数
	tcps_frames 		*iSyncFrame[4]; //同步帧
}ALGO_Input_Param;

typedef struct
{
    unsigned int point_size;
	TrackingBox  track_point[tk_points_num];
}tcps_algo_track;

typedef struct
{
	long long int		time[2];
	unsigned int 		det_size;	
	TrackingBox 		det_res[tk_person_number];
	unsigned int 		match_size;
	TCPS_POINT		    match_point[tk_person_number];
	unsigned int 		sort_size;
	TrackingBox 		sort_res[tk_person_number];
	unsigned int 		pop_size;
	POP_RESULT_T 	    pop_res[tk_person_number];
    unsigned int        track_size;
	tcps_algo_track     track_persons[tk_person_number];
	int 				up_num;
	int 				down_num;
}tcps_algores;

typedef struct
{
    unsigned int    m_psize;
    TCPS_POINT      m_point[tk_points_num];
}tcps_algo_tarckperson;

typedef struct
{
    KELIU_SAVERESULT_T    m_convertres;
    unsigned int          m_personnum;
    tcps_algo_tarckperson m_trackperson[tk_person_number];
}tcps_algo_tracklist;

//算法运行输入参数
typedef struct
{
    tcps_algores        m_dynamicres;
    int                 m_devnum;
    tcps_algo_tracklist m_tracklist[2];
}ALGO_Output_Res;

typedef struct
{
	long long int		time;
    int                 devid;
	unsigned int 		sort_size;
	//TrackingBox2 		sort_res[tk_person_number];
	TrackingBox         sort_res[tk_person_number];
}tx2_sort_net;

/********************************************************************
名称：	TCPS_Algo_Create

功能：	算法句柄创建

参数：	无

返回：  HANDLE   模块句柄

其他：	无
*********************************************************************/
//TCPS_Export HANDLE Tcps_Algo_Create();

/********************************************************************
名称：	TCPS_Algo_Init

功能：	算法句柄初始化

参数：	

返回：  int		初始化返回值(0表示成功, 其他值通过错误码进行查询)

其他：	无
*********************************************************************/
TCPS_Export int Tcps_Algo_Init();


/********************************************************************
名称：	Tcps_Algo_Put

功能：	算法句柄初始化

参数：	HANDLE				_hHandle			算法句柄
		ALGO_Input_Param* _pstInputParam		算法输入参数结构体地址

返回：  int		初始化返回值(0表示成功, 其他值通过错误码进行查询)

其他：	无
*********************************************************************/
TCPS_Export int Tcps_Algo_Put(tx2_syncframe_out *_pstInputParam);//(ALGO_Input_Param* _pstInputParam);



/********************************************************************
名称：	Tcps_Algo_Get

功能：	算法句柄初始化

参数：	HANDLE				_hHandle			算法句柄
		ALGO_Input_Param* _pstInputParam		算法输入参数结构体地址

返回：  int		初始化返回值(0表示成功, 其他值通过错误码进行查询)

其他：	无
*********************************************************************/
//TCPS_Export int Tcps_Algo_Get(ALGO_Output_Param* _pstOutputParam);

TCPS_Export int Tcps_Algo_Get(ALGO_Output_Res *poutres, int msec);

/********************************************************************
名称：	TCPS_Algo_Release

功能：	算法句柄释放

参数：	HANDLE				_hHandle			算法句柄

返回：  int		释放返回值(0表示成功, 其他值通过错误码进行查询)

其他：	无
*********************************************************************/
TCPS_Export int TCPS_Algo_Release();

#endif
