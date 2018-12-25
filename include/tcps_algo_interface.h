#ifndef _TCPS_ALGO_INTERFACE_H_
#define _TCPS_ALGO_INTERFACE_H_

#include "tcps_common.h"
#include <opencv2/core/core.hpp>  
#include <opencv2/highgui/highgui.hpp>  
#include <opencv2/imgproc/imgproc.hpp>
#include "sort.h"


#define		WIN32_PLATFORM			0				//WIN32
#define		LINUX_PLATFORM			1				//ARM/LINUX
#define		ALGORITHM_PLATFORM		LINUX_PLATFORM	//**�㷨���в���ϵͳ�������Ӧ��ѡ�񣬱���ѡ��**
#define     tk_points_num			(64)

#if (ALGORITHM_PLATFORM == WIN32_PLATFORM)
#define TCPS_Export		__declspec(dllexport)

#else
#define TCPS_Export		extern

#endif

typedef void *HANDLE;

//����������
typedef enum
{
	//���ٿ���:
	CMD_ResetTrack	= 1,		//���ø���
	//��־���ƣ�
	CMD_SaveLog		= 1 << 1,	//�����洢��־
	CMD_CloseLog	= 1 << 2,	//�رմ洢��־
	CMD_PrintLog	= 1 << 3,	//������ӡ��־
	CMD_HiddenLog	= 1 << 4,	//�رմ�ӡ��־
	//��ʾ���ƣ�
	CMD_ShowAmpImg	= 1 << 5,	//��ʾ���Ŷ�ͼ
	CMD_ShowRGB		= 1 << 6,	//��ʾԭͼ
	CMD_ShowRGBAmp	= 1 << 7,	//��ʾ���Ŷȵ�����ԭͼ
	CMD_CloseImg	= 1 << 8,	//�ر�ͼ����ʾ
	CMD_ShowDetect	= 1 << 9,	//��ʾ����
	CMD_HidDetect	= 1 << 10,	//�رռ���
	CMD_ShowTrack	= 1 << 11,	//��ʾ���ٹ켣
	CMD_HidTrack	= 1 << 12,	//�رո��ٹ켣
	CMD_ShowAmp		= 1 << 13,	//��ʾ���Ŷȵ���
	CMD_HidAmp		= 1 << 14,	//�ر����Ŷȵ���
}ALGO_CRTL_CMD;



//�㷨�����������
typedef struct
{
	int 				iSyncNum;		//ͬ��֡·��
	tcps_frames 		*iSyncFrame[4]; //ͬ��֡
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

//�㷨�����������
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
���ƣ�	TCPS_Algo_Create

���ܣ�	�㷨�������

������	��

���أ�  HANDLE   ģ����

������	��
*********************************************************************/
//TCPS_Export HANDLE Tcps_Algo_Create();

/********************************************************************
���ƣ�	TCPS_Algo_Init

���ܣ�	�㷨�����ʼ��

������	

���أ�  int		��ʼ������ֵ(0��ʾ�ɹ�, ����ֵͨ����������в�ѯ)

������	��
*********************************************************************/
TCPS_Export int Tcps_Algo_Init();


/********************************************************************
���ƣ�	Tcps_Algo_Put

���ܣ�	�㷨�����ʼ��

������	HANDLE				_hHandle			�㷨���
		ALGO_Input_Param* _pstInputParam		�㷨��������ṹ���ַ

���أ�  int		��ʼ������ֵ(0��ʾ�ɹ�, ����ֵͨ����������в�ѯ)

������	��
*********************************************************************/
TCPS_Export int Tcps_Algo_Put(tx2_syncframe_out *_pstInputParam);//(ALGO_Input_Param* _pstInputParam);



/********************************************************************
���ƣ�	Tcps_Algo_Get

���ܣ�	�㷨�����ʼ��

������	HANDLE				_hHandle			�㷨���
		ALGO_Input_Param* _pstInputParam		�㷨��������ṹ���ַ

���أ�  int		��ʼ������ֵ(0��ʾ�ɹ�, ����ֵͨ����������в�ѯ)

������	��
*********************************************************************/
//TCPS_Export int Tcps_Algo_Get(ALGO_Output_Param* _pstOutputParam);

TCPS_Export int Tcps_Algo_Get(ALGO_Output_Res *poutres, int msec);

/********************************************************************
���ƣ�	TCPS_Algo_Release

���ܣ�	�㷨����ͷ�

������	HANDLE				_hHandle			�㷨���

���أ�  int		�ͷŷ���ֵ(0��ʾ�ɹ�, ����ֵͨ����������в�ѯ)

������	��
*********************************************************************/
TCPS_Export int TCPS_Algo_Release();

#endif
