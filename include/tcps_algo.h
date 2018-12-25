#ifndef _TCPS_ALGO_H_
#define _TCPS_ALGO_H_


#pragma once

//#include "OpenNI.h"

#include <time.h>
//#include<direct.h> 

//#include "Logger.h"
#include"detection.h"
#include"parameters.h"
#include"coordinate_transform.h"
#include"probability.h"
#include"trajectory.h"
#include"common.h"
#include "tcps_algo_interface.h"
#include "tcps_mempool.h"
#include "tcps_common.h"



#define       det_3dbox_size 	(150)

typedef struct
{
	int	m_devid;
	memfifo_obj         m_fifo[4];
}tcps_algodev_obj;

typedef struct
{
	int 			m_devnum;
	tcps_algodev_obj 	m_dev[4];
	void*             m_pool[2];
}tcps_algodev_list;

typedef struct
{
	int 			m_len;
	long long int    m_time[2];
	char*             m_boxframe;
	char *		m_rgbframe;
	char * 		m_rgbframe2;
}tcps_box_obj;



extern struct parameters parameter_;
extern vector<TrackingBox> frameTrackingResult;
extern Sort *mot_tracker;

extern struct SortRes  sortres;

extern ofstream outfile;

extern int up_num;
extern int down_num;

int tcps_sort_init();
//void tcps_algo_kp(Mat depth, Mat rgb, int camera_id, vector<TrackingBox> &dets);
//void tcps_algo_put(Mat depth0, Mat rgb0, int camera_id0, Mat depth1, Mat rgb1, int camera_id1);

int create_algo_fifo();

//int result2dets( tcps_imgpred  m_algopred, vector<TrackingBox> &det_news);

int Tcps_Algo_Init();

int Tcps_Algo_Put(tx2_syncframe_out *_pstInputParam);//(ALGO_Input_Param* _pstInputParam);

int Tcps_Algo_Get(ALGO_Output_Res *poutres, int msec);//(tcps_algores * rgb2result, int msec);


#endif
