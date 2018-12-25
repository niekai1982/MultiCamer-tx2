#include"tcps_algo.h"
#include "tcps_algo_interface.h"
#include <sys/time.h>//add 20181129
#include <sys/syscall.h>  
#include <pthread.h>
#define gettid() syscall(__NR_gettid)


using namespace std;
using namespace cv;

int up_num = 0;
int down_num = 0;

Sort *mot_tracker;
struct parameters parameter_;
struct SortRes  sortres;

tcps_algodev_list 		g_algodev;
int m_algofifo_flag = -1;


int tcps_sort_init()
{
	
	int readparameters = ReadParameters( parameter_);
	mot_tracker=new Sort(parameter_.max_age, parameter_.min_hits);

	return 0;
}


int create_algo_fifo()
{

	int ret = -1;

	for(int i=0; i<algo_input_num; i++)
	{
		g_algodev.m_dev[i].m_devid = i;
		for(int j=0; j<ALGO_FIFO_NUM; j++)
		{
			ret = tcps_memfifo_create(&g_algodev.m_dev[i].m_fifo[j], 32);
			if(ret < 0)
			{
				printf("tcps_memfifo_create g_algodev.m_dev[%d].m_fifo[%d] failed\n", i, j);
				return ret;
			}
			else
			{
				DEBUG("tcps_memfifo_create g_algodev.m_dev[%d].m_fifo[%d] successed\n", i, j);
			}
			
		}
		
	}

	return 0;
}


vector<TrackingBox> result2dets(int devid, tcps_imgpred  m_algopred)
{

	int ret = -1;
	
	vector<TrackingBox> box;
	
	for(int i=0; i<m_algopred.keliuresult.det_size; i++)
	{
		TrackingBox tmp;
		
		tmp.box.center.x = m_algopred.keliuresult.det_res[i].depth_x;
		tmp.box.center.y = m_algopred.keliuresult.det_res[i].depth_y;
		tmp.box.center.z = m_algopred.keliuresult.det_res[i].depth_z;
		tmp.box.size.x = det_3dbox_size;
		tmp.box.size.y = det_3dbox_size;
		tmp.box.size.z = det_3dbox_size;
		
		tmp.id = -1;
		tmp.frame = -1;
		//tmp.pro = -1;//open rgb
		tmp.pro = 1;//close rgb
		tmp.camera_id = devid;
	
		box.push_back(tmp);

	}

	return box;
}



void* tcpsrgbalgo_hander_thread( void *arg )
{
	
	printf("====>%s(%d): dectid=%lun \n", __func__, __LINE__, gettid());
	
	int ret = -1;
	char 			*tmp   = NULL;
	tcps_frames          *pframe[algo_input_num]   = {NULL};
	tcps_box_obj 		*box = NULL;
	vector<TrackingBox> dets[2];
	
	int devid[2] = {-1, -1};
	int size = 0;
	long long int srctime[2] = {0};
	tx2_syncframe_out *input_param = NULL;
				
	while(1)
	{
		if(m_algofifo_flag != 1)
		{
			usleep(1000 * 10);
			continue;
		}

		ret = tcps_memfifo_get(&(g_algodev.m_dev[0].m_fifo[tcps_fifo_put]), &tmp, 5);
		
		if(ret != 0)
		{
			usleep(5000);
			continue;
		}
		else
		{
			input_param = (tx2_syncframe_out *)tmp;
			for(int i=0; i<input_param->syncnum; i++)
			{
				pframe[i] = (tcps_frames  *)input_param->syncframe[i];
				devid[i] = pframe[i]->m_devid+1;
				dets[i] = result2dets(devid[i], pframe[i]->m_algopred);
				srctime[i] = pframe[i]->m_algopred.keliuresult.time;	
			}
		}
	
		cv::Mat depth(depth_height, depth_width, CV_16UC1, pframe[0]->m_depthimg.p_data);
		cv::Mat rgb1(rgb_height, rgb_width, CV_8UC3, pframe[0]->m_rgbimg.p_data);

		cv::Mat depth2(depth_height, depth_width, CV_16UC1, pframe[1]->m_depthimg.p_data);
		cv::Mat rgb2(rgb_height, rgb_width, CV_8UC3, pframe[1]->m_rgbimg.p_data);

		//////probability_process(depth, rgb1, dets[0], 0.3);//rgb-pro
		
		world_union_coor(devid[0], dets[0]);
		
		//////probability_process(depth2, rgb2, dets[1], 0.3);//rgb-pro
		
		world_union_coor(devid[1], dets[1]);
		
		vector<TrackingBox> dets_new;
		vector<TrackingBox> dets_src;
		dets_src.insert(dets_src.end(), dets[0].begin(), dets[0].end());//将vec1压入
		dets_src.insert(dets_src.end(), dets[1].begin(), dets[1].end());//继续将vec2压入
		dets_new = remove_closer_WorldPoint(dets_src, 0.2);

		box = (tcps_box_obj *)tcps_memnode_alloc(g_algodev.m_pool[1], &size);
		if (NULL == box)
		{
			printf("alloc failed, box is %p\n", box);
			continue;
		}
		box->m_time[0] = srctime[0];
		box->m_time[1] = srctime[1];
		box->m_rgbframe = (char *)pframe[0]->m_rgbimg.p_data;
		box->m_rgbframe2 = (char *)pframe[1]->m_rgbimg.p_data;
		box->m_boxframe = (char *)box + sizeof(tcps_box_obj);
		int len = dets_new.size();
		box->m_len = len;
	
		for(int j=0; j<dets_new.size(); j++)
		{
			memcpy( box->m_boxframe + j * sizeof(TrackingBox), &(dets_new[j]), sizeof(TrackingBox));
		}
		
		ret = tcps_memfifo_put(&(g_algodev.m_dev[0].m_fifo[tcps_fifo_rgb2depth]), (char *)box);
		if(ret <= 0)
		{
			printf("====>%s(%d): ============>\n", __func__, __LINE__);
			tcps_memnode_release((void *)(g_algodev.m_pool[1]), (void *)box);
		}		
		usleep(5000);
	}
}

void* tcpsdepthalgo_hander_thread( void *arg )
{
	printf("====>%s(%d): dectid=%lun \n", __func__, __LINE__, gettid());
	int ret = -1;
	char *   depthnode = NULL;
	tcps_frames          *srcframe[algo_input_num]   = {NULL};
	tcps_box_obj 		*box = NULL;
	tcps_algores  * keliu_result = NULL;
	int size = 0;

	
	while(1)
	{
		if(m_algofifo_flag != 1)
		{
			usleep(1000*10);
			continue;
		}
		
		vector<TrackingBox> dets_new;

		ret = tcps_memfifo_get(&(g_algodev.m_dev[0].m_fifo[tcps_fifo_rgb2depth]), &depthnode, 5);
		if(ret != 0)
		{
			usleep(5000);
			continue;
		}
		else if(ret == 0)
		{
			box = (tcps_box_obj *)depthnode;
			for(int j=0; j<box->m_len; j++)
			{
				TrackingBox tmpbox;
				memcpy(&tmpbox, &box->m_boxframe[j * sizeof(TrackingBox)], sizeof(TrackingBox));
				dets_new.push_back(tmpbox);
			}
			
			keliu_result = (tcps_algores *)tcps_memnode_alloc(g_algodev.m_pool[1], &size);
			if(NULL == keliu_result)
			{
				printf("alloc keliu_result failed\n");
				tcps_memnode_release((void *)(g_algodev.m_pool[1]), (void *)depthnode);
				continue;
			}
			keliu_result->time[0] = box->m_time[0];
			keliu_result->time[1] = box->m_time[1];
			keliu_result->det_size = dets_new.size();
			
			for(int i=0; i<keliu_result->det_size; i++)
			{
				keliu_result->det_res[i] = dets_new[i];
			}

			sortres = mot_tracker->SortUpdate(dets_new);
			//cout << "matchedPairs:" << sortres.matchedPairs.size() << endl;
			//cout << "unmatchedDetections:" << sortres.unmatchedDetections.size() << endl;
			//cout << "unmatchedTrajectories:" << sortres.unmatchedTrajectories.size() << endl;
			vector<vector<StateType_mix>> trks_hist = sortres.trks_hist;
			vector<vector<StateType_mix>> pop_trks_hist = sortres.pop_trks_hist;
			vector<TrackingBox> frameTrackingResult = sortres.frameTrackingResult;

			keliu_result->match_size = sortres.matchedPairs.size();
			for(int k=0; k<sortres.matchedPairs.size(); k++)
			{
				keliu_result->match_point[k].x = sortres.matchedPairs[k].x;
				keliu_result->match_point[k].y = sortres.matchedPairs[k].y;
			}
			
			keliu_result->sort_size = sortres.frameTrackingResult.size();
			
			//show result
			if (sortres.frameTrackingResult.size() != 0)
			{
				for (int i = 0; i < sortres.frameTrackingResult.size(); i++)
				{
					keliu_result->sort_res[i] = sortres.frameTrackingResult[i];	
				}
			}
			if (pop_trks_hist.size())
			{
				for (int i = 0; i < pop_trks_hist.size(); i++)
				{
					trk_analysis_result trk_analysis_result;
					trk_analysis_result = trj_analysis_mix(pop_trks_hist[i], parameter_.prob_theta, parameter_.proportion, parameter_.age_thresh, parameter_.dist_thresh);
					if (trk_analysis_result.cout_add)
					{
						trk_analysis_result.trk_direction == true ? up_num++ : down_num++;
					}
					
					keliu_result->pop_res[i].pop_id = pop_trks_hist[i][0].id;
					keliu_result->pop_res[i].pop_size = pop_trks_hist[i].size();
				}
			}
			int pnum = 0;
			int sum = 0;
			if (trks_hist.size())
			{			
				for (int i = 0; i < trks_hist.size(); i++)
				{						
					if (trks_hist[i].size()) 
					{				
						pnum = (trks_hist[i].size() <= tk_points_num) ? trks_hist[i].size() : tk_points_num;	
						sum = trks_hist[i].size();					
						int kk = pnum - 1;
						for (int j = trks_hist[i].size() - 1; j >= sum - pnum; j--)
						{
							keliu_result->track_persons[i].track_point[kk].camera_id = trks_hist[i][j].camera_id;
							keliu_result->track_persons[i].track_point[kk].box = trks_hist[i][j].box;
							keliu_result->track_persons[i].track_point[kk].id = trks_hist[i][j].id;
							keliu_result->track_persons[i].track_point[kk].pro = trks_hist[i][j].pro;
							keliu_result->track_persons[i].track_point[kk].box.center = trks_hist[i][j].box.center;
							kk--;
						}
						keliu_result->track_persons[i].point_size = pnum;
					}
				}
			}
			keliu_result->track_size = trks_hist.size();
			keliu_result->up_num = up_num;
			keliu_result->down_num = down_num;
			ret = tcps_memfifo_put(&(g_algodev.m_dev[0].m_fifo[tcps_fifo_get]), (char *)keliu_result);
			if(ret <= 0)
			{
				tcps_memnode_release((void *)(g_algodev.m_pool[1]), (void *)keliu_result);
			}
			
			tcps_memnode_release((void *)(g_algodev.m_pool[1]), (void *)depthnode);
		
		}
		
		usleep(5000);
	
	}
}



int Tcps_Algo_Init()
{

	int ret = -1;
	ret = tcps_sort_init();
	if(ret != 0)
	{
		printf("tcps_sort_init failed\n");
	}
	else{
		DEBUG("tcps_sort_init successed\n");
	}

	g_algodev.m_pool[0] = tcps_mempool_init(64, 1024 * 1024 * 8);//img pool
	g_algodev.m_pool[1] = tcps_mempool_init(64, 1024*1024);//trackingbox pool

	ret = create_algo_fifo();

	if(ret == 0)
	{
		m_algofifo_flag = 1;
		DEBUG("create_algo_fifo successed\n");
	}
	else{
		printf("create_algo_fifo failed\n");
		goto exit;
	}

	
	pthread_attr_t pthread_attr, pthread_attr2, pthread_attr3;



	pthread_t tk_rgbalgo_pth;
	pthread_attr_init( &pthread_attr );
	pthread_attr_setscope( &pthread_attr, PTHREAD_SCOPE_SYSTEM );
	pthread_attr_setdetachstate( &pthread_attr, PTHREAD_CREATE_DETACHED );
	pthread_create( &tk_rgbalgo_pth, &pthread_attr, tcpsrgbalgo_hander_thread, NULL );
	pthread_attr_destroy( &pthread_attr );	

	

	pthread_t tk_depthalgo_pth;
	pthread_attr_init( &pthread_attr2 );
	pthread_attr_setscope( &pthread_attr2, PTHREAD_SCOPE_SYSTEM );
	pthread_attr_setdetachstate( &pthread_attr2, PTHREAD_CREATE_DETACHED );
	pthread_create( &tk_depthalgo_pth, &pthread_attr2, tcpsdepthalgo_hander_thread, NULL );
	pthread_attr_destroy( &pthread_attr2 );
	


/*
	//save algo result
	pthread_t tk_savealgores_pth;
	pthread_attr_init( &pthread_attr3 );
	pthread_attr_setscope( &pthread_attr3, PTHREAD_SCOPE_SYSTEM );
	pthread_attr_setdetachstate( &pthread_attr3, PTHREAD_CREATE_DETACHED );
	pthread_create( &tk_savealgores_pth, &pthread_attr3, tcpssavealgores_hander_thread, NULL);
	pthread_attr_destroy( &pthread_attr3 );
*/

	ret = 0;
exit:	
	return ret;


}


int Tcps_Algo_Put(tx2_syncframe_out *_pstInputParam)
{

	int ret = -1;

	tx2_syncframe_out *          pframe   = _pstInputParam;
	
	if(m_algofifo_flag != 1)
	{
		//usleep(msec * 1000);
		return -1;
	}
	ret = tcps_memfifo_put(&(g_algodev.m_dev[0].m_fifo[tcps_fifo_put]), (char *)pframe);
	
	return ret;

}


int draw_algores(tcps_algores 	* algo_result, tcps_algo_tracklist *poutres)
{
	int outnum = 0;
	int k = 0;
	int m = 0;
	for (int i = 0; i < algo_result->sort_size; i++)
	{
		unsigned int id = (unsigned int)algo_result->sort_res[i].id;
		
		TrackingBox tmpRest;
		tmpRest = algo_result->sort_res[i];

		unworld_union_coor(tmpRest);

		world2depth(tmpRest);

		Rect box;
		box.x = tmpRest.box.center.x - tmpRest.box.size.x / 2;
		box.y = tmpRest.box.center.y - tmpRest.box.size.y / 2;
		box.width = tmpRest.box.size.x;
		box.height = tmpRest.box.size.y;
		int rx = box.x + box.width;
		int ry = box.y + box.height;
		bool l = (box.x < rgb_height) && (box.y < rgb_width) && (box.x>0) && (box.y>0);
		bool r = (rx< rgb_height) && (ry < rgb_width) && (rx>0) && (ry>0);
		bool valid = l&&r;
		
		String prob = "prob:" + to_string(algo_result->sort_res[i].pro);
		
		if(algo_result->sort_res[i].pro >= 0.3)
		{
			
			if (algo_result->sort_res[i].camera_id == 1)
			{
				poutres[0].m_convertres.devid = 1;
				poutres[0].m_convertres.id = tmpRest.id;
				poutres[0].m_convertres.sort_res[k].box.x = box.x;
				poutres[0].m_convertres.sort_res[k].box.y = box.y;
				poutres[0].m_convertres.sort_res[k].box.width = box.width;
				poutres[0].m_convertres.sort_res[k].box.height = box.height;
				k++;	
			}
			else if (algo_result->sort_res[i].camera_id == 2)
			{
				poutres[1].m_convertres.devid = 2;
				poutres[1].m_convertres.id = tmpRest.id;
				poutres[1].m_convertres.sort_res[m].box.x = box.x;
				poutres[1].m_convertres.sort_res[m].box.y = box.y;
				poutres[1].m_convertres.sort_res[m].box.width = box.width;
				poutres[1].m_convertres.sort_res[m].box.height = box.height;
				m++;
			}
		}

	}

	poutres[0].m_convertres.sort_size = k;//sort_size = k;
	poutres[1].m_convertres.sort_size = m;

	//传输跟踪轨迹
	unsigned int id = 0;
	unsigned int pserson0 = 0;
	unsigned int pserson1 = 0;
	unsigned int p_size0 = 0;
	unsigned int p_size1 = 0;
	for (int j = 0; j < algo_result->track_size; j++)
	{
		TrackingBox tmptrack;
		p_size0 = 0;
		p_size1 = 0;
		for (int k = 0; k < algo_result->track_persons[j].point_size; k++)
		{
			id = algo_result->track_persons[j].track_point[k].camera_id;
			tmptrack = algo_result->track_persons[j].track_point[k];
			unworld_union_coor(tmptrack);
			world2depth(tmptrack);
			if (tmptrack.box.center.x < 0 || tmptrack.box.center.x >= rgb_width || 
				tmptrack.box.center.y < 0 || tmptrack.box.center.y >= rgb_height)
				//&& algo_result->track_persons[j].track_point[k].pro < 0.3))
			{
				continue;
			}

			if (1 == id)
			{
				poutres[0].m_trackperson[pserson0].m_point[p_size0].x = tmptrack.box.center.x;
				poutres[0].m_trackperson[pserson0].m_point[p_size0].y = tmptrack.box.center.y;
				p_size0++;
				poutres[0].m_trackperson[pserson0].m_psize = p_size0;
				
			}
			else if (2 == id)
			{
				poutres[1].m_trackperson[pserson1].m_point[p_size1].x = tmptrack.box.center.x;
				poutres[1].m_trackperson[pserson1].m_point[p_size1].y = tmptrack.box.center.y;
				p_size1++;
				poutres[1].m_trackperson[pserson1].m_psize = p_size1;
			}
		}
		if (poutres[0].m_trackperson[pserson0].m_psize > 0)
		{
			pserson0++;
		}
		if (poutres[1].m_trackperson[pserson1].m_psize > 0)
		{
			pserson1++;
		}
	}

	poutres[0].m_personnum = pserson0;
	poutres[1].m_personnum = pserson1;
	outnum = 2;
	return outnum;
}


int Tcps_Algo_Get(ALGO_Output_Res *poutres, int msec)
{
	int ret = -1;
	char *   depthnode = NULL;

	if(m_algofifo_flag != 1)
	{
		usleep(msec * 1000);
		return -1;
	}
	
	ret = tcps_memfifo_get(&(g_algodev.m_dev[0].m_fifo[tcps_fifo_get]), &depthnode, msec);
	
	if(ret == 0)
	{
		
		memcpy( &poutres->m_dynamicres, depthnode, sizeof(tcps_algores));
		draw_algores((tcps_algores *)depthnode, poutres->m_tracklist);
		tcps_memnode_release((void *)(g_algodev.m_pool[1]), (void *)depthnode);
		
	}
	if(ret != 0)
	{
		return -2;
	}
	
	return 0;	
}

