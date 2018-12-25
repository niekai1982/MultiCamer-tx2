
#include <opencv2/core/core.hpp>  
#include <opencv2/highgui/highgui.hpp>  
#include <opencv2/imgproc/imgproc.hpp>
#include <fcntl.h>
#include "tx2_dec.h"
#include <sys/time.h>
#include <sys/types.h>

//#include "lib_decode.h"
using namespace std;
using namespace cv;

tx2_dec_obj g_decdev[DEVS_DEC_MAX];
tx2_decs_admin g_admin;

int decid[DEVS_DEC_MAX];

int tx2_decdepth_getframe(int chnid, tx2_dec_frame *pframe)
{
	if (chnid < 0 || chnid >= CHNS_DEC_MAX || NULL == pframe)
	{
		return tfailure;
	}

	bool           res = false;
	int 		   ret = tfailure;
	int 		   cnt = 0;
	void 		  *pframe2  = g_decdev[chnid].p_dectmp[size_800x1p5];
	void 		  *pframe1  = g_decdev[chnid].p_dectmp[size_800x3];
	Dec_GetData_T  getdata;
	long long int  timestamp;
	
	getdata.dataptr = (void *)pframe2;
	getdata.data_len = dec_depthheight * dec_depthwidth * 3 / 2;
	cv::Mat nv12( dec_depthheight*3/2, dec_depthwidth, CV_8UC1, (unsigned char*)getdata.dataptr);
	cv::Mat RGB( dec_depthheight, dec_depthwidth, CV_8UC3, (unsigned char*)pframe1);
	
	//获取depth解码帧
	while (cnt < 2)
	{
		timestamp = g_decdev[chnid].p_decobj[dectype_depth]->get_decdata(getdata);
		cnt++;
		if (timestamp <= 1)
		{
			usleep(1000 * 5);
			continue;
		}
		break;
	}

	if (timestamp <= 1)
	{
		goto exit;
	}

	pframe->timestamp = timestamp;
	//I420->RGB(640*800*3)
	cv::cvtColor(nv12, RGB, CV_YUV2BGR_I420);
	//RGB(640*800*3)->depth(640*400*2)
	res = g_decdev[chnid].m_decdepth->decompressionAfterprocessing((unsigned char *)pframe1, (unsigned char *)pframe->pdata, dec_depthheight * dec_depthwidth * 3, img_depthwidth*img_depthheight*2);
	if (!res)
	{
		ret = tfailure;
		goto exit;
	}
	
	pframe->size = img_depthwidth*img_depthheight*2;
	ret = tsuccess;
	
exit:
	return ret;
}


int tx2_decrgb_getrame(int chnid, tx2_dec_frame *pframe)
{
	if (chnid < 0 || chnid >= CHNS_DEC_MAX || NULL == pframe)
	{
		return tfailure;
	}
	
	int            ret = tfailure;
	int 		   cnt = 0;
	Dec_GetData_T  getdata;
	long long int  timestamp = 0;

	void *pframe2  = g_decdev[chnid].p_dectmp[size_400x2];
	getdata.dataptr = (void *)pframe2;
	getdata.data_len = rgb_width * rgb_height * 3 / 2;

	cv::Mat nv12( rgb_height*3/2, rgb_width, CV_8UC1, (unsigned char*)getdata.dataptr);
	cv::Mat RGB( rgb_height, rgb_width, CV_8UC3, (unsigned char*)pframe->pdata);

	//获取depth深度帧
	while (cnt < 2)
	{
		timestamp = g_decdev[chnid].p_decobj[dectype_rgb]->get_decdata(getdata);
		cnt++;

		if (timestamp <= 1)
		{
			usleep(1000 * 5);
			continue;
		}

		break;
	}

	if (timestamp <= 1)
	{
		goto exit;
	}

	//I420->RGB(640*400*3)
	cv::cvtColor(nv12, RGB, CV_YUV2BGR_I420);
	pframe->size = img_rgbwidth * img_rgbheight * 3;//DEC_SIZE_400x3;
	pframe->timestamp = timestamp;

	ret = tsuccess;
	
exit:
	return ret;

}

static void *tx2_decsmix_syncthread(void *para)
{
	printf("====>%s(%d): dectid=%lun \n", __func__, __LINE__, gettid());
	pthread_detach(pthread_self());
	int ret   = tfailure;
	int len   = 0;
	int flag  = 0;
	int chnid = *((int *)para);
	tcps_frames   *pmixframe   = NULL;
	tx2_dec_frame stdecframe[3];
	tx2_dec_frame *pvalid[3] = {NULL};
	tx2_dec_frame *ptmp = NULL;
	memset(stdecframe, 0, sizeof(tx2_dec_frame) * 3);

	while (g_decdev[chnid].m_startflag)
	{
		//申请rgb帧(640 * 400 * 3)
		if (NULL == stdecframe[dectype_rgb].pdata)
		{
			stdecframe[dectype_rgb].pdata = tcps_memnode_alloc(g_decdev[chnid].p_decpool[size_400x3], &len);
			if (NULL == stdecframe[dectype_rgb].pdata)
			{
				continue;
			}
			stdecframe[dectype_rgb].size = rgb_width * rgb_height * 3;
			stdecframe[dectype_rgb].timestamp = 0;
			stdecframe[dectype_rgb].dectype = dectype_rgb;
			//pdecframe[dectype_rgb]->pdata = (void *)((char *)pdecframe[dectype_rgb] + sizeof(tx2_dec_frame));
		}

		//申请depth帧 (640 * 400 * 2)
		if (NULL == stdecframe[dectype_depth].pdata)
		{
			stdecframe[dectype_depth].pdata= tcps_memnode_alloc(g_decdev[chnid].p_decpool[size_400x2], &len);
			if (NULL == stdecframe[dectype_depth].pdata)
			{
				continue;
			}
			stdecframe[dectype_depth].size = depth_width * depth_height * 2;
			stdecframe[dectype_depth].timestamp = 0;
			stdecframe[dectype_depth].dectype = dectype_depth;
			//stdecframe[dectype_depth]->pdata = (void *)((char *)pdecframe[dectype_depth] + sizeof(tx2_dec_frame));
		}

		//获取同步帧
		if (stdecframe[dectype_mix].timestamp <= 0)
		{
			ret = tcps_memfifo_get(&g_decdev[chnid].m_decfifo[size_64k], ((char **)&(stdecframe[dectype_mix].pdata)), 400);
			if (0 != ret || NULL == stdecframe[dectype_mix].pdata)
			{
				continue;
			}
			pmixframe = (tcps_frames *)stdecframe[dectype_mix].pdata;
			stdecframe[dectype_mix].size = sizeof(tcps_frames);
			stdecframe[dectype_mix].dectype = dectype_mix;
			stdecframe[dectype_mix].timestamp = pmixframe->m_rgbimg.m_timestamp;
		}

		//获取rgb解码帧
		if (stdecframe[dectype_rgb].timestamp <= 0)
		{
			ret = tx2_decrgb_getrame(chnid, &stdecframe[dectype_rgb]);
			if (tsuccess != ret)
			{
				continue;
			}
		}

		//获取depth解码帧
		if (stdecframe[dectype_depth].timestamp <= 0)
		{
			ret = tx2_decdepth_getframe(chnid, &stdecframe[dectype_depth]);
			if (tsuccess != ret)
			{
				continue;
			}
		}

		//比较时间戳 按从大到小排序
		pvalid[0] = &stdecframe[dectype_rgb];
		pvalid[1] = &stdecframe[dectype_depth];
		pvalid[2] = &stdecframe[dectype_mix];
		for (int i = 0; i < 2; i++)
		{
			for(int j = i + 1; j < 3; j++)
			{
				if (pvalid[j]->timestamp > pvalid[i]->timestamp)
				{
					ptmp = pvalid[i];
					pvalid[i] = pvalid[j];
					pvalid[j] = ptmp;
				}
			}
		}
		flag = 1;
		for (int k = 1; k < 3; k++)
		{
			if (pvalid[k]->timestamp == pvalid[0]->timestamp)
			{
				continue;
			}
			flag = 0;
			pvalid[k]->timestamp = 0;
			if (dectype_mix == pvalid[k]->dectype)
			{
				//释放同步帧
				tcps_memnode_release(g_decdev[chnid].p_decpool[size_64k], pvalid[k]->pdata);
			}	
		}

		if (1 == flag)
		{
			pmixframe->m_rgbimg.p_data   = stdecframe[dectype_rgb].pdata;
			pmixframe->m_depthimg.p_data = stdecframe[dectype_depth].pdata;
			//推入fifo
			ret = tcps_memfifo_put(&(g_decdev[chnid].m_outfifo), (char *)pmixframe);
			stdecframe[dectype_rgb].pdata = NULL;
			stdecframe[dectype_depth].pdata = NULL;
			stdecframe[dectype_mix].timestamp = 0;
			stdecframe[dectype_rgb].timestamp = 0;
			stdecframe[dectype_depth].timestamp = 0;
			if (ret > 0)
			{
				if (0xF5 == g_admin.m_chnid[chnid])
				{
					tx2_decsuser_request(chnid);
				}
			}
		}
	}

	return NULL;
}


//创建rgb/depth混合解码器 (rgb + depth)
int tx2_decsmix_create(int chnid, tx2_dec_params *param)
{
	if (chnid < 0 || chnid >= CHNS_DEC_MAX || NULL == param)
	{
		return tfailure;
	}

	int 	ret 	= tfailure;
	LibDec* pdecobj = NULL;
	pthread_t tid[CHNS_DEC_MAX];

	decid[chnid] = chnid;
	g_decdev[chnid].m_decid   = chnid;
	g_decdev[chnid].m_dectype = param->m_dectype;
	
	//创建解码器
	g_decdev[chnid].p_decobj[dectype_rgb]= new LibDec(chnid * 2);   				//rgb解码器
	pdecobj= g_decdev[chnid].p_decobj[dectype_rgb];
	//初始化
	ret = pdecobj->init_dec( param->m_decrgbattr.m_decBuffParam, param->m_decrgbattr.m_decParam);
	if (tsuccess != ret)
	{
		printf("%s(%d): decoder init failed ret[%d]\n", __func__, __LINE__, ret);
		goto exit;
	}
	g_decdev[chnid].p_decobj[dectype_depth]= new LibDec(chnid * 2 + 1);   		   //depth解码器
	pdecobj= g_decdev[chnid].p_decobj[dectype_depth];
	//初始化
	ret = pdecobj->init_dec( param->m_decdepthattr.m_decBuffParam, param->m_decdepthattr.m_decParam);
	if (0 != ret)
	{
		printf("%s(%d): decoder init failed ret[%d]\n", __func__, __LINE__, ret);
		goto exit;
	}
	g_decdev[chnid].m_decdepth = new MoveSenseDataProcessing();

	g_decdev[chnid].p_reqblock = block_alloc(32);
	//申请fifo
	tcps_memfifo_create(&g_decdev[chnid].m_decfifo[size_400x2], 16);			    //深度fifo
	tcps_memfifo_create(&g_decdev[chnid].m_decfifo[size_400x3], 16);				//rgbfifo
	tcps_memfifo_create(&g_decdev[chnid].m_decfifo[size_64k], 16);					//同步fifo
	tcps_memfifo_create(&g_decdev[chnid].m_outfifo, 16);							//输出fifo
	//tcps_memfifo_create(&g_decdev[chnid].m_decfifo[size_tmp], 16);					//同步fifo
	//初始化解码池
	g_decdev[chnid].p_decpool[size_400x2] = tcps_mempool_init(128, DEC_SIZE_400x2);	//深度
	g_decdev[chnid].p_decpool[size_400x3] = tcps_mempool_init(128, DEC_SIZE_400x3);	//rgb
	g_decdev[chnid].p_decpool[size_64k] = tcps_mempool_init(128, DEC_SIZE_64k);		//帧
	//g_decdev[chnid].p_decpool[size_tmp] = tcps_mempool_init(128, DEC_SIZE_64k);		//帧

	g_decdev[chnid].p_dectmp[size_400x3] = (void *)malloc(DEC_SIZE_400x3);
	g_decdev[chnid].p_dectmp[size_400x2] = (void *)malloc(DEC_SIZE_400x1p5);
	g_decdev[chnid].p_dectmp[size_800x3] = (void *)malloc(DEC_SIZE_800x3);
	g_decdev[chnid].p_dectmp[size_800x1p5] = (void *)malloc(DEC_SIZE_800x3);

	//开启线程
	g_decdev[chnid].m_decid = chnid;
	g_decdev[chnid].m_startflag = 1;
	pthread_create(&tid[0], NULL, tx2_decsmix_syncthread, &decid[chnid]);			//rgb + depth 匹配

exit:
	//if (ret)
	return ret;
}


int tx2_decsmix_destroy(int chnid)
{
	int ret = tfailure;
	
	//关闭线程
	g_decdev[chnid].m_startflag = 0;
	
	//释放应答内存
	block_free(g_decdev[chnid].p_reqblock);
	
	//销毁解码类
	g_decdev[chnid].p_decobj[dectype_rgb]->~LibDec();
	g_decdev[chnid].p_decobj[dectype_depth]->~LibDec();
	
	//销毁fifo
	tcps_memfifo_destroy(&g_decdev[chnid].m_decfifo[size_400x2]);
	tcps_memfifo_destroy(&g_decdev[chnid].m_decfifo[size_400x3]);
	tcps_memfifo_destroy(&g_decdev[chnid].m_decfifo[size_64k]);
	tcps_memfifo_destroy(&g_decdev[chnid].m_outfifo);
	
	//销毁帧池
	tcps_mempool_exit(g_decdev[chnid].p_decpool[size_400x2]);
	tcps_mempool_exit(g_decdev[chnid].p_decpool[size_400x3]);
	tcps_mempool_exit(g_decdev[chnid].p_decpool[size_64k]);

	//释放临时内存
	free(g_decdev[chnid].p_dectmp[size_400x3]);
	free(g_decdev[chnid].p_dectmp[size_400x2]);
	free(g_decdev[chnid].p_dectmp[size_800x3]);
	free(g_decdev[chnid].p_dectmp[size_800x1p5]);
	
	ret = tsuccess;
	return ret;
}

int tx2_decsmix_putfrm(int chnid, tcps_frames *pframe)
{
	if ((NULL == pframe) || (chnid < 0) || (chnid >= CHNS_DEC_MAX))
	{
		return tfailure;
	}

	int ret = tfailure;
	int len = 0;
	Dec_SendData_T sendrgbdata;
	Dec_SendData_T senddepthdata;
	tcps_frames   *pmixfrm   = NULL;

	if (pframe->m_depthimg.m_framelen <=0 || pframe->m_depthimg.m_timestamp <= 0)
	{
		goto exit;
	}
	
	//申请同步帧
	pmixfrm = (tcps_frames *)tcps_memnode_alloc(g_decdev[chnid].p_decpool[size_64k], &len);
	if (NULL == pmixfrm)
	{
		goto exit;
	}
	
	memcpy((void *)pmixfrm, (void *)pframe, sizeof(tcps_frames));

	//解rgb
	sendrgbdata.timestamp = pframe->m_rgbimg.m_timestamp;
	sendrgbdata.frm_type = 1;
	sendrgbdata.data_len = pframe->m_rgbimg.m_framelen;
	sendrgbdata.dataptr  = pframe->m_rgbimg.p_data;
	ret = g_decdev[chnid].p_decobj[dectype_rgb]->send_encdata(sendrgbdata);

	//解depth
	senddepthdata.timestamp = pframe->m_depthimg.m_timestamp;
	senddepthdata.frm_type = 1;
	senddepthdata.data_len = pframe->m_depthimg.m_framelen;
	senddepthdata.dataptr  = pframe->m_depthimg.p_data;

	ret = g_decdev[chnid].p_decobj[dectype_depth]->send_encdata(senddepthdata);
	if (0 == ret)
	{
		ret = tcps_memfifo_put(&g_decdev[chnid].m_decfifo[size_64k], (char *)pmixfrm);
		if (ret < 0)
		{
			//释放帧
			tcps_memnode_release(g_decdev[chnid].p_decpool[size_64k], (void *)pmixfrm);
			ret = tfailure;
			goto exit;
		}
	}
	
	ret = tsuccess;
exit:
	return ret;
}

int tx2_decsmix_getfrm(int chnid, tcps_frames **pframe, int msec)
{
	if (chnid < 0 || chnid >= CHNS_DEC_MAX || NULL == pframe)
	{
		return tfailure;
	}

	int          ret 	 = tfailure;
	tcps_frames *pframe0 = NULL;

	//获取帧
	ret = tcps_memfifo_get(&(g_decdev[chnid].m_outfifo), (char **)&pframe0, msec);
	if (0 != ret)
	{
		goto exit;
	}

	ret 	= tsuccess;
	*pframe = pframe0;
	
exit:
	return ret;
}

int tx2_decsmix_release(int chnid, tcps_frames *pframe)
{
	if (chnid < 0 || chnid >= CHNS_DEC_MAX || NULL == pframe)
	{
		return tfailure;
	}

	if (NULL != pframe->m_rgbimg.p_data)
	{
		tcps_memnode_release(g_decdev[chnid].p_decpool[size_400x3], pframe->m_rgbimg.p_data);
	}
	if (NULL != pframe->m_depthimg.p_data)
	{
		tcps_memnode_release(g_decdev[chnid].p_decpool[size_400x2], pframe->m_depthimg.p_data);
	}
	
	tcps_memnode_release(g_decdev[chnid].p_decpool[size_64k], pframe);

	return tsuccess;
}

static void *tx2_decsrgb_syncthread(void *para)
{
	pthread_detach(pthread_self());
	printf("====>%s(%d): dectid=%lun \n", __func__, __LINE__, gettid());
	int ret   = tfailure;
	int len   = 0;
	int flag  = 0;
	int chnid = *((int *)para);
	tcps_frames   *pmixframe   = NULL;
	tx2_dec_frame stdecframe[3];
	tx2_dec_frame *pvalid[3] = {NULL};
	tx2_dec_frame *ptmp = NULL;
	memset(stdecframe, 0, sizeof(tx2_dec_frame) * 3);
	
	while (g_decdev[chnid].m_startflag)
	{
		//申请rgb帧(640 * 400 * 3)
		if (NULL == stdecframe[dectype_rgb].pdata)
		{
			stdecframe[dectype_rgb].pdata = tcps_memnode_alloc(g_decdev[chnid].p_decpool[size_400x3], &len);
			if (NULL == stdecframe[dectype_rgb].pdata)
			{
				continue;
			}
			stdecframe[dectype_rgb].size = rgb_width * rgb_height * 3;
			stdecframe[dectype_rgb].timestamp = 0;
			stdecframe[dectype_rgb].dectype = dectype_rgb;
			//pdecframe[dectype_rgb]->pdata = (void *)((char *)pdecframe[dectype_rgb] + sizeof(tx2_dec_frame));
		}

		//获取同步帧
		if (stdecframe[dectype_mix].timestamp <= 0)
		{
			ret = tcps_memfifo_get(&g_decdev[chnid].m_decfifo[size_64k], ((char **)&(stdecframe[dectype_mix].pdata)), 400);
			if (0 != ret || NULL == stdecframe[dectype_mix].pdata)
			{
				continue;
			}
			pmixframe = (tcps_frames *)stdecframe[dectype_mix].pdata;
			stdecframe[dectype_mix].size = sizeof(tcps_frames);
			stdecframe[dectype_mix].dectype = dectype_mix;
			stdecframe[dectype_mix].timestamp = pmixframe->m_rgbimg.m_timestamp;
		}

		//获取rgb解码帧
		if (stdecframe[dectype_rgb].timestamp <= 0)
		{
			ret = tx2_decrgb_getrame(chnid, &stdecframe[dectype_rgb]);
			if (tsuccess != ret)
			{
				continue;
			}
		}

		//比较时间戳 按从大到小排序
		pvalid[0] = &stdecframe[dectype_rgb];
		pvalid[1] = &stdecframe[dectype_mix];
		if (pvalid[0]->timestamp < pvalid[1]->timestamp)
		{
			ptmp = pvalid[0];
			pvalid[0] = pvalid[1];
			pvalid[1] = ptmp;
		}

		flag = 1;
		for (int k = 1; k < 2; k++)
		{
			if (pvalid[k]->timestamp == pvalid[0]->timestamp)
			{
				continue;
			}

			flag = 0;
			pvalid[k]->timestamp = 0;
			if (dectype_mix == pvalid[k]->dectype)
			{
				//释放同步帧
				tcps_memnode_release(g_decdev[chnid].p_decpool[size_64k], pvalid[k]->pdata);
			}	
		}

		if (1 == flag)
		{
			pmixframe->m_rgbimg.p_data   = stdecframe[dectype_rgb].pdata;
			//推入fifo
			ret = tcps_memfifo_put(&(g_decdev[chnid].m_outfifo), (char *)pmixframe);
			stdecframe[dectype_rgb].pdata = NULL;
			stdecframe[dectype_mix].timestamp = 0;
			stdecframe[dectype_rgb].timestamp = 0;
			if (ret > 0)
			{
				if (0xF5 == g_admin.m_chnid[chnid])
				{
					tx2_decsuser_request(chnid);
				}
			}
		}
	}

	return NULL;
}

int tx2_decsrgb_create(int chnid, tx2_dec_params *param)
{
	if (chnid < 0 || chnid >= CHNS_DEC_MAX || NULL == param)
	{
		return tfailure;
	}

	int 	ret 	= tfailure;
	LibDec* pdecobj = NULL;
	pthread_t tid[CHNS_DEC_MAX];

	decid[chnid] = chnid;
	
	g_decdev[chnid].p_reqblock = block_alloc(32);
	g_decdev[chnid].m_decid   = chnid;
	g_decdev[chnid].m_dectype = param->m_dectype;

	//创建解码器
	g_decdev[chnid].p_decobj[dectype_rgb]= new LibDec(chnid * 2);   //rgb解码器
	pdecobj= g_decdev[chnid].p_decobj[dectype_rgb];

	//初始化
	ret = pdecobj->init_dec( param->m_decrgbattr.m_decBuffParam, param->m_decrgbattr.m_decParam);

	//申请fifo
	tcps_memfifo_create(&g_decdev[chnid].m_decfifo[size_400x3], 128);				    //rgbfifo
	tcps_memfifo_create(&g_decdev[chnid].m_decfifo[size_64k], 128);					    //同步fifo
	tcps_memfifo_create(&g_decdev[chnid].m_outfifo, 128);								//输出fifo

	//初始化解码池
	g_decdev[chnid].p_decpool[size_400x3] = tcps_mempool_init(128, DEC_SIZE_400x3);	//rgb
	g_decdev[chnid].p_decpool[size_64k] = tcps_mempool_init(128, DEC_SIZE_64k);		//帧

	g_decdev[chnid].p_dectmp[size_400x3] = (void *)malloc(DEC_SIZE_400x3);
	g_decdev[chnid].p_dectmp[size_400x2] = (void *)malloc(DEC_SIZE_400x1p5);
	g_decdev[chnid].p_dectmp[size_800x3] = (void *)malloc(DEC_SIZE_800x3);
	g_decdev[chnid].p_dectmp[size_800x1p5] = (void *)malloc(DEC_SIZE_800x3);

	//开启线程
	g_decdev[chnid].m_decid = chnid;
	g_decdev[chnid].m_startflag = 1;
	pthread_create(&tid[0], NULL, tx2_decsrgb_syncthread, &decid[chnid]);	//rgb + depth 匹配

	return ret;
}

int tx2_decsrgb_destroy(int chnid)
{
	int ret = tfailure;
	
	//关闭线程
	g_decdev[chnid].m_startflag = 0;
	
	//释放应答内存
	block_free(g_decdev[chnid].p_reqblock);
	
	//销毁解码类
	g_decdev[chnid].p_decobj[dectype_rgb]->~LibDec();
	
	//销毁fifo
	tcps_memfifo_destroy(&g_decdev[chnid].m_decfifo[size_400x3]);
	tcps_memfifo_destroy(&g_decdev[chnid].m_decfifo[size_64k]);
	tcps_memfifo_destroy(&g_decdev[chnid].m_outfifo);
	
	//销毁帧池
	tcps_mempool_exit(g_decdev[chnid].p_decpool[size_400x3]);
	tcps_mempool_exit(g_decdev[chnid].p_decpool[size_64k]);

	//释放临时内存
	free(g_decdev[chnid].p_dectmp[size_400x3]);
	free(g_decdev[chnid].p_dectmp[size_400x2]);
	free(g_decdev[chnid].p_dectmp[size_800x3]);
	free(g_decdev[chnid].p_dectmp[size_800x1p5]);
	
	ret = tsuccess;
	return ret;
}

int tx2_decsrgb_putfrm(int chnid, tcps_frames *pframe)
{
	if ((NULL == pframe) || (chnid < 0) || (chnid >= CHNS_DEC_MAX))
	{
		return tfailure;
	}

	int ret = tfailure;
	int len = 0;
	Dec_SendData_T sendrgbdata;
	tcps_frames   *pmixfrm   = NULL;

	//申请同步帧
	pmixfrm = (tcps_frames *)tcps_memnode_alloc(g_decdev[chnid].p_decpool[size_64k], &len);
	if (NULL == pmixfrm)
	{
		goto exit;
	}

	memcpy((void *)pmixfrm, (void *)pframe, sizeof(tcps_frames));

	//推入fifo
	ret = tcps_memfifo_put(&g_decdev[chnid].m_decfifo[size_64k], (char *)pmixfrm);
	if (ret < 0)
	{
		//释放帧
		tcps_memnode_release(g_decdev[chnid].p_decpool[size_64k], (void *)pmixfrm);
		ret = tfailure;
		goto exit;
	}

	//解rgb
	sendrgbdata.timestamp = pframe->m_rgbimg.m_timestamp;
	sendrgbdata.frm_type = 1;
	sendrgbdata.data_len = pframe->m_rgbimg.m_framelen;
	sendrgbdata.dataptr  = pframe->m_rgbimg.p_data;

	ret = g_decdev[chnid].p_decobj[dectype_rgb]->send_encdata(sendrgbdata);
	ret = tsuccess;
	
exit:
	return ret;
}

int tx2_decsrgb_getfrm(int chnid, tcps_frames **pframe, int msec)
{
	if (chnid < 0 || chnid >= CHNS_DEC_MAX || NULL == pframe)
	{
		return tfailure;
	}

	int          ret 	 = tfailure;
	tcps_frames *pframe0 = NULL;

	//获取帧
	ret = tcps_memfifo_get(&(g_decdev[chnid].m_outfifo), (char **)&pframe0, msec);
	if (0 != ret)
	{
		//printf("%s(%d): tcps_memfifo_get error\n", __func__, __LINE__);
		goto exit;
	}

	ret 	= tsuccess;
	*pframe = pframe0;
	
exit:
	return ret;
}

int tx2_decrgb_release(int chnid, tcps_frames *pframe)
{
	if (chnid < 0 || chnid >= CHNS_DEC_MAX || NULL == pframe)
	{
		return tfailure;
	}

	if (NULL != pframe->m_rgbimg.p_data)
	{
		tcps_memnode_release(g_decdev[chnid].p_decpool[size_400x3], pframe->m_rgbimg.p_data);
	}

	tcps_memnode_release(g_decdev[chnid].p_decpool[size_64k], pframe);

	return tsuccess;
}

int tx2_decsdev_create(int chnid, tx2_dec_type dectype, tx2_dec_params *param)
{
	if (chnid < 0 || chnid >= CHNS_DEC_MAX || NULL == param)
	{
		return tfailure;
	}
	
	int ret = tfailure;
	
	if (dectype_mix == dectype)
	{
		ret = tx2_decsmix_create(chnid, param);
	}
	else if (dectype_rgb == dectype)
	{
		ret = tx2_decsrgb_create(chnid, param);
	}

	return ret;
}

int tx2_decsdev_destroy(int chnid, tcps_frames *pframe)
{
	if (chnid < 0 || chnid >= CHNS_DEC_MAX || NULL == pframe)
	{
		return tfailure;
	}
	
	int ret = tfailure;
	
	if (dectype_mix == g_decdev[chnid].m_dectype)
	{
		ret = tx2_decsmix_destroy(chnid);
	}
	else if (dectype_rgb == g_decdev[chnid].m_dectype)
	{
		ret = tx2_decsrgb_destroy(chnid);
	}

	return ret;
}

int tx2_decsdev_putfrm(int chnid, tcps_frames *pframe)
{
	if (chnid < 0 || chnid >= CHNS_DEC_MAX || NULL == pframe)
	{
		return tfailure;
	}

	int ret = tfailure;
	
	if (dectype_mix == g_decdev[chnid].m_dectype)
	{
		ret = tx2_decsmix_putfrm(chnid, pframe);
	}
	else if (dectype_rgb == g_decdev[chnid].m_dectype)
	{
		ret = tx2_decsrgb_putfrm(chnid, pframe);
	}

	return ret;
}

int tx2_decsdev_getfrm(int chnid, tcps_frames **pframe, int msec)
{
	if (chnid < 0 || chnid >= CHNS_DEC_MAX || NULL == pframe)
	{
		return tfailure;
	}

	int ret = tfailure;
	
	if (dectype_mix == g_decdev[chnid].m_dectype)
	{
		ret = tx2_decsmix_getfrm(chnid, pframe, msec);
	}
	else if (dectype_rgb == g_decdev[chnid].m_dectype)
	{
		ret = tx2_decsrgb_getfrm(chnid, pframe, msec);
	}
	
	return ret;
}

int tx2_decsdev_release(int chnid, tcps_frames *pframe)
{
	if (chnid < 0 || chnid >= CHNS_DEC_MAX || NULL == pframe)
	{
		return tfailure;
	}

	int ret = tfailure;
	
	if (dectype_mix == g_decdev[chnid].m_dectype)
	{
		ret = tx2_decsmix_release(chnid, pframe);
	}
	else if (dectype_rgb == g_decdev[chnid].m_dectype)
	{
		ret = tx2_decrgb_release(chnid, pframe);
	}

	return ret;
}

int tx2_decsadmin_init(int chns, int *chnid)
{
	if (chns < 0 || chns > CHNS_DEC_MAX || NULL == chnid)
	{
		return tfailure;
	}
	
	int ret = tfailure;
	int ichnid = 0;
	g_admin.m_chns = chns;
	for (int i = 0; i < chns; i++)
	{
		ichnid = chnid[i];
		g_admin.m_chnid[ichnid] = 0xF5;
	}
	g_admin.p_resblock = block_alloc(32);
	ret = kfifo_alloc(&g_admin.m_fifo, 1, 32);

	return ret;
}

int tx2_decsuser_request(int chnid)
{
	g_decdev[chnid].p_reqblock->m_datasize = sizeof(int);
	memcpy(g_decdev[chnid].p_reqblock->data,&chnid, sizeof(int));
	kfifo_put(&g_admin.m_fifo, g_decdev[chnid].p_reqblock);

	return tsuccess;
}

int tx2_decsadmin_respond(int msec)
{
	int  chnid = -1;
	int ret = kfifo_get(&g_admin.m_fifo, g_admin.p_resblock, msec);
	if (0 != ret)
	{
		chnid = -1;
		goto exit;
	}

	memcpy(&chnid, g_admin.p_resblock->data, g_admin.p_resblock->m_datasize);
	
exit:
	return chnid;
}
