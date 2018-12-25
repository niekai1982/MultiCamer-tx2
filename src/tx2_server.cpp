
#include <stdio.h>
#include <string.h>

#include <opencv2/core/core.hpp>  
#include <opencv2/highgui/highgui.hpp>  
#include <opencv2/imgproc/imgproc.hpp>

#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>

#include "tcps_common.h"
#include "tcps_queue.h"
#include "tx2_server.h"

#include <stdlib.h>
#include <unistd.h>
#include "tx2_dec.h"
#include "lib_decode.h"
#include "tcps_algo_interface.h"
#include "client.h"

#define TCPS_FRAMERATE					(40)
#define src_num 						(tk_inputcamera_number)
#define H264_KELIU_ENCPTHNUM			(tk_inputcamera_number)

using namespace std;
using namespace cv;

int dev_list[CLIENT_SUM] = {-1, -1, -1, -1};
unsigned short server_port[CLIENT_SUM] = {8888, 8889, 8890, 8891};
tx2_client_list g_clientlist;
tx2_virtdev_obj g_virtdevlist[CLIENT_SUM];
int virtdev_init = -1;
int algo_init = 0;
tx2_save_obj g_savelist[CLIENT_SUM];
int saveid[CLIENT_SUM];
tx2_display_obj g_displaylist[CLIENT_SUM];

char *tmp_sort = (char *)malloc(128 * 32);


int socket_Select(int hSocket,int nTimeOut,int bRead)
{
	fd_set fdset;
	timeval tv;
	FD_ZERO(&fdset);
	FD_SET(hSocket,&fdset);
	//nTimeOut = nTimeOut > 1000 ? 1000 : nTimeOut;
	tv.tv_sec = 5;
	tv.tv_usec = nTimeOut;
	int maxsock = hSocket;
	int iRet = 0;
	if (bRead)
	{
		iRet = select(maxsock + 1,&fdset,NULL,NULL,&tv);
	} 
	else
	{
		iRet = select(maxsock + 1,NULL,&fdset,NULL,&tv);
	}
	if (iRet <= 0)
	{
		return FALSE;
	} 
	else if (FD_ISSET(hSocket,&fdset))
	{
		return TRUE;
	}
	return FALSE;
}

static void *tx2_clientin_thread(void *para)
{
	char buff[1024 * 128];
	client_item *pclient = (client_item *)para; 
	printf("====>%s(%d): dectid=%lun \n", __func__, __LINE__, gettid());
	while (pclient->m_run)
	{	
		if (!socket_Select(pclient->m_socket,100,TRUE))
		{
			continue;
		}	

		int num = recv(pclient->m_socket,buff,sizeof(buff),0);
		if (num <= 0)
		{
			continue;
		}

		tcps_queue_put(&(pclient->m_queue), buff, num);
	}
	
	return NULL;
}

static void *tx2_clientout_thread(void *para)
{
	printf("====>%s(%d): dectid=%lun \n", __func__, __LINE__, gettid());
	client_item *pclient = (client_item *)para;
	int            ret      = tfailure;
	tcps_frames   *tmpbuf 	= NULL;
	tcps_frames   *tmpbuf1 	= NULL;
	char          frmbuf[1024 * 128 * 10];
	char          *buff;
	
	unsigned char  usDevID = 0;
	unsigned int uiFrameLen = 0;  //帧长度
	unsigned short usDataLen = 0; //数据长度
	unsigned int   resultlen = 0; //算法帧长度

	int size = 0;
	int bufsize = 0;
	int step = 2;
	int cnt  = 0;
	int dcnt = 0;
	int dstep = 2;
	char revbuf[1024];
	int det_begin_index = 0;
	int flag = 0;

	while (1)
	{
		size = tcps_queue_get(&(pclient->m_queue), revbuf + cnt, step, 1000);
		step = step - size;
		cnt  = cnt + size;
		if (step > 0)
		{
			continue;
		}

		//寻找帧头
		if (!((revbuf[0] == 0x7E) && (revbuf[1] == 0xE7)))
		{
			revbuf[0] = revbuf[1];
			cnt = 1;
			step = 1;
			continue;
		}
		else if (cnt < 15)
		{
			step = 13;
			cnt = 2;
			continue;
		}
		
		usDevID = revbuf[2];
		
		if (-1 == dev_list[usDevID])
		{
			dev_list[usDevID] = pclient->m_id;
		}
		
		usDataLen = revbuf[13]<<8;
		usDataLen += revbuf[14];
		uiFrameLen = revbuf[5]<<24;
		uiFrameLen += revbuf[6]<<16;
		uiFrameLen += revbuf[7]<<8;
		uiFrameLen += revbuf[8];
		//usDataLen = uiFrameLen;
		//申请接收内存
		flag = 0;
		tmpbuf = (tcps_frames *)tcps_memnode_alloc(pclient->m_pool[type_dec], &bufsize);
		if (NULL == tmpbuf)
		{
			tmpbuf = (tcps_frames *)frmbuf;
			flag = 1;
		}
		tmpbuf->m_devid = usDevID;
		//获取帧数据
		step = 2;
		cnt = 0;
		dstep = uiFrameLen;
		dcnt = 0;
		while (1)
		{
			size = tcps_queue_get(&(pclient->m_queue), (char *)tmpbuf + sizeof(tcps_frames)+ dcnt, dstep, 1000);
			dcnt = dcnt +size;
			dstep = dstep -size;
			if (dstep > 0)
			{
				continue;
			}
			break;
		}
		if (1 == flag)
		{
			continue;
		}
		
		buff = (char *)tmpbuf + sizeof(tcps_frames);
		//解析帧包
		resultlen = buff[0]<<24;
		resultlen += buff[1]<<16;
		resultlen += buff[2]<<8;
		resultlen += buff[3];
					
		tmpbuf->m_rgbimg.m_frmid = buff[4]<<24;
		tmpbuf->m_rgbimg.m_frmid += buff[5]<<16;
		tmpbuf->m_rgbimg.m_frmid += buff[6]<<8;
		tmpbuf->m_rgbimg.m_frmid += buff[7];
		
		tmpbuf->m_rgbimg.m_timestamp = ((long long int)buff[8])<<56;
		tmpbuf->m_rgbimg.m_timestamp += ((long long int)buff[9])<<48;
		tmpbuf->m_rgbimg.m_timestamp += ((long long int)buff[10])<<40;
		tmpbuf->m_rgbimg.m_timestamp += ((long long int)buff[11])<<32;
		tmpbuf->m_rgbimg.m_timestamp += ((long long int)buff[12])<<24;
		tmpbuf->m_rgbimg.m_timestamp += buff[13]<<16;
		tmpbuf->m_rgbimg.m_timestamp += buff[14]<<8;
		tmpbuf->m_rgbimg.m_timestamp += buff[15];
		tmpbuf->m_rgbimg.m_imgwidth = buff[16]<<8;
		tmpbuf->m_rgbimg.m_imgwidth += buff[17];
		
		tmpbuf->m_rgbimg.m_imgheight = buff[18]<<8;
		tmpbuf->m_rgbimg.m_imgheight += buff[19];
		
		tmpbuf->m_rgbimg.m_pixelwidth = buff[20];
		
		tmpbuf->m_rgbimg.m_framelen   = buff[21]<<24;
		tmpbuf->m_rgbimg.m_framelen   += buff[22]<<16;
		tmpbuf->m_rgbimg.m_framelen   += buff[23]<<8;
		tmpbuf->m_rgbimg.m_framelen   += buff[24];

		tmpbuf->m_depthimg.m_timestamp = tmpbuf->m_rgbimg.m_timestamp;
		tmpbuf->m_depthimg.m_imgwidth = buff[25]<<8;
		tmpbuf->m_depthimg.m_imgwidth += buff[26];
		
		tmpbuf->m_depthimg.m_imgheight = buff[27]<<8;
		tmpbuf->m_depthimg.m_imgheight += buff[28];
		
		tmpbuf->m_depthimg.m_pixelwidth = buff[29];
		
		tmpbuf->m_depthimg.m_framelen   = buff[30]<<24;
		tmpbuf->m_depthimg.m_framelen   += buff[31]<<16;
		tmpbuf->m_depthimg.m_framelen   += buff[32]<<8;
		tmpbuf->m_depthimg.m_framelen   += buff[33];

		det_begin_index = 34;

		tmpbuf->m_algopred.keliuresult.devid = tmpbuf->m_devid;
		tmpbuf->m_algopred.keliuresult.frameid = tmpbuf->m_rgbimg.m_frmid;
		tmpbuf->m_algopred.keliuresult.time = tmpbuf->m_rgbimg.m_timestamp;
		
		tmpbuf->m_algopred.keliuresult.up_num = buff[det_begin_index++]<<24;
		tmpbuf->m_algopred.keliuresult.up_num += buff[det_begin_index++]<<16;
		tmpbuf->m_algopred.keliuresult.up_num += buff[det_begin_index++]<<8;
		tmpbuf->m_algopred.keliuresult.up_num += buff[det_begin_index++];
		
		tmpbuf->m_algopred.keliuresult.down_num = buff[det_begin_index++]<<24;
		tmpbuf->m_algopred.keliuresult.down_num += buff[det_begin_index++]<<16;
		tmpbuf->m_algopred.keliuresult.down_num += buff[det_begin_index++]<<8;
		tmpbuf->m_algopred.keliuresult.down_num += buff[det_begin_index++];
		
		//det
		int depth_x = 0;
		int depth_y = 0;
		int depth_z = 0;
		int size_x = 0;
		int size_y = 0;
		int size_z = 0;
		tmpbuf->m_algopred.keliuresult.det_size = buff[det_begin_index++]<<8;
		tmpbuf->m_algopred.keliuresult.det_size += buff[det_begin_index++];
		for(int i=0; i<tmpbuf->m_algopred.keliuresult.det_size; i++)
		{
			
			tmpbuf->m_algopred.keliuresult.det_res[i].det_rec.x = buff[(det_begin_index++)]<<24;
			tmpbuf->m_algopred.keliuresult.det_res[i].det_rec.x += buff[(det_begin_index++)]<<16;
			tmpbuf->m_algopred.keliuresult.det_res[i].det_rec.x += buff[(det_begin_index++)]<<8;
			tmpbuf->m_algopred.keliuresult.det_res[i].det_rec.x += buff[(det_begin_index++)];

			tmpbuf->m_algopred.keliuresult.det_res[i].det_rec.y = buff[(det_begin_index++)]<<24;
			tmpbuf->m_algopred.keliuresult.det_res[i].det_rec.y += buff[(det_begin_index++)]<<16;
			tmpbuf->m_algopred.keliuresult.det_res[i].det_rec.y += buff[(det_begin_index++)]<<8;
			tmpbuf->m_algopred.keliuresult.det_res[i].det_rec.y += buff[(det_begin_index++)];

			tmpbuf->m_algopred.keliuresult.det_res[i].det_rec.width = buff[(det_begin_index++)]<<24;
			tmpbuf->m_algopred.keliuresult.det_res[i].det_rec.width += buff[(det_begin_index++)]<<16;
			tmpbuf->m_algopred.keliuresult.det_res[i].det_rec.width += buff[(det_begin_index++)]<<8;
			tmpbuf->m_algopred.keliuresult.det_res[i].det_rec.width += buff[(det_begin_index++)];

			tmpbuf->m_algopred.keliuresult.det_res[i].det_rec.height = buff[(det_begin_index++)]<<24;
			tmpbuf->m_algopred.keliuresult.det_res[i].det_rec.height += buff[(det_begin_index++)]<<16;
			tmpbuf->m_algopred.keliuresult.det_res[i].det_rec.height += buff[(det_begin_index++)]<<8;
			tmpbuf->m_algopred.keliuresult.det_res[i].det_rec.height += buff[(det_begin_index++)];

			depth_x = buff[(det_begin_index++)]<<24;
			DEBUG("buff[%d] = %x\n", det_begin_index, buff[(det_begin_index)]);
			depth_x |= buff[(det_begin_index++)]<<16;
			DEBUG("buff[%d] = %x\n", det_begin_index, buff[(det_begin_index)]);
			depth_x |= buff[(det_begin_index++)]<<8;
			DEBUG("buff[%d] = %x\n", det_begin_index, buff[(det_begin_index)]);
			depth_x |= buff[(det_begin_index++)];
			DEBUG("buff[%d] = %x\n", det_begin_index, buff[(det_begin_index)]);
			DEBUG("depth_x = %d\n", depth_x);
			tmpbuf->m_algopred.keliuresult.det_res[i].depth_x = (float)depth_x/10000;
			depth_y = buff[(det_begin_index++)]<<24;
			depth_y |= buff[(det_begin_index++)]<<16;
			depth_y |= buff[(det_begin_index++)]<<8;
			depth_y |= buff[(det_begin_index++)];
			tmpbuf->m_algopred.keliuresult.det_res[i].depth_y = (float)depth_y/10000;
			depth_z = buff[(det_begin_index++)]<<24;
			depth_z |= buff[(det_begin_index++)]<<16;
			depth_z |= buff[(det_begin_index++)]<<8;
			depth_z |= buff[(det_begin_index++)];
			tmpbuf->m_algopred.keliuresult.det_res[i].depth_z = (float)depth_z/10000;
/*			
			size_x = buff[(det_begin_index++)]<<24;
			size_x += buff[(det_begin_index++)]<<16;
			size_x += buff[(det_begin_index++)]<<8;
			size_x += buff[(det_begin_index++)];
			tmpbuf->m_algopred.keliuresult.det_res[i].size_x = (float)size_x/10000;

			size_y = buff[(det_begin_index++)]<<24;
			size_y += buff[(det_begin_index++)]<<16;
			size_y += buff[(det_begin_index++)]<<8;
			size_y += buff[(det_begin_index++)];
			tmpbuf->m_algopred.keliuresult.det_res[i].size_y = (float)size_y/10000;

			size_z = buff[(det_begin_index++)]<<24;
			size_z += buff[(det_begin_index++)]<<16;
			size_z += buff[(det_begin_index++)]<<8;
			size_z += buff[(det_begin_index++)];
			tmpbuf->m_algopred.keliuresult.det_res[i].size_z = (float)size_z/10000;

			z_value = buff[(det_begin_index++)]<<24;
			z_value += buff[(det_begin_index++)]<<16;
			z_value += buff[(det_begin_index++)]<<8;
			z_value += buff[(det_begin_index++)];
			//printf("pro = %d\n", pro);
			tmpbuf->m_algopred.keliuresult.det_res[i].z_value = (float)z_value/10000;

			DEBUG("det: z_value = %f\n", tmpbuf->m_algopred.keliuresult.det_res[i].z_value);
*/
			DEBUG("det:   x = %f, y = %f, w = %f, h = %f, depth_x = %f, depth_y = %f, depth_z = %f\n",
				tmpbuf->m_algopred.keliuresult.det_res[i].det_rec.x,
				tmpbuf->m_algopred.keliuresult.det_res[i].det_rec.y,
				tmpbuf->m_algopred.keliuresult.det_res[i].det_rec.width,
				tmpbuf->m_algopred.keliuresult.det_res[i].det_rec.height,
				tmpbuf->m_algopred.keliuresult.det_res[i].depth_x,
				tmpbuf->m_algopred.keliuresult.det_res[i].depth_y,
				tmpbuf->m_algopred.keliuresult.det_res[i].depth_z);

		}

		//match
		tmpbuf->m_algopred.keliuresult.match_size = buff[det_begin_index++]<<8;
		tmpbuf->m_algopred.keliuresult.match_size += buff[det_begin_index++];
		for(int i=0; i<tmpbuf->m_algopred.keliuresult.match_size; i++)
		{
			tmpbuf->m_algopred.keliuresult.match_point[i].x = buff[(det_begin_index++)]<<24;
			tmpbuf->m_algopred.keliuresult.match_point[i].x += buff[(det_begin_index++)]<<16;
			tmpbuf->m_algopred.keliuresult.match_point[i].x += buff[(det_begin_index++)]<<8;
			tmpbuf->m_algopred.keliuresult.match_point[i].x += buff[(det_begin_index++)];

			tmpbuf->m_algopred.keliuresult.match_point[i].y = buff[(det_begin_index++)]<<24;
			tmpbuf->m_algopred.keliuresult.match_point[i].y += buff[(det_begin_index++)]<<16;
			tmpbuf->m_algopred.keliuresult.match_point[i].y += buff[(det_begin_index++)]<<8;
			tmpbuf->m_algopred.keliuresult.match_point[i].y += buff[(det_begin_index++)];			
		}

		//sort
		tmpbuf->m_algopred.keliuresult.sort_size = buff[det_begin_index++]<<8;
		tmpbuf->m_algopred.keliuresult.sort_size += buff[det_begin_index++];
		int pro= 0;
		for(int i=0; i<tmpbuf->m_algopred.keliuresult.sort_size; i++)
		{
			
			tmpbuf->m_algopred.keliuresult.sort_res[i].id = buff[(det_begin_index++)]<<24;
			tmpbuf->m_algopred.keliuresult.sort_res[i].id += buff[(det_begin_index++)]<<16;
			tmpbuf->m_algopred.keliuresult.sort_res[i].id += buff[(det_begin_index++)]<<8;
			tmpbuf->m_algopred.keliuresult.sort_res[i].id += buff[(det_begin_index++)];
			
			tmpbuf->m_algopred.keliuresult.sort_res[i].box.x = buff[(det_begin_index++)]<<24;
			tmpbuf->m_algopred.keliuresult.sort_res[i].box.x += buff[(det_begin_index++)]<<16;
			tmpbuf->m_algopred.keliuresult.sort_res[i].box.x += buff[(det_begin_index++)]<<8;
			tmpbuf->m_algopred.keliuresult.sort_res[i].box.x += buff[(det_begin_index++)];

			tmpbuf->m_algopred.keliuresult.sort_res[i].box.y = buff[(det_begin_index++)]<<24;
			tmpbuf->m_algopred.keliuresult.sort_res[i].box.y += buff[(det_begin_index++)]<<16;
			tmpbuf->m_algopred.keliuresult.sort_res[i].box.y += buff[(det_begin_index++)]<<8;
			tmpbuf->m_algopred.keliuresult.sort_res[i].box.y += buff[(det_begin_index++)];

			tmpbuf->m_algopred.keliuresult.sort_res[i].box.width = buff[(det_begin_index++)]<<24;
			tmpbuf->m_algopred.keliuresult.sort_res[i].box.width += buff[(det_begin_index++)]<<16;
			tmpbuf->m_algopred.keliuresult.sort_res[i].box.width += buff[(det_begin_index++)]<<8;
			tmpbuf->m_algopred.keliuresult.sort_res[i].box.width += buff[(det_begin_index++)];

			tmpbuf->m_algopred.keliuresult.sort_res[i].box.height = buff[(det_begin_index++)]<<24;
			tmpbuf->m_algopred.keliuresult.sort_res[i].box.height += buff[(det_begin_index++)]<<16;
			tmpbuf->m_algopred.keliuresult.sort_res[i].box.height += buff[(det_begin_index++)]<<8;
			tmpbuf->m_algopred.keliuresult.sort_res[i].box.height += buff[(det_begin_index++)];

			
			pro = buff[(det_begin_index++)]<<24;
			pro |= buff[(det_begin_index++)]<<16;
			pro |= buff[(det_begin_index++)]<<8;
			pro |= buff[(det_begin_index++)];
			tmpbuf->m_algopred.keliuresult.sort_res[i].pro = (float)pro/10000;
		}

		//pop
		tmpbuf->m_algopred.keliuresult.pop_size = buff[det_begin_index++]<<8;
		tmpbuf->m_algopred.keliuresult.pop_size += buff[det_begin_index++];
		for(int i=0; i<tmpbuf->m_algopred.keliuresult.pop_size; i++)
		{
			//pop id 
			tmpbuf->m_algopred.keliuresult.pop_res[i].pop_id = buff[(det_begin_index++)]<<24;
			tmpbuf->m_algopred.keliuresult.pop_res[i].pop_id += buff[(det_begin_index++)]<<16;
			tmpbuf->m_algopred.keliuresult.pop_res[i].pop_id += buff[(det_begin_index++)]<<8;
			tmpbuf->m_algopred.keliuresult.pop_res[i].pop_id += buff[(det_begin_index++)];

			//pop size
			tmpbuf->m_algopred.keliuresult.pop_res[i].pop_size = buff[(det_begin_index++)]<<24;
			tmpbuf->m_algopred.keliuresult.pop_res[i].pop_size += buff[(det_begin_index++)]<<16;
			tmpbuf->m_algopred.keliuresult.pop_res[i].pop_size += buff[(det_begin_index++)]<<8;
			tmpbuf->m_algopred.keliuresult.pop_res[i].pop_size += buff[(det_begin_index++)];
		}
		
		tmpbuf->m_rgbimg.p_data = (char *)&buff[det_begin_index];
		tmpbuf->m_depthimg.p_data = tmpbuf->m_rgbimg.p_data + tmpbuf->m_rgbimg.m_framelen;

		tmpbuf1 = (tcps_frames *)tcps_memnode_alloc(pclient->m_pool[type_save], &bufsize);
		if (NULL != tmpbuf1)
		{
			memcpy((char *)tmpbuf1, (char *)tmpbuf, sizeof(tcps_frames) + uiFrameLen);
			ret = tcps_memfifo_put(&(pclient->m_fifo[type_save]), (char *)tmpbuf1);
			if (ret <= 0)
			{
				tcps_memnode_release(pclient->m_pool[type_save], (void *)tmpbuf1);
			}
		}

		ret = tcps_memfifo_put(&(pclient->m_fifo[type_dec]), (char *)tmpbuf);
		if (ret <= 0)
		{
		    //释放帧内存
		    tcps_memnode_release(pclient->m_pool[type_dec], (void *)tmpbuf);
		}

		usleep(1000);
	}
		
	return NULL;
}

static void * tx2_server_tcpstart(void *para)
{
	printf("====>%s(%d): dectid=%lun \n", __func__, __LINE__, gettid());
	int 	    sockfd = -1;
	struct sockaddr_in service;
	pthread_t   tid;
	pthread_t   tid1;

	if ((sockfd = socket(AF_INET,SOCK_STREAM,0)) == -1)
	{
		perror("Creating socket failed.\n");
		return NULL;
	}

	bzero(&service, sizeof(service));
	service.sin_family      = AF_INET;
	service.sin_addr.s_addr = htonl(INADDR_ANY);
	service.sin_port        = htons(server_port[0]);
	
	if (0 != bind(sockfd,(struct sockaddr *)&service,sizeof(sockaddr_in)))
	{
		perror("Bind() error.\n");
		close(sockfd);
		return NULL;
	}
	
	if (0 != listen(sockfd,5))
	{
		perror("listen() error.\n");
		return NULL;
	}

	int i = 0;
	int flag = 0;
	struct sockaddr_in tmpAddr;
	while (1)
	{
		if (socket_Select(sockfd,100,TRUE))
		{
			struct sockaddr_in clientAddr;
			int iLen = sizeof(sockaddr_in);
			int accSock = accept(sockfd,(struct sockaddr *)&clientAddr,(socklen_t *)&iLen);
			if (accSock == -1)
			{
				continue;
			}
			flag = 0;
			for (int j = 0; j < g_clientlist.m_num; j++)
			{
				tmpAddr = g_clientlist.m_client[j].m_client;
				if (tmpAddr.sin_addr.s_addr == clientAddr.sin_addr.s_addr)
				{
					//关掉历史socket,记录新的socket
					close(g_clientlist.m_client[j].m_socket);
					g_clientlist.m_client[j].m_socket = accSock;
					flag = 1;
					break;
				}
			}
			if (1 == flag)
			{
				continue;
			}
			g_clientlist.m_client[i].m_socket = accSock;
			g_clientlist.m_client[i].m_client = clientAddr;
			g_clientlist.m_client[i].m_id     = i;
			g_clientlist.m_client[i].m_run 	  = 1;
			pthread_create(&tid, NULL, tx2_clientin_thread, &(g_clientlist.m_client[i]));
			pthread_create(&tid1, NULL, tx2_clientout_thread, &(g_clientlist.m_client[i]));
			i++;
			g_clientlist.m_num = i;
			//usleep(1000 * 10);
		}
	}
}

int tx2_server_tcpinit()
{
	int ret = tfailure;
	
	pthread_t tid;
	
	for (int i = 0; i < CLIENT_SUM; i++)
	{
		tcps_memfifo_create(&(g_clientlist.m_client[i].m_fifo[type_dec]), 128);
		tcps_memfifo_create(&(g_clientlist.m_client[i].m_fifo[type_save]), 128);
		tcps_queue_alloc(&(g_clientlist.m_client[i].m_queue), 1024 * 1024 * 8 * 2);
		g_clientlist.m_client[i].m_pool[type_dec] = tcps_mempool_init(128, 1024 * 512);
		g_clientlist.m_client[i].m_pool[type_save] = tcps_mempool_init(128, 1024 * 512);
	}

	//开启tx2服务端
	pthread_create(&tid, NULL, tx2_server_tcpstart, NULL);

	ret = tsuccess;
	return ret;
}

int tx2_server_getframes(int devid, int chnid, tcps_frames **pframe, int msec)
{
	if ((NULL == pframe) || (devid < 0) || (devid >= CLIENT_SUM))
	{
		return tfailure;
	}

	int ret = tfailure;
	char *pnode = NULL;
	int clientid = dev_list[devid];

	if (clientid >= 0)
	{
		ret = tcps_memfifo_get(&(g_clientlist.m_client[clientid].m_fifo[chnid]), &pnode, msec);
		if (ret != 0)
		{
			goto exit;
		}
	}
	else
	{
		usleep(1000 * msec);
		goto exit;
	}
	
	*pframe = (tcps_frames *)pnode;
	ret = tsuccess;
	
exit:
	return ret;
}

int tx2_server_freeframes(int devid, int chnid, tcps_frames *pframe)
{
	if ((NULL == pframe) || (devid < 0) || (devid >= CLIENT_SUM))
	{
		return tfailure;
	}

	int clientid = dev_list[devid];
	if (clientid >= 0)
	{
		tcps_memnode_release(g_clientlist.m_client[clientid].m_pool[chnid], (void *)pframe);
	}
	
	return tsuccess;
}

static void *tx2_server_syncthread(void *para)
{
	printf("====>%s(%d): dectid=%lun \n", __func__, __LINE__, gettid());
	int 				  ret = -1;
	int                   len = 0;
	int                   devid[4]  = {-1};
	tcps_frames          *pframe[4]   = {NULL};
	tx2_syncframe_input   syncframe[4];
	tx2_syncframe_input  *pframemax[4] = {NULL};
	tcps_frames          *ptmpframe   = NULL;
	long long int         tmpdiffer   = 0;
	int 			      frmcnt      = 0;
	tx2_syncframe_out	 *psyncfrm    = NULL;
	int                   tmpid       = 0;
	tcps_frames          *pfrm0       = NULL;
	tcps_frames			 *pfrm1		  = NULL;

	while (1)
	{
		//顺序获取需要同步的各路帧
		//应该先quary两个fifo,都没有数据，再等待
		devid[0] = tx2_decsadmin_respond(TCPS_FRAMERATE * 5 * 10);
		if (devid[0] < 0)
		{
			continue;
		}

		devid[1] = (0 == devid[0]) ? 1 : 0;
		for	(int i = 0; i < 2; i++)
		{
			pframe[i] = NULL;
			ret = tx2_decsdev_getfrm(devid[i], &(pframe[i]), TCPS_FRAMERATE * 3);
			syncframe[i].m_clientid  = devid[i];
			syncframe[i].m_pframe    = pframe[i];
			syncframe[i].m_timestamp = 0;
			if (NULL != pframe[i])
			{
				syncframe[i].m_timestamp = pframe[i]->m_rgbimg.m_timestamp;
			}
			pframemax[i] = &(syncframe[i]);
		}
	    
		if ((NULL == pframemax[0]->m_pframe) && (NULL == pframemax[1]->m_pframe))
		{
			continue;
		}
		pframemax[0]->m_timestamp = (pframemax[0]->m_timestamp < 0) ? 0 : pframemax[0]->m_timestamp;
		pframemax[1]->m_timestamp = (pframemax[1]->m_timestamp < 0) ? 0 : pframemax[1]->m_timestamp;
		if ((pframemax[0]->m_timestamp == 0) && (pframemax[0]->m_timestamp == 0))
		{
			if ((NULL != pframemax[0]->m_pframe) && (pframemax[0]->m_clientid < 4) && 
				pframemax[0]->m_clientid >= 0)
			{
				tx2_decsdev_release(pframemax[0]->m_clientid, (tcps_frames *)pframemax[0]->m_pframe);
			}

			if ((NULL != pframemax[1]->m_pframe) && (pframemax[1]->m_clientid < 4) &&
				pframemax[1]->m_clientid >= 0)
			{
				tx2_decsdev_release(pframemax[1]->m_clientid, (tcps_frames *)pframemax[1]->m_pframe);
			}
			continue;
		}
		if (syncframe[0].m_timestamp < syncframe[1].m_timestamp)
		{
			pframemax[0] = &(syncframe[1]);
			pframemax[1] = &(syncframe[0]);
		}
		ptmpframe = NULL;
		for (frmcnt = 0; frmcnt < 5; frmcnt++)
		{
			tmpdiffer = (llabs(pframemax[0]->m_timestamp - pframemax[1]->m_timestamp)) / 1000;
			if (tmpdiffer <= 30)
			{
				//同一帧
				//申请同步帧
				psyncfrm = (tx2_syncframe_out *)tcps_memnode_alloc(g_clientlist.p_syncpool, &len);
				if (NULL == psyncfrm)
				{
					continue;
				}
				pfrm0 = (tcps_frames *)pframemax[0]->m_pframe;
				pfrm1 = (tcps_frames *)pframemax[1]->m_pframe;
				psyncfrm->syncframe[0] = (pfrm0->m_devid < pfrm1->m_devid) ? pfrm0 : pfrm1;
				psyncfrm->syncframe[1] = (pfrm0->m_devid < pfrm1->m_devid) ? pfrm1 : pfrm0;
				psyncfrm->syncnum = 2;
				//推入同步fifo
				ret = tcps_memfifo_put(&g_clientlist.m_syncfifo, (char *)psyncfrm);
					
				if ((ret <= 0) && (NULL != psyncfrm))
				{
					//释放帧
					tx2_decsdev_release(pframemax[0]->m_clientid, (tcps_frames *)pframemax[0]->m_pframe);
					tx2_decsdev_release(pframemax[1]->m_clientid, (tcps_frames *)pframemax[1]->m_pframe);
					tcps_memnode_release(&g_clientlist.m_syncfifo, (void *)psyncfrm);
				}
				break;
			}
			else
			{
				//丢帧
				if (NULL == pframemax[1]->m_pframe)
				{
					//释放无效帧
					tmpid = pframemax[0]->m_clientid;
					ptmpframe = (tcps_frames *)pframemax[0]->m_pframe;
					tx2_decsdev_release(tmpid, ptmpframe);
					break;
				}
				else
				{
					//释放帧
					tmpid = pframemax[1]->m_clientid;
					ptmpframe = (tcps_frames *)pframemax[1]->m_pframe;
					tx2_decsdev_release(tmpid, ptmpframe);
					ptmpframe = NULL;
					if (frmcnt >= 4)
					{
						tmpid = pframemax[0]->m_clientid;
						ptmpframe = (tcps_frames *)pframemax[0]->m_pframe;
						tx2_decsdev_release(tmpid, ptmpframe);
						break;
					}
					ret = tx2_decsdev_getfrm(pframemax[1]->m_clientid, &ptmpframe, TCPS_FRAMERATE + 10);
					syncframe[3].m_clientid = pframemax[1]->m_clientid;
					syncframe[3].m_timestamp = 0;
					syncframe[3].m_pframe = ptmpframe;
					if (NULL != ptmpframe)
					{
						syncframe[3].m_timestamp = ptmpframe->m_rgbimg.m_timestamp;
					}
					pframemax[1] = &syncframe[3];
				}
			}
		}
	}
	
	return NULL;
}

int tx2_server_getsyncframe(tx2_syncframe_out **psyncframes, int msec)
{
	if (NULL == psyncframes)
	{
		return tfailure;
	}
	
	int   			   ret      = tfailure;
	char 			  *pnode    = NULL; 
	tx2_syncframe_out *psyncout = NULL;
	if (NULL != g_clientlist.p_syncpool)
	{
		ret = tcps_memfifo_get(&g_clientlist.m_syncfifo, &pnode, msec);
		if (0 == ret)
		{
			psyncout = (tx2_syncframe_out *)pnode;
			*psyncframes = psyncout;
		}
	}
	
	return ret;
}

int tx2_server_fresssyncframe(tx2_syncframe_out *psyncframes)
{
	if (NULL == psyncframes)
	{
		return tfailure;
	}

	tcps_frames		  *pframe0   = NULL;
	tcps_frames		  *pframe1   = NULL;
	tx2_syncframe_out *psyncout = NULL;

	psyncout = psyncframes;
	pframe0  = (tcps_frames *)(psyncout->syncframe[0]);
	pframe1  = (tcps_frames *)(psyncout->syncframe[1]);
			
	tcps_memnode_release(g_clientlist.p_syncpool, (void *)psyncout);
	tx2_decsdev_release(pframe0->m_devid, pframe0);
	tx2_decsdev_release(pframe1->m_devid, pframe1);

	return tsuccess;
}

int tx2_server_syncinit()
{
	int ret = tfailure;
	pthread_t tid;
	
	//创建同步fifo
	ret = tcps_memfifo_create(&g_clientlist.m_syncfifo, 32);
	if (tsuccess != ret)
	{
		goto exit;
	}

	//创建同步帧池
	g_clientlist.p_syncpool = tcps_mempool_init(32, 128);
	
	g_clientlist.m_sync = 0xffff;
	pthread_condattr_init(&g_clientlist.condAttr);
	pthread_mutex_init(&(g_clientlist.m_lock), NULL);
	pthread_condattr_setclock(&(g_clientlist.condAttr), CLOCK_MONOTONIC);
 	pthread_cond_init(&(g_clientlist.m_rcond), &g_clientlist.condAttr);
	
	//创建同步线程
	pthread_create(&tid, NULL, tx2_server_syncthread, NULL);
	
exit:
	return ret;
}

static void *tx2_server_decsthread(void *para)
{
	printf("====>%s(%d): dectid=%lun \n", __func__, __LINE__, gettid());
	pthread_detach(pthread_self());
	
	int ret = tfailure;
	tcps_frames *pframe[4];
	
	while(1)
	{
		for (int i = 0; i < 4; i++)
		{
			pframe[i] = NULL;
			ret = tx2_server_getframes(i, type_dec, &(pframe[i]), 4);
			if (0 != ret || NULL == pframe[i])
			{
				continue;
			}

			//送解码
			tx2_decsdev_putfrm(i, pframe[i]); 
			
			//释放帧
			tx2_server_freeframes(i, type_dec, pframe[i]);
		}
		usleep(1000*1);
	}

	return NULL;
}

int tx2_server_decsinit()
{
	int 	  ret = tfailure;
	pthread_t tid;
	int chnid[4] = {0, 1};
	tx2_dec_params stDecParam[4];
	
	//创建解码器
	for (int i = 0; i < 4; i++)
	{
		stDecParam[i].m_chnid 	 = i;
		stDecParam[i].m_dectype = (i < 2) ? dectype_mix : dectype_rgb;
		//解码的分辨率
		stDecParam[i].m_decrgbattr.m_decParam.width    = rgb_width;
		stDecParam[i].m_decrgbattr.m_decParam.height   = rgb_height;
		stDecParam[i].m_decdepthattr.m_decParam.width  = depth_decwidth;
		stDecParam[i].m_decdepthattr.m_decParam.height = depth_decheight;
		//解码类型
		stDecParam[i].m_decrgbattr.m_decParam.dec_type = 0;
		stDecParam[i].m_decrgbattr.m_decParam.fps = 25;
		stDecParam[i].m_decdepthattr.m_decParam.dec_type = 0;
		stDecParam[i].m_decdepthattr.m_decParam.fps = 25;

		stDecParam[i].m_decrgbattr.m_decBuffParam.sendbuff_size  = rgb_width*rgb_height*3/2;
		stDecParam[i].m_decrgbattr.m_decBuffParam.sendbuff_count = 16;
		stDecParam[i].m_decrgbattr.m_decBuffParam.getbuff_size   = rgb_width*rgb_height*3/2;
		stDecParam[i].m_decrgbattr.m_decBuffParam.getbuff_count  = 16;

		stDecParam[i].m_decdepthattr.m_decBuffParam.sendbuff_size  = depth_decwidth*depth_decheight*3/2;
		stDecParam[i].m_decdepthattr.m_decBuffParam.sendbuff_count = 16;
		stDecParam[i].m_decdepthattr.m_decBuffParam.getbuff_size   = depth_decwidth*depth_decheight*3/2;
		stDecParam[i].m_decdepthattr.m_decBuffParam.getbuff_count  = 16;
		ret = tx2_decsdev_create(i, stDecParam[i].m_dectype, &stDecParam[i]);
		if (tsuccess != ret)
		{
			goto exit;
		}
	}

	tx2_decsadmin_init(2, chnid);
	//开启解码线程
	pthread_create(&tid, NULL, tx2_server_decsthread, NULL);
	
exit:
	return ret;
}


int tcps_GetDateTime( time_t  input_time, T_IPC_Date &stDateTime )
{
	time_t m_time;
	m_time = (time_t)(input_time/1000000);
	memset(&stDateTime, 0x0, sizeof(T_IPC_Date));
	
	struct tm *timenow; //实例化tm结构指针
	
	//time函数读取现在的时间(国际标准时间非北京时间)，然后传值给now
	//time(&now);
	//localtime函数把从time取得的时间now换算成你电脑中的时间(就是你设置的地区)
	timenow = localtime(&m_time);

	stDateTime.year = timenow->tm_year % 100;//年只取后两位
	stDateTime.month = timenow->tm_mon + 1;
	stDateTime.day = timenow->tm_mday;
	stDateTime.hour = timenow->tm_hour;
	stDateTime.minute = timenow->tm_min;
	stDateTime.second = timenow->tm_sec;
	stDateTime.msecond = (unsigned int)((input_time/1000)%1000);
	stDateTime.week = timenow->tm_wday + 1;
	
	return 0;

}

void tjtongka_getkeliu_k(KELIU_SAVERESULT_T result, char * k_s)
{

	T_IPC_Date stDate;
	tcps_GetDateTime( (time_t)result.time, stDate );
	
	sprintf( k_s, "head=YR,func=K,devid=%d,time=%02d-%02d-%02d %02d:%02d:%02d:%03u,innum=%d,outnum=%d\0",
		result.devid, stDate.year, stDate.month, stDate.day, stDate.hour, stDate.minute, stDate.second, stDate.msecond, result.up_num, result.down_num);
	return;

}


void tjtongka_getkeliu_d(tx2_sort_net result, char * d_s)
{

	char title[128] = {0};
	char tmp[1024] = {0};
	
	T_IPC_Date stDate;
	tcps_GetDateTime( (time_t)result.time, stDate );
	
	sprintf( title, "head=YR,func=D,devid=%d,time=%02d-%02d-%02d %02d:%02d:%02d:%03u,sortnum=%d,",
		result.devid, stDate.year, stDate.month, stDate.day, stDate.hour, stDate.minute, stDate.second, stDate.msecond, result.sort_size);
	if(result.sort_size > 0)
	{
		tmp_sort = (char *)realloc(tmp_sort, result.sort_size * 128);
		char *pstr_sort = tmp_sort;
		memset(tmp_sort, 0, result.sort_size * 128);
		for(int i=0; i<result.sort_size; i++)
		{
			sprintf(pstr_sort, "persid=%d,x=%f,y=%f,z=%f", result.sort_res[i].id, result.sort_res[i].box.center.x, result.sort_res[i].box.center.y, result.sort_res[i].box.center.z);
			tmp_sort[strlen(tmp_sort)] = ((i + 1) == (int)result.sort_size) ? '\0' : '@';
			pstr_sort = tmp_sort + strlen(tmp_sort);
		}
	}
	else{
		memset(tmp_sort, 0x00, strlen(tmp_sort));
	}
	sprintf(d_s, "%s%s", title, tmp_sort);
	return;
}
int send_sort_net( ALGO_Output_Res * algores, tx2_sort_net  *sort_net)
{
	int ret = -1;
	
	sort_net->time = algores->m_dynamicres.time[0];
	sort_net->devid = 0;
	sort_net->sort_size = algores->m_dynamicres.sort_size;
	for(int i=0; i<sort_net->sort_size; i++)
	{
		sort_net->sort_res[i] = algores->m_dynamicres.sort_res[i];
	}

	return 0;
	

}
static void *tx2_server_algoputthread(void *para)
{
	printf("====>%s(%d): dectid=%lun \n", __func__, __LINE__, gettid());
	pthread_detach(pthread_self());
	
	int 			   ret = tfailure;
	tx2_syncframe_out *psyncframe = NULL;
	
	while (1)
	{
		//获取同步帧
		ret = tx2_server_getsyncframe(&psyncframe, 400);
		if (0 != ret)
		{
			continue;
		}
		if (1 != algo_init)
		{
			tx2_server_fresssyncframe(psyncframe);
			usleep(1000 * 5);
			continue;
		}
		//送同步fifo
		ret = tcps_memfifo_put(&g_virtdevlist[func_dynamic].m_fifo[0], (char *)psyncframe);
		if (ret > 0)
		{
			//送算法
			ret = Tcps_Algo_Put(psyncframe);
		}
		else
		{
			tx2_server_fresssyncframe(psyncframe);
		}
	}
	
	return NULL;
}

static void *tx2_server_algogetthread(void *para)
{
	pthread_detach(pthread_self());
	printf("====>%s(%d): dectid=%lun \n", __func__, __LINE__, gettid());
	ALGO_Output_Res    stalgores;
	int  			   ret	      = tfailure;
	int                flag       = 0;
	int                flag1      = 0;
	char              *pnode 	  = NULL;
	tx2_syncframe_out *psyncframe = NULL;
	tcps_frames       *pframe[2]  = {NULL};
	memset(&stalgores, 0, sizeof(ALGO_Output_Res));
	float fw, fh;
	float fw2, fh2;
	Rect rec[128];
	Rect rec2[128];
	unsigned int delta_x = 0;
	unsigned int delta_y = 0;
	float scale_x = 1;
	float scale_y = 1;
	unsigned int devid[2] = {0};
	struct __block *sortnet_block = NULL;
	sortnet_block = block_alloc( sortnet_block_size );
	char caKeLiu2[256];
	tx2_sort_net  sort_net;
	
	while (1)
	{
		//获取算法结果, 超时丢帧
		if (flag <= 0)
		{
			memset(&stalgores, 0, sizeof(ALGO_Output_Res));
			flag = 1;
			
			ret = Tcps_Algo_Get(&stalgores, 400);
			
			if (0 != ret)
			{
				flag = 0;
				stalgores.m_dynamicres.time[0] = 0;
			}
		}
		//获取视频帧
		if (NULL == pnode)
		{
			pnode = NULL;
			ret = tcps_memfifo_get(&g_virtdevlist[func_dynamic].m_fifo[0], &pnode, 400);
			if (NULL == pnode)
			{
				continue;
			}
			psyncframe = (tx2_syncframe_out *)pnode;
			pframe[0] = (tcps_frames *)psyncframe->syncframe[0];
			pframe[1] = (tcps_frames *)psyncframe->syncframe[1];
		}
		//比较时间戳 (algo 比 帧慢 丢结果)
		if (stalgores.m_dynamicres.time[0] < pframe[0]->m_rgbimg.m_timestamp)
		{
			if (flag == 1)
			{
				flag = 0;
				continue;
			}
		}
		else if (stalgores.m_dynamicres.time[0] == pframe[0]->m_rgbimg.m_timestamp)
		{
			//画框
			Mat im_RGB0( rgb_height, rgb_width, CV_8UC3, (unsigned char *)pframe[0]->m_rgbimg.p_data );
			devid[0] = pframe[0]->m_devid;
			for(int j = 0; j < stalgores.m_tracklist[devid[0]].m_convertres.sort_size; j++)
			{	
				fw = (stalgores.m_tracklist[devid[0]].m_convertres.sort_res[j].box.width)*scale_x -20;
				fh = (stalgores.m_tracklist[devid[0]].m_convertres.sort_res[j].box.height)*scale_y - 20;
				rec[j].x = (unsigned int)((stalgores.m_tracklist[devid[0]].m_convertres.sort_res[j].box.x < delta_x) ? 0 : (stalgores.m_tracklist[devid[0]].m_convertres.sort_res[j].box.x - delta_x)*scale_x + 10);
				rec[j].y = (unsigned int)((stalgores.m_tracklist[devid[0]].m_convertres.sort_res[j].box.y < delta_y) ? 0 : (stalgores.m_tracklist[devid[0]].m_convertres.sort_res[j].box.y - delta_y)*scale_y + 10);
				rec[j].width = (unsigned int)(((rec[j].x + fw) > rgb_width) ? (rgb_width - rec[j].x) : fw);
				rec[j].height = (unsigned int)(((rec[j].y + fh) > rgb_height) ? (rgb_height - rec[j].y) : fh);

				//fw2 = (pframe[0]->m_algopred.keliuresult.det_res[j].det_rec.width)*scale_x -20;
				//fh2 = (pframe[0]->m_algopred.keliuresult.det_res[j].det_rec.height)*scale_y - 20;
				//rec2[j].x = (unsigned int)((pframe[0]->m_algopred.keliuresult.det_res[j].det_rec.x < delta_x) ? 0 : (pframe[0]->m_algopred.keliuresult.det_res[j].det_rec.x - delta_x)*scale_x + 10);
				//rec2[j].y = (unsigned int)((pframe[0]->m_algopred.keliuresult.det_res[j].det_rec.y < delta_y) ? 0 : (pframe[0]->m_algopred.keliuresult.det_res[j].det_rec.y - delta_y)*scale_y + 10);
				//rec2[j].width = (unsigned int)(((rec2[j].x + fw2) > rgb_width) ? (rgb_width - rec2[j].x) : fw2);
				//rec2[j].height = (unsigned int)(((rec2[j].y + fh2) > rgb_height) ? (rgb_height - rec2[j].y) : fh2);
				//rectangle(im_RGB0, rec2[j], Scalar(255, 0, 0), 2);	


				rectangle(im_RGB0, rec[j], Scalar(0, 0, 255), 2);	
			}
			for (int m = 0; m < stalgores.m_tracklist[devid[0]].m_personnum; m++)
			{
				vector<Point> points0;
				for (int n = 0; n < stalgores.m_tracklist[devid[0]].m_trackperson[m].m_psize; n++)
				{
					TCPS_POINT *point0 = &(stalgores.m_tracklist[devid[0]].m_trackperson[m].m_point[n]);
					Point cent_point0 = Point(point0->x, point0->y);
					points0.push_back(cent_point0);
				}
				const Point* ppt0[1] = { &points0[0] };
				int npt0[] = { points0.size() };
				cv::polylines(im_RGB0, ppt0, npt0, 1, false, Scalar(0, 111, 255), 3, 1, 0);
			}
			Mat im_RGB1( rgb_height, rgb_width, CV_8UC3, (unsigned char *)pframe[1]->m_rgbimg.p_data );
			devid[1] = pframe[1]->m_devid;
			for(int k = 0; k < stalgores.m_tracklist[devid[1]].m_convertres.sort_size; k++)
			{	
				fw = (stalgores.m_tracklist[devid[1]].m_convertres.sort_res[k].box.width)*scale_x -20;
				fh = (stalgores.m_tracklist[devid[1]].m_convertres.sort_res[k].box.height)*scale_y - 20;
				rec[k].x = (unsigned int)((stalgores.m_tracklist[devid[1]].m_convertres.sort_res[k].box.x < delta_x) ? 0 : (stalgores.m_tracklist[devid[1]].m_convertres.sort_res[k].box.x - delta_x)*scale_x + 10);
				rec[k].y = (unsigned int)((stalgores.m_tracklist[devid[1]].m_convertres.sort_res[k].box.y < delta_y) ? 0 : (stalgores.m_tracklist[devid[1]].m_convertres.sort_res[k].box.y - delta_y)*scale_y + 10);
				rec[k].width = (unsigned int)(((rec[k].x + fw) > rgb_width) ? (rgb_width - rec[k].x) : fw);
				rec[k].height = (unsigned int)(((rec[k].y + fh) > rgb_height) ? (rgb_height - rec[k].y) : fh);

				//fw2 = (pframe[1]->m_algopred.keliuresult.det_res[k].det_rec.width)*scale_x -20;
				//fh2 = (pframe[1]->m_algopred.keliuresult.det_res[k].det_rec.height)*scale_y - 20;
				//rec2[k].x = (unsigned int)((pframe[1]->m_algopred.keliuresult.det_res[k].det_rec.x < delta_x) ? 0 : (pframe[1]->m_algopred.keliuresult.det_res[k].det_rec.x - delta_x)*scale_x + 10);
				//rec2[k].y = (unsigned int)((pframe[1]->m_algopred.keliuresult.det_res[k].det_rec.y < delta_y) ? 0 : (pframe[1]->m_algopred.keliuresult.det_res[k].det_rec.y - delta_y)*scale_y + 10);
				//rec2[k].width = (unsigned int)(((rec2[k].x + fw2) > rgb_width) ? (rgb_width - rec2[k].x) : fw2);
				//rec2[k].height = (unsigned int)(((rec2[k].y + fh2) > rgb_height) ? (rgb_height - rec2[k].y) : fh2);
				//rectangle(im_RGB1, rec2[k], Scalar(255, 0, 0), 2);

				rectangle(im_RGB1, rec[k], Scalar(0, 0, 255), 2);	
				//cv::putText(im_RGB1, to_string(stalgores.m_convertres[1].id), Point2d(rec[k].y, rec[k].x), FONT_HERSHEY_DUPLEX, 1, Scalar(255, 255, 0), 1);
			}
			for (int m = 0; m < stalgores.m_tracklist[devid[1]].m_personnum; m++)
			{
				vector<Point> points1;
				for (int n = 0; n < stalgores.m_tracklist[devid[1]].m_trackperson[m].m_psize; n++)
				{
					TCPS_POINT *point1 = &(stalgores.m_tracklist[devid[1]].m_trackperson[m].m_point[n]);
					Point cent_point1 = Point(point1->x, point1->y);
					points1.push_back(cent_point1);
				}
				const Point* ppt1[1] = { &points1[0] };
				int npt1[] = { points1.size() };
				cv::polylines(im_RGB1, ppt1, npt1, 1, false, Scalar(0, 111, 255), 3, 1, 0);
			}			
			//算法结果时间戳置0，重新取结果
			flag = 0;
			flag1 = 1;
		}	
		
		//推入显示fifo
		if (NULL != pnode)
		{
			ret = tcps_memfifo_put(&g_virtdevlist[func_dynamic].m_fifo[1], (char *)psyncframe);
			if (ret <= 0)
			{
				//释放帧
				tx2_server_fresssyncframe(psyncframe);
			}
			pnode = NULL;
		}
		//推入后台fifo
		if (1 == flag1)
		{
			memset(&sort_net, 0x00, sizeof(tx2_sort_net));
			ret = send_sort_net(&stalgores, &sort_net);
			if(ret == 0)
			{
				tjtongka_getkeliu_d(sort_net, caKeLiu2);
				memcpy( sortnet_block->data, (char *)caKeLiu2, strlen(caKeLiu2));
				
				sortnet_block->m_datasize = strlen(caKeLiu2);
				ret = kfifo_put(&(g_virtdevlist[func_dynamic].m_kfifo[0]), sortnet_block);
			}
			flag1 = 0;
		}
	}
	
	return NULL;
}


static void *tx2_server_keliuthread(void *para)
{
	pthread_detach(pthread_self());
	printf("====>%s(%d): dectid=%lun \n", __func__, __LINE__, gettid());
	int ret = -1;
	tcps_frames          *pframe[4]   = {NULL};

	float fw, fh;
	Rect rec[128];
	unsigned int delta_x = 0;
	unsigned int delta_y = 0;
	float scale_x = 1;
	float scale_y = 1;
	char caKeLiu0[256];
	char caKeLiu1[256];

	struct __block *result_block = NULL;
	result_block = block_alloc( tcps_block_size );
	struct __block *result_block2 = NULL;
	result_block2 = block_alloc( tcps_block_size );
	
	while(1)
	{
		if(virtdev_init != 1)
		{
			usleep(5000);
			continue;
		}
		
		if((g_virtdevlist[0].m_init == 1)&&(g_virtdevlist[1].m_init == 1))
		{
			for(int i=2; i<tk_inputcamera_number; i++)
			{
				pframe[i] = NULL;
				ret = tx2_decsdev_getfrm(i, &pframe[i], 5);//single
			

				//发送一帧nv12数据
				if (NULL  == pframe[i])
				{
					usleep(5000);
					continue;
				}
				
				Mat im_RGB( rgb_height, rgb_width, CV_8UC3, (unsigned char *)pframe[i]->m_rgbimg.p_data );
				for(int j = 0; j < pframe[i]->m_algopred.keliuresult.sort_size; j++)
				{	
					if((pframe[i]->m_algopred.keliuresult.sort_res[j].pro) >= 0.3)
					{
						fw = (pframe[i]->m_algopred.keliuresult.sort_res[j].box.width)*scale_x -20;
						fh = (pframe[i]->m_algopred.keliuresult.sort_res[j].box.height)*scale_y - 20;
						rec[j].x = (unsigned int)((pframe[i]->m_algopred.keliuresult.sort_res[j].box.x < delta_x) ? 0 : (pframe[i]->m_algopred.keliuresult.sort_res[j].box.x - delta_x)*scale_x + 10);
						rec[j].y = (unsigned int)((pframe[i]->m_algopred.keliuresult.sort_res[j].box.y < delta_y) ? 0 : (pframe[i]->m_algopred.keliuresult.sort_res[j].box.y - delta_y)*scale_y + 10);
						rec[j].width = (unsigned int)(((rec[j].x + fw) > rgb_width) ? (rgb_width - rec[j].x) : fw);
						rec[j].height = (unsigned int)(((rec[j].y + fh) > rgb_height) ? (rgb_height - rec[j].y) : fh);
					
						rectangle(im_RGB, rec[j], Scalar(0, 0, 255), 2);	
					}
					
					
				}
				cv::putText(im_RGB, "GetonNum" + to_string(pframe[i]->m_algopred.keliuresult.up_num), Point2d(50, 50), FONT_HERSHEY_DUPLEX, 1, Scalar(255, 255, 255), 1, 8);
				cv::putText(im_RGB, "DownNum" + to_string(pframe[i]->m_algopred.keliuresult.down_num), Point2d(50, 350), FONT_HERSHEY_DUPLEX, 1, Scalar(0, 255, 0), 1);

				
			}

			if(pframe[2] != NULL)
			{
				ret = tcps_memfifo_put(&(g_virtdevlist[func_keliu0].m_fifo[0]), (char *)pframe[2]);
				if(ret <= 0)
				{
					printf("put func_keliu0 failed\n");
					tx2_decsdev_release(2, pframe[2]);//single
				}
				tjtongka_getkeliu_k(pframe[2]->m_algopred.keliuresult, caKeLiu0);
			
				memcpy( result_block->data, (char *)caKeLiu0, strlen(caKeLiu0));
				
				result_block->m_datasize = strlen(caKeLiu0);
				ret = kfifo_put(&(g_virtdevlist[func_keliu0].m_kfifo[0]), result_block);

			}
			if(pframe[3] != NULL)
			{
				ret = tcps_memfifo_put(&(g_virtdevlist[func_keliu1].m_fifo[0]), (char *)pframe[3]);
				if(ret <= 0)
				{
					printf("put func_keliu1 failed\n");
					tx2_decsdev_release(3, pframe[3]);//single
				}
				tjtongka_getkeliu_k(pframe[3]->m_algopred.keliuresult, caKeLiu1);
			
				memcpy( result_block2->data, (char *)caKeLiu1, strlen(caKeLiu1));
				
				result_block2->m_datasize = strlen(caKeLiu1);
				ret = kfifo_put(&(g_virtdevlist[func_keliu1].m_kfifo[0]), result_block2);
			}
			
		}
		usleep(1000);
	}
}


static void *tx2_server_netthread(void *para)
{
	int ret = -1;
	struct __block *net_block = {NULL};
	net_block = block_alloc( tcps_block_size );
	printf("====>%s(%d): dectid=%lun \n", __func__, __LINE__, gettid());

	while(1)
	{
		if(virtdev_init != 1)
		{
			usleep(5000);
			continue;
		}
		
		for(int i=0; i<2; i++)
		{
			ret = kfifo_get( &(g_virtdevlist[i].m_kfifo[0]), net_block, 5);
			if(ret == 0)
			{
				//发送透传数据样例
				ret = client_createdataevent( ( char * )net_block->data, net_block->m_datasize);
				if(ret < 0)
				{
					printf("keliuk error, ret = %d\n", ret);
				}
			}
				
		}
		
		ret = kfifo_get( &(g_virtdevlist[func_dynamic].m_kfifo[0]), net_block, 5);
		if(ret == 0)
		{
			//tjtongka_getkeliu_d(pframe[i]->m_algopred.keliuresult, caKeLiu2);
			//发送透传数据样例
			ret = client_createdataevent( ( char * )net_block->data, net_block->m_datasize);
			if(ret < 0)
			{
				printf("keliud error, ret = %d\n", ret);
			}
		}
		
		usleep(5000);
		
	}
}
int tx2_server_virtdevinit()
{
	int ret = tfailure;
	pthread_t tid[4];
	
	for (int i = 0; i < 3; i++)
	{
		//创建fifo
		tcps_memfifo_create(&g_virtdevlist[i].m_fifo[0], 32);
		kfifo_alloc(&g_virtdevlist[i].m_kfifo[0], 32, 1024 * 128);
		if (func_dynamic == i)
		{
			tcps_memfifo_create(&g_virtdevlist[i].m_fifo[1], 32);
		}
		g_virtdevlist[i].m_init = 1;
	}
	virtdev_init = 1;
	DEBUG("virtdev_init successed\n");
	
	//创建线程
	pthread_create(&tid[0], NULL, tx2_server_algogetthread, NULL);
	pthread_create(&tid[1], NULL, tx2_server_algoputthread, NULL);
	pthread_create(&tid[2], NULL, tx2_server_keliuthread, NULL);
	
	ret = tsuccess;
	return ret;
}

static void *tx2_server_savethread(void *para)
{
	pthread_detach(pthread_self());
	printf("====>%s(%d): dectid=%lun \n", __func__, __LINE__, gettid());
	int            ret            = tfailure;
	tx2_save_obj  *tmpsaveobj     = NULL;
	tcps_frames   *tmpframe[4]    = {NULL};
	char          *tmpbuf[3]      = {NULL};
	time_t         m_time;
	T_IPC_Date     stDate;
	int            m_len          = 0;
	time_t  	   timestamp      = 0;
	char           path[256];
	char           path1[256];
	char           path2[256];
	struct 		   timeval now;
	FILE 		   *fp[4] 		  = {NULL};
	int             fd[4]         = {0};
	int            first  		  = 0;
	int            devid  		  = *((int *)para);
	
	while(1)
	{
		//获取视频帧
		if (0 == first)
		{
			tmpframe[0] = NULL;
			ret = tx2_server_getframes(devid, type_save, &(tmpframe[0]), 400);
			if (0 != ret || NULL == tmpframe[0])
			{
				continue;	
			}
		}

		if (NULL != tmpframe[0])
		{
			if ((save_datalen - (g_savelist[devid].m_wdepthlen + tmpframe[0]->m_depthimg.m_framelen) >= 0)
				&& ((save_datalen - (g_savelist[devid].m_wrgblen + tmpframe[0]->m_rgbimg.m_framelen) >= 0)))
			{
				first = 0;
				tmpbuf[0] = g_savelist[devid].p_depthdata + g_savelist[devid].m_wdepthlen;
				memcpy(tmpbuf[0], &tmpframe[0]->m_depthimg.m_timestamp, sizeof(long long int));
				tmpbuf[1] = tmpbuf[0] + 8;
				memcpy(tmpbuf[1], &tmpframe[0]->m_depthimg.m_framelen, sizeof(int));
				tmpbuf[0] += 32; 
				memcpy(tmpbuf[0], tmpframe[0]->m_depthimg.p_data, tmpframe[0]->m_depthimg.m_framelen);
				g_savelist[devid].m_wdepthlen += tmpframe[0]->m_depthimg.m_framelen + 32;
				
				tmpbuf[0] = g_savelist[devid].p_rgbdata + g_savelist[devid].m_wrgblen;
				memcpy(tmpbuf[0], &tmpframe[0]->m_rgbimg.m_timestamp, sizeof(long long int));
				tmpbuf[1] = tmpbuf[0] + 8;
				memcpy(tmpbuf[1], &tmpframe[0]->m_rgbimg.m_framelen, sizeof(int));
				tmpbuf[0] += 32;
				memcpy(tmpbuf[0], tmpframe[0]->m_rgbimg.p_data, tmpframe[0]->m_rgbimg.m_framelen);
				g_savelist[devid].m_wrgblen += tmpframe[0]->m_rgbimg.m_framelen + 32;
				//释放帧
				tx2_server_freeframes(devid, type_save, tmpframe[0]);
			}
			else
			{
				tmpsaveobj = &g_savelist[devid];
				first = 1;
			}
		}

		if (NULL != tmpsaveobj)
		{
			gettimeofday(&now, NULL);
			timestamp = now.tv_sec * 1000000;
			tcps_GetDateTime( timestamp, stDate );
			//每天一个文件,创建目录 cameraid/年-月-日
			sprintf(path, "%s/%02d-%02d-%02d", tmpsaveobj->p_path, stDate.year, stDate.month, stDate.day);
			if (access( path, 0 ) == -1 )
			{
				if( mkdir(path, 0777) != 0 )
				{
					usleep(1000 * 100);
					continue;
				}
			}
			//创建文件
			sprintf(path1, "%s/%02d-%02d-%02d_rgb", path, stDate.hour, stDate.minute, stDate.second);
			sprintf(path2, "%s/%02d-%02d-%02d_depth", path, stDate.hour, stDate.minute, stDate.second);
			fp[0] = fopen(path1, "wb");
			fp[1] = fopen(path2, "wb");
			if (NULL != fp[0] && NULL != fp[1])
			{
				fwrite(tmpsaveobj->p_rgbdata, tmpsaveobj->m_wrgblen, 1, fp[0]);
				fwrite(tmpsaveobj->p_depthdata, tmpsaveobj->m_wdepthlen, 1, fp[1]);
				fd[0] = fileno(fp[0]);
				fd[1] = fileno(fp[1]);
				fsync(fd[0]);
				fsync(fd[1]);
				fclose(fp[0]);
				fclose(fp[1]);
				tmpsaveobj->m_wrgblen   = 0;
				tmpsaveobj->m_wdepthlen = 0;
				tmpsaveobj = NULL;
			}
			else
			{
				if (NULL != fp[0])
				{
					fclose(fp[0]);
				}
				if (NULL != fp[1])
				{
					fclose(fp[1]);
				}
				tmpsaveobj->m_wrgblen   = 0;
				tmpsaveobj->m_wdepthlen = 0;
				tmpsaveobj = NULL;
				usleep(1000 * 10);
			}
		}
	}

	return NULL;
}

int tx2_server_saveinit(char *savepath)
{
	if (NULL == savepath)
	{
		return tfailure;
	}

	int ret = tsuccess;
	pthread_t tid[4];
	
	for (int i = 0; i < 4; i++)
	{
		g_savelist[i].p_path = (char *)malloc(256);
		sprintf(g_savelist[i].p_path, "%s/image/camera_%d", savepath, i);
		if( access( g_savelist[i].p_path, 0 ) == -1 )
		{
			if (mkdir(g_savelist[i].p_path, 0777 ) != 0)
			{
				ret = tfailure;
				goto exit;
			}
		}
		g_savelist[i].m_wrgblen   = 0;
		g_savelist[i].p_rgbdata   = (char *)malloc(save_datalen);
		g_savelist[i].m_wdepthlen = 0;
		g_savelist[i].p_depthdata = (char *)malloc(save_datalen);

		//开启存储线程
		saveid[i] = i;
		pthread_create(&tid[i], NULL, tx2_server_savethread, &saveid[i]);
	}

	
exit:
	return ret;
}

void* display_hander_thread0( void *arg )
{

	printf("display_hander_thread begin!\n");
	int ret = -1;
	tcps_frames          *pframe[4]   = {NULL};
	printf("====>%s(%d): dectid=%lun \n", __func__, __LINE__, gettid());
	float fw, fh;
	Rect rec[128];
	unsigned int delta_x = 0;
	unsigned int delta_y = 0;
	float scale_x = 1;
	float scale_y = 1;

	cv::Mat nv12(rgb_height*3/2, rgb_width, CV_8UC1);
	Display_Data_T displaydata[4];
	tx2_syncframe_out *psyncout = NULL;
	int i = 0;
	char * tmp111 = NULL;
	while(1)
	{
		if(virtdev_init != 1)
		{
			usleep(5000);
			continue;
		}
		
		
		for(i=1; i<2;i++)//tk_inputcamera_number; i++)
		{
			if (i <= 1)
			{
				pframe[0] = NULL;
				pframe[1] = NULL;
				psyncout = NULL;
				
				ret = tcps_memfifo_get(&(g_virtdevlist[func_dynamic].m_fifo[1]), &tmp111, 10);
				if (0 == ret && NULL != tmp111)
				{
					psyncout = (tx2_syncframe_out *)tmp111;
					pframe[0] = (tcps_frames *)(psyncout->syncframe[0]);
					pframe[1] = (tcps_frames *)(psyncout->syncframe[1]);
				}
			}
			else
			{
				pframe[i] = NULL;
				ret = tcps_memfifo_get(&(g_virtdevlist[i-2].m_fifo[0]), (char **)(&(pframe[i])), 5);	
			}
		}
		
		for(i=0; i<2;i++)//tk_inputcamera_number; i++)
		{
			//发送一帧nv12数据
			if (NULL  == pframe[i])
			{
				continue;
			}
			
			Mat im_RGB( rgb_height, rgb_width, CV_8UC3, (unsigned char *)pframe[i]->m_rgbimg.p_data );
			
			cvtColor(im_RGB,nv12, CV_BGR2YUV_I420);	
			displaydata[pframe[i]->m_devid].data_len = 640*400*3/2;
			displaydata[pframe[i]->m_devid].dataptr = nv12.data;
			//display[pframe[i]->m_devid]->send_displaydata(displaydata[pframe[i]->m_devid]);
			g_displaylist[pframe[i]->m_devid].p_display->send_displaydata(displaydata[pframe[i]->m_devid]);
		}
		
		for(i=1; i<2;i++)//tk_inputcamera_number; i++)
		{
			if ((i <= 1) && (NULL != psyncout))
			{
				tx2_server_fresssyncframe(psyncout);//sync
			}
			else
			{
				if (NULL != pframe[i])
				{	
					tx2_decsdev_release(i, pframe[i]);//single
				}
			}
		}
			
		usleep(5000);
	}

	return NULL;
}

void* display_hander_thread1( void *arg )
{

	printf("display_hander_thread begin!\n");
	int ret = -1;
	tcps_frames          *pframe[4]   = {NULL};
	printf("====>%s(%d): dectid=%lun \n", __func__, __LINE__, gettid());
	float fw, fh;
	Rect rec[128];
	unsigned int delta_x = 0;
	unsigned int delta_y = 0;
	float scale_x = 1;
	float scale_y = 1;

	cv::Mat nv12(rgb_height*3/2, rgb_width, CV_8UC1);
	Display_Data_T displaydata[4];
	tx2_syncframe_out *psyncout = NULL;
	int i = 0;
	char * tmp111 = NULL;
	while(1)
	{
		if(virtdev_init != 1)
		{
			usleep(5000);
			continue;
		}
		
		
		for(i=2; i<tk_inputcamera_number; i++)
		{
			if (i <= 1)
			{
				pframe[0] = NULL;
				pframe[1] = NULL;
				psyncout = NULL;
				
				ret = tcps_memfifo_get(&(g_virtdevlist[func_dynamic].m_fifo[1]), &tmp111, 10);
				if (0 == ret && NULL != tmp111)
				{
					psyncout = (tx2_syncframe_out *)tmp111;
					pframe[0] = (tcps_frames *)(psyncout->syncframe[0]);
					pframe[1] = (tcps_frames *)(psyncout->syncframe[1]);
				}
			}
			else
			{
				pframe[i] = NULL;
				ret = tcps_memfifo_get(&(g_virtdevlist[i-2].m_fifo[0]), (char **)(&(pframe[i])), 5);	
			}
		}
		
		for(i=2; i<tk_inputcamera_number; i++)
		{
			//发送一帧nv12数据
			if (NULL  == pframe[i])
			{
				continue;
			}
			
			Mat im_RGB( rgb_height, rgb_width, CV_8UC3, (unsigned char *)pframe[i]->m_rgbimg.p_data );
			
			cvtColor(im_RGB,nv12, CV_BGR2YUV_I420);	
			displaydata[pframe[i]->m_devid].data_len = 640*400*3/2;
			displaydata[pframe[i]->m_devid].dataptr = nv12.data;
			//display[pframe[i]->m_devid]->send_displaydata(displaydata[pframe[i]->m_devid]);
			g_displaylist[pframe[i]->m_devid].p_display->send_displaydata(displaydata[pframe[i]->m_devid]);
		}
		
		for(i=2; i<tk_inputcamera_number; i++)
		{
			if ((i <= 1) && (NULL != psyncout))
			{
				tx2_server_fresssyncframe(psyncout);//sync
			}
			else
			{
				if (NULL != pframe[i])
				{	
					tx2_decsdev_release(i, pframe[i]);//single
				}
			}
		}
			
		usleep(5000);
	}

	return NULL;
}

int tx2_display_init()
{
	int ret = tfailure;
	int initflag = 0;
	/***********************显示类对象初始化**********************/
	//设置缓冲大小
	Buffer_Param_T buffParam;
	buffParam.buff_size = rgb_width*rgb_height*3/2;
	buffParam.buff_count = 3;
	//设置显示参数
	Display_Param_T displayParam;
	//显示的分辨率
	displayParam.width = 960;
	displayParam.height = 540;
	//显示的位置
	displayParam.x_offset = 0;
	displayParam.y_offset = 0;
	displayParam.fps = 25;
	//要显示数据的格式，目前必须是nv12
	displayParam.frm_type = 0;
	//要显示数据的分辨率
	displayParam.frm_width = rgb_width;
	displayParam.frm_height = rgb_height;
	//创建显示类实例
	LibDisplay *display[tk_inputcamera_number] = {NULL};
	for(int i=0; i<tk_inputcamera_number; i++)
	{
		display[i] = new LibDisplay(i);
	}
	#if 1
	//初始化显示类实例
	for (int i = 0; i < 8; i++)
	{
		DEBUG("====>%s(%d): ============>\n", __func__, __LINE__);	
		ret = display[0]->init_display( buffParam, displayParam );
		if(ret != 0)
		{
			printf("display->init_display 0 failed\n");
			usleep(1000000);
			continue;
		}
		else{
			DEBUG("display->init_display 0 successed\n");
		}

		displayParam.x_offset = 0;
		displayParam.y_offset = 540;
		ret = display[1]->init_display( buffParam, displayParam );
		if(ret != 0)
		{
			printf("display->init_display 1 failed\n");
			usleep(1000000);
			continue;
		}
		else{
			DEBUG("display->init_display 1 successed\n");
		}

		#if 1
		displayParam.x_offset = 960;
		displayParam.y_offset = 0;
		ret = display[2]->init_display( buffParam, displayParam );
		if(ret != 0)
		{
			printf("display->init_display 2 failed\n");
			usleep(1000000);
			continue;
		}
		else{
			DEBUG("display->init_display 2 successed\n");
		}
		displayParam.x_offset = 960;
		displayParam.y_offset = 540;
		ret = display[3]->init_display( buffParam, displayParam );
		if(ret != 0)
		{
			printf("display->init_display 3 failed\n");
			usleep(1000000);
			continue;
		}
		else{
			DEBUG("display->init_display 3 successed\n");
		}
		#endif
		initflag = 1;
		break;
	}
	/***********************显示类对象初始化**********************/
	if (initflag != 1)
	{
		ret = tfailure;
		goto exit;
	}
	#endif
	for (int j = 0; j < 4; j++)
	//for (int j = 0; j < 2; j++)
	{
		g_displaylist[j].m_init = 1;
		g_displaylist[j].p_display = display[j];
	}

	//创建显示线程
	pthread_attr_t pthread_attr_display;
	pthread_t tk_display_pth;
	pthread_attr_init( &pthread_attr_display );
	pthread_attr_setscope( &pthread_attr_display, PTHREAD_SCOPE_SYSTEM );
	pthread_attr_setdetachstate( &pthread_attr_display, PTHREAD_CREATE_DETACHED );
	pthread_create( &tk_display_pth, &pthread_attr_display, display_hander_thread0, NULL );
	pthread_attr_destroy( &pthread_attr_display);
	
	pthread_t tk_display_pth1;
	pthread_create( &tk_display_pth1, &pthread_attr_display, display_hander_thread1, NULL );
	ret = tsuccess;
	
exit:
	return ret;
}

int tx2_server_init()
{
	int ret = tfailure;
	ret = tx2_server_tcpinit();
	if (tsuccess != ret)
	{
		tk_printf(0, "%s(%d): tx2_server_tcpinit error\n", __func__, __LINE__);
		goto exit;
	}

	ret = tx2_server_decsinit();
	if (tsuccess != ret)
	{
		tk_printf(0, "%s(%d): tx2_server_decsinit error\n", __func__, __LINE__);
		goto exit;
	}

	ret = tx2_server_syncinit();
	if (tsuccess != ret)
	{
		tk_printf(0, "%s(%d): tx2_server_syncinit error\n", __func__, __LINE__);
		//goto exit;
	}
	ret = tx2_server_virtdevinit();
	if (tsuccess != ret)
	{
		tk_printf(0, "%s(%d): tx2_server_virtdevinit error\n", __func__, __LINE__);
		//goto exit;
	}

	//显示初始化
	ret = tx2_display_init();
	if (tsuccess != ret)
	{
		tk_printf(0, "%s(%d): tx2_display_init error\n", __func__, __LINE__);
		//goto exit;
	}
	
	ret = tx2_server_saveinit("/media/nvidia/yuanrundisk");//("/media/nvidia/yuanrundisk");
	if (tsuccess != ret)
	{
		tk_printf(0, "%s(%d): tx2_server_saveinit error\n", __func__, __LINE__);
	}
	
exit:
	return ret;
}

int tx2_client_init()
{

	int ret = -1;

	//signal( SIGINT, signalhandler );	
	//signal( SIGQUIT, signalhandler );
	//signal( SIGSEGV, signalhandler );

	pthread_attr_t pthread_attr_client;
	pthread_t client_pth;
	pthread_attr_init( &pthread_attr_client );
	pthread_attr_setscope( &pthread_attr_client, PTHREAD_SCOPE_SYSTEM );
	pthread_attr_setdetachstate( &pthread_attr_client, PTHREAD_CREATE_DETACHED );
	pthread_create( &client_pth, &pthread_attr_client, client_thread, NULL );
	pthread_attr_destroy( &pthread_attr_client );	


	pthread_attr_t pthread_attr_net;
	pthread_t net_pth;
	pthread_attr_init( &pthread_attr_net );
	pthread_attr_setscope( &pthread_attr_net, PTHREAD_SCOPE_SYSTEM );
	pthread_attr_setdetachstate( &pthread_attr_net, PTHREAD_CREATE_DETACHED );
	pthread_create( &net_pth, &pthread_attr_net, tx2_server_netthread, NULL );
	pthread_attr_destroy( &pthread_attr_net );

	//配置日志自动检测路径，路径具体到系统日志和算法日志目录的上一级
	//系统日志目录logs，算法日志目录res，日志目录下包含时间目录
	//系统日志命名log_年-月-日-时-分-秒.txt
	//算法日志命令res_年-月-日-时-分-秒.txt
	//系统日志和算法日志每隔15分钟存储一个文件，线程检测5分钟没有修改文件开始上传
	//client_setlogpath( ( char * )"/home/firefly/Desktop/", 22 );
	////client_setlogpath( ( char * )path, strlen(path) );
	//printf("client_setlogpath psth is %s\n", path);	


	//设置下载的回调函数，该函数在下载完成以后回调
	////client_setdownloadcallback( update );


}


int main()
{
	int ret = tfailure;

	//初始化服务端
	tx2_server_init();

	ret = Tcps_Algo_Init();
	if (0 == ret)
	{
		algo_init = 1;
	}

	tx2_client_init();
	
	while (1)
	{
		sleep(10);
	}
	
	return 0;
}



