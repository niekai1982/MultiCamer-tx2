#include"coordinate_transform.h"



float tcps3_K122[3][3] = {  9.9804037600314799e-01, -6.0945884799619016e-02,
       -1.4177693517865443e-02, 6.1462402176468652e-02,
       9.9733107522743469e-01, 3.9409383456062215e-02,
       1.1738014576280653e-02, -4.0203550983470317e-02,
       9.9912256180216774e-01 };


Mat	tcps3_T122 = (cv::Mat_<float>(3, 1) << 1.0560722736157956e+02, -1.4804797781279985e+03, -2.6489422458227637e+01);


void Algorithm_depthtoworld(TrackingBox &det, int dx, int dy, unsigned short int dvalue, float *wx, float *wy, float *wz)
{
	static float tcps_cuInv, tcps_cvInv, tcps_fxInv, tcps_fyInv;
	if (det.camera_id == 0)
	//if(devnum == 0)
	{
		tcps_cuInv = 6.3509456777660114e+02; tcps_cvInv = 4.3060213472817173e+02;
		tcps_fxInv = 9.0947478592892082e+02; tcps_fyInv = 9.1075255678243877e+02;
	}
	if (det.camera_id == 1)
	//if(devnum == 1)
	{
		//tcps
		tcps_cuInv = 312.582916/2.0; tcps_cvInv = 235.685043/2.0;
		tcps_fxInv = 455.217987/2.0; tcps_fyInv = 455.804626/2.0;
		//beijing
		/*tcps_cuInv = 6.4059414761453615e+02; tcps_cvInv = 3.7929279571647123e+02;
		tcps_fxInv = 9.1248483025281553e+02; tcps_fyInv = 9.1389557858222668e+02;*/
	}
	if (det.camera_id == 2)
	//if(devnum == 2)
	{
		//tcps
		tcps_cuInv = 327.327209/2.0; tcps_cvInv = 211.666229/2.0;
		tcps_fxInv = 455.438690/2.0; tcps_fyInv = 455.806732/2.0;
		/*tcps_cuInv = 6.4003950142410702e+02; tcps_cvInv = 3.7221583331232364e+02;
		tcps_fxInv = 9.0621766910772817e+02; tcps_fyInv = 9.0734169576025397e+02;*/

	}


	float z = dvalue;
	//*wx = ((dx - tcps_cuInv) * z / tcps_fxInv) * 20;
	//*wy = ((dy - tcps_cvInv) * z / tcps_fyInv) * 20;
	*wx = ((dx - tcps_cuInv) * z / tcps_fxInv);
	*wy = ((dy - tcps_cvInv) * z / tcps_fyInv);
	*wz = z;
	//return 0;
}

void Algorithm_worldtodepth(TrackingBox &det, float wx, float wy, float wz, float *dx, float *dy, float *dz)
{
	static float tcps_cuInv, tcps_cvInv, tcps_fxInv, tcps_fyInv;
	if (det.camera_id == 0)
	{
		tcps_cuInv = 6.3509456777660114e+02; tcps_cvInv = 4.3060213472817173e+02;
		tcps_fxInv = 9.0947478592892082e+02; tcps_fyInv = 9.1075255678243877e+02;
	}
	if (det.camera_id == 1)
	{
		//tcps
		tcps_cuInv = 312.582916/2.0; tcps_cvInv = 235.685043/2.0;
		tcps_fxInv = 455.217987/2.0; tcps_fyInv = 455.804626/2.0;
		/*tcps_cuInv = 6.4059414761453615e+02; tcps_cvInv = 3.7929279571647123e+02;
		tcps_fxInv = 9.1248483025281553e+02; tcps_fyInv = 9.1389557858222668e+02;*/
	}
	if (det.camera_id == 2)
	{
		//tcps
		tcps_cuInv = 327.327209/2.0; tcps_cvInv = 211.666229/2.0;
		tcps_fxInv = 455.438690/2.0; tcps_fyInv = 455.806732/2.0;
		/*tcps_cuInv = 6.4003950142410702e+02; tcps_cvInv = 3.7221583331232364e+02;
		tcps_fxInv = 9.0621766910772817e+02; tcps_fyInv = 9.0734169576025397e+02;*/
	}
	*dz = wz;
	float idx = (wx * (tcps_fxInv / wz)) + tcps_cuInv;
	float idy = (wy * (tcps_fyInv / wz)) + tcps_cvInv;
	//*dx = idx / 20;
	//*dy = idy / 20;
	*dx = idx;
	*dy = idy;



}


void Depth2Word(TrackingBox &dets, Mat depth_input, int width, int longsize, int hight)
{

	int num = 0;
	float mean_z = 0;
	for (int i = 0; i < 15; i++)
	{
		for (int j = 0; j < 15; j++)
		{
			float z = depth_input.at<ushort>(int(dets.box.center.y)+j, int(dets.box.center.x)+i);
			if (z > 0)
			{
				num++;
				mean_z += z;

			}
			
		}
	
	}
	int value_depth = mean_z / num;
	
	float pWorldX, pWorldY, pWorldZ;
	
	Algorithm_depthtoworld(dets, int(dets.box.center.x), int(dets.box.center.y), value_depth, &pWorldX, &pWorldY, &pWorldZ);
	
	dets.box.center.x = int(pWorldX);
	dets.box.center.y = int(pWorldY);
	dets.box.center.z = pWorldZ;
	dets.box.size.x = width;
	dets.box.size.y = longsize;
	dets.box.size.z = hight;
	dets.id = -1;
	
}

void world2depth(TrackingBox &box3d)
{
	float pWorldX = box3d.box.center.x, pWorldY = box3d.box.center.y, pWorldZ = box3d.box.center.z;
	float pWorld_LminX = box3d.box.center.x-box3d.box.size.x/2, pWorld_LminY = box3d.box.center.y-box3d.box.size.y / 2, pWorld_LminZ = box3d.box.center.z;
	float pWorld_RmaxX = box3d.box.center.x+ box3d.box.size.x / 2, pWorld_RmaxY = box3d.box.center.y+ box3d.box.size.y / 2, pWorld_RmaxZ = box3d.box.center.z;

	float p0_DepthX=0, p0_DepthY=0, p0_DepthZ=0;
	float p0_Depth_LminX = 0, p0_Depth_LminY = 0, p0_Depth_LminZ = 0;
	float p0_Depth_RmaxX = 0, p0_Depth_RmaxY = 0, p0_Depth_RmaxZ = 0;
	Algorithm_worldtodepth(box3d, pWorldX, pWorldY, pWorldZ, &p0_DepthX, &p0_DepthY, &p0_DepthZ);
	Algorithm_worldtodepth(box3d, pWorld_LminX, pWorld_LminY, pWorld_LminZ, &p0_Depth_LminX, &p0_Depth_LminY, &p0_Depth_LminZ);
	Algorithm_worldtodepth(box3d, pWorld_RmaxX, pWorld_RmaxY, pWorld_RmaxZ, &p0_Depth_RmaxX, &p0_Depth_RmaxY, &p0_Depth_RmaxZ);
	box3d.box.center.x = p0_DepthX;
	box3d.box.center.y = p0_DepthY;
	box3d.box.center.z = p0_DepthZ;

	box3d.box.size.x = p0_Depth_RmaxX- p0_Depth_LminX;
	box3d.box.size.y = p0_Depth_RmaxY- p0_Depth_LminY;
	

}

void world_union_coor(int devid, vector<TrackingBox> &dets)
{
	float wu_x = 0, wu_y = 0, wu_z = 0;

	Mat tcps3_R122(cv::Size(3, 3), CV_32FC1, tcps3_K122);
	
	if (dets.size() >0)
	{
		for (int i = 0; i < dets.size(); i++)
		{
			Mat w2, w3;
			Mat WC = (cv::Mat_<float>(3, 1) << dets[i].box.center.x, dets[i].box.center.y, dets[i].box.center.z);
			
			if (devid == 1)
			{
				w2  =tcps3_R122*WC + tcps3_T122;
			}
			else if (devid == 2)
			{
				w2 = WC;
			}
			
			//同一世界坐标系下的
			dets[i].box.center.x = w2.at<float>(0, 0);
			dets[i].box.center.y = w2.at<float>(1, 0);
			dets[i].box.center.z = w2.at<float>(2, 0);
			
		}	
	}
}

vector<TrackingBox>  remove_closer_WorldPoint(vector<TrackingBox> dets, float dist_thrs)
{
	vector<int> a(dets.size(), 0);//标志位
	vector<TrackingBox> dets_1;
	if (dets.size() > 1)
	{

		double d = 0;
		TrackingBox tmp;
		for (int i = 0; i<dets.size(); i++)
		{
			if (a[i] > 0)
			{
				continue;
			}
			else
			{
				a[i] = 1;
				//dets_1.push_back(dets[i]);
				//cout << "new_dets:" << dets[i].box.center << endl;
			}
			for (int j = i + 1; j<dets.size(); j++)
			{
				Rect_<float> bb_test; Rect_<float> bb_gt;
				bb_test.x = dets[i].box.center.x;
				bb_test.y = dets[i].box.center.y;
				bb_test.width = dets[i].box.size.x;
				bb_test.height = dets[i].box.size.y;

				bb_gt.x = dets[j].box.center.x;
				bb_gt.y = dets[j].box.center.y;
				bb_gt.width = dets[j].box.size.x;
				bb_gt.height = dets[j].box.size.y;
				d = GetIOU(bb_test, bb_gt);
				//cout << "删除的IOU=" << d << " ,gt=" << bb_gt << " ,tes=" << bb_test << endl;
				//d = sqrt(pow(((dets[i]).box.center.x - (dets[j]).box.center.x), 2) + pow(((dets[i].box.center.y - (dets[j]).box.center.y)), 2));
				//cout << "distance of"<< dets[i].box.center <<" and "<< dets[j].box.center<<" is:" << d << endl;
				if (d >dist_thrs)
				{
					//a[j] = 1;
#if 0
					//max value
					if (dets[j].box.center.z>dets[i].box.center.z)
					{
						a[j] = 1;
						a[i] = 2;
					}
					else
					{
						a[i] = 1;
						a[j] = 2;
					}
# endif					
					//mean value
					if (dets[j].box.center.z>dets[i].box.center.z)
					{
						a[j] = 1;
						a[i] = 2;
						dets[j].box.center.x = (dets[j].box.center.x + dets[i].box.center.x) / 2;
						dets[j].box.center.y = (dets[j].box.center.y + dets[i].box.center.y) / 2;
						dets[j].box.center.z = (dets[j].box.center.z + dets[i].box.center.z) / 2;
					}
					else
					{
						a[i] = 1;
						a[j] = 2;
						dets[i].box.center.x = (dets[j].box.center.x + dets[i].box.center.x) / 2;
						dets[i].box.center.y = (dets[j].box.center.y + dets[i].box.center.y) / 2;
						dets[i].box.center.z = (dets[j].box.center.z + dets[i].box.center.z) / 2;
					}


				}
			}

		}

		for (int i = 0; i < a.size(); i++)
		{
			if (a[i] == 1)
			{
				dets_1.push_back(dets[i]);

			}
		}
	}
	else
	{
		dets_1 = dets;
	}
	return dets_1;
}
void unworld_union_coor(TrackingBox &det)
{
	float wu_x = 0, wu_y = 0, wu_z = 0;


	Mat tcps3_R122(cv::Size(3, 3), CV_32FC1, tcps3_K122);

	Mat tcps3_R122_invert;
	
	invert(tcps3_R122, tcps3_R122_invert);

	Mat w2;
	Mat UWC = (cv::Mat_<float>(3, 1) << det.box.center.x, det.box.center.y, det.box.center.z);
	//if (det.camera_id == 0)
	//{
		//w2 = R021_invert*((R122_invert*(UWC - T122)) - T021);
	//}
	 if (det.camera_id == 1)
	{
		w2 = tcps3_R122_invert*(UWC - tcps3_T122);
	
	}
	else if (det.camera_id == 2)
	{
		w2 = UWC;
	}


	det.box.center.x = w2.at<float>(0, 0);
	det.box.center.y = w2.at<float>(1, 0);
	det.box.center.z = w2.at<float>(2, 0);

}


