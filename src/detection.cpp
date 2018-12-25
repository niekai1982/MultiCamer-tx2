#include"detection.h"
#include"common.h"

int frame_count = 0;
int total_frames = 0;
Mat mat_resize16(Mat src, int width_ration, int height_ration)
{
	Mat dst(src.rows / width_ration, src.cols / height_ration, src.type());

	for (int i = 0; i < dst.rows; i++)
	{
		ushort *data_dst = dst.ptr<ushort>(i);
		int a = log(width_ration) / log(2);
		ushort *data_src = src.ptr<ushort>(i << a);
		for (int j = 0; j < dst.cols; j++)
		{
			int b = log(height_ration) / log(2);
			data_dst[j] = data_src[j << a];
		}
	}
	return dst;
}

void d2vector2Mat(vector< vector<float> > src, Mat & dst, int type)
{
	Mat temp(src.size(), src.at(0).size(), type);
	for (int i = 0; i < temp.rows; ++i)
	{
		uchar* data = temp.ptr<uchar>(i);
		for (int j = 0; j<temp.cols; ++j)
		{
			data[j] = src[i][j];
		}
	}
	temp.copyTo(dst);

}
Mat d1vector2Mat(vector<float> data, int datamat_row, int type)
{

	Mat datamat(data.size(), datamat_row, type);
	for (int i = 0; i < data.size(); i++)
	{
		uchar* a = datamat.ptr<uchar>(i);
		for (int j = 0; j < datamat_row; j++)
		{
			a[j] = data[i];
		}

	}

	return datamat.t();

}
vector<Point>  remove_peak(MaxInf &maxinf, int dist_thrs)
{
	vector<int> a(maxinf.maxLocs.size(), 0);
	vector<Point> local_max_coor_1;
	if (maxinf.maxLocs.size() > 1)
	{

		double d = 0;
		Point tmp;
		for (int i = 0; i<maxinf.maxLocs.size(); i++)
		{
		
			for (int j = i + 1; j<maxinf.maxLocs.size(); j++)
			{
				d = sqrt(pow(((maxinf.maxLocs[i]).x - (maxinf.maxLocs[j]).x), 2) + pow(((maxinf.maxLocs[i].y - (maxinf.maxLocs[j]).y)), 2));

				if (d <dist_thrs)
				{
					maxinf.maxValues[i] > maxinf.maxValues[j] ? a[j] = 1 : a[i] = 1;
				}
			}


		}
	}
	for (int k = 0; k < a.size(); k++)
	{
		if (a[k] == 0)
		{
			local_max_coor_1.push_back(maxinf.maxLocs[k]);
		}
	}
	return local_max_coor_1;
}
void peak_local_max(Mat src, const int minPeakDistance, const double minHeightDiff, const double minPeakHeight, MaxInf &max_inf/*vector<Point> &maxLocs_new*/)
{
	
	Mat src_padding;
	//note padding use 0 or 255 or not use padding 
	copyMakeBorder(src, src_padding, minPeakDistance, minPeakDistance, minPeakDistance, minPeakDistance, BORDER_CONSTANT, 0);
	
	int minLoc = 0;
	double minVal = 0, maxVal = 0, diff = 0;
	Point minLocPt, maxLocPt;

	Mat m(src_padding);
	
	for (int i = minPeakDistance; i < src_padding.rows - minPeakDistance; i++)
	{
		uchar* data = src_padding.ptr<uchar>(i);

		for (int j = minPeakDistance; j < src_padding.cols - minPeakDistance; j++)
		{

			Mat m_part = m(Range(i - minPeakDistance, i + minPeakDistance), Range(j - minPeakDistance, j + minPeakDistance));//获取[Loc-kernelSize,Loc+kernelSize]范围内数据
			minMaxLoc(m_part, &minVal, &maxVal, &minLocPt, &maxLocPt);//计算最大值位置

			int value = data[j];
			//if (int(maxVal) != minVal && (int(maxVal) == value))
			if (int(maxVal) != 0 && (int(maxVal) == value))
			{
				maxLocPt = Point(j - minPeakDistance, i - minPeakDistance);
				
				if (!(maxLocPt.y == minPeakDistance && maxVal > minPeakHeight &&
					maxVal - minVal > minHeightDiff && maxVal - minVal > diff))
				{
					max_inf.maxLocs.push_back(maxLocPt);
					max_inf.maxValues.push_back(maxVal);
					
				}
			}
		}
	}
	

}

vector<TrackingBox> seed_local_maximum(Mat input_data, int min_distance, bool exclude_border = false, int h_thr = 10, int region_size = 5)
{
	vector<TrackingBox> local_max_coor_M;
	Mat f;
	f = input_data.clone();
	int dist_thrs = 10;
	MaxInf max_inf;

	peak_local_max(f, min_distance, 3, 500, max_inf);//local_max_coor ->f
	vector<Point> local_max_coor = remove_peak(max_inf, dist_thrs);
	//vector<Point> local_max_coor = max_inf.maxLocs;
	for (auto idx_point = local_max_coor.crbegin(); idx_point != local_max_coor.crend(); idx_point++)

	{

		int value = f.at<char>((*idx_point).y, (*idx_point).x);
		//region_size = (uchar)log2(value);
		//region_size = 5 * value;
		Point top_left(max(0, (*idx_point).x - region_size), max(0, (*idx_point).y - region_size));
		Point buttom_right(min(input_data.cols, (*idx_point).x + region_size), min(input_data.rows, (*idx_point).y + region_size));
		//cout << top_left.x <<":"<< top_left.y << endl;
		//cout << buttom_right.x << ":" << buttom_right.y << endl;

		Mat d = f(Range(top_left.y, buttom_right.y), Range(top_left.x, buttom_right.x));
		vector<vector<float>> all_key_points;
		vector<float> cent;
		cent.push_back((*idx_point).y);
		cent.push_back((*idx_point).x);
		cent.push_back(f.data[(*idx_point).x + ((*idx_point).y)*f.cols]);
		for (int i = 0; i<d.rows; i++)
		{
			uchar* data = d.ptr<uchar>(i);
			for (int j = 0; j<d.cols; j++)
			{
				if ((data[j] >(f.data[(*idx_point).x + ((*idx_point).y)*f.cols] - h_thr)) && (data[j] <= (f.data[(*idx_point).x + ((*idx_point).y)*f.cols])))
				{
					vector<float> key_points;
					key_points.push_back(i + top_left.y);//neki 
					key_points.push_back(j + top_left.x);
					key_points.push_back(data[j]);
					all_key_points.push_back(key_points);
				}
			}
		}

		//cout <<"all_key_points.size() :"<< all_key_points.size() << endl;
		if (all_key_points.size() > 0)
		{
			Mat res;
			d2vector2Mat(all_key_points, res, CV_8UC1);
			res.convertTo(res, CV_32FC1);
			int cent_row = res.rows;
			//Mat cent = Mat((*idx_point).x, (*idx_point).y, f.at<float>((*idx_point).x, (*idx_point).y));
			Mat centmat;
			centmat = d1vector2Mat(cent, cent_row, CV_8UC1);
			centmat.convertTo(centmat, CV_32FC1);

			for (int i = 0; i < 10; i++)
			{
				Mat d(3, cent_row, CV_32FC1);
				subtract(res, centmat, d);

				float sz[3] = { 3,5,10 };

				Mat val;
				val.create(cent_row, 3, CV_32FC1);
				float *val_ptr = (float*)val.data;
				float *d_ptr = (float*)d.data;
				for (int j = 0; j < d.rows; j++)
				{
					val_ptr[j * 3] = (1 / (sqrt(2 * PI)*sz[0])) * exp(-0.5 * pow(((d_ptr[j * 3]) / sz[0]), 2));;

					val_ptr[j * 3 + 1] = (1 / (sqrt(2 * PI)*sz[1]))*exp(-0.5*pow(((d_ptr[j * 3 + 1]) / sz[1]), 2));

					val_ptr[j * 3 + 2] = (1 / (sqrt(2 * PI)*sz[2]))*exp(-0.5*pow(((d_ptr[j * 3 + 2]) / sz[2]), 2));

				}
				Mat val_res = val.mul(res);
				//Mat sum_c_res(res.cols,1,CV_8UC1);
				Mat sum_r_val(1, val.rows, CV_8UC1);
				Mat sum_val_res(1, val_res.rows, CV_8UC1);
				//reduce(res,sum_c_res,1,CV_REDUCE_SUM);
				reduce(val, sum_r_val, 0, CV_REDUCE_SUM);
				reduce(val_res, sum_val_res, 0, CV_REDUCE_SUM);
				Mat mx;
				divide(sum_val_res, sum_r_val, mx);
				cent = mx;


			}
			// error
			TrackingBox bbox;
			bbox.box.center.x = cent[0];
			bbox.box.center.y = cent[1];
			local_max_coor_M.push_back(bbox);
		}


	}
	return local_max_coor_M;
}

vector<TrackingBox>  remove_closer_point(vector<TrackingBox> &local_max_coor, int dist_thrs)
{
	vector<int> a(local_max_coor.size(), 0);
	vector<TrackingBox> local_max_coor_1;
	if (local_max_coor.size() > 1)
	{

		double d = 0;
		TrackingBox tmp;
		for (int i = 0; i<local_max_coor.size(); i++)
		{
			if (a[i] > 0)
			{
				continue;
			}
			else
			{
				a[i] = 1;
				local_max_coor_1.push_back(local_max_coor[i]);
			}
			for (int j = i + 1; j<local_max_coor.size(); j++)
			{
				d = sqrt(pow(((local_max_coor[i]).box.center.x - (local_max_coor[j]).box.center.x), 2) + pow(((local_max_coor[i].box.center.y - (local_max_coor[j]).box.center.y)), 2));

				if (d <dist_thrs)
				{
					a[j] = 1;
					//local_max_coor.erase(it);
					//vector<TrackingBox>(local_max_coor).swap(local_max_coor);
				}
			}

		}
	}

	return local_max_coor_1;
}

void frame_zone5_max_point(Mat depth_input, int camera_id, parameters parameter, Mat result, vector<TrackingBox> &dets)
{
	int down_scale[2];
	down_scale[0] = 16;
	down_scale[1] = 12;

	Mat depth_resize;
	depth_resize = mat_resize16(depth_input, down_scale[0], down_scale[0]);

	
	for (int i = 0; i < depth_resize.rows; i++)
	{
		ushort* data = depth_resize.ptr<ushort>(i);
		for (int j = 0; j < depth_resize.cols; j++)
		{
			if (data[j] > parameter.depth_RH)
			{
				data[j] = 0;
			}

		}
	}
	for (int i = 0; i < depth_resize.rows; i++)
	{
		ushort* data = depth_resize.ptr<ushort>(i);
		for (int j = 0; j < depth_resize.cols; j++)
		{

			if (data[j] > 0)
			{
				data[j] = parameter.depth_RH - data[j];
			}
		}
	}

	
	for (int i = 0; i < depth_resize.rows; i++)
	{
		ushort* data = depth_resize.ptr<ushort>(i);
		for (int j = 0; j < depth_resize.cols; j++)
		{
			data[j] = (data[j] - parameter.depth_RL) * 255 / parameter.depth_RH;
		}
	}
	depth_resize.convertTo(depth_resize, CV_8UC1);
	//LOG("depth_resize ....");


	//fliter    # preprocess
	Mat gaussian_fliter;
	Mat  depth_m;
	GaussianBlur(depth_resize, gaussian_fliter, Size(3, 3), 1, 0);
	medianBlur(gaussian_fliter, depth_m, 5);
	//填补空洞
	for (int i = 0; i < gaussian_fliter.rows; i++)
	{
		uchar *data = gaussian_fliter.ptr<uchar>(i);
		uchar *data_m = depth_m.ptr<uchar>(i);
		for (int j = 0; j < gaussian_fliter.cols; j++)
		{
			if (data[j] < 30)
			{
				data[j] = data_m[j];
			}
		}
	}

	for (int i = 0; i < gaussian_fliter.rows; i++)
	{
		uchar *data = gaussian_fliter.ptr<uchar>(i);
		for (int j = 0; j < gaussian_fliter.cols; j++)
		{
			if (data[j] < parameter.depth_value)
			{
				data[j] = 0;
			}
		}
	}

	//LOG("local_max_coor start ....");
	vector<TrackingBox> local_max_coor;
	vector<TrackingBox> local_max_coor_tmp;
	int min_distance = 3;
	//LOG("start fingding local maximum point ......");
	local_max_coor = seed_local_maximum(gaussian_fliter, min_distance = 3);
	//LOG("local_max_coor end ....");
	if (local_max_coor.size()>1)
	{
		//LOG("remove_closer_point ......");
		int dist_thrs;
		local_max_coor = remove_closer_point(local_max_coor, dist_thrs = 2);
	}
	if (local_max_coor.size() > 0)
	{
		int id = -1;
		TrackingBox box;

		for (auto it = local_max_coor.begin(); it != local_max_coor.end(); it++)
		{
			if ((*it).box.center.y*down_scale[0] >15 && (*it).box.center.x*down_scale[0] >15 && (*it).box.center.y*down_scale[0] <(depth_input.cols - 15) && (*it).box.center.x*down_scale[0] < (depth_input.rows - 15))
			{
				id++;
				box.box.center.x = cvFloor((*it).box.center.y*down_scale[0]);
				box.box.center.y = cvFloor((*it).box.center.x*down_scale[0]);
				box.box.center.z = depth_input.at<ushort>(box.box.center.y, box.box.center.x);
				box.camera_id = camera_id;
				box.pro = -1;
				Point p4 = Point(cvFloor(box.box.center.x), cvFloor(box.box.center.y));
				circle(result, p4, 2, Scalar(0, 120, 120), -1);
				rectangle(result, Rect2d(box.box.center.x - 20, box.box.center.y - 20, 40, 40), Scalar(0, 0, 0), 3);
				//cout << "det:" << box.box.center << endl;
				dets.push_back(box);


			}

		}
	}

	depth_input.release();
}
#if 0
Mat unshort2show(Mat depth)
{
	Mat IF = depth.clone();
	for (int i = 0; i < IF.rows; i++)
	{
		ushort* data = IF.ptr<ushort>(i);
		for (int j = 0; j < IF.cols; j++)
		{
			if (data[j] > 3000)
				data[j] = 0;
		}
	}
	for (int i = 0; i < IF.rows; i++)
	{
		ushort *data = IF.ptr<ushort>(i);
		for (int j = 0; j < IF.cols; j++)
		{
			if (data[j] != 0)
			{
				data[j] = 3000 - data[j];
			}

		}
	}

	double minVal = 0, maxVal = 0, diff = 0;
	Point minLocPt, maxLocPt;


	for (int i = 0; i < IF.rows; i++)
	{
		ushort* data = IF.ptr<ushort>(i);
		for (int j = 0; j < IF.cols; j++)
		{
			data[j] = (data[j] - 0) * 255 / 5000;
		}
	}

	IF.convertTo(IF, CV_8UC1);

	return IF;


}
#endif
