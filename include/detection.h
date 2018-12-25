#pragma once

#include <opencv2/core/core.hpp>  
#include <opencv2/highgui/highgui.hpp>  
#include <opencv2/imgproc/imgproc.hpp>
#include <iomanip>
#include <iostream>  
#include <stdio.h>
#include <fstream>
#include<string>
#include"sort.h"
#include"parameters.h"
using namespace std;
using namespace cv;

#define MAX_LINE 1024 
#define WIDTH  640
#define HEIGHT  400
const float PI = 3.14;

struct MaxInf
{
	vector<Point> maxLocs;
	vector<double> maxValues;
};

void d2vector2Mat(vector< vector<float> > src, Mat & dst, int type);

Mat d1vector2Mat(vector<float> data, int datamat_row, int type);

Mat mat_resize16(Mat src, int width_ration, int height_ration);

vector<Point>  remove_peak(MaxInf &maxinf, int dist_thrs);

void peak_local_max(Mat src, const int minPeakDistance, const double minHeightDiff, const double minPeakHeight, MaxInf &max_inf);

vector<TrackingBox>  remove_closer_point(vector<TrackingBox> &local_max_coor, int dist_thrs);

void frame_zone5_max_point(Mat depth_input, int camera_id, parameters parameter, Mat result, vector<TrackingBox> &dets);

vector<TrackingBox>  remove_closer_point(vector<TrackingBox> &local_max_coor, int dist_thrs);

Mat unshort2show(Mat depth);
//void frame_zone5_max_point(Mat depth_input, int camera_id, int depth_RL, int depth_RH, int depth_value, Mat result, vector<TrackingBox> &dets);
