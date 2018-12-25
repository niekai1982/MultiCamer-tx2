#pragma once

#include <opencv2/video/tracking.hpp>
#include <opencv2/highgui/highgui.hpp>
#include<iostream>
using namespace std;
using namespace cv;

typedef struct Box3D
{
	Point3f center;//center of the 3dbox
	Point3f size;
}Box3D;
typedef struct StateTypeBox
{
	
	Box3D box;
	float pro;
	int id;
	int camera_id;
}StateTypeBox;
//#define StateType Rect_<float>
#define StateType_mix StateTypeBox
#define StateType_box Box3D
#define MAX_COUNT 1000000
class KalmanBoxTracker
{
private:
	// >>>> Kalman Filter
	cv::KalmanFilter kf;
	cv::Mat measurement;
	

	void init_kf(StateType_mix stateMat);

public:
	std::vector<StateType_mix> history;
    static int count;//很重要，决定跟踪id的更新。
	int hits;
	int hit_streak;//某一个id连续被跟踪的个数
	int age;	
	int time_since_update;
    int id;
	int camera_id;
	float trk_prob;
	

	KalmanBoxTracker()
	{
		init_kf(StateType_mix());
		time_since_update = 0;
		hits = 0;
		hit_streak = 0;
		age = 2;
		id = count;
		trk_prob = 0;
		camera_id = -1;
	}
	KalmanBoxTracker(StateType_mix initRect)
	{
	
		init_kf(initRect);
		time_since_update = 0;
		hits = 0;
		hit_streak = 0;
		age = 0;
		id = count;
		trk_prob = 0;
		camera_id = initRect.camera_id;
		count++;
		if (count > MAX_COUNT)
		{
			count = 0;
		}
		//cout<<"cout:"
	}

	~KalmanBoxTracker()
	{
		history.clear();
	}

	//StateType convert_bbox_to_z(StateType x);
	//StateType_box convert_x_to_bbox(float cx, float cy, float s, float r);
	void Update(StateType_mix stateMat);
	StateType_mix Predict();
	StateType_box Get_state();

};