#ifndef __SORT_H_
#define __SORT_H_

#include "opencv2/video/tracking.hpp"
#include "opencv2/highgui.hpp"
#include <set>
#include <stdio.h>

#include "Hungarian2.h"
#include "kalmantracker.h"


typedef struct TrackingBox
{
	int frame;
	int id;
	StateType_box box;
	//int camera_id[3];
	int camera_id;
	float pro;
}TrackingBox;
typedef struct AssRes
{
	vector<cv::Point> matchedPairs;
	set<int> unmatchedDetections;
	set<int> unmatchedTrajectories;
}AssRes;
typedef struct SortRes
{
	vector<cv::Point> matchedPairs;
	set<int> unmatchedDetections;
	set<int> unmatchedTrajectories;
	vector<vector<StateType_mix>> trks_hist;
	vector<vector<StateType_mix>> pop_trks_hist;
	vector<TrackingBox> frameTrackingResult;

}SortRes;
//
//static float tcps_cuInv = 320.5, tcps_cvInv = 229.5;
//static float tcps_fxInv = 455.9, tcps_fyInv = 456.4;




//static float tcps_cuInv = 313.703918, tcps_cvInv = 227.947052;
//static float tcps_fxInv = 0.002200, tcps_fyInv = 0.002199;


class Sort
{
private:	
public:
    int max_age;
	int min_hits;
	int frame_count;
	vector<KalmanBoxTracker> trackers;
	
	vector<TrackingBox> frame_dets;
	Sort()
	{
		 max_age = 5;
		 min_hits = 5;
		 frame_count = 0;
		 trackers.clear();
		 
		 
	}
   Sort(int age, int hits)
	{
		max_age = age;
		min_hits = hits;
		frame_count = 0;
		trackers.clear();


	}

	SortRes SortUpdate( vector<TrackingBox> frame_dets);
};
double GetIOU(Rect_<float> bb_test,Rect_<float> bb_gt);
#endif // !__SORT_H_
