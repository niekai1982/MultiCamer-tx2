#include "kalmantracker.h"
int KalmanBoxTracker::count = 0;

//Computes IUO 
double iou(vector<double> bb_test, vector<double> bb_gt)
{
	double xx1 = max(bb_test[0], bb_gt[0]);
	double yy1 = max(bb_test[1], bb_gt[1]);
	double xx2 = min(bb_test[2], bb_gt[2]);
	double yy2 = min(bb_test[3], bb_gt[3]);
	double w = max(0.0, (xx2 - xx1));
	double h = max(0.0, (yy2 - yy1));
	double wh = w * h;
	double o = wh / ((bb_test[2] - bb_test[0]) * (bb_test[3] - bb_test[1]) + (bb_gt[2] - bb_gt[0]) * (bb_gt[3] - bb_gt[1]) - wh);
	return o;

}


void KalmanBoxTracker::init_kf(StateType_mix stateMat)
{
	int stateNum = 9;
	int measureNum = 5;
	int controlNum = 5;

	kf = KalmanFilter(stateNum, measureNum, controlNum);

	// Transition State Matrix A :7X7
	kf.transitionMatrix = (Mat_<float>(stateNum, stateNum) <<
		1, 0, 0, 0, 0, 1, 0, 0, 0,
		0, 1, 0, 0, 0, 0, 1, 0, 0,
		0, 0, 1, 0, 0, 0, 0, 1, 0,
		0, 0, 0, 1, 0, 0, 0, 0, 1,
		0, 0, 0, 0, 1, 0, 0, 0, 0,
		0, 0, 0, 0, 0, 1, 0, 0, 0,
		0, 0, 0, 0, 0, 0, 1, 0, 0,
		0, 0, 0, 0, 0, 0, 0, 1, 0,
		0, 0, 0, 0, 0, 0, 0, 0, 1);


	//cv::setIdentity(kf.transitionMatrix);
	//B
	kf.controlMatrix = (Mat_<float>(stateNum, stateNum) <<
		1, 0, 0, 0, 1, 0, 0, 0, 0,
		0, 1, 0, 0, 0, 1, 0, 0, 0,
		0, 0, 1, 0, 0, 0, 1, 0, 0,
		0, 0, 0, 1, 0, 0, 0, 1, 0,
		0, 0, 0, 0, 1, 0, 0, 0, 1,
		0, 0, 0, 0, 0, 1, 0, 0, 0,
		0, 0, 0, 0, 0, 0, 1, 0, 0,
		0, 0, 0, 0, 0, 0, 0, 1, 0,
		0, 0, 0, 0, 0, 0, 0, 0, 1);
	//cv::setIdentity(kf.controlMatrix);
	setIdentity(kf.measurementMatrix);//H
	setIdentity(kf.processNoiseCov, Scalar::all(1e-3));//Q
	setIdentity(kf.measurementNoiseCov, Scalar::all(1e-4));//R
	
	setIdentity(kf.errorCovPost, Scalar::all(100));//P
	measurement = Mat::zeros(measureNum, 1, CV_32F);
	kf.statePost.at<float>(0,0) = stateMat.box.center.x ;//wx
	kf.statePost.at<float>(1,0) = stateMat.box.center.y ;//wy
	kf.statePost.at<float>(2,0) = stateMat.box.center.z;//wz
	
	//kf.statePost.at<float>(2, 0) = stateMat.width*stateMat.height;
	kf.statePost.at<float>(3, 0) = stateMat.box.size.x * stateMat.box.size.y;//area
	kf.statePost.at<float>(4, 0) = stateMat.box.size.x/ stateMat.box.size.y;//ratio

	
}
void KalmanBoxTracker::Update(StateType_mix stateMat)
{ 
	time_since_update = 0;
	//history.clear();
	hits++;
	hit_streak++;

	

	measurement.at<float>(0, 0) = stateMat.box.center.x ;
	measurement.at<float>(1, 0) = stateMat.box.center.y ;
	measurement.at<float>(2, 0) = stateMat.box.center.z ;
	measurement.at<float>(3, 0) = stateMat.box.size.x * stateMat.box.size.y;;
	measurement.at<float>(4, 0) = stateMat.box.size.x / stateMat.box.size.y;
	trk_prob = stateMat.pro;
	camera_id = stateMat.camera_id;
	
	
	kf.correct(measurement);
	
	
}
StateType_mix KalmanBoxTracker::Predict()
{
	//Advances the state vector and returns the predicted bounding box estimate.
	if ((kf.statePost.at<float>(8) + kf.statePost.at<float>(3)) <= 0) //#面积和预测面积之和小于零，则面积为零
	{
		kf.statePost.at<float>(8) *= 0.0;
	}
   

	Mat p=kf.predict();

	
	age++;

	if (time_since_update > 0)
	{
		hit_streak = 0;
	}
	time_since_update += 1;

	StateType_mix predictBox;
		
	    predictBox.box.center.x =p.at<float>(0, 0);
		predictBox.box.center.y = p.at<float>(1, 0);
		predictBox.box.center.z = p.at<float>(2, 0);
		
		predictBox.box.size.x = sqrt(p.at<float>(3, 0)*p.at<float>(4, 0));
		predictBox.box.size.y = p.at<float>(3, 0)/ predictBox.box.size.x;
		

	    predictBox.pro = trk_prob;
		predictBox.id = id;
		predictBox.camera_id = camera_id;
	//[box,prob=0]
	history.push_back(predictBox);
	////add 2018-09-13
	//if (history.size() > 10)
	//{
	//	vector<StateType_mix>::iterator k = history.begin();
	//	history.erase(k);//删

	//}

	return history.back();


}
StateType_box KalmanBoxTracker::Get_state()
{
	StateType_box box3d;
	//Mat s = kf.statePost;
	Mat s = kf.statePost;
	box3d.center.x = s.at<float>(0, 0);
	box3d.center.y = s.at<float>(1, 0);
	box3d.center.z = s.at<float>(2, 0);
	box3d.size.x = sqrt(s.at<float>(3, 0)*s.at<float>(4, 0));
	box3d.size.y = s.at<float>(3, 0) / box3d.size.x;
	return box3d;

}

struct MatchResult
{
	vector<vector<int>> matches;
	vector<int> unmatched_detections, unmatched_trackers;
};



