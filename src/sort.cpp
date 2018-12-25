
#include"sort.h"
#include<iterator>


// Computes IOU between two bounding boxes

double GetIOU(Rect_<float> bb_test, Rect_<float> bb_gt)
{
	float in = (bb_test & bb_gt).area();
	float un = bb_test.area() + bb_gt.area() - in;

	if (un < DBL_EPSILON)
		return 0;

	return (double)(in / un);
}



double GetIOU2D(Box3D b3d_test, Box3D b3d_gt)
{
	Rect_<float> bb_test;
	Rect_<float> bb_gt;
	bb_test.x = b3d_test.center.x + b3d_test.size.x / 2;
	bb_test.y = b3d_test.center.y + b3d_test.size.y / 2;
	bb_test.width = b3d_test.size.x;
	bb_test.height = b3d_test.size.y;

	bb_gt.x = b3d_gt.center.x + b3d_gt.size.x / 2;
	bb_gt.y = b3d_gt.center.y + b3d_gt.size.y / 2;
	bb_gt.width = b3d_gt.size.x;
	bb_gt.height = b3d_gt.size.y;
	float in = (bb_test & bb_gt).area();
	float un = bb_test.area() + bb_gt.area() - in;

	if (un < DBL_EPSILON)
		return 0;
	return (double)(in / un);
}
double IOU3D(Box3D b3d_test, Box3D b3d_gt)
{
	//box3d b3d_test mimum
	float test_xl = b3d_test.center.x - b3d_test.size.x / 2;
	float test_yl = b3d_test.center.y - b3d_test.size.y / 2;
	float test_zl = b3d_test.center.z - b3d_test.size.z / 2;
	//box3d b3d_test maximum
	float test_xr = b3d_test.center.x + b3d_test.size.x / 2;
	float test_yr = b3d_test.center.y + b3d_test.size.y / 2;
	float test_zr = b3d_test.center.z + b3d_test.size.z / 2;
	float test_volume = b3d_test.size.x*b3d_test.size.y*b3d_test.size.z;
	
	//	//box3d b3d_gt mimum
	float gt_xl = b3d_gt.center.x - b3d_gt.size.x / 2;
	float gt_yl = b3d_gt.center.y - b3d_gt.size.y / 2;
	float gt_zl = b3d_gt.center.z - b3d_gt.size.z / 2;
	//box3d b3d_gt maximum
	float gt_xr = b3d_gt.center.x + b3d_gt.size.x / 2;
	float gt_yr = b3d_gt.center.y + b3d_gt.size.y / 2;
	float gt_zr = b3d_gt.center.z + b3d_gt.size.z / 2;
	float gt_volume = b3d_gt.size.x*b3d_gt.size.y*b3d_gt.size.z;
	
	//	cobox mimum
	float co_xl = max(test_xl, gt_xl);
	float co_yl = max(test_yl, gt_yl);
	float co_zl = max(test_zl, gt_zl);
	//cobox maximum
	float co_xr = min(test_xr, gt_xr);
	float co_yr = min(test_xr, gt_xr);
	float co_zr = min(test_xr, gt_xr);

	float in = (co_xr - co_xl)*(co_yr- co_yl)*(co_zr- co_zl);
	float un = test_volume + gt_volume - in;

	if (un < DBL_EPSILON)
		return 0;

	return (double)(in / un);
}



AssRes associate_detections_to_trackers(vector<TrackingBox> detFrameData, vector<Box3D> predictedBoxes, double iouThreshold = 0.2/*0.002*/)
{
	
	
	unsigned int trkNum = 0;
	unsigned int detNum = 0;
	
	vector<int> assignment;
	set<int> unmatchedDetections;
	set<int> unmatchedTrajectories;
	set<int> allItems;
	set<int> matchedItems;
	vector<cv::Point> matchedPairs;
	vector<KalmanBoxTracker> trackers;

	

	vector<TrackingBox> frameTrackingResult;
	AssRes associate;
	//vector<TrackingBox> frameTrackingResult;


    vector<vector<double>> iouMatrix;
	trkNum = predictedBoxes.size();//trk for one frame
	detNum = detFrameData.size();//each frame det

	iouMatrix.clear();
	iouMatrix.resize(trkNum, vector<double>(detNum, 0));
	
	for (unsigned int i = 0; i < trkNum; i++) // compute iou matrix 
	{
		for (unsigned int j = 0; j < detNum; j++)
		{
			
			// use 1-iou because the hungarian algorithm computes a minimum-cost assignment.
			
			iouMatrix[i][j] = 1 - GetIOU2D(predictedBoxes[i], detFrameData[j].box);
			//cout << "predictedBoxes[i]:" << predictedBoxes[i].center.x << " , " << predictedBoxes[i].center.y << " , " << predictedBoxes[i].center.z << endl;
			//cout << "detFrameData[i]:" << detFrameData[j].box.center.x << " , " << detFrameData[j].box.center.y << " , " << detFrameData[j].box.center.z << endl;
			//cout << "iou:" << GetIOU2D(predictedBoxes[i], detFrameData[j].box) << endl;
			
			//iouMatrix[i][j] = 1 - GetIOU(predictedBoxes[i], detFrameData[j].box);
		}
	}
	
	HungarianAlgorithm HungAlgo;
	assignment.clear();
	
	HungAlgo.Solve(iouMatrix, assignment);
	
	unmatchedTrajectories.clear();
	unmatchedDetections.clear();
	allItems.clear();
	matchedItems.clear();
	
	if (detNum > trkNum) //	there are unmatched detections
	{
		
		for (unsigned int n = 0; n < detNum; n++)
			allItems.insert(n);

		for (unsigned int i = 0; i < trkNum; ++i)
			matchedItems.insert(assignment[i]);

		set_difference(allItems.begin(), allItems.end(),
			matchedItems.begin(), matchedItems.end(),
			insert_iterator<set<int>>(unmatchedDetections, unmatchedDetections.begin()));
	}
	else if (detNum < trkNum) // there are unmatched trajectory/predictions
	{
		for (unsigned int i = 0; i < trkNum; ++i)
		if (assignment[i] == -1) // unassigned label will be set as -1 in the assignment algorithm
			unmatchedTrajectories.insert(i);
	}
	else
		;

	
	matchedPairs.clear();
	for (unsigned int i = 0; i < trkNum; ++i)
	{
		if (assignment[i] == -1) // pass over invalid values
			continue;
		
		if (1 - iouMatrix[i][assignment[i]] < iouThreshold)
		{
		
			
			if (float(1 - iouMatrix[i][assignment[i]]) == 0)
			{
				waitKey(10);
			}
			//1 - iouMatrix[i][assignment[i]]值越大交集越多，1表示完全重合
			unmatchedTrajectories.insert(i);
			unmatchedDetections.insert(assignment[i]);
		}
		else
		{
			
			matchedPairs.push_back(cv::Point(i, assignment[i]));
		}
			
	}

	associate.matchedPairs = matchedPairs;
	associate.unmatchedDetections = unmatchedDetections;
	associate.unmatchedTrajectories = unmatchedTrajectories;
	
	return associate;
}

vector<KalmanBoxTracker>  remove_closer_WorldPoint_trackers(vector<KalmanBoxTracker> &trackers, int dist_thrs)
{
	vector<int> a(trackers.size(), 0);
	vector<KalmanBoxTracker> trackers_1;
	if (trackers.size() > 1)
	{

		double d = 0;
		KalmanBoxTracker tmp;
		for (int i = 0; i<trackers.size(); i++)
		{
			if (a[i] > 0)
			{
				continue;
			}
			else
			{
				a[i] = 1;
				trackers_1.push_back(trackers[i]);
				
			}
			for (int j = i + 1; j<trackers.size(); j++)
			{
				d = sqrt(pow(((trackers[i]).Get_state().center.x - (trackers[j]).Get_state().center.x), 2) + pow(((trackers[i].Get_state().center.y - (trackers[j]).Get_state().center.y)), 2));
				
				if (d <dist_thrs)
				{
					if (trackers[i].age > trackers[j].age)
					{
						a[j] = 1;
					}
					
				}
			}

		}
	}
	else
	{
		trackers_1 = trackers;
	}
	return trackers_1;
}


SortRes Sort::SortUpdate(vector<TrackingBox> frame_dets)
{
	//cout << "in sort update" << endl;
	//cout << "frame_dets.size()="  <<frame_dets.size()<< endl;
	SortRes sortres;
	//cout << "sortres initi:" << endl;

	// update matched trackers with assigned detections.
	// each prediction is corresponding to a tracker
	frame_count += 1;
	static int iNoDetectNum = 0;
	AssRes associate;
	int detIdx, trkIdx;
	vector<Box3D> predictedBoxes;
	vector<TrackingBox> frameTrackingResult;

	
	vector<vector<StateType_mix>> trks_hist;
	vector<vector<StateType_mix>> pop_trks_hist;
	//cout<<"11111111111111"<<endl;
	if (frame_dets.size() != 0)
	{
	
		//cout << "dets with no trackers,initialize kalman trackers" << endl; 
		if (trackers.size() == 0) // the first frame met
		{
			//cout << "initialize kalman trackers using first detections.detFrameData all frame dets bbox" << endl;
			// initialize kalman trackers using first detections.detFrameData all frame dets bbox
			for (unsigned int i = 0; i < frame_dets.size(); i++)
			{
				StateType_mix dets;

				dets.box = frame_dets[i].box;
				//dets.pro = frame_dets[i].pro;
			
				KalmanBoxTracker trk = KalmanBoxTracker(dets);
															  //KalmanBoxTracker trk = KalmanBoxTracker(frame_dets[i].box);
				trk.trk_prob = frame_dets[i].pro;
				trk.camera_id = frame_dets[i].camera_id;
				
				trackers.push_back(trk);
			
				
			}
		
			for (unsigned int id = 0; id < frame_dets.size(); id++)
			{
				TrackingBox tb = frame_dets[id];
				
				sortres.frameTrackingResult.push_back(tb);
				//add 8-20
				//sortres.frameTrackingResult.push_back(fra);
				Point a = Point(id, id);
				sortres.matchedPairs.push_back(a);
			}
			
			return sortres;
			//continue;
		}

		else
		{
			predictedBoxes.clear();
			for (auto it = trackers.begin(); it != trackers.end();)
			{
				StateType_mix p_mix_box;
			
			
				p_mix_box = (*it).Predict();//kalman track,time_since_update++
				Box3D pBox = p_mix_box.box;

				predictedBoxes.push_back(pBox);
				it++;
				//cout << "sort predict:" << p_mix_box.box.center.x << "," << p_mix_box.box.center.y << "," << p_mix_box.box.center.z << endl;
				
			}
			
			associate = associate_detections_to_trackers(frame_dets, predictedBoxes);
			
			for (unsigned int i = 0; i <associate.matchedPairs.size(); i++)
			{
				//cout << "with detection:" << endl;
			
				trkIdx = associate.matchedPairs[i].x;
				detIdx = associate.matchedPairs[i].y;
				
				StateType_mix dets;
				dets.box = frame_dets[detIdx].box;
				dets.pro = frame_dets[detIdx].pro;
				dets.camera_id = frame_dets[detIdx].camera_id;
				dets.id = frame_dets[detIdx].id;
		
				
				trackers[trkIdx].Update(dets);
				
				
			}

			//Update Unassigned Tracks
			for (auto utk : associate.unmatchedTrajectories)
			{
			
				//if (trackers[utk].age > 3)
				if (trackers[utk].time_since_update < 3)
				{
					StateType_mix p_mix_box;
					p_mix_box = trackers[utk].Predict();
					//trackers[utk].Update(p_mix_box);

				}
			

			}


	


			// create and initialise new trackers for unmatched detections
			for (auto umd : associate.unmatchedDetections)
			{
				StateType_mix umd_dets;
				umd_dets.box = frame_dets[umd].box;
				umd_dets.pro = frame_dets[umd].pro;
				umd_dets.camera_id = frame_dets[umd].camera_id;
				
				KalmanBoxTracker tracker = KalmanBoxTracker(umd_dets);
			
				trackers.push_back(tracker);

			}
		
			
			if (trackers.size() != 0)
			{

				for (auto it = trackers.begin(); it != trackers.end();)
				{

					if (it != trackers.end() && (*it).time_since_update > 3)
					{
						
						pop_trks_hist.push_back((*it).history);
						it = trackers.erase(it);

					}
					else
						it++;

				}

			}

			vector<KalmanBoxTracker> trackers_new= trackers;
			//trackers_new = remove_closer_WorldPoint_trackers(trackers, 100);
			frameTrackingResult.clear();
			for (auto it = trackers_new.begin(); it != trackers_new.end();)
			{
				
				
				{
					TrackingBox res;
					res.box = (*it).Get_state();
				
				
					res.id = (*it).id;
					res.frame = frame_count;
					res.pro = (*it).trk_prob;
					res.camera_id = (*it).camera_id;
					
					frameTrackingResult.push_back(res);

					trks_hist.push_back((*it).history);
					//cout<<(*it).history;
				}

				// remove dead tracklet
				//if (it != trackers.end() && (*it).time_since_update > max_age)
				if (it != trackers_new.end() && (*it).time_since_update > 3)
				{
					
					pop_trks_hist.push_back((*it).history);
					it = trackers_new.erase(it);

				}
				else
					it++;
			}
			
			sortres.trks_hist = trks_hist;
			sortres.pop_trks_hist = pop_trks_hist;
			sortres.frameTrackingResult = frameTrackingResult;
			sortres.matchedPairs = associate.matchedPairs;
			sortres.unmatchedDetections = associate.unmatchedDetections;
			sortres.unmatchedTrajectories = associate.unmatchedTrajectories;
		
			return sortres;

		}


	}
	else
	{
		if (trackers.size() != 0)//no detection but have trackers,so need trackers.predict
		{
			for (auto it = trackers.begin(); it != trackers.end();)
			{
				if (((*it).time_since_update < max_age+1))
				{
					StateType_mix p_mix_box;
					p_mix_box = (*it).Predict();//kalman track,time_since_update++
				
					TrackingBox res;
					res.box = (*it).Get_state();
					(*it).time_since_update++;
					res.id = (*it).id;
					res.frame = frame_count;
					res.pro = (*it).trk_prob;
					res.camera_id = (*it).camera_id;

					frameTrackingResult.push_back(res);

					trks_hist.push_back((*it).history);
					it++;
				}
				else //(it != trackers.end() && (*it).time_since_update > max_age)
				{
					pop_trks_hist.push_back((*it).history);
					it=trackers.erase(it);
				}
				
			}
			sortres.trks_hist = trks_hist;
			sortres.pop_trks_hist = pop_trks_hist;
			sortres.frameTrackingResult = frameTrackingResult;
			sortres.matchedPairs = associate.matchedPairs;
			sortres.unmatchedDetections = associate.unmatchedDetections;
			sortres.unmatchedTrajectories = associate.unmatchedTrajectories;
			return sortres;
		}
		else //tracker.size()==0 ,no detction no trackers
		{
			return sortres;
		}

	}
	
	

}


