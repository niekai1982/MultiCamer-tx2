#pragma once
#include"kalmantracker.h"
#include<fstream>


struct trk_analysis_result
{
	bool cout_add;
	bool trk_direction;
};

trk_analysis_result trj_analysis_mix(vector<StateType_mix> pop_trk,  float prob_theta, float proportion, int age_thresh /*= 10*/, int dist_thresh /*= 60*/);