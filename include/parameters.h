#pragma once
#include<string>
#include<vector>
#include<iostream>
#include<fstream>

#include"common.h"
using namespace std;

#define MAX_LINE 1024
extern ifstream infile;

struct parameters
{
	bool txt_key;
	bool save_key;

	int depth_RL;
	int depth_RH;
	int depth_value;
	//peak zone
	int min_distance;
	//sort
	int max_age;
	int min_hits;
	//probability
	float prob_theta;
	float proportion;
	int age_thresh;
	int dist_thresh;

};
void SplitString(const string & s, vector<string>& v, const string& c);
int ReadParameters( parameters &parameter_);