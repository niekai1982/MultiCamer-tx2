#pragma once
#include"sort.h"

struct camera_parameters
{
	static float tcps_cuInv;
	static float tcps_cvInv;
	static float tcps_fxInv;
	static float tcps_fyInv;


};


void Algorithm_depthtoworld(int devnum, TrackingBox &det, int dx, int dy, unsigned short int dvalue, float *wx, float *wy, float *wz);
//void Algorithm_depthtoworld(TrackingBox &det, int dx, int dy, unsigned short int dvalue, float *wx, float *wy, float *wz);
void Algorithm_worldtodepth(TrackingBox &det, float wx, float wy, float wz, float *dx, float *dy, float *dz);
void world2depth(TrackingBox &box3d);
void world2rgb(TrackingBox &box3d);
void world2rgbs(vector<TrackingBox> &box3ds);

void unworld_union_coor(TrackingBox &det);
void world_union_coor(int devid, vector<TrackingBox> &dets);
//void unworld_union_coors(vector<TrackingBox> &dets);
int reigon_mean_depth(int y, int x, int reigon_size, Mat depth_input);
void Depth2Words(vector<TrackingBox> &dets, Mat depth_input, int width, int longsize, int hight);
void Depth2Word(TrackingBox &dets, Mat depth_input, int width, int longsize, int hight);
void world_union_coor(vector<TrackingBox> &dets);
//void Depth2Word(int devnum, TrackingBox &dets, Mat depth_input, int width, int longsize, int hight);

//void unworld_union2depth(TrackingBox &det);
vector<KalmanBoxTracker>  remove_closer_WorldPoint_trackers(vector<KalmanBoxTracker> &trackers, int dist_thrs);

vector<TrackingBox>  remove_closer_WorldPoint(vector<TrackingBox> dets, float dist_thrs);