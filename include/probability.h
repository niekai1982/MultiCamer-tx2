#pragma once
#include "common.h"
#include "cudaUtility.h"
#include "mathFunctions.h"
#include "pluginImplement.h"
#include "tensorNet.h"
#include "loadImage.h"
#include <chrono>
#include"coordinate_transform.h"
#include"sort.h"

extern char* model ;
extern char* weight ;

extern char* INPUT_BLOB_NAME ;
extern char* OUTPUT_BLOB_NAME;
extern uint32_t BATCH_SIZE ;
extern std::vector<std::string> output_vector ;
extern TensorNet *tensorNet;
//tensorNet.LoadNetwork(model, weight, INPUT_BLOB_NAME, output_vector, BATCH_SIZE);
extern DimsCHW dimsData ;
extern DimsCHW dimsOut ;
extern void* imgData;

int init_pro();
float* allocateMemory(DimsCHW dims, char* info);
void loadImg(cv::Mat &input, int re_width, int re_height, float *data_unifrom, const float3 mean);
void probability_process(Mat frame_depth, Mat frame_rgb, vector<TrackingBox> &det_news/*world*/, float threshold);//modify 20181126

//void probability_process(int devnum, Mat frame_depth, Mat frame_bgr, vector<TrackingBox> &det_news, float threshold);


