#include"probability.h"
#include "tx2_server.h"
#include <sys/time.h>

char* model ;
char* weight ;
char* INPUT_BLOB_NAME ;
char* OUTPUT_BLOB_NAME ;
uint32_t BATCH_SIZE;

std::vector<std::string> output_vector ;
TensorNet *tensorNet;

DimsCHW dimsData ;
DimsCHW dimsOut;
float* data ;
float* output ;
int height ;
int width ;

void* imgCPU;
void* imgCUDA;
Timer timer;
size_t size;

void* imgData;



////////////////////

int init_pro()
{
	
	model = "/home/nvidia/Desktop/MobileNet-SSD-TensorRT-master/model/61012_224_mb_iplugin.prototxt";
	weight = "/home/nvidia/Desktop/MobileNet-SSD-TensorRT-master/model/61012_224_mb.caffemodel";
	INPUT_BLOB_NAME = "data";
	OUTPUT_BLOB_NAME = "detection_out";
	BATCH_SIZE = 1;
	output_vector = { OUTPUT_BLOB_NAME };
	tensorNet = new TensorNet;
	tensorNet->LoadNetwork(model, weight, INPUT_BLOB_NAME, output_vector, BATCH_SIZE);

	dimsData = tensorNet->getTensorDims(INPUT_BLOB_NAME);
	dimsOut = tensorNet->getTensorDims(OUTPUT_BLOB_NAME);
	data = allocateMemory(dimsData, (char*)"input blob");
	output = allocateMemory(dimsOut, (char*)"output blob");
	height = 224;
	width = 224;
	size = width * height * sizeof(float3);
	imgData = malloc(size);
	if (CUDA_FAILED(cudaMalloc(&imgCUDA, size)))
	{
		cout << "Cuda Memory allocation error occured." << endl;
	}
	memset(imgData, 0, size);
	return 0;
}



float* allocateMemory(DimsCHW dims, char* info)
{
	float* ptr;
	size_t size;
	size = BATCH_SIZE * dims.c() * dims.h() * dims.w();
	assert(!cudaMallocManaged(&ptr, size * sizeof(float)));
	return ptr;
}

void loadImg(cv::Mat &input, int re_width, int re_height, float *data_unifrom, const float3 mean)
{
	int i;
	int j;
	int line_offset;
	int offset_g;
	int offset_b;
	cv::Mat dst;

	unsigned char *line = NULL;
	float *unifrom_data = data_unifrom;
	//cout << "imput image:" << input.size << endl;
	//cout << "re_width:" << re_width << endl;
	//cout << "re_height:" << re_height << endl;

	cv::resize(input, dst, cv::Size(re_width, re_height), (0.0), (0.0), cv::INTER_LINEAR);
	offset_g = re_width * re_height;
	offset_b = re_width * re_height * 2;
	for (i = 0; i < re_height; ++i)
	{
		line = dst.ptr< unsigned char >(i);
		line_offset = i * re_width;
		for (j = 0; j < re_width; ++j)
		{
			// r
			unifrom_data[line_offset + j] = ((float)(line[j * 3 + 2] - mean.x));
			// g
			unifrom_data[offset_g + line_offset + j] = ((float)(line[j * 3 + 1] - mean.y));
			// b
			unifrom_data[offset_b + line_offset + j] = ((float)(line[j * 3] - mean.z));
		}
	}
	cout << unifrom_data[0] << endl;
	cout << unifrom_data[1] << endl;
	cout << unifrom_data[2] << endl;
}


void probability_process(Mat frame_depth, Mat frame_bgr, vector<TrackingBox> &det_news, float threshold)
{
	
	vector<TrackingBox> rgb_pro;
	
	Mat srcImg = frame_bgr.clone();
	
	loadImg(srcImg, height, width, (float*)imgData, make_float3(123.0, 117.0, 104.0));
	
	cudaError_t cudaStatus; 
	cudaStatus=cudaMemcpyAsync(imgCUDA, imgData, size, cudaMemcpyHostToDevice);
	
	if (cudaStatus != cudaSuccess)
	{
		printf("cudaMemcpyAsync dev_a failed!\n");
	}
	void* buffers[] = { imgCUDA, output };
	
	tensorNet->imageInference(buffers, output_vector.size() + 1, BATCH_SIZE);
	
	for (int k = 0; k<100; k++)
	{
		
		if (output[7 * k + 1] == -1)
			break;
		float classIndex = output[7 * k + 1];
		float confidence = output[7 * k + 2];
		float xmin = output[7 * k + 3];
		float ymin = output[7 * k + 4];
		float xmax = output[7 * k + 5];
		float ymax = output[7 * k + 6];
		
			int x1 = static_cast<int>(xmin * srcImg.cols);
			int y1 = static_cast<int>(ymin * srcImg.rows);
			int x2 = static_cast<int>(xmax * srcImg.cols);
			int y2 = static_cast<int>(ymax * srcImg.rows);
			int width = x2 - x1;
			int hight = y2 - y1;
		
		if (confidence > 0.5)
		{

			int center_x = x1 + width/2;
			int center_y = y1 + hight/2;
			
			TrackingBox box;
			box.box.center.x = center_x;
			box.box.center.y = center_y;

			if( det_news.size() > 0 )
			{	
				Rect_<float> bb_test;
					
				box.camera_id = det_news[0].camera_id;	
				Depth2Word(box, frame_depth, 150, 150, 150);
				bb_test.x = box.box.center.x - box.box.size.x/2;
				bb_test.y = box.box.center.y - box.box.size.y/2;
				bb_test.width = box.box.size.x;
				bb_test.height = box.box.size.y;

				for (int i = 0; i < det_news.size(); i++)
				{
					Rect_<float> bb_gt;
					
					//depth 
					bb_gt.x = det_news[i].box.center.x - det_news[i].box.size.x / 2;
					bb_gt.y = det_news[i].box.center.y - det_news[i].box.size.y / 2;
					bb_gt.width = det_news[i].box.size.x;
					bb_gt.height = det_news[i].box.size.y;
			
					double IOU = GetIOU(bb_test, bb_gt);
				
					if (IOU > threshold)
					{
						det_news[i].pro = confidence;
					}
				}

			}
			
		}
	}
}
