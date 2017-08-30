/* Usage

There are 3 pipeline functions.
Their difference are the type of input data :

1. input video stream
2. input camera data
3. input camera and LiDAR data

After declaring a variable of Pipeline class, choose the function you want to apply according to input data type. 

*/

#ifndef PIPELINE_H
#define PIPELINE_H
#include <opencv2/opencv.hpp>
#include "Preprocess.h"
#include "Threshold.h"
#include "Parameter.h"
#include "Perspective.h"
#include "LaneDetection.h"
#include "time.h"

class Pipeline 
{
public:
	Pipeline();
	void video(char* path);
	void camera_only(char* path);
	void fusion(char* dir_path, char* camera_path, char* lidar_path, int di_num);
	void show_img();
private:
	Mat draw_histogram(Mat src, int channel);

	Mat _bgr, _di, _wrp, _adj, _tsh, _dst;
	Preprocess _preprocess;
	Threshold _threshold;
	Perspective _perspective;
	LaneDetection _laneDetection;
};

#endif
