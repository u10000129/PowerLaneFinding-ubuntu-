/* Usage

Edge detection according to specified filter and color space transformation.

*/

/*

Automatic choose edge detection method. 

Ex: daylight - sobel_x | bgr_r | hls_s
    shadow   - sobel_x | sobel_y | bgr_r

The difficulty is that how to find the standard to know this image is daylight or shadow...etc.

Automatic choose edge detection threshold value.

*/

#ifndef THRESHOLD_H
#define THRESHOLD_H

#define d_sobel_kernel_size   3
#define MIN_SOBEL_X_THRESH    30  // 15
#define MAX_SOBEL_X_THRESH    60  // 40
#define MIN_SOBEL_Y_THRESH    20  // 15
#define MAX_SOBEL_Y_THRESH    255  // 30
#define MIN_SOBEL_MAG_THRESH  30  // 18
#define MAX_SOBEL_MAG_THRESH  255  // 60
#define MIN_SOBEL_DIR_THRESH  0 // 0.99
#define MAX_SOBEL_DIR_THRESH  0.4  // 1.0
#define MIN_BGR_R_THRESH      190 // 60
#define MAX_BGR_R_THRESH      255 // 70
#define MIN_HLS_S_THRESH      80 // 20
#define MAX_HLS_S_THRESH      255 // 60
#define MIN_YUV_U_THRESH      125 // 60
#define MAX_YUV_U_THRESH      130 // 125
#define MIN_LAP_THRESH        30  // 30
#define MAX_LAP_THRESH        200 // 200
#define MIN_CAN_THRESH        50 // 50
#define MAX_CAN_THRESH        150 // 150
#define ENTROPY_THRESHOLD     3.6

#include <opencv2/opencv.hpp>
using namespace cv;

class Threshold {
public:
	// constructor
	Threshold();
	Threshold(int sobel_kernel_size);

	// several threshold methods
	// Including 
	// 1. sobel x y
	// 2. sobel magnitude
	// 3. sobel direction
	// 4. rgb r channel 
	// 5. hls s channel 
	// 6. yuv u channel 
	// 7. laplace edge detection
	// 8. canny edge detection 
	Mat abs_sobel_thresh(char orient = 'x', int thresh_min = -1, int thresh_max = 255);
	Mat mag_thresh(int thresh_min = -1, int thresh_max = 255);
	Mat dir_thresh(double thresh_min = -1, double thresh_max = CV_PI / 2);
	Mat rgb_thresh(int thresh_min = -1, int thresh_max = 255);
	Mat hls_thresh(int thresh_min = -1, int thresh_max = 255);
	Mat yuv_thresh(int thresh_min = -1, int thresh_max = 255);
	Mat lap_thresh(int thresh_min = -1, int thresh_max = 255);
	Mat can_thresh(int thresh_min = -1, int thresh_max = 255);

	// combine multiple threshold methods
	Mat combine_thresh(Mat src);

private:
	int _sobel_kernel_size;
	Mat _sobel_x, _sobel_y, _s_channel, _r_channel, _u_channel, _clahe_gray, _gray;
	int _type;
	Ptr<CLAHE> _clahe;

	// convert source image(rgb) to gray, hls and yuv.
	void source_image_process(Mat src);

	// process edge-detection to source image
	Mat threshold_process(Mat src, double thresh_min, double thresh_max, bool scale = true);

	// calculate entrophy of source picture and choose edge detection method
	void entropy_cal();
};


#endif // !THRESHOLD_H