#include "../include/Pipeline.h"
#include <string>
#include <sstream>

Pipeline::Pipeline() {

}

/*
Input param:
char* path - path to the video

Content:
1. open the video according to path.
2. apply following steps to each frame:
   (1). resize
   (2). perspective transform
   (3). edge detection
   (4). lane detection
*/
void Pipeline::video(char* path) {
	VideoCapture cap(path);
	if (!cap.isOpened()) {
		cout << "!!! Failed to open file: " << endl;
		return;
	}
	Mat frame;
	int num = 0;
	for (;;) {
		if (!cap.read(frame))
			break;
		resize(frame, frame, Size(frame.cols / SCALE, frame.rows / SCALE));
		_wrp = _perspective.warp(frame);
		_tsh = _threshold.combine_thresh(_wrp);
		_dst = _laneDetection.finding_lane_line(_tsh);
		imshow("frame", frame);
		imshow("wrp", _wrp);
		imshow("tsh", _tsh);
		imshow("window", _dst);
		char key = waitKey(10);
		if (key == 27) // ESC
			break;
	}
}

/*
Input param:
char* path - path to the image

Content:
1. open the image according to path
(2. resize // put RESIZE into second param of read function.)
(3. undistort // put UNDISTORT into second param of read function.)
(Use '|' to combline RESIZE and UNDISTORT if you wish. Ex: _preprocess.read(path, RESIZE | UNDISTORT);)
4. perspective transform
5. lane detection
*/
void Pipeline::camera_only(char* path) {
	//preprocess.preprocess();
	_bgr = _preprocess.read(path, RESIZE);
	//_adj = preprocess.brightness_and_contrast_auto(_bgr);
	_wrp = _perspective.warp(_bgr);
	_tsh = _threshold.combine_thresh(_wrp);
	_dst = _laneDetection.finding_lane_line(_tsh);
}

/*
Input param:
char* dir_path - path to the directory of image
char* camera_path - file name of camera image
char* lidar_path - file name of lidar image
int di_num - the number of lidar image you want to stack

Content:
1. process image data
   (1). open the image according to path
   ((2). resize // put RESIZE into second param of read function.)
   ((3). undistort // put UNDISTORT into second param of read function.)
   (Use '|' to combline RESIZE and UNDISTORT if you wish. Ex: _preprocess.read(path, RESIZE | UNDISTORT);)
   (4). perspective transform
2. process lidar data
   (1). open few lidar images according to dir_path and di_num and stack them together
   (2). normalize stack image
   (3). perspective transform
   (4). calculate histogram of warp image to locate road lanes
3. lane detection
*/
void Pipeline::fusion(char* dir_path, char* camera_path, char* lidar_path, int di_num) {
	Mat tmp;
	Mat di_wrp, di_hist;
	string s_dir_path = string(dir_path);
	string s_camera_path = string(camera_path);
	string s_lidar_path = string(lidar_path);
	int lidar_num;

	// camera part
	//preprocess.preprocess();
	_bgr = _preprocess.read(s_dir_path + "/" + s_camera_path, RESIZE);
	//_adj = preprocess.brightness_and_contrast_auto(_bgr);
	_wrp = _perspective.warp(_bgr);
	_tsh = _threshold.combine_thresh(_wrp);
        
        // lidar part
	lidar_num = atoi(lidar_path);
	_di = Mat::zeros(_bgr.size(), _bgr.type());
	for (int i = 0; i < di_num; i++) {
                stringstream ss;
                ss << lidar_num-i;
                tmp = _preprocess.read(s_dir_path + "/" + ss.str() + "DI.bmp", RESIZE);
		addWeighted(tmp, 1, _di, 1, 0.0, _di);
	}
	_di = _preprocess.normalize_intensity(_di);
	di_wrp = _perspective.warp(_di);
	di_hist = draw_histogram(di_wrp, 1);

	// combine
	_dst = _laneDetection.finding_lane_line(_tsh, di_hist);
}

/*
Content:
show few images and write image.
*/
void Pipeline::show_img() {
	//imshow("bgr", _bgr);
	//imshow("di", _di);	
	//imshow("adj", _adj);
	//imshow("wrp", _wrp);
	//imshow("tsh", _tsh);
	imshow("dst", _dst);

	waitKey();

	//imwrite("bgr.jpg", _bgr);
	//imwrite("wrp.jpg", _wrp);
	//imwrite("tsh.jpg", _tsh);
	//imwrite("presentation-pics/lidar-fail, camera succeed/897-fusion.bmp", dst);
}

/*
Input param:
Mat src - lidar data
int channel - which channel we want to draw histogram

Return value:
   histogram of specified channel of source image

Content:
   draw histogram of specified channel of source image
*/
Mat Pipeline::draw_histogram(Mat src, int channel) {
	vector<Mat> channels;
	split(src, channels);

	Mat sum;
	reduce(channels[channel], sum, 0, CV_REDUCE_AVG, CV_32FC1);
	normalize(sum, sum, 255, 0, NORM_INF);
	Mat histogram(Mat::zeros(256, sum.cols, CV_8UC1));
	for (int i = 0; i < sum.cols; i++) {
		Point pt1(i, 255);
		Point pt2(i, 255 - sum.at<float>(0, i));
		line(histogram, pt1, pt2, Scalar(255));
	}

	return histogram;
}
