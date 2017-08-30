/* Usage

Perspective transformation
*/

/* TO-DO
Automatic decide ROI (region of interest). The ROI here means the region we want to warp.
*/

#ifndef PERSPECTIVE_H
#define PERSPECTIVE_H

#include<opencv2/opencv.hpp>
using namespace cv;

class Perspective {
public:
	Perspective();
	Mat warp(Mat img);
private:
	Point2f _src[4];
	Point2f _dst[4];
	Mat _M;
};

#endif // PERSPECTIVE_H