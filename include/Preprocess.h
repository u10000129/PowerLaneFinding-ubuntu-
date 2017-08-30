/* Usage

This class (Preprocess) comprise of few image pre-processing functions.
Including:

1. Read image
2. Resize image
3. Undistort image
4. Brightness and contrast adjustment
5. Intensity normalization (Only use for normalize intensity of LiDAR data)

Use read() function to input image. If you wish to resize or undistort image, enter the param accordingly.

If you want to adjust brightness, call brightness_and_contrast_auto().

If you want to normalize the intensity of LiDAR data, call normalize_intensity().

*/
#ifndef PREPROCESS_H
#define PREPROCESS_H

#include <opencv2/opencv.hpp>
#include <vector>
#include <string>
using namespace std;
using namespace cv;

class Preprocess
{
public:
	Preprocess();
	void preprocess();
	Mat read(string name, int param = 0);
	Mat brightness_and_contrast_auto(const Mat &src, float clipHistPercent = 0);
	Mat normalize_intensity(Mat img);
private:
	// for camera undistort
	int _nx, _ny;
	vector< vector<Point3f> > _object_points;
	vector< vector<Point2f> > _image_points;
	Mat _intrinsic;
	Mat _distCoeffs;
	Mat calibration(Mat img);
};

#endif // !PREPROCESS_H
