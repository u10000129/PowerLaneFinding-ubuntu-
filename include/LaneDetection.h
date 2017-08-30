/* Usage

Lane detection using histogram analysis and sliding window.

*/

#ifndef LANEDETECTION_H
#define LANEDETECTION_H

#include <opencv2/opencv.hpp>
#include <vector>
using namespace cv;
using namespace std;

class LaneDetection
{
public:
	// constructor, initialize some private parameters.
	LaneDetection();

    // initialize some private parameters and call 3 functions.
	// 1. find base points
	// 2. find line (according to base points)
	// 3. calculate lane curvature according to points found
	Mat finding_lane_line(Mat lanes, Mat hist);

	Mat finding_lane_line(Mat lanes);

private:
	int _degree;
	double _xm_per_pix;
	double _ym_per_pix;
	Mat _out_img; // output image with rectangle on lanes. use for debug
	Mat _process_img;
    vector<Point> _idx; // left and right base points
	bool _first_time;
	int _nwindows; // how many blocks we want to use for lane finding
	int _window_height; // the height of window
	int _margin; // the width of margin
	int _minpix; // a threshold to define whether the points in window is good
	vector<double> _left_lane_inds_x, _left_lane_inds_y; // collect all good points in right and left lanes
	vector<double> _right_lane_inds_x, _right_lane_inds_y;

	void hist_mask(Mat lanes, Mat hist);

	void decide_base_points();

	// find 2 base points according to the density of points in lanes
	vector<Point> find_base_points(int find_loc);

	// find 2 lines in picture according to base points
	void find_line();

	// calculate curvature of right and left lanes
	void get_lane_curvature();

	// polyfit using online resource
	bool polynomialfit(int obs, int degree, vector<double> dx, vector<double> dy, double *store);
};

#endif // !LANEDETECTION_H