#include "../include/LaneDetection.h"
#include "../include/Parameter.h"
#include "time.h"
#include <fstream>
#include <numeric>

/*
Content:
Initialize variables of lane finding and curvature calculation.
*/
LaneDetection::LaneDetection() {
	// for lane finding
	_nwindows = 7;
	_margin = 100 / SCALE;
	_minpix = 50 / SCALE;
	_first_time = true;

	// for curvature calculation
	_degree = 2;
	_xm_per_pix = 3.7 / 700;
	_ym_per_pix = 30.0 / 720;
}

/*
Input param:
Mat lanes - source image after edge detection
Mat hist - histogram of lidar data

Return value:
Mat - source image with lane detection

Content:
1. find base point
2. use lidar data to locate road lanes
3. lane detection
4. lane curvature calculation
*/
Mat LaneDetection::finding_lane_line(Mat lanes, Mat hist) {
	_window_height = int(lanes.rows / _nwindows);
	hist_mask(lanes, hist);
	lanes.copyTo(_out_img);
	_left_lane_inds_x.clear();
	_left_lane_inds_y.clear();
	_right_lane_inds_x.clear();
	_right_lane_inds_y.clear();

	if(_first_time) decide_base_points();
	_first_time = false;
	find_line();
	get_lane_curvature();
	return _out_img;
}

/*
Input param:
Mat lanes - source image after edge detection

Return value:
Mat - source image with lane detection

Content:
1. find base point
2. lane detection
3. lane curvature calculation
*/
Mat LaneDetection::finding_lane_line(Mat lanes) {
	_window_height = int(lanes.rows / _nwindows);
	lanes.copyTo(_out_img);
	_left_lane_inds_x.clear();
	_left_lane_inds_y.clear();
	_right_lane_inds_x.clear();
	_right_lane_inds_y.clear();

	if (_first_time) decide_base_points();
	_first_time = false;
	find_line();
	get_lane_curvature();
	return _out_img;
}
 
/*
Input param:
Mat lanes - source image after edge detection
Mat hist - histogram of lidar data

Content:
1. find left and right peak of histogram
2. use peak found to locate lane and mask out other regions
*/
void LaneDetection::hist_mask(Mat lanes, Mat hist) {
	Mat hist_avg;
	int left_peak, right_peak;
	
	reduce(hist, hist_avg, 0, CV_REDUCE_AVG);

	int max = -1;

	for (int i = hist_avg.cols*0.1; i < hist_avg.cols*0.4; i++) {
		if (hist_avg.at<uchar>(0, i) > max) {
			max = hist_avg.at<uchar>(0, i);
			left_peak = i;
		}
	}
	
	max = -1;

	for (int i = hist_avg.cols * 0.6; i < hist_avg.cols * 0.9; i++) {
		if (hist_avg.at<uchar>(0, i) > max) {
			max = hist_avg.at<uchar>(0, i);
			right_peak = i;
		}
	}

	int offset = 120 / SCALE;
	Mat roi;
	
	cout << left_peak << endl;
	cout << right_peak << endl;

	int lpt = (left_peak - offset) > 0 ? left_peak - offset : 0;
	int rpt = (right_peak + offset) > 0 ? right_peak + offset : 0;

	roi = lanes(Rect(Point(0, 0), Point(lpt, lanes.rows-1)));
	roi.setTo(0);
	roi = lanes(Rect(Point(left_peak+offset, 0), Point(right_peak-offset, lanes.rows - 1)));
	roi.setTo(0);
	roi = lanes(Rect(Point(rpt, 0), Point(lanes.cols-1, lanes.rows - 1)));
	roi.setTo(0);
}

/*
Content:
1. find base points in bottom half image
2. if fail to find base points, find them in top half image
*/
void LaneDetection::decide_base_points() {
	vector<Point> bot, top;
	int near_parameter = 10;

	bot = find_base_points(0);
	if (bot[0].x == 0 || bot[1].x == 0) {
		top = find_base_points(1);
		if (bot[0].x != 0) {
			if (abs(bot[0].x - top[0].x) > near_parameter)
				bot[1].x = top[0].x;
			else if(abs(bot[0].x - top[1].x) > near_parameter)
				bot[1].x = top[1].x;
		}
		if (bot[1].x != 0) {
			if (abs(bot[1].x - top[0].x) > near_parameter)
				bot[0].x = top[0].x;
			else if(abs(bot[1].x - top[1].x) > near_parameter)
				bot[0].x = top[1].x;
		}
	}

	if (bot[0].x > bot[1].x)
		swap(bot[0], bot[1]);

	_idx = bot;
}

/*
Input param:
int find_loc - base points finding location. 0 for bottom half, 1 for top half

Return value:
vector<Point> - right and left base points.

Content:
1. find base points of right and left half
*/
vector<Point> LaneDetection::find_base_points(int find_loc) {
	Mat lanes_avg, finding_location;
	vector<Point> idx; idx.resize(2);
	if(find_loc == 0) // bottom half 
		finding_location = _out_img(Rect(Point(0, (IMG_ROW_SIZE - 1) / SCALE), Point(IMG_COL_SIZE / SCALE, (IMG_ROW_SIZE / 2) / SCALE)));
	else if(find_loc == 1) // top half
		finding_location = _out_img(Rect(Point(0, 0), Point(IMG_COL_SIZE / SCALE, (IMG_ROW_SIZE / 2) / SCALE)));

	reduce(finding_location, lanes_avg, 0, CV_REDUCE_AVG);
	int edges = lanes_avg.cols*0.1;
	for (int i=0, j=lanes_avg.cols*0.9; i < edges; i++, j++) {
		lanes_avg.at<uchar>(0, i) = 0;
		lanes_avg.at<uchar>(0, j) = 0;
	}
	
	minMaxLoc(lanes_avg, NULL, NULL, NULL, &idx[0]);

	if ((lanes_avg.cols - idx[0].x) < (idx[0].x - 0)) {
		for (int i = idx[0].x - 150/SCALE; i < lanes_avg.cols; i++)
			lanes_avg.at<uchar>(0, i) = 0;
	}
	else {
		for (int i = 0; i < idx[0].x + 150/SCALE; i++) 
			lanes_avg.at<uchar>(0, i) = 0;
	}
	
	minMaxLoc(lanes_avg, NULL, NULL, NULL, &idx[1]);
	
	return idx;
}

/*
Content:
find lane according to base points and sliding window
*/
void LaneDetection::find_line() {
	int cur_x_left = _idx[0].x;
	int cur_x_right = _idx[1].x;
	int win_y_low, win_y_high;
	int win_x_left_low, win_x_left_high;
	int win_x_right_low, win_x_right_high;
	_out_img.copyTo(_process_img);

	// store next idx
	int l_x = 0;
	int r_x = 0;
	for (int i = 0; i < _nwindows; i++) {
		// Identify window boundaries in x and y(and right and left)
		win_y_low = _out_img.rows - i*_window_height;
		win_y_high = _out_img.rows - (i + 1)*_window_height;
		win_x_left_low = (cur_x_left - _margin >= 0) ? cur_x_left - _margin : 0;
		win_x_left_high = (cur_x_left + _margin < _out_img.cols) ? cur_x_left + _margin : _out_img.cols - 1;
		win_x_right_low = (cur_x_right - _margin >= 0) ? cur_x_right - _margin : 0;
		win_x_right_high = (cur_x_right + _margin < _out_img.cols) ? cur_x_right + _margin : _out_img.cols - 1;
		// Identify the nonzero pixels in x and y within the window
		Mat non_zero_coordinates;
		int left_non_zero = 0;
		int right_non_zero = 0;
		vector<Point> left_inds, right_inds;
		// Find and collect non-zero points
		for (int k = win_y_high; k < win_y_low; k++) {
			for (int a = win_x_left_low, b = win_x_right_low; a < win_x_left_high, b < win_x_right_high; a++, b++) {
				if (_out_img.at<uchar>(Point(a, k)) != 0) {
					left_inds.push_back(Point(a, k));
					_left_lane_inds_x.push_back(a * _xm_per_pix);
					_left_lane_inds_y.push_back(k * _ym_per_pix);
					left_non_zero++;
				}
				if (_out_img.at<uchar>(Point(b, k)) != 0) {
					right_inds.push_back(Point(b, k));
					_right_lane_inds_x.push_back(b * _xm_per_pix);
					_right_lane_inds_y.push_back(k * _ym_per_pix);
					right_non_zero++;
				}
			}
		}
		Point zero(0, 0);
		Point sum;
		if (left_non_zero > _minpix) {
			sum = accumulate(left_inds.begin(), left_inds.end(), zero);
			cur_x_left = sum.x / left_inds.size();
			if (i == 0) l_x = cur_x_left;
		}
		if (right_non_zero > _minpix) {
			sum = accumulate(right_inds.begin(), right_inds.end(), zero);
			cur_x_right = sum.x / right_inds.size();
			if (i == 0) r_x = cur_x_right;
		}
		// Draw the windows on the visualization image
		// comment out to speed up
		cv::rectangle(_process_img, Point(win_x_left_low, win_y_low), Point(win_x_left_high, win_y_high), 255, 2);
		cv::rectangle(_process_img, Point(win_x_right_low, win_y_low), Point(win_x_right_high, win_y_high), 255, 2);
	}
	if(l_x != 0) _idx[0].x = l_x;
	if(r_x != 0) _idx[1].x = r_x;
	_process_img.copyTo(_out_img);
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
void LaneDetection::get_lane_curvature() {
	double *store;
	double right_curverad, left_curverad;
	int vec_num = _right_lane_inds_x.size();
	store = new double[_degree+1];
	polynomialfit(vec_num, _degree, _right_lane_inds_x, _right_lane_inds_y, store);
	right_curverad = (1 + pow(pow((2 * store[2] * _out_img.rows * _ym_per_pix + store[1]), 2), 1.5)) / abs(2 * store[2]);
	//cout << "right lane curvature: " << right_curverad << endl;
	
	vec_num = _left_lane_inds_x.size();
	polynomialfit(vec_num, _degree, _left_lane_inds_x, _left_lane_inds_y, store);
	left_curverad = (1 + pow(pow((2 * store[2] * _out_img.rows * _ym_per_pix + store[1]), 2), 1.5)) / abs(2 * store[2]);
	//cout << "left lane curvature: " << left_curverad << endl;
	delete[] store;
}

/*
check "http://www.bragitoff.com/2015/09/c-program-for-polynomial-fit-least-squares/"
*/
bool LaneDetection::polynomialfit(int obs, int degree, vector<double> dx, vector<double> dy, double *store) {
	int i, j, k;
	vector<double> X;                           //Array that will store the values of sigma(xi),sigma(xi^2),sigma(xi^3)....sigma(xi^2n)
	X.assign(2*degree+1, 0.);
	for (i = 0; i<2 * degree + 1; i++)
	{
		X[i] = 0;
		for (j = 0; j<obs; j++)
			X[i] = X[i] + pow(dx[j], i);        //consecutive positions of the array will store N,sigma(xi),sigma(xi^2),sigma(xi^3)....sigma(xi^2n)
	}

	vector<double> row;
	row.assign(degree + 2, 0.);
	vector< vector<double> > B;                   //B is the Normal matrix(augmented) that will store the equations
	B.assign(degree + 1, row);
	for (i = 0; i <= degree; i++)
		for (j = 0; j <= degree; j++)
			B[i][j] = X[i + j];                 //Build the Normal matrix by storing the corresponding coefficients at the right positions except the last column of the matrix
	vector<double> Y;                           //Array to store the values of sigma(yi),sigma(xi*yi),sigma(xi^2*yi)...sigma(xi^n*yi)
	Y.assign(degree + 1, 0.);
	for (i = 0; i<degree + 1; i++){
		Y[i] = 0;
		for (j = 0; j<obs; j++)
			Y[i] = Y[i] + pow(dx[j], i)*dy[j];        //consecutive positions will store sigma(yi),sigma(xi*yi),sigma(xi^2*yi)...sigma(xi^n*yi)
	}
	for (i = 0; i <= degree; i++)
		B[i][degree + 1] = Y[i];                //load the values of Y as the last column of B(Normal Matrix but augmented)
	degree = degree + 1;                        //n is made n+1 because the Gaussian Elimination part below was for n equations, but here n is the degree of polynomial and for n degree we get n+1 equations
	for (i = 0; i<degree; i++)                  //From now Gaussian Elimination starts(can be ignored) to solve the set of linear equations (Pivotisation)
		for (k = i + 1; k<degree; k++)
			if (B[i][i]<B[k][i])
				for (j = 0; j <= degree; j++)
				{
					double temp = B[i][j];
					B[i][j] = B[k][j];
					B[k][j] = temp;
				}

	for (i = 0; i<degree - 1; i++)              //loop to perform the gauss elimination
		for (k = i + 1; k<degree; k++)
		{
			double t = B[k][i] / B[i][i];
			for (j = 0; j <= degree; j++)
				B[k][j] = B[k][j] - t*B[i][j];   //make the elements below the pivot elements equal to zero or elimnate the variables
		}
	for (i = degree - 1; i >= 0; i--)            //back-substitution
	{                                            //x is an array whose values correspond to the values of x,y,z..
		store[i] = B[i][degree];                 //make the variable to be calculated equal to the rhs of the last equation
		for (j = 0; j<degree; j++)
			if (j != i)                          //then subtract all the lhs values except the coefficient of the variable whose value                                   is being calculated
				store[i] =store[i] - B[i][j] * store[j];
		store[i] = store[i] / B[i][i];           //now finally divide the rhs by the coefficient of the variable to be calculated
	}   

	return true; /* we do not "analyse" the result (cov matrix mainly)
				 to know if the fit is "good" */

}
