#include "../include/Perspective.h"
#include "../include/Parameter.h"

/*
Content:
1. decide ROI manually
2. call getPerspectiveTransform() to get _M
*/
Perspective::Perspective() {
	// fusion2 897
	_src[0] = Point2f((330.) / SCALE, 400. / SCALE);
	_src[1] = Point2f((710.) / SCALE, 400. / SCALE);
	_src[2] = Point2f((1050.) / SCALE, 655. / SCALE);
	_src[3] = Point2f((18.) / SCALE, 655. / SCALE);
	float offset = 100.;
	_dst[0] = Point2f((0. + offset) / SCALE, 0. / SCALE);
	_dst[1] = Point2f((1280. - offset) / SCALE, 0. / SCALE);
	_dst[2] = Point2f((1280. - offset) / SCALE, 720. / SCALE);
	_dst[3] = Point2f((0. + offset) / SCALE, 720. / SCALE); 

	// fusion1 4853 
	/*_src[0] = Point2f((365.) / SCALE, 405. / SCALE);
	_src[1] = Point2f((675.) / SCALE, 405. / SCALE);
	_src[2] = Point2f((1195.) / SCALE, 720. / SCALE);
	_src[3] = Point2f((115.) / SCALE, 720. / SCALE);
	float offset = 100.;
	_dst[0] = Point2f((0. + offset) / SCALE, 0. / SCALE);
	_dst[1] = Point2f((1280. - offset) / SCALE, 0. / SCALE);
	_dst[2] = Point2f((1280. - offset) / SCALE, 720. / SCALE);
	_dst[3] = Point2f((0. + offset) / SCALE, 720. / SCALE); */

	// fusion2 2803 
	/*_src[0] = Point2f((385.) / SCALE, 375. / SCALE);
	_src[1] = Point2f((685.) / SCALE, 375. / SCALE);
	_src[2] = Point2f((1070.) / SCALE, 660. / SCALE);
	_src[3] = Point2f((55.) / SCALE, 660. / SCALE);
	float offset = 100.;
	_dst[0] = Point2f((0. + offset) / SCALE, 0. / SCALE);
	_dst[1] = Point2f((1280. - offset) / SCALE, 0. / SCALE);
	_dst[2] = Point2f((1280. - offset) / SCALE, 720. / SCALE);
	_dst[3] = Point2f((0. + offset) / SCALE, 720. / SCALE); */
	
	// fusion2 775
	/*_src[0] = Point2f((450.) / SCALE, 375. / SCALE);
	_src[1] = Point2f((690.) / SCALE, 375. / SCALE);
	_src[2] = Point2f((1240.) / SCALE, 720. / SCALE);
	_src[3] = Point2f((130.) / SCALE, 720. / SCALE);
	float offset = 100.;
	_dst[0] = Point2f((0. + offset) / SCALE, 0. / SCALE);
	_dst[1] = Point2f((1280. - offset) / SCALE, 0. / SCALE);
	_dst[2] = Point2f((1280. - offset) / SCALE, 720. / SCALE);
	_dst[3] = Point2f((0. + offset) / SCALE, 720. / SCALE);*/

	/*_src[0] = Point2f( (330.) /SCALE, 240. /SCALE );
	_src[1] = Point2f( (480.) /SCALE, 240. /SCALE );
	_src[2] = Point2f( (704.) /SCALE, 382. /SCALE );
	_src[3] = Point2f( (164.) /SCALE, 382. /SCALE );
	float offset = 100.;
	_dst[0] = Point2f( (0.  +offset) /SCALE, 0.   /SCALE );
	_dst[1] = Point2f( (854.-offset) /SCALE, 0.   /SCALE );
	_dst[2] = Point2f( (854.-offset) /SCALE, 480. /SCALE );
	_dst[3] = Point2f( (0.  +offset) /SCALE, 480. /SCALE );*/

	/*_src[0] = Point2f( (120. / 1280.*IMG_COL_SIZE)/SCALE, (720. / 720.*IMG_ROW_SIZE)/SCALE );
	_src[1] = Point2f( (586. / 1280.*IMG_COL_SIZE)/SCALE, (450. / 720.*IMG_ROW_SIZE)/SCALE );
	_src[2] = Point2f( (694. / 1280.*IMG_COL_SIZE)/SCALE, (450. / 720.*IMG_ROW_SIZE)/SCALE );
	_src[3] = Point2f( (1160. / 1280.*IMG_COL_SIZE)/SCALE, (720. / 720.*IMG_ROW_SIZE)/SCALE );

	_dst[0] = Point2f( (120. / 1280.*IMG_COL_SIZE)/SCALE, (1280. / 720.*IMG_ROW_SIZE)/SCALE );
	_dst[1] = Point2f( (120. / 1280.*IMG_COL_SIZE)/SCALE, (0. / 720.*IMG_ROW_SIZE)/SCALE );
	_dst[2] = Point2f( (600. / 1280.*IMG_COL_SIZE)/SCALE, (0. / 720.*IMG_ROW_SIZE)/SCALE );
	_dst[3] = Point2f( (600. / 1280.*IMG_COL_SIZE)/SCALE, (1280. / 720.*IMG_ROW_SIZE)/SCALE );*/
	_M = getPerspectiveTransform(_src, _dst);
}

/*
Input param:
Mat img - image we want to warp.

Return value:
Mat - warped image

Content:
1. warp image 
*/
Mat Perspective::warp(Mat img) {
	Mat dst;

	warpPerspective(img, dst, _M, img.size());

	/*line(img, Point2f(361. / SCALE, 203. / SCALE), Point2f(436. / SCALE, 203. / SCALE), Scalar(255), 2);
	line(img, Point2f(436. / SCALE, 203. / SCALE), Point2f(704. / SCALE, 382. / SCALE), Scalar(255), 2);
	line(img, Point2f(704. / SCALE, 382. / SCALE), Point2f(164. / SCALE, 382. / SCALE), Scalar(255), 2);
	line(img, Point2f(164. / SCALE, 382. / SCALE), Point2f(361. / SCALE, 203. / SCALE), Scalar(255), 2);*/

	return dst;
}
