#include "../include/Pipeline.h"
int main(void)
{
	Pipeline pipeline;
        char image_path[] = "../fusion2/897RGB.bmp";  
	//pipeline.camera_only(image_path);
        char dir_path[] = "../fusion2";
        char cam_path[] = "897RGB.bmp";
        char lid_path[] = "897DI.bmp";
	//int64 e1 = getTickCount();
        pipeline.fusion(dir_path, cam_path, lid_path, 7);
	//int64 e2 = getTickCount();
        //cout << (e2-e1)/getTickFrequency() << endl;
        pipeline.show_img();

	return 0;
}
