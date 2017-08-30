## Power lane finding

##### _note: this project is inspired by udacity nanoprogram and is not complete yet_

##### _Usage_
1. cd build
2. cmake ..
3. make
4. ./PowerLaneFinding

#### Motivation  
Lane finding is one of the most important topics in auto-driving. Traditionaly we use computer vision techniques to make use of camera images. However, camera images will be hugely affected when the weather is bad or the environment is noisy. For example, if the image is shadowed or overexposed, it will lead computer vision process to a bad result, or it may took too much effort to deal with it(ex: shadow removal, brightness and contrast adjustment).  
As a result, we choose **LiDAR** as our second sensor. LiDAR uses nearly-infrared laser to map the land, which is unaffected by visible light. Based on the characteristic that the reflective intensity of road lanes is higher than other regions, we can use the data of LiDAR as second standard. The final goal of this project is using the fusion of camera and LiDAR to 
achieve powerful lane finding.

#### Pipeline
0. LiDAR & Camera fusion (not introduced in this project)
1. Image preprocess (resize, undistortion...etc)
2. Perspective transform
3. Edge detection
4. Lane detection

![pipeline](https://github.com/u10000129/PowerLaneFinding-ubuntu-/blob/master/images/pipeline.jpg)

#### UML

#### Current Result
+ camera succeed, lidar succeed  
![775org](https://github.com/u10000129/PowerLaneFinding-ubuntu-/blob/master/images/lidar-succeed%2C%20camera-succeed/775RGB-org.bmp)  
_original image_  
![775camera](https://github.com/u10000129/PowerLaneFinding-ubuntu-/blob/master/images/lidar-succeed%2C%20camera-succeed/775RGB.bmp)  
_lane detection without lidar data_  
![775fusion](https://github.com/u10000129/PowerLaneFinding-ubuntu-/blob/master/images/lidar-succeed%2C%20camera-succeed/775RGB-fusion.bmp)  
_lane detection with lidar data_  
+ camera succeed, lidar fail
![897org](https://github.com/u10000129/PowerLaneFinding-ubuntu-/blob/master/images/lidar-fail%2C%20camera%20succeed/897RGB-org.bmp)
_original image_
![897camera](https://github.com/u10000129/PowerLaneFinding-ubuntu-/blob/master/images/lidar-fail%2C%20camera%20succeed/897RGB.bmp)
_lane detection without lidar data_
![897fusion](https://github.com/u10000129/PowerLaneFinding-ubuntu-/blob/master/images/lidar-fail%2C%20camera%20succeed/897-fusion.bmp)
_lane detection with lidar data_
+ camera fail, lidar succeed
![4853org](https://github.com/u10000129/PowerLaneFinding-ubuntu-/blob/master/images/lidar-succeed%2C%20camera%20fail/4853RGB-org.bmp)
_original image_
![4853camera](https://github.com/u10000129/PowerLaneFinding-ubuntu-/blob/master/images/lidar-succeed%2C%20camera%20fail/4853RGB.bmp)
_lane detection without lidar data_
![4853fusion](https://github.com/u10000129/PowerLaneFinding-ubuntu-/blob/master/images/lidar-succeed%2C%20camera%20fail/4853RGB-fusion.bmp)
_lane detection with lidar data_
+ camera fail, lidar fail
![2803org](https://github.com/u10000129/PowerLaneFinding-ubuntu-/blob/master/images/lidar-fail%2C%20camera-fail/2803RGB-org.bmp)
_original image_
![2803camera](https://github.com/u10000129/PowerLaneFinding-ubuntu-/blob/master/images/lidar-fail%2C%20camera-fail/2803RGB.bmp)
_lane detection without lidar data_
![2803fusion](https://github.com/u10000129/PowerLaneFinding-ubuntu-/blob/master/images/lidar-fail%2C%20camera-fail/2803RGB-fusion.bmp)
_lane detection with lidar data_

#### Future Work
1. Find an adaptive method for edge detection:  
   We have mutiple methods to deal with different scenes. For example, sobel filter is good at most most scenes but bad at shadowy image. R channel in BGR color space and S channel in HSV color space is good at shadowy image, but is afraid of bright spots.   
   Currently we change our edge detection method manually according to each frame. It's bad because in this way we can't apply the program on video, which means we can't use it on auto-driving car.  
   As a result, we have to find some way to differentiate scenes by scenes, and then we can apply edge detection adaptively. 
2. Find a reliable way to adopt camera or lidar data:  
   As you can see in the result above, camera is good sometimes but bad in another frame, so does lidar.   
　 As a result, we have to find some way to let program know when to adopt camera result and when to adopt lidar result.
3. ROI selection:  
   Currently we choose ROI manually. We have to find a reliable way to choose ROI automatically so we can apply program on auto-driving car.
4. Lane tracking:  
   Once we find the road lanes, we can just track the lane, which take much less effort and make the program more reliable.
