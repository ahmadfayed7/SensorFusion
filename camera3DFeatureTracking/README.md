# SFND 3D Object Tracking

Welcome to the final project of the camera course. By completing all the lessons, you now have a solid understanding of keypoint detectors, descriptors, and methods to match them between successive images. Also, you know how to detect objects in an image using the YOLO deep-learning framework. And finally, you know how to associate regions in a camera image with Lidar points in 3D space. Let's take a look at our program schematic to see what we already have accomplished and what's still missing.

<img src="images/course_code_structure.png" width="779" height="414" />

In this final project, you will implement the missing parts in the schematic. To do this, you will complete four major tasks: 
1. First, you will develop a way to match 3D objects over time by using keypoint correspondences. 
2. Second, you will compute the TTC based on Lidar measurements. 
3. You will then proceed to do the same using the camera, which requires to first associate keypoint matches to regions of interest and then to compute the TTC based on those matches. 
4. And lastly, you will conduct various tests with the framework. Your goal is to identify the most suitable detector/descriptor combination for TTC estimation and also to search for problems that can lead to faulty measurements by the camera or Lidar sensor. In the last course of this Nanodegree, you will learn about the Kalman filter, which is a great way to combine the two independent TTC measurements into an improved version which is much more reliable than a single sensor alone can be. But before we think about such things, let us focus on your final project in the camera course. 

## Dependencies for Running Locally
* cmake >= 2.8
  * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1 (Linux, Mac), 3.81 (Windows)
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* Git LFS
  * Weight files are handled using [LFS](https://git-lfs.github.com/)
  * Install Git LFS before cloning this Repo.
* OpenCV >= 4.1
  * This must be compiled from source using the `-D OPENCV_ENABLE_NONFREE=ON` cmake flag for testing the SIFT and SURF detectors.
  * The OpenCV 4.1.0 source code can be found [here](https://github.com/opencv/opencv/tree/4.1.0)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools](https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory in the top level project directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./3D_object_tracking`.



##  Performance Evaluation 1 - LiDAR-based TTC

The table below shows the TTC estimate at a particular frame index. Recall that we are using LiDAR points that are bounded within object bounding boxes created by the YOLO Object Detection algorithm, so this is independent of any feature detectors and descriptors used. We will also show the minimum `x` coordinate in the point cloud at each frame to help with the analysis

|#Image 	     |TTC LiDAR (in seconds)	    |Min X (in m) |
|............ |...........................|.............|
|  1	         |   12.9722	                |   7.913     |
|  2	         |   12.264	                 |   7.849     |
|  3	         |   13.9161	                |   7.793     |
|  4	         |   7.11572	                |   7.685     |
|  5	         |   16.2511	                |   7.638     |
|  6	         |   12.4213	                |   7.577     |
|  7	         |   34.3404	                |   7.555     |
|  8	         |   9.34376	                |   7.475     |
|  9	         |   18.1318	                |   7.434     |
|  10	        |   18.0318	                |   7.393     |
|  11	        |   3.83244	                |   7.205     |
|  12	        |   10.8537	                |   7.272     |
|  13	        |   9.22307	                |   7.194     |
|  14	        |   10.9678	                |   7.129     |
|  15	        |   8.09422	                |   7.042     |
|  16	        |   3.17535	                |   6.827     |
|  17	        |   9.99424	                |   6.896     |
|  18	        |   8.30978	                |   6.814     |

The first three frames seem plausible but when we get to frames 4-6, it drops right down to 6.79 seconds. The reason why is because the minimum distance dropped from 7.79 m from frame 3 down to 7.68 m to frame 4\. For the first three frames, the distance to the ego vehicle seems plausible as it looks like we're gradually slowing down. However, at frame 4 there is a sudden jump in closeness meaning that the TTC thus decreases more quickly. The first three frames have a difference of 0.06 m between successive frames, but for frame 4, we have a 0.11 m difference. This is roughly double the distance resulting in the TTC decreasing by half in proportion.


We can see that shape of the point cloud of the back of the preceding vehicle is quite similar between the two frames. However, the cluster of points is shifted by the distances mentioned, thus reporting the incorrect TTC. It is not so much the point cloud but the constant velocity model breaks with sudden changes in distance. If we used a constant acceleration model this would report a more accurate TTC. Under the constant velocity model, this certainly goes to show that using LiDAR on its own is not a reliable source of information to calculate the TTC. Another instance is from frames 6 to 7 where the distances shorten from 7.58 m to 7.55 m, but the TTC increases to 34.34 seconds, but clearly we can visually see that the appearance of the preceding vehicle has not changed much between the frames so this is obviously a false reading. 

As we can clearly see, the point clouds are well formed but due to the constant velocity model, a short displacement between frames breaks down this assumption quickly.

##  Performance Evaluation 2

We will show graphs demonstrating the TTC using the camera-based approach for each possible combination of detector and descriptor. Each graph will show results for each possible detector where each trace on the graph will plot the TTC trend of each frame for a descriptor. As noted in the midterm report, it is not possible to use AKAZE descriptors with anything else other than AKAZE keypoints. Also, using the SIFT detector and the ORB descriptor causes out-of-memory issues, so we skip using that as well. To simulate the industry, standard, we use FLANN matching with kNN selection for efficiency.

[comaprison sheet Link](https://docs.google.com/spreadsheets/d/1Lkm2Tru3qH8b9d6L_UkeXKDMlcvaY_BHcA_qELgAex8/edit?usp=sharing)

With the  results in sheet :
1-  I recomend to use AZAKE detector or SIFT detector   (stable calcualtions)
2- totaly not recommend to use HARRIS detector          (not stable calcualtions)


