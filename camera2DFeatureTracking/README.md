# **SFND: 2D Feature Tracking**

<img src="images/keypoints.png" width="820" height="248" />

## Dependencies for Running Locally
* cmake >= 2.8
  * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1 (Linux, Mac), 3.81 (Windows)
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* OpenCV >= 4.1
  * This must be compiled from source using the `-D OPENCV_ENABLE_NONFREE=ON` cmake flag for testing the SIFT and SURF detectors.
  * The OpenCV 4.1.0 source code can be found [here](https://github.com/opencv/opencv/tree/4.1.0)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools](https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory in the top level directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./2D_feature_tracking`.

### **Rubric**

#### **MP1: Data Buffer Optimisation**
Implemented a ring buffer with ```dataBufferSize=2```.
<br>
Push the current image into the buffer as long as the buffer size is less than ```dataBufferSize```.
<br>
If the ring buffer size is greater than or equal to ```dataBufferSize``` then erase the oldest element.

#### **MP2: Keypoint Detection**
Implemented detectors [HARRIS, FAST, BRISK, ORB, AKAZE, SIFT] through three functions:

```detKeypointsShiTomasi```
<br>
```detKeypointsHarris```
<br>
```detKeypointsModern```

The number of keypoints detected as well as the detection time were stored for performance evaluation.

#### **MP3: Keypoint Removal**

Removed all keypoints outside of a pre-defined rectangle.
<br>
Used only the points within the ROI for firther processing.

#### **MP4: Keypoint Descriptors**

Implemented descriptors [BRIEF, ORB, FREAK, AKAZE, SIFT]
<br>
The descriptor extraction time was logged for performance evaluation.

#### **MP5: Descriptor Matching**

Implemented FLANN matching as well as K-Nearest Neighbour selection.
<br>
Matching methods [MAT_BF, FLANN] can be selected based on an input string.

#### **MP6: Descriptor Distance Ratio**

For KNN, filtered keypoints for matching based on a minimum distance ratio of 0.8.

#### **MP7: Number of Keypoints on Preceding Vehicle**

| Detector      | Number of Keypoints in ROI |
| ------------- | -------------------------- |
| **AKAZE**     | 1670                       |
| **BRISK**     | 2762                       |
| **FAST**      | 1491                       |
| **HARRIS**    | 196                        |
| **ORB**       | 1161                       |
| **SHITOMASI** | 1179                       |
| **SIFT**      | 1386                       |

#### **MP8: Number of Matched Keypoints**

| Detector\Descriptor | BRIEF | ORB   | FREAK | AKAZE  | SIFT |
| ------------------- | ----- | ----- | ----- | -----  | ---- |
| **SHITOMASI**       | 904   | 900   | 829   | N/A    | 915  |
| **HARRIS**          | 128   | 125   | 122   | N/A    | 133  |
| **FAST**            | 1015  | 1016  | 926   | N/A    | 1015 |
| **BRISK**           | 1550  | 1588  | 1495  | N/A    | 1536 |
| **ORB**             | 550   | 760   | 412   |  N/A   | 718  |
| **AKAZE**           | 1253  | 1206  | 1251  | 1250   | 1277 |
| **SIFT**            | 753   | N/A   | 683   |  N/A   | 832  |


#### **MP9: Average Processing Time  (detector time + descriptor time)**

| Detector\Descriptor | BRIEF   | ORB     | FREAK   | AKAZE  | SIFT   |
| ------------------- | -----   | -----   | -----   | -----  | ----   |
| **SHITOMASI**       | 30.06   | 20.01   | 108.57  | N/A    | 42.81  |
| **HARRIS**          | 1628.91 | 1580.92 | 1676.75 | N/A    | 1636.1 |
| **FAST**            | 2.96    | 3.6     | 92.92   | N/A    | 34.1   |
| **BRISK**           | 83.68   | 95.07   | 179.35  | N/A    | 133.628|
| **ORB**             | 16.57   | 24.88   | 104.01  |  N/A   | 68.1242|
| **AKAZE**           | 110.005 | 99.83   | 185.6   | 156.757| 112.08 |
| **SIFT**            | 181.28  | N/A     | 258.94  |  N/A   | 245.78 |


#### **Top Three Detector/Descriptor Combinations:**

Detector/Descriptor  | Number of Matched Keypoints | Processing Time |
-------------------- | --------------------------- | --------------- |
FAST+BRIEF           | 1015 keypoints              | 2.96 ms         |
FAST+ORB             | 1016 keypoints              | 3.6 ms          |
SHITOMASI+ORB        | 900 keypoints              | 20.01 ms         |
