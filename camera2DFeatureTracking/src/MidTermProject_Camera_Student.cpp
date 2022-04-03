/* INCLUDES FOR THIS PROJECT */
#include <iostream>
#include <fstream>
#include <sstream>
#include <iomanip>
#include <vector>
#include <cmath>
#include <limits>
#include <opencv2/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/xfeatures2d.hpp>
#include <opencv2/xfeatures2d/nonfree.hpp>

#include "dataStructures.h"
#include "matching2D.hpp"

using namespace std;

/* MAIN PROGRAM */
void image_Process(string detector, string descriptor)
{
cout<<" process images use  descriptor : "<<descriptor <<" and detector : "<<detector<<endl;
    /* INIT VARIABLES AND DATA STRUCTURES */

    // data location
    string dataPath = "../";

    /* start of debugung section*/
    string detectorType = detector; //SHITOMASI, HARRIS, FAST, BRISK, ORB, AKAZE, SIFT
    string descriptorType = descriptor; // BRIEF, ORB, FREAK, AKAZE, SIFT
    string matcherType = "MAT_BF";        // MAT_BF, MAT_FLANN
    string descriptorType_m = "DES_BINARY"; // DES_BINARY, DES_HOG
    if(descriptorType =="SIFT")
    {
       descriptorType_m = "DES_HOG";
    }
    string selectorType = "SEL_NN";       // SEL_NN, SEL_KNN
    bool bFocusOnVehicle = true;
    bool bLimitKpts = false;
    bool bVis = false;
     /* end  of  debugung section*/
       // Timing measures to keep track of how long on average it took
      // for the detector and computing the descriptors
    double detector_time = 0.0;
    double descriptor_time = 0.0;
    int num_images = 0;
    int num_matched_keypoints =0;
    int num_keypoints =0 ;
    // camera
    string imgBasePath = dataPath + "images/";
    string imgPrefix = "KITTI/2011_09_26/image_00/data/000000"; // left camera, color
    string imgFileType = ".png";
    int imgStartIndex = 0; // first file index to load (assumes Lidar and camera names have identical naming convention)
    int imgEndIndex = 9;   // last file index to load
    int imgFillWidth = 4;  // no. of digits which make up the file index (e.g. img-0001.png)

    // misc
    int dataBufferSize = 2;       // no. of images which are held in memory (ring buffer) at the same time
    vector<DataFrame> dataBuffer; // list of data frames which are held in memory at the same time
    

    /* MAIN LOOP OVER ALL IMAGES */

    for (size_t imgIndex = 0; imgIndex <= imgEndIndex - imgStartIndex; imgIndex++)
    {
        /* LOAD IMAGE INTO BUFFER */

        // assemble filenames for current index
        ostringstream imgNumber;
        imgNumber << setfill('0') << setw(imgFillWidth) << imgStartIndex + imgIndex;
        string imgFullFilename = imgBasePath + imgPrefix + imgNumber.str() + imgFileType;

        // load image from file and convert to grayscale
        cv::Mat img, imgGray;
        img = cv::imread(imgFullFilename);
        cv::cvtColor(img, imgGray, cv::COLOR_BGR2GRAY);

        //// STUDENT ASSIGNMENT
        //// TASK MP.1 -> replace the following code with ring buffer of size dataBufferSize

        // push image into data frame buffer
        DataFrame frame;
        frame.cameraImg = imgGray;
      //  dataBuffer.push_back(frame);
     if(dataBuffer.size() == dataBufferSize)
     {
         dataBuffer.erase(dataBuffer.begin());
     }
       dataBuffer.push_back(frame);


        //// EOF STUDENT ASSIGNMENT
       // cout << "#1 : LOAD IMAGE INTO BUFFER done" << endl;

        /* DETECT IMAGE KEYPOINTS */

        // extract 2D keypoints from current image
        vector<cv::KeyPoint> keypoints; // create empty feature list for current image
        

        //// STUDENT ASSIGNMENT
        //// TASK MP.2 -> add the following keypoint detectors in file matching2D.cpp and enable string-based selection based on detectorType
        //// -> HARRIS, FAST, BRISK, ORB, AKAZE, SIFT

        if (detectorType.compare("SHITOMASI") == 0)
        {
           detector_time+= detKeypointsShiTomasi(keypoints, imgGray, false);
        }
        else if (detectorType.compare("HARRIS") == 0)
        {
            detector_time+= detKeypointsHarris(keypoints, imgGray, false);
        }
        else{

           detector_time+=  detKeypointsModern(keypoints, imgGray,detectorType, false);
        }
        //// EOF STUDENT ASSIGNMENT

        //// STUDENT ASSIGNMENT
        //// TASK MP.3 -> only keep keypoints on the preceding vehicle

        // only keep keypoints on the preceding vehicle
        
        cv::Rect vehicleRect(535, 180, 180, 150);
        if (bFocusOnVehicle)
        {
      keypoints.erase(remove_if(keypoints.begin(), keypoints.end(),
                                [&vehicleRect](const cv::KeyPoint& point) {
                                  return !vehicleRect.contains(point.pt);
                                }),
                      keypoints.end());
        num_keypoints +=keypoints.size();
   
        }

        //// EOF STUDENT ASSIGNMENT

        // optional : limit number of keypoints (helpful for debugging and learning)
        
        if (bLimitKpts)
        {
            int maxKeypoints = 50;

            if (detectorType.compare("SHITOMASI") == 0)
            { // there is no response info, so keep the first 50 as they are sorted in descending quality order
                keypoints.erase(keypoints.begin() + maxKeypoints, keypoints.end());
            }
            cv::KeyPointsFilter::retainBest(keypoints, maxKeypoints);
            //cout << " NOTE: Keypoints have been limited!"<<" "<<"new key points size =" <<keypoints.size()<< endl;
        }

        // push keypoints and descriptor for current frame to end of data buffer
        (dataBuffer.end() - 1)->keypoints = keypoints;
       // cout << "#2 : DETECT KEYPOINTS done" << endl;

        /* EXTRACT KEYPOINT DESCRIPTORS */

        //// STUDENT ASSIGNMENT
        //// TASK MP.4 -> add the following descriptors in file matching2D.cpp and enable string-based selection based on descriptorType
        //// -> BRIEF, ORB, FREAK, AKAZE, SIFT

        cv::Mat descriptors;
        descriptor_time +=descKeypoints((dataBuffer.end() - 1)->keypoints, (dataBuffer.end() - 1)->cameraImg, descriptors, descriptorType);
        //// EOF STUDENT ASSIGNMENT

        // push descriptors for current frame to end of data buffer
        (dataBuffer.end() - 1)->descriptors = descriptors;

       // cout << "#3 : EXTRACT DESCRIPTORS done" << endl;

        if (dataBuffer.size() > 1) // wait until at least two images have been processed
        {

            /* MATCH KEYPOINT DESCRIPTORS */

            vector<cv::DMatch> matches;


            //// STUDENT ASSIGNMENT
            //// TASK MP.5 -> add FLANN matching in file matching2D.cpp
            //// TASK MP.6 -> add KNN match selection and perform descriptor distance ratio filtering with t=0.8 in file matching2D.cpp

            matchDescriptors((dataBuffer.end() - 2)->keypoints, (dataBuffer.end() - 1)->keypoints,
                             (dataBuffer.end() - 2)->descriptors, (dataBuffer.end() - 1)->descriptors,
                             matches, descriptorType_m, matcherType, selectorType);

            //// EOF STUDENT ASSIGNMENT

            // store matches in current data frame
            (dataBuffer.end() - 1)->kptMatches = matches;

           // cout << "#4 : MATCH KEYPOINT DESCRIPTORS done"<< endl;
            // cout << "# of matched="<<matches.size()<< endl;
            num_matched_keypoints += matches.size();
            // visualize matches between current and previous image
            
            if (bVis)
            {
                cv::Mat matchImg = ((dataBuffer.end() - 1)->cameraImg).clone();
                cv::drawMatches((dataBuffer.end() - 2)->cameraImg, (dataBuffer.end() - 2)->keypoints,
                                (dataBuffer.end() - 1)->cameraImg, (dataBuffer.end() - 1)->keypoints,
                                matches, matchImg,
                                cv::Scalar::all(-1), cv::Scalar::all(-1),
                                vector<char>(), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);

                string windowName = "Matching keypoints between two camera images";
                cv::namedWindow(windowName, 7);
                cv::imshow(windowName, matchImg);
                cout << "Press key to continue to next image" << endl;
                cv::waitKey(0); // wait for key to be pressed
            }
            bVis = false;
        }
     num_images++;
    } // eof loop over all images
    detector_time /= num_images;
  descriptor_time /= num_images;

//***** task 7*****
  if (bFocusOnVehicle) {
    
    cout << "#number of keypoints =: " << num_keypoints << "\n";
  }
//***** task 8*****
 
  cout << "#matched keypoints =" << num_matched_keypoints << "\n";

   //***** task 9*****
  cout << "Detector Time: " << 1000 * detector_time << " ms";
  cout << "\nDescriptor Time: " << 1000 * descriptor_time << " ms\n";
    cout << "*****************************************************" << "\n";
   
}


int main(int argc, const char *argv[])
{
    

    image_Process("SHITOMASI", "BRIEF");
    image_Process("HARRIS", "BRIEF");
    image_Process("FAST", "BRIEF");
    image_Process("BRISK", "BRIEF");
    image_Process("ORB", "BRIEF");
    image_Process("AKAZE", "BRIEF");
    image_Process("SIFT", "BRIEF");

    image_Process("SHITOMASI", "ORB");
    image_Process("HARRIS", "ORB");
    image_Process("FAST", "ORB");
    image_Process("BRISK", "ORB");
    image_Process("ORB", "ORB");
    image_Process("AKAZE", "ORB");
    //image_Process("SIFT", "ORB");   //not applicable
 
    image_Process("SHITOMASI", "FREAK");
    image_Process("HARRIS", "FREAK");
    image_Process("FAST", "FREAK");
    image_Process("BRISK", "FREAK");
    image_Process("ORB", "FREAK");
    image_Process("AKAZE", "FREAK");
    image_Process("SIFT", "FREAK");

// //     image_Process("SHITOMASI", "AKAZE");  //not applicable
// //     image_Process("HARRIS", "AKAZE");  //not applicable
// //     image_Process("FAST", "AKAZE");  //not applicable
// //    image_Process("BRISK", "AKAZE");  //not applicable
// //     image_Process("ORB", "AKAZE");  //not applicable
    image_Process("AKAZE", "AKAZE");
// //    /// image_Process("SIFT", "AKAZE");  //not applicable

    image_Process("SHITOMASI", "SIFT");
    image_Process("HARRIS", "SIFT");
    image_Process("FAST", "SIFT");
   image_Process("BRISK", "SIFT");
    image_Process("ORB", "SIFT");
   image_Process("AKAZE", "SIFT");
   image_Process("SIFT", "SIFT");

    return 0;
}