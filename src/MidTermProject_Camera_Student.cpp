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
int main(int argc, const char *argv[])
{

    // Task MP. 7, 8 and 9 variables defined here //

    int total_keypoints = 0;
    int total_preceding_vehicle_keypoints = 0;
    int total_matches = 0;

    double total_time;
    double detector_time;
    double total_detector_time;
    double total_descriptor_time;
    double descriptor_time;

    
    /* INIT VARIABLES AND DATA STRUCTURES */

    // data location
    string dataPath = "/home/gaurav/Desktop/c++/SFND_2D_Feature_Tracking-master/";

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
    bool bVis = false;            // visualize results

    //// STUDENT ASSIGNMENT
    //// TASK MP.2 -> add the following keypoint detectors in file matching2D.cpp and enable string-based selection based on detectorType
    //// -> HARRIS, FAST, BRISK, ORB, AKAZE, SIFT

    //string detectorType = "SHITOMASI";
    //string detectorType = "HARRIS";
    string detectorType = "FAST";
    //string detectorType = "BRISK";
    //string detectorType = "ORB";
    //string detectorType = "AKAZE";
    //string detectorType = "SIFT";

    //// TASK MP.4 -> add the following descriptors in file matching2D.cpp and enable string-based selection based on descriptorType
    
    string descriptorType = "BRISK"; // BRISK, BRIEF, ORB, FREAK, AKAZE, SIFT
	//string descriptorType = "BRIEF";
    //string descriptorType = "ORB";
    //string descriptorType = "FREAK";
    //string descriptorType = "AKAZE";
    //string descriptorType = "SIFT";


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
		dataBuffer.push_back(frame);
		
		// To load only two images at a time, we erase the first image of the vector when the vector size crosses 2... //
		if(dataBufferSize < dataBuffer.size())
			dataBuffer.erase(dataBuffer.begin());
		
        //// EOF STUDENT ASSIGNMENT TASK MP.1 //

        cout << "#1 : LOAD IMAGE INTO BUFFER done" << endl;

        /* DETECT IMAGE KEYPOINTS */

        // extract 2D keypoints from current image
        vector<cv::KeyPoint> keypoints; // create empty feature list for current image

        // MP.9 detector timer is included in every if loop statement so that the timing is accurate //

        detector_time = (double)cv::getTickCount();         // start of timer for detector //
        if (detectorType.compare("SHITOMASI") == 0)
        {
            detKeypointsShiTomasi(keypoints, imgGray, false);
        }
        
        // STUDENT ASSIGNMENT TASK MP.2 CONTINUED //
		// Gaurav Borgaonkar keypoint detectors included //

        else if (detectorType.compare("HARRIS") == 0)
        {
            detKeypointsHarris(keypoints, imgGray, false);
        }
        else        //Modern detectors FAST, BRISK, ORB, AKAZE, SIFT //
        {
            detKeypointsModern(keypoints, imgGray, detectorType, false);
        }
        detector_time = ((double)cv::getTickCount() - detector_time) / cv::getTickFrequency();  // end of detector timer //
        total_detector_time = total_detector_time + detector_time;

        cout << detectorType << " detector with n = " << keypoints.size() << " keypoints in " << 1000 * detector_time / 1.0 << " ms" << endl;

        total_keypoints = total_keypoints + keypoints.size();   // TASK MP. 7 - Counting total no. of keypoints on the preceding vehicle //

        // EOF STUDENT ASSIGNMENT TASK MP.2 //

        //// STUDENT ASSIGNMENT TASK MP.3 //

        //// TASK MP.3 -> only keep keypoints on the preceding vehicle
        
        // only keep keypoints on the preceding vehicle
        bool bFocusOnVehicle = false;
        cv::Rect vehicleRect(535, 180, 180, 150);   // extracting only keypoints from the preceding vehicle area //

        // Implementation 1 // ... something wrong here ...

        /*
        if (bFocusOnVehicle)
        {
            for (auto it = keypoints.begin(); it < keypoints.end(); it++)
            {
                if(!vehicleRect.contains(it->pt))
                    keypoints.erase(it);
            }
            cout<<"Number of preceding Vehicle keypoints are - "<<keypoints.size()<<endl;
        }
        */
        
        // Implementation 2 //

        
        if (bFocusOnVehicle)
        {
            
            vector<cv::KeyPoint> filteredKeypoints;
            for (auto kpts = keypoints.begin(); kpts < keypoints.end(); kpts++)
            {
                if (vehicleRect.contains(kpts->pt))
                    filteredKeypoints.push_back(*kpts);
            }
            keypoints.clear();
            keypoints = filteredKeypoints;  // add filtered keypoint to the keypoints vector //
            cout<<"Number of preceding vehicle keypoints are - "<<keypoints.size()<<endl;
            
        }   //end of if loop
        
        
        total_preceding_vehicle_keypoints =  total_preceding_vehicle_keypoints + keypoints.size();    // TASK MP. 7 Total number of keypoints on the preceding vehicle //

        //// EOF STUDENT ASSIGNMENT TASK MP.3

        // optional : limit number of keypoints (helpful for debugging and learning)
        bool bLimitKpts = false;
        if (bLimitKpts)
        {
            int maxKeypoints = 50;

            if (detectorType.compare("SHITOMASI") == 0)
            { // there is no response info, so keep the first 50 as they are sorted in descending quality order
                keypoints.erase(keypoints.begin() + maxKeypoints, keypoints.end());
            }
            cv::KeyPointsFilter::retainBest(keypoints, maxKeypoints);
            cout << " NOTE: Keypoints have been limited!" << endl;
        }

        // push keypoints and descriptor for current frame to end of data buffer
        (dataBuffer.end() - 1)->keypoints = keypoints;
        cout << "#2 : DETECT KEYPOINTS done" << endl;

        /* EXTRACT KEYPOINT DESCRIPTORS */

        //// STUDENT ASSIGNMENT
        //// TASK MP.4 -> add the following descriptors in file matching2D.cpp and enable string-based selection based on descriptorType
        //// -> BRIEF, ORB, FREAK, AKAZE, SIFT

        cv::Mat descriptors;
        descriptor_time = descKeypoints((dataBuffer.end() - 1)->keypoints, (dataBuffer.end() - 1)->cameraImg, descriptors, descriptorType); //changed function type to double //
        total_descriptor_time = total_descriptor_time + descriptor_time;

        //// EOF STUDENT ASSIGNMENT

        // push descriptors for current frame to end of data buffer
        (dataBuffer.end() - 1)->descriptors = descriptors;

        cout << "#3 : EXTRACT DESCRIPTORS done" << endl;

        if (dataBuffer.size() > 1) // wait until at least two images have been processed
        {

            /* MATCH KEYPOINT DESCRIPTORS */

            vector<cv::DMatch> matches;
            string matcherType = "MAT_FLANN";        // MAT_BF, MAT_FLANN
            string descriptorType = "DES_BINARY"; // DES_BINARY, DES_HOG
            string selectorType = "SEL_KNN";       // SEL_NN, SEL_KNN

            //// STUDENT ASSIGNMENT
            //// TASK MP.5 -> add FLANN matching in file matching2D.cpp - keypoints
            //// TASK MP.6 -> add KNN match selection and perform descriptor distance ratio filtering with t=0.8 in file matching2D.cpp

            matchDescriptors((dataBuffer.end() - 2)->keypoints, (dataBuffer.end() - 1)->keypoints,
                             (dataBuffer.end() - 2)->descriptors, (dataBuffer.end() - 1)->descriptors,
                             matches, descriptorType, matcherType, selectorType);

            //// EOF STUDENT ASSIGNMENT
            total_matches = total_matches + matches.size();     // TASK MP. 8 - Counting the total number of matches //

            // store matches in current data frame
            (dataBuffer.end() - 1)->kptMatches = matches;

            cout << "#4 : MATCH KEYPOINT DESCRIPTORS done" << endl;

            // visualize matches between current and previous image
            bVis = false;
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
                cout<<endl;

            }
            bVis = false;
        }

    } // eof loop over all images
    
    total_time = 1000 * (total_detector_time + total_descriptor_time);

    double ratio;
    ratio = total_matches / total_time;

    cout<<"Total number of keypoints for "<<detectorType<<" is "<<total_keypoints<<endl;
    cout<<"Total number of keypoints on the preceding vehicle for "<<detectorType<<" is "<<total_preceding_vehicle_keypoints<<endl;
    cout<<"Total number of matches for "<<detectorType<<" + "<<descriptorType<<" is "<<total_matches<<endl;
    cout<<"total time for detection = "<<total_detector_time * 1000<<" milliseconds..."<<endl;
    cout<<"total time for descriptor extraction = "<<total_descriptor_time * 1000<<" milliseconds..."<<endl;
    cout<<"Total time for "<<detectorType<<" + "<<descriptorType<<" combination is "<<total_time<<" ms"<<endl<<endl;
    cout<<"The ratio (efficiency) for "<<detectorType<<" + "<<descriptorType<<" combination is "<<ratio<<" matches/milliseconds"<<endl;

    
    return 0;
}
