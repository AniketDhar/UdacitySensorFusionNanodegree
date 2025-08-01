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
#include "Logger.hpp"

//using namespace std;

/* MAIN PROGRAM */
int main(int argc, const char *argv[])
{
    // Prompt user to select detector and descriptor
    std::string detectorType, descriptorType;
    char loggingSelection;
    std::cout << "Enter detector type (SHITOMASI, HARRIS, FAST, BRISK, ORB, AKAZE, SIFT): ";
    std::cin >> detectorType;
    std::cout << "Enter descriptor type (BRISK, BRIEF, ORB, FREAK, AKAZE, SIFT): ";
    std::cin >> descriptorType;
    std::cout << "Do you want to log results? (Y/N): ";
    std::cin >> loggingSelection;

    bool enableLogging = (loggingSelection == 'Y' || loggingSelection == 'y');
    std::cout << "Logging set to : " << enableLogging << std::endl;

    // Validate combination
    if ((detectorType != "AKAZE" && descriptorType == "AKAZE") ||
        (detectorType == "SIFT" && descriptorType == "ORB"))
    {
        std::cerr << "Invalid detector/descriptor combination." << std::endl;
        return 1;
    }

    // Data location
    std::string dataPath = "../";
    std::string imgBasePath = dataPath + "images/";
    std::string imgPrefix = "KITTI/2011_09_26/image_00/data/000000";
    std::string imgFileType = ".png";
    int imgStartIndex = 0, imgEndIndex = 9, imgFillWidth = 4;
    int dataBufferSize = 2;
    // only keep keypoints on the preceding vehicle
    bool bFocusOnVehicle = true;
    cv::Rect vehicleRect(535, 180, 180, 150);
    
    std::vector<DataFrame> dataBuffer;
    int totalMatches = 0;
    int totalKeypoints = 0;
    int totalVehicleKeypoints = 0;
    double totalNeighborhoodSize = 0.0;
    double totalDetTime = 0.0, totalDescTime = 0.0;
    bool bVis = false;            // visualize results
    size_t imgIndex;

    /* MAIN LOOP OVER ALL IMAGES */

    for (imgIndex = 0; imgIndex <= imgEndIndex - imgStartIndex; ++imgIndex)
    {
        /* LOAD IMAGE INTO BUFFER */

        // assemble filenames for current index
        std::ostringstream imgNumber;
        imgNumber << std::setfill('0') << std::setw(imgFillWidth) << imgStartIndex + imgIndex;
        std::string imgFullFilename = imgBasePath + imgPrefix + imgNumber.str() + imgFileType;

        // load image from file and convert to grayscale
        cv::Mat img = cv::imread(imgFullFilename), imgGray;
        cv::cvtColor(img, imgGray, cv::COLOR_BGR2GRAY);

        // push image into data frame buffer
        DataFrame frame;
        frame.cameraImg = imgGray;
        if (dataBuffer.size() >= dataBufferSize)
            dataBuffer.erase(dataBuffer.begin());
        dataBuffer.push_back(frame);

        std::cout << "#1 : LOAD IMAGE INTO BUFFER done" << std::endl;
        
        /* DETECT IMAGE KEYPOINTS */

        // extract 2D keypoints from current image
        std::vector<cv::KeyPoint> keypoints;
        double t_det_start = cv::getTickCount();

        if (detectorType == "SHITOMASI")
            detKeypointsShiTomasi(keypoints, imgGray, false);
        else if (detectorType == "HARRIS")
            detKeypointsHarris(keypoints, imgGray, false);
        else
            detKeypointsModern(keypoints, imgGray, detectorType, false);

        double t_det_end = cv::getTickCount();
        double detTimeMs = 1000.0 * (t_det_end - t_det_start) / cv::getTickFrequency();

        totalKeypoints += keypoints.size();
        std::vector<cv::KeyPoint> vehicleKeypoints;
        double neighborhoodSum = 0.0;

        // Focus keypoints on vehicle
        if(bFocusOnVehicle)
        {           
            for (const auto &kp : keypoints)
            {
                if (vehicleRect.contains(kp.pt))
                {
                    vehicleKeypoints.push_back(kp);
                    neighborhoodSum += kp.size;
                }
            }
            
            totalVehicleKeypoints += vehicleKeypoints.size();
            totalNeighborhoodSize += neighborhoodSum;            
            keypoints = vehicleKeypoints;
        }

        // optional : limit number of keypoints (helpful for debugging and learning)
        bool bLimitKpts = true;
        if (bLimitKpts)
        {
            int maxKeypoints = 100;

            if (detectorType.compare("SHITOMASI") == 0)
            { // there is no response info, so keep the first 50 as they are sorted in descending quality order
                keypoints.erase(keypoints.begin() + maxKeypoints, keypoints.end());
            }
            cv::KeyPointsFilter::retainBest(keypoints, maxKeypoints);
            std::cout << " NOTE: Keypoints have been limited!" << std::endl;
        }

        // push keypoints and descriptor for current frame to end of data buffer
        dataBuffer.back().keypoints = keypoints;
        std::cout << "#2 : DETECT KEYPOINTS done" << std::endl;

        /* EXTRACT KEYPOINT DESCRIPTORS */

        // Descriptor extraction
        cv::Mat descriptors;
        double t_desc_start = cv::getTickCount();
        descKeypoints(keypoints, imgGray, descriptors, descriptorType);
        double t_desc_end = cv::getTickCount();
        double descTimeMs = 1000.0 * (t_desc_end - t_desc_start) / cv::getTickFrequency();

        totalDetTime += detTimeMs;
        totalDescTime += descTimeMs;

        dataBuffer.back().descriptors = descriptors;
        std::cout << "#3 : EXTRACT DESCRIPTORS done" << std::endl;

        int matchCount = 0;
        if (dataBuffer.size() > 1)
        {
            /* MATCH KEYPOINT DESCRIPTORS */
            std::vector<cv::DMatch> matches;
            std::string matcherType = "MAT_BF";        // MAT_BF, MAT_FLANN
            std::string descriptorCategory = (descriptorType== "SIFT") ? "DES_HOG" : "DES_BINARY"; // DES_BINARY, DES_HOG
            std::string selectorType = "SEL_KNN";       // SEL_NN, SEL_KNN

            matchDescriptors(dataBuffer.end()[-2].keypoints, dataBuffer.back().keypoints,
                             dataBuffer.end()[-2].descriptors, dataBuffer.back().descriptors,
                             matches, descriptorCategory, matcherType, selectorType);
            // store matches in current data frame
            dataBuffer.back().kptMatches = matches;
            matchCount = matches.size();
            totalMatches += matchCount;
            std::cout << "#4 : MATCH KEYPOINT DESCRIPTORS done" << std::endl;

            // visualize matches between current and previous image
            bVis = true;
            if (bVis)
            {
                cv::Mat matchImg = ((dataBuffer.end() - 1)->cameraImg).clone();
                cv::drawMatches((dataBuffer.end() - 2)->cameraImg, (dataBuffer.end() - 2)->keypoints,
                                (dataBuffer.end() - 1)->cameraImg, (dataBuffer.end() - 1)->keypoints,
                                matches, matchImg,
                                cv::Scalar::all(-1), cv::Scalar::all(-1),
                                std::vector<char>(), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);

                std::string windowName = "Matching keypoints between two camera images";
                cv::namedWindow(windowName, 7);
                cv::imshow(windowName, matchImg);
                std::cout << "Press key to continue to next image" << std::endl;
                cv::waitKey(0); // wait for key to be pressed
            }
            bVis = false;
        }
    }

    if(enableLogging)
    {
        // Initialize logger
        Logger logger("../logs/performance_summary.csv");

        // Log average values after all images
        auto numImages = static_cast<double>(imgIndex);
        double avgKeypoints = totalKeypoints / numImages;
        double avgVehicleKeypoints = totalVehicleKeypoints / numImages;
        double avgNeighborhoodSize = totalVehicleKeypoints > 0 ? totalNeighborhoodSize / totalVehicleKeypoints : 0.0;
        double avgDetTime = totalDetTime / numImages;
        double avgDescTime = totalDescTime / numImages;
        double avgMatches = totalMatches / numImages;

        logger.log(detectorType, descriptorType, avgKeypoints, avgVehicleKeypoints, avgNeighborhoodSize,
                avgMatches, avgDetTime, avgDescTime);
    }

    return 0;
}
