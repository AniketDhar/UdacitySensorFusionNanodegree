
/* INCLUDES FOR THIS PROJECT */
#include <iostream>
#include <fstream>
#include <sstream>
#include <iomanip>
#include <vector>
#include <cmath>
#include <limits>
#include <filesystem>
#include <opencv2/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/xfeatures2d.hpp>
#include <opencv2/xfeatures2d/nonfree.hpp>

#include "dataStructures.h"
#include "matching2D.hpp"
#include "objectDetection2D.hpp"
#include "lidarData.hpp"
#include "camFusion.hpp"

/* MAIN PROGRAM */
int main(int argc, const char *argv[])
{
    /* INIT VARIABLES AND DATA STRUCTURES */

    // data location
    std::string dataPath = "../";

    // camera
    std::string imgBasePath = dataPath + "images/";
    std::string imgPrefix = "KITTI/2011_09_26/image_02/data/000000"; // left camera, color
    std::string imgFileType = ".png";
    int imgStartIndex = 0; // first file index to load (assumes Lidar and camera names have identical naming convention)
    int imgEndIndex = 18;   // last file index to load
    int imgStepWidth = 1; 
    int imgFillWidth = 4;  // no. of digits which make up the file index (e.g. img-0001.png)

    // object detection
    std::string yoloBasePath = dataPath + "dat/yolo/";
    std::string yoloClassesFile = yoloBasePath + "coco.names";
    std::string yoloModelConfiguration = yoloBasePath + "yolov3.cfg";
    std::string yoloModelWeights = yoloBasePath + "yolov3.weights";

    // Lidar
    std::string lidarPrefix = "KITTI/2011_09_26/velodyne_points/data/000000";
    std::string lidarFileType = ".bin";

    // calibration data for camera and lidar
    cv::Mat P_rect_00(3,4,cv::DataType<double>::type); // 3x4 projection matrix after rectification
    cv::Mat R_rect_00(4,4,cv::DataType<double>::type); // 3x3 rectifying rotation to make image planes co-planar
    cv::Mat RT(4,4,cv::DataType<double>::type); // rotation matrix and translation vector
    
    RT.at<double>(0,0) = 7.533745e-03; RT.at<double>(0,1) = -9.999714e-01; RT.at<double>(0,2) = -6.166020e-04; RT.at<double>(0,3) = -4.069766e-03;
    RT.at<double>(1,0) = 1.480249e-02; RT.at<double>(1,1) = 7.280733e-04; RT.at<double>(1,2) = -9.998902e-01; RT.at<double>(1,3) = -7.631618e-02;
    RT.at<double>(2,0) = 9.998621e-01; RT.at<double>(2,1) = 7.523790e-03; RT.at<double>(2,2) = 1.480755e-02; RT.at<double>(2,3) = -2.717806e-01;
    RT.at<double>(3,0) = 0.0; RT.at<double>(3,1) = 0.0; RT.at<double>(3,2) = 0.0; RT.at<double>(3,3) = 1.0;
    
    R_rect_00.at<double>(0,0) = 9.999239e-01; R_rect_00.at<double>(0,1) = 9.837760e-03; R_rect_00.at<double>(0,2) = -7.445048e-03; R_rect_00.at<double>(0,3) = 0.0;
    R_rect_00.at<double>(1,0) = -9.869795e-03; R_rect_00.at<double>(1,1) = 9.999421e-01; R_rect_00.at<double>(1,2) = -4.278459e-03; R_rect_00.at<double>(1,3) = 0.0;
    R_rect_00.at<double>(2,0) = 7.402527e-03; R_rect_00.at<double>(2,1) = 4.351614e-03; R_rect_00.at<double>(2,2) = 9.999631e-01; R_rect_00.at<double>(2,3) = 0.0;
    R_rect_00.at<double>(3,0) = 0; R_rect_00.at<double>(3,1) = 0; R_rect_00.at<double>(3,2) = 0; R_rect_00.at<double>(3,3) = 1;
    
    P_rect_00.at<double>(0,0) = 7.215377e+02; P_rect_00.at<double>(0,1) = 0.000000e+00; P_rect_00.at<double>(0,2) = 6.095593e+02; P_rect_00.at<double>(0,3) = 0.000000e+00;
    P_rect_00.at<double>(1,0) = 0.000000e+00; P_rect_00.at<double>(1,1) = 7.215377e+02; P_rect_00.at<double>(1,2) = 1.728540e+02; P_rect_00.at<double>(1,3) = 0.000000e+00;
    P_rect_00.at<double>(2,0) = 0.000000e+00; P_rect_00.at<double>(2,1) = 0.000000e+00; P_rect_00.at<double>(2,2) = 1.000000e+00; P_rect_00.at<double>(2,3) = 0.000000e+00;    

    // misc
    double sensorFrameRate = 10.0 / imgStepWidth; // frames per second for Lidar and camera
    int dataBufferSize = 2;       // no. of images which are held in memory (ring buffer) at the same time
    std::vector<DataFrame> dataBuffer; // list of data frames which are held in memory at the same time
    bool bVis = false;            // visualize results

    // keypoint/descriptor choice for this run
    std::string detectorType = "ORB";     // SHITOMASI, HARRIS, FAST, BRISK, ORB, AKAZE, SIFT
    std::string descriptorType = "ORB";       // BRISK, BRIEF, ORB, FREAK, AKAZE, SIFT

    // Validate combination
    if ((detectorType != "AKAZE" && descriptorType == "AKAZE") ||
        (detectorType == "SIFT" && descriptorType == "ORB"))
    {
        std::cerr << "Invalid detector/descriptor combination." << std::endl;
        return 1;
    }

    // Optional Logging of ttc values in a CSV
    const bool enableLogging = false;
    const std::string resultsDir   = "../results";

    if (!std::filesystem::exists(resultsDir)) 
        std::filesystem::create_directories(resultsDir);

    const int numImgs  = (imgEndIndex - imgStartIndex) / imgStepWidth + 1;
    const int numPairs = numImgs - 1;
    // Camera TTC logging for one det-desc combo run 
    std::vector<double> ttcCamRow(numPairs, std::numeric_limits<double>::quiet_NaN());
    // LiDAR TTC logging (one value per image pair)
    std::vector<double> ttcLidarRow(numPairs, std::numeric_limits<double>::quiet_NaN());
    const std::string comboKey = detectorType + "/" + descriptorType; // e.g. "ORB/SIFT"
    const std::string csvPathCamera  = resultsDir + "/ttc_camera_frame_wise.csv";
    const std::string csvPathLidar = resultsDir + "/ttc_lidar_frame_wise.csv";

    /* MAIN LOOP OVER ALL IMAGES */

    for (size_t imgIndex = 0; imgIndex <= imgEndIndex - imgStartIndex; imgIndex+=imgStepWidth)
    {
        /* LOAD IMAGE INTO BUFFER */

        // assemble filenames for current index
        std::ostringstream imgNumber;
        imgNumber << std::setfill('0') << std::setw(imgFillWidth) << imgStartIndex + imgIndex;
        std::string imgFullFilename = imgBasePath + imgPrefix + imgNumber.str() + imgFileType;

        // load image from file 
        cv::Mat img = cv::imread(imgFullFilename);

        // push image into data frame buffer
        DataFrame frame;
        frame.cameraImg = img;
        dataBuffer.emplace_back(frame);

        if (dataBuffer.size() > dataBufferSize)
            dataBuffer.erase(dataBuffer.begin()); // remove oldest frame

        std::cout << "#1 : LOAD IMAGE INTO BUFFER done" << std::endl;


        /* DETECT & CLASSIFY OBJECTS */

        float confThreshold = 0.2;
        float nmsThreshold = 0.4;        
        detectObjects(dataBuffer.back().cameraImg, dataBuffer.back().boundingBoxes, confThreshold, nmsThreshold,
                      yoloBasePath, yoloClassesFile, yoloModelConfiguration, yoloModelWeights, bVis);

        std::cout << "#2 : DETECT & CLASSIFY OBJECTS done" << std::endl;


        /* CROP LIDAR POINTS */

        // load 3D Lidar points from file
        std::string lidarFullFilename = imgBasePath + lidarPrefix + imgNumber.str() + lidarFileType;
        std::vector<LidarPoint> lidarPoints;
        loadLidarFromFile(lidarPoints, lidarFullFilename);

        // remove Lidar points based on distance properties
        float minZ = -1.5, maxZ = -0.9, minX = 2.0, maxX = 20.0, maxY = 2.0, minR = 0.1; // focus on ego lane
        cropLidarPoints(lidarPoints, minX, maxX, maxY, minZ, maxZ, minR);
    
        dataBuffer.back().lidarPoints = lidarPoints;

        std::cout << "#3 : CROP LIDAR POINTS done" << std::endl;


        /* CLUSTER LIDAR POINT CLOUD */

        // associate Lidar points with camera-based ROI
        float shrinkFactor = 0.10; // shrinks each bounding box by the given percentage to avoid 3D object merging at the edges of an ROI
        clusterLidarWithROI(dataBuffer.back().boundingBoxes, dataBuffer.back().lidarPoints, shrinkFactor, P_rect_00, R_rect_00, RT);

        // Visualize 3D objects
        // bVis = true;
        if(bVis)
        {
            show3DObjects(dataBuffer.back().boundingBoxes, cv::Size(4.0, 20.0), cv::Size(2000, 2000), true);
        }
        bVis = false;

        std::cout << "#4 : CLUSTER LIDAR POINT CLOUD done" << std::endl;
    
        /* DETECT IMAGE KEYPOINTS */

        // convert current image to grayscale
        cv::Mat imgGray;
        cv::cvtColor(dataBuffer.back().cameraImg, imgGray, cv::COLOR_BGR2GRAY);

        // extract 2D keypoints from current image
        std::vector<cv::KeyPoint> keypoints; // create empty feature list for current image

        if (detectorType == "SHITOMASI")
            detKeypointsShiTomasi(keypoints, imgGray, false);
        else if (detectorType == "HARRIS")
            detKeypointsHarris(keypoints, imgGray, false);
        else
            detKeypointsModern(keypoints, imgGray, detectorType, false);

        // Crop ROI 
        bool bFocusOnVehicle = true;
        cv::Rect vehicleRect(535, 180, 180, 150);
        std::vector<cv::KeyPoint> vehicleKeypoints;

        // Focus keypoints on vehicle
        if(bFocusOnVehicle)
        {           
            for (const auto &kp : keypoints)
            {
                if (vehicleRect.contains(kp.pt))
                {
                    vehicleKeypoints.push_back(kp);
                }
            }           
            keypoints = vehicleKeypoints;
        }

        // optional : limit number of keypoints (helpful for debugging and learning)
        bool bLimitKpts = false;
        if (bLimitKpts)
        {
            int maxKeypoints = 50;

            if (detectorType == "SHITOMASI")
            { // there is no response info, so keep the first 50 as they are sorted in descending quality order
                keypoints.erase(keypoints.begin() + maxKeypoints, keypoints.end());
            }
            cv::KeyPointsFilter::retainBest(keypoints, maxKeypoints);
            std::cout << " NOTE: Keypoints have been limited!" << std::endl;
        }

        // push keypoints and descriptor for current frame to end of data buffer
        dataBuffer.back().keypoints = keypoints;

        std::cout << "#5 : DETECT KEYPOINTS done" << std::endl;


        /* EXTRACT KEYPOINT DESCRIPTORS */

        cv::Mat descriptors;
        descKeypoints(keypoints, dataBuffer.back().cameraImg, descriptors, descriptorType);

        // push descriptors for current frame to end of data buffer
        dataBuffer.back().descriptors = descriptors;

        std::cout << "#6 : EXTRACT DESCRIPTORS done" << std::endl;


        if (dataBuffer.size() > 1) // wait until at least two images have been processed
        {

            /* MATCH KEYPOINT DESCRIPTORS */

            std::vector<cv::DMatch> matches;
            std::string matcherType = "MAT_BF";        // MAT_BF, MAT_FLANN
            std::string descriptorCategory = (descriptorType== "SIFT") ? "DES_HOG" : "DES_BINARY"; // DES_BINARY, DES_HOG
            std::string selectorType = "SEL_NN";       // SEL_NN, SEL_KNN

            matchDescriptors(dataBuffer.end()[-2].keypoints, dataBuffer.back().keypoints,
                             dataBuffer.end()[-2].descriptors, dataBuffer.back().descriptors,
                             matches, descriptorCategory, matcherType, selectorType);

            // store matches in current data frame
            dataBuffer.back().kptMatches = matches;

            std::cout << "#7 : MATCH KEYPOINT DESCRIPTORS done" << std::endl;

            
            /* TRACK 3D OBJECT BOUNDING BOXES */

            //// STUDENT ASSIGNMENT
            //// TASK FP.1 -> match list of 3D objects (vector<BoundingBox>) between current and previous frame (implement ->matchBoundingBoxes)
            std::map<int, int> bbBestMatches;
            matchBoundingBoxes(matches, bbBestMatches, dataBuffer[dataBuffer.size() - 2], dataBuffer.back()); // associate bounding boxes between current and previous frame using keypoint matches
            //// EOF STUDENT ASSIGNMENT

            // store matches in current data frame
            dataBuffer.back().bbMatches = bbBestMatches;

            std::cout << "#8 : TRACK 3D OBJECT BOUNDING BOXES done" << std::endl;


            /* COMPUTE TTC ON OBJECT IN FRONT */
            // pick one BB per pair for camera and Lidar TTC logging
            double bestTtcCamera = std::numeric_limits<double>::quiet_NaN();
            double bestTtcLidar  = std::numeric_limits<double>::quiet_NaN();
            size_t bestKptMatchCount = 0;

            // loop over all BB match pairs
            for (const auto& [prevId, currId] : dataBuffer.back().bbMatches)
            {
                // find bounding boxes associates with current match
                BoundingBox* prevBB = nullptr;
                BoundingBox* currBB = nullptr;

                auto& currBoxes = dataBuffer.back().boundingBoxes;
                auto& prevBoxes = dataBuffer[dataBuffer.size() - 2].boundingBoxes;

                for (auto& box : currBoxes)
                {
                    if (currId == box.boxID) // check whether current match partner corresponds to this BB
                    {
                        currBB = &box;
                        break; // no need to continue after finding the match
                    }
                }

                for (auto& box : prevBoxes)
                {
                    if (prevId == box.boxID) // check whether current match partner corresponds to this BB
                    {
                        prevBB = &box;
                        break;
                    }
                }


                // compute TTC for current match
                if( currBB->lidarPoints.size()>0 && prevBB->lidarPoints.size()>0 ) // only compute TTC if we have Lidar points
                {
                    //// STUDENT ASSIGNMENT
                    //// TASK FP.2 -> compute time-to-collision based on Lidar data (implement -> computeTTCLidar)
                    double ttcLidar; 
                    computeTTCLidar(prevBB->lidarPoints, currBB->lidarPoints, sensorFrameRate, ttcLidar);
                    //// EOF STUDENT ASSIGNMENT

                    //// STUDENT ASSIGNMENT
                    //// TASK FP.3 -> assign enclosed keypoint matches to bounding box (implement -> clusterKptMatchesWithROI)
                    //// TASK FP.4 -> compute time-to-collision based on camera (implement -> computeTTCCamera)
                    double ttcCamera;
                    clusterKptMatchesWithROI(*currBB, dataBuffer[dataBuffer.size() - 2].keypoints, dataBuffer.back().keypoints, dataBuffer.back().kptMatches);                    
                    computeTTCCamera(dataBuffer[dataBuffer.size() - 2].keypoints, dataBuffer.back().keypoints, currBB->kptMatches, sensorFrameRate, ttcCamera);
                    //// EOF STUDENT ASSIGNMENT

                    // choose representative TTC for this pair: BB with max keypoint matches
                    if (!std::isnan(ttcCamera))
                    {
                        const size_t kcnt = currBB->kptMatches.size();
                        if (kcnt > bestKptMatchCount)
                        {
                            bestKptMatchCount = kcnt;
                            bestTtcCamera = ttcCamera;
                            bestTtcLidar = ttcLidar;
                        }
                    }

                    bVis = true;
                    if (bVis)
                    {
                        cv::Mat visImg = dataBuffer.back().cameraImg.clone();
                        showLidarImgOverlay(visImg, currBB->lidarPoints, P_rect_00, R_rect_00, RT, &visImg);
                        cv::rectangle(visImg, cv::Point(currBB->roi.x, currBB->roi.y), cv::Point(currBB->roi.x + currBB->roi.width, currBB->roi.y + currBB->roi.height), cv::Scalar(0, 255, 0), 2);
                        
                        std::ostringstream oss;
                        oss << "TTC Lidar: " << ttcLidar << " s, TTC Camera: " << ttcCamera << " s";
                        putText(visImg, oss.str(), cv::Point2f(80, 50), cv::FONT_HERSHEY_PLAIN, 2, cv::Scalar(0,0,255));

                        // Save frame to results directory for GIF creation
                        static int frameIdx = 0; // persistent frame counter
                        const std::string frameDir = std::filesystem::path(resultsDir) / std::filesystem::path("results_" + detectorType + "_" + descriptorType);
                        std::filesystem::path framesPath = std::filesystem::path(resultsDir) / frameDir;
                        if (!std::filesystem::exists(framesPath)) 
                            std::filesystem::create_directories(framesPath);
                        std::ostringstream outName;
                        outName << framesPath.string() + "/frame_" << std::setw(4) << std::setfill('0') << frameIdx++ << + "_" + detectorType + "_" + descriptorType + ".png";
                        cv::imwrite(outName.str(), visImg);

                        std::string windowName = "Final Results : TTC";
                        cv::namedWindow(windowName, 4);
                        cv::imshow(windowName, visImg);
                        std::cout << "Press key to continue to next frame" << std::endl;
                        cv::waitKey(0);
                    }
                    bVis = false;

                } // eof TTC computation
            } // eof loop over all BB matches            

            // store per-pair camera TTC if logging is enabled
            if (enableLogging)
            {
                const int pairIdx = static_cast<int>((imgIndex - imgStartIndex) / imgStepWidth);
                if (pairIdx >= 0 && pairIdx < numPairs)
                    ttcCamRow[pairIdx] = bestTtcCamera;
                    ttcLidarRow[pairIdx] = bestTtcLidar;
            }
        }

    } // eof loop over all images

    /* === WRITE CSV (one row per run) === */
    if (enableLogging)
    {
        // write camera csv
        const bool exists = std::filesystem::exists(csvPathCamera);
        std::ofstream ofs(csvPathCamera, std::ios::app);
        if (!ofs)
        {
            std::cerr << "Cannot open " << csvPathCamera << " for writing\n";
        }
        else
        {
            if (!exists)
            {
                ofs << "Detector/Descriptor";
                for (int p = 0; p < numPairs; ++p)
                    ofs << ",Img-" << std::setw(2) << std::setfill('0') << (p+1);
                ofs << "\n";
            }

            ofs << comboKey;
            for (double v : ttcCamRow)
            {
                if (std::isnan(v) || std::isinf(v)) ofs << ",NaN";
                else ofs << "," << v;
            }
            ofs << "\n";

            std::cout << "Saved TTC camera row to " << csvPathCamera
                      << " for combo " << comboKey << "\n";
        }

        // write Lidar csv
        const bool existsL = std::filesystem::exists(csvPathLidar);
        std::ofstream ofsl(csvPathLidar, std::ios::app);
        if (!ofsl) {
            std::cerr << "Cannot open " << csvPathLidar << " for writing\n";
        } else {
            if (!existsL) {
                ofsl << "Detector/Descriptor";
                for (int p = 0; p < numPairs; ++p)
                    ofsl << ",Img-" << std::setw(2) << std::setfill('0') << (p+1);
                ofsl << "\n";
            }
            ofsl << comboKey;
            for (double v : ttcLidarRow) {
                if (std::isnan(v) || std::isinf(v)) ofsl << ",NaN";
                else ofsl << "," << v;
            }
            ofsl << "\n";
            std::cout << "Saved TTC LiDAR row to " << csvPathLidar
                    << " for combo " << comboKey << "\n";
        }
    }

    return 0;
}
