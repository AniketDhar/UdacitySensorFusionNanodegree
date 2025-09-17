#include <iostream>
#include <numeric>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include "structIO.hpp"
#include "dataStructures.h"

void loadCalibrationData(cv::Mat &P_rect_00, cv::Mat &R_rect_00, cv::Mat &RT)
{
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

}

void showLidarTopview(const std::vector<LidarPoint> &lidarPoints, const cv::Size worldSize, const cv::Size imageSize)
{
    // create topview image
    cv::Mat topviewImg(imageSize, CV_8UC3, cv::Scalar(0, 0, 0));

    // plot Lidar points into image
    for (const auto& lpt : lidarPoints)
    {
        float xw = lpt.x; // world position in m with x facing forward from sensor
        float yw = lpt.y; // world position in m with y facing left from sensor
        float zw = lpt.z; // world position in m with y facing left from sensor
       
        if(zw > -1.40f)
        { 
            // Convert from world coordinates to image coordinates
            int y = static_cast<int>((-xw * imageSize.height / worldSize.height) + imageSize.height);
            int x = static_cast<int>((-yw * imageSize.height / worldSize.height) + imageSize.width / 2);
      
            // Color based on distance (closer = red, farther = green)
            float maxVal = static_cast<float>(worldSize.height);
            int red = std::min(255, static_cast<int>(255 * std::abs((xw - maxVal) / maxVal)));
            int green = std::min(255, static_cast<int>(255 * (1 - std::abs((xw - maxVal) / maxVal))));

            cv::circle(topviewImg, cv::Point(x, y), 5, cv::Scalar(0, green, red), -1);
        }
    }

    // plot distance markers every 2 meters
    constexpr float lineSpacing = 2.0; // gap between distance markers
    int nMarkers = static_cast<int>(std::floor(worldSize.height / lineSpacing));

    for (size_t i = 0; i < nMarkers; ++i)
    {
        int y = static_cast<int>((-(i * lineSpacing) * imageSize.height / worldSize.height) + imageSize.height);
        cv::line(topviewImg, cv::Point(0, y), cv::Point(imageSize.width, y), cv::Scalar(255, 0, 0));
    }

    // display image
    const std::string windowName = "Top-View Perspective of LiDAR data";
    //cv::namedWindow(windowName, 2);
    cv::imshow(windowName, topviewImg);
    cv::waitKey(0); // wait for key to be pressed
}

void clusterLidarWithROI(std::vector<BoundingBox> &boundingBoxes, const std::vector<LidarPoint> &lidarPoints)
{
    // store calibration data in OpenCV matrices
    cv::Mat P_rect_xx(3,4,cv::DataType<double>::type); // 3x4 projection matrix after rectification
    cv::Mat R_rect_xx(4,4,cv::DataType<double>::type); // 3x3 rectifying rotation to make image planes co-planar
    cv::Mat RT(4,4,cv::DataType<double>::type); // rotation matrix and translation vector
    loadCalibrationData(P_rect_xx, R_rect_xx, RT);

    const double shrinkFactor = 0.10;

    // loop over all Lidar points and associate them to a 2D bounding box
    cv::Mat X(4, 1, cv::DataType<double>::type);
    cv::Mat Y(3, 1, cv::DataType<double>::type);

    for (const auto& lidarPoint : lidarPoints)
    {
        // assemble vector for matrix-vector-multiplication
        // homogeneous coordinates
        X.at<double>(0, 0) = lidarPoint.x;
        X.at<double>(1, 0) = lidarPoint.y;
        X.at<double>(2, 0) = lidarPoint.z;
        X.at<double>(3, 0) = 1.0;

        // project Lidar point into camera image plane
        Y = P_rect_xx * R_rect_xx * RT * X;
        double w = Y.at<double>(2, 0);
        cv::Point2f pt(Y.at<double>(0, 0) / w, Y.at<double>(1, 0) / w);

        std::vector<BoundingBox*> enclosingBoxes; // pointers to all bounding boxes which enclose the current Lidar point

        for(auto& box : boundingBoxes)
        {
            // shrink current bounding box slightly to avoid having too many outlier points around the edges
            cv::Rect smallerBox(
                box.roi.x + shrinkFactor * box.roi.width / 2.0,
                box.roi.y + shrinkFactor * box.roi.height / 2.0,
                box.roi.width * (1.0 - shrinkFactor),
                box.roi.height * (1.0 - shrinkFactor)
            );

            // check wether point is within current bounding box
            if (smallerBox.contains(pt))
            {
                enclosingBoxes.emplace_back(&box);
            }
        } // eof loop over all bounding boxes
        
      // Check wether point has been enclosed by one or by multiple boxes. 
      // Accordingly, add Lidar point to bounding box if it falls exactly in one ROI
      if(enclosingBoxes.size() == 1)
      {
        enclosingBoxes[0]->lidarPoints.emplace_back(lidarPoint);
      }

    } // eof loop over all Lidar points
}

int main()
{
    std::vector<LidarPoint> lidarPoints;
    readLidarPts("../dat/C53A3_currLidarPts.dat", lidarPoints);

    std::vector<BoundingBox> boundingBoxes;
    readBoundingBoxes("../dat/C53A3_currBoundingBoxes.dat", boundingBoxes);

    clusterLidarWithROI(boundingBoxes, lidarPoints);
    for (const auto &box : boundingBoxes)
    {
        if (!box.lidarPoints.empty())
        {
            showLidarTopview(box.lidarPoints, cv::Size(10.0, 25.0), cv::Size(1000, 2000));
        }
    }   

    return 0;
}