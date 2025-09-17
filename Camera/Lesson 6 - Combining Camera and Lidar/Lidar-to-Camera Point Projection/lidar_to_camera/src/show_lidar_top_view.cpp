#include <iostream>
#include <numeric>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include "structIO.hpp"

using namespace std;

void showLidarTopview()
{
    std::vector<LidarPoint> lidarPoints;
    readLidarPts("../dat/C51_LidarPts_0000.dat", lidarPoints);

    cv::Size worldSize(10.0, 20.0); // width and height of sensor field in m
    cv::Size imageSize(1000, 2000); // corresponding top view image in pixel

    // create topview image
    cv::Mat topviewImg(imageSize, CV_8UC3, cv::Scalar(0, 0, 0));

    // Plot Lidar points into image (Top-View)
    for (const auto &lidarPt : lidarPoints)
    {
        float xw = lidarPt.x; // world position in forward distance in meters
        float yw = lidarPt.y; // world position in left/right offset in meters
        float zw = lidarPt.z; // world position in height in meters

        // Filter out points on the road surface 
        constexpr float roadSurfaceLevel = -1.5f; 
        if (zw < roadSurfaceLevel) continue; // skip points below road surface

        // Map world coordinates to image coordinates 
        int y = static_cast<int>((-xw * imageSize.height / worldSize.height) + imageSize.height);
        int x = static_cast<int>((-yw * imageSize.width / worldSize.width) + imageSize.width / 2.0f);

        // Color mapping: X=0.0m -> red, X=20.0m -> green
        constexpr float maxVal = 20.0f;
        int red   = std::min(255, static_cast<int>(255 * (1.0f - xw / maxVal)));
        int green = std::min(255, static_cast<int>(255 * (xw / maxVal)));

        cv::circle(topviewImg, cv::Point(x, y), 5, cv::Scalar(0, green, red), -1);
    }

    // plot distance markers
    float lineSpacing = 2.0; // gap between distance markers
    int nMarkers = floor(worldSize.height / lineSpacing);
    for (size_t i = 0; i < nMarkers; ++i)
    {
        int y = (-(i * lineSpacing) * imageSize.height / worldSize.height) + imageSize.height;
        cv::line(topviewImg, cv::Point(0, y), cv::Point(imageSize.width, y), cv::Scalar(255, 0, 0));
    }

    // display image
    string windowName = "Top-View Perspective of LiDAR data";
    cv::namedWindow(windowName, 2);
    cv::imshow(windowName, topviewImg);
    cv::waitKey(0); // wait for key to be pressed
}

int main()
{
    showLidarTopview();
}