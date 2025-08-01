#include <iostream>
#include <numeric>
#include <vector>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/features2d.hpp>


// Helper function for non-maximum suppression using keypoint overlap
bool isNonOverlapping(const cv::KeyPoint& candidate, std::vector<cv::KeyPoint>& keypoints, double maxOverlap)
{
    for (auto it = keypoints.begin(); it != keypoints.end(); ++it)
    {
        double overlap = cv::KeyPoint::overlap(candidate, *it);
        if (overlap > maxOverlap)
        {
            if (candidate.response > it->response)
            {
                *it = candidate;  // Replace the weaker keypoint
            }
            return false;  // In either case, don't add a duplicate
        }
    }
    return true;  // No conflict → can safely add
}

void cornernessHarris()
{
    // load image from file
    cv::Mat img;
    img = cv::imread("../images/img1.png");
    cv::cvtColor(img, img, cv::COLOR_BGR2GRAY); // convert to grayscale

    // Detector parameters
    int blockSize = 2;     // for every pixel, a blockSize × blockSize neighborhood is considered
    int apertureSize = 3;  // aperture parameter for Sobel operator (must be odd)
    float minResponse = 100.0f; // minimum value for a corner in the 8bit scaled response matrix
    double k = 0.04;       // Harris parameter (see equation for details)

    // Detect Harris corners and normalize output
    cv::Mat dst, dst_norm, dst_norm_scaled;
    dst = cv::Mat::zeros(img.size(), CV_32FC1);
    cv::cornerHarris(img, dst, blockSize, apertureSize, k, cv::BORDER_DEFAULT);
    cv::normalize(dst, dst_norm, 0, 255, cv::NORM_MINMAX, CV_32FC1, cv::Mat());
    cv::convertScaleAbs(dst_norm, dst_norm_scaled);

    // visualize results
    std::string windowName = "Harris Corner Detector Response Matrix";
    cv::namedWindow(windowName, 4);
    cv::imshow(windowName, dst_norm_scaled);
    cv::waitKey(0);

    std::vector<cv::KeyPoint> keypoints;
    double maxOverlap = 0.0; // max. permissible overlap between two features in %, used during non-maxima suppression
    for (size_t j = 0; j < dst_norm.rows; ++j)
    {
        for (size_t i = 0; i < dst_norm.cols; ++i)
        {
            auto response = dst_norm.at<float>(j, i);
            if (response > minResponse)
            { // only store points above a threshold

                cv::KeyPoint newKeyPoint(cv::Point2f(i, j), 2 * apertureSize, -1, response);

                // perform non-maximum suppression (NMS) in local neighbourhood around new key point
                if (isNonOverlapping(newKeyPoint, keypoints, maxOverlap))
                {
                    keypoints.push_back(newKeyPoint);
                }
            }
        } 
    }     

    // Visualization: keypoints
    windowName = "Harris Corner Detection Results";
    cv::namedWindow(windowName, 5);
    cv::Mat visImage = dst_norm_scaled.clone();
    cv::drawKeypoints(dst_norm_scaled, keypoints, visImage, cv::Scalar::all(-1), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
    cv::imshow(windowName, visImage);
    cv::waitKey(0);

}

int main()
{
    cornernessHarris();
}