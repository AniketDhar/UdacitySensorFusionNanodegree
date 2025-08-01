#include <numeric>
#include <unordered_map>
#include <functional>
#include "matching2D.hpp"


void visualizeKeypoints(const cv::Mat &img, const std::vector<cv::KeyPoint> &keypoints, const std::string &windowName)
{
    cv::Mat visImage = img.clone();
    cv::drawKeypoints(img, keypoints, visImage, cv::Scalar::all(-1), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
    imshow(windowName, visImage);
    cv::waitKey(0);
}

// Find best matches for keypoints in two camera images based on several matching methods
void matchDescriptors(std::vector<cv::KeyPoint> &kPtsSource, std::vector<cv::KeyPoint> &kPtsRef, cv::Mat &descSource, cv::Mat &descRef,
                      std::vector<cv::DMatch> &matches, std::string descriptorCategory, std::string matcherType, std::string selectorType)
{
    // configure matcher
    bool crossCheck = false;
    cv::Ptr<cv::DescriptorMatcher> matcher;

    if (matcherType.compare("MAT_BF") == 0)
    {
        int normType = descriptorCategory.compare("DES_BINARY") == 0 ? cv::NORM_HAMMING : cv::NORM_L2;
        matcher = cv::BFMatcher::create(normType, crossCheck);
        std::cout << "BF matching (" << descriptorCategory << ") with cross-check=" << crossCheck << std::endl;
    }
    else if (matcherType.compare("MAT_FLANN") == 0)
    {
        if (descSource.type() != CV_32F)
        { // OpenCV bug workaround : convert binary descriptors to floating point due to a bug in current OpenCV implementation
            descSource.convertTo(descSource, CV_32F);
            descRef.convertTo(descRef, CV_32F);
        }

        matcher = cv::DescriptorMatcher::create(cv::DescriptorMatcher::FLANNBASED);
        std::cout << "FLANN matching";
    }

    // perform matching task
    if (selectorType.compare("SEL_NN") == 0)
    { // nearest neighbor (best match)

        matcher->match(descSource, descRef, matches); // Finds the best match for each descriptor in desc1
    }
    else if (selectorType.compare("SEL_KNN") == 0)
    { // k nearest neighbors (k=2)

        // implement k-nearest-neighbor matching
        std::vector<std::vector<cv::DMatch>> knnMatches;
        const int k = 2;
        double t = static_cast<double>(cv::getTickCount()); 
        matcher->knnMatch(descSource, descRef, knnMatches, k); // Finds the best match for each descriptor
        t = (static_cast<double>(cv::getTickCount()) - t) / cv::getTickFrequency();
        std::cout << " (KNN) with n=" << knnMatches.size() << " matches in " << 1000 * t / 1.0 << " ms" << std::endl;

        // filter matches using descriptor distance ratio test
        constexpr double ratioThreshold = 0.8f;
        size_t numFiltered = 0;
        for (const auto& matchPair : knnMatches)
        {
            if (matchPair.size() == 2 && matchPair[0].distance < ratioThreshold * matchPair[1].distance)
            {
                matches.push_back(matchPair[0]);
            }
            else
            {
                ++numFiltered;
            }
        }
        std::cout << numFiltered << " matches were filtered out"<< std::endl;
    }
}

// Use one of several types of state-of-art descriptors to uniquely identify keypoints
void descKeypoints(std::vector<cv::KeyPoint> &keypoints, cv::Mat &img, cv::Mat &descriptors, std::string descriptorType)
{
    using ExtractorFactory = std::function<cv::Ptr<cv::DescriptorExtractor>()>;

    // select appropriate descriptor
    static const std::unordered_map<std::string, ExtractorFactory> descriptorFactories = {
        {"BRISK", [] { return cv::BRISK::create(); }},
        {"BRIEF", [] { return cv::xfeatures2d::BriefDescriptorExtractor::create(); }},
        {"ORB",   [] { return cv::ORB::create(); }},
        {"FREAK", [] { return cv::xfeatures2d::FREAK::create(); }},
        {"AKAZE", [] { return cv::AKAZE::create(); }},
        {"SIFT",  [] { return cv::SIFT::create(); }}
    };

    const auto it = descriptorFactories.find(descriptorType);
    if (it == descriptorFactories.end())
    {
        std::cerr << "#3 : EXTRACT DESCRIPTORS failed. Invalid descriptorType '" << descriptorType
                  << "'. Valid options: BRISK, BRIEF, ORB, FREAK, AKAZE, SIFT" << std::endl;
        std::exit(EXIT_FAILURE);
    }

    auto extractor = it->second();

    // perform feature description
    double t = static_cast<double>(cv::getTickCount());
    extractor->compute(img, keypoints, descriptors);
    t = (static_cast<double>(cv::getTickCount()) - t) / cv::getTickFrequency();
    std::cout << descriptorType << " descriptor extraction in " << 1000 * t / 1.0 << " ms" << std::endl;
}

// Detect keypoints in image using the traditional Shi-Thomasi detector
void detKeypointsShiTomasi(std::vector<cv::KeyPoint> &keypoints, cv::Mat &img, bool bVis)
{
    // compute detector parameters based on image size
    const int blockSize = 4;       //  size of an average block for computing a derivative covariation matrix over each pixel neighborhood
    const double maxOverlap = 0.0; // max. permissible overlap between two features in %
    const double minDistance = (1.0 - maxOverlap) * blockSize;
    const int maxCorners = img.rows * img.cols / std::max(1.0, minDistance); // max. num. of keypoints

    const double qualityLevel = 0.01; // minimal accepted quality of image corners
    const double k = 0.04;

    // Apply corner detection
    double t = static_cast<double>(cv::getTickCount());
    std::vector<cv::Point2f> corners;
    cv::goodFeaturesToTrack(img, corners, maxCorners, qualityLevel, minDistance, cv::Mat(), blockSize, false, k);

    // add corners to result vector
    for (auto it = corners.begin(); it != corners.end(); ++it)
    {

        cv::KeyPoint newKeyPoint;
        newKeyPoint.pt = cv::Point2f((*it).x, (*it).y);
        newKeyPoint.size = blockSize;
        keypoints.push_back(newKeyPoint);
    }
    t = (static_cast<double>(cv::getTickCount()) - t) / cv::getTickFrequency();
    std::cout << "Shi-Tomasi detection with n=" << keypoints.size() << " keypoints in " << 1000 * t / 1.0 << " ms" << std::endl;

    // visualize results
    if (bVis)
    {
        visualizeKeypoints(img, keypoints, "Shi-Tomasi Corner Detector Results");
    }
}

void detKeypointsHarris(std::vector<cv::KeyPoint> &keypoints, cv::Mat &img, bool bVis)
{
    // Detector parameters
    const int blockSize = 2;
    const int apertureSize = 3;
    const double k = 0.04;
    const int thresh = 100;  // Minimum threshold for Harris response

    // Harris response matrix
    cv::Mat dst, dst_norm;
    dst = cv::Mat::zeros(img.size(), CV_32FC1);

    cv::cornerHarris(img, dst, blockSize, apertureSize, k);
    cv::normalize(dst, dst_norm, 0, 255, cv::NORM_MINMAX, CV_32FC1);

    // Non-maximum suppression and keypoint extraction
    for (int y = 0; y < dst_norm.rows; ++y) {
        for (int x = 0; x < dst_norm.cols; ++x) {
            float response = dst_norm.at<float>(y, x);
            if (response > thresh) 
            {
                keypoints.emplace_back(cv::Point2f(x, y), 2 * apertureSize, -1, response);
            }
        }
    }

    std::cout << "HARRIS detection with n=" << keypoints.size() << " keypoints" << std::endl;
    if (bVis) 
    {
        visualizeKeypoints(img, keypoints, "Harris Keypoints Detector Results");
    }
}

void detKeypointsModern(std::vector<cv::KeyPoint> &keypoints, cv::Mat &img, const std::string &detectorType, bool bVis)
{
    cv::Ptr<cv::FeatureDetector> detector;

    if (detectorType == "FAST")       detector = cv::FastFeatureDetector::create();
    else if (detectorType == "BRISK") detector = cv::BRISK::create();
    else if (detectorType == "ORB")   detector = cv::ORB::create();
    else if (detectorType == "AKAZE") detector = cv::AKAZE::create();
    else if (detectorType == "SIFT")  detector = cv::SIFT::create();
    else {
        std::cerr << "Unknown detector type: " << detectorType << '\n';
        return;
    }

    double t = static_cast<double>(cv::getTickCount());
    detector->detect(img, keypoints);
    t = (static_cast<double>(cv::getTickCount()) - t) / cv::getTickFrequency();
    std::cout << detectorType << " detection with n=" << keypoints.size()
              << " keypoints in " << 1000 * t << " ms" << std::endl;

    if (bVis) 
    {
        visualizeKeypoints(img, keypoints, detectorType + " Keypoints Detector Results");
    }
}

