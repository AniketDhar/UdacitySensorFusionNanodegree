// PCL lib Functions for processing point clouds 

#include "processPointClouds.h"
#include "ransacPlane.hpp"
#include "euclideanCluster.hpp"


//constructor:
template<typename PointT>
ProcessPointClouds<PointT>::ProcessPointClouds() {}


//de-constructor:
template<typename PointT>
ProcessPointClouds<PointT>::~ProcessPointClouds() {}


template<typename PointT>
void ProcessPointClouds<PointT>::numPoints(typename pcl::PointCloud<PointT>::Ptr cloud)
{
    std::cout << cloud->points.size() << std::endl;
}


template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::FilterCloud(typename pcl::PointCloud<PointT>::Ptr cloud, float filterRes, Eigen::Vector4f minPoint, Eigen::Vector4f maxPoint)
{

    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();

    // Function to do voxel grid point reduction and region based filtering
    // Voxel downsampling
    auto voxelizedCloud = std::make_shared<pcl::PointCloud<PointT>>();
    pcl::VoxelGrid<PointT> volxelGrid;
    volxelGrid.setInputCloud(cloud);
    volxelGrid.setLeafSize(filterRes, filterRes, filterRes);
    volxelGrid.filter(*voxelizedCloud);

    // ROI filtering
    auto roiCloud = std::make_shared<pcl::PointCloud<PointT>>();
    pcl::CropBox<PointT> cropBox(true);
    cropBox.setInputCloud(voxelizedCloud);
    cropBox.setMin(minPoint);
    cropBox.setMax(maxPoint);
    cropBox.filter(*roiCloud);

    // Remove egocar roof points
    // get rooftop point indices
    std::vector<int> indices;
    pcl::CropBox<PointT> roofFilter(true);
    roofFilter.setInputCloud(roiCloud);
    roofFilter.setMin(Eigen::Vector4f(-1.5, -1.8, -1, 1));
    roofFilter.setMax(Eigen::Vector4f(2.5, 1.8, -0.5, 1));
    roofFilter.filter(indices);

    auto inliers = std::make_shared<pcl::PointIndices>();
    inliers->indices.insert(inliers->indices.end(), indices.begin(), indices.end());

    // remove roof indices
    auto filteredCloud = std::make_shared<pcl::PointCloud<PointT>>();
    pcl::ExtractIndices<PointT> extractIdx;
    extractIdx.setInputCloud(roiCloud);
    extractIdx.setIndices(inliers);
    extractIdx.setNegative(true);
    extractIdx.filter(*filteredCloud);

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "filtering took " << elapsedTime.count() << " milliseconds" << std::endl;

    return filteredCloud;
}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SeparateClouds(pcl::PointIndices::Ptr inliers, typename pcl::PointCloud<PointT>::Ptr cloud) 
{
    // Create two new point clouds, one cloud with obstacles and other with segmented plane
    typename pcl::PointCloud<PointT>::Ptr obstCloud (new pcl::PointCloud<PointT>());
    typename pcl::PointCloud<PointT>::Ptr planeCloud (new pcl::PointCloud<PointT>());

    for( int index : inliers->indices)
    {
        planeCloud->points.push_back(cloud->points[index]);
    }

    // Create the filtering object
    pcl::ExtractIndices<PointT> extract;
    // extract points not in the plane
    extract.setInputCloud(cloud);
    extract.setIndices(inliers);
    extract.setNegative(true);
    extract.filter(*obstCloud);

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult(obstCloud, planeCloud);
    return segResult;
}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SegmentPlane(typename pcl::PointCloud<PointT>::Ptr cloud,
                                                                                                                             int maxIterations, 
                                                                                                                             float distanceThreshold)
{
    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();
	pcl::PointIndices::Ptr inliers{new pcl::PointIndices()};
    pcl::ModelCoefficients::Ptr coefficients{new pcl::ModelCoefficients()};
    pcl::SACSegmentation<PointT> seg;

    // find inliers for the cloud.
    // set segmentation optimization params
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setMaxIterations(maxIterations);
    seg.setDistanceThreshold(distanceThreshold);

    // apply segmentation to the cloud
    seg.setInputCloud(cloud);
    seg.segment(*inliers, *coefficients);
    if (inliers->indices.size() == 0)
    {
        std::cerr << "Could not estimate a planar model for the given dataset." << std::endl;
    }

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "plane segmentation took " << elapsedTime.count() << " milliseconds" << std::endl;

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult = SeparateClouds(inliers, cloud);
    return segResult;
}

// Segment using custom RANSAC implementation
template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SegmentPlaneCustom(typename pcl::PointCloud<PointT>::Ptr cloud, 
                                                                                                                                    int maxIterations, 
                                                                                                                                    float distanceThreshold)
{
    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();

    // Run RANSAC to get inlier indices
    std::unordered_set<int> inliers = RansacPlane<PointT>(cloud, maxIterations, distanceThreshold);

    typename pcl::PointCloud<PointT>::Ptr planeCloud(new pcl::PointCloud<PointT>());
    typename pcl::PointCloud<PointT>::Ptr obstacleCloud(new pcl::PointCloud<PointT>());

    for (int i = 0; i < cloud->points.size(); ++i)
    {
        if (inliers.count(i))
            planeCloud->points.push_back(cloud->points[i]);
        else
            obstacleCloud->points.push_back(cloud->points[i]);
    }

    planeCloud->width = planeCloud->points.size();
    planeCloud->height = 1;
    planeCloud->is_dense = true;

    obstacleCloud->width = obstacleCloud->points.size();
    obstacleCloud->height = 1;
    obstacleCloud->is_dense = true;

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "plane segmentation took " << elapsedTime.count() << " milliseconds" << std::endl;

    return std::make_pair(obstacleCloud, planeCloud);
}


template<typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::Clustering(typename pcl::PointCloud<PointT>::Ptr cloud, 
                                                                                    float clusterTolerance, 
                                                                                    int minSize, int maxSize)
{

    // Time clustering process
    auto startTime = std::chrono::steady_clock::now();

    std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;

    // perform euclidean clustering to group detected obstacles
    // Create a KDTree for clustering obstacles
    typename pcl::search::KdTree<PointT>::Ptr kdTree(new pcl::search::KdTree<PointT>);
    kdTree->setInputCloud(cloud);

    std::vector<pcl::PointIndices> clusterIndices;
    typename pcl::EuclideanClusterExtraction<PointT> euclidClus;
    euclidClus.setClusterTolerance(clusterTolerance);
    euclidClus.setMinClusterSize(minSize);
    euclidClus.setMaxClusterSize(maxSize);
    euclidClus.setSearchMethod(kdTree);
    euclidClus.setInputCloud(cloud);
    euclidClus.extract(clusterIndices);

    for (const auto& indices : clusterIndices)
    {
        typename pcl::PointCloud<PointT>::Ptr cluster(new pcl::PointCloud<PointT>);
        cluster->reserve(indices.indices.size());

        for(int idx : indices.indices)
        {
            cluster->push_back(cloud->points[idx]);
        }

        cluster->width = static_cast<uint32_t>(cluster->size());;
        cluster->height = 1;
        cluster->is_dense = true;

        clusters.push_back(cluster);
    }

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "clustering took " << elapsedTime.count() << " milliseconds and found " << clusters.size() << " clusters" << std::endl;

    return clusters;
}

template<typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::ClusteringCustom(typename pcl::PointCloud<PointT>::Ptr cloud, 
                                                                                            float clusterTolerance, 
                                                                                            int minSize, int maxSize)
{
    // Time clustering process
    auto startTime = std::chrono::steady_clock::now();

    // Build KdTree
    auto* tree = new KdTree;
    for (int i = 0; i < cloud->size(); ++i)
    {
        const auto& pt = cloud->points[i];
        tree->insert({pt.x, pt.y, pt.z}, i);
    }

    // Use Euclidean clustering
    std::vector<std::vector<int>> clusterIndices = euclideanCluster<PointT>(cloud, tree, clusterTolerance);
    std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;

    for (const auto& indices : clusterIndices)
    {
        if (indices.size() < minSize || indices.size() > maxSize)
            continue;

        typename pcl::PointCloud<PointT>::Ptr cluster(new pcl::PointCloud<PointT>);
        for (int idx : indices)
            cluster->points.emplace_back(cloud->points[idx]);

        cluster->width = cluster->size();
        cluster->height = 1;
        cluster->is_dense = true;
        clusters.emplace_back(cluster);
    }

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "clustering took " << elapsedTime.count() << " milliseconds and found " << clusters.size() << " clusters" << std::endl;

    return clusters;

}

template<typename PointT>
Box ProcessPointClouds<PointT>::BoundingBox(typename pcl::PointCloud<PointT>::Ptr cluster)
{

    // Find bounding box for one of the clusters
    PointT minPoint, maxPoint;
    pcl::getMinMax3D(*cluster, minPoint, maxPoint);

    Box box;
    box.x_min = minPoint.x;
    box.y_min = minPoint.y;
    box.z_min = minPoint.z;
    box.x_max = maxPoint.x;
    box.y_max = maxPoint.y;
    box.z_max = maxPoint.z;

    return box;
}


template<typename PointT>
void ProcessPointClouds<PointT>::savePcd(typename pcl::PointCloud<PointT>::Ptr cloud, std::string file)
{
    pcl::io::savePCDFileASCII (file, *cloud);
    std::cerr << "Saved " << cloud->points.size () << " data points to "+file << std::endl;
}


template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::loadPcd(std::string file)
{

    typename pcl::PointCloud<PointT>::Ptr cloud (new pcl::PointCloud<PointT>);

    if (pcl::io::loadPCDFile<PointT> (file, *cloud) == -1) //* load the file
    {
        PCL_ERROR ("Couldn't read file \n");
    }
    std::cerr << "Loaded " << cloud->points.size () << " data points from "+file << std::endl;

    return cloud;
}


template<typename PointT>
std::vector<std::filesystem::path> ProcessPointClouds<PointT>::streamPcd(std::string dataPath)
{

    std::vector<std::filesystem::path> paths(std::filesystem::directory_iterator{dataPath}, std::filesystem::directory_iterator{});

    // sort files in accending order so playback is chronological
    sort(paths.begin(), paths.end());

    return paths;

}