/* \author Aaron Brown */
// Create simple 3d highway enviroment using PCL
// for exploring self-driving car sensors

#include <memory>
#include "sensors/lidar.h"
#include "render/render.h"
#include "processPointClouds.h"
// using templates for processPointClouds so also include .cpp to help linker
#include "processPointClouds.cpp"
#include <filesystem>
#include <thread>
namespace fs = std::filesystem;

// #define PCL_METHOD
#define CUSTOM_METHOD


template <typename PointT>
void renderClusters(
    const std::vector<typename pcl::PointCloud<PointT>::Ptr>& clusters,
    std::shared_ptr<ProcessPointClouds<PointT>> processor,
    pcl::visualization::PCLVisualizer::Ptr& viewer)
{
    std::vector<Color> colors = {
        Color(1, 0.5, 0),  // orange
        Color(0, 1, 1),    // cyan
        Color(1, 0, 1),    // magenta
        Color(1, 1, 0),    // yellow
        Color(0.5, 0, 1)   // purple
    };

    int clusterId = 0;
    for (const auto& cluster : clusters)
    {
        // std::cout << "Cluster " << clusterId << " size: ";
        // processor->numPoints(cluster);

        Color color = colors[clusterId % colors.size()];
        std::string clusterName = "obstacleCloud" + std::to_string(clusterId);

        renderPointCloud(viewer, cluster, clusterName, color);

        Box box = processor->BoundingBox(cluster);
        renderBox(viewer, box, clusterId, color);

        ++clusterId;
    }
}

std::vector<Car> initHighway(bool renderScene, pcl::visualization::PCLVisualizer::Ptr& viewer)
{

    Car egoCar( Vect3(0,0,0), Vect3(4,2,2), Color(0,1,0), "egoCar");
    Car car1( Vect3(15,0,0), Vect3(4,2,2), Color(0,0,1), "car1");
    Car car2( Vect3(8,-4,0), Vect3(4,2,2), Color(0,0,1), "car2");	
    Car car3( Vect3(-12,4,0), Vect3(4,2,2), Color(0,0,1), "car3");
  
    std::vector<Car> cars;
    cars.push_back(egoCar);
    cars.push_back(car1);
    cars.push_back(car2);
    cars.push_back(car3);

    if(renderScene)
    {
        renderHighway(viewer);
        egoCar.render(viewer);
        car1.render(viewer);
        car2.render(viewer);
        car3.render(viewer);
    }

    return cars;
}


void simpleHighway(pcl::visualization::PCLVisualizer::Ptr& viewer)
{
    // ----------------------------------------------------
    // -----Open 3D viewer and display simple highway -----
    // ----------------------------------------------------
    
    // RENDER OPTIONS
    bool renderScene = false;
    std::vector<Car> cars = initHighway(renderScene, viewer);
    
    // Create lidar sensor 
    auto lidar = std::make_unique<Lidar>(cars, 0);
    pcl::PointCloud<pcl::PointXYZ>::Ptr inputCloud = lidar->scan();
    // renderRays(viewer, lidar->position, inputCloud);
    // renderPointCloud(viewer, inputCloud, "inputCloud");

    // Create point processor
    auto pointProcessor = std::make_shared<ProcessPointClouds<pcl::PointXYZ>>();
    // Segment the point cloud
    std::pair<pcl::PointCloud<pcl::PointXYZ>::Ptr, pcl::PointCloud<pcl::PointXYZ>::Ptr> segmentCloud = pointProcessor->SegmentPlane(inputCloud, 100, 0.2);
    renderPointCloud(viewer, segmentCloud.first, "obstacleCloud", Color(1,0,0));
    renderPointCloud(viewer, segmentCloud.second, "planeCloud", Color(0,1,0));

    // Cluster obstacles
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> clusters = pointProcessor->Clustering(segmentCloud.first, 1.0, 3, 30);
    // Visualize the clusters
    renderClusters<pcl::PointXYZ>(clusters, pointProcessor, viewer);
}

// For single pcd file
void cityBlock(pcl::visualization::PCLVisualizer::Ptr& viewer)
{
    // ----------------------------------------------------
    // -----Open 3D viewer and display City Block ---------
    // ----------------------------------------------------

    // Create point processor with intensity
    auto pointProcessorI = std::make_shared<ProcessPointClouds<pcl::PointXYZI>>();
    pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloud = pointProcessorI->loadPcd("../src/sensors/data/pcd/data_1/0000000000.pcd");
    // renderPointCloud(viewer, inputCloud, "inputCloud");

    // Filter point cloud
    float leafSize = 0.2;
    Eigen::Vector4f min(-15, -6, -2, 1);
    Eigen::Vector4f max(15, 6, 5, 1);

    pcl::PointCloud<pcl::PointXYZI>::Ptr filteredCloud = pointProcessorI->FilterCloud(inputCloud, leafSize, min, max);

    // Segment filtered point cloud
    std::pair<pcl::PointCloud<pcl::PointXYZI>::Ptr, pcl::PointCloud<pcl::PointXYZI>::Ptr> segmentedCloud = pointProcessorI->SegmentPlane(filteredCloud,
                                                                                                                                             100, 0.15);
    renderPointCloud(viewer, segmentedCloud.first, "obstacleCloud", Color(1,0,0));
    renderPointCloud(viewer, segmentedCloud.second, "planeCloud", Color(0,1,0));

    // Cluster the obstacles
    std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> clusters = pointProcessorI->Clustering(segmentedCloud.first, 0.45, 15, 500);
    
    // Visualize the clusters
    renderClusters<pcl::PointXYZI>(clusters, pointProcessorI, viewer);
}

// For streaming pcd files
void cityBlock(pcl::visualization::PCLVisualizer::Ptr& viewer, 
                std::shared_ptr<ProcessPointClouds<pcl::PointXYZI>> pointProcessorI,
                const pcl::PointCloud<pcl::PointXYZI>::Ptr& inputCloud) 
{
    // Filter point cloud
    float leafSize = 0.2;
    Eigen::Vector4f min(-15, -6, -2, 1);
    Eigen::Vector4f max(15, 6, 5, 1);

    pcl::PointCloud<pcl::PointXYZI>::Ptr filteredCloud = pointProcessorI->FilterCloud(inputCloud, leafSize, min, max);

    // Segment filtered point cloud
    float maxIterations = 100;
    float distanceThresh = 0.15;

    // std::pair<pcl::PointCloud<pcl::PointXYZI>::Ptr, pcl::PointCloud<pcl::PointXYZI>::Ptr> segmentedCloud;
    #if defined CUSTOM_METHOD
        auto segmentedCloud = pointProcessorI->SegmentPlaneCustom(filteredCloud, maxIterations, distanceThresh);
    #elif defined PCL_METHOD
        auto segmentedCloud = pointProcessorI->SegmentPlane(filteredCloud, maxIterations, distanceThresh);
    #endif

    renderPointCloud(viewer, segmentedCloud.first, "obstacleCloud", Color(1,0,0));
    renderPointCloud(viewer, segmentedCloud.second, "planeCloud", Color(0,1,0));

    // Cluster the obstacles
    float clusterTolerance = 0.5;
    int minPoints = 15;
    int maxPoints = 600;

    // std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> clusters;
    #if defined CUSTOM_METHOD
        auto clusters = pointProcessorI->ClusteringCustom(segmentedCloud.first, clusterTolerance, minPoints, maxPoints);
    #elif defined PCL_METHOD
        auto clusters = pointProcessorI->Clustering(segmentedCloud.first, clusterTolerance, minPoints, maxPoints);
    #endif

    // Visualize the clusters
    renderClusters<pcl::PointXYZI>(clusters, pointProcessorI, viewer);

}

//setAngle: SWITCH CAMERA ANGLE {XY, TopDown, Side, FPS}
void initCamera(CameraAngle setAngle, pcl::visualization::PCLVisualizer::Ptr& viewer)
{

    viewer->setBackgroundColor (0, 0, 0);
    
    // set camera position and angle
    viewer->initCameraParameters();
    // distance away in meters
    int distance = 16;
    
    switch(setAngle)
    {
        case XY : viewer->setCameraPosition(-distance, -distance, distance, 1, 1, 0); break;
        case TopDown : viewer->setCameraPosition(0, 0, distance, 1, 0, 1); break;
        case Side : viewer->setCameraPosition(0, -distance, 0, 0, 0, 1); break;
        case FPS : viewer->setCameraPosition(-10, 0, 0, 0, 0, 1);
    }

    if(setAngle!=FPS)
        viewer->addCoordinateSystem (1.0);
}


int main (int argc, char** argv)
{
    std::cout << "starting enviroment" << std::endl;
    vtkObject::GlobalWarningDisplayOff(); // Turn off VTK warning for deprecated vtkOpenGLPolyDataMapper::GetVertexShaderCode()

    pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    CameraAngle setAngle = FPS;
    initCamera(setAngle, viewer);
    // simpleHighway(viewer);
    // cityBlock(viewer);

    // Create point processor with intensity
    auto pointProcessorI = std::make_shared<ProcessPointClouds<pcl::PointXYZI>>();
    std::vector<fs::path> stream{pointProcessorI->streamPcd("../src/sensors/data/pcd/data_1")};
    auto streamIterator = stream.begin();

    pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloudI;

    while (!viewer->wasStopped())
    {
        viewer->removeAllPointClouds();
        viewer->removeAllShapes();

        if (streamIterator == stream.end())
            streamIterator = stream.begin();

        // Load current PCD file
        inputCloudI = pointProcessorI->loadPcd(streamIterator->string());

        // Run your processing pipeline
        cityBlock(viewer, pointProcessorI, inputCloudI);

        ++streamIterator;
        
        // Pause for visualization
        std::this_thread::sleep_for(std::chrono::milliseconds(300));
        viewer->spinOnce();
    }
}