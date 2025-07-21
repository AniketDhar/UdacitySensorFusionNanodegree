/* \author Aaron Brown */
// Quiz on implementing simple RANSAC line fitting

#include "../../render/render.h"
#include <unordered_set>
#include <random>
#include <cmath>
#include "../../processPointClouds.h"
// using templates for processPointClouds so also include .cpp to help linker
#include "../../processPointClouds.cpp"

pcl::PointCloud<pcl::PointXYZ>::Ptr CreateData()
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
  	// Add inliers
  	float scatter = 0.6;
  	for(int i = -5; i < 5; i++)
  	{
  		double rx = 2*(((double) rand() / (RAND_MAX))-0.5);
  		double ry = 2*(((double) rand() / (RAND_MAX))-0.5);
  		pcl::PointXYZ point;
  		point.x = i+scatter*rx;
  		point.y = i+scatter*ry;
  		point.z = 0;

  		cloud->points.push_back(point);
  	}
  	// Add outliers
  	int numOutliers = 10;
  	while(numOutliers--)
  	{
  		double rx = 2*(((double) rand() / (RAND_MAX))-0.5);
  		double ry = 2*(((double) rand() / (RAND_MAX))-0.5);
  		pcl::PointXYZ point;
  		point.x = 5*rx;
  		point.y = 5*ry;
  		point.z = 0;

  		cloud->points.push_back(point);

  	}
  	cloud->width = cloud->points.size();
  	cloud->height = 1;

  	return cloud;

}

pcl::PointCloud<pcl::PointXYZ>::Ptr CreateData3D()
{
	ProcessPointClouds<pcl::PointXYZ> pointProcessor;
	return pointProcessor.loadPcd("../../../sensors/data/pcd/simpleHighway.pcd");
}


pcl::visualization::PCLVisualizer::Ptr initScene()
{
	pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer ("2D Viewer"));
	viewer->setBackgroundColor (0, 0, 0);
  	viewer->initCameraParameters();
  	viewer->setCameraPosition(0, 0, 15, 0, 1, 0);
  	viewer->addCoordinateSystem (1.0);
  	return viewer;
}

std::unordered_set<int> Ransac2D(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int maxIterations, float distanceTol)
{
	std::unordered_set<int> inliersResult;
	int num_points = cloud->points.size();

    if (num_points < 2) return inliersResult;  // Not enough points
	std::mt19937 rng(std::random_device{}());
    std::uniform_int_distribution<int> dist(0, num_points - 1);
	
	// For max iterations 
	for (int iter = 0; iter < maxIterations; ++iter)
	{
		// Randomly sample subset and fit line
		int idx1 = dist(rng);
        int idx2 = dist(rng);

        // Ensure two distinct points
        while (idx2 == idx1) 
		{
            idx2 = dist(rng);
        }

        const auto& point1 = cloud->points[idx1];
        const auto& point2 = cloud->points[idx2];

        // Fit 2D line: Ax + By + C = 0
        float A = point1.y - point2.y;
        float B = point2.x - point1.x;
        float C = point1.x * point2.y - point2.x * point1.y;

		float norm = std::sqrt(A*A + B*B);
        if (norm == 0) continue; // Degenerate case

		std::unordered_set<int> inliers;

		for (int i = 0; i < num_points; ++i)
		{
			const auto& pt = cloud->points[i];
			// Measure distance between every point and fitted line
			float d = std::fabs(A * pt.x + B * pt.y + C) / norm;
			// If distance is smaller than threshold count it as inlier
			if (d <= distanceTol) 
			{
                inliers.insert(i);
            }
		}

		if (inliers.size() > inliersResult.size()) 
		{
            inliersResult = std::move(inliers);
        }
    }

	// Return indicies of inliers from fitted line with most inliers
	return inliersResult;
}

std::unordered_set<int> RansacPlane(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int maxIterations, float distanceTol)
{
	std::unordered_set<int> inliersResult;
	int num_points = cloud->points.size();

    if (num_points < 3) return inliersResult;  // Not enough points
	std::mt19937 rng(std::random_device{}());
    std::uniform_int_distribution<int> dist(0, num_points - 1);
	
	// For max iterations 
	for (int iter = 0; iter < maxIterations; ++iter)
	{
		// Randomly sample subset and fit plane
		int idx1 = dist(rng);
        int idx2 = dist(rng);
		int idx3 = dist(rng);

        // Ensure three distinct points
        while (idx2 == idx1) idx2 = dist(rng);
		while (idx3 == idx1 || idx3 == idx2) idx3 = dist(rng);

        // Fit 3D line: Ax + By + Cz + D = 0
		const auto& point1 = cloud->points[idx1];
		const auto& point2 = cloud->points[idx2];
		const auto& point3 = cloud->points[idx3];

		// vector calculation
		// vector v1 travels from point1 to point2
		// vector2 travels from point1 to point3
		float v1x = point2.x - point1.x;
		float v1y = point2.y - point1.y;
		float v1z = point2.z - point1.z;
		float v2x = point3.x - point1.x;
		float v2y = point3.y - point1.y;
		float v2z = point3.z - point1.z; 

		// normal vector by taking cross product v1 x v2
        float A = v1y * v2z - v2y * v1z;
        float B = v1z * v2x - v2z * v1x;
        float C = v1x * v2y - v2x * v1y;
		float D = -(A * point1.x + B * point1.y * C* point1.z);

		float norm = std::sqrt(A*A + B*B + C*C);
        if (norm == 0) continue; // Degenerate case

		std::unordered_set<int> inliers;

		for (int i = 0; i < num_points; ++i)
		{
			const auto& pt = cloud->points[i];
			// Measure distance between every point and fitted plane
			float d = std::fabs(A * pt.x + B * pt.y + C * pt.z * D) / norm;
			// If distance is smaller than threshold count it as inlier
			if (d <= distanceTol) 
			{
                inliers.insert(i);
            }
		}

		if (inliers.size() > inliersResult.size()) 
		{
            inliersResult = std::move(inliers);
        }
    }

	// Return indicies of inliers from fitted line with most inliers
	return inliersResult;
}

int main ()
{
	vtkObject::GlobalWarningDisplayOff(); // Turn off VTK warning for deprecated vtkOpenGLPolyDataMapper::GetVertexShaderCode()

	// Create viewer
	pcl::visualization::PCLVisualizer::Ptr viewer = initScene();

	// Create data
	// pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = CreateData();
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = CreateData3D();

	// std::unordered_set<int> inliers = Ransac2D(cloud, 60, 0.6);
	std::unordered_set<int> inliers = RansacPlane(cloud, 60, 0.1);

	pcl::PointCloud<pcl::PointXYZ>::Ptr  cloudInliers(new pcl::PointCloud<pcl::PointXYZ>());
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloudOutliers(new pcl::PointCloud<pcl::PointXYZ>());

	for(int index = 0; index < cloud->points.size(); index++)
	{
		pcl::PointXYZ point = cloud->points[index];
		if(inliers.count(index))
			cloudInliers->points.push_back(point);
		else
			cloudOutliers->points.push_back(point);
	}

	// Render point cloud with inliers and outliers
	if(inliers.size())
	{
		std::cout << cloudInliers->points.size() << " " << cloudOutliers->points.size() << std::endl;
		renderPointCloud(viewer,cloudInliers,"inliers",Color(0,1,0));
  		renderPointCloud(viewer,cloudOutliers,"outliers",Color(1,0,0));
	}
  	else
  	{
		std::cout << "No inliers found" << std::endl;
  		renderPointCloud(viewer,cloud,"data");
  	}
	
  	while (!viewer->wasStopped ())
  	{
  	  viewer->spinOnce ();
  	}
  	
}
