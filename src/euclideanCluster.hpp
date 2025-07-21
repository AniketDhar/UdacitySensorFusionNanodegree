#ifndef EUCLIDEAN_CLUSTERS_HPP
#define EUCLIDEAN_CLUSTERS_HPP

#include "kdtree.h"
#include <pcl/point_cloud.h>
#include <vector>


template <typename PointT>
void clusterHelper(const int& idx, typename pcl::PointCloud<PointT>::Ptr cloud, std::vector<bool>& processed, std::vector<int>& cluster, KdTree* tree, float distanceTol)
{
	processed[idx] = true;
	cluster.emplace_back(idx);

    std::vector<float> target{cloud->points[idx].x, cloud->points[idx].y, cloud->points[idx].z};
	std::vector<int> nearbyIdx = tree->search(target, distanceTol);
	for(const auto& id : nearbyIdx)
	{
		if(!processed[id])
		{
			clusterHelper<PointT>(id, cloud, processed, cluster, tree, distanceTol);
		}
	}
}

template <typename PointT>
std::vector<std::vector<int>> euclideanCluster(typename pcl::PointCloud<PointT>::Ptr cloud, KdTree* tree, float distanceTol)
{
	std::vector<std::vector<int>> clusters;
	std::vector<bool> processed(cloud->size(), false);

	for(int i = 0; i < cloud->size(); ++i)
	{
		if(!processed[i])
		{
			std::vector<int> cluster;
			clusterHelper<PointT>(i, cloud, processed, cluster, tree, distanceTol);
			clusters.emplace_back(cluster);
		}
	}

	return clusters;

}

#endif // EUCLIDEAN_CLUSTERS_HPP