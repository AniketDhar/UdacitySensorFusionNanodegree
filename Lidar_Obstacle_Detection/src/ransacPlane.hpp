#ifndef RANSAC_PLANE_HPP
#define RANSAC_PLANE_HPP

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <unordered_set>
#include <random>
#include <cmath>

template <typename PointT>
std::unordered_set<int> RansacPlane(typename pcl::PointCloud<PointT>::Ptr cloud,
                                    int maxIterations,
                                    float distanceTol)
{
    std::unordered_set<int> inliersResult;
    const int num_points = cloud->points.size();
    if (num_points < 3) return inliersResult;

    std::mt19937 rng(std::random_device{}());
    std::uniform_int_distribution<int> dist(0, num_points - 1);

    for (int iter = 0; iter < maxIterations; ++iter)
    {
        // Select 3 unique random points
        int idx1 = dist(rng), idx2 = dist(rng), idx3 = dist(rng);
        while (idx2 == idx1) idx2 = dist(rng);
        while (idx3 == idx1 || idx3 == idx2) idx3 = dist(rng);

        const auto& p1 = cloud->points[idx1];
        const auto& p2 = cloud->points[idx2];
        const auto& p3 = cloud->points[idx3];

        // Compute normal vector from cross product
        float v1x = p2.x - p1.x, v1y = p2.y - p1.y, v1z = p2.z - p1.z;
        float v2x = p3.x - p1.x, v2y = p3.y - p1.y, v2z = p3.z - p1.z;

        float A = v1y * v2z - v2y * v1z;
        float B = v1z * v2x - v2z * v1x;
        float C = v1x * v2y - v2x * v1y;
        float D = -(A * p1.x + B * p1.y + C * p1.z);

        float norm = std::sqrt(A * A + B * B + C * C);
        if (norm == 0) continue;

        std::unordered_set<int> inliers;
        for (int i = 0; i < num_points; ++i)
        {
            const auto& pt = cloud->points[i];
            float d = std::fabs(A * pt.x + B * pt.y + C * pt.z + D) / norm;
            if (d <= distanceTol)
                inliers.insert(i);
        }

        if (inliers.size() > inliersResult.size())
            inliersResult = std::move(inliers);
    }

    return inliersResult;
}

#endif //RANSAC_PLANE_HPP