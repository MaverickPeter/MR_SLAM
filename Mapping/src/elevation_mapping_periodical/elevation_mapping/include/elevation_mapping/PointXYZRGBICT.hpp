/*
 * PointXYZRGBICT.hpp
 *
 *  Created on: July 1, 2021
 *      Author: Peter XU
 *	 Institute: ZJU, Robotics 104
 */

#define PCL_NO_PRECOMPILE

#include <pcl/pcl_macros.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/impl/instantiate.hpp>

#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/impl/voxel_grid.hpp>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/impl/passthrough.hpp>
#include <pcl/kdtree/kdtree.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/kdtree/impl/kdtree_flann.hpp>
#include <pcl/search/impl/kdtree.hpp>
#include <boost/shared_ptr.hpp>

struct PointXYZRGBICT
{
    PCL_ADD_POINT4D;
    PCL_ADD_RGB;

    float covariance;
    float intensity;
    float travers;

    PointXYZRGBICT() {}
    PointXYZRGBICT(const PointXYZRGBICT& input)
    {
        this->x = input.x;
        this->y = input.y;
        this->z = input.z;
        this->rgb = input.rgb;
        this->intensity = input.intensity;
        this->covariance = input.covariance;
        this->travers = input.travers;
    }

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
} EIGEN_ALIGN16;

POINT_CLOUD_REGISTER_POINT_STRUCT (PointXYZRGBICT,
                                   (float, x, x)
                                   (float, y, y)
                                   (float, z, z)
                                   (float, rgb, rgb)
                                   (float, intensity, intensity)
                                   (float, covariance, covariance)
                                   (float, travers, travers)
);

PCL_INSTANTIATE(VoxelGrid, PointXYZRGBICT);
PCL_INSTANTIATE(KdTree, PointXYZRGBICT);
PCL_INSTANTIATE(KdTreeFLANN, PointXYZRGBICT);

typedef pcl::PointCloud<PointXYZRGBICT> PointCloudXYZRGBICT;
