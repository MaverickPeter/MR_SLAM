/*
 * PointXYZRGBC.hpp
 *
 *  Created on: Dec 1, 2019
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
#include <pcl/kdtree/kdtree.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/kdtree/impl/kdtree_flann.hpp>
#include <pcl/search/impl/kdtree.hpp>
#include <boost/shared_ptr.hpp>

struct PointXYZRGBC
{
    PCL_ADD_POINT4D;
    PCL_ADD_RGB;

    float covariance;

    PointXYZRGBC() {}
    PointXYZRGBC(const PointXYZRGBC& input)
    {
        this->x = input.x;
        this->y = input.y;
        this->z = input.z;
        this->rgb = input.rgb;
        this->covariance = input.covariance;
    }

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
} EIGEN_ALIGN16;

POINT_CLOUD_REGISTER_POINT_STRUCT (PointXYZRGBC,
                                   (float, x, x)
                                   (float, y, y)
                                   (float, z, z)
                                   (float, rgb, rgb)
                                   (float, covariance, covariance)
);

PCL_INSTANTIATE(VoxelGrid, PointXYZRGBC);
PCL_INSTANTIATE(KdTree, PointXYZRGBC);
PCL_INSTANTIATE(KdTreeFLANN, PointXYZRGBC);

typedef pcl::PointCloud<PointXYZRGBC> PointCloudXYZRGBC;
