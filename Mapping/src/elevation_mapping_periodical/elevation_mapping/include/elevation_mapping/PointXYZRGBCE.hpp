/*
 * PointXYZRGBCE.hpp
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

struct PointXYZRGBCE
{
    PCL_ADD_POINT4D;
    PCL_ADD_RGB;

    float covariance;
    uint32_t edge;

    PointXYZRGBCE() {}
    PointXYZRGBCE(const PointXYZRGBCE& input)
    {
        this->x = input.x;
        this->y = input.y;
        this->z = input.z;
        this->rgb = input.rgb;
        this->covariance = input.covariance;
        this->edge = input.edge;
    }

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
} EIGEN_ALIGN16;

POINT_CLOUD_REGISTER_POINT_STRUCT (PointXYZRGBCE,
                                   (float, x, x)
                                   (float, y, y)
                                   (float, z, z)
                                   (float, rgb, rgb)
                                   (float, covariance, covariance)
                                   (uint32_t, edge, edge)
);

PCL_INSTANTIATE(VoxelGrid, PointXYZRGBCE);
PCL_INSTANTIATE(KdTree, PointXYZRGBCE);
PCL_INSTANTIATE(KdTreeFLANN, PointXYZRGBCE);

typedef pcl::PointCloud<PointXYZRGBCE> PointCloudXYZRGBCE;
