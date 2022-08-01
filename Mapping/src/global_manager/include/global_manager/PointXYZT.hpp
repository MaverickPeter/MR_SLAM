/*
 * PointXYZT.hpp
 *
 *  Created on: March 7, 2022
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

struct PointXYZT
{
    PCL_ADD_POINT4D;
    float travers;

    PointXYZT() {}
    PointXYZT(const PointXYZT& input)
    {
        this->x = input.x;
        this->y = input.y;
        this->z = input.z;
        this->travers = input.travers;
    }

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
} EIGEN_ALIGN16;

POINT_CLOUD_REGISTER_POINT_STRUCT (PointXYZT,
                                   (float, x, x)
                                   (float, y, y)
                                   (float, z, z)
                                   (float, travers, travers)
);

PCL_INSTANTIATE(VoxelGrid, PointXYZT);
PCL_INSTANTIATE(KdTree, PointXYZT);
PCL_INSTANTIATE(KdTreeFLANN, PointXYZT);

typedef pcl::PointCloud<PointXYZT> PointCloudXYZT;
