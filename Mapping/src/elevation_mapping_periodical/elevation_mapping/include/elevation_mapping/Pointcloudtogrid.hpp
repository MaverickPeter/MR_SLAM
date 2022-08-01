/*
 * Pointcloudtogrid.hpp
 *
 *  Created on: Aug 1, 2019
 *      Author: Peter XXC
 *	 Institute: ZJU, Robotics 104
 */

#pragma once

// Elevation Mapping
#include "elevation_mapping/ElevationMap.hpp"
#include "elevation_mapping/RobotMotionMapUpdater.hpp"
#include "elevation_mapping/sensor_processors/SensorProcessorBase.hpp"
#include "elevation_mapping/WeightedEmpiricalCumulativeDistributionFunction.hpp"

// Grid Map
#include <grid_map_msgs/GetGridMap.h>
#include <grid_map_msgs/ProcessFile.h>
#include "grid_map_core/TypeDefs.hpp"
#include "grid_map_core/SubmapGeometry.hpp"
#include "grid_map_core/BufferRegion.hpp"

// Eigen
#include <Eigen/Core>
#include <Eigen/Geometry>

// PCL
#include <pcl/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h>

// ROS
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <message_filters/cache.h>
#include <message_filters/subscriber.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <std_srvs/Empty.h>

// Boost
#include <boost/thread.hpp>

// Eigen
#include <Eigen/Core>
#include <Eigen/Geometry>

#define NOT_OCCUPIED 0
#define OCCUPIED 1
#define OUT_OF_SCOPE 2

using namespace grid_map;
using namespace pcl;

struct Cell_feature
{
    //short cell_position[2] = {-1, -1};
    float highest = 0.0;
    float lowest = 0.0;
    float h_var = 0.0;
    float l_var = 0.0;
    int flag = NOT_OCCUPIED;
};

namespace elevation_mapping {

class PointcloudToGrid
{

public:

    /*!
   * Constructor.
   */
    PointcloudToGrid();

    /*!
    * Destructor.
    */
    virtual ~PointcloudToGrid();

    /*!
    * Callback function for processing pointcloud to grid map.
    * @param pointCloud the point cloud to be processed.
    */
    
    void pointCloudProcess(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr &InputCloud, Eigen::VectorXf& variances, struct Cell_feature cell_with_pos[]);

    /*!
    * Set the geometry of the grid map.
    * @param length the side lengths in x, and y-direction of the grid map [m].
    * @param resolution the cell size in [m/cell].
    */
    void setScale(const Length& length, const double resolution);

private:

    //! Side length of the map in x- and y-direction [m].
    Length length_;
    
    //! Map resolution in xy plane [m/cell].
    double resolution_;

    //! Map grid cell number.
    Index grid_num;
      
    //! Size of the buffer (rows and cols of the data structure).
    Size size_;

    //! cell struct.
    //Cell_feature *cell_with_pos;

};

}