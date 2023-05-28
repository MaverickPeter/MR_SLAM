#ifndef POINTMAP_LAYER_H_
#define POINTMAP_LAYER_H_
#include <ros/ros.h>

// Costmap
#include <costmap_2d/obstacle_layer.h>
#include <costmap_2d/GenericPluginConfig.h>
#include <dynamic_reconfigure/server.h>
#include <nav_msgs/OccupancyGrid.h>

// PCL
#include <pcl/PCLPointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pointMap_layer/PointXYZRGBICT.hpp>


using namespace pcl;

namespace costmap_2d
{

typedef PointXYZRGBICT Anypoint;
typedef pcl::PointCloud<Anypoint> pointCloud;

class PointMapLayer : public ObstacleLayer
{
public:
  PointMapLayer();
  virtual ~PointMapLayer();

  virtual void onInitialize();
  virtual void updateBounds(double robot_x, double robot_y, double robot_yaw, double* min_x, double* min_y, double* max_x,
                              double* max_y);

private:
  void pointMapCB(const sensor_msgs::PointCloud2& msg);

  double mark_x_, mark_y_;
  pointCloud ob_pointCloud;
  ros::Subscriber point_map_sub_;

};
}
#endif
