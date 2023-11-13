#ifndef POINTMAP_LAYER_H_
#define POINTMAP_LAYER_H_
#include <ros/ros.h>
#include <std_msgs/Bool.h>

// Costmap
#include <costmap_2d/obstacle_layer.h>
#include <costmap_2d/GenericPluginConfig.h>
#include <dynamic_reconfigure/server.h>
#include <nav_msgs/OccupancyGrid.h>
#include <fstream>

// PCL
#include <pcl/PCLPointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pointMap_layer/PointXYZT.hpp>


using namespace pcl;

namespace costmap_2d
{
  std::ofstream Record_txt;
  int frame;

typedef PointXYZT Anypoint;
typedef pcl::PointCloud<Anypoint> pointCloud;

class PointMapLayer : public ObstacleLayer
{
public:
  PointMapLayer();
  virtual ~PointMapLayer();

  virtual void onInitialize();
  virtual void updateBounds(double robot_x, double robot_y, double robot_yaw, double* min_x, double* min_y, double* max_x,
                              double* max_y);
  virtual void updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j);


private:
  void pointMapCB(const sensor_msgs::PointCloud2& msg);
  void optSignalCB(const std_msgs::Bool& msg);

  int count;
  bool isUpdated;
  bool isOptimizing;
  bool rolling_window_;
  double mark_x_, mark_y_;
  double travers_thresh;
  double z_thresh;
  int thresh_type;

  pointCloud ob_pointCloud;
  ros::Subscriber point_map_sub_;
  ros::Subscriber map_opt_sub_;

};
}
#endif
