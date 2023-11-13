#ifndef ELEVATION_MAP_LAYER_H_
#define ELEVATION_MAP_LAYER_H_

#include <math.h>

// grid map
#include <grid_map_msgs/GridMap.h>
#include <grid_map_ros/grid_map_ros.hpp>

// costmap
#include <costmap_2d/obstacle_layer.h>
#include <costmap_2d/GenericPluginConfig.h>

namespace costmap_2d
{

class ElevationMapLayer : public ObstacleLayer
{
public:
  ElevationMapLayer();

  virtual ~ElevationMapLayer();

  virtual void onInitialize();

  virtual void updateBounds(double robot_x, double robot_y, double robot_yaw, 
                            double* min_x, double* min_y, 
                            double* max_x, double* max_y);

private:
  void elevationMapCB(const grid_map_msgs::GridMapConstPtr& msg);

  ros::Subscriber elevation_map_sub_;
  grid_map::GridMap elevation_map_;

  bool elevation_map_available_;
  double travers_thresh;
  
};

}
#endif
