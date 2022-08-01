/*
 * ElevationMap.hpp
 *
 *  Created on: Dec 22, 2019
 *      Author: Peter XU
 *	 Institute: ZJU, CSC 104
 */

#pragma once

// Grid Map
#include <grid_map_ros/grid_map_ros.hpp>

// OpenCV
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>
#include <cv_bridge/cv_bridge.h>

// Eigen
#include <Eigen/Core>
#include <Eigen/Geometry>

// PCL
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

// Kindr
#include <kindr/Core>

// Boost
#include <boost/thread/recursive_mutex.hpp>

// ROS
#include <ros/ros.h>
#include <string>

using namespace std;

namespace elevation_mapping {
// typedef PointXYZRGBIC Anypoint;
// typedef pcl::PointCloud<Anypoint> pointCloud;

/*!
 * Elevation map stored as grid map handling elevation height, variance, color etc.
 */
class ElevationMap
{
 public:

  /*!
   * Constructor.
   */
  ElevationMap(ros::NodeHandle nodeHandle, string robot_name);

  /*!
   * Destructor.
   */
  virtual ~ElevationMap();

  /*!
   * Set the geometry of the elevation map. Clears all the data.
   * @param length the side lengths in x, and y-direction of the elevation map [m].
   * @param resolution the cell size in [m/cell].
   * @param position the 2d position of the elevation map in the elevation map frame [m].
   * @return true if successful.
   */
  void setGeometry(const grid_map::Length& length, const double& resolution, const grid_map::Position& position);

  /*!
   * Clears all data of the elevation map (data and time).
   * @return true if successful.
   */
  bool clear();

  /*!
   * Move the grid map w.r.t. to the grid map frame.
   * @param position the new location of the elevation map in the map frame.
   */
  void move(const grid_map::Index M_startindex, grid_map::Position M_position);

  void opt_move(grid_map::Position M_position, float update_height);
  
  /*!
   * Gets a reference to the raw grid map.
   * @return the raw grid map.
   */
  grid_map::GridMap& getRawGridMap();

  /*!
   * Gets a reference to the fused grid map.
   * @return the fused grid map.
   */
  grid_map::GridMap& getFusedGridMap();

  /*!
   * Gets the time of last map update.
   * @return time of the last map update.
   */
  ros::Time getTimeOfLastUpdate();

  /*!
   * Gets the time of last map fusion.
   * @return time of the last map fusion.
   */
  ros::Time getTimeOfLastFusion();

  /*!
   * Get the pose of the elevation map frame w.r.t. the inertial parent frame of the robot (e.g. world, map etc.).
   * @return pose of the elevation map frame w.r.t. the parent frame of the robot.
   */
  const kindr::HomTransformQuatD& getPose();

  /*!
   * Gets the position of a raw data point (x, y of cell position & height of cell value) in
   * the parent frame of the robot.
   * @param index the index of the requested cell.
   * @param position the position of the data point in the parent frame of the robot.
   * @return true if successful, false if no valid data available.
   */
  bool getPosition3dInRobotParentFrame(const Eigen::Array2i& index, kindr::Position3D& position);

  /*!
   * Gets the fused data mutex.
   * @return reference to the fused data mutex.
   */
  boost::recursive_mutex& getFusedDataMutex();

  /*!
   * Gets the raw data mutex.
   * @return reference to the raw data mutex.
   */
  boost::recursive_mutex& getRawDataMutex();

  /*!
   * Set the frame id.
   * @param frameId the frame id.
   */
  void setFrameId(const std::string& frameId);

  /*!
   * Get the frame id.
   * @return the frameId.
   */
  const std::string& getFrameId();

  friend class ElevationMapping;

 private:

  /*!
   * Reads and verifies the ROS parameters.
   * @return true if successful.
   */
  bool readParameters();

  void dfs(vector<std::vector<int>>& map, int x, int y, int length);

  grid_map::GridMap findFrontiers(grid_map::GridMap& gridMap, int length);

  sensor_msgs::ImagePtr show(ros::Time timestamp, std::string robot_name, float trackPointTransformed_x, float trackPointTransformed_y, int length, float *elevation, float *var, int *point_colorR, int *point_colorG, int *point_colorB, float *rough, float *slope, float *traver, float *intensity);

  //! ROS nodehandle.
  ros::NodeHandle nodeHandle_;

  //! Raw elevation map as grid map.
  grid_map::GridMap rawMap_;
  
  grid_map::GridMap visualMap_;

  //! Underlying map, used for ground truth maps, multi-robot mapping etc.
  grid_map::GridMap underlyingMap_;

  //! True if underlying map has been set, false otherwise.
  bool hasUnderlyingMap_;

  //! Pose of the elevation map frame w.r.t. the inertial parent frame of the robot (e.g. world, map etc.).
  kindr::HomTransformQuatD pose_;

  //! ROS publishers.
  ros::Publisher elevationMapRawPublisher_;
  ros::Publisher orthomosaicPublisher_;
  ros::Publisher visualMapPublisher_;
  ros::Publisher VpointsPublisher_;
  ros::Publisher frontierPublisher_;

  //! Underlying map subscriber.
  ros::Subscriber underlyingMapSubscriber_;

  std::string orthoDir;
  string robot_name_;
  string robot_id;

  //! For exploration map generation
  std::vector<std::vector<int>> tmpMap;
  std::vector<std::vector<int>> floodMap;
  std::vector<std::vector<int>> dilatedMap;

  int dx[8] = {0,0,-1,1,-1,-1,1,1};
  int dy[8] = {1,-1,0,0,-1,1,-1,1};

  //! Parameters. Are set through the ElevationMapping class.
  double minVariance_;
  double maxVariance_;
  double mahalanobisDistanceThreshold_;
  double multiHeightNoise_;
  double minHorizontalVariance_;
  double maxHorizontalVariance_;
  double visibilityCleanupDuration_;
  double scanningDuration_;
  std::string underlyingMapTopic_;
  bool enableVisibilityCleanup_;
};

} /* namespace */
