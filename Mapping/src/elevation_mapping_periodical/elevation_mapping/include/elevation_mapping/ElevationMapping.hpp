/*
 * ElevationMapping.hpp
 *
 *  Created on: Nov 12, 2019
 *      Author: Peter XU
 *	 Institute: ZJU, CSC 104
 */

#pragma once

// Elevation Mapping
#include "elevation_mapping/ElevationMap.hpp"
#include "elevation_mapping/RobotMotionMapUpdater.hpp"
#include "elevation_mapping/sensor_processors/SensorProcessorBase.hpp"
#include "elevation_mapping/WeightedEmpiricalCumulativeDistributionFunction.hpp"
#include "elevation_mapping/sensor_processors/StructuredLightSensorProcessor.hpp"
#include "elevation_mapping/sensor_processors/StereoSensorProcessor.hpp"
#include "elevation_mapping/sensor_processors/LaserSensorProcessor.hpp"
#include "elevation_mapping/sensor_processors/PerfectSensorProcessor.hpp"
#include "elevation_mapping/GridUtilHash.hpp"
#include "dislam_msgs/SubMap.h"
#include "dislam_msgs/GetInitMap.h"

// Grid Map
#include <grid_map_msgs/GetGridMap.h>
#include <grid_map_msgs/ProcessFile.h>
#include <grid_map_ros/grid_map_ros.hpp>
#include <grid_map_msgs/GridMap.h>

// Eigen
#include <Eigen/Core>
#include <Eigen/Geometry>

// PCL
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/features/normal_3d.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/common/transforms.h> 
#include <pcl/ModelCoefficients.h>
#include <pcl/features/feature.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/surface/mls.h>
#include <pcl/surface/bilateral_upsampling.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/search/search.h>

// ROS
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <octomap_msgs/Octomap.h>
#include <message_filters/cache.h>
#include <message_filters/subscriber.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/subscriber.h>
#include <std_srvs/Empty.h>
#include <message_filters/sync_policies/approximate_time.h>

// Boost
#include <boost/thread.hpp>
#include <math.h>
#include <assert.h>
#include <nav_msgs/Odometry.h>
#include <slam_msg/Keyframe.h>
#include <slam_msg/Keyframes.h>
#include <std_msgs/Bool.h>
#include <mutex>

//octomap
#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/GetOctomap.h>
#include <octomap_msgs/BoundingBoxQuery.h>
#include <octomap_msgs/conversions.h>

#include <octomap_ros/conversions.h>
#include <octomap/octomap.h>
#include <octomap/OcTreeKey.h>
#include <octomap/ColorOcTree.h>


using namespace pcl;
typedef unordered_map<const GridPoint, GridPointData, GridPointHashFunc, GridPointEqual> umap;

namespace elevation_mapping {
/*!
 * The elevation mapping main class. Coordinates the ROS interfaces, the timing,
 * and the data handling between the other classes.
 */


/********** Utility functions **********/
/*!
  * cv::Mat converter
  */
Eigen::Matrix<double,3,4> toMatrix34(const cv::Mat &cvMat);
Eigen::Matrix<double,4,4> toMatrix44(const cv::Mat &cvMat);

/*!
  * MLS-based pointcloud interpolation
  */
void pointcloudinterpolation(pointCloud::Ptr &input);


class ElevationMapping
{
 typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2, sensor_msgs::Image> MySyncPolicy;

 public:

  /*!
   * Constructor.
   * @param nodeHandle the ROS node handle.
   */
  ElevationMapping(ros::NodeHandle& nodeHandle, string robot_name);

  /*!
   * Destructor.
   */
  virtual ~ElevationMapping();

  /*!
  * Subscribers start
  */
  void Run();

  /*!
  * Synchronization of image and point cloud start
  */  
  void startsync();

  /*!
  * Visualize point cloud
  */
  void visualPointMap();

  /*!
  * Visualize Octo map
  */
  void visualOctomap();

  /*!
  * Composing global map
  */
  void composingGlobalMap();

  /*!
  * Composing global map thread
  */
  void composingGlobalMapThread();

  /*!
  * Merge input cloud to the existing local map
  */
  void updateLocalMap(const sensor_msgs::PointCloud2ConstPtr& rawPointCloud);
  
  /*!
  * Correct and merge submaps to a global map
  */
  void updateGlobalMap();

  /*!
  * Subscribe map saving signal
  */
  void mapSavingSignal(const std_msgs::Bool::ConstPtr& saveSignal);
  
  /*!
  * Subscribe optimization frames
  */ 
  void optKeyframeCallback(const slam_msg::Keyframes::ConstPtr& optKeyFrame);

  /*!
  * Subscribe new keyframe signal
  */ 
  void newKeyframeSignal(const std_msgs::Bool::ConstPtr& newKeyframeSignal);

  /*!
  * Subscribe dense map buiding signal
  */
  void denseMappingSignal(const std_msgs::Bool::ConstPtr& denseSignal);

  /*!
   * Callback function for new data to be added to the elevation map.
   * @param rawPointCloud the point cloud to be fused with the existing data.
   * @param image the image to be fused with the existing data.
   */
  void Callback(const sensor_msgs::PointCloud2ConstPtr& rawPointCloud, const sensor_msgs::Image::ConstPtr& image);
  
  /*!
  * Process grid map cells
  */
  void processmapcells();

  /*!
   * Process input point cloud in main Callback function
   * @param pointCloud the raw point cloud to be fused with the existing data.
   */
  void processpoints(pcl::PointCloud<Anypoint>::ConstPtr pointCloud);
  
  /*!
   * Service for retrieving incremental submap.
   */
  bool mapService(dislam_msgs::GetInitMap::Request &req, dislam_msgs::GetInitMap::Response &res);

  
  
  /********** Utility functions **********/

  double calculate_memory_usage();

  /*!
  * Convert from hash map to point cloud
  */  
  void localHashtoPointCloud(umap local_map, pointCloud::Ptr& outCloud);
  
  /*!
  * Convert from point cloud to hash map
  */  
  void pointCloudtoHash(pointCloud localPointCloud, umap& out);
  
  /*!
  * Convert from grid map to point cloud
  */
  void gridMaptoPointCloud(grid_map::GridMap& gridmap, pointCloud::Ptr& pc);
  
  /*!
  * Saving point cloud map to a pcd file
  */
  void pointCloudtoOctomap(pointCloud localPointCloud, octomap::ColorOcTree& tree);
  
  /*!
   * Saving point cloud map to a pcd file
   */
  void savingMap();

  /*!
  * Saving submap point cloud to pcd files
  */
  void savingSubMap();

  //! names
  string robot_id;
  string submapDir;
  string robot_name;
  string map_saving_file_;
  string cameraParamsFile;
  string map_frame_id;


 private:

  /*!
   * Reads and verifies the ROS parameters.
   * @return true if successful.
   */
  bool readParameters();

  /*!
   * Performs the initialization procedure.
   * @return true if successful.
   */
  bool initialize();

  /*!
   * Update the elevation map from the robot motion up to a certain time.
   * @param time to which the map is updated to.
   * @return true if successful.
   */
  bool updatePrediction(const ros::Time& time);

  /*!
   * Updates the location of the map to follow the tracking point. Takes care
   * of the data handling the goes along with the relocalization.
   * @return true if successful.
   */
  bool updateMapLocation();

  /*!
   * Updates the location of the pointcloud map
   * @return true if successful.
   */
  bool updatepointsMapLocation(const ros::Time& timeStamp);

  /*!
   * Reset and start the map update timer.
   */
  void resetMapUpdateTimer();

  /*!
   * Stop the map update timer.
   */
  void stopMapUpdateTimer();

  pcl::PointCloud<Anypoint>::Ptr pointCloudProcessed;

  //! ROS nodehandle.
  ros::NodeHandle& nodeHandle_;

  //! ROS subscribers.
  ros::Subscriber pointCloudSubscriber_;
  message_filters::Subscriber<geometry_msgs::PoseWithCovarianceStamped> robotPoseSubscriber_;

  //! Callback thread for the fusion services.
  boost::thread fusionServiceThread_;

  //! Callback queue for fusion service thread.
  ros::CallbackQueue fusionServiceQueue_;

  //! Cache for the robot pose messages.
  message_filters::Cache<geometry_msgs::PoseWithCovarianceStamped> robotPoseCache_;

  //! Gridmap scale.
  int length_;

  //! Keyframe num for optimization
  int optKeyframeNum;

  //! Gridmap resolution.
  double resolution_;

  //! Current location.
  float trackPointTransformed_x;
  float trackPointTransformed_y;
  float trackPointTransformed_z;

  //! Some flags for trigger
  bool optFlag; // optimization flag
  bool JumpFlag;  // odom jump flag
  bool initFlag;  // initialization flag
  bool denseSubmap; // dense submap flag
  bool JumpOdomFlag;  // odom change flag
  bool newLocalMapFlag; // new local map flag
  bool insertionFlag;
  int JumpCount;

  //! initial cache size of robot pose
  int robotPoseCacheSize_;
  
  //! local map size in meters
  double localMapSize_;

  //! traversity threshold
  double traversThre;

  //! for loop closure
  float prev_center_z;
  float* preopt_position;
  float position_shift[2];
  float current_position[2];
  float later_trackPointTransformed_z;

  //! local hash map 
  umap localMap_;

  ofstream Record_txt;
  int frame;

  //! orthoImage 
  sensor_msgs::ImagePtr orthoImage;

  //! octomap
  double octoResolution_;
  octomap::ColorOcTree *octoTree;
  octomap::ColorOcTree *globalOctoTree;

  //! visual point clouds
  pointCloud global_map;
  pointCloud visualCloud_;
  pointCloud visualCloudOCC_;
  pointCloud localPointMap_;

  //! trajectory storage
  vector<Eigen::Isometry3f> trajectory_;

  //! publishers and subscribers
  ros::Publisher subMapPublisher_;
  ros::Publisher lastmapPublisher_;
  ros::Publisher octomapPublisher_;
  ros::Publisher pointMapPublisher_;
  ros::Publisher globalMapPublisher_;
  ros::Publisher keyFramePCPublisher_;
  ros::Publisher globalOctomapPublisher_;
  ros::Subscriber optKeyframeSub_;
  ros::Subscriber keyFrameSignalSub_;
  ros::Subscriber savingSignalSub_;
  ros::Subscriber denseSubmapSignalSub_;
  ros::ServiceServer mapServiceServer_;

  //! map handles 
  vector<PointXY> localMapLoc_;
  vector<pointCloud> globalMap_;
  vector<Eigen::Isometry3f> optGlobalMapLoc_;

  //! grid map utils
  grid_map::GridMap prevMap_;
  grid_map::Position prev_position;

  //! message filter for synchronization
  message_filters::Connection connection;
  message_filters::Subscriber<sensor_msgs::PointCloud2> vel_sub;
  message_filters::Subscriber<sensor_msgs::Image> cameraR_sub;
  message_filters::Synchronizer<MySyncPolicy> sync; // sync must defined after related subscribers

  //! mutex for multi-thread
  std::mutex LocalMapMutex_;
  std::mutex GlobalMapMutex_;
  boost::recursive_mutex MapMutex_;

  //! TF listener and broadcaster.
  tf::TransformListener transformListener_;
  tf::StampedTransform trackPoseTransformed_;

  //! Point which the elevation map follows.
  kindr::Position3D trackPoint_;
  std::string trackPointFrameId_;

  //! ROS topics for subscriptions.
  std::string pointCloudTopic_;
  std::string robotPoseTopic_;

  //! Elevation map.
  ElevationMap map_;
  ElevationMap gmap_;

  //! Sensor processors.
  SensorProcessorBase::Ptr sensorProcessor_;

  //! Robot motion elevation map updater.
  RobotMotionMapUpdater robotMotionMapUpdater_;

  //! If true, robot motion updates are ignored.
  bool ignoreRobotMotionUpdates_;

  //! Time of the last point cloud update.
  ros::Time lastPointCloudUpdateTime_;

  //! Timer for the robot motion update.
  ros::Timer mapUpdateTimer_;

  //! Maximum time that the map will not be updated.
  ros::Duration maxNoUpdateDuration_;

  //! Time tolerance for updating the map with data before the last update.
  //! This is useful when having multiple sensors adding data to the map.
  ros::Duration timeTolerance_;

  //! If map is fused after every change for debugging/analysis purposes.
  bool isContinouslyFusing_;

  //! Z-axis value after loop closing
  float closeLoopZ;

};

} /* namespace */
