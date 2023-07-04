// read config file
#include <sys/types.h>
#include <dirent.h>
#include "omp.h"

#include <atomic>
#include <mutex>
#include <math.h>
#include <thread>
#include <string>
#include <stdio.h>
#include <unistd.h>
#include <chrono>   
#include <limits.h>
#include <bitset>
#include <unistd.h>
#include <regex>
#include <forward_list>
#include <unordered_map>
#include <boost/thread.hpp>

// YAML
// #include <yaml-cpp/yaml.h>

// ROS
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <tf2_ros/transform_broadcaster.h>

// open3d
// #include "open3d/Open3D.h"
// #include "open3d/t/pipelines/registration/Registration.h"


// opencv
#include <opencv2/core/core.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>

// global manager
#include <global_manager/kdtree.h>
#include <global_manager/typedefs.h>

#include "distributed_mapper/distributed_mapper_utils.h"
#include <distributed_mapper/evaluation_utils.h>
#include <distributed_mapper/between_chordal_factor.h>

// pcl
#include <pcl_ros/point_cloud.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/gicp.h>
#include <pcl/filters/filter_indices.h>
#include <pcl/common/transforms.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/statistical_outlier_removal.h>

// messages
#include "dislam_msgs/SubMap.h"
#include "dislam_msgs/DiSCO.h"
#include "dislam_msgs/GetInitMap.h"
#include "dislam_msgs/Loop.h"
#include "dislam_msgs/Loops.h"
#include <std_msgs/Bool.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/image_encodings.h>

// tf
#include "tf/transform_datatypes.h"
#include <tf_conversions/tf_eigen.h>
#include <tf/transform_listener.h>

// visualization
#include <global_manager/pose_graph_tool.hpp>

// fast gicp
#include <fast_gicp/gicp/fast_gicp.hpp>
#include <fast_gicp/gicp/fast_vgicp.hpp>

#ifdef USE_VGICP_CUDA
#include <fast_gicp/gicp/fast_vgicp_cuda.hpp>
#endif

// using namespace open3d;
using namespace gtsam;
using namespace chrono;
using namespace distributed_mapper;
// using namespace open3d::t::pipelines::registration;
// using namespace multirobot_util;

namespace global_manager
{
/**
 * @defgroup node ROS node
 * @brief ROS interface.
 * @details Manages maps discovery, transforms estimation and the global
 * map publishing.
 * @{
 */

/**
 * @brief ROS node class.
 * @details Runs robot discovery, transforms estimation and map compositing at
 * predefined rates (from ROS parameters). Does not spin on its own.
 *
 */

// Define robot handler
typedef struct RobotHandle {
  // protects map
  int robot_id;
  bool initState;
  std::mutex mutex;
  SubMapVec submaps;  // submaps get from GEM
  TrajVec trajectory;
  KeyFrameVec keyframes;  // keyframe pointcloud
  PointCloudConstPtr map; // map constructed by submaps
  std::vector<int> disco_index;
  OrthoVector ortho_image;  // not used for this version
  PointCloudI lastKeyframe; // for keyframe pointcloud enhance
  PointCloudI keyframePC; // keyframe pointcloud
  TimeStampVec timestamps;  // timestamp

  DiSCOVec disco_base;  // disco database
  DiSCOFFTVec disco_fft;  // disco fft database
  ros::Subscriber map_sub;  // subscriber for submap
  ros::Subscriber disco_sub;  // subscriber for disco
  ros::Publisher keyframe_pub;  // publisher for keyframe viz
  ros::ServiceClient init_map_client_;  // local map services
  std::string robot_name;
  float transformLast[6];
  float transformCurrent[6];
  float transformAftMapped[6];
  float transformTobeMapped[6];
  
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloudKeyPoses3D;
  pcl::PointCloud<PointTypePose>::Ptr cloudKeyPoses6D;
}robotHandle_;


/*
 * Utility functions
 */
const string robotNames_ = string("abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ");

bool isRotationMatrix(Eigen::Matrix3f R);

void fftshift(std::complex<float> *data, int count);

void swap(std::complex<float> *v1, std::complex<float> *v2);

void addNormal(PointCloudIPtr cloud, PointCloudINPtr cloudWithNormals);

void getFileNames(string path, std::vector<string>& files);

void readConfigs(std::vector<string>& files, std::vector<Eigen::Isometry3f, Eigen::aligned_allocator<Eigen::Isometry3f>>& poses, int start_robot_id_);

std::vector<int> sort_indexes(const std::vector<int> v);

Eigen::Isometry3f Pose3toIsometry(Pose3 T);

Eigen::Isometry3f toIsometry3f(const cv::Mat &cvMat);

Eigen::Vector3d rotationMatrixToEulerAngles(Eigen::Matrix4f &R);

Eigen::Quaternionf euler2Quaternion(const double roll, const double pitch, const double yaw);


class GlobalManager
{
private:

  ros::NodeHandle node_;

  /* node parameters */
  bool use_pcm;
  bool publish_tf;
  bool loopClosureEnable_;
  bool odometryLoopEnable_;  
  bool useOtherDescriptor_;
  bool useRefinedInitials_;
  bool manualRobotConfig_;
  bool enableElevationMapping_;
  double disco_dim_;
  double pcm_thresh_;
  double icp_iters_;
  double submap_size_;
  double disco_width_;
  double disco_height_;
  double start_robot_id_;
  double composing_rate_;
  double discovery_rate_;
  double tf_publish_rate_;
  double pose_graph_pub_rate_;
  double submap_voxel_leaf_size_;
  double globalmap_voxel_leaf_size_;
  double keyframe_search_candidates_;
  double loop_detection_rate_;
  std::string registration_method_;
  std::string manual_config_dir_;
  std::string robot_submap_topic_;
  std::string robot_disco_topic_;
  std::string keyframe_pc_topic_;
  std::string initmap_service_topic_;
  std::string robot_namespace_;
  std::string pg_saving_filename_;
  std::string keyframe_saving_dir_;
  std::string global_map_frame_;
  std::string elevation_map_saving_filename_;
  std::string global_map_saving_filename_;

  // map stacks
  std::mutex map_mutex;
  std::vector<int> merged_map_size;
  PointCloud merged_map;
  PointCloud local_maps;
  PointCloudI merged_pointcloud;
  std::mutex map_tf_mutex;
  std::mutex merged_map_mutex;
  std::vector<PointCloud> local_map_stack;
  std::vector<std::vector<PointCloud>> global_map_stack;

  // pose graph optimization
  std::mutex graph_mutex_;
  std::mutex new_graph_mutex;
  std::vector<Eigen::Isometry3f, Eigen::aligned_allocator<Eigen::Isometry3f>> mapTF;
  std::vector<Eigen::Isometry3f, Eigen::aligned_allocator<Eigen::Isometry3f>> lastTF;
  std::vector<Eigen::Isometry3f, Eigen::aligned_allocator<Eigen::Isometry3f>> currentRef;
  std::vector<pcl::PointXYZI> currentRobotPosPoint;
  GraphAndValuesVec graphAndValuesVec;
  GraphAndValuesVec DistGraphAndValuesVec;
  std::vector<int> disconnectedRobotIDVec;
  std::vector<std::vector<Eigen::Isometry3f, Eigen::aligned_allocator<Eigen::Isometry3f>>> originMapTF;
  std::vector<std::vector<Eigen::Isometry3f, Eigen::aligned_allocator<Eigen::Isometry3f>>> optMapTF;
  noiseModel::Diagonal::shared_ptr priorNoise;
  noiseModel::Base::shared_ptr robustNoiseModel;
  noiseModel::Diagonal::shared_ptr odometryNoise;
  noiseModel::Diagonal::shared_ptr constraintNoise;
  noiseModel::Diagonal::shared_ptr loopNoise;
  int loop_num;
  double timeLaserOdometry;
  bool aLoopIsClosed = false;
  bool keyframeUpdated = false;
  bool mapNeedsToBeCorrected = false;

  // geometry check
  double icp_filter_size_;
  std::queue<std::tuple<uint64_t, uint64_t, Eigen::Isometry3d>> loops_buf;

  // pose graph visualization
  visualization_msgs::Marker trajMarker_;
  visualization_msgs::Marker loopEdgeMarker_;
  visualization_msgs::MarkerArray loopEdges;
  visualization_msgs::MarkerArray trajPoints;
  std::vector<std::pair<uint64_t, uint64_t>> loopEdgePairs_; 
  std::vector<std::vector<geometry_msgs::Point>> loopEdgesVec_; 
  std::vector<std::vector<geometry_msgs::Point>> trajPointsVec_;

  // distributed mapper
  bool debug;
  double gamma = 1.0f; // Gamma value for over relaxation methods
  size_t maxIter = 150; // Maximum number of iterations of optimizer
  bool useLandmarks = false; // use landmarks -- landmarks are given symbols as upper case of robot name, for eg: if robot is 'a', landmark will be 'A'
  bool useFlaggedInit = true; // to use flagged initialization or not
  bool useBetweenNoise = false; // use between factor noise or not
  bool disconnectedGraph = false; // flag to check whether the graphs are connected or not
  bool useChrLessFullGraph = false; // whether full graph has character indexes or not  
  double poseEstimateChangeThreshold = 1e-2; // Difference between pose estimate provides an early stopping condition
  double historyKeyframeSearchRadius = 10.0; // default 10.0; key frame that is within n meters from current pose will be considerd for loop closure
  double rotationEstimateChangeThreshold = 1e-2; // Difference between rotation estimate provides an early stopping condition
  std::vector<boost::shared_ptr<DistributedMapper>> distMappers; // vector of distributed optimizers, one for each robot
  DistributedMapper::UpdateType updateType = DistributedMapper::incUpdate; // updateType differetiates between Distributed Jacobi/Jacobi OverRelaxation (postUpdate) and Gauss-Seidel/Successive OverRelaxation (incUpdate)

  // kdtree for loop closure 
  int disco_ind;
  int DISCO_DIM; // default 1024; the dimension of place recognitiondescriptor
  int latestFrameIDLoopCloure;
  int NUM_CANDIDATES_FROM_TREE;  // default 10; query num from kdtree
  double DISCO_DIST_THRES;
  double acceptedKeyframeFitnessScore; // default 0.3; the smaller the better alignment
  pcl::KdTreeFLANN<PointTI>::Ptr kdtreeHistoryKeyPoses; // for single agent loop detection
  pcl::KdTreeFLANN<PointTypePose>::Ptr kdtreeHistoryKeyPoses6D; // for single agent loop detection

  DiSCOVec disco_mat_;
  struct kdtree *kdtree;
  DiSCOVec disco_to_search_;
  std::mutex kdtree_mutex_;
  std::unique_ptr<KDTree> disco_tree_;

  // publishing
  ros::Publisher merged_pointcloud_publisher_;
  ros::Publisher merged_elevation_map_publisher_;
  ros::Publisher query_cloud_publisher_;
  ros::Publisher database_cloud_publisher_;
  ros::Publisher aligned_cloud_publisher_;

  ros::Publisher pose_graph_publisher_;
  ros::Publisher optimizing_state_publisher_;

  // subscribing
  ros::Subscriber map_saving_subscriber_;
  ros::Subscriber loop_info_subscriber_;

  // transforms for tf
  tf2_ros::TransformBroadcaster tf_publisher_;
  std::atomic_flag tf_current_flag_;  // whether tf_transforms_ are up to date
                                      // with transforms_
  std::vector<geometry_msgs::TransformStamped> tf_transforms_;

  // maps robots namespaces to maps. does not own
  std::vector<int> robotIDStack;  // save robotid
  std::vector<string> configFiles;  // save config absolute filenames
  std::unordered_map<std::string, robotHandle_*> robots_;
  std::vector<Eigen::Isometry3f, Eigen::aligned_allocator<Eigen::Isometry3f>> initPoses;
  std::vector<std::string> robotNameVec;

  // owns maps -- iterator safe
  size_t nrRobots;
  std::mutex subscriptions_mutex_;
  std::forward_list<robotHandle_> subscriptions_;

  void publishTF();
  void savingPoseGraph();
  void savingKeyframes(Values fullInitial);
  void savingGlobalMap(Values fullInitial);
  void savingElevationMap();
  void publishPoseGraph();
  void publishMergedMap();
  void publishMergedPointcloud();
  Key robotID2Key(int robotid);
  int Key2robotID(Key robotKey);

  std::string robotNameFromTopic(const std::string& topic);
  bool isRobotMapTopic(const ros::master::TopicInfo& topic);
  bool isRobotDiSCOTopic(const ros::master::TopicInfo& topic);
  void mapSaving(const std_msgs::Bool::ConstPtr& savingSignal);
  void mapUpdate(const dislam_msgs::SubMapConstPtr& msg, robotHandle_& subscription);
  void discoUpdate(const dislam_msgs::DiSCOConstPtr& msg, robotHandle_& subscription);
  pcl::Registration<PointTI, PointTI>::Ptr select_registration_method(std::string type);

public:
  /**
   * @brief Initiates and get some variables from the launch file under current ROS core.
   * @details When constructed, some prepration is done for further calculation.
   */  
  GlobalManager(ros::NodeHandle private_nh);
  
  /**
   * @brief Destructor.
   * @details When deconstructed, some memories should be released.
   */  
  ~GlobalManager();

  /**
   * @brief Initiates discovery of new robots (maps) under current ROS core.
   * @details When new maps topics are found, they are added for merging. This
   * function is thread-safe
   */
  void discovery();

  /**
   * @brief Subscribe msg from front-end.
   * @details When new msgs are acquired, they are merged into the storage
   */
  void discoveryThread();

  /**
   * @brief Process init situation.
   * @details When the system is init, this thread get init map through service
   */
  void initMapProcessThread();

  /**
   * @brief Loop closing thread.
   * @details Detect and perform loop closing
   */
  void loopClosingThread();

  /**
   * @brief TF publishing thread.
   * @details publish /odom -> /map
   */
  void publishTFThread();

  /**
   * @brief Pose graph publishing thread.
   * @details publish pose graph
   */
  void publishPoseGraphThread();

  /**
   * @brief Map compositing thread.
   * @details Composite a global map
   */
  void mapComposingThread();

  /**
   * @brief Get init map of all robots.
   * @details When the system is init, this process get init map through service
   */
  void getInitMaps();
  
  /**
   * @brief Correct pose using merged pose graph.
   * @details When pose graph is constructed, they are optimized 
   * @return Corrected pose graph and corrected pose size
   */
  std::pair<Values, std::vector<int>> correctPoses();

  /**
   * @brief Perform ICP check between two point cloud.
   * @details When loops info come in ICP check is for reject outliers 
   * @return ICP fitness score and transform
   */
  std::pair<float, Pose3> ICPCheck(uint64_t id1, uint64_t id2, Eigen::Isometry3d initPose);

  /**
   * @brief Perform ICP check in thread.
   * @details Perform icp check in queue
   */
  void geometryCheckThread();

  /**
   * @brief Update transform using the corrected pose graph.
   * @details When pose graph is optimized, the transform is updated 
   */
  void updateTransform(const std::pair<Values, std::vector<int>> correctedPosePair);

  /**
   * @brief Calculate relative orientation between two disco.
   * @details Implement phase correlation ifft part. 
   */
  float calcRelOri(DiSCOFFT newDiSCO, DiSCOFFT oldDiSCO);

  /**
   * @brief Construct full pose graph.
   * @details Collect all subgraph to construct a full 
   * @return Full graphs
   */
  GraphAndValues readFullGraph();

  /**
   * @brief copyInitial copies the initial graph to optimized graph as a fall back option
   * @details Copy the initials of all graphs
   * @return Full initials
   */
  Values copyInitial();
  
  /**
   * @brief Construct distMapper optimizer.
   * @details Collect all subgraph to construct a optimizer 
   * @return Robots' optimize node size
   */
  std::vector<int> constructOptimizer(bool savingMode=false);

  /**
   * @brief Detect inter loop closure.
   * @details Using DiSCO distance for inter loop closing recognize
   */
  std::tuple<std::vector<int>, std::vector<int>, std::vector<pcl::PointXYZI>> detectLoopClosure(std::vector<float> curr_desc, DiSCOFFT curr_fft, int curr_ind, int currentRobotIDinStack);

  /**
   * @brief Perform loop closure.
   * @details Including detection of loop closure and optimize the pose
   */
  void performLoopClosure();

  /**
   * @brief Update Graph and Values after pcm filter.
   * @details Remove outliers detected by PCM and update the graph and values.
   */
  void updateGraphAndValueVec(const gtsam::Values& newValues);

  /**
   * @brief Process loop closure info given by loop detection node.
   * @details add edges to the maintained graph using subscribed loop info
   */
  void processLoopClosureWithInitials(const dislam_msgs::LoopsConstPtr& msg);

  /**
   * @brief Process loop closure info given by loop detection node.
   * @details add edges to the maintained graph using subscribed loop info
   */
  void processLoopClosureWithFinePose(const dislam_msgs::LoopsConstPtr& msg);

  /**
   * @brief Composes and publishes the global map based on estimated
   * transformations
   * @details This function is thread-safe
   */
  void mapComposing();

  /**
   * @brief Compose global map
   * @details This function is thread-safe
   * @return merged map
   */
  PointCloud composeGlobalMap();

  /**
   * @brief Merge nearest keyframe point cloud for icp
   * @details This function is thread-safe
   * @return merged keyframe
   */
  void mergeNearestKeyframes(PointCloudIPtr& Keyframe, int robot_id, int loop_id, int submap_size);

};

///@} group node

}  // namespace global_manager