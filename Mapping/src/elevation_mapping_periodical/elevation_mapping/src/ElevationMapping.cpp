/*
 * ElevationMapping.cpp
 *
 *  Created on: Nov 12, 2019
 *      Author: Peter XU
 *	 Institute: ZJU, CSC 104
 */
#include "elevation_mapping/ElevationMapping.hpp"

#include <thread>

//multi threads
#include <cstdlib>
#include <pthread.h>
#include <unistd.h>

// opencv
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

// Kindr
#include <kindr/Core>
#include <kindr_ros/kindr_ros.hpp>

// Boost
#include <boost/bind.hpp>
#include <boost/make_shared.hpp>
#include <boost/thread/recursive_mutex.hpp>

// STL
#include <string>
#include <math.h>
#include <limits>

using namespace std;
using namespace grid_map;
using namespace ros;
using namespace tf;
using namespace pcl;
using namespace kindr;
using namespace kindr_ros;

// GPU functions api
void Move(float *current_Position, float resolution, int length, float *h_central_coordinate, int *h_start_indice, float *position_shift);
void Init_GPU_elevationmap(int length, float resolution, float h_mahalanobisDistanceThreshold_, float h_obstacle_threshold);
void Map_closeloop(float *update_position, float height_update, int length, float resolution);
void Raytracing(int length_);
void Fuse(int length, int point_num, int *point_index, int *point_colorR, int *point_colorG, int *point_colorB, float *point_intensity, float *point_height, float *point_var);
void Map_feature(int length, float *elevation, float *var, int *point_colorR, int *point_colorG, int *point_colorB, float *rough, float *slope, float *traver, float *intensity);
void Map_optmove(float *opt_p, float height_update, float resolution,  int length, float *opt_alignedPosition);

namespace elevation_mapping {

class ElevationMapping;

ElevationMapping::ElevationMapping(ros::NodeHandle& nodeHandle, string robot_name_)
    : nodeHandle_(nodeHandle),
      map_(nodeHandle, robot_name_),
      gmap_(nodeHandle, robot_name_),
      robot_name(robot_name_),
      robotMotionMapUpdater_(nodeHandle),
      isContinouslyFusing_(false),
      ignoreRobotMotionUpdates_(false),
      vel_sub(nodeHandle, "/voxel_grid/output", 1),
      cameraR_sub(nodeHandle, "/stereo_grey/left/image_raw", 1),
      sync(MySyncPolicy(200), vel_sub, cameraR_sub)
{
  ROS_INFO("Elevation mapping node started.");
  
  // hash initialization
  localMap_.rehash(10000);
 
  // publishers
  pointMapPublisher_ = nodeHandle_.advertise<sensor_msgs::PointCloud2>("/" + robot_name + "/history_point", 1);
  subMapPublisher_ = nodeHandle_.advertise<dislam_msgs::SubMap>("/" + robot_name + "/submap", 1);
  globalMapPublisher_ = nodeHandle_.advertise<sensor_msgs::PointCloud2>("/" + robot_name + "/global_point", 1);
  lastmapPublisher_ =  nodeHandle_.advertise<grid_map_msgs::GridMap>("/" + robot_name + "/opt_map", 1);
  keyFramePCPublisher_ = nodeHandle_.advertise<sensor_msgs::PointCloud2>("/" + robot_name + "/keyframe_pc", 1);
  octomapPublisher_ = nodeHandle_.advertise<octomap_msgs::Octomap>("/" + robot_name + "/local_octomap", 1);
  globalOctomapPublisher_ = nodeHandle_.advertise<octomap_msgs::Octomap>("/" + robot_name + "/global_octomap", 1);
  mapServiceServer_ = nodeHandle_.advertiseService("/" + robot_name + "/map_srv", &ElevationMapping::mapService, this);

  readParameters();
  initialize();
}


/*
 * Destruct function
 */
ElevationMapping::~ElevationMapping()
{
  nodeHandle_.shutdown();
}


/*
 * Map saving signal callback
 */
void ElevationMapping::Run()
{
  // subscriber for map saving api
  denseSubmapSignalSub_ = nodeHandle_.subscribe("/" + robot_name + "/dense_mapping", 1, &ElevationMapping::denseMappingSignal, this);
  savingSignalSub_ = nodeHandle_.subscribe("/" + robot_name + "/map_saving", 1, &ElevationMapping::mapSavingSignal, this);
  optKeyframeSub_ = nodeHandle_.subscribe("/" + robot_name + "/opt_keyframes", 1, &ElevationMapping::optKeyframeCallback, this);
  keyFrameSignalSub_ = nodeHandle_.subscribe("/" + robot_name + "/new_keyframe", 1, &ElevationMapping::newKeyframeSignal, this);
}


/*
 * Map composing thread
 */
void ElevationMapping::composingGlobalMapThread()
{
  ros::Rate rate(5.0);
  while(ros::ok()){
    rate.sleep();
    composingGlobalMap();
  }
  ROS_ERROR("ROS down !!!");
}


/*
 * Sync lidar and camera topics
 */
void ElevationMapping::startsync()
{
  connection = sync.registerCallback(boost::bind(&ElevationMapping::Callback, this, _1, _2));
}


/*
 * Parameters utility
 */
bool ElevationMapping::readParameters()
{
  ROS_INFO("Elevation mapping node parameters loading ... ");

  // ElevationMapping parameters.
  nodeHandle_.param("track_point_frame_id", trackPointFrameId_, string("/robot"));
  nodeHandle_.param("camera_params_yaml", cameraParamsFile, string("/home/mav-lab/intrinsic.yaml"));
  nodeHandle_.param("robot_local_map_size", localMapSize_, 20.0);
  nodeHandle_.param("travers_threshold", traversThre, 0.0);
  nodeHandle_.param("octomap_resolution", octoResolution_, 0.04);
  nodeHandle_.param("track_point_x", trackPoint_.x(), 0.0);
  nodeHandle_.param("track_point_y", trackPoint_.y(), 0.0);
  nodeHandle_.param("track_point_z", trackPoint_.z(), 0.0);

  nodeHandle_.param("robot_pose_cache_size", robotPoseCacheSize_, 200);
  ROS_ASSERT(robotPoseCacheSize_ >= 0);

  double minUpdateRate;
  nodeHandle_.param("min_update_rate", minUpdateRate, 2.0);
  maxNoUpdateDuration_.fromSec(1.0 / minUpdateRate);
  ROS_ASSERT(!maxNoUpdateDuration_.isZero());

  double timeTolerance;
  nodeHandle_.param("time_tolerance", timeTolerance, 0.0);
  timeTolerance_.fromSec(timeTolerance);
  
  // ElevationMap parameters. TODO Move this to the elevation map class.
  nodeHandle_.param("map_frame_id", map_frame_id, string("/map"));
  map_.setFrameId(map_frame_id);

  grid_map::Length length;
  grid_map::Position position;
  double resolution;
  nodeHandle_.param("length_in_x", length(0), 1.5);
  nodeHandle_.param("length_in_y", length(1), 1.5);
  nodeHandle_.param("position_x", position.x(), 0.0);
  nodeHandle_.param("position_y", position.y(), 0.0);
  nodeHandle_.param("resolution", resolution, 0.01);
  map_.setGeometry(length, resolution, position);
  
  nodeHandle_.param("map_saving_file", map_saving_file_, string("/home/mav-lab/slam_ws/test.pcd"));
  nodeHandle_.param("submap_saving_dir", submapDir, string("/home/mav-lab/slam_ws/"));

  nodeHandle_.param("min_variance", map_.minVariance_, pow(0.003, 2));
  nodeHandle_.param("max_variance", map_.maxVariance_, pow(0.03, 2));
  nodeHandle_.param("mahalanobis_distance_threshold", map_.mahalanobisDistanceThreshold_, 2.5);
  nodeHandle_.param("multi_height_noise", map_.multiHeightNoise_, pow(0.003, 2));
  nodeHandle_.param("min_horizontal_variance", map_.minHorizontalVariance_, pow(resolution / 2.0, 2)); // two-sigma
  nodeHandle_.param("max_horizontal_variance", map_.maxHorizontalVariance_, 0.5);
  nodeHandle_.param("underlying_map_topic", map_.underlyingMapTopic_, string());
  nodeHandle_.param("enable_visibility_cleanup", map_.enableVisibilityCleanup_, true);
  nodeHandle_.param("scanning_duration", map_.scanningDuration_, 1.0);

  float obstacle_threshold = traversThre;
  length_ = length(0) / resolution;
  resolution_ = resolution;

  // GPU initialization
  Init_GPU_elevationmap(length_, resolution_, map_.mahalanobisDistanceThreshold_, obstacle_threshold);
 
  // SensorProcessor parameters.
  string sensorType;
  nodeHandle_.param("sensor_processor/type", sensorType, string("structured_light"));
  if (sensorType == "structured_light") {
    sensorProcessor_.reset(new StructuredLightSensorProcessor(nodeHandle_, transformListener_));
  } else if (sensorType == "stereo") {
    sensorProcessor_.reset(new StereoSensorProcessor(nodeHandle_, transformListener_));
  } else if (sensorType == "laser") {
    sensorProcessor_.reset(new LaserSensorProcessor(nodeHandle_, transformListener_));
  } else if (sensorType == "perfect") {
    sensorProcessor_.reset(new PerfectSensorProcessor(nodeHandle_, transformListener_));
  } else {
    ROS_ERROR("The sensor type %s is not available.", sensorType.c_str());
  }

  if (!sensorProcessor_->readParameters()) return false;
  if (!robotMotionMapUpdater_.readParameters()) return false;

  return true;
}


/*
 * Some variable initailization
 */
bool ElevationMapping::initialize()
{
  ROS_INFO("Elevation mapping node initializing ... ");

  Duration(1.0).sleep(); // Need this to get the TF caches fill up.
  resetMapUpdateTimer();
  Record_txt.open("/home/mav-lab/DenseTimeAndMemUsage.txt");

  octoTree = new octomap::ColorOcTree(octoResolution_);
  globalOctoTree = new octomap::ColorOcTree(octoResolution_);  
  denseSubmap = false;
  newLocalMapFlag = 1;
  JumpOdomFlag = 0;
  JumpCount = 0;
  JumpFlag = 0;
  initFlag = 1;
  optFlag = 0;
  ROS_INFO("Done.");
  
  return true;
}


/*
 * Process raw point cloud
 */
void ElevationMapping::processpoints(pcl::PointCloud<Anypoint>::ConstPtr pointCloud)
{ 
  ros::Time begin_time = ros::Time::now ();

  // point cloud attributes
  int point_num = pointCloud->size();
  int point_index[point_num];
  float point_height[point_num];
  float point_var[point_num];
  
  int point_colorR[point_num];
  int point_colorG[point_num];
  int point_colorB[point_num];
  float point_intensity[point_num];

  PointCloud<Anypoint>::Ptr pointProcessed(new PointCloud<Anypoint>);

  if (!this->sensorProcessor_->process(pointCloud, pointProcessed, point_colorR, point_colorG, point_colorB, point_index, point_intensity, point_height, point_var)) {
    ROS_ERROR("Point cloud could not be processed.");
    this->resetMapUpdateTimer();
  }

  boost::recursive_mutex::scoped_lock lock(MapMutex_);

  // GPU functions for fusing multi-frame point cloud
  Fuse(length_, point_num, point_index, point_colorR, point_colorG, point_colorB, point_intensity, point_height, point_var);
  
  lock.unlock();
}


/*
 * Process map cells
 */
void ElevationMapping::processmapcells()
{
  ros::Time begin_time = ros::Time::now ();
  boost::recursive_mutex::scoped_lock lock(MapMutex_);

  if (!this->updatePrediction(this->lastPointCloudUpdateTime_)) {
    ROS_ERROR("Updating process noise failed.");
    this->resetMapUpdateTimer();
    return;
  }
  lock.unlock();   
}


/*
 * Main callback
 */
void ElevationMapping::Callback(const sensor_msgs::PointCloud2ConstPtr& rawPointCloud, const sensor_msgs::Image::ConstPtr& image)
{
  cout << "callback" << endl;
  ros::Time begin_time = ros::Time::now ();

  // conver image msg to cv::Mat
  sensor_msgs::PointCloud2 output = *rawPointCloud;
  pcl::PCLPointCloud2 Point_cloud;
  pcl_conversions::toPCL(output, Point_cloud);
  pcl::PointCloud<Anypoint>::Ptr pointCloud(new pcl::PointCloud<Anypoint>);
  pcl::fromPCLPointCloud2(Point_cloud, *pointCloud);
  cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(image, sensor_msgs::image_encodings::BGR8);
  cv::Mat img = cv_ptr -> image;
  
  // calculate extrinsic parameters
  /*float P_x, P_y;
  float width, height;
  Eigen::Vector4d P_lidar;
  Eigen::Vector3d P_img;
  Eigen::Vector3d P_XY;
  Eigen::MatrixXd P_lidar2img(3, 4);
  Eigen::MatrixXd Tcamera(3, 4);
  Eigen::MatrixXd TLidar(4, 4);
  cv::Mat TCAM, TV;

  // Read camera instrinsic and camera-lidar extrinsics parameters
  cv::FileStorage fsSettings(cameraParamsFile, cv::FileStorage::READ);
  if(!fsSettings.isOpened())
  {
    cerr << "ERROR: Wrong path to settings" << endl;
  }
*/
  //fsSettings["T.camera"] >> TCAM;
  //fsSettings["T.lidar"] >> TV;
  //Tcamera = toMatrix34(TCAM);
  //TLidar = toMatrix44(TV);

  // calculate transform camera to lidar
  //P_lidar2img = Tcamera * TLidar;

  ros::Time timeStamp;
  timeStamp.fromNSec(1000 * pointCloud->header.stamp);
  cout << "project" << endl;
  // Project image to point cloud
  for(int i = 0; i < pointCloud->points.size(); i++)
  {
    //P_lidar << pointCloud->points[i].x, 
    //          pointCloud->points[i].y,
    //          pointCloud->points[i].z,
    //          1;
    //P_img = P_lidar2img * P_lidar;
    
    //P_x = P_img.x() / P_img.z();  // NCLT need to flip x, y
    //P_y = P_img.y() / P_img.z();

    /*cv::Point midPoint;

    midPoint.x = P_x;
    midPoint.y = P_y;
    */
    // Project image to lidar point cloud
    if(0){ // NCLT need to flip height, width  
      /*int b = img.at<cv::Vec3b>(midPoint.y,midPoint.x)[0];
      int g = img.at<cv::Vec3b>(midPoint.y,midPoint.x)[1];
      int r = img.at<cv::Vec3b>(midPoint.y,midPoint.x)[2];
      cv::circle(img, midPoint, 1, cv::Scalar(b, g, r));
      pointCloud->points[i].b = b;
      pointCloud->points[i].g = g;
      pointCloud->points[i].r = r;*/
    }
    else{
      pointCloud->points[i].b = 0;
      pointCloud->points[i].g = 0;
      pointCloud->points[i].r = 0;
      pointCloud->points[i].intensity = 0;
    }
  }

  // pcl::StatisticalOutlierRemoval<Anypoint>sor;
  // sor.setInputCloud(pointCloud);
  // sor.setMeanK(30);
  // sor.setStddevMulThresh(1.0);
  // sor.filter(*pointCloud);

  lastPointCloudUpdateTime_.fromNSec(1000 * pointCloud->header.stamp);

  double t1 = ros::Time::now ().toSec();


  ROS_INFO("ElevationMap received a point cloud (%i points) for elevation mapping.", static_cast<int>(pointCloud->size()));
  sensorProcessor_->updateTransformations(timeStamp);
  updatepointsMapLocation(timeStamp);
  updateMapLocation();

  // thread for process
  std::thread processPointThread(&ElevationMapping::processpoints, this, pointCloud);
  std::thread processMapCellThread(&ElevationMapping::processmapcells, this);
  processPointThread.join();
  processMapCellThread.join();  

  // point cloud attributes 
  float elevation[length_ * length_];
  int point_colorR[length_ * length_];
  int point_colorG[length_ * length_];
  int point_colorB[length_ * length_];
  float rough[length_ * length_];
  float slope[length_ * length_];
  float traver[length_ * length_];
  float var[length_ * length_];
  float intensity[length_ * length_];

  // calculate point cloud attributes
  Map_feature(length_, elevation, var, point_colorR, point_colorG, point_colorB, rough, slope, traver, intensity);

  // get orthomosaic image
  orthoImage = map_.show(timeStamp, robot_name, trackPointTransformed_x, trackPointTransformed_y, length_, elevation, var, point_colorR, point_colorG, point_colorB, rough, slope, traver, intensity);

  // update local map and visual the local point cloud
  updateLocalMap(rawPointCloud);
  // visualOctomap();
  visualPointMap();

  // raytracing for obstacle removal

  prevMap_ = map_.visualMap_;
  // double MemUsage = calculate_memory_usage();

  Raytracing(length_);

  double t2 = ros::Time::now ().toSec();
  // ROS_WARN("Callback: %lf", t2-t1);
  // Record_txt << frame << "\t" << t2-t1 << "\t" << MemUsage << "\t" << "\n";
  // frame ++;
  // ROS_WARN("Callback: %lf", t2-t1);
}


double ElevationMapping::calculate_memory_usage()
{
  double usage_KB = 0;
  usage_KB += sizeof(decltype(globalMap_.back())) * globalMap_.capacity() / 1024.0;
  usage_KB += sizeof(Anypoint) * visualCloud_.size() / 1024.0;
  printf("the PRM consumes %f KB \n", usage_KB);
  return usage_KB;
}


/*
 * Signal triggered saving map utility function
 */
void ElevationMapping::savingMap()
{
  pcl::PointCloud<Anypoint> out_color;
  ROS_INFO("NUM: %d", visualCloud_.size());

  pcl::PointCloud<Anypoint>::Ptr cloud_in(new pcl::PointCloud<Anypoint>);
  pcl::PointCloud<Anypoint>::Ptr cloud_filtered(new pcl::PointCloud<Anypoint>);

  // saving global point cloud
  for(int i = 0; i < visualCloud_.size(); i++){
    Anypoint pt;
    pt.x = visualCloud_.points[i].x;
    pt.y = visualCloud_.points[i].y;
    pt.z = visualCloud_.points[i].z;
    pt.r = visualCloud_.points[i].r;
    pt.g = visualCloud_.points[i].g;
    pt.b = visualCloud_.points[i].b;
    pt.travers = visualCloud_.points[i].travers;
    pt.frontier = visualCloud_.points[i].frontier;
    pt.intensity = visualCloud_.points[i].intensity;
    pt.covariance = visualCloud_.points[i].covariance;
    out_color.push_back(pt);
  }

  ROS_INFO("Saving Map to %s", map_saving_file_);
  pcl::io::savePCDFile(map_saving_file_, out_color);
}


/*
 * Signal triggered saving submaps utility function
 */
void ElevationMapping::savingSubMap()
{
  ROS_WARN("SUB MAP NUM: %d", globalMap_.size());

  std::string currentName_;
  pointCloud cloudpt;

  for(int i = 0; i < globalMap_.size(); i++){
    cloudpt = globalMap_[i];
    std::ostringstream cc;
    cc << i;
    currentName_ = (submapDir + cc.str() + ".pcd").c_str();
    ROS_INFO("Saving Map to %s", currentName_);
    pcl::io::savePCDFile(currentName_, cloudpt);
  }
}


/*
 * Map composing for octomap visualization
 */
void ElevationMapping::composingGlobalMap()
{
  // ROS_INFO("composingGlobalMap");
  if(globalMap_.size() > 1){
    pointCloud cloudpt;

    for(int i = 0; i < globalMap_.size(); i++){
      cloudpt += globalMap_[i];
    }

    sensor_msgs::PointCloud2 output;
    pcl::toROSMsg(cloudpt, output);
    output.header.frame_id = map_frame_id;
    globalMapPublisher_.publish(output);

    octomap_msgs::Octomap octomsg;
    pointCloudtoOctomap(cloudpt, *globalOctoTree);
    octomap_msgs::fullMapToMsg(*globalOctoTree, octomsg);
    octomsg.header.frame_id = map_frame_id;
    octomsg.resolution = globalOctoTree->getResolution();  
    globalOctomapPublisher_.publish(octomsg);
    
  }
}


/*
 * Publish global map for visualization
 */
void ElevationMapping::visualPointMap()
{
  pointCloud::Ptr grid_pc;
  gridMaptoPointCloud(map_.visualMap_, grid_pc);

  if(!optFlag && (visualCloud_.size() > 0 || grid_pc->size() > 0)){
    sensor_msgs::PointCloud2 output;
    pcl::toROSMsg(visualCloud_+*grid_pc, output);
    output.header.frame_id = map_frame_id;
    pointMapPublisher_.publish(output);
  }else if(optFlag && visualCloud_.size() > 0){
    sensor_msgs::PointCloud2 output;
    pcl::toROSMsg(visualCloudOCC_+*grid_pc, output);
    output.header.frame_id = map_frame_id;
    pointMapPublisher_.publish(output);
  }
}


/*
 * Publish global map in octotree for visualization
 */
void ElevationMapping::visualOctomap()
{
  octomap_msgs::Octomap octomsg;
  if(visualCloud_.size() > 0){
    pointCloudtoOctomap(visualCloud_, *octoTree);

    octomap_msgs::fullMapToMsg(*octoTree, octomsg);
    octomsg.header.frame_id = map_frame_id;
    octomsg.resolution = octoTree->getResolution();  
    octomapPublisher_.publish(octomsg);
  }
}


/*
 * Map saving callback
 */
void ElevationMapping::mapSavingSignal(const std_msgs::Bool::ConstPtr& savingSignal)
{
  if(savingSignal->data == 1)
    savingMap();
    // savingSubMap();
  ROS_WARN("Saving Global Map at: %s.", map_saving_file_.c_str());
}


/*
 * Dense submap build signal callback
 */
void ElevationMapping::denseMappingSignal(const std_msgs::Bool::ConstPtr& denseSignal)
{
  if(denseSignal->data == 1)
    denseSubmap = true;
  ROS_WARN("Start dense submap build.");
}


/*
 * Loop closing signal callback
 */
void ElevationMapping::optKeyframeCallback(const slam_msg::Keyframes::ConstPtr& optKeyFrame)
{
  optFlag = 1;
  visualCloudOCC_ = visualCloud_;
  JumpOdomFlag = 1;
  optGlobalMapLoc_.clear();

  // Save the opt transform matrix
  for(int i = 0; i < optKeyFrame->keyframes.size(); i++){
    int optId = optKeyFrame->keyframes[i].id;
    Eigen::Quaternionf q(optKeyFrame->keyframes[i].rotation[3], optKeyFrame->keyframes[i].rotation[0], optKeyFrame->keyframes[i].rotation[1], optKeyFrame->keyframes[i].rotation[2]);
    Eigen::Isometry3f T(q);
    T.pretranslate(Eigen::Vector3f(optKeyFrame->keyframes[i].position[0], optKeyFrame->keyframes[i].position[1], optKeyFrame->keyframes[i].position[2]));
    optGlobalMapLoc_.push_back(T);
  }
  optKeyframeNum = optGlobalMapLoc_.size();
  ROS_WARN("Start optimizing global map with submap num %d", optKeyframeNum);
}


/*
 * New keyframe signal callback
 */
void ElevationMapping::newKeyframeSignal(const std_msgs::Bool::ConstPtr& newKeyframeSignal)
{
  if(newKeyframeSignal->data){
  	newLocalMapFlag = 1;
  	ROS_WARN("NEW KEYFRAME!");
  }
}


/*
 * Service for retrieving incremental submap
 */
bool ElevationMapping::mapService(dislam_msgs::GetInitMap::Request &req, dislam_msgs::GetInitMap::Response &res)
{
  ROS_WARN("in map service");
  // Save local hash to point cloud
  pointCloud::Ptr out_pc, grid_pc;

  // Save local map into global map stack
  unique_lock<mutex> lockLocal(LocalMapMutex_);
  gridMaptoPointCloud(map_.visualMap_, grid_pc);
  // localHashtoPointCloud(localMap_, out_pc);
  lockLocal.unlock();

  sensor_msgs::PointCloud2 output;
  pcl::toROSMsg(*grid_pc, output);
  output.header.frame_id = map_frame_id;
  
  res.submap = output;
  ROS_WARN("out map service");

  return true;
}


/*
 * Update local map and if the local map is full add it to global stack
 */
void ElevationMapping::updateLocalMap(const sensor_msgs::PointCloud2ConstPtr& rawPointCloud)
{
  int index, index_x, index_y;
  float delta_x, delta_y;

  float current_x = current_position[0];
  float current_y = current_position[1];

  delta_x = position_shift[0];
  delta_y = position_shift[1];

  // check if a new local map is needed
  // if(initFlag == 0 && sqrt(pow((trackPoseTransformed_.getOrigin().x() - trajectory_.back().translation().x()),2) + pow((trackPoseTransformed_.getOrigin().y() - trajectory_.back().translation().y()),2)) >= localMapSize_){
  //   newLocalMapFlag = 1;
  //   keyFramePCPublisher_.publish(rawPointCloud);
  //   ROS_WARN("NEW KEYFRAME ****************");
  // }

  // main if for generate local map
  if(newLocalMapFlag == 1 && (JumpFlag == 1 || optFlag == 0)){// && JumpOdomFlag == 0){ // At the local_map's boundary
    if(!localMap_.empty() && initFlag == 0){ // Not the init state

      // Get keyframe pose
      Eigen::Quaternionf q(trackPoseTransformed_.getRotation().w(), trackPoseTransformed_.getRotation().x(), trackPoseTransformed_.getRotation().y(), trackPoseTransformed_.getRotation().z());
      Eigen::Isometry3f prev_center(q);
      prev_center.pretranslate(Eigen::Vector3f(trackPoseTransformed_.getOrigin().x(), trackPoseTransformed_.getOrigin().y(), trackPoseTransformed_.getOrigin().z()));
      trajectory_.push_back(prev_center);

      // Save the center of every keyframe
      PointXY localMapCenter_;
      localMapCenter_.x = trajectory_.back().translation().x();
      localMapCenter_.y = trajectory_.back().translation().y();
      localMapLoc_.push_back(localMapCenter_);

      ROS_WARN("newLocalMapFlag %d, Push new keyframe! x = %lf, y = %lf", newLocalMapFlag, trajectory_.back().translation().x(), trajectory_.back().translation().y());

      // Save local hash to point cloud
      pointCloud::Ptr out_pc, grid_pc;
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr dense_output;

      // Save local map into global map stac
      unique_lock<mutex> lockLocal(LocalMapMutex_);
      gridMaptoPointCloud(map_.visualMap_, grid_pc);
      localHashtoPointCloud(localMap_, out_pc);
      lockLocal.unlock();

      ROS_WARN("get local map");
      if(denseSubmap)
        pointcloudinterpolation(out_pc);

      denseSubmap = false;
      unique_lock<mutex> lock(GlobalMapMutex_);
      globalMap_.push_back(*out_pc);
      lock.unlock();
      ROS_WARN("push_back global map");

      // Publish submap
      dislam_msgs::SubMap output;
      sensor_msgs::PointCloud2 pc;
      pcl::toROSMsg(*grid_pc+*out_pc, pc);
      pc.header.frame_id = map_frame_id;
      pc.header.stamp = ros::Time(0);
      output.submap = pc;
      // output.orthoImage = *orthoImage;
      output.keyframePC = *rawPointCloud;
      output.pose.position.x = trackPoseTransformed_.getOrigin().x();
      output.pose.position.y = trackPoseTransformed_.getOrigin().y();
      output.pose.position.z = trackPoseTransformed_.getOrigin().z();
      output.pose.orientation.x = trackPoseTransformed_.getRotation().x();
      output.pose.orientation.y = trackPoseTransformed_.getRotation().y();
      output.pose.orientation.z = trackPoseTransformed_.getRotation().z();
      output.pose.orientation.w = trackPoseTransformed_.getRotation().w();

      subMapPublisher_.publish(output);
      ROS_WARN("publish submap");

      // Quick clear unordered_map
      umap tmp;
      localMap_.swap(tmp);
      newLocalMapFlag = 0;

    }else if(initFlag == 1){  // Init process
      Eigen::Isometry3f prev_center = Eigen::Isometry3f::Identity();  // Initial map center
      prev_center.pretranslate(Eigen::Vector3f(0.0, 0.0, 0.0));
      trajectory_.push_back(prev_center);

      PointXY localMapCenter_;
      localMapCenter_.x = trajectory_.back().translation().x();
      localMapCenter_.y = trajectory_.back().translation().y();
      localMapLoc_.push_back(localMapCenter_);
      umap tmp;
      localMap_.swap(tmp);
      
      optFlag = 0;
      initFlag = 0;
      JumpOdomFlag = 0;
      JumpFlag = 0;
      
      newLocalMapFlag = 0;
      ROS_WARN("Init......");
      return;
    }
    
  }

  ROS_INFO("Move_x: %lf, Move_y: %lf ",delta_x, delta_y);
  int count = 0;

  // Local mapping
  if(abs(delta_x) >= resolution_ || abs(delta_y) >= resolution_ && initFlag == 0){// && (JumpOdomFlag == 0 || newLocalMapFlag == 0)){
    for (GridMapIterator iterator(prevMap_); !iterator.isPastEnd(); ++iterator) {   // Add L shape infomation of the previous grid map into the local map

      grid_map::Position position;
      prevMap_.getPosition(*iterator, position);
      index_x = (*iterator).transpose().x();
      index_y = (*iterator).transpose().y();
      index = index_x * length_ + index_y;

      if(prevMap_.at("elevation", *iterator) != -10){
       if(((position.x() < (current_x - length_ * resolution_ / 2) || position.y() < (current_y - length_ * resolution_ / 2 )) && (delta_x > 0  && delta_y > 0))
           || ((position.x() > (current_x + length_ * resolution_ / 2 ) || position.y() > (current_y + length_ * resolution_ / 2 )) && (delta_x < 0 && delta_y < 0))
           || ((position.x() < (current_x - length_ * resolution_ / 2 ) || position.y() > (current_y + length_ * resolution_ / 2 )) && (delta_x > 0 && delta_y < 0))
           || ((position.x() > (current_x + length_ * resolution_ / 2 ) || position.y() < (current_y - length_ * resolution_ / 2 )) && (delta_x < 0 && delta_y > 0))
           || ((position.x() < (current_x - length_ * resolution_ / 2 )) && (delta_x > 0 && delta_y == 0))
           || ((position.x() > (current_x + length_ * resolution_ / 2 )) && (delta_x < 0 && delta_y == 0))
           || ((position.y() < (current_y - length_ * resolution_ / 2 )) && (delta_y > 0 && delta_x == 0))
           || ((position.y() > (current_y + length_ * resolution_ / 2 )) && (delta_y < 0 && delta_x == 0))){ 
            
            GridPoint save_pos(position.x(), position.y());
            GridPointData save_data(prevMap_.at("elevation", *iterator), prevMap_.at("variance", *iterator), prevMap_.at("color_r", *iterator), 
                                    prevMap_.at("color_g", *iterator), prevMap_.at("color_b", *iterator), prevMap_.at("frontier", *iterator), 
                                    prevMap_.at("intensity", *iterator), prevMap_.at("traver", *iterator));
            
            // Update the grid information within local map
            auto got = localMap_.find(save_pos);
            if(got == localMap_.end()){
              localMap_.insert(make_pair(save_pos, save_data));
            }else{
              localMap_.erase(got);
              localMap_.insert(make_pair(save_pos, save_data));
              count++;
            }

            // Add information to real-time visualization
            Anypoint pt;
            pt.x = position.x();
            pt.y = position.y();
            pt.z = prevMap_.at("elevation", *iterator);
            pt.r = prevMap_.at("color_r", *iterator);
            pt.g = prevMap_.at("color_g", *iterator);
            pt.b = prevMap_.at("color_b", *iterator);
            pt.travers = prevMap_.at("traver", *iterator);
            pt.intensity = prevMap_.at("intensity", *iterator);
            pt.covariance = prevMap_.at("variance", *iterator);
            pt.frontier = prevMap_.at("frontier", *iterator);
            visualCloud_.push_back(pt);
        }
      }
    }
    // ROS_WARN("eliminating same position point %d", count);
  }
  JumpFlag = 0;
}


/*
 * Update global map and if the loop is detected
 */
void ElevationMapping::updateGlobalMap()
{
  ros::Rate r(15);
  ROS_WARN("UPDATE GLOBAL MAP");
	while(ros::ok())
	{
		if(optFlag == 1){ // optimize global map with opt poses
			double t1 = ros::Time::now ().toSec();
      ROS_WARN("UPDATE GLOBAL MAP, %d", optKeyframeNum);
      ROS_WARN("GLOBAL MAP SIZE, %d", globalMap_.size());

      if(optKeyframeNum > globalMap_.size()){
        optKeyframeNum = globalMap_.size();
      }

      visualCloud_.clear();
      pointCloud cloudUpdated;

      for(int i = 0; i < optKeyframeNum; i++){
        Eigen::Isometry3f T = Eigen::Isometry3f::Identity();

        if(i > 0){
          T = optGlobalMapLoc_[i] * trajectory_[i].inverse();
          Eigen::Matrix4f transformMatrix = T.matrix();

          cout << "transformMatrix ***************" << endl << trajectory_[i].matrix() << endl;
          cout << "optGlobalMapLoc_ ***************" << endl << optGlobalMapLoc_[i].matrix() << endl;
        
          pcl::transformPointCloud(globalMap_[i], cloudUpdated, transformMatrix);              
          
          unique_lock<mutex> lock(GlobalMapMutex_);  
          globalMap_[i] = cloudUpdated;

          lock.unlock();
          trajectory_[i] = optGlobalMapLoc_[i];
        }
      }
      ROS_WARN("transform cloud");


      for(int i = 0; i < optKeyframeNum; i++){
        // Fuse occlude points with opt global map
        PointXY localMapCenter_;
        localMapCenter_.x = localMapLoc_[i].x;
        localMapCenter_.y = localMapLoc_[i].y;
        PointCloud<PointXY>::Ptr cloud(new PointCloud<PointXY>());
        int index = 0;

        for(int j = 0; j < optKeyframeNum; j++){  // Get the center points of all local map
          PointXY pt;
          pt.x = localMapLoc_[j].x;
          pt.y = localMapLoc_[j].y;
          cloud->push_back(pt);
        }

        KdTreeFLANN<PointXY> kdtree;
        vector<int> pointIdxRadiusSearch;
        vector<float> pointRadiusSquaredDistance;
        PointXY searchPoint;
        kdtree.setInputCloud(cloud);
        vector<vector<int>> pointIdx;
        vector<vector<float>> pointRadiusDistance;
        float radius = 15.0;   // TODO: Param
        searchPoint = localMapCenter_;

        if(kdtree.radiusSearch(searchPoint, radius, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0){ // Find the adjacent local map using kdtree with their center points
          pointIdx.push_back(pointIdxRadiusSearch);
        }

        int count = 0;
        umap out_new, out_old;

        if(pointIdx[index].size() > 2){
          
          for(auto j = 1; j < pointIdx[index].size(); j++){  
            // Convert pointcloud to hash for fusing
            out_new.clear();
            out_old.clear();
            // ROS_WARN(" global: %d",globalMap_.size());
            pointCloudtoHash(globalMap_[pointIdx[index][j]], out_new);
            // ROS_WARN(" count: %d",pointIdx[index][j]);
            pointCloudtoHash(globalMap_[i], out_old);

            for(umap::iterator iter = out_new.begin(); iter != out_new.end();){
              // Find Point with Near Position
              GridPoint tmp(iter->first.x, iter->first.y);
              GridPointData tmp_data(iter->second.elevation, iter->second.var, iter->second.r, iter->second.g, iter->second.b, iter->second.frontier, iter->second.intensity, iter->second.travers);
              umap::const_iterator got = out_old.find(tmp);

              if(got != out_old.end() && got->second.var > 0 && got->second.var < 1){
                // Update Point Elevation and Variance
                count++;
                tmp_data.elevation = pow(iter->second.var, 2) * got->second.elevation + pow(got->second.var, 2) * iter->second.elevation / pow(got->second.var, 2) + pow(iter->second.var, 2);
                tmp_data.var = pow(got->second.var, 2) * pow(iter->second.var, 2) / pow(got->second.var, 2) + pow(iter->second.var, 2);
                tmp_data.frontier = (got->second.frontier && iter->second.frontier);

                // Delete the same point in two hash, and add fused point to both.
                out_old.erase(tmp);
                iter = out_new.erase(iter);
                out_new.insert(make_pair(tmp, tmp_data));
                out_old.insert(make_pair(tmp, tmp_data));
                // if(iter->second.var < got->second.var){
                //   out_new.insert(make_pair(tmp, tmp_data));
                // }else{
                //   out_old.insert(make_pair(tmp, tmp_data));
                // }
              }
              else{
                iter++;
              }
            }

            // Put the optimized local map into the visual stack
            unique_lock<mutex> lock(GlobalMapMutex_);
            pointCloud::Ptr tmp, tmp_now;
            localHashtoPointCloud(out_new, tmp); 
            globalMap_[pointIdx[index][j]] = *tmp;
            localHashtoPointCloud(out_old, tmp_now); 
            globalMap_[i] = *tmp_now;
            lock.unlock();
          }

          index++;
        }

        ROS_WARN("OPTIMIZE count: %d",count);

      }

      // Visual step
      unique_lock<mutex> lock(GlobalMapMutex_);
      for(int i = 0; i < globalMap_.size(); i++){
        visualCloud_ += globalMap_[i];
      }

      double t2 = ros::Time::now ().toSec();
      ROS_WARN("UPDATE GLOBAL MAP: %lf", t2-t1);
      optFlag = 0;
	  }
	  r.sleep();
	}
}


/*
 * Update prediction
 */
bool ElevationMapping::updatePrediction(const ros::Time& time)
{
  if (ignoreRobotMotionUpdates_) return true;

  ROS_DEBUG("Updating map with latest prediction from time %f.", robotPoseCache_.getLatestTime().toSec());
 
  if (time + timeTolerance_ < map_.getTimeOfLastUpdate()) {
   // map_.ElevationMap::getRawGridMap().setTimestamp(0);
  }
  if (time + timeTolerance_ < map_.getTimeOfLastUpdate()) {
    ROS_ERROR("Requested update with time stamp %f, but time of last update was %f.", time.toSec(), map_.getTimeOfLastUpdate().toSec());
    return false;
  } else if (time < map_.getTimeOfLastUpdate()) {
    ROS_DEBUG("Requested update with time stamp %f, but time of last update was %f. Ignoring update.", time.toSec(), map_.getTimeOfLastUpdate().toSec());
    return true;
  }

  HomTransformQuatD robotPose;
  geometry_msgs::Pose Tf_robotpose;

  Tf_robotpose.position.x = sensorProcessor_->M2StransformTf.getOrigin().x();
	Tf_robotpose.position.y = sensorProcessor_->M2StransformTf.getOrigin().y();
	Tf_robotpose.position.z = sensorProcessor_->M2StransformTf.getOrigin().z();
	Tf_robotpose.orientation.x = sensorProcessor_->M2StransformTf.getRotation().getX();
  Tf_robotpose.orientation.y = sensorProcessor_->M2StransformTf.getRotation().getY();
  Tf_robotpose.orientation.z = sensorProcessor_->M2StransformTf.getRotation().getZ();
  Tf_robotpose.orientation.w = sensorProcessor_->M2StransformTf.getRotation().getW();
	// std::cout << "Tf_robotpose " << Tf_robotpose.position.x << "  "<< Tf_robotpose.position.y << std::endl; 
  convertFromRosGeometryMsg(Tf_robotpose, robotPose);

  if(Tf_robotpose.position.x == 0 && Tf_robotpose.position.y == 0)
    return true;
  // Covariance is stored in row-major in ROS: http://docs.ros.org/api/geometry_msgs/html/msg/PoseWithCovariance.html
  Eigen::Matrix<double, 6, 6> robotPoseCovariance;
  robotPoseCovariance.setZero();
                        
  // Compute map variance update from motio n prediction.
  robotMotionMapUpdater_.update(map_, robotPose, robotPoseCovariance, time);
   
  return true;
}


/*
 * Update point location
 */
bool ElevationMapping::updatepointsMapLocation(const ros::Time& timeStamp)
{
  ROS_DEBUG("Elevation map is checked for relocalization.");

  geometry_msgs::PointStamped trackPoint;
  trackPoint.header.frame_id = trackPointFrameId_;
  trackPoint.header.stamp = timeStamp;
  convertToRosGeometryMsg(trackPoint_, trackPoint.point);
  geometry_msgs::PointStamped trackPointTransformed;

  try {
    transformListener_.transformPoint(map_.getFrameId(), trackPoint, trackPointTransformed);
  } catch (TransformException &ex) {
    ROS_ERROR("%s", ex.what());
    return false;
  }

  // get tf
  try{
    transformListener_.lookupTransform(map_frame_id, trackPointFrameId_, timeStamp, trackPoseTransformed_);
  }catch (tf::TransformException &ex) {
    ROS_ERROR("%s",ex.what());
    ros::Duration(1.0).sleep();
  }

  trackPointTransformed_x = trackPointTransformed.point.x;
  trackPointTransformed_y = trackPointTransformed.point.y;
  trackPointTransformed_z = trackPointTransformed.point.z;

  // ROS_WARN("JumpOdomFlag %d, trackPointTransformed_z %lf, later_trackPointTransformed_z %lf", JumpOdomFlag, trackPointTransformed_z, later_trackPointTransformed_z);

  // JumpOdom is not sync to the tf in front-end odometry
  if(JumpOdomFlag == 1 && abs(trackPointTransformed_z - later_trackPointTransformed_z) <= 0.02){
    JumpCount ++;
  }else if(JumpCount >= 3){
    JumpOdomFlag = 0;
    JumpFlag = 1;
    JumpCount = 0;
  }
  return true;
}


/*
 * Update grid map location
 */
bool ElevationMapping::updateMapLocation()
{
  float current_p[3];
  current_p[0] = trackPointTransformed_x;
  current_p[1] = trackPointTransformed_y;
  current_p[2] = trackPointTransformed_z;
  grid_map::Index M_startindex;
  grid_map::Position M_position;
  
  // handle odom jump phenomenone when optimization is done
  if(JumpOdomFlag == 1){    

    float opt_position[2];
    float opt_alignedPosition[2];
    float height_update = trackPointTransformed_z - later_trackPointTransformed_z;

    opt_position[0] = trackPointTransformed_x;
    opt_position[1] = trackPointTransformed_y;

    Map_optmove(opt_position, height_update, resolution_,  length_, opt_alignedPosition);

    M_position.x() = opt_alignedPosition[0];
    M_position.y() = opt_alignedPosition[1];
    // map_.opt_move(M_position, height_update);
    prevMap_ = map_.visualMap_;

  }else{

    int d_startindex[2];

    // move the grid map to this location
    Move(current_p , resolution_,  length_, current_position, d_startindex, position_shift);

    M_startindex.x() = d_startindex[0];
    M_startindex.y() = d_startindex[1];
    M_position.x() = current_position[0];
    M_position.y() = current_position[1];

    map_.move(M_startindex, M_position);
  }

  later_trackPointTransformed_z = trackPointTransformed_z;
  return true;
}


/*
 * Reset timer
 */
void ElevationMapping::resetMapUpdateTimer()
{
  mapUpdateTimer_.stop();
  Duration periodSinceLastUpdate = ros::Time::now() - map_.getTimeOfLastUpdate();
  if (periodSinceLastUpdate > maxNoUpdateDuration_) periodSinceLastUpdate.fromSec(0.0);
  mapUpdateTimer_.setPeriod(maxNoUpdateDuration_ - periodSinceLastUpdate);
  mapUpdateTimer_.start();
}


/*
 * Stop timer
 */
void ElevationMapping::stopMapUpdateTimer()
{
  mapUpdateTimer_.stop();
}


/*
 * Utility function: interpolation
 */
void pointcloudinterpolation(pointCloud::Ptr &input)
{
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr inputcloudPtr(new pcl::PointCloud<pcl::PointXYZRGB>());
  pcl::PointCloud<pcl::PointXYZRGB> inputcloud;
  
  for(auto it = input->begin(); it != input->end(); it++){
    pcl::PointXYZRGB pt;
    pt.x = it->x;
    pt.y = it->y;
    pt.z = it->z;
    pt.r = it->r;
    pt.g = it->g;
    pt.b = it->b;
    inputcloud.push_back(pt);
  }
  *inputcloudPtr = inputcloud;

	pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr dense_points (new pcl::PointCloud<pcl::PointXYZRGB> ());
  pointCloud::Ptr dense_points_converted (new pointCloud ());
	pcl::MovingLeastSquares<pcl::PointXYZRGB, pcl::PointXYZRGB> mls;

  double search_radius = 0.5; //0.03
  double sampling_radius = 0.2; //0.005
  double step_size = 0.1; //0.005
  double gauss_param = (double)std::pow(search_radius,2);
  int pol_order = 5;

	mls.setComputeNormals(true); 
	mls.setInputCloud(inputcloudPtr);
	mls.setPolynomialFit(true);
	mls.setSearchMethod(tree);
  mls.setSearchRadius(search_radius);
  mls.setUpsamplingMethod(pcl::MovingLeastSquares<pcl::PointXYZRGB, pcl::PointXYZRGB>::RANDOM_UNIFORM_DENSITY);
  mls.setUpsamplingRadius(sampling_radius);
  mls.setUpsamplingStepSize(step_size);
  mls.setPolynomialOrder(pol_order);
  mls.setSqrGaussParam(gauss_param);// (the square of the search radius works best in general)
  //mls.setDilationVoxelSize();//Used only in the VOXEL_GRID_DILATION upsampling method 
  mls.setPointDensity(1000); //15
	mls.process(*dense_points);

  pcl::copyPointCloud(*dense_points, *dense_points_converted);
  *input += *dense_points_converted;

	pcl::io::savePCDFile("/home/mav-lab/dense_cloud.pcd", *input);
}


/*
 * Utility function: convert local hash map to point cloud
 */
void ElevationMapping::localHashtoPointCloud(umap localMap, pointCloud::Ptr& outCloud)
{
  pointCloud hashPointCloud;
  for(auto it = localMap.begin(); it != localMap.end(); it++){
    if(it->second.elevation != -10){
      Anypoint pt;
      pt.x = it->first.x;
      pt.y = it->first.y;
      pt.z = it->second.elevation;
      pt.r = it->second.r;
      pt.g = it->second.g;
      pt.b = it->second.b;
      pt.intensity = it->second.intensity;
      pt.covariance = it->second.var;
      pt.travers = it->second.travers;
      pt.frontier = it->second.frontier;
      hashPointCloud.push_back(pt);
    }
  }
  outCloud = hashPointCloud.makeShared();
}


/*
 * Utility function: convert colored point cloud to Octomap
 */
void ElevationMapping::pointCloudtoOctomap(pointCloud localPointCloud, octomap::ColorOcTree& tree)
{
  for(auto p:localPointCloud.points) {
    // insert point into tree
    tree.updateNode(octomap::point3d(p.x, p.y, p.z), true);
  }
  for(auto p:localPointCloud.points) {
    // integrate color point into tree
    tree.integrateNodeColor( p.x, p.y, p.z, p.r, p.g, p.b );
  }
  tree.updateInnerOccupancy();
}


/*
 * Utility function: convert point cloud to local hash map
 */
void ElevationMapping::pointCloudtoHash(pointCloud localPointCloud, umap& out)
{
  for(size_t i = 0; i < localPointCloud.size (); ++i){
    float round_x, round_y, round_z;
    round_x = (ceil(localPointCloud.points[i].x / resolution_)) * resolution_ - resolution_ / 2.0;
    round_y = (ceil(localPointCloud.points[i].y / resolution_)) * resolution_ - resolution_ / 2.0;
    round_z = localPointCloud.points[i].z;

    GridPoint save_pos(round_x, round_y);
    GridPointData save_data(round_z, localPointCloud.points[i].covariance, localPointCloud.points[i].r, localPointCloud.points[i].g, localPointCloud.points[i].b, 
                            localPointCloud.points[i].frontier, localPointCloud.points[i].intensity, localPointCloud.points[i].travers);
    out.insert(make_pair(save_pos, save_data));
  }
}


/*
 * Utility function: convert grid map to point cloud
 */
void ElevationMapping::gridMaptoPointCloud(grid_map::GridMap& gridmap, pointCloud::Ptr& pc)
{
  Anypoint point;
  pointCloud out_pc;

  int index, index_x, index_y;
  grid_map::Index startIndex = gridmap.getStartIndex();

  for (GridMapIterator iterator(gridmap); !iterator.isPastEnd(); ++iterator) {
    index_x = (*iterator).transpose().x();
    index_y = (*iterator).transpose().y();
    index = index_x * length_ + index_y;
    if(gridmap.at("elevation", *iterator) != -10 && gridmap.at("traver", *iterator) != -10 || !std::isnan(gridmap.at("traver", *iterator)) || !std::isnan(gridmap.at("elevation", *iterator)))
    {
      grid_map::Position gridPosition;
      gridmap.getPosition(*iterator, gridPosition);

      point.x = gridPosition.x();
      point.y = gridPosition.y();
      point.z = gridmap.at("elevation", *iterator);
      point.r = gridmap.at("color_r", *iterator);
      point.g = gridmap.at("color_g", *iterator);
      point.b = gridmap.at("color_b", *iterator);
      point.intensity = gridmap.at("intensity", *iterator);
      point.covariance =  gridmap.at("variance", *iterator);
      point.travers = gridmap.at("traver", *iterator);     
      point.frontier = gridmap.at("frontier", *iterator);
      out_pc.push_back(point);
    }else{
      grid_map::Position gridPosition;
      gridmap.getPosition(*iterator, gridPosition);

      point.x = gridPosition.x();
      point.y = gridPosition.y();
      point.z = 10.0;
      point.r = 0.0;
      point.g = 0.0;
      point.b = 0.0;
      point.intensity = 0.0;
      point.covariance = 0.0;
      point.travers = 1.0;     
      point.frontier = 0;
      out_pc.push_back(point);
    }
  }
  pc = out_pc.makeShared();
}


/*
 * Utility functions: convert cv::Mat to Eigen::Matrix44
 */
Eigen::Matrix<double,4,4> toMatrix44(const cv::Mat &cvMat)
{
    Eigen::Matrix<double,4,4> M;

    M << cvMat.at<double>(0,0), cvMat.at<double>(0,1), cvMat.at<double>(0,2), cvMat.at<double>(0,3),
    cvMat.at<double>(1,0), cvMat.at<double>(1,1), cvMat.at<double>(1,2), cvMat.at<double>(1,3),
    cvMat.at<double>(2,0), cvMat.at<double>(2,1), cvMat.at<double>(2,2), cvMat.at<double>(2,3),
    cvMat.at<double>(3,0), cvMat.at<double>(3,1), cvMat.at<double>(3,2), cvMat.at<double>(3,3);

    return M;
}


/*
 * Utility functions: convert cv::Mat to Eigen::Matrix34
 */
Eigen::Matrix<double,3,4> toMatrix34(const cv::Mat &cvMat)
{
    Eigen::Matrix<double,3,4> M;

    M << cvMat.at<double>(0,0), cvMat.at<double>(0,1), cvMat.at<double>(0,2), cvMat.at<double>(0,3),
    cvMat.at<double>(1,0), cvMat.at<double>(1,1), cvMat.at<double>(1,2), cvMat.at<double>(1,3),
    cvMat.at<double>(2,0), cvMat.at<double>(2,1), cvMat.at<double>(2,2), cvMat.at<double>(2,3);

    return M;
}

} /* namespace */
