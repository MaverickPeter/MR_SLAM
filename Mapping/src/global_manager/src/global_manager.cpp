#include <global_manager/global_manager.h>

#include <ros/assert.h>
#include <ros/console.h>
#include <tf2_eigen/tf2_eigen.h>
#include <fftw3.h>

using namespace gtsam;

namespace global_manager
{
GlobalManager::GlobalManager(ros::NodeHandle private_nh) : nrRobots(0), node_(private_nh)
{
  std::string merged_elevation_map_topic, merged_pointcloud_topic, pose_graph_topic, opt_state_topic;

  // Get parameters from launch file
  private_nh.param("publish_tf", publish_tf, true);
  private_nh.param("loop_detection_debug", debug, true);
  private_nh.param("loop_closure_enable", loopClosureEnable_, true);
  private_nh.param("odometry_loop_enable", odometryLoopEnable_, true);
  private_nh.param("use_other_descriptor", useOtherDescriptor_, false);
  private_nh.param("manual_robots_config", manual_robots_config_, false);

  private_nh.param("disco_dim", disco_dim_, 1.0);
  private_nh.param("icp_iters", icp_iters_, 10.0);
  private_nh.param("disco_width", disco_width_, 120.0);
  private_nh.param("start_robot_id", start_robot_id_, 1.0);
  private_nh.param("disco_height", disco_height_, 40.0);
  private_nh.param("composing_rate", composing_rate_, 1.0);
  private_nh.param("tf_publish_rate", tf_publish_rate_, 1.0);
  private_nh.param("voxel_leaf_size", voxel_leaf_size_, 0.3);
  private_nh.param("loop_detection_rate", loop_detection_rate_, 10.0);
  private_nh.param("pose_graph_pub_rate", pose_graph_pub_rate_, 10.0);
  private_nh.param("keyframe_search_candidates", keyframe_search_candidates_, 20.0);
  private_nh.param("keyframe_search_radius", historyKeyframeSearchRadius, 10.0);

  private_nh.param<std::string>("robot_namespace", robot_namespace_, "");
  private_nh.param<std::string>("opt_state_topic", opt_state_topic, "opt");
  private_nh.param<std::string>("global_map_frame", global_map_frame_, "map");
  private_nh.param<std::string>("pose_graph_topic", pose_graph_topic, "graph");
  private_nh.param<std::string>("manual_config_dir", manual_config_dir_, "./");
  private_nh.param<std::string>("submap_topic", robot_submap_topic_, "submap");
  private_nh.param<std::string>("descriptor_topic", robot_disco_topic_, "disco");
  private_nh.param<std::string>("keyframe_pc_topic", keyframe_pc_topic_, "keyframe");
  private_nh.param<std::string>("merged_elevation_map_topic", merged_elevation_map_topic, "map");
  private_nh.param<std::string>("merged_point_cloud_topic", merged_pointcloud_topic, "map");
  private_nh.param<std::string>("pg_saving_filename", pg_saving_filename_, "./fullGraph.g2o");
  private_nh.param<std::string>("keyframe_saving_dir", keyframe_saving_dir_, "./Keyframes/");
  private_nh.param<std::string>("initmap_service_topic", initmap_service_topic_, "map_srv");
  private_nh.param<std::string>("elevation_map_saving_filename", elevation_map_saving_filename_, "./globalMap.pcd");
  private_nh.param<std::string>("global_map_saving_filename", global_map_saving_filename_, "./globalMap.pcd");

  private_nh.param("DiSCO_dist_thres", DISCO_DIST_THRES, 3.0);
  private_nh.param("icp_fitness_score", acceptedKeyframeFitnessScore, 0.9);

  NUM_CANDIDATES_FROM_TREE = (int)keyframe_search_candidates_;
  DISCO_DIM = (int)disco_dim_;

  /* Publishing */
  merged_elevation_map_publisher_ = node_.advertise<PointCloud>(merged_elevation_map_topic, 1);
  merged_pointcloud_publisher_ = node_.advertise<PointCloud>(merged_pointcloud_topic, 1);
  pose_graph_publisher_ = node_.advertise<visualization_msgs::MarkerArray>(pose_graph_topic, 1);
  optimizing_state_publisher_ = node_.advertise<std_msgs::Bool>(opt_state_topic, 1);

  map_saving_subscriber_ = private_nh.subscribe("/map_saving", 1, &GlobalManager::mapSaving, this);
  loop_info_subscriber_ = private_nh.subscribe("/loop_info", 10, &GlobalManager::processLoopClosure, this);

  // Get all config filenames
  if(manual_robots_config_){
    getFileNames(manual_config_dir_, configFiles);
    readConfigs(configFiles, initPoses, start_robot_id_);
  }

  // Initialize some variables
  loop_num = 0;
  disco_ind = 0;
  kdtree = kdtree_init(DISCO_DIM);
  kdtreeHistoryKeyPoses.reset(new pcl::KdTreeFLANN<PointTI>());
  kdtreeHistoryKeyPoses6D.reset(new pcl::KdTreeFLANN<PointTypePose>());

  initGraphNodeMarkers(trajMarker_, global_map_frame_);
  initGraphEdgeMarkers(loopEdgeMarker_, global_map_frame_);

  // Init edge type
  gtsam::Vector VectorPrior(6);
  VectorPrior << 1e-6, 1e-6, 1e-6, 1e-4, 1e-4, 1e-4;
  // VectorPrior << 1, 1, 1, 1, 1, 1;
  priorNoise = noiseModel::Diagonal::Variances(VectorPrior);

  gtsam::Vector VectorOdom(6);
  VectorOdom << 1, 1, 1, 1, 1, 1;
  odometryNoise = noiseModel::Diagonal::Variances(VectorOdom);
  cout << "Initialization Done! " << endl;
}


GlobalManager::~GlobalManager()
{

};


/*
 * Map saving signal capture
 */
void GlobalManager::mapSaving(const std_msgs::Bool::ConstPtr& savingSignal)
{
  if(savingSignal->data){
    // Get the graph
    vector<int> initial_size;
    initial_size = constructOptimizer(true);
    GraphAndValues fullGraphAndValues = readFullGraph();
    std::pair<Values, vector<int>> correctedPosePair = correctPoses(false);
    Values fullInitial = correctedPosePair.first;

    savingElevationMap();
    savingPoseGraph();
    savingKeyframes(fullInitial);
    savingGlobalMap(fullInitial);
  }
}


/*
 * Saving global map to output file
 */
void GlobalManager::savingGlobalMap(Values fullInitial)
{
  ROS_INFO("Saving Global Map to %s", global_map_saving_filename_.c_str());

  // Get all keyframe point clouds and merge into global map
  PointCloudI Keyframe;
  PointCloudI globalMap;
  std::unique_lock<std::mutex> lock(subscriptions_mutex_);
  for(auto& subscription: subscriptions_){
    std::lock_guard<std::mutex> lock2(subscription.mutex);
    for(int i = 0; i < subscription.keyframes.size(); i++){
      Key g2oid = robotID2Key(subscription.robot_id - start_robot_id_) + i + 1;
      if(fullInitial.exists(g2oid)){
        Eigen::Isometry3f T = Pose3toIsometry(fullInitial.at<Pose3>(g2oid));
        Eigen::Matrix4f transformMatrix = T.matrix();
        pcl::transformPointCloud(*subscription.keyframes[i], Keyframe, transformMatrix); 
        globalMap += Keyframe;
      }
    }
  }
  pcl::io::savePCDFile(global_map_saving_filename_, globalMap);
}


/*
 * Saving global elevation map to output file
 */
void GlobalManager::savingElevationMap()
{
  ROS_INFO("Saving Elevation Map to %s", elevation_map_saving_filename_.c_str());
  if(merged_map.size() > 0){
    pcl::io::savePCDFile(elevation_map_saving_filename_, merged_map);
  }
}


/*
 * Saving pose graph to output file
 */
void GlobalManager::savingPoseGraph()
{
  ROS_INFO("Saving Pose Graph to %s", pg_saving_filename_.c_str());
  
  // Get the graph
  vector<int> initial_size;
  initial_size = constructOptimizer(true);
  GraphAndValues fullGraphAndValues = readFullGraph();
  NonlinearFactorGraph fullGraph = *(fullGraphAndValues.first);
  Values fullInitial = *(fullGraphAndValues.second);
  
  // Write full graph
  writeG2o(fullGraph, fullInitial, pg_saving_filename_);
  cout << "fullInitial.size() = " << fullInitial.size() << endl;

  std::pair<Values, vector<int>> correctedPosePair = correctPoses(true);

  GraphAndValues fullGraphAndValuesOpt = readFullGraph();
  NonlinearFactorGraph fullGraphOpt = *(fullGraphAndValuesOpt.first);
  Values fullInitialOpt = *(fullGraphAndValuesOpt.second);
  
  // Write full graph
  writeG2o(fullGraphOpt, fullInitialOpt, "/home/client/graph/full_graph_optimized.g2o");
  cout << "optimized fullInitial.size() = " << fullInitialOpt.size() << endl;
}


/*
 * Saving keyframe point cloud to output file
 */
void GlobalManager::savingKeyframes(Values fullInitial)
{
  ROS_INFO("Saving Keyframe pointcloud to %s", keyframe_saving_dir_.c_str());

  // Check format
  if(keyframe_saving_dir_[keyframe_saving_dir_.length()-1] != '/')
    keyframe_saving_dir_ = keyframe_saving_dir_ + "/";

  int robotConsecutiveID = 0;
  // Get all keyframe point clouds
  std::unique_lock<std::mutex> lock(subscriptions_mutex_);
  for(auto& subscription: subscriptions_){
    std::lock_guard<std::mutex> lock2(subscription.mutex);

    for(int i = 0; i < subscription.keyframes.size(); i++){
      int robotid = subscription.robot_id - start_robot_id_;
      uint64_t g2oid = robotID2Key(robotid) + i + 1;
      
      if(fullInitial.exists(g2oid)){
        int newid = g2oid - robotID2Key(robotid) + robotConsecutiveID;

        stringstream ss;
        ss << setw(6) << setfill('0') << newid ;
        string newidstr;
        ss >> newidstr;

        string directory = keyframe_saving_dir_ + newidstr;
        if(!boost::filesystem::is_directory(directory)) {
          boost::filesystem::create_directory(directory);
        }

        // Get the graph
        Pose3 currentPose = fullInitial.at<Pose3>(g2oid);
        Eigen::Isometry3f currPose = Pose3toIsometry(currentPose);

        std::ofstream ofs(directory + "/data");
        // ofs << "stamp " << newid << "\n";

        ofs << "estimate\n";
        ofs << currPose.matrix() << "\n";

        ofs << "odom\n";
        ofs << currPose.matrix() << "\n";

        // ofs << "accum_distance " << 0.0 << "\n";
        ofs << "id " << newid  << "\n";

        string filename = directory + "/cloud.pcd";
        pcl::io::savePCDFileBinary(filename, *subscription.keyframes[i]);
      }
    }
    robotConsecutiveID += subscription.keyframes.size();
  }
  lock.unlock();
}


/*
 * Subscribe thread to get msg from front-end
 */
void GlobalManager::discoveryThread()
{
  discovery();
}


/*
 * Dynamic robots discovery
 */
void GlobalManager::discovery()
{
  ROS_DEBUG("Robot discovery started.");
  while(1){
    ros::master::V_TopicInfo topic_infos;
    std::string robot_name;
    std::string map_topic;
    std::string disco_topic;
    std::string keyframe_topic;
    std::string map_service_topic;

    ros::master::getTopics(topic_infos);

    for (const auto& topic : topic_infos) {
      // we check only map and disco topic
      if (!isRobotMapTopic(topic) && !isRobotDiSCOTopic(topic)) {
        continue;
      }

      robot_name = robotNameFromTopic(topic.name);

      // !!!NOTICE: robotIDStack save the incoming robot id list, subscription doesn't hold actual robot id.
      std::string robotID = std::regex_replace(robot_name, std::regex("[^0-9]*([0-9]+).*"), std::string("$1"));
      
      // Check if this robot is already found
      if (robots_.count(robot_name)) {
        // we already know this robot
        continue;
      }else{

        // saving incoming order
        robotIDStack.push_back(std::stoi(robotID));

        /*********************************
        *** Init some storage variables
        ***********************************/
        // initPoses
        if(!manual_robots_config_)
          initPoses.push_back(Eigen::Isometry3f::Identity());

        // tf
        std::vector<Eigen::Isometry3f, Eigen::aligned_allocator<Eigen::Isometry3f>> newRobotOriginMapTF;
        std::vector<Eigen::Isometry3f, Eigen::aligned_allocator<Eigen::Isometry3f>> newRobotOptMapTF;
        std::vector<geometry_msgs::Point> newTrajVec;
        Eigen::Isometry3f identity = Eigen::Isometry3f::Identity();
        
        // map
        std::vector<PointCloud> newRobotMapStack;        
        PointCloudPtr _submap_in(new PointCloud);
        pcl::PointXYZI initPoint;
        PointTypePose initPose;
        initPoint.x = initPose.x = 0.0;
        initPoint.y = initPose.y = 0.0;
        initPoint.z = initPose.z = 0.0;
        initPose.roll = 0.0;
        initPose.pitch = 0.0;
        initPose.yaw = 0.0;
        merged_map_size.push_back(0);

        // graph
        GraphPtr graph(new NonlinearFactorGraph);
        ValuesPtr initial(new Values);
        Rot3 R_init = Rot3::RzRyRx(0.0, 0.0, 0.0);
        Point3 t_init = Point3(0,0,0);
        uint64_t id0, id1;
        id0 = robotID2Key(nrRobots);
        id1 = robotID2Key(nrRobots) + 1;
        initial->insert(id0, Pose3(R_init, t_init));
        initial->insert(id1, Pose3(R_init, t_init));
        NonlinearFactor::shared_ptr factor(new BetweenFactor<Pose3>(id0, id1, Pose3(R_init, t_init), priorNoise));
        graph->push_back(factor);
    
        auto pair = make_pair(graph, initial);
        graphAndValuesVec.emplace_back(pair);

        // tf
        newRobotOriginMapTF.emplace_back(identity);
        newRobotOptMapTF.emplace_back(identity);
        originMapTF.emplace_back(newRobotOriginMapTF);
        optMapTF.emplace_back(newRobotOptMapTF);
        currentRobotPosPoint.emplace_back(initPoint);
        local_map_stack.emplace_back(*_submap_in);

        // map tf
        std::lock_guard<std::mutex> mapTFLock(map_tf_mutex);
        if(!manual_robots_config_){
          currentRef.emplace_back(Eigen::Isometry3f::Identity());
          mapTF.emplace_back(Eigen::Isometry3f::Identity());
        }else{
          currentRef.emplace_back(initPoses[nrRobots]);
          mapTF.emplace_back(initPoses[nrRobots]);
        }
        lastTF.emplace_back(Eigen::Isometry3f::Identity());

        // graph visualization
        geometry_msgs::Point point;
        point.x = initPoses[nrRobots](0,3);
        point.y = initPoses[nrRobots](1,3);
        point.z = initPoses[nrRobots](2,3);
        newTrajVec.emplace_back(point);
        trajPointsVec_.push_back(newTrajVec);
        loopEdgesVec_.push_back(newTrajVec);
        
        // map
        newRobotMapStack.emplace_back(*_submap_in);
        global_map_stack.emplace_back(newRobotMapStack);
      }

      ROS_INFO("adding robot [%s] to system", robot_name.c_str());
      {
        std::lock_guard<std::mutex> lock(subscriptions_mutex_);
        subscriptions_.emplace_front();
        robotNameVec.emplace_back(robot_name);
        ++nrRobots;
      }


      // no locking here. robots_ are used only in this procedure
      // !!!NOTICE: robotHandle is inserted from the front of the forward_list
      robotHandle_& subscription = subscriptions_.front();
      subscription.robot_name = robot_name;
      subscription.robot_id = std::stoi(robotID); // nrRobot - 1;
      subscription.initState = false;
      robots_.insert({robot_name, &subscription});

      subscription.cloudKeyPoses3D.reset(new pcl::PointCloud<pcl::PointXYZI>());
      subscription.cloudKeyPoses6D.reset(new pcl::PointCloud<PointTypePose>());

      /* subscribe map callbacks */
      map_topic = ros::names::append(robot_name, robot_submap_topic_);
      ROS_INFO("\033[1;31m Subscribing to MAP topic: %s \033[0m", map_topic.c_str());
      subscription.map_sub = node_.subscribe<dislam_msgs::SubMap>(
          map_topic, 1, [this, &subscription](const dislam_msgs::SubMapConstPtr& msg) {
            mapUpdate(msg, subscription);
      });

      /* subscribe disco callbacks */
      disco_topic = ros::names::append(robot_name, robot_disco_topic_);
      ROS_INFO("\033[1;33m Subscribing to DiSCO topic: %s \033[0m", disco_topic.c_str());
      subscription.disco_sub = node_.subscribe<dislam_msgs::DiSCO>(
          disco_topic, 1, [this, &subscription](const dislam_msgs::DiSCOConstPtr& msg) {
            discoUpdate(msg, subscription);
      });

      /* publish previous keyframe */
      keyframe_topic = ros::names::append(robot_name, keyframe_pc_topic_);
      subscription.keyframe_pub = node_.advertise<sensor_msgs::PointCloud2>(keyframe_topic, 1);

      /* register service */
      map_service_topic = ros::names::append(robot_name, initmap_service_topic_);
      subscription.init_map_client_ = node_.serviceClient<dislam_msgs::GetInitMap>(map_service_topic);
    }
  }

  ROS_DEBUG("Robot discovery finished.");
}


/*
 * !!!Deprecated. Interface Demo. Thread to get init map.
 */
void GlobalManager::initMapProcessThread()
{
  ros::Rate rate(10.0);
  while(ros::ok()){
    rate.sleep();

    auto start = system_clock::now();
    
    getInitMaps();
    
    auto end = system_clock::now();
    auto duration = duration_cast<microseconds>(end - start);
    // ROS_DEBUG("publishTF: %lfs", double(duration.count()) * microseconds::period::num / microseconds::period::den);
  }
  ROS_ERROR("ROS down !!!");
}


/*
 * !!!Deprecated. Interface Demo. Call service to get init map.
 */
void GlobalManager::getInitMaps()
{
  // lock and check every robots init state
  std::unique_lock<std::mutex> lock(subscriptions_mutex_);
  for(auto& subscription: subscriptions_){
    std::lock_guard<std::mutex> lock2(subscription.mutex);
    dislam_msgs::GetInitMap srv;
    if(subscription.initState){
      srv.request.initState = subscription.initState;
      if (subscription.init_map_client_.call(srv)){
        ROS_INFO("Get initMap");
        PointCloudPtr initSubmap(new PointCloud);
        pcl::fromROSMsg(srv.response.submap, *initSubmap);
        
      }
      else{
        ROS_ERROR("Failed to call service init map");
      }
    }
  }
  lock.unlock();

}


/*
 * Publish TF thread
 */
void GlobalManager::publishTFThread()
{
  if(publish_tf){
    ros::Rate rate(tf_publish_rate_);
    while(ros::ok()){
      rate.sleep();

      auto start = system_clock::now();
      
      publishTF();
      // publishPoseGraph();

      auto end = system_clock::now();
      auto duration = duration_cast<microseconds>(end - start);
      ROS_DEBUG("publishTF: %lfs", double(duration.count()) * microseconds::period::num / microseconds::period::den);
    }
    ROS_ERROR("ROS down !!!");
  }
}


/*
 * Publish pose graph thread
 */
void GlobalManager::publishPoseGraphThread()
{
  ros::Rate rate(pose_graph_pub_rate_);
	while(ros::ok()){
    rate.sleep();

    publishPoseGraph();
  }
  ROS_ERROR("ROS down !!!");
}


/*
 * Map composing thread
 */
void GlobalManager::mapComposingThread()
{
  ros::Rate rate(composing_rate_);
  while(ros::ok()){
    rate.sleep();

    auto start = system_clock::now();
    if(nrRobots != 0){
      mapComposing();
      publishMergedMap();
      publishMergedPointcloud();
    }
    auto end = system_clock::now();
    auto duration = duration_cast<microseconds>(end - start);
    ROS_DEBUG("mapComposingThread: %lfs", double(duration.count()) * microseconds::period::num / microseconds::period::den);
  }
  ROS_ERROR("ROS down !!!");
}


/*
 * Loop closing thread
 */
void GlobalManager::loopClosingThread()
{
  if(loopClosureEnable_ == false)
    return;
  
  ros::Rate rate(loop_detection_rate_);
  while(ros::ok()){
    rate.sleep();
    
    auto start = system_clock::now();
    
    if(!useOtherDescriptor_)
      performLoopClosure();
      
    auto end = system_clock::now();
    auto duration = duration_cast<microseconds>(end - start);
    ROS_DEBUG("performLoopClosure: %lfs", double(duration.count()) * microseconds::period::num / microseconds::period::den);
    
    auto update_start = system_clock::now();
    
    if(aLoopIsClosed){
      auto correct_start = system_clock::now();
      
      std::pair<Values, vector<int>> correctedPosePair = correctPoses(false);
      
      auto correct_end = system_clock::now();
      auto correct_duration = duration_cast<microseconds>(correct_end - correct_start);
      ROS_DEBUG("correctPoses: %lfs", double(correct_duration.count()) * microseconds::period::num / microseconds::period::den);
      
      updateTransform(correctedPosePair);
      mapNeedsToBeCorrected = true;
    }
    
    auto update_end = system_clock::now();
    auto update_duration = duration_cast<microseconds>(update_end - update_start);
    ROS_DEBUG("updateTransform: %lfs", double(update_duration.count()) * microseconds::period::num / microseconds::period::den);
  }
  ROS_ERROR("ROS down !!!");
}


/**
 * @brief Process loop closure info given by loop detection node.
 * @details add edges to the maintained graph using subscribed loop info
 */
void GlobalManager::processLoopClosure(const dislam_msgs::LoopsConstPtr& msg)
{
  ROS_INFO("\033[1;32m received loop update \033[0m");

  // Get all loop infos
  std::vector<dislam_msgs::Loop> interloops = msg->Loops;
  // string ddd = "/home/client/graph/00.g2o";
  // writeG2o(*graphAndValuesVec[0].first, *graphAndValuesVec[0].second, ddd);
  // string dd1 = "/home/client/graph/01.g2o";
  // writeG2o(*graphAndValuesVec[1].first, *graphAndValuesVec[1].second, dd1);
  // string dd2 = "/home/client/graph/02.g2o";
  // writeG2o(*graphAndValuesVec[2].first, *graphAndValuesVec[2].second, dd2);

  std::lock_guard<std::mutex> lock(graph_mutex_);
  // Iter all loop infos and get ready for pose graph insertion
  for(auto iter : interloops){
    uint64_t id1 = iter.id0;
    uint64_t id2 = iter.id1;
    if(Key2robotID(id1) == Key2robotID(id2))
      break;

    Eigen::Quaternionf factorPoseq(iter.pose.orientation.w, iter.pose.orientation.x, iter.pose.orientation.y, iter.pose.orientation.z);
    Eigen::Isometry3f factorPose(factorPoseq);
    factorPose.pretranslate(Eigen::Vector3f(iter.pose.position.x, iter.pose.position.y, iter.pose.position.z));
    Pose3 factorPose3 = Pose3(((Eigen::Isometry3d)factorPose).matrix());
    
    if(id1 < id2){
      NonlinearFactor::shared_ptr interRobotFactorID1(new BetweenFactor<Pose3>(id1, id2, factorPose3, odometryNoise));
      graphAndValuesVec[Key2robotID(id1)].first->push_back(interRobotFactorID1);
      graphAndValuesVec[Key2robotID(id2)].first->push_back(interRobotFactorID1);
      // Add to graph visualization
      cout << " add edge into current graph " << Key2robotID(id2) << endl;
      std::pair<uint64_t, uint64_t> edge = make_pair(id1, id2);
      loopEdgePairs_.push_back(edge);
      loop_num++;
    }else{
      NonlinearFactor::shared_ptr interRobotFactorID2(new BetweenFactor<Pose3>(id2, id1, factorPose3.inverse(), odometryNoise));
      graphAndValuesVec[Key2robotID(id1)].first->push_back(interRobotFactorID2);
      graphAndValuesVec[Key2robotID(id2)].first->push_back(interRobotFactorID2);
      // Add to graph visualization
      cout << " add edge into current graph inverse id2: " << id2 << " id1: " << id1 << endl;
      std::pair<uint64_t, uint64_t> edge = make_pair(id2, id1);
      loopEdgePairs_.push_back(edge);
      loop_num++;
    }
  }
  // string distOptimized = "/home/client/graph/0.g2o";
  // writeG2o(*graphAndValuesVec[0].first, *graphAndValuesVec[0].second, distOptimized);
  if(loop_num >= 2)
    aLoopIsClosed = true;
}


/*
 * Loop detection and check its validation. If true loop, then add loop factor to the graph
 */
void GlobalManager::performLoopClosure()
{
  ROS_DEBUG("Performing Loop Closure.");

  // Index represents robotid, content represents the robot newest desc
  DiSCOVec query_desc;
  DiSCOFFTVec query_fft;
  // Index represents robotid, content represents the looped robotids and its correspoinding disco index 
  std::vector<std::tuple<std::vector<int>, std::vector<int>, std::vector<pcl::PointXYZI>>> loopAllInfo;
  // Current descriptor's index in disco_database, index represents robotid, content represents the disco index
  std::vector<int> curr_ind;
  std::vector<int> curr_id;

  if(nrRobots == 0)
    return;
  
  // Copy every robot newest desc to a local variable
  std::unique_lock<std::mutex> lock(subscriptions_mutex_);
  for(auto& subscription: subscriptions_){
    std::lock_guard<std::mutex> lock2(subscription.mutex);
    
    if(subscription.disco_base.size() < 2 || subscription.submaps.size() < 2)
      return;

    // cout << "subscription.disco_base.size(): " << subscription.disco_base.size() << " subscription.submaps.size(): " << subscription.submaps.size() << endl;
    if(subscription.disco_base.size() > subscription.submaps.size()){// sync reason: keyframe and disco are not always synchronized
      DiSCOFFT curr_fft;
      std::vector<float> curr_desc;
      curr_desc = subscription.disco_base[subscription.submaps.size() - 2]; 
      curr_fft = subscription.disco_fft[subscription.submaps.size() - 2]; 
      query_fft.push_back(curr_fft);
      query_desc.push_back(curr_desc);
      curr_ind.push_back(subscription.submaps.size() - 2);
      curr_id.push_back(subscription.robot_id);
    }else{
      DiSCOFFT curr_fft;
      std::vector<float> curr_desc;
      curr_desc = subscription.disco_base.back();
      curr_fft = subscription.disco_fft.back();
      query_fft.push_back(curr_fft);
      query_desc.push_back(curr_desc);
      curr_ind.push_back(subscription.submaps.size() - 1);
      curr_id.push_back(subscription.robot_id);
    }
  }

  // Because the forward_list insert from the front
  reverse(curr_ind.begin(), curr_ind.end());
  reverse(query_fft.begin(), query_fft.end());
  reverse(query_desc.begin(), query_desc.end());
  lock.unlock();
  
  // Every newest descriptor to detect inter loop closure
  for(int i = 0; i < query_desc.size(); i++){
    auto result = detectLoopClosure(query_desc[i], query_fft[i], curr_ind[i], i);
    loopAllInfo.push_back(result);
  }

  float noiseScore = 1; // constant is ok...
  gtsam::Vector Vector6(6);
  float x, y, z, icp_roll, icp_pitch, icp_yaw;
  Eigen::Affine3f correctionKeyframe;
  Vector6 << noiseScore, noiseScore, noiseScore, noiseScore, noiseScore, noiseScore;
  constraintNoise = noiseModel::Diagonal::Variances(Vector6);
  robustNoiseModel = gtsam::noiseModel::Robust::Create(
              gtsam::noiseModel::mEstimator::Cauchy::Create(1), // optional: replacing Cauchy by DCS or GemanMcClure
              gtsam::noiseModel::Diagonal::Variances(Vector6)
          ); // - checked it works. but with robust kernel, map modification may be delayed (i.e,. requires more true-positive loop factors)

  // Iter all loop info
  for(auto loopInfo = loopAllInfo.begin(); loopInfo != loopAllInfo.end(); loopInfo++){

    PointCloudIPtr currentKeyframe(new PointCloudI);
    int currentRobotIDinStack = loopInfo - loopAllInfo.begin();
    int currentDiSCOID = curr_ind[currentRobotIDinStack];
    int realRobotID = robotIDStack[currentRobotIDinStack];

    std::unique_lock<std::mutex> obslock(subscriptions_mutex_);
    auto currentRobotHandle = subscriptions_.begin();
    std::advance(currentRobotHandle, nrRobots - currentRobotIDinStack - 1);
    pcl::copyPointCloud(*currentRobotHandle->keyframes[currentDiSCOID], *currentKeyframe);
    // auto currentPose = currentRobotHandle->trajectory[currentDiSCOID];
    auto currentYaw = currentRobotHandle->cloudKeyPoses6D->points.back().yaw;
    obslock.unlock();

    // cout << "current Robot: " << realRobotID << " current DiSCO ID: " << currentDiSCOID;
    // cout << " current traj x: " << currentPose(0,3) << " y: " << currentPose(1,3) << " z: " << currentPose(2,3) << endl;

    bool isValidloopFactor = false;
    auto interLoopRobots = std::get<0>(*loopInfo); // robot index
    auto interDiscoIndex = std::get<1>(*loopInfo);  // disco index for each robot
    auto relPoses = std::get<2>(*loopInfo);  // disco index for each robot

    // use ICP result to check validation of each loop
    auto start = system_clock::now();
    for(auto i = 0; i < interLoopRobots.size(); i++){
      int interLoopRobotIDinStack = interLoopRobots[i];
      int queryDiSCOIndex = interDiscoIndex[i];
      int realInterLoopRobotID = robotIDStack[interLoopRobotIDinStack];

      if(interLoopRobotIDinStack != -1 && relPoses[i].intensity != -1 && currentRobotIDinStack != interLoopRobotIDinStack){// && (currentRobotIDinStack != interLoopRobotIDinStack || (currentRobotIDinStack == interLoopRobotIDinStack && abs(currentDiSCOID - queryDiSCOIndex) > 2 ))){ // check the validation of the info, discard consecutive loops
        cout << "ICP check between robot " << realRobotID << " des: " << currentDiSCOID << " and robot " << realInterLoopRobotID << " des: " << queryDiSCOIndex << endl;
        
        // Get query keyframe 
        std::unique_lock<std::mutex> obslock(subscriptions_mutex_);
        PointCloudIPtr queryKeyframe(new PointCloudI);       
        PointCloudIPtr queryKeyframeTransformed(new PointCloudI);        
        auto interLoopRobotHandle = subscriptions_.begin();
        std::advance(interLoopRobotHandle, nrRobots - interLoopRobotIDinStack - 1);
        pcl::copyPointCloud(*interLoopRobotHandle->keyframes[queryDiSCOIndex], *queryKeyframe);
        // auto queryPose = interLoopRobotHandle->trajectory[queryDiSCOIndex]; 
        obslock.unlock();

        // cout << "query Robot: " << realInterLoopRobotID << " query DiSCO ID: " << queryDiSCOIndex;
        // cout << " query traj x: " << queryPose(0,3) << " y: " << queryPose(1,3) << " z: " << queryPose(2,3) << endl;

        auto start = system_clock::now();

        // pcl::io::savePCDFileASCII ("/home/client/graph/query_"+ to_string(realInterLoopRobotID) + "_" + to_string(queryDiSCOIndex) + "_" + to_string(relPoses[i].intensity) + ".pcd", *queryKeyframe);

        // DiSCO get the relative angle between two scans
        float relative_x = relPoses[i].x*cos(currentYaw) + relPoses[i].y*sin(currentYaw);
        float relative_y = -relPoses[i].x*sin(currentYaw) + relPoses[i].y*cos(currentYaw);

        Eigen::Isometry3f T = Pose3toIsometry(Pose3(Rot3::RzRyRx(0.0, 0.0, -relPoses[i].intensity / 180.0 * M_PI), Point3(-relative_x, -relative_y, 0.0)));
        Eigen::Matrix4f transformMatrix = T.matrix();
        pcl::transformPointCloud(*queryKeyframe, *queryKeyframeTransformed, transformMatrix); 

        cout << "relative angle: " << relPoses[i].intensity << " relative_x: " << relative_x << " relative y: " << relative_y << " relative z: " << relPoses[i].z << endl;
        // pcl::io::savePCDFileASCII ("/home/client/graph/current_"+ to_string(realRobotID) + "_" + to_string(currentDiSCOID) + "_" + to_string(relPoses[i].intensity) + ".pcd", *currentKeyframe);
        // pcl::io::savePCDFileASCII ("/home/client/graph/query_after_"+ to_string(realInterLoopRobotID) + "_" + to_string(queryDiSCOIndex) + "_" + to_string(relPoses[i].intensity) + ".pcd", *queryKeyframeTransformed);

        auto end = system_clock::now();
        auto duration = duration_cast<microseconds>(end - start);
        // cout << "Downsampling spend " << double(duration.count()) * microseconds::period::num / microseconds::period::den << "s" << endl;
        // cout << "Downsampled cloud size: current - " << currentKeyframe->size() << " query - " << queryKeyframe->size() << endl;
        start = system_clock::now();

        // PointCloudINPtr currentKeyframeNormal(new PointCloudIN);
        // PointCloudINPtr queryKeyframeNormal(new PointCloudIN);
        // addNormal(currentKeyframe, currentKeyframeNormal);
        // addNormal(queryKeyframe, queryKeyframeNormal);

        pcl::IterativeClosestPoint<PointTI, PointTI> icp;
        icp.setMaxCorrespondenceDistance(2.0);
        icp.setMaximumIterations((int)icp_iters_);
        icp.setTransformationEpsilon(1e-3);
        icp.setEuclideanFitnessEpsilon(1e-3);
        // icp.setRANSACIterations(0);
        icp.setInputSource(currentKeyframe);
        icp.setInputTarget(queryKeyframeTransformed);
        PointCloudIPtr unused_result(new PointCloudI);
        icp.align(*unused_result); 

        end = system_clock::now();
        duration = duration_cast<microseconds>(end - start);
        cout <<  "ICP align spend " << double(duration.count()) * microseconds::period::num / microseconds::period::den << "s" << endl;

        std::cout << "[DSC] ICP fit score: " << icp.getFitnessScore() << std::endl;
        // string distOptimized = "/home/client/graph/" + boost::lexical_cast<string>(currentRobotIDinStack) + "Graph_origin.g2o";
        // writeG2o(*graphAndValuesVec[currentRobotIDinStack].first, *graphAndValuesVec[currentRobotIDinStack].second, distOptimized);
        
        // check ICP result
        if ( icp.hasConverged() == false || icp.getFitnessScore() > acceptedKeyframeFitnessScore ) {
          std::cout << "[DSC] Reject this loop (bad icp fit score, > " << acceptedKeyframeFitnessScore << ")" << std::endl;
          isValidloopFactor = false;
        }
        else {
          std::cout << "[DSC] The detected loop factor is added between Robot " << realRobotID 
                    << " keyframe id: [ " << currentDiSCOID << " ] and nearest Robot " << realInterLoopRobotID 
                    << " keyframe id:  [ " << queryDiSCOIndex << " ]" << std::endl;

          isValidloopFactor = true;
        }
        
        // Add factors to graph
        if( isValidloopFactor ) {
          correctionKeyframe = icp.getFinalTransformation(); // get transformation in camera frame (because points are in camera frame)
          pcl::getTranslationAndEulerAngles (correctionKeyframe, x, y, z, icp_roll, icp_pitch, icp_yaw);
          cout << "origin icp_yaw: " << icp_yaw / M_PI * 180.0 << " x: " << x << " y: " << y << endl;

          uint64_t id1, id2;
          // need to +1 because the Index begin at 1
          id1 = robotID2Key(realRobotID-start_robot_id_) + currentDiSCOID + 1;
          id2 = robotID2Key(realInterLoopRobotID-start_robot_id_) + queryDiSCOIndex + 1; 

          // Need to compensate the relative angle
          Eigen::Affine3f relMatrix = Eigen::Affine3f::Identity();
          double theta = -relPoses[i].intensity / 180.0 * M_PI;  // The angle of rotation in radians
          relMatrix(0, 0) = std::cos (theta);
          relMatrix(0, 1) = sin (theta);
          relMatrix(1, 0) = -sin (theta);
          relMatrix(1, 1) = std::cos (theta);
          correctionKeyframe = relMatrix * correctionKeyframe;
          pcl::getTranslationAndEulerAngles (correctionKeyframe, x, y, z, icp_roll, icp_pitch, icp_yaw);
          cout << "icp_yaw: " << icp_yaw / M_PI * 180.0 << " x: " << x << " y: " << y << endl;

          gtsam::Pose3 poseFrom = Pose3(Rot3::RzRyRx(icp_roll, icp_pitch, icp_yaw), Point3(x + relative_x, y + relative_y, z));
          gtsam::Pose3 poseTo = Pose3(Rot3::RzRyRx(0.0, 0.0, 0.0), Point3(0.0, 0.0, 0.0));

          std::lock_guard<std::mutex> lock(graph_mutex_);
          
          // Add inter robot factor to the graph
          {
            // first id need to be same as the robot stack id. 
            if(id1 < id2){
              NonlinearFactor::shared_ptr interRobotFactorID1(new BetweenFactor<Pose3>(id1, id2, poseFrom.between(poseTo), constraintNoise));
              graphAndValuesVec[realRobotID-start_robot_id_].first->push_back(interRobotFactorID1);
              graphAndValuesVec[realInterLoopRobotID-start_robot_id_].first->push_back(interRobotFactorID1);
              // Add to graph visualization
              cout << " add edge into current graph " << endl;
              std::pair<uint64_t, uint64_t> edge = make_pair(id1, id2);
              loopEdgePairs_.push_back(edge);
            }else{
              NonlinearFactor::shared_ptr interRobotFactorID2(new BetweenFactor<Pose3>(id2, id1, poseTo.between(poseFrom), constraintNoise));
              graphAndValuesVec[realRobotID-start_robot_id_].first->push_back(interRobotFactorID2);
              graphAndValuesVec[realInterLoopRobotID-start_robot_id_].first->push_back(interRobotFactorID2);
              // Add to graph visualization
              cout << " add edge into current graph " << endl;
              std::pair<uint64_t, uint64_t> edge = make_pair(id2, id1);
              loopEdgePairs_.push_back(edge);
            }

            std::cout << "[DSC] Add loop factor into the graph" << std::endl; 
            // string distOptimized = "/home/client/graph/" + boost::lexical_cast<string>(currentRobotIDinStack) + "Graph_origin.g2o";
            // writeG2o(*graphAndValuesVec[currentRobotIDinStack].first, *graphAndValuesVec[currentRobotIDinStack].second, distOptimized);
            // string distOptimized_inter = "/home/client/graph/" + boost::lexical_cast<string>(interLoopRobotIDinStack) + "Graph_origin.g2o";
            // writeG2o(*graphAndValuesVec[interLoopRobotIDinStack].first, *graphAndValuesVec[interLoopRobotIDinStack].second, distOptimized_inter);
        
            // pcl::io::savePCDFileASCII ("/home/client/graph/current_" + to_string(currentDiSCOID) + ".pcd", *currentKeyframe);
            // pcl::io::savePCDFileASCII ("/home/client/graph/query_" + to_string(queryDiSCOIndex) + ".pcd", *queryKeyframe);
          }
          
          // Enable loop
          Eigen::Isometry3f T = Eigen::Isometry3f::Identity();
          Eigen::Isometry3f IncrementTrans = Eigen::Isometry3f::Identity();
          if(currentRobotIDinStack != interLoopRobotIDinStack)
            loop_num++;
          if(loop_num > 1)
            aLoopIsClosed = true;

        } // check DiSCO validation
      } // check the validation of the info
    } // ICP check

    auto end = system_clock::now();
    auto duration = duration_cast<microseconds>(end - start);
    cout <<  "ICP check spend " << double(duration.count()) * microseconds::period::num / microseconds::period::den << "s" << endl;
  }
}


/*
 * Detect loop closure for a robot with its newest descriptor
 * Output: loop robotid and its disco index
 */
std::tuple<std::vector<int>, std::vector<int>, std::vector<pcl::PointXYZI>> GlobalManager::detectLoopClosure(std::vector<float> curr_desc, DiSCOFFT curr_fft, int curr_ind, int currentRobotIDinStack)
{
  ROS_DEBUG("Detecting Loop Closure.");
  std::unique_lock<std::mutex> locktree(kdtree_mutex_);
  
  float query[DISCO_DIM];
  int realRobotID = robotIDStack[currentRobotIDinStack];

  // Dynamic kdtree with insert operation
  kdtree_build(kdtree);

  memcpy(query, &curr_desc[0], curr_desc.size() * sizeof(curr_desc[0]));
  kdtree_knn_search(kdtree, query, NUM_CANDIDATES_FROM_TREE);

  auto kd_result = kdtree_knn_result(kdtree);
  locktree.unlock();

  // Index represents num candidates, content represents the origin place of the corresponding candidate
  std::vector<int> discoIndex;
  std::vector<pcl::PointXYZI> relPoses;  // point.xyz -> relative x y z; point.intensity -> relative yaw
  std::vector<int> closestRobotID;

  // Skip very first observations
  if(kdtree->count < 3){
    std::vector<int> falseReturn;
    std::vector<pcl::PointXYZI> falseReturnF;
    pcl::PointXYZI falseReturnP;
    falseReturnP.x = falseReturnP.y = falseReturnP.z = 0.0;
    falseReturnP.intensity = -1.0;
    falseReturn.push_back(-1);
    falseReturnF.push_back(falseReturnP);
    std::tuple<std::vector<int>, std::vector<int>, std::vector<pcl::PointXYZI>> results{falseReturn, falseReturn, falseReturnF};
    cout << "False return" << endl;
    return results;
  }

  if(odometryLoopEnable_){
    // Find the multi-agent closest history keyframe
    std::vector<int> pointSearchIndLoop;
    std::vector<float> pointSearchSqDisLoop;
    std::vector<int> traj_cloud_size;
    pcl::PointCloud<PointTypePose>::Ptr traj_cloud6D(new pcl::PointCloud<PointTypePose>);

    std::vector<PointTypePose> currentRobotPosPoint6D;
    std::unique_lock<std::mutex> lock(subscriptions_mutex_);
    for(auto& subscription: subscriptions_){
      std::lock_guard<std::mutex> lock2(subscription.mutex);
      *traj_cloud6D += *subscription.cloudKeyPoses6D;
      traj_cloud_size.push_back(subscription.cloudKeyPoses6D->size());
      currentRobotPosPoint6D.push_back(subscription.cloudKeyPoses6D->back());
    }
    lock.unlock();

    // Tackle forward list
    reverse(currentRobotPosPoint6D.begin(), currentRobotPosPoint6D.end());

    // Kdtree find the single agent closest history key frame
    kdtreeHistoryKeyPoses6D->setInputCloud(traj_cloud6D);
    kdtreeHistoryKeyPoses6D->radiusSearch(currentRobotPosPoint6D[currentRobotIDinStack], historyKeyframeSearchRadius, pointSearchIndLoop, pointSearchSqDisLoop, 0);
    int RSclosestHistoryFrameID = -1;
    int interRobotID = -1;
    int interRobotIDinStack = -1;
    int curMinID = 1000000;
    pcl::PointXYZI relPose;

    // Policy: take Oldest one (to fix error of the whole trajectory)
    for (int j = 1; j < pointSearchIndLoop.size(); ++j){
      int idinKdtree = pointSearchIndLoop[j];
      int kfIdinRobotStack = 0;
      int trajCloudSize = 0;

      // Check the id belongs to which robot
      for(int index = 0; index < traj_cloud_size.size(); index++){
        trajCloudSize += traj_cloud_size[index];
        if(idinKdtree - trajCloudSize + 1 <= 0){
          kfIdinRobotStack = idinKdtree - trajCloudSize + traj_cloud_size[index];
          break;
        }
      }

      // Get the real robot id
      if(realRobotID != (int)traj_cloud6D->points[idinKdtree].intensity){
        interRobotID = (int)traj_cloud6D->points[idinKdtree].intensity;
        relPose.intensity = (currentRobotPosPoint6D[currentRobotIDinStack].yaw - traj_cloud6D->points[idinKdtree].yaw) / M_PI * 180.0;
        relPose.x = currentRobotPosPoint6D[currentRobotIDinStack].x - traj_cloud6D->points[idinKdtree].x;
        relPose.y = currentRobotPosPoint6D[currentRobotIDinStack].y - traj_cloud6D->points[idinKdtree].y;
        relPose.z = currentRobotPosPoint6D[currentRobotIDinStack].z - traj_cloud6D->points[idinKdtree].z;
        RSclosestHistoryFrameID = kfIdinRobotStack;
      }

      // Get the robot id in the robotidstack
      vector<int>::iterator it = find(robotIDStack.begin(), robotIDStack.end(), interRobotID);
      interRobotIDinStack = it - robotIDStack.begin();

      // Multi-robot loop
      if(RSclosestHistoryFrameID != -1 && interRobotID != -1){
        closestRobotID.emplace_back(interRobotIDinStack);
        discoIndex.emplace_back(RSclosestHistoryFrameID);
        relPoses.emplace_back(relPose);  
        cout << "[RS Loop found] btn current robot " << realRobotID << " index: " << curr_ind << " and robot " << interRobotID << " index: " << discoIndex.back() << " interRobotIDinStack: " << interRobotIDinStack << endl;
      }
    }
  }else{
    // Find the single agent closest history keyframe
    std::vector<int> pointSearchIndLoop;
    std::vector<float> pointSearchSqDisLoop;

    std::vector<int> traj_cloud_size;
    pcl::PointCloud<PointTypePose>::Ptr traj_cloud6D(new pcl::PointCloud<PointTypePose>);
    
    // copy trajectory cloud from subscriptions  
    std::vector<PointTypePose> currentRobotPosPoint6D;
    std::unique_lock<std::mutex> lock(subscriptions_mutex_);
    for(auto& subscription: subscriptions_){
      std::lock_guard<std::mutex> lock2(subscription.mutex);
      *traj_cloud6D += *subscription.cloudKeyPoses6D;
      traj_cloud_size.push_back(subscription.cloudKeyPoses6D->size());
      currentRobotPosPoint6D.push_back(subscription.cloudKeyPoses6D->back());
    }
    lock.unlock();

    // Tackle forward list
    reverse(currentRobotPosPoint6D.begin(), currentRobotPosPoint6D.end());

    // Kdtree find the single agent closest history key frame
    kdtreeHistoryKeyPoses6D->setInputCloud(traj_cloud6D);
    kdtreeHistoryKeyPoses6D->radiusSearch(currentRobotPosPoint6D[currentRobotIDinStack], historyKeyframeSearchRadius, pointSearchIndLoop, pointSearchSqDisLoop, 0);
    int RSclosestHistoryFrameID = -1;
    int interRobotID = -1;
    int interRobotIDinStack = -1;
    int curMinID = 1000000;

    // Policy: take Oldest one (to fix error of the whole trajectory)
    for (int j = 0; j < pointSearchIndLoop.size(); ++j){
      int id = pointSearchIndLoop[j];
      if( id < curMinID && id < curr_ind) {
        curMinID = id;
        RSclosestHistoryFrameID = curMinID;
      }
    }

    // Single robot loop
    // if(RSclosestHistoryFrameID != -1){
    //   closestRobotID.emplace_back(currentRobotIDinStack);
    //   discoIndex.emplace_back(RSclosestHistoryFrameID);
    //   pcl::PointXYZI relPose;
    //   relPose.x = 0.0;
    //   relPose.y = 0.0;
    //   relPose.z = 0.0;
    //   relPose.intensity = 0.0;
    //   relPoses.emplace_back(relPose);  
    //   cout << "[RS Loop found] btn current robot " << realRobotID << " index: " << curr_ind << " and robot " << realRobotID << " index: " << discoIndex.back() << " interRobotIDinStack: " << interRobotIDinStack << endl;
    // }
  }

  

  // // First one is itself, iter all other candidates
  // for(int candidate_iter_idx = 1; candidate_iter_idx < NUM_CANDIDATES_FROM_TREE; candidate_iter_idx++){
    
  //   if(kd_result[candidate_iter_idx].distance < DISCO_DIST_THRES){
  //     int iterRobotIDinSub = nrRobots;

  //     // Find which the candidate belongs to which robot
  //     std::unique_lock<std::mutex> lock(subscriptions_mutex_);
  //     for(auto& subscription: subscriptions_){
  //       vector<int>::iterator it = find(subscription.disco_index.begin(), subscription.disco_index.end(), kd_result[candidate_iter_idx].coord_index);
        
  //       // Find which the candidate belongs to which robot, and return robotID and index in this robot map stack
  //       if(it != subscription.disco_index.end()){
  //         if((it - subscription.disco_index.begin()) < curr_ind){
  //           int retrieved_ind = it - subscription.disco_index.begin();
  //           int realIterRobotID = subscription.robot_id;
            
  //           // if(curr_ind != (retrieved_ind + 1)){
  //           // closestRobotID.emplace_back(subscription.robot_id);
  //           closestRobotID.emplace_back(iterRobotIDinSub - 1);
  //           discoIndex.emplace_back(retrieved_ind);

  //           DiSCOFFT closestDiSCO = subscription.disco_fft[discoIndex.back()];
  //           float relAngle = calcRelOri(curr_fft, closestDiSCO);
  //           relAngles.emplace_back(relAngle);
  //           // }

  //           std::cout.precision(5);
  //           cout << "[Loop found] Nearest distance: "  << kd_result[candidate_iter_idx].distance << " btn current robot " << realRobotID << " index: " << curr_ind << " and robot " << realIterRobotID << " index: " << discoIndex.back() << endl;
  //           break;
  //         }
  //       }
  //       iterRobotIDinSub --;
  //     }
  //     lock.unlock();
  //   }else{
  //     // Report not a loop and return -1 (default value for no loop)
  //     int iterRobotIDinSub = nrRobots;

  //     // Find which the candidate belongs to which robot
  //     std::unique_lock<std::mutex> lock(subscriptions_mutex_);
  //     for(auto& subscription: subscriptions_){
  //       vector<int>::iterator it = find(subscription.disco_index.begin(), subscription.disco_index.end(), kd_result[candidate_iter_idx].coord_index);
        
  //       // Find which the candidate belongs to which robot, and return robotID and index in this robot map stack
  //       if(it != subscription.disco_index.end()){          
  //         int notFound = -1;
  //         closestRobotID.emplace_back(notFound);
  //         discoIndex.emplace_back(notFound);
  //         relAngles.emplace_back(notFound);  
  //         std::cout.precision(5); 
  //         int realIterRobotID = robotIDStack[iterRobotIDinSub-1];
  //         cout << "[Not A Loop] Nearest distance: "  << kd_result[candidate_iter_idx].distance << " btn current robot " << realRobotID << " index: " << curr_ind << " and robot " << realIterRobotID << " index: " << discoIndex.back() << endl;
  //         break;
  //       }
  //       iterRobotIDinSub --;
  //     }
  //     lock.unlock();
  //   }
  // }
  
  int candidate_iter_idx = 1;
  int inter_loop_num = 0;
  while(inter_loop_num < NUM_CANDIDATES_FROM_TREE && candidate_iter_idx < kd_result.size()){

    if(kd_result[candidate_iter_idx].distance < DISCO_DIST_THRES){
      int iterRobotIDinSub = nrRobots;

      // Find which the candidate belongs to which robot
      std::unique_lock<std::mutex> lock(subscriptions_mutex_);
      for(auto& subscription: subscriptions_){
        vector<int>::iterator it = find(subscription.disco_index.begin(), subscription.disco_index.end(), kd_result[candidate_iter_idx].coord_index);
        
        // Find which the candidate belongs to which robot, and return robotID and index in this robot map stack
        if(it != subscription.disco_index.end()){
          if((it - subscription.disco_index.begin()) < curr_ind){
            int retrieved_ind = it - subscription.disco_index.begin();
            int realIterRobotID = subscription.robot_id;
            
            if(realRobotID == realIterRobotID)
              break;

            // if(curr_ind != (retrieved_ind + 1)){
            // closestRobotID.emplace_back(subscription.robot_id);
            closestRobotID.emplace_back(iterRobotIDinSub - 1);
            discoIndex.emplace_back(retrieved_ind);

            DiSCOFFT closestDiSCO = subscription.disco_fft[discoIndex.back()];
            float relAngle = calcRelOri(curr_fft, closestDiSCO);
            pcl::PointXYZI relPose;
            relPose.x = 0.0;
            relPose.y = 0.0;
            relPose.z = 0.0;
            relPose.intensity = -relAngle;
            relPoses.emplace_back(relPose);
            // }

            std::cout.precision(5);
            cout << "[Loop found] Nearest distance: "  << kd_result[candidate_iter_idx].distance << " btn current robot " << realRobotID << " index: " << curr_ind << " and robot " << realIterRobotID << " index: " << discoIndex.back() << endl;
            inter_loop_num++;
            break;
          }
        }
        iterRobotIDinSub --;
      }
      lock.unlock();
    }
    candidate_iter_idx++;
  }

  knn_list_reset(kdtree);
  
  std::tuple<std::vector<int>, std::vector<int>, std::vector<pcl::PointXYZI>> results{closestRobotID, discoIndex, relPoses};
  return results;
}


/*
 * Construct optimizer utilized in distributed mapper
 */
vector<int> GlobalManager::constructOptimizer(bool savingMode)
{
  // int nrCommLinks = 0;
  distMappers.clear();
  disconnectedRobotIDVec.clear();

  // Index represent robotid
  vector<int> initial_size;
  initial_size.clear();

  // Init vector
  for(int i = 0; i < nrRobots; i++){
    initial_size.push_back(0);
  }

  // Fast clear
  GraphAndValuesVec tmp;
  DistGraphAndValuesVec.swap(tmp);

  std::vector<int> robotIDSortedIndex = sort_indexes(robotIDStack);
  
  // Load subgraph and construct distMapper optimizers
  std::lock_guard<std::mutex> lock(graph_mutex_);

  for(size_t iter = 0; iter < nrRobots; iter++){

    // int robot = robotIDSortedIndex[iter];
    int robot = iter;
    
    // Construct a distributed jacobi object with the given robot name
    boost::shared_ptr<DistributedMapper> distMapper(new DistributedMapper(robotNames_[iter], useChrLessFullGraph));

    // Read G2o files
    GraphAndValues graphAndValuesG2o = graphAndValuesVec[robot];
    Values initial = *(graphAndValuesG2o.second);
    initial_size[robot] = initial.size();

    cout << "robot: " << robot << endl;
    cout << "nrRobots: " << nrRobots << endl;
    cout << "graphAndValuesVec: " << graphAndValuesVec.size() << endl;
    cout << "initial: " << initial.size() << endl;

    for(size_t it = 0; it < nrRobots; it++){
      string distOptimized = "/home/client/graph/" + boost::lexical_cast<string>(it) + ".g2o";
      writeG2o(*graphAndValuesVec[it].first, *graphAndValuesVec[it].second, distOptimized);
    } 

    // Continue if empty
    if(initial.empty()){
      // disconnectedGraph = true;
      cout << "initial empty" << endl;
      continue;
    }

    // Construct graphAndValues using cleaned up initial values
    // GraphAndValues graphAndValues = make_pair(graphAndValuesG2o.first, boost::make_shared<Values>(initial));
    // DistGraphAndValuesVec.push_back(graphAndValuesG2o);

    // Use between noise or not in optimizePoses
    distMapper->setUseBetweenNoiseFlag(useBetweenNoise);

    // Use landmarks
    distMapper->setUseLandmarksFlag(useLandmarks);

    // Load subgraphs
    distMapper->loadSubgraphAndCreateSubgraphEdge(graphAndValuesG2o);

    // Add prior to the first robot
    if(iter == 0){
      Key firstKey = KeyVector(initial.keys()).at(0);
      distMapper->addPrior(firstKey, initial.at<Pose3>(firstKey), priorNoise);
    }

    // Verbosity level
    distMapper->setVerbosity(DistributedMapper::ERROR);

    // Check for graph connectivity
    std::set<char> neighboringRobots = distMapper->getNeighboringChars();
    if(neighboringRobots.size() == 0 && !savingMode){
      // disconnectedGraph = true;
      disconnectedRobotIDVec.push_back(iter);
      cout << "no neighbor robot found, robot id: " << iter << endl;
    }else{
      DistGraphAndValuesVec.push_back(graphAndValuesG2o);
    }
      
    // Number of communication links
    // vector<size_t> commEdges = distMapper->seperatorEdge();
    // nrCommLinks+=commEdges.size();

    // Push to the set of optimizers
    distMappers.push_back(distMapper);
  }
  return initial_size;
}


/**
 * @brief Correct pose using merged pose graph.
 * @details When pose graph is constructed, they are optimized 
 */
std::pair<Values, vector<int>> GlobalManager::correctPoses(bool updateGraph)
{  
  disconnectedGraph = false;

  // Maintaining the initial size of the optimizer in the correction step
  vector<int> initial_size;
  initial_size = constructOptimizer();

  // Vectors containing logs
  vector < Values > rotationTrace;
  vector < Values > poseTrace;
  vector < Values > subgraphRotationTrace;
  vector < Values > subgraphPoseTrace;
  vector < VectorValues > rotationVectorValuesTrace;

  // noiseModel::Diagonal::shared_ptr priorModel = noiseModel::Isotropic::Variance(6, 1e-12); // prior noise
  noiseModel::Isotropic::shared_ptr model = noiseModel::Isotropic::Variance(12, 1);

  cout << "DisconnectedGraph Flag: " << disconnectedGraph << endl;
  if(!disconnectedGraph){
    try{
      // int max_clique_size = 0;
      // double pcm_threshold = 0.5;
      // double pose_estimate_change_threshold = 1e-2;
      // bool use_PCM = false;
      // bool use_flagged_init = true;
      // bool use_covariance = false;
      // bool use_heuristics = true;
      // vector <Values> estimates = distributedOptimizer(distMappers, maxIter, max_clique_size, updateType, gamma, rotationEstimateChangeThreshold,
      //                                                   poseEstimateChangeThreshold, use_flagged_init, useLandmarks, debug, true,
      //                                                   pcm_threshold, use_covariance, use_PCM, use_heuristics,
      //                                                   DistGraphAndValuesVec, rotationTrace, poseTrace, subgraphRotationTrace,
      //                                                   subgraphPoseTrace, rotationVectorValuesTrace);

      // // vector< Values > estimates = distributedOptimizer(distMappers, maxIter, updateType, gamma, rotationEstimateChangeThreshold, 
      // //                                                   poseEstimateChangeThreshold, useFlaggedInit, useLandmarks, debug, rotationTrace, 
      // //                                                   poseTrace, subgraphRotationTrace, subgraphPoseTrace, rotationVectorValuesTrace);
      // if(debug)
      //   cout << "Done" << endl;

      // // Aggregate estimates from all the robots
      // Values distributed;
      // for(size_t i = 0; i < estimates.size(); i++){
      //   for(const Values::ConstKeyValuePair& key_value: estimates[i]){
      //     Key key = key_value.key;
      //     if(!distributed.exists(key))
      //       distributed.insert(key, estimates[i].at<Pose3>(key));
      //   }
      // }

      // if(debug)
      //   cout << "Done Aggregating" << endl;
      
      ////////////////////////////////////////////////////////////////////////////////
      // Read full graph and add prior
      ////////////////////////////////////////////////////////////////////////////////

      GraphAndValues fullGraphAndValues = readFullGraph();
      NonlinearFactorGraph fullGraph = *(fullGraphAndValues.first);
      Values fullInitial = *(fullGraphAndValues.second);

      // Write centralized two stage + GN
      string debug2 = "/home/client/graph/readFullGraph.g2o";
      writeG2o(fullGraph, fullInitial, debug2);

      // Add prior
      NonlinearFactorGraph fullGraphWithPrior = fullGraph.clone();
      Key priorKey = KeyVector(fullInitial.keys()).at(0);

      NonlinearFactor::shared_ptr prior(new PriorFactor<Pose3>(priorKey, fullInitial.at<Pose3>(priorKey), priorNoise));
      fullGraphWithPrior.push_back(prior);

      // ////////////////////////////////////////////////////////////////////////////////
      // // Chordal Graph
      // ////////////////////////////////////////////////////////////////////////////////
      NonlinearFactorGraph chordalGraph = evaluation_utils::convertToChordalGraph(fullGraph, model, useBetweenNoise);
      // ////////////////////////////////////////////////////////////////////////////////
      // // Initial Error
      // ////////////////////////////////////////////////////////////////////////////////
      std::cout << "Initial Error: " << chordalGraph.error(fullInitial) << std::endl;

      // ////////////////////////////////////////////////////////////////////////////////
      // // Centralized Two Stage
      // ////////////////////////////////////////////////////////////////////////////////
      // Values centralized_Onestage = evaluation_utils::centralizedEstimation(fullGraphWithPrior, model, priorNoise, useBetweenNoise);
      // std::cout << "Centralized Two Stage Error: " << chordalGraph.error(centralized_Onestage) << std::endl;

      // // Write centralized full graph
      // string centralizedTwoStageFile = "/home/client/graph/centralizedTwoStage.g2o";
      // writeG2o(fullGraph, centralized, centralizedTwoStageFile);

      // ////////////////////////////////////////////////////////////////////////////////
      // // Centralized Two Stage + Gauss Newton
      // ////////////////////////////////////////////////////////////////////////////////
      Values centralized = evaluation_utils::centralizedGNEstimation(fullGraphWithPrior, model, priorNoise, useBetweenNoise);
      std::cout << "Centralized Two Stage + GN Error: " << chordalGraph.error(centralized) << std::endl;

      // Write centralized two stage + GN
      // string dist_optimized = "/home/client/graph/fullGraph.g2o";
      // writeG2o(fullGraph, centralized, dist_optimized);
      
      // auto errors = evaluation_utils::evaluateEstimates(nrRobots,
      //     fullGraphAndValues,
      //     priorNoise,
      //     model,
      //     useBetweenNoise,
      //     distributed,
      //     debug);
          
      ////////////////////////////////////////////////////////////////////////////////
      // Distributed Error
      ////////////////////////////////////////////////////////////////////////////////
      // std::cout << "Distributed Error: " << chordalGraph.error(distributed) << std::endl;

      ////////////////////////////////////////////////////////////////////////////////
      // Original Gauss-Newton method
      ////////////////////////////////////////////////////////////////////////////////
      // // Use Gauss-Newton method optimizes the initial values
      // GaussNewtonParams parameters;

      // // print per iteration
      // parameters.setVerbosity("ERROR");
      // GaussNewtonOptimizer optimizer(fullGraphWithPrior, fullInitial, parameters);
      // Values centralized = optimizer.optimize();

      ////////////////////////////////////////////////////////////////////////////////
      // Original LevenbergMarquardtOptimizer method
      ////////////////////////////////////////////////////////////////////////////////
      // Use LevenbergMarquardtOptimizer method optimizes the initial values
      // LevenbergMarquardtParams parameters;

      // // print per iteration
      // parameters.setVerbosity("ERROR");
      // LevenbergMarquardtOptimizer optimizer(fullGraphWithPrior, fullInitial, parameters);
      // Values centralized = optimizer.optimize();

      // Update reference cordination
      for(int i = 0; i < nrRobots; i++){
        std::vector<int>::iterator it = find(disconnectedRobotIDVec.begin(), disconnectedRobotIDVec.end(), i);
        if(it != disconnectedRobotIDVec.end())
          continue;
        Pose3 robotBegin = centralized.at<Pose3>(robotID2Key(i) + 1);
        Pose3 robotEnd = centralized.at<Pose3>(robotID2Key(i) + initial_size[i] - 1);
        Eigen::Isometry3f optrobotEnd = Pose3toIsometry(robotEnd);
        Eigen::Isometry3f optrobotBegin = Pose3toIsometry(robotBegin);

        currentRef[i] = optrobotEnd * lastTF[i].inverse();
        // Fill the vacant while performing loop closing
        // TODO!!!!!!!!!!!
        for(int j = initial_size[i]; j < optMapTF.size(); j++){
          optMapTF[i][j] = currentRef[i];
        }
      }
      
      // Fill up the disconnected graph
      for(int i = 0; i < disconnectedRobotIDVec.size(); i++){
        int disconnectedRobotID = disconnectedRobotIDVec[i];
        Values initial = *(graphAndValuesVec[disconnectedRobotID].second);
        
        for(const Values::ConstKeyValuePair& key_value: initial){
          Key key = key_value.key;
          if(!centralized.exists(key))
            centralized.insert(key, initial.at<Pose3>(key));
        }
      }

      // Write centralized two stage + GN
      string debug = "/home/client/graph/fullGraph.g2o";
      writeG2o(fullGraphWithPrior, fullInitial, debug);
      
      aLoopIsClosed = false;
      std_msgs::Bool optState;
      optState.data = true;
      optimizing_state_publisher_.publish(optState);

      if(updateGraph){
        // Update graphAndValuesVec
        for(int i = 0; i < nrRobots; i++){
          for(int j = 0; j < initial_size[i]; j++){
            (*(graphAndValuesVec[i].second)).update(robotID2Key(i) + j, centralized.at<Pose3>(robotID2Key(i) + j));
          }
        }
      }

      return make_pair(centralized, initial_size);
      cout << "Optimization done " << endl;
    }
    catch(...){
      // Graph is disconnected
      cout << "Optimization failed " << endl;
      Values initial = copyInitial();

      // Fill the full graph
      for(int i = 0; i < disconnectedRobotIDVec.size(); i++){
        int disconnectedRobotID = disconnectedRobotIDVec[i];
        Values disconnectedInitial = *(graphAndValuesVec[disconnectedRobotID].second);
        
        for(const Values::ConstKeyValuePair& key_value: disconnectedInitial){
          Key key = key_value.key;
          if(!initial.exists(key))
            initial.insert(key, disconnectedInitial.at<Pose3>(key));
        }
      }
      return make_pair(initial, initial_size);
    }

  }else{
    // Graph is disconnected
    cout << "Graph is disconnected: " << endl;
    Values initial = copyInitial();
    return make_pair(initial, initial_size);
  }
  aLoopIsClosed = false;
}


/*
 * Read full graph from graphandvalues vec
 */
GraphAndValues GlobalManager::readFullGraph()
{  
  std::lock_guard<std::mutex> lock(graph_mutex_);

  // Combined graph and Values
  NonlinearFactorGraph::shared_ptr combinedGraph(new NonlinearFactorGraph);
  Values::shared_ptr combinedValues(new Values);
  
  // std::vector<int> robotIDSortedIndex = sort_indexes(robotIDStack);

  for(size_t robot = 0; robot < DistGraphAndValuesVec.size(); robot++){
    
    // int robot = robotIDSortedIndex[iter];

    // Load graph and initial
    NonlinearFactorGraph graph = *(DistGraphAndValuesVec[robot].first);
    Values initial = *(DistGraphAndValuesVec[robot].second);

    // Iterate over initial and push it to the combinedValues, each initial value is present only once
    for(const Values::ConstKeyValuePair& key_value: initial){
      Key key = key_value.key;
      if(!combinedValues->exists(key))
        combinedValues->insert(key, initial.at<Pose3>(key));
    }

    // Iterate over the graph and push the factor if it is not already present in combinedGraph
    for(size_t ksub=0; ksub<graph.size(); ksub++){ //for each factor in the new subgraph
      bool isPresent = false;
      for(size_t k=0; k<combinedGraph->size(); k++){

        boost::shared_ptr<BetweenFactor<Pose3> > factorSub =
            boost::dynamic_pointer_cast<BetweenFactor<Pose3> >(graph.at(ksub));
        Key factorSubKey1 = factorSub->key1();
        Key factorSubKey2 = factorSub->key2();

        boost::shared_ptr<BetweenFactor<Pose3> > factorCombined =
            boost::dynamic_pointer_cast<BetweenFactor<Pose3> >(combinedGraph->at(k));
        Key factorCombinedKey1 = factorCombined->key1();
        Key factorCombinedKey2 = factorCombined->key2();

        // values don't match exactly that's why check with keys as well
        if(factorCombined->equals(*factorSub) || ((factorSubKey1 == factorCombinedKey1) && (factorSubKey2 == factorCombinedKey2))){
          isPresent = true;
          break;
        }
      }
      if(isPresent == false) // we insert the factor
        combinedGraph->add(graph.at(ksub));
    }
  }
  // Return graph and values
  return make_pair(combinedGraph, combinedValues);
}


/*
 * Update submaps transformation
 */
void GlobalManager::updateTransform(const std::pair<Values, std::vector<int>> correctedPosePair)
{
  ROS_DEBUG("update Transform");
  Values correctedPose = correctedPosePair.first;
  std::vector<int> querySize = correctedPosePair.second;

  std::lock_guard<std::mutex> optMapTFLock(map_tf_mutex);
  for(int i = 0; i < nrRobots; i++){
    Pose3 robotBegin = correctedPose.at<Pose3>(robotID2Key(i));
    Pose3 robotEnd = correctedPose.at<Pose3>(robotID2Key(i) + querySize[i] - 1);

    for(int j = 0; j < querySize[i]; j++){
      Pose3 robotPose = correctedPose.at<Pose3>(robotID2Key(i) + j);
      Eigen::Isometry3f robotPoseIso = Pose3toIsometry(robotPose);

      // Update pose graph visualization
      trajPointsVec_[i][j].x = robotPoseIso(0,3);
      trajPointsVec_[i][j].y = robotPoseIso(1,3);
      trajPointsVec_[i][j].z = robotPoseIso(2,3);
      
      // First of the originMapTF is not identity matrix
      optMapTF[i][j] = robotPoseIso * originMapTF[i][j].inverse();
    }
    // Calculate transformations
    Eigen::Isometry3f optrobotEnd = Pose3toIsometry(robotEnd);
    Eigen::Isometry3f optrobotBegin = Pose3toIsometry(robotBegin);

    mapTF[i] = currentRef[i];

    // Fill the vacant while performing loop closing
    for(int j = querySize[i]; j < optMapTF.size(); j++){
      optMapTF[i][j] = currentRef[i];
    }
  }

  // Update loop edges
  std::vector<std::vector<geometry_msgs::Point>> tmp;
  std::vector<geometry_msgs::Point> newTrajVec;
  loopEdgesVec_.swap(tmp);
  for(int i = 0; i < nrRobots; i++){
    geometry_msgs::Point point;
    point.x = 0.0;
    point.y = 0.0;
    point.z = 0.0;
    newTrajVec.emplace_back(point);
    loopEdgesVec_.push_back(newTrajVec);
  }


  for(int i = 0; i < loopEdgePairs_.size(); i++){
    if(correctedPose.exists(loopEdgePairs_[i].first) && correctedPose.exists(loopEdgePairs_[i].second)){
      Pose3 edgeBegin = correctedPose.at<Pose3>(loopEdgePairs_[i].first);
      Pose3 edgeEnd = correctedPose.at<Pose3>(loopEdgePairs_[i].second);
    
      std::vector<geometry_msgs::Point> newTrajVec;

      int robotID = Key2robotID(loopEdgePairs_[i].first);

      if(loopEdgesVec_[robotID].size() == 1){
        geometry_msgs::Point trajEdgeUpdated;
        trajEdgeUpdated.x = edgeBegin.translation().x();
        trajEdgeUpdated.y = edgeBegin.translation().y();
        trajEdgeUpdated.z = edgeBegin.translation().z();
        loopEdgesVec_[robotID][0] = trajEdgeUpdated;

        trajEdgeUpdated.x = edgeEnd.translation().x();
        trajEdgeUpdated.y = edgeEnd.translation().y();
        trajEdgeUpdated.z = edgeEnd.translation().z();
        loopEdgesVec_[robotID].push_back(trajEdgeUpdated);  
      }else{
        geometry_msgs::Point trajEdgeUpdated;
        trajEdgeUpdated.x = edgeBegin.translation().x();
        trajEdgeUpdated.y = edgeBegin.translation().y();
        trajEdgeUpdated.z = edgeBegin.translation().z();
        loopEdgesVec_[robotID].push_back(trajEdgeUpdated);

        trajEdgeUpdated.x = edgeEnd.translation().x();
        trajEdgeUpdated.y = edgeEnd.translation().y();
        trajEdgeUpdated.z = edgeEnd.translation().z();
        loopEdgesVec_[robotID].push_back(trajEdgeUpdated);  
      }
    }
  }
}


/**
 * @brief copyInitial copies the initial graph to optimized graph as a fall back option
 * @param nrRobots is the number of robots
 * @param dataDir is the directory containing the initial graph
 */
Values GlobalManager::copyInitial()
{
  cout << "Copying initial to optimized" << endl;
  GraphAndValues graphAndValuesG2o = readFullGraph();
  NonlinearFactorGraph graph = *(graphAndValuesG2o.first);
  Values initial = *(graphAndValuesG2o.second);
  cout << "Copying initial done" << endl;
  return initial;
}


/*
 * Listen and process map topic from robots
 */
void GlobalManager::mapUpdate(const dislam_msgs::SubMapConstPtr& msg,
                           robotHandle_& subscription)
{
  ROS_INFO("\033[1;31m received map update from: %s \033[0m", subscription.robot_name.c_str());
  // cout << "subscription.robot_id: " << subscription.robot_id << endl;

  std::lock_guard<std::mutex> lock(subscription.mutex);
  
  // Found the right index
  int robotid = subscription.robot_id - start_robot_id_;

  pcl::PointCloud<pcl::PointXYZI>::Ptr _laser_cloud_in(new pcl::PointCloud<pcl::PointXYZI>);
  PointCloudPtr _submap_in(new PointCloud);
  Eigen::Quaternionf q(msg->pose.orientation.w, msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z);
  Eigen::Isometry3f traj(q);

  // Add tracjectory(pose) to robotHandle
  traj.pretranslate(Eigen::Vector3f(msg->pose.position.x, msg->pose.position.y, msg->pose.position.z));
  subscription.trajectory.emplace_back(traj);
  // cout << "traj: " << traj.matrix() << endl;

  Eigen::Isometry3f trajUpdated = currentRef[robotid] * traj;

  // Add to graph visualize
  geometry_msgs::Point trajPoint;
  trajPoint.x = trajUpdated(0,3);
  trajPoint.y = trajUpdated(1,3);
  trajPoint.z = trajUpdated(2,3);
  trajPointsVec_[robotid].push_back(trajPoint);

  // Transform msg type
  pcl::fromROSMsg(msg->submap, *_submap_in);
  pcl::fromROSMsg(msg->keyframePC, *_laser_cloud_in);

  // Add submap and keyframe point cloud to robotHandle
  subscription.submaps.emplace_back(_submap_in);
  
  pcl::VoxelGrid<PointTI> voxel;
  voxel.setInputCloud (_laser_cloud_in);
  voxel.setLeafSize (voxel_leaf_size_, voxel_leaf_size_, voxel_leaf_size_);
  voxel.filter (*_laser_cloud_in);

  // Remove ground points
  pcl::PassThrough<pcl::PointXYZI> pass;
  pass.setInputCloud (_laser_cloud_in); 
  pass.setFilterFieldName ("z");
  pass.setFilterLimits (-1.0, 30);
  pass.filter (*_laser_cloud_in);

  // Add to map TF stack
  std::unique_lock<std::mutex> maplock(map_mutex);
  global_map_stack[robotid].emplace_back(*_submap_in);
  maplock.unlock();

  tf::Quaternion quat;
  double roll, pitch, yaw;
  tf::quaternionMsgToTF(msg->pose.orientation, quat);
  tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
  Rot3 R = Rot3::Quaternion(quat.w(), quat.x(), quat.y(), quat.z());
  Point3 t = Point3(msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);

  subscription.transformTobeMapped[0] = roll;
  subscription.transformTobeMapped[1] = pitch;
  subscription.transformTobeMapped[2] = yaw;
  subscription.transformTobeMapped[3] = msg->pose.position.x;
  subscription.transformTobeMapped[4] = msg->pose.position.y;
  subscription.transformTobeMapped[5] = msg->pose.position.z;
  
  for(int i = 0; i < _laser_cloud_in->size(); i++){
    _laser_cloud_in->points[i].intensity = robotid * 30;
  }

  // Add graph factor to robotHandle
  if (subscription.cloudKeyPoses3D->points.empty()){
    ROS_DEBUG("Init graph");

    // Add keyframe to robot handle
    subscription.keyframes.emplace_back(_laser_cloud_in);
    subscription.lastKeyframe = *_laser_cloud_in;
    
    // Graph preparation
    GraphPtr graph(new NonlinearFactorGraph);
    ValuesPtr initial(new Values);
    
    Rot3 R_init = Rot3::RzRyRx(0.0, 0.0, 0.0);
    Point3 t_init = Point3(0,0,0);
    uint64_t id0, id1;

    // std::lock_guard<std::mutex> mapTFLock(map_tf_mutex);
    if(manual_robots_config_){
      // TF with manual setup
      mapTF[robotid] = initPoses[robotid];
      currentRef[robotid] = initPoses[robotid];
    }else{
      // Simple first tf without map origin
      mapTF[robotid] = Eigen::Isometry3f::Identity();
      currentRef[robotid] = Eigen::Isometry3f::Identity();
    }
    lastTF[robotid] = traj;

    // Every robot has its own map tf for map composing
    std::vector<Eigen::Isometry3f, Eigen::aligned_allocator<Eigen::Isometry3f>> newRobotOriginMapTF;
    std::vector<Eigen::Isometry3f, Eigen::aligned_allocator<Eigen::Isometry3f>> newRobotOptMapTF;
    Eigen::Isometry3f identity = Eigen::Isometry3f::Identity();

    originMapTF[robotid].emplace_back(traj);
    optMapTF[robotid].emplace_back(currentRef[robotid] * identity);

    // Id rule: id = char(robotid) << indexBits + storage index
    id0 = robotID2Key(robotid);
    id1 = robotID2Key(robotid) + subscription.trajectory.size();

    ROS_DEBUG("id0: %lld, id1:%lld, robotid: %d", id0, id1, robotid);
    
    // Add initial coordination
    initial->insert(id0, Pose3(R_init, t_init));
    initial->insert(id1, Pose3(R,t));
    
    // Add prior factor
    if(robotid == 0){  
      NonlinearFactor::shared_ptr factor(new BetweenFactor<Pose3>(id0, id1, Pose3(R,t), priorNoise));
      graph->push_back(factor);
    }else{
      NonlinearFactor::shared_ptr factor(new BetweenFactor<Pose3>(id0, id1, Pose3(R,t), odometryNoise));
      graph->push_back(factor);
    }
    auto pair = make_pair(graph, initial);

    std::lock_guard<std::mutex> lock(graph_mutex_);
    graphAndValuesVec[robotid] = pair;

    for (int i = 0; i < 6; ++i)
      subscription.transformLast[i] = subscription.transformTobeMapped[i];
    
    subscription.initState = false;
  }else{
    auto start = system_clock::now();
    ROS_DEBUG("Add to graph");

    // Merge previous one keyframe
    PointCloudI cloudUpdated;
    Eigen::Isometry3f prevPose = subscription.trajectory[subscription.trajectory.size() - 2];
    Eigen::Isometry3f T = traj.inverse() * prevPose;
    Eigen::Matrix4f transformMatrix = T.matrix();
    pcl::transformPointCloud(subscription.lastKeyframe, cloudUpdated, transformMatrix);
    
    // voxel.setInputCloud (_laser_cloud_in);
    // voxel.setLeafSize (voxel_leaf_size_, voxel_leaf_size_, voxel_leaf_size_);
    // voxel.filter (*_laser_cloud_in);
    
    pcl::copyPointCloud(*_laser_cloud_in, subscription.lastKeyframe);
    *_laser_cloud_in += cloudUpdated;
    subscription.keyframes.emplace_back(_laser_cloud_in);

    sensor_msgs::PointCloud2 output;
    pcl::toROSMsg(*_laser_cloud_in, output);
    subscription.keyframe_pub.publish(output);

    // Prep for factor
    uint64_t id1, id2;
    id2 = robotID2Key(robotid) + subscription.trajectory.size();
    id1 = id2 - 1; 

    // Add Odom factor
    gtsam::Pose3 poseTo = Pose3(Rot3::RzRyRx(subscription.transformTobeMapped[0], subscription.transformTobeMapped[1], subscription.transformTobeMapped[2]), Point3(subscription.transformTobeMapped[3], subscription.transformTobeMapped[4], subscription.transformTobeMapped[5]));
    gtsam::Pose3 poseFrom = Pose3(Rot3::RzRyRx(subscription.transformLast[0], subscription.transformLast[1], subscription.transformLast[2]), Point3(subscription.transformLast[3], subscription.transformLast[4], subscription.transformLast[5]));

    ROS_DEBUG("id1: %lld, id2: %lld, robotid: %d", id1, id2, robotid);

    std::lock_guard<std::mutex> lock(graph_mutex_);
    graphAndValuesVec[robotid].second->insert(id2, Pose3(R,t));

    NonlinearFactor::shared_ptr factor(new BetweenFactor<Pose3>(id1, id2, poseFrom.between(poseTo), odometryNoise));
    graphAndValuesVec[robotid].first->push_back(factor);

    // Save some transform for map composing
    Eigen::Isometry3f identity = Eigen::Isometry3f::Identity();
    std::lock_guard<std::mutex> originMapTFLock(map_tf_mutex);

    originMapTF[robotid].emplace_back(traj);
    optMapTF[robotid].emplace_back(currentRef[robotid] * identity);
    lastTF[robotid] = traj;

    auto end   = system_clock::now();
    auto duration = duration_cast<microseconds>(end - start);
    ROS_DEBUG("Add to graph done");
    cout <<  "map update spend " << double(duration.count()) * microseconds::period::num / microseconds::period::den << "s" << endl;
  }

  pcl::PointXYZI thisPose3D;
  PointTypePose thisPose6D;

  thisPose3D.x = msg->pose.position.x;
  thisPose3D.y = msg->pose.position.y;
  thisPose3D.z = msg->pose.position.z;
  thisPose3D.intensity = subscription.robot_id;
  subscription.cloudKeyPoses3D->points.emplace_back(thisPose3D);

  thisPose6D.x = thisPose3D.x;
  thisPose6D.y = thisPose3D.y;
  thisPose6D.z = thisPose3D.z;
  thisPose6D.intensity = thisPose3D.intensity;
  thisPose6D.roll  = roll;
  thisPose6D.pitch = pitch;
  thisPose6D.yaw   = yaw;
  thisPose6D.time = msg->keyframePC.header.stamp.toSec();
  subscription.cloudKeyPoses6D->points.push_back(thisPose6D);
  currentRobotPosPoint[robotid] = thisPose3D;

  timeLaserOdometry = msg->keyframePC.header.stamp.toSec();
  for (int i = 0; i < 6; ++i){
    subscription.transformLast[i] = subscription.transformTobeMapped[i];
  }

  /**
   * save updated transform
   */
  if (subscription.cloudKeyPoses3D->points.size() > 1){

  }
  ROS_DEBUG("map update done with index: %d", subscription.cloudKeyPoses3D->points.size());
}


/*
 * Listen and process disco topic from robots
 */
void GlobalManager::discoUpdate(const dislam_msgs::DiSCOConstPtr& msg,
                           robotHandle_& subscription)
{
  ROS_INFO("\033[1;33m received disco update from: %s \033[0m", subscription.robot_name.c_str());
  std::lock_guard<std::mutex> lock(subscription.mutex);

  std::vector<float> disco_r = msg->fftr;
  std::vector<float> disco_i = msg->ffti;
  std::vector<float> disco = msg->signature;

  subscription.disco_base.emplace_back(disco);
  subscription.disco_fft.emplace_back(std::make_pair(disco_r, disco_i));

  std::lock_guard<std::mutex> locktree(kdtree_mutex_);

  float disco_f[DISCO_DIM];
  memcpy(disco_f, &disco[0], disco.size() * sizeof(disco[0]));
  kdtree_insert(kdtree, disco_f);
  subscription.disco_index.emplace_back(disco_ind);
  disco_ind ++;
  ROS_DEBUG("disco update done with index: %d", subscription.disco_base.size());
}


/*
 * Compose map from map stack
 */
PointCloud GlobalManager::composeGlobalMap()
{
  ROS_DEBUG("get Maps.");

  PointCloud globalClouds;
  PointCloud globalCloudUpdated;
  PointCloud localCloudUpdated;

  local_map_stack.clear();
  local_maps.clear();

  auto start = system_clock::now();
  std::vector<bool> init_state_vec;

  // lock and check every robots init state
  std::unique_lock<std::mutex> lock(subscriptions_mutex_);

  for(auto& subscription: subscriptions_){
    std::lock_guard<std::mutex> lock2(subscription.mutex);

    dislam_msgs::GetInitMap srv;
    srv.request.initState = true;

    // Service get map periodically
    if (subscription.init_map_client_.call(srv)){
      PointCloudPtr initSubmap(new PointCloud);
      pcl::fromROSMsg(srv.response.submap, *initSubmap);
      local_map_stack[subscription.robot_id - start_robot_id_] = *initSubmap;
    }else{
      ROS_ERROR("Failed to call service init map");
    }
    init_state_vec.emplace_back(subscription.initState);
  }

  lock.unlock();

  auto end = system_clock::now();
  auto duration = duration_cast<microseconds>(end - start);
  ROS_DEBUG("call service: %lfs", double(duration.count()) * microseconds::period::num / microseconds::period::den);
 
  auto merge_start = system_clock::now();

  // TODO: GEM merge
  for (int i = 0; i < nrRobots; i++) {
    Eigen::Isometry3f T = Eigen::Isometry3f::Identity();
    // if(init_state_vec[i])
    //   T = initPoses[i];
    // else
    T = currentRef[i];
    
    // Add local map into merged map
    Eigen::Matrix4f transformMatrix = T.matrix();
    pcl::transformPointCloud(local_map_stack[i], localCloudUpdated, transformMatrix); 
    local_maps += localCloudUpdated;
    
    if(mapNeedsToBeCorrected){
      merged_pointcloud.clear();
      // Transform and reorganize global map stack
      // for (int j = 0; j < optMapTF[i].size(); j++) {
      //   Eigen::Isometry3f T = optMapTF[i][j];
      //   Eigen::Matrix4f transformMatrix = T.matrix();
      //   pcl::transformPointCloud(global_map_stack[i][j], globalCloudUpdated, transformMatrix); 
      //   globalClouds += globalCloudUpdated;
      // }

      PointCloudI Keyframe;
      for(auto& subscription: subscriptions_){
        std::lock_guard<std::mutex> lock2(subscription.mutex);

        int numberOfCores = 16;
        int SKIP_FRAMES = 2; // sparse map visulalization to save computations 
        #pragma omp parallel for num_threads(numberOfCores)
        for(int k = 1; k < subscription.keyframes.size(); k+=SKIP_FRAMES){
          Eigen::Isometry3f T = optMapTF[subscription.robot_id - start_robot_id_][k] * originMapTF[subscription.robot_id - start_robot_id_][k];
          Eigen::Matrix4f transformMatrix = T.matrix();
          PointCloudI cloud = *subscription.keyframes[k-1];
          pcl::transformPointCloud(cloud, Keyframe, transformMatrix); 
          merged_pointcloud += Keyframe;
        }
      }

    }else if(merged_map_size[i] != optMapTF[i].size()){
      // Eigen::Isometry3f T = optMapTF[i].back();
      // Eigen::Matrix4f transformMatrix = T.matrix();
      // pcl::transformPointCloud(global_map_stack[i].back(), globalCloudUpdated, transformMatrix); 
      // merged_map += globalCloudUpdated;
      // merged_map_size[i] ++;

      PointCloudI Keyframe;
      for(auto& subscription: subscriptions_){
        if(subscription.keyframes.empty())
          continue;
        std::lock_guard<std::mutex> lock2(subscription.mutex);
        Eigen::Isometry3f T = optMapTF[subscription.robot_id - start_robot_id_].back() * originMapTF[subscription.robot_id - start_robot_id_].back();
        Eigen::Matrix4f transformMatrix = T.matrix();
        pcl::transformPointCloud(*subscription.keyframes.back(), Keyframe, transformMatrix); 
        merged_pointcloud += Keyframe;
      }
    }
  }

  pcl::VoxelGrid<PointTI> voxel;
  voxel.setInputCloud (merged_pointcloud.makeShared());
  voxel.setLeafSize (0.5, 0.5, 0.5);
  voxel.filter (merged_pointcloud);

  pcl::PassThrough<PointT> pass;
  pass.setInputCloud (local_maps.makeShared()); 
  pass.setFilterFieldName ("z");
  pass.setFilterLimits (-8.0, 5.0);
  pass.filter (local_maps);

  auto merge_end = system_clock::now();
  auto merge_duration = duration_cast<microseconds>(merge_end - merge_start);
  ROS_DEBUG("merge map: %lfs", double(merge_duration.count()) * microseconds::period::num / microseconds::period::den);
 
  ROS_DEBUG("compose Maps done.");

  return globalClouds;
}


/*
 * Composing maps according to computed transforms
 */
void GlobalManager::mapComposing()
{
  ROS_DEBUG("Map Composing started.");
  if (global_map_stack.size() != nrRobots || originMapTF.empty() || aLoopIsClosed){
    ROS_DEBUG("global_map_stack size: %d, originMapTF.empty(): %d", global_map_stack.size(), originMapTF.empty());
    return;
  }

  std::lock_guard<std::mutex> mergedMaplock(merged_map_mutex);

  // Get merged map
  if(mapNeedsToBeCorrected){
    merged_map.clear();
    // PointCloudPtr cloud(new PointCloud);
    merged_map = composeGlobalMap();
  }else{
    composeGlobalMap();
  }

  // auto filter_start = system_clock::now();

  // // Remove ground points
  // pcl::PassThrough<PointT> pass;
  // pass.setInputCloud (merged_map.makeShared()); 
  // pass.setFilterFieldName ("z");
  // pass.setFilterLimits (-8.0, 5.0);
  // pass.filter (merged_map);

  // auto filter_end = system_clock::now();
  // auto filter_duration = duration_cast<microseconds>(filter_end - filter_start);
  // ROS_INFO("filter map: %lfs", double(filter_duration.count()) * microseconds::period::num / microseconds::period::den);
 
  // pcl::VoxelGrid<PointT> voxel;
  // // Add keyframe to robot handle
  // voxel.setInputCloud (merged_map.makeShared());
  // voxel.setLeafSize (0.2, 0.2, 0.2);
  // voxel.filter (merged_map);

  // if (merged_map.empty()) 
  //   return;

  // pcl::StatisticalOutlierRemoval<PointT>sor;
  // sor.setInputCloud(cloud);
  // sor.setMeanK(50);
  // sor.setStddevMulThresh(0.5);
  // sor.filter(merged_map);

  ROS_DEBUG("Map Composing finished.");
}


/*
 * Publish merged map
 */
void GlobalManager::publishMergedMap()
{
  auto start = system_clock::now();

  // std::lock_guard<std::mutex> mergedMaplock(merged_map_mutex);

  sensor_msgs::PointCloud2 output;
  pcl::toROSMsg(merged_map+local_maps, output);
  output.header.frame_id = global_map_frame_;
  merged_elevation_map_publisher_.publish(output);
  
  auto end = system_clock::now();
  auto duration = duration_cast<microseconds>(end - start);
  ROS_DEBUG("publishMergedMap: %lfs", double(duration.count()) * microseconds::period::num / microseconds::period::den);
   
}


/*
 * Publish merged point cloud
 */
void GlobalManager::publishMergedPointcloud()
{
  auto start = system_clock::now();

  // std::lock_guard<std::mutex> mergedMaplock(merged_map_mutex);

  sensor_msgs::PointCloud2 output;
  pcl::toROSMsg(merged_pointcloud, output);
  output.header.frame_id = global_map_frame_;
  merged_pointcloud_publisher_.publish(output);
  
  auto end = system_clock::now();
  auto duration = duration_cast<microseconds>(end - start);
  ROS_DEBUG("publishMergedPointcloud: %lfs", double(duration.count()) * microseconds::period::num / microseconds::period::den);
   
}


/*
 * Get robot name from topic name
 */
std::string GlobalManager::robotNameFromTopic(const std::string& topic)
{
  return ros::names::parentNamespace(topic);
}


/*
 * Identifies map topic via suffix
 */
bool GlobalManager::isRobotMapTopic(const ros::master::TopicInfo& topic)
{
  /* test whether topic is robot_submap_topic_ */
  std::string topic_namespace = ros::names::parentNamespace(topic.name);
  bool is_map_topic =
      ros::names::append(topic_namespace, robot_submap_topic_) == topic.name;

  /* test whether topic contains *anywhere* robot namespace */
  auto pos = topic.name.find(robot_namespace_);
  bool contains_robot_namespace = pos != std::string::npos;

  /* we support submap and pointcloud2 as maps */
  bool is_occupancy_grid = topic.datatype == "dislam_msgs/SubMap";

  /* we don't want to subcribe on published merged map */
  bool is_our_topic = merged_elevation_map_publisher_.getTopic() == topic.name;

  return is_occupancy_grid && !is_our_topic && contains_robot_namespace &&
         is_map_topic;
}


/*
 * Identify disco topic via suffix
 */
bool GlobalManager::isRobotDiSCOTopic(const ros::master::TopicInfo& topic)
{
  /* test whether topic is robot_disco_topic_ */
  std::string topic_namespace = ros::names::parentNamespace(topic.name);
  bool is_map_topic =
      ros::names::append(topic_namespace, robot_disco_topic_) == topic.name;

  /* test whether topic contains *anywhere* robot namespace */
  auto pos = topic.name.find(robot_namespace_);
  bool contains_robot_namespace = pos != std::string::npos;

  /* we support only DiSCO as descriptors */
  bool is_occupancy_grid = topic.datatype == "dislam_msgs/DiSCO";

  return is_occupancy_grid && contains_robot_namespace && is_map_topic;
}


/*
 * Publish transform /map to /robotid/map
 */
void GlobalManager::publishTF()
{
  // no locking. tf_transforms_ is accessed only from tf thread
  // subscriptions_ is a forward_list, need to be sorted
  std::vector<int> robotIDSortedIndex = sort_indexes(robotIDStack);

  for (int i = 0; i < nrRobots; i++) {
    int robotid = i;
    int queryID = robotIDSortedIndex[i];

    geometry_msgs::TransformStamped tf_transform;
    tf_transform.header.stamp = ros::Time::now();
    tf_transform.header.frame_id = "/map";
    tf_transform.child_frame_id = robotNameVec[queryID] + "/odom";

    if (robotid < mapTF.size()){
      tf_transform.transform.translation.x = mapTF[robotid].translation().x();
      tf_transform.transform.translation.y = mapTF[robotid].translation().y();
      tf_transform.transform.translation.z = mapTF[robotid].translation().z();
      tf_transform.transform.rotation.x = Pose3(((Eigen::Isometry3d)mapTF[robotid]).matrix()).rotation().toQuaternion().x();
      tf_transform.transform.rotation.y = Pose3(((Eigen::Isometry3d)mapTF[robotid]).matrix()).rotation().toQuaternion().y();
      tf_transform.transform.rotation.z = Pose3(((Eigen::Isometry3d)mapTF[robotid]).matrix()).rotation().toQuaternion().z();
      tf_transform.transform.rotation.w = Pose3(((Eigen::Isometry3d)mapTF[robotid]).matrix()).rotation().toQuaternion().w();
    }else{
      tf_transform.transform.translation.x = initPoses[robotid].translation().x();
      tf_transform.transform.translation.y = initPoses[robotid].translation().y();
      tf_transform.transform.translation.z = initPoses[robotid].translation().z();
      tf_transform.transform.rotation.x = Pose3(((Eigen::Isometry3d)initPoses[robotid]).matrix()).rotation().toQuaternion().x();
      tf_transform.transform.rotation.y = Pose3(((Eigen::Isometry3d)initPoses[robotid]).matrix()).rotation().toQuaternion().y();
      tf_transform.transform.rotation.z = Pose3(((Eigen::Isometry3d)initPoses[robotid]).matrix()).rotation().toQuaternion().z();
      tf_transform.transform.rotation.w = Pose3(((Eigen::Isometry3d)initPoses[robotid]).matrix()).rotation().toQuaternion().w();
    }
    tf_publisher_.sendTransform(tf_transform);
  }
}


/*
 * Publish pose graph for visualization
 */
void GlobalManager::publishPoseGraph()
{
  // Trajectory vertex visualization
  for(int i = 0; i < nrRobots; i ++){
    trajMarker_.points = trajPointsVec_[i];
    trajMarker_.id = i;

    // Assign trajectory color
    if(i == 0){
      trajMarker_.color.r = 1.0f;
      trajMarker_.color.g = 1.0f;
      trajMarker_.color.b = 0.0f;
    }else if(i == 1){
      trajMarker_.color.r = 0.0f;
      trajMarker_.color.g = 1.0f;
      trajMarker_.color.b = 0.0f;
    }else if(i == 2){
      trajMarker_.color.r = 1.0f;
      trajMarker_.color.g = 0.0f;
      trajMarker_.color.b = 0.0f;
    }
    trajPoints.markers.push_back(trajMarker_);
  }

  // Loop edge visualization
  for(int i = 0; i < nrRobots; i++){
    loopEdgeMarker_.points = loopEdgesVec_[i];
    loopEdgeMarker_.color.r = 0.0f;
    loopEdgeMarker_.color.g = 0.0f;
    loopEdgeMarker_.color.b = 0.0f;
    loopEdgeMarker_.id = i;
    loopEdges.markers.push_back(loopEdgeMarker_);
  }

  pose_graph_publisher_.publish(trajPoints);
  pose_graph_publisher_.publish(loopEdges);

}


///////////////////// Utility Functions /////////////////////
/*
 * Utility function: read all robots' init poses
 */
void readConfigs(std::vector<string>& files, std::vector<Eigen::Isometry3f>& poses, int start_robot_id_)
{
  ROS_INFO("Read init poses");

  // Initialize poses vector
  for(int i = 0; i < files.size(); i++){
    Eigen::Isometry3f TinitPose = Eigen::Isometry3f::Identity();
    poses.push_back(TinitPose);
  }

  // Iterate files
  for(int i = 0; i < files.size(); i++){
    string path = files[i];
    string::size_type iPos = path.find_last_of('/') + 1;
    string filename = path.substr(iPos, path.length() - iPos);
    string name = filename.substr(0, filename.rfind("."));  // robot name
    std::string robotID = std::regex_replace(name, std::regex("[^0-9]*([0-9]+).*"), std::string("$1"));

    cv::Mat initPose;
    Eigen::Isometry3f TinitPose;

    // Read init pose from yaml
    cv::FileStorage fsSettings(path, cv::FileStorage::READ);
    if(!fsSettings.isOpened())
      cerr << "ERROR: Wrong path to init pose config" << endl;
    
    fsSettings["T.initPose"] >> initPose;
    TinitPose = toIsometry3f(initPose);

    poses[std::stoi(robotID) - start_robot_id_] = TinitPose;
    cout << "Init pose of robot " << std::stoi(robotID) << endl;
    cout << TinitPose.matrix() << endl;
  }
}


/*
 * Utility function: convert cv::Mat to Eigen::Isometry3f
 */
Eigen::Isometry3f toIsometry3f(const cv::Mat &cvMat)
{
    Eigen::Isometry3f M;

    for(int i = 0; i < 4; i++){
      for(int j = 0; j < 4; j++){
        M(i,j) = (float)cvMat.at<double>(i,j);
      }
    }

    return M;
}


/*
 * Utility function: get all filenames in a dir
 */
void getFileNames(string path, vector<string>& filenames)
{
  DIR *pDir;
  struct dirent* ptr;
  if(!(pDir = opendir(path.c_str())))
    return;

  while((ptr = readdir(pDir))!=0) {
    if(strcmp(ptr->d_name, ".") != 0 && strcmp(ptr->d_name, "..") != 0 && path.back() != '/'){
      filenames.push_back(path + "/" + ptr->d_name);
    }else if(strcmp(ptr->d_name, ".") != 0 && strcmp(ptr->d_name, "..") != 0){
      filenames.push_back(path + ptr->d_name);
    }
  }
  closedir(pDir);
}


/*
 * Utility function: transform point cloud
 */
pcl::PointCloud<pcl::PointXYZI>::Ptr transformPointCloud(pcl::PointCloud<pcl::PointXYZI>::Ptr cloudIn, PointTypePose* transformIn)
{
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloudOut(new pcl::PointCloud<pcl::PointXYZI>());

  pcl::PointXYZI *pointFrom;
  pcl::PointXYZI pointTo;

  int cloudSize = cloudIn->points.size();
  cloudOut->resize(cloudSize);
  
  for (int i = 0; i < cloudSize; ++i){

      pointFrom = &cloudIn->points[i];
      float x1 = cos(transformIn->yaw) * pointFrom->x - sin(transformIn->yaw) * pointFrom->y;
      float y1 = sin(transformIn->yaw) * pointFrom->x + cos(transformIn->yaw)* pointFrom->y;
      float z1 = pointFrom->z;

      float x2 = x1;
      float y2 = cos(transformIn->roll) * y1 - sin(transformIn->roll) * z1;
      float z2 = sin(transformIn->roll) * y1 + cos(transformIn->roll)* z1;

      pointTo.x = cos(transformIn->pitch) * x2 + sin(transformIn->pitch) * z2 + transformIn->x;
      pointTo.y = y2 + transformIn->y;
      pointTo.z = -sin(transformIn->pitch) * x2 + cos(transformIn->pitch) * z2 + transformIn->z;
      pointTo.intensity = pointFrom->intensity;

      cloudOut->points[i] = pointTo;
  }
  return cloudOut;
}


/*
 * Utility function: transform robotid to Key in distributed mapper
 */
Key GlobalManager::robotID2Key(int robotid)
{
  int char_a = 97;
  static const size_t keyBits = sizeof(uint64_t) * 8;
  static const size_t chrBits = sizeof(unsigned char) * 8;
  static const size_t indexBits = keyBits - chrBits;
  Key outkey = char(char_a + robotid);
  return outkey << indexBits;
}


/*
 * Utility function: transform Key in distributed mapper to robotid
 */
int GlobalManager::Key2robotID(Key robotKey)
{
  int char_a = 97;
  static const size_t keyBits = sizeof(uint64_t) * 8;
  static const size_t chrBits = sizeof(unsigned char) * 8;
  static const size_t indexBits = keyBits - chrBits;
  char total = robotKey >> indexBits;
  return total - char_a;
}


/*
 * Utility function: from quaternion to euler angles
 */
Eigen::Quaternionf
euler2Quaternion( const double roll,
                  const double pitch,
                  const double yaw )
{
    Eigen::AngleAxisf rollAngle(roll, Eigen::Vector3f::UnitX());
    Eigen::AngleAxisf pitchAngle(pitch, Eigen::Vector3f::UnitY());
    Eigen::AngleAxisf yawAngle(yaw, Eigen::Vector3f::UnitZ());

    Eigen::Quaternionf q = yawAngle * pitchAngle * rollAngle;
    return q;
}


/*
 * Utility function: check if a matrix is a rotation matrix
 */
bool isRotationMatrix(Eigen::Matrix4f R)
{
  double err = 1e-6;
  Eigen::Matrix4f shouldIdentity;
  shouldIdentity = R*R.transpose();
  Eigen::Matrix4f I = Eigen::Matrix4f::Identity();
  return (shouldIdentity - I).norm() < err;
}


/*
 * Utility function: transform rotation matrix to euler angles
 */
Eigen::Vector3d rotationMatrixToEulerAngles(Eigen::Matrix4f &R)
{
  // assert(isRotationMatrix(R));
  double sy = sqrt(R(0,0) * R(0,0) + R(1,0) * R(1,0));
  bool singular = sy < 1e-6;
  double x, y, z;
  if(!singular){
    x = atan2( R(2,1), R(2,2) );
    y = atan2( -R(2,0), sy );
    z = atan2( R(1,0), R(0,0) );
  }else{
    x = atan2( -R(1,2), R(1,1) );
    y = atan2( -R(2,0), sy );
    z = 0; 
  }
  return {x, y, z};
}


/*
 * Utility function: transform Pose3 type in gtsam to isometry in Eigen
 */
Eigen::Isometry3f Pose3toIsometry(Pose3 T)
{
  Eigen::Quaternionf q(T.rotation().toQuaternion().w(), T.rotation().toQuaternion().x(), T.rotation().toQuaternion().y(), T.rotation().toQuaternion().z());
  Eigen::Isometry3f eigenPose(q);
  eigenPose.pretranslate(Eigen::Vector3f(T.translation().x(), T.translation().y(), T.translation().z()));
  return eigenPose;
}


/*
 * Utility function: add normals to the input point cloud
 */
void addNormal(pcl::PointCloud<PointTI>::Ptr cloud, pcl::PointCloud<PointTIN>::Ptr cloudWithNormals)
{
  pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>);

  pcl::search::KdTree<PointTI>::Ptr searchTree (new pcl::search::KdTree<PointTI>);
  searchTree->setInputCloud(cloud);

  pcl::NormalEstimation<PointTI, pcl::Normal> normalEstimator;
  normalEstimator.setInputCloud(cloud);
  normalEstimator.setSearchMethod(searchTree);
  normalEstimator.setKSearch(15);
  normalEstimator.compute(*normals);

  pcl::concatenateFields(*cloud, *normals, *cloudWithNormals);
}


/*
 * Utility function: sort with indexes keep
 */
std::vector<int> sort_indexes(const std::vector<int> v) 
{
  // initialize original index locations
  std::vector<int> idx(v.size());
  iota(idx.begin(), idx.end(), 0);

  // sort indexes based on comparing values in v
  // using std::stable_sort instead of std::sort
  // to avoid unnecessary index re-orderings
  // when v contains elements of equal values 
  stable_sort(idx.begin(), idx.end(),
       [&v](int i1, int i2) {return v[i1] < v[i2];});

  return idx;
}


/*
 * Utility function: calculate relative orientation between two DiSCO
 */
float GlobalManager::calcRelOri(DiSCOFFT newDiSCO, DiSCOFFT oldDiSCO)
{
  double eps = 1e-10;

  std::vector<float> real_a = newDiSCO.first;
  std::vector<float> real_b = oldDiSCO.first;
  std::vector<float> imag_a = newDiSCO.second;
  std::vector<float> imag_b = oldDiSCO.second;

  int width = disco_width_;
  int height = disco_height_;

  fftw_complex *cross_img = (fftw_complex*) fftw_malloc(sizeof(fftw_complex)*height*width);
  double r0 = 0.0;
  for(int i = 0; i < real_a.size(); i++){
    cross_img[i][0] = real_a[i] * real_b[i] + imag_a[i] * imag_b[i];
    cross_img[i][1] = real_a[i] * imag_b[i] + real_b[i] * imag_a[i];
    r0 += sqrt(real_a[i] * real_a[i] + imag_a[i] * imag_a[i] + eps) * sqrt(real_b[i] * real_b[i] + imag_b[i] * imag_b[i] + eps);
    cross_img[i][0] / (r0 + eps);
    cross_img[i][0] / (r0 + eps);
  }

  fftw_plan cross_backward_plan = fftw_plan_dft_2d(height, width, cross_img, cross_img,
                                                    FFTW_BACKWARD, FFTW_ESTIMATE);
  fftw_execute(cross_backward_plan);
  fftw_destroy_plan(cross_backward_plan);
  Eigen::VectorXf cross_real = Eigen::VectorXf::Zero(height*width);
  for (int i= 0; i < height*width; i++)
  {
    cross_real(i) = cross_img[i][0];
  }

  std::ptrdiff_t max_loc;
  float unuse = cross_real.maxCoeff(&max_loc);

  int height_offset = floor(((int) max_loc)/ width);
  int width_offset = (int)max_loc - width * height_offset;

  // if (width_offset > 0.5 * width)
  //   width_offset = width_offset - width;

  float relAngle = width_offset * 3.0;
  return relAngle;
}


void swap(std::complex<float> *v1, std::complex<float> *v2)
{
  std::complex<float> tmp = *v1;
  *v1 = *v2;
  *v2 = tmp;
}


void fftshift(std::complex<float> *data, int count)
{
  int k = 0;
  int c = (int) floor((float)count/2);
  // For odd and for even numbers of element use different algorithm
  if (count % 2 == 0)
  {
    for (k = 0; k < c; k++)
      swap(&data[k], &data[k+c]);
  }
  else
  {
    std::complex<float> tmp = data[0];
    for (k = 0; k < c; k++)
    {
      data[k] = data[c + k + 1];
      data[c + k + 1] = data[k + 1];
    }
    data[c] = tmp;
  }
}


void ifftshift(std::complex<float> *data, int count)
{
  int k = 0;
  int c = (int) floor((float)count/2);
  if (count % 2 == 0)
  {
    for (k = 0; k < c; k++)
      swap(&data[k], &data[k+c]);
  }
  else
  {
    std::complex<float> tmp = data[count - 1];
    for (k = c-1; k >= 0; k--)
    {
      data[c + k + 1] = data[k];
      data[k] = data[c + k];
    }
    data[c] = tmp;
  }
}

}  // namespace global_manager

