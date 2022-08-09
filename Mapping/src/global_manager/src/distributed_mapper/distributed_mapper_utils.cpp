// Copyright (C) 2019 by Pierre-Yves Lajoie <lajoie.py@gmail.com>

#include "distributed_mapper/distributed_mapper_utils.h"

namespace distributed_mapper{

#define USE_STOPPING_CONDITION 1
#define PUSH_SINGLE_SUBGRAPH 0

std::vector<size_t>
orderRobots(const std::vector< boost::shared_ptr<DistributedMapper> >& dist_mappers,
            const size_t& nr_robots,
            const std::string& robot_names,
            const bool& use_flagged_init,
            const bool& use_landmarks){
  std::vector<size_t> ordering;
  ordering.push_back(0); // First one always has the prior

  if(nr_robots > 2){
    if(use_flagged_init){
      // Adjacency matrix is such that adj_matrix(i,j) returns the number of loopclosures connecting robot i to robot j.
      // TODO: Test if the matrix is symmetric with zero diagonal entries
      gtsam::Matrix adj_matrix = gtsam::zeros(nr_robots, nr_robots);
      for(size_t robot_i = 0; robot_i < nr_robots; robot_i++){
        gtsam::Values neighbors = dist_mappers[robot_i]->neighbors(); // Get neighboring values
        for(const gtsam::Values::ConstKeyValuePair& key_value: neighbors){
          gtsam::Key key = key_value.key;
          char symbol = gtsam::symbolChr(key);
          if(use_landmarks) symbol=tolower(symbol); // for communication links between two landmarks
          size_t robot_j = robot_names.find(symbol);
          adj_matrix(robot_i, robot_j) = adj_matrix(robot_i, robot_j) +1;
        }
      }

      std::multimap<int,int> adjacency_map; // <num_loopclosures, subgraph_id>
      std::multimap<int,int>::reverse_iterator adjacency_map_iterator; // from large to small

      while(1){
        adjacency_map.clear();
        for(size_t robot_i = 0; robot_i < nr_robots; robot_i++){
          if(std::find(ordering.begin(), ordering.end(), robot_i)==ordering.end()){
            // for each robot not in the ordering, compute the number of edges
            // towards robots inside the ordering and select the one with the largest number
            int robot_i_nr_anchors = 0;
            for(size_t robot_j: ordering){
              robot_i_nr_anchors += adj_matrix(robot_i, robot_j);
            }
            if(robot_i_nr_anchors != 0){
              adjacency_map.insert(std::pair<int, int>(robot_i_nr_anchors, robot_i));
            }
          }
        }
        if(adjacency_map.size() == 0)
          break;

#if PUSH_SINGLE_SUBGRAPH == 1
        ordering.push_back((*(adjacency_map.rbegin())).second);

#else
        //   std::cout << std::endl << "--------------------------" << std::endl;
        for(adjacency_map_iterator = adjacency_map.rbegin();
            adjacency_map_iterator != adjacency_map.rend(); ++adjacency_map_iterator){
          //     std::cout << (*adjacency_map_iterator).first << " " << (*adjacency_map_iterator).second << std::endl;

          ordering.push_back((*adjacency_map_iterator).second);
        }
        //    std::cout << std::endl << "--------------------------" << std::endl;
#endif

      }

      //  for(size_t i=0; i< ordering.size(); i++){
      //    std::cout << ordering[i] << std::endl;
      //  }
    }
    else{
      for(size_t i =1; i<nr_robots; i++)
        ordering.push_back(i);

      // Randomize just to compare against flagged init
      std::random_shuffle ( ordering.begin(), ordering.end() );

    }
  }
  else{ // 2 robots, trivial ordering
    ordering.push_back(1);
  }

  if(ordering.size() != nr_robots){
    // Can happen in case of flagged initialization if a robot is not connected initially to any of the other robots
    for(size_t robot_i = 0; robot_i < nr_robots; robot_i++){
      if(std::find(ordering.begin(), ordering.end(), robot_i)==ordering.end()){
        ordering.push_back(robot_i);
      }
    }
  }

  return ordering;
}

std::pair<gtsam::Values, gtsam::VectorValues> logrotation_trace(const boost::shared_ptr<DistributedMapper>& dist_mapper_robot){

  gtsam::Values distributed_iter; // For logging
  gtsam::VectorValues distributed_vector_values_iter;

  // Convert to poses for logging
  dist_mapper_robot->convertLinearizedRotationToPoses();
  gtsam::Values current_estimate = dist_mapper_robot->currentEstimate();
  gtsam::Values distributed_robot_i = dist_mapper_robot->getConvertedEstimate(current_estimate);
  for(const gtsam::Values::ConstKeyValuePair& key_value: distributed_robot_i){
    gtsam::Key key = key_value.key;
    distributed_iter.insert(key, distributed_robot_i.at<gtsam::Pose3>(key));
  }
  for(const gtsam::Values::ConstKeyValuePair& key_value: current_estimate){
    gtsam::Key key = key_value.key;

    if(dist_mapper_robot->use_chr_less_full_graph_){
      int symbolIndex = gtsam::symbolIndex(key);
      distributed_vector_values_iter.insert(symbolIndex, dist_mapper_robot->linearizedRotationAt(key));
    }
    else{
      distributed_vector_values_iter.insert(key, dist_mapper_robot->linearizedRotationAt(key));
    }
  }

  return std::make_pair(distributed_iter, distributed_vector_values_iter);
}

void optimizeRotation(std::vector< boost::shared_ptr<DistributedMapper> >& dist_mappers,
                      const size_t& max_iter,
                      const size_t& nr_robots,
                      const std::string& robot_names,
                      const std::vector<size_t>& ordering,
                      const bool& debug,
                      const double& rotation_estimate_change_threshold,
                      const bool& use_landmarks,
                      boost::optional<std::vector<gtsam::Values>&> rotation_trace,
                      boost::optional<std::vector<gtsam::Values>&> subgraph_rotation_trace,
                      boost::optional<std::vector<gtsam::VectorValues>&> rotation_vector_values_trace,
                      boost::optional<gtsam::VectorValues&> rotation_centralized,
                      boost::optional<gtsam::GaussianFactorGraph&> rotation_graph){

  // Compute the centralized rotation error if it is available
  double centralized_error = -1;
  if(rotation_graph){
    centralized_error = rotation_graph->error(*rotation_centralized);
  }

  // Before starting no robot is optimized
  for(size_t robot: ordering){
    dist_mappers[robot]->updateInitialized(false);
    dist_mappers[robot]->clearNeighboringRobotInit();
  }

  if(debug)
    std::cout << "[optimizeRotation] Starting rotation iteration"  << std::endl;


  // Iterations
  for(size_t iter=0; iter < max_iter; iter++){
    gtsam::Values distributed_iter; // For logging
    gtsam::VectorValues distributed_vector_values_iter; // For logging
    for(size_t robot: ordering){   // Iterate each optimizer once // Parallelize this

      if(debug)
        std::cout << "[optimizeRotation] Optimizing robot: " << robot << std::endl;

      gtsam::Values subgraph_iter; // For logging

      // Ask other robots for updated estimates and update it
      for(const gtsam::Values::ConstKeyValuePair& key_value: dist_mappers[robot]->neighbors()){
        gtsam::Key key = key_value.key;

        // dist_mappers only contains the robots that are currently communicating, so we check
        // if the neighbor keys is from one of these robots, if so, we update it (cummunication)
        char symbol = gtsam::symbolChr(key);
        if(use_landmarks)symbol=tolower(symbol); // for communication links between two landmarks
        size_t neighboring_robot_id = robot_names.find(symbol);
        if (neighboring_robot_id != std::string::npos){ // if the robot is actually communicating
          gtsam::Vector rotation_estimate = dist_mappers[neighboring_robot_id]->linearizedRotationAt(key);
          dist_mappers[robot]->updateNeighborLinearizedRotations(key, rotation_estimate); // this requires communication

          bool neighboring_robot_initialized = dist_mappers[neighboring_robot_id]->isRobotInitialized();
          dist_mappers[robot]->updateNeighboringRobotInitialized(symbol, neighboring_robot_initialized); // this requires communication
        }
        else{
          // Robot we are not communicating with are considered as optimized
          dist_mappers[robot]->updateNeighboringRobotInitialized(symbol, true);
        }
      }

      if(debug)
        std::cout << "[optimizeRotation] Queried neighbors" << std::endl;

      if(rotation_trace){
        std::pair<gtsam::Values, gtsam::VectorValues> log = logrotation_trace(dist_mappers[robot]);
        distributed_iter.insert(log.first);
        subgraph_iter.insert(log.first);
        distributed_vector_values_iter.insert(log.second);
      }

      if(debug)
        std::cout << "[optimizeRotation] Estimating rotation"  << std::endl;

      // Iterate
      dist_mappers[robot]->estimateRotation(); // optimization

      if(debug)
        std::cout << "[optimizeRotation] Estimate rotation complete"  << std::endl;

      /*  Distributed Jacobi: update_type_ = postUpdate, gamma = 1
    *  Gauss Seidel: update_type_ = incUpdate, gamma = 1
    *  Jacobi Overrelax: update_type_ = postUpdate, gamma != 1
    *  Succ Overrelax: update_type_ = incUpdate, gamma != 1
    */
      if(dist_mappers[robot]->update_type_ == DistributedMapper::incUpdate){
        dist_mappers[robot]->updateRotation();
      }

      if(debug)
        std::cout << "[optimizeRotation] Update linearized rotation complete"  << std::endl;


      // This robot is initialized
      dist_mappers[robot]->updateInitialized(true);


      // Trace
      if(subgraph_rotation_trace)
        subgraph_rotation_trace->push_back(subgraph_iter);

    }

    if(debug)
      std::cout << "[optimizeRotation] Loop over robots complete" << std::endl;


    // If DistributeJacobi/Jacobi OverRelaxation, we update all the robots at the end of each iteration
    for(size_t robot: ordering){   // Iterate over each robot
      if(dist_mappers[robot]->update_type_ == DistributedMapper::postUpdate){
        dist_mappers[robot]->updateRotation();
      }
    }

    // Trace
    if(rotation_trace){
      rotation_trace->push_back(distributed_iter);
    }

    if(rotation_vector_values_trace){
      rotation_vector_values_trace->push_back(distributed_vector_values_iter);
    }

    // If the centralized rotation is available, use it as a stopping condition (just for analysis)
    // Stop if the distributed estimate is within 5% of centralized estimate
    if(rotation_centralized){

      // If graph error reaches within 10% break
      double distributed_error = -1;
      if(rotation_graph){
        distributed_error = rotation_graph->error(distributed_vector_values_iter);
        double error_change = fabs(distributed_error - centralized_error)*1.0f/centralized_error;
        //std::cout << "Rotation Error Change: " << error_change << std::endl;

        if(error_change < 0.1){
          //  std::cout << "Rotation Breaking" << std::endl;
          break;
        }
      }

      /*
      // if change is less than 50% break
      double changeCentralized = ((*rotation_centralized).subtract(distributed_vector_values_iter).norm())/distributed_vector_values_iter.size();
      std::cout << "Rotation Change: " << changeCentralized << std::endl;
      if(changeCentralized < 0.5){
          std::cout << "Rotation Breaking" << std::endl;
          break;
        }
        */
    }
    else{

#if USE_STOPPING_CONDITION == 1
      //  STOP IF
      // 1- change in the individual cost of each robot is small enough
      // 2- change in the estimate of each robot is small enough
      bool stop = true;
      for(size_t robot = 0; robot < nr_robots; robot++){
        double change = dist_mappers[robot]->latestChange();
        std::cout << "[optimizeRotation] Change (Robot " << robot << "): " << change << std::endl;
      }
      for(size_t robot = 0; robot < nr_robots; robot++){
        double change = dist_mappers[robot]->latestChange();
        // std::cout << "[optimizeRotation] Change (Robot " << robot << "): " << change << std::endl;
        if(change > rotation_estimate_change_threshold){
          stop = false;
          break;
        }
      }

      if(stop && iter != 0){ // Maybe the change is wrong at zero iteration
        break;
      }
#endif
    }

  }
}


void optimizePose(std::vector< boost::shared_ptr<DistributedMapper> >& dist_mappers,
                  const size_t& max_iter,
                  const size_t& nr_robots,
                  const std::string& robot_names,
                  const std::vector<size_t>& ordering,
                  const bool& debug,
                  const double& pose_estimate_change_threshold,
                  const bool& use_landmarks,
                  boost::optional<std::vector<gtsam::Values>&> pose_trace,
                  boost::optional<std::vector<gtsam::Values>&> subgraph_pose_trace,
                  boost::optional<gtsam::Values&> pose_centralized,
                  boost::optional<gtsam::NonlinearFactorGraph&> pose_graph){

  // Compute the centralized pose error if it is available
  double centralized_error = -1;
  if(pose_graph){
    centralized_error = pose_graph->error(*pose_centralized);
  }


  // Before starting no robot is optimized
  for(size_t robot: ordering){
    dist_mappers[robot]->updateInitialized(false);
    dist_mappers[robot]->clearNeighboringRobotInit();
  }

  if(debug)
    std::cout << "[optimizePoses] Starting pose iteration"  << std::endl;


  for(size_t iter=0; iter < max_iter; iter++){
    gtsam::Values distributed_iter; // For logging

    for(size_t robot: ordering){   // Iterate each optimizer once

      gtsam::Values subgraph_iter; // For logging
      gtsam::Values neighbors = dist_mappers[robot]->neighbors();  // Get neighboring values

      // Ask other robots for updated estimates and update it
      for(const gtsam::Values::ConstKeyValuePair& key_value: neighbors){
        gtsam::Key key = key_value.key;

        // dist_mappers only contains the robots that are currently communicating, so we check
        // if the neighbor keys is from one of these robots, if so, we update it (communication)
        char symbol = gtsam::symbolChr(key);
        if(use_landmarks)symbol=tolower(symbol); // for communication links between two landmarks
        size_t neighboring_robot_id = robot_names.find(symbol);
        if (neighboring_robot_id != std::string::npos){ // if the robot is actually communicating
          gtsam::Vector poseEstimate = dist_mappers[neighboring_robot_id]->linearizedPosesAt(key);
          dist_mappers[robot]->updateNeighborLinearizedPoses(key, poseEstimate);
          bool neighboring_robot_initialized = dist_mappers[neighboring_robot_id]->isRobotInitialized();
          dist_mappers[robot]->updateNeighboringRobotInitialized(symbol, neighboring_robot_initialized); // this requires communication
          //std::cout << "Robot " << neighboring_robot_id << " at " << key << ". Est[0]=" << poseEstimate << std::endl;
          //std::cout << "Robot " << neighboring_robot_id << " is init? " << neighboring_robot_initialized << std::endl;
        }
        else{
          // Robot we are not communicating with are considered as optimized
          dist_mappers[robot]->updateNeighboringRobotInitialized(symbol, true);
        }

      }

      if(debug)
        std::cout << "[optimizePoses] Queried neighbors"  << std::endl;

      // Logging
      if(pose_trace){
        // Convert to poses for logging
        gtsam::VectorValues linearized_poses = dist_mappers[robot]->linearizedPoses();
        gtsam::Values current_estimate = dist_mappers[robot]->currentEstimate();
        gtsam::Values retracted_estimate = evaluation_utils::retractPose3Global(current_estimate, linearized_poses);
        gtsam::Values distributed_robot_i = dist_mappers[robot]->getConvertedEstimate(retracted_estimate);
        for(const gtsam::Values::ConstKeyValuePair& key_value: distributed_robot_i){
          gtsam::Key key = key_value.key;
          distributed_iter.insert(key, distributed_robot_i.at<gtsam::Pose3>(key));
          subgraph_iter.insert(key, distributed_robot_i.at<gtsam::Pose3>(key));
        }
      }

      if(debug)
        std::cout << "[optimizePoses] Estimating poses"  << std::endl;

      // Iterate
      dist_mappers[robot]->estimatePoses();

      if(debug)
        std::cout << "[optimizePoses] Estimate poses complete"  << std::endl;


      if(dist_mappers[robot]->update_type_ == DistributedMapper::incUpdate){
        dist_mappers[robot]->updatePoses();
      }

      if(debug)
        std::cout << "[optimizePoses] Update linearized poses complete"  << std::endl;


      // This robot is initialized
      dist_mappers[robot]->updateInitialized(true);

      // Trace
      if(subgraph_pose_trace)
        subgraph_pose_trace->push_back(subgraph_iter);
    }

    if(debug)
      std::cout << "[optimizePoses] Loop over robots complete"  << std::endl;


    // If DistributeJacobi/Jacobi OverRelaxation, we update all the robots at the end of each iteration
    for(size_t robot: ordering){   // Iterate over each robot
      if(dist_mappers[robot]->update_type_ == DistributedMapper::postUpdate){
        dist_mappers[robot]->updatePoses();
      }
    }

    // Trace
    if(pose_trace)
      pose_trace->push_back(distributed_iter);

    // If the centralized pose is available, use it as a stopping condition (just for analysis)
    // Stop if the distributed estimate is within 5% of centralized estimate
    if(pose_centralized){
      // If graph error reaches within 10% break
      double distributed_error = -1;
      if(pose_graph){
        distributed_error = pose_graph->error(distributed_iter);
        double error_change = fabs(distributed_error - centralized_error)*1.0f/centralized_error;
        //std::cout << "Pose Error Change: " << error_change << std::endl;

        if(error_change < 0.1){
          //  std::cout << "Pose Breaking" << std::endl;
          break;
        }
      }

      /*
      double changeCentralized = (pose_centralized->localCoordinates(distributed_iter).norm())/distributed_iter.size();
      std::cout << "Pose Change: " << changeCentralized << std::endl;
      // if change is less than 5% break
      if(changeCentralized < 0.5){
          std::cout << "Pose Breaking" << std::endl;
          break;
        }
        */
    }
    else{
#if USE_STOPPING_CONDITION == 1
      // 2- change in the estimate of each robot is small enough
      bool stop = true;
      for(size_t robot = 0; robot < nr_robots; robot++){     // Iterate each optimizer once
        double change = dist_mappers[robot]->latestChange();
        std::cout << "[optimizePoses] Change (Robot " << robot << "): " << change << std::endl;
      }
      for(size_t robot = 0; robot < nr_robots; robot++){     // Iterate each optimizer once
        double change = dist_mappers[robot]->latestChange();
        //std::cout << "[optimizePoses] Change (Robot " << robot << "): " << change << std::endl;
        if(change > pose_estimate_change_threshold){
          stop = false;
          break;
        }
      }
      if(stop && iter!=0)
        break;
#endif
    }

  }
}


std::vector< gtsam::Values >
distributedOptimizer(std::vector< boost::shared_ptr<DistributedMapper> >& dist_mappers,
                     const size_t& max_iter,
                     int& max_clique_size,
                     const DistributedMapper::UpdateType& update_type,
                     const double& gamma,
                     const double& rotation_estimate_change_threshold,
                     const double& pose_estimate_change_threshold,
                     const bool& use_flagged_init,
                     const bool& use_landmarks,
                     const bool& debug,
                     const bool& contains_odometry,
                     const double& pcm_threshold,
                     const bool& use_covariance,
                     const bool& use_pcm,
                     const bool& use_heuristics,
                     boost::optional<std::vector<gtsam::GraphAndValues>&> graph_and_values_vec,
                     boost::optional<std::vector<gtsam::Values>&> rotation_trace,
                     boost::optional<std::vector<gtsam::Values>&> pose_trace,
                     boost::optional<std::vector<gtsam::Values>&> subgraph_rotation_trace,
                     boost::optional<std::vector<gtsam::Values>&> subgraph_pose_trace,
                     boost::optional<std::vector<gtsam::VectorValues>&> rotation_vector_values_trace,
                     boost::optional<gtsam::VectorValues&> rotation_centralized,
                     boost::optional<gtsam::Values&> pose_centralized,
                     boost::optional<gtsam::NonlinearFactorGraph&> graph_without_prior,
                     boost::optional<gtsam::GaussianFactorGraph&> centralized_rotation_graph){

  size_t nr_robots = dist_mappers.size();
  std::string robot_names = "";
  for(size_t robot = 0; robot < nr_robots; robot++){
    robot_names += dist_mappers[robot]->robotName(); // this is the string containing chars with robot names
    dist_mappers[robot]->setFlaggedInit(use_flagged_init);
    dist_mappers[robot]->setUpdateType(update_type);
    dist_mappers[robot]->setGamma(gamma);
  }

  if (use_pcm && contains_odometry) {
    auto max_clique_info = distributed_pcm::DistributedPCM::solveCentralized(dist_mappers, graph_and_values_vec.get(),
                                                             pcm_threshold, use_covariance, use_heuristics);
    max_clique_size = max_clique_info.first;
  }

  if(debug)
    std::cout << "Starting Optimizer"  << std::endl;

  ////////////////////////////////////////////////////////////////////////////////////////////
  // Flagged Init
  ////////////////////////////////////////////////////////////////////////////////////////////
  std::vector<size_t> ordering = orderRobots(dist_mappers,
                                             nr_robots,
                                             robot_names,
                                             use_flagged_init,
                                             use_landmarks);

  if(debug)
    std::cout << "Ordering complete"  << std::endl;


  ////////////////////////////////////////////////////////////////////////////////////////////
  // Iterate rotation
  ////////////////////////////////////////////////////////////////////////////////////////////

  // Remove key_anchor_ belonging to the prior from the centralized rotation
  if(rotation_centralized){
    if(rotation_centralized->exists(key_anchor_)){
      rotation_centralized->erase(key_anchor_);
    }
  }

  // Optimize it
  optimizeRotation(dist_mappers,
                   max_iter,
                   nr_robots,
                   robot_names,
                   ordering,
                   debug,
                   rotation_estimate_change_threshold,
                   use_landmarks,
                   rotation_trace,
                   subgraph_rotation_trace,
                   rotation_vector_values_trace,
                   rotation_centralized,
                   centralized_rotation_graph);

  if(debug)
    std::cout << "Optimize rotation complete"  << std::endl;


  ////////////////////////////////////////////////////////////////////////////////////////////
  // Initialize poses: this does not require communication (this part essentially computes the
  // linearization point for the following Jacobi iterations)
  ////////////////////////////////////////////////////////////////////////////////////////////
  // Convert to poses and update neighbors (Project to SO3 + add zero translation)
  for(size_t robot = 0; robot < nr_robots; robot++){
    dist_mappers[robot]->convertLinearizedRotationToPoses();
  }
  // Update neighboring values
  for(size_t robot = 0; robot < nr_robots; robot++){
    // Get neighboring values
    gtsam::Values neighbors = dist_mappers[robot]->neighbors();
    // Ask other robots for updated estimates and update it
    for(const gtsam::Values::ConstKeyValuePair& key_value: neighbors){
      gtsam::Key key = key_value.key;
      // pick linear rotation estimate from *robot*
      gtsam::VectorValues lin_rot_estimate_neighbor;
      lin_rot_estimate_neighbor.insert( key,  dist_mappers[robot]->neighborsLinearizedRotationsAt(key) );
      // make a pose out of it
      gtsam::Values rot_estimate_neighbor = gtsam::InitializePose3::normalizeRelaxedRotations(lin_rot_estimate_neighbor);
      gtsam::Values pose_estimate_neighbor = evaluation_utils::pose3WithZeroTranslation(rot_estimate_neighbor);
      // store it
      dist_mappers[robot]->updateNeighbor(key, pose_estimate_neighbor.at<gtsam::Pose3>(key));
    }
  }

  if(debug){
    std::cout << "Converted rotation to poses"  << std::endl;
    for(size_t robot = 0; robot < nr_robots; robot++){
      evaluation_utils::printKeys(dist_mappers[robot]->currentEstimate());
    }
  }


  ////////////////////////////////////////////////////////////////////////////////////////////
  // Iterate poses
  ////////////////////////////////////////////////////////////////////////////////////////////
  if(pose_centralized){
    if(pose_centralized->exists(key_anchor_)){
      pose_centralized->erase(key_anchor_);
    }
  }

  optimizePose(dist_mappers,
               max_iter,
               nr_robots,
               robot_names,
               ordering,
               debug,
               pose_estimate_change_threshold,
               use_landmarks,
               pose_trace,
               subgraph_pose_trace,
               pose_centralized,
               graph_without_prior);

  if(debug)
    std::cout << "Optimized poses"  << std::endl;

  // Do global retract
  for(size_t robot = 0; robot < nr_robots; robot++){
    dist_mappers[robot]->retractPose3Global();
  }

  // Get current estimate
  std::vector< gtsam::Values > estimates(nr_robots);
  for(size_t robot = 0; robot < nr_robots; robot++){
    estimates[robot] = dist_mappers[robot]->currentEstimate();
  }

  return estimates;
}


////////////////////////////////////////////////////////////////////////////////
// Logging
////////////////////////////////////////////////////////////////////////////////
void logResults(const size_t& nr_robots,
                const std::string& trace_file,
                const gtsam::Values& centralized,
                std::vector< boost::shared_ptr<DistributedMapper> >& dist_mappers){

  for(size_t robot = 0; robot < nr_robots; robot++){
    std::pair<std::vector<double>, std::vector<double> > trace = dist_mappers[robot]->trace();
    std::vector<double> rotation_trace = trace.first;
    std::vector<double> pose_trace = trace.second;
    // Dump to a file
    std::string trace_file_robot = trace_file+ "_" + boost::lexical_cast<std::string>(robot) + ".txt";
    std::fstream trace_stream(trace_file_robot.c_str(), std::fstream::out);
    for(size_t i=0; i<rotation_trace.size(); i++){
      trace_stream << rotation_trace[i] << " ";
    }
    trace_stream << "-1" << std::endl;

    for(size_t i=0; i<pose_trace.size(); i++){
      trace_stream << pose_trace[i] << " ";
    }
    trace_stream << "-1" << std::endl;

    // Log centralized estimate error for plotting
    std::pair<double, double> errors = dist_mappers[robot]->logCentralizedError(centralized);
    trace_stream << errors.first << std::endl; // centralized error
    trace_stream << errors.second << std::endl; // distributed error

    // Log the estimate change trace
    std::pair<std::vector<double>, std::vector<double> > change_trace = dist_mappers[robot]->traceEstimateChange();
    std::vector<double> rotation_change_trace = change_trace.first;
    std::vector<double> pose_change_trace = change_trace.second;

    for(size_t i=0; i<rotation_change_trace.size(); i++){
      trace_stream << rotation_change_trace[i] << " ";
    }
    trace_stream << "-1" << std::endl;

    for(size_t i=0; i<pose_change_trace.size(); i++){
      trace_stream << pose_change_trace[i] << " ";
    }
    trace_stream << "-1" << std::endl;
    trace_stream.close();
  }
}

}
