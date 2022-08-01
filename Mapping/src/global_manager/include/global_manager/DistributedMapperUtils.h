#pragma once 

#include <global_manager/DistributedMapper.h>
#include <fstream>
#include <algorithm>
#include <map>
#include <utility>


namespace distributed_mapper{

#define USE_STOPPING_CONDITION 1
#define PUSH_SINGLE_SUBGRAPH 0


/**
 * @brief orderRobots orders the robots using flagged init
 * @param distMappers is the vector of estimators communicating at that moment, only those will be optimized.
 * @param nrRobots is the number of robots
 * @param robotNames is the name of robots
 * @param useFlaggedInit orders using flagged initialization
 * @return
 */
std::vector<size_t>
orderRobots(std::vector< boost::shared_ptr<DistributedMapper> > distMappers,
            size_t nrRobots,
            std::string robotNames,
            bool useFlaggedInit,
            bool useLandmarks = false){
  std::vector<size_t> ordering;
  ordering.push_back(0); // First one always has the prior

  if(nrRobots > 2){
      if(useFlaggedInit){
          // Adjacency matrix is such that adjMatrix(i,j) returns the number of separators connecting robot i to robot j.
          // TODO: Test if the matrix is symmetric with zero diagonal entries
          gtsam::Matrix adjMatrix = gtsam::zeros(nrRobots, nrRobots);
          for(size_t robot_i = 0; robot_i < nrRobots; robot_i++){
              gtsam::Values neighbors = distMappers[robot_i]->neighbors(); // Get neighboring values
              for(const gtsam::Values::ConstKeyValuePair& key_value: neighbors){
                gtsam::Key key = key_value.key;
                char symbol = gtsam::symbolChr(key);
                if(useLandmarks)symbol=tolower(symbol); // for communication links between two landmarks
                size_t robot_j = robotNames.find(symbol);
                adjMatrix(robot_i, robot_j) = adjMatrix(robot_i, robot_j) +1;
              }
            }

          std::multimap<int,int> adjacencyMap; // <num_separators, subgraph_id>
          std::multimap<int,int>::reverse_iterator adjacencyMapIterator; // from large to small

          while(1){
              adjacencyMap.clear();
              for(size_t robot_i = 0; robot_i < nrRobots; robot_i++){
                  if(std::find(ordering.begin(), ordering.end(), robot_i)==ordering.end()){
                      // for each robot not in the ordering, compute the number of edges
                      // towards robots inside the ordering and select the one with the largest number
                      int robot_i_nrAnchors = 0;
                      for(size_t robot_j: ordering){
                        robot_i_nrAnchors += adjMatrix(robot_i, robot_j);
                      }
                      if(robot_i_nrAnchors != 0){
                          adjacencyMap.insert(std::pair<int, int>(robot_i_nrAnchors, robot_i));
                        }
                    }
                }
              if(adjacencyMap.size() == 0)
                break;

#if PUSH_SINGLE_SUBGRAPH == 1
              ordering.push_back((*(adjacencyMap.rbegin())).second);

#else
              //   std::cout << std::endl << "--------------------------" << std::endl;
              for(adjacencyMapIterator = adjacencyMap.rbegin();
                  adjacencyMapIterator != adjacencyMap.rend(); ++adjacencyMapIterator){
                  //     std::cout << (*adjacencyMapIterator).first << " " << (*adjacencyMapIterator).second << std::endl;

                  ordering.push_back((*adjacencyMapIterator).second);
                }
              //    std::cout << std::endl << "--------------------------" << std::endl;
#endif

            }

          //  for(size_t i=0; i< ordering.size(); i++){
          //    std::cout << ordering[i] << std::endl;
          //  }
        }
      else{
          for(size_t i =1; i<nrRobots; i++)
            ordering.push_back(i);

          // Randomize just to compare against flagged init
          std::random_shuffle ( ordering.begin(), ordering.end() );

        }
    }
  else{ // 2 robots, trivial ordering
      ordering.push_back(1);
    }

  if(ordering.size() != nrRobots){
      // Can happen in case of flagged initialization if a robot is not connected initially to any of the other robots
      for(size_t robot_i = 0; robot_i < nrRobots; robot_i++){
          if(std::find(ordering.begin(), ordering.end(), robot_i)==ordering.end()){
              ordering.push_back(robot_i);
            }
        }
    }

  return ordering;
}

/**
 * @brief logRotationTrace
 * @param distMapper_robot
 * @param distributed_vectorvalues_iter
 * @param distributed_iter
 * @param subgraph_iter
 */
std::pair<gtsam::Values, gtsam::VectorValues> logRotationTrace(boost::shared_ptr<DistributedMapper> distMapper_robot){

  gtsam::Values distributed_iter; // For logging
  gtsam::VectorValues distributed_vectorvalues_iter;

  // Convert to poses for logging
  distMapper_robot->convertLinearizedRotationToPoses();
  gtsam::Values current_estimate = distMapper_robot->currentEstimate();
  gtsam::Values distributed_robot_i = distMapper_robot->getConvertedEstimate(current_estimate);
  for(const gtsam::Values::ConstKeyValuePair& key_value: distributed_robot_i){
    gtsam::Key key = key_value.key;
    distributed_iter.insert(key, distributed_robot_i.at<gtsam::Pose3>(key));
  }
  for(const gtsam::Values::ConstKeyValuePair& key_value: current_estimate){
    gtsam::Key key = key_value.key;

    if(distMapper_robot->useChrLessFullGraph_){
        int symbolIndex = gtsam::symbolIndex(key);
        distributed_vectorvalues_iter.insert(symbolIndex, distMapper_robot->linearizedRotationAt(key));
      }
    else{
        distributed_vectorvalues_iter.insert(key, distMapper_robot->linearizedRotationAt(key));
      }
  }

  return std::make_pair(distributed_iter, distributed_vectorvalues_iter);
}


/**
 * @brief optimizeRotation optimizes the rotation in a distributed sense
 * @param distMappers is the vector of estimators communicating at that moment, only those will be optimized.
 * @param maxIter is the maximum number of iteratinos
 * @param nrRobots is the number of robots
 * @param robotNames is the name of robots
 * @param ordering is the prioritized ordering
 * @param rotationEstimateChangeThreshold provides an early stopping condition
 * @param rotationTrace contains the overall converted estimate at any iteration
 * @param subgraphRotationTrace contains the Values for a particular subgraph being optimized
 * @param rotationVectorValuesTrace contains the linearized rotation estimate
 */
void optimizeRotation(std::vector< boost::shared_ptr<DistributedMapper> > distMappers,
                      size_t maxIter,
                      size_t nrRobots,
                      std::string robotNames,
                      std::vector<size_t> ordering,
                      bool debug,
                      double rotationEstimateChangeThreshold,
                      bool useLandmarks = false,
                      boost::optional<std::vector<gtsam::Values>&> rotationTrace = boost::none,
                      boost::optional<std::vector<gtsam::Values>&> subgraphRotationTrace = boost::none,
                      boost::optional<std::vector<gtsam::VectorValues>&> rotationVectorValuesTrace = boost::none,
                      boost::optional<gtsam::VectorValues&> rotationCentralized = boost::none,
                      boost::optional<gtsam::GaussianFactorGraph&> rotationGraph = boost::none){

  // Compute the centralized rotation error if it is available
  double centralizedError = -1;
  if(rotationGraph){
      centralizedError = rotationGraph->error(*rotationCentralized);
    }

  // Before starting no robot is optimized
  for(size_t robot: ordering){
    distMappers[robot]->updateInitialized(false);
    distMappers[robot]->clearNeighboringRobotInit();
  }

  if(debug)
    std::cout << "[optimizeRotation] Starting rotation iteration"  << std::endl;


  // Iterations
  for(size_t iter=0; iter < maxIter; iter++){
      gtsam::Values distributed_iter; // For logging
      gtsam::VectorValues distributed_vectorvalues_iter; // For logging
      for(size_t robot: ordering){   // Iterate each optimizer once // Parallelize this

        if(debug)
          std::cout << "[optimizeRotation] Optimizing robot: " << robot << " Total robots: " << ordering.size() << std::endl;

        gtsam::Values subgraph_iter; // For logging

        // Ask other robots for updated estimates and update it
        for(const gtsam::Values::ConstKeyValuePair& key_value: distMappers[robot]->neighbors()){
          gtsam::Key key = key_value.key;

          // distMappers only contains the robots that are currently communicating, so we check
          // if the neighbor keys is from one of these robots, if so, we update it (cummunication)
          char symbol = gtsam::symbolChr(key);
          if(useLandmarks)symbol=tolower(symbol); // for communication links between two landmarks
          size_t neighboringRobotId = robotNames.find(symbol);
          if (neighboringRobotId != std::string::npos){ // if the robot is actually communicating
              gtsam::Vector rotationEstimate = distMappers[neighboringRobotId]->linearizedRotationAt(key);
              distMappers[robot]->updateNeighborLinearizedRotations(key, rotationEstimate); // this requires communication

              bool neighboringRobotInitialized = distMappers[neighboringRobotId]->isRobotInitialized();
              distMappers[robot]->updateNeighboringRobotInitialized(symbol, neighboringRobotInitialized); // this requires communication
            }
          else{
              // Robot we are not communicating with are considered as optimized
              distMappers[robot]->updateNeighboringRobotInitialized(symbol, true);
            }
        }

        if(debug)
          std::cout << "[optimizeRotation] Queried neighbors" << std::endl;

        if(rotationTrace){
            std::pair<gtsam::Values, gtsam::VectorValues> log = logRotationTrace(distMappers[robot]);
            distributed_iter.insert(log.first);
            subgraph_iter.insert(log.first);
            distributed_vectorvalues_iter.insert(log.second);
          }

        if(debug)
          std::cout << "[optimizeRotation] Estimating rotation"  << std::endl;

        // Iterate
        distMappers[robot]->estimateRotation(); // optimization

        if(debug)
          std::cout << "[optimizeRotation] Estimate rotation complete"  << std::endl;

        /*  Distributed Jacobi: updateType_ = postUpdate, gamma = 1
      *  Gauss Seidel: updateType_ = incUpdate, gamma = 1
      *  Jacobi Overrelax: updateType_ = postUpdate, gamma != 1
      *  Succ Overrelax: updateType_ = incUpdate, gamma != 1
      */
        if(distMappers[robot]->updateType_ == DistributedMapper::incUpdate){
            distMappers[robot]->updateRotation();
          }

        if(debug)
          std::cout << "[optimizeRotation] Update linearized rotation complete"  << std::endl;


        // This robot is initialized
        distMappers[robot]->updateInitialized(true);


        // Trace
        if(subgraphRotationTrace)
          subgraphRotationTrace->push_back(subgraph_iter);

      }

      if(debug)
        std::cout << "[optimizeRotation] Loop over robots complete" << std::endl;


      // If DistributeJacobi/Jacobi OverRelaxation, we update all the robots at the end of each iteration
      for(size_t robot: ordering){   // Iterate over each robot
        if(distMappers[robot]->updateType_ == DistributedMapper::postUpdate){
            distMappers[robot]->updateRotation();
          }
      }

      // Trace
      if(rotationTrace){
          rotationTrace->push_back(distributed_iter);
        }

      if(rotationVectorValuesTrace){
          rotationVectorValuesTrace->push_back(distributed_vectorvalues_iter);
        }

      // If the centralized rotation is available, use it as a stopping condition (just for analysis)
      // Stop if the distributed estimate is within 5% of centralized estimate
      if(rotationCentralized){

          // If graph error reaches within 10% break
          double distributedError = -1;
          if(rotationGraph){
              distributedError = rotationGraph->error(distributed_vectorvalues_iter);
              double errorChange = fabs(distributedError - centralizedError)*1.0f/centralizedError;
              //std::cout << "Rotation Error Change: " << errorChange << std::endl;

              if(errorChange < 0.1){
                  //  std::cout << "Rotation Breaking" << std::endl;
                  break;
                }
            }

          /*
          // if change is less than 50% break
          double changeCentralized = ((*rotationCentralized).subtract(distributed_vectorvalues_iter).norm())/distributed_vectorvalues_iter.size();
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
          for(size_t robot = 0; robot < nrRobots; robot++){
              double change = distMappers[robot]->latestChange();
              std::cout << "[optimizeRotation] Change (Robot " << robot << "): " << change << std::endl;
              if(change > rotationEstimateChangeThreshold){
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


/**
 * @brief optimizePose optimizes the pose in a distributed sense
 * @param distMappers is the vector of estimators communicating at that moment, only those will be optimized.
 * @param maxIter is the maximum number of iterations
 * @param nrRobots is the number of robots
 * @param robotNames is the name of robots
 * @param ordering is the prioritized ordering
 * @param poseEstimateChangeThreshold provides an early stopping condition
 */
void optimizePose(std::vector< boost::shared_ptr<DistributedMapper> > distMappers,
                  size_t maxIter,
                  size_t nrRobots,
                  std::string robotNames,
                  std::vector<size_t> ordering,
                  bool debug,
                  double poseEstimateChangeThreshold,
                  bool useLandmarks = false,
                  boost::optional<std::vector<gtsam::Values>&> poseTrace = boost::none,
                  boost::optional<std::vector<gtsam::Values>&> subgraphPoseTrace = boost::none,
                  boost::optional<gtsam::Values&> poseCentralized = boost::none,
                  boost::optional<gtsam::NonlinearFactorGraph&> poseGraph = boost::none){

  // Compute the centralized pose error if it is available
  double centralizedError = -1;
  if(poseGraph){
      centralizedError = poseGraph->error(*poseCentralized);
    }


  // Before starting no robot is optimized
  for(size_t robot: ordering){
    distMappers[robot]->updateInitialized(false);
    distMappers[robot]->clearNeighboringRobotInit();
  }

  if(debug)
    std::cout << "[optimizePoses] Starting pose iteration"  << std::endl;


  for(size_t iter=0; iter < maxIter; iter++){
      gtsam::Values distributed_iter; // For logging

      for(size_t robot: ordering){   // Iterate each optimizer once

        gtsam::Values subgraph_iter; // For logging
        gtsam::Values neighbors = distMappers[robot]->neighbors();  // Get neighboring values

        // Ask other robots for updated estimates and update it
        for(const gtsam::Values::ConstKeyValuePair& key_value: neighbors){
          gtsam::Key key = key_value.key;

          // distMappers only contains the robots that are currently communicating, so we check
          // if the neighbor keys is from one of these robots, if so, we update it (communication)
          char symbol = gtsam::symbolChr(key);
          if(useLandmarks)symbol=tolower(symbol); // for communication links between two landmarks
          size_t neighboringRobotId = robotNames.find(symbol);
          if (neighboringRobotId != std::string::npos){ // if the robot is actually communicating
              gtsam::Vector poseEstimate = distMappers[neighboringRobotId]->linearizedPosesAt(key);
              distMappers[robot]->updateNeighborLinearizedPoses(key, poseEstimate);
              bool neighboringRobotInitialized = distMappers[neighboringRobotId]->isRobotInitialized();
              distMappers[robot]->updateNeighboringRobotInitialized(symbol, neighboringRobotInitialized); // this requires communication
            }
          else{
              // Robot we are not communicating with are considered as optimized
              distMappers[robot]->updateNeighboringRobotInitialized(symbol, true);
            }

        }

        if(debug)
          std::cout << "[optimizePoses] Queried neighbors"  << std::endl;

        // Logging
        if(poseTrace){
            // Convert to poses for logging
            gtsam::VectorValues linearizedPoses = distMappers[robot]->linearizedPoses();
            gtsam::Values currentEstimate = distMappers[robot]->currentEstimate();
            gtsam::Values retractedEstimate = multirobot_util::retractPose3Global(currentEstimate, linearizedPoses);
            gtsam::Values distributed_robot_i = distMappers[robot]->getConvertedEstimate(retractedEstimate);
            for(const gtsam::Values::ConstKeyValuePair& key_value: distributed_robot_i){
              gtsam::Key key = key_value.key;
              distributed_iter.insert(key, distributed_robot_i.at<gtsam::Pose3>(key));
              subgraph_iter.insert(key, distributed_robot_i.at<gtsam::Pose3>(key));
            }
          }


        if(debug)
          std::cout << "[optimizePoses] Estimating poses"  << std::endl;

        // Iterate
        distMappers[robot]->estimatePoses();

        if(debug)
          std::cout << "[optimizePoses] Estimate poses complete"  << std::endl;


        if(distMappers[robot]->updateType_ == DistributedMapper::incUpdate){
            distMappers[robot]->updatePoses();
          }

        if(debug)
          std::cout << "[optimizePoses] Update linearized poses complete"  << std::endl;


        // This robot is initialized
        distMappers[robot]->updateInitialized(true);

        // Trace
        if(subgraphPoseTrace)
          subgraphPoseTrace->push_back(subgraph_iter);
      }

      if(debug)
        std::cout << "[optimizePoses] Loop over robots complete"  << std::endl;


      // If DistributeJacobi/Jacobi OverRelaxation, we update all the robots at the end of each iteration
      for(size_t robot: ordering){   // Iterate over each robot
        if(distMappers[robot]->updateType_ == DistributedMapper::postUpdate){
            distMappers[robot]->updatePoses();
          }
      }

      // Trace
      if(poseTrace)
        poseTrace->push_back(distributed_iter);

      // If the centralized pose is available, use it as a stopping condition (just for analysis)
      // Stop if the distributed estimate is within 5% of centralized estimate
      if(poseCentralized){
          // If graph error reaches within 10% break
          double distributedError = -1;
          if(poseGraph){
              distributedError = poseGraph->error(distributed_iter);
              double errorChange = fabs(distributedError - centralizedError)*1.0f/centralizedError;
              //std::cout << "Pose Error Change: " << errorChange << std::endl;

              if(errorChange < 0.1){
                  //  std::cout << "Pose Breaking" << std::endl;
                  break;
                }
            }

          /*
          double changeCentralized = (poseCentralized->localCoordinates(distributed_iter).norm())/distributed_iter.size();
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
          for(size_t robot = 0; robot < nrRobots; robot++){     // Iterate each optimizer once
              double change = distMappers[robot]->latestChange();
              std::cout << "[optimizePoses] Change (Robot " << robot << "): " << change << std::endl;
              if(change > poseEstimateChangeThreshold){
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

/**
 * @brief distributedOptimization  performs distributed estimation using Jacobi algorithm
 * @param distMappers is the vector of estimators communicating at that moment, only those will be optimized.
 * @param maxIter is the maximum number of iteration
 * @param rotationTrace if exists, it will store the trace of rotation estimates after each iteration
 * @param poseTrace if exists, it will store the trace of pose estimates after each iteration
 * @return the estimated vector of poses for each robot
 */
std::vector< gtsam::Values >
distributedOptimizer(std::vector< boost::shared_ptr<DistributedMapper> > distMappers,
                     size_t maxIter,
                     DistributedMapper::UpdateType updateType = DistributedMapper::incUpdate,
                     double gamma = 1.0f,
                     double rotationEstimateChangeThreshold = 1e-5,
                     double poseEstimateChangeThreshold = 1e-6,
                     bool useFlaggedInit = false,
                     bool useLandmarks = false,
                     bool debug = false,                     
                     boost::optional<std::vector<gtsam::Values>&> rotationTrace = boost::none,
                     boost::optional<std::vector<gtsam::Values>&> poseTrace  = boost::none,
                     boost::optional<std::vector<gtsam::Values>&> subgraphRotationTrace = boost::none,
                     boost::optional<std::vector<gtsam::Values>&> subgraphPoseTrace  = boost::none,
                     boost::optional<std::vector<gtsam::VectorValues>&> rotationVectorValuesTrace  = boost::none,
                     boost::optional<gtsam::VectorValues&> rotationCentralized = boost::none,
                     boost::optional<gtsam::Values&> poseCentralized = boost::none,
                     boost::optional<gtsam::NonlinearFactorGraph&> graphWithoutPrior = boost::none,
                     boost::optional<gtsam::GaussianFactorGraph&> centralizedRotationGraph = boost::none){

  size_t nrRobots = distMappers.size();
  std::string robotNames = "";
  for(size_t robot = 0; robot < nrRobots; robot++){
      robotNames += distMappers[robot]->robotName(); // this is the string containing chars with robot names
      distMappers[robot]->setFlaggedInit(useFlaggedInit);
      distMappers[robot]->setUpdateType(updateType);
      distMappers[robot]->setGamma(gamma);
    }

  // if (use_pcm && contains_odometry) {
  //   auto max_clique_info = distributed_pcm::DistributedPCM::solveCentralized(dist_mappers, graph_and_values_vec.get(),
  //                                                            pcm_threshold, use_covariance, use_heuristics);
  //   max_clique_size = max_clique_info.first;
  // }

  if(debug)
    std::cout << "Starting Optimizer"  << std::endl;

  ////////////////////////////////////////////////////////////////////////////////////////////
  // Flagged Init
  ////////////////////////////////////////////////////////////////////////////////////////////
  std::vector<size_t> ordering = orderRobots(distMappers,
                                             nrRobots,
                                             robotNames,
                                             useFlaggedInit,
                                             useLandmarks);

  if(debug)
    std::cout << "Ordering complete" << " Ordering size: " << ordering.size() << " nrRobots: " << nrRobots << std::endl;


  ////////////////////////////////////////////////////////////////////////////////////////////
  // Iterate rotation
  ////////////////////////////////////////////////////////////////////////////////////////////

  // Remove keyAnchor belonging to the prior from the centralized rotation
  if(rotationCentralized){
      if(rotationCentralized->exists(keyAnchor)){
          rotationCentralized->erase(keyAnchor);
        }
    }

  // Optimize it
  optimizeRotation(distMappers,
                   maxIter,
                   nrRobots,
                   robotNames,
                   ordering,
                   debug,
                   rotationEstimateChangeThreshold,
                   useLandmarks,
                   rotationTrace,
                   subgraphRotationTrace,
                   rotationVectorValuesTrace,
                   rotationCentralized,
                   centralizedRotationGraph);

  if(debug)
    std::cout << "Optimize rotation complete"  << std::endl;


  ////////////////////////////////////////////////////////////////////////////////////////////
  // Initialize poses: this does not require communication (this part essentially computes the
  // linearization point for the following Jacobi iterations)
  ////////////////////////////////////////////////////////////////////////////////////////////
  // Convert to poses and update neighbors (Project to SO3 + add zero translation)
  for(size_t robot = 0; robot < nrRobots; robot++){
      distMappers[robot]->convertLinearizedRotationToPoses();
    }
  // Update neighboring values
  for(size_t robot = 0; robot < nrRobots; robot++){
      // Get neighboring values
      gtsam::Values neighbors = distMappers[robot]->neighbors();
      // Ask other robots for updated estimates and update it
      for(const gtsam::Values::ConstKeyValuePair& key_value: neighbors){
        gtsam::Key key = key_value.key;
        // pick linear rotation estimate from *robot*
        gtsam::VectorValues linRotEstimateNeighbor;
        linRotEstimateNeighbor.insert( key,  distMappers[robot]->neighborsLinearizedRotationsAt(key) );
        // make a pose out of it
        gtsam::Values rotEstimateNeighbor = gtsam::InitializePose3::normalizeRelaxedRotations(linRotEstimateNeighbor);
        gtsam::Values poseEstimateNeighbor = multirobot_util::pose3WithZeroTranslation(rotEstimateNeighbor);
        // store it
        distMappers[robot]->updateNeighbor(key, poseEstimateNeighbor.at<gtsam::Pose3>(key));
      }
    }

  if(debug){
      std::cout << "Converted rotation to poses"  << std::endl;
      for(size_t robot = 0; robot < nrRobots; robot++){
          multirobot_util::printKeys(distMappers[robot]->currentEstimate());
        }
    }


  ////////////////////////////////////////////////////////////////////////////////////////////
  // Iterate poses
  ////////////////////////////////////////////////////////////////////////////////////////////
  if(poseCentralized){
      if(poseCentralized->exists(keyAnchor)){
          poseCentralized->erase(keyAnchor);
        }
    }

  optimizePose(distMappers,
               maxIter,
               nrRobots,
               robotNames,
               ordering,
               debug,
               poseEstimateChangeThreshold,
               useLandmarks,
               poseTrace,
               subgraphPoseTrace,
               poseCentralized,
               graphWithoutPrior);

  if(debug)
    std::cout << "Optimized poses"  << std::endl;

  // Do global retract
  for(size_t robot = 0; robot < nrRobots; robot++){
      distMappers[robot]->retractPose3Global();
    }

  // Get current estimate
  std::vector< gtsam::Values > estimates(nrRobots);
  for(size_t robot = 0; robot < nrRobots; robot++){
      estimates[robot] = distMappers[robot]->currentEstimate();
    }

  return estimates;
}


////////////////////////////////////////////////////////////////////////////////
// Logging
////////////////////////////////////////////////////////////////////////////////
void logResults(size_t nrRobots,
                std::string traceFile,
                gtsam::Values centralized,
                std::vector< boost::shared_ptr<DistributedMapper> > distMappers){

  for(size_t robot = 0; robot < nrRobots; robot++){
      std::pair<std::vector<double>, std::vector<double> > trace = distMappers[robot]->trace();
      std::vector<double> rotationTrace = trace.first;
      std::vector<double> poseTrace = trace.second;
      // Dump to a file
      std::string traceFileRobot = traceFile+ "_" + boost::lexical_cast<std::string>(robot) + ".txt";
      std::fstream traceStream(traceFileRobot.c_str(), std::fstream::out);
      for(size_t i=0; i<rotationTrace.size(); i++){
          traceStream << rotationTrace[i] << " ";
        }
      traceStream << "-1" << std::endl;

      for(size_t i=0; i<poseTrace.size(); i++){
          traceStream << poseTrace[i] << " ";
        }
      traceStream << "-1" << std::endl;

      // Log centralized estimate error for plotting
      std::pair<double, double> errors = distMappers[robot]->logCentralizedError(centralized);
      traceStream << errors.first << std::endl; // centralized error
      traceStream << errors.second << std::endl; // distributed error

      // Log the estimate change trace
      std::pair<std::vector<double>, std::vector<double> > change_trace = distMappers[robot]->traceEstimateChange();
      std::vector<double> rotationChangeTrace = change_trace.first;
      std::vector<double> poseChangeTrace = change_trace.second;

      for(size_t i=0; i<rotationChangeTrace.size(); i++){
          traceStream << rotationChangeTrace[i] << " ";
        }
      traceStream << "-1" << std::endl;

      for(size_t i=0; i<poseChangeTrace.size(); i++){
          traceStream << poseChangeTrace[i] << " ";
        }
      traceStream << "-1" << std::endl;
      traceStream.close();
    }
}

}
