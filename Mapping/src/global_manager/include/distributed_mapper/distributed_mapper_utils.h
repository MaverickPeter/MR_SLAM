#pragma once 

#include "distributed_pcm/distributed_pcm.h"
#include <fstream>
#include <algorithm>
#include <map>
#include <utility>


namespace distributed_mapper{

/**
 * @brief orderRobots orders the robots using flagged init
 * @param dist_mappers is the vector of estimators communicating at that moment, only those will be optimized.
 * @param nr_robots is the number of robots
 * @param robot_names is the name of robots
 * @param use_flagged_init orders using flagged initialization
 * @return
 */
std::vector<size_t>
orderRobots(const std::vector< boost::shared_ptr<DistributedMapper> >& dist_mappers,
            const size_t& nr_robots,
            const std::string& robot_names,
            const bool& use_flagged_init,
            const bool& use_landmarks = false);

/**
 * @brief logrotation_trace
 * @param dist_mapper_robot
 * @param distributed_vectorvalues_iter
 * @param distributed_iter
 * @param subgraph_iter
 */
std::pair<gtsam::Values, gtsam::VectorValues> logrotation_trace(const boost::shared_ptr<DistributedMapper>& dist_mapper_robot);


/**
 * @brief optimizeRotation optimizes the rotation in a distributed sense
 * @param dist_mappers is the vector of estimators communicating at that moment, only those will be optimized.
 * @param max_iter is the maximum number of iteratinos
 * @param nr_robots is the number of robots
 * @param robot_names is the name of robots
 * @param ordering is the prioritized ordering
 * @param rotation_estimate_change_threshold provides an early stopping condition
 * @param rotation_trace contains the overall converted estimate at any iteration
 * @param subgraph_rotation_trace contains the Values for a particular subgraph being optimized
 * @param rotation_vector_values_trace contains the linearized rotation estimate
 */
void optimizeRotation(std::vector< boost::shared_ptr<DistributedMapper> >& dist_mappers,
                      const size_t& max_iter,
                      const size_t& nr_robots,
                      const std::string& robot_names,
                      const std::vector<size_t>& ordering,
                      const bool& debug,
                      const double& rotation_estimate_change_threshold,
                      const bool& use_landmarks = false,
                      boost::optional<std::vector<gtsam::Values>&> rotation_trace = boost::none,
                      boost::optional<std::vector<gtsam::Values>&> subgraph_rotation_trace = boost::none,
                      boost::optional<std::vector<gtsam::VectorValues>&> rotation_vector_values_trace = boost::none,
                      boost::optional<gtsam::VectorValues&> rotation_centralized = boost::none,
                      boost::optional<gtsam::GaussianFactorGraph&> rotation_graph = boost::none);


/**
 * @brief optimizePose optimizes the pose in a distributed sense
 * @param dist_mappers is the vector of estimators communicating at that moment, only those will be optimized.
 * @param max_iter is the maximum number of iterations
 * @param nr_robots is the number of robots
 * @param robot_names is the name of robots
 * @param ordering is the prioritized ordering
 * @param pose_estimate_change_threshold provides an early stopping condition
 */
void optimizePose(std::vector< boost::shared_ptr<DistributedMapper> >& dist_mappers,
                  const size_t& max_iter,
                  const size_t& nr_robots,
                  const std::string& robot_names,
                  const std::vector<size_t> &ordering,
                  const bool& debug,
                  const double& pose_estimate_change_threshold,
                  const bool& use_landmarks = false,
                  boost::optional<std::vector<gtsam::Values>&> pose_trace = boost::none,
                  boost::optional<std::vector<gtsam::Values>&> subgraph_pose_trace = boost::none,
                  boost::optional<gtsam::Values&> pose_centralized = boost::none,
                  boost::optional<gtsam::NonlinearFactorGraph&> pose_graph = boost::none);

/**
 * @brief distributedOptimization  performs distributed estimation using Jacobi algorithm
 * @param dist_mappers is the vector of estimators communicating at that moment, only those will be optimized.
 * @param max_iter is the maximum number of iteration
 * @param rotation_trace if exists, it will store the trace of rotation estimates after each iteration
 * @param pose_trace if exists, it will store the trace of pose estimates after each iteration
 * @return the estimated vector of poses for each robot
 */
std::vector< gtsam::Values >
distributedOptimizer(std::vector< boost::shared_ptr<DistributedMapper> >& dist_mappers,
                     const size_t& max_iter,
                     int& max_clique_size,
                     const  DistributedMapper::UpdateType& update_type = DistributedMapper::incUpdate,
                     const double& gamma = 1.0f,
                     const double& rotation_estimate_change_threshold = 1e-5,
                     const double& pose_estimate_change_threshold = 1e-6,
                     const bool& use_flagged_init = false,
                     const bool& use_landmarks = false,
                     const bool& debug = false,
                     const bool& contains_odometry = true,
                     const double& pcm_threshold = 0.99,
                     const bool& use_covariance = false,
                     const bool& use_pcm = true,
                     const bool& use_heuristics = true,
                     boost::optional<std::vector<gtsam::GraphAndValues>&> graph_and_values_vec = boost::none,
                     boost::optional<std::vector<gtsam::Values>&> rotation_trace = boost::none,
                     boost::optional<std::vector<gtsam::Values>&> pose_trace  = boost::none,
                     boost::optional<std::vector<gtsam::Values>&> subgraph_rotation_trace = boost::none,
                     boost::optional<std::vector<gtsam::Values>&> subgraph_pose_trace  = boost::none,
                     boost::optional<std::vector<gtsam::VectorValues>&> rotation_vector_values_trace  = boost::none,
                     boost::optional<gtsam::VectorValues&> rotation_centralized = boost::none,
                     boost::optional<gtsam::Values&> pose_centralized = boost::none,
                     boost::optional<gtsam::NonlinearFactorGraph&> graph_without_prior = boost::none,
                     boost::optional<gtsam::GaussianFactorGraph&> centralized_rotation_graph = boost::none);


////////////////////////////////////////////////////////////////////////////////
// Logging
////////////////////////////////////////////////////////////////////////////////
void logResults(const size_t& nr_robots,
                const std::string& trace_file,
                const gtsam::Values& centralized,
                std::vector< boost::shared_ptr<DistributedMapper> >& dist_mappers);

}
