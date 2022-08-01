// Copyright (C) 2019 by Pierre-Yves Lajoie <lajoie.py@gmail.com>

#include "distributed_mapper/run_distributed_mapper.h"

using namespace std;
using namespace gtsam;

namespace distributed_mapper {

/**
 * @brief function to run the whole pipeline
 */
std::tuple<double, double, int> runDistributedMapper(const size_t& nr_robots, const string& log_dir, const string& data_dir, const string& trace_file, const bool& use_XY, const bool& use_OP,
                                                     const bool& debug, const noiseModel::Diagonal::shared_ptr& prior_model, const noiseModel::Isotropic::shared_ptr& model,
                                                     const size_t& max_iter, const double& rotation_estimate_change_threshold, const double& pose_estimate_change_threshold,
                                                     const double& gamma, const bool& use_flagged_init, const distributed_mapper::DistributedMapper::UpdateType& update_type,
                                                     const bool& use_between_noise,  const bool& use_chr_less_full_graph, const bool& use_landmarks, const double& pcm_threshold, const bool& use_covariance,
                                                     const bool& use_PCM,
                                                     const bool& use_heuristics) {

  vector <GraphAndValues> graph_and_values_vec; // vector of all graphs and initials

  // Config
  string robot_names_;
  if (use_XY) {
    robot_names_ = string("xyz"); // robot names
  } else {
    robot_names_ = string("abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ"); // robot names
  }

  if (use_OP) {
    robot_names_ = string("opqrstuvwxyz"); // robot names
  }

  if (use_landmarks) {
    robot_names_ = string("abcdefghijklmnopqrstyvwxyz"); // robot names
    // ABC... are used for objects
  }


  bool disconnected_graph_ = false; // Flag to check whether graphs are connected or not

  ////////////////////////////////////////////////////////////////////////////////
  // Distributed Optimization
  ////////////////////////////////////////////////////////////////////////////////

  // Vector of distributed optimizers, one for each robot
  vector <boost::shared_ptr<DistributedMapper>> dist_mappers;

  // Load subgraph and construct dist_mapper optimizers
  for (size_t robot = 0; robot < nr_robots; robot++) {

    // Construct a distributed jacobi object with the given robot name
    boost::shared_ptr <DistributedMapper> dist_mapper(
        new DistributedMapper(robot_names_[robot], use_chr_less_full_graph));

    // Read G2o files
    string data_file_i = data_dir + boost::lexical_cast<string>(robot) + ".g2o";
    GraphAndValues graph_and_values_g2o = readG2o(data_file_i, true);
    Values initial = *(graph_and_values_g2o.second);

    // Continue if empty
    if (initial.empty()) {
      disconnected_graph_ = true;
      continue;
    }

    // Construct graph_and_values using cleaned up initial values
    GraphAndValues graph_and_values = make_pair(graph_and_values_g2o.first, boost::make_shared<Values>(initial));
    graph_and_values_vec.push_back(graph_and_values);

    // Use between noise or not in optimizePoses
    dist_mapper->setUseBetweenNoiseFlag(use_between_noise);

    // Use landmarks
    dist_mapper->setUseLandmarksFlag(use_landmarks);

    // Load subgraphs
    dist_mapper->loadSubgraphAndCreateSubgraphEdge(graph_and_values);

    // Add prior to the first robot
    if (robot == 0) {
      Key first_key = KeyVector(initial.keys()).at(0);
      dist_mapper->addPrior(first_key, initial.at<Pose3>(first_key), prior_model);
    }

    // Verbosity level
    dist_mapper->setVerbosity(DistributedMapper::ERROR);

    // Check for graph connectivity
    std::set<char> neighboring_robots = dist_mapper->getNeighboringChars();
    if (neighboring_robots.size() == 0)
      disconnected_graph_ = true;

    // Push to the set of optimizers
    dist_mappers.push_back(dist_mapper);
  }

  // Vectors containing logs
  vector <Values> rotation_trace;
  vector <Values> pose_trace;
  vector <Values> subgraph_rotation_trace;
  vector <Values> subgraph_pose_trace;
  vector <VectorValues> rotation_vector_values_trace;

  if (debug)
    cout << "Optimizing" << endl;
  // Distributed Estimate

  if (!disconnected_graph_) {
    try {
      // try optimizing
      int max_clique_size = 0;
      vector <Values> estimates = distributedOptimizer(dist_mappers, max_iter, max_clique_size, update_type,
                                                       gamma, rotation_estimate_change_threshold,
                                                       pose_estimate_change_threshold,
                                                       use_flagged_init, use_landmarks, debug, true,
                                                       pcm_threshold, use_covariance, use_PCM, use_heuristics,
                                                       graph_and_values_vec,
                                                       rotation_trace, pose_trace, subgraph_rotation_trace,
                                                       subgraph_pose_trace, rotation_vector_values_trace);

      if (debug)
        cout << "Done" << endl;

      // Aggregate estimates from all the robots
      Values distributed;
      for (size_t i = 0; i < estimates.size(); i++) {
        for (const Values::ConstKeyValuePair &key_value: estimates[i]) {
          Key key = key_value.key;
          if (!distributed.exists(key))
            distributed.insert(key, estimates[i].at<Pose3>(key));
        }

        // Write the corresponding estimate to disk
        string dist_optimized_i = data_dir + boost::lexical_cast<string>(i) + "_optimized.g2o";
        writeG2o(*(graph_and_values_vec[i].first), estimates[i], dist_optimized_i);
      }

      if (debug)
        cout << "Done Aggregating" << endl;

      GraphAndValues full_graph_and_values = evaluation_utils::readFullGraph(nr_robots, graph_and_values_vec);

      // Write optimized full graph
      string dist_optimized = data_dir + "fullGraph_optimized.g2o";
      writeG2o(*(full_graph_and_values.first), distributed, dist_optimized);

      auto errors = evaluation_utils::evaluateEstimates(nr_robots,
          full_graph_and_values,
          prior_model,
          model,
          use_between_noise,
          distributed,
          debug);

      return std::make_tuple(std::get<0>(errors), std::get<1>(errors), max_clique_size);
    }
    catch (...) {
      // Optimization failed (maybe due to disconnected graph)
      // Copy initial to optimized g2o files in that case
      evaluation_utils::copyInitial(nr_robots, data_dir);
    }
  } else {
    // Graph is disconnected
    cout << "Graph is disconnected: " << endl;
    evaluation_utils::copyInitial(nr_robots, data_dir);
  }
}
}