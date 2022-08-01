// Copyright (C) 2018 by Pierre-Yves Lajoie <lajoie.py@gmail.com>

#include "graph_utils/graph_utils_functions.h"

#include <fstream>
#include <iostream>
#include <math.h>
#include <eigen3/Eigen/Geometry>

namespace graph_utils {

void poseCompose(const graph_utils::PoseWithCovariance &a,
                 const graph_utils::PoseWithCovariance &b,
                 graph_utils::PoseWithCovariance &out) {
  gtsam::Matrix Ha, Hb;
  out.pose = a.pose.compose(b.pose, Ha, Hb);
  out.covariance_matrix = Ha * a.covariance_matrix * Ha.transpose() +
          Hb * b.covariance_matrix * Hb.transpose();
}

void poseInverse(const graph_utils::PoseWithCovariance &a,
                 graph_utils::PoseWithCovariance &out) {
  gtsam::Matrix Ha;
  out.pose = a.pose.inverse(Ha);
  out.covariance_matrix = Ha * a.covariance_matrix * Ha.transpose();
}

void poseBetween(const graph_utils::PoseWithCovariance &a,
                 const graph_utils::PoseWithCovariance &b,
                 graph_utils::PoseWithCovariance &out){
    gtsam::Matrix Ha, Hb;
    out.pose = a.pose.between(b.pose, Ha, Hb);
    out.covariance_matrix = Ha * a.covariance_matrix * Ha.transpose() +
                            Hb * b.covariance_matrix * Hb.transpose();
}

Trajectory buildTrajectory(const Transforms& transforms) {
    // Initialization
    Trajectory trajectory;
    trajectory.start_id = transforms.start_id;
    trajectory.end_id = transforms.end_id;
    size_t current_pose_id = trajectory.start_id;
    PoseWithCovariance temp_pose, total_pose;

    // Add first pose at the origin
    TrajectoryPose current_pose;
    // TODO: Read covariance from file or add option
    current_pose.pose.covariance_matrix = graph_utils::FIXED_COVARIANCE;
    current_pose.id = current_pose_id;
    temp_pose = current_pose.pose;
    trajectory.trajectory_poses.insert(std::make_pair(current_pose_id, current_pose));

    // Initialization
    std::pair<size_t, size_t> temp_pair = std::make_pair(current_pose_id, current_pose_id + 1);
    auto temp_it = transforms.transforms.find(temp_pair);

    // Compositions in chain on the trajectory transforms.
    while (temp_it != transforms.transforms.end() && !(*temp_it).second.is_loopclosure) {
        poseCompose(temp_pose, (*temp_it).second.pose, total_pose);
        temp_pose = total_pose;
        current_pose_id++;
        current_pose.id = current_pose_id;
        current_pose.pose = total_pose;
        trajectory.trajectory_poses.insert(std::make_pair(current_pose_id, current_pose));
        temp_pair = std::make_pair(current_pose_id, current_pose_id + 1);
        temp_it = transforms.transforms.find(temp_pair);
    }

    return trajectory;
}

void printConsistencyGraph(const Eigen::MatrixXi& consistency_matrix, const std::string& file_name) {
    // Intialization
    int nb_consistent_measurements = 0;
    
    // Format edges.
    std::stringstream ss;
    for (int i = 0; i < consistency_matrix.rows(); i++) {
      for (int j = i; j < consistency_matrix.cols(); j++) {
        if (consistency_matrix(i,j) == 1) {
          ss << i+1 << " " << j+1 << std::endl;
          nb_consistent_measurements++;
        }
      }
    }
    
    // Write to file
    std::ofstream output_file;
    output_file.open(file_name);
    output_file << "%%MatrixMarket matrix coordinate pattern symmetric" << std::endl;
    output_file << consistency_matrix.rows() << " " << consistency_matrix.cols() << " " << nb_consistent_measurements << std::endl;
    output_file << ss.str();
    output_file.close();
}

bool isInTrajectory(const Trajectory& trajectory, const size_t& pose_id) {
  return trajectory.trajectory_poses.find(pose_id) != trajectory.trajectory_poses.end();
}

void printConsistentLoopClosures(const LoopClosures& loop_closures, const std::vector<int>& max_clique_data, const std::string& file_name){
  std::ofstream output_file;
  output_file.open(file_name);
  for (const auto& loop_closure_id: max_clique_data) {
    output_file << loop_closures[loop_closure_id].first << " " << loop_closures[loop_closure_id].second << std::endl;
  }
  output_file.close();
}

}