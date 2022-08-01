// Copyright (C) 2018 by Pierre-Yves Lajoie <lajoie.py@gmail.com>

#include "robot_measurements/robot_local_map.h"
#include "graph_utils/graph_utils_functions.h"

namespace robot_measurements {

RobotLocalMap::RobotLocalMap(const graph_utils::Transforms& transforms,
                            const graph_utils::LoopClosures& loop_closures):
                            RobotMeasurements(transforms, loop_closures) {
    trajectory_ = graph_utils::buildTrajectory(transforms_);
}

RobotLocalMap::RobotLocalMap(const graph_utils::Trajectory& trajectory,
                            const graph_utils::Transforms& transforms,
                            const graph_utils::LoopClosures& loop_closures):
                            RobotMeasurements(transforms, loop_closures){
    trajectory_ = trajectory;                    
}

RobotLocalMap::RobotLocalMap():
                     RobotMeasurements(){   
}

void RobotLocalMap::addTransform(const gtsam::BetweenFactor<gtsam::Pose3>& factor, const gtsam::Matrix& covariance_matrix) {
    RobotMeasurements::addTransform(factor, covariance_matrix);
    trajectory_.start_id = transforms_.start_id;
    trajectory_.end_id = transforms_.end_id;

    if (trajectory_.trajectory_poses.empty()) {
        // Add first pose at the origin
        graph_utils::TrajectoryPose first_pose;
        first_pose.pose.covariance_matrix = covariance_matrix;
        first_pose.id = transforms_.start_id;
        trajectory_.trajectory_poses.insert(std::make_pair(trajectory_.start_id, first_pose));
    }

    graph_utils::PoseWithCovariance total_pose;
    gtsam::Key key1 = factor.key1();
    gtsam::Key key2 = factor.key2();
    if (gtsam::Symbol(key1).chr() != gtsam::Symbol(trajectory_.start_id).chr()) {
        key1 = factor.key2();
        key2 = factor.key1();
    }
    poseCompose(trajectory_.trajectory_poses.at(key1).pose, 
                transforms_.transforms.at(std::make_pair(factor.key1(), factor.key2())).pose, total_pose);
    graph_utils::TrajectoryPose current_pose;
    current_pose.id = key2;
    current_pose.pose = total_pose;
    trajectory_.trajectory_poses.insert(std::make_pair(current_pose.id, current_pose));
}

const graph_utils::Trajectory& RobotLocalMap::getTrajectory() const {
    return trajectory_;
}

}