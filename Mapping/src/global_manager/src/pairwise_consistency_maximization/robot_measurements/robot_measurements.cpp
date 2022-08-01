// Copyright (C) 2018 by Pierre-Yves Lajoie <lajoie.py@gmail.com>

#include "robot_measurements/robot_measurements.h"
#include "graph_utils/graph_utils_functions.h"

namespace robot_measurements {

RobotMeasurements::RobotMeasurements(const graph_utils::Transforms& transforms,
                                     const graph_utils::LoopClosures& loop_closures){
    num_poses_ = transforms.transforms.size() + 1;
    loop_closures_ = loop_closures;
    transforms_ = transforms;
    nb_degree_freedom_ = 6; // TODO: support 2D case
}

RobotMeasurements::RobotMeasurements(){
    nb_degree_freedom_ = 6; // TODO: support 2D case
    num_poses_ = 1;      
    id_initialized_ = false;   
}

void RobotMeasurements::addTransform(const gtsam::BetweenFactor<gtsam::Pose3>& factor, const gtsam::Matrix& covariance_matrix) {
    graph_utils::Transform transform;
    transform.i = factor.key1();
    transform.j = factor.key2();
    transform.pose.pose = factor.measured();
    transform.pose.covariance_matrix = covariance_matrix;
    
    transform.is_loopclosure = false;
    if (gtsam::Symbol(transform.i).chr() != gtsam::Symbol(transform.j).chr()) {
        transform.is_loopclosure = true;
        loop_closures_.emplace_back(std::make_pair(transform.i, transform.j));
    }

    if (!transform.is_loopclosure) {
        if (!id_initialized_) {
            transforms_.start_id = transform.i;
            transforms_.end_id = transform.j;
            id_initialized_ = true;
        } else {
            transforms_.start_id = std::min(transforms_.start_id, transform.i);
            transforms_.end_id = std::max(transforms_.end_id, transform.j);
        }
        num_poses_++;
    }
    transforms_.transforms.insert(
            std::make_pair(std::make_pair(factor.key1(), factor.key2()), transform));
}

void RobotMeasurements::removeTransform(const std::pair<gtsam::Key,gtsam::Key>& index) {
    transforms_.transforms.erase(index);
}

const graph_utils::Transforms& RobotMeasurements::getTransforms() const {
    return transforms_;
}

const size_t& RobotMeasurements::getNumPoses() const {
    return num_poses_;
}

const graph_utils::LoopClosures& RobotMeasurements::getLoopClosures() const {
    return loop_closures_;
}

const uint8_t& RobotMeasurements::getNbDegreeFreedom() const {
    return nb_degree_freedom_;
}

}