// Copyright (C) 2018 by Pierre-Yves Lajoie <lajoie.py@gmail.com>

#include <gtsam/inference/Symbol.h>
#include "robot_measurements/interrobot_measurements.h"

namespace robot_measurements {

graph_utils::LoopClosures extractLoopClosures(const graph_utils::Transforms& transforms) {
  graph_utils::LoopClosures loop_closures;
  for (auto transform : transforms.transforms) {
    loop_closures.emplace_back(std::make_pair(transform.first.first, transform.first.second));
  }
  return loop_closures;
}

InterRobotMeasurements::InterRobotMeasurements(const graph_utils::Transforms& transforms,
                                               const unsigned char& robot1_id,
                                               const unsigned char& robot2_id):
                                               RobotMeasurements(transforms, extractLoopClosures(transforms)),
                                               robot1_id_(robot1_id), robot2_id_(robot2_id){}

const unsigned char& InterRobotMeasurements::getRobot1ID() const {
  return robot1_id_;
};

const unsigned char& InterRobotMeasurements::getRobot2ID() const {
  return robot2_id_;
};

}