// Copyright (C) 2018 by Pierre-Yves Lajoie <lajoie.py@gmail.com>

#ifndef DISTRIBUTED_MAPPER_INTERROBOT_MEASUREMENTS_H
#define DISTRIBUTED_MAPPER_INTERROBOT_MEASUREMENTS_H

#include "robot_measurements.h"

namespace robot_measurements {
/** \class InterRobotMeasurements
 * Class that contains the interrobot measurements between 2 robots.
 */
class InterRobotMeasurements : public RobotMeasurements {
 public:
  /**
   * \brief Constructor
   * @param file_name Name of the file containing the robot measurements.
   */
  InterRobotMeasurements(const graph_utils::Transforms& transforms,
                         const unsigned char& robot1_id,
                         const unsigned char& robot2_id);

  /*
   * Accessors
   */

  /** \brief Accessor
   * @return the first robot ID
   */
  const unsigned char& getRobot1ID() const;

  /** \brief Accessor
   * @return the second robot ID
   */
  const unsigned char& getRobot2ID() const;

 private:
  unsigned char robot1_id_; ///< ID of the first robot.
  unsigned char robot2_id_; ///< ID of the second robot.
};
}

#endif //DISTRIBUTED_MAPPER_INTERROBOT_MEASUREMENTS_H
