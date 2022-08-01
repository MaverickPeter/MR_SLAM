#pragma once

#include <global_manager/BetweenChordalFactor.h>
#include <gtsam/slam/InitializePose3.h>
#include <gtsam/slam/dataset.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/nonlinear/GaussNewtonOptimizer.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/linear/VectorValues.h>
#include <gtsam/base/Value.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Rot3.h>
#include <gtsam/inference/Symbol.h>
#include <boost/lexical_cast.hpp>
#include <fstream>      // std::fstream

namespace distributed_mapper{


  // MACRO for comparing centralized and distributed solution
#define COMPARE_CENTRALIZED_DISTRIBUTED_VECTORVALUES(nrRobots, centralized, distributed, tol) \
  { \
  for(size_t i = 0; i < nrRobots; i++){ \
  gtsam::VectorValues result = distributed.at(i); \
  BOOST_FOREACH(const VectorValues::KeyValuePair& key_value, result) { \
  gtsam::Symbol key = key_value.first; \
  char robot = gtsam::symbolChr(key); \
  int index = gtsam::symbolIndex(key); \
  if(index == 9999999) continue; \
  EXPECT(assert_equal(centralized.at(index), key_value.second, , tol));\
} \
} \
}

  // MACRO for comparing centralized and distributed solution
#define COMPARE_VALUES_DATASET(nrRobots, centralized, distributed, tol) \
  { \
  for(size_t i = 0; i < nrRobots; i++){ \
  gtsam::Values result = distributed.at(i); \
  for(const Values::KeyValuePair& key_value: result) { \
  gtsam::Symbol key = key_value.key; \
  int index = gtsam::symbolIndex(key); \
  Pose3 actual = result.at<Pose3>(key); \
  EXPECT(assert_equal(centralized.at<Pose3>(index), actual, tol));\
} \
} \
}

  // MACRO for comparing centralized and distributed solution
#define COMPARE_VALUES_SIMULATION(nrRobots, centralized, distributed, tol) \
  { \
  for(size_t i = 0; i < nrRobots; i++){ \
  gtsam::Values result = distributed.at(i); \
  for(const Values::KeyValuePair& key_value: result) { \
  gtsam::Symbol key = key_value.key; \
  Pose3 actual = result.at<Pose3>(key); \
  EXPECT(assert_equal(centralized.at<Pose3>(key), actual, tol));\
} \
} \
}


  /**
 * Multi-Robot utility functions
 */
  namespace multirobot_util{


    /**
   * @brief printKeys prints the keys corresponding to the values
   */
    void printKeys(gtsam::Values values);

    /**
   * @brief printKeys prints the keys corresponding to the factor graph
   */
    void printKeys(gtsam::NonlinearFactorGraph graph);


    /**
   * @brief rowMajorVector takes a rotation matrix converts it into a vector
   */
    gtsam::Vector rowMajorVector(gtsam::Matrix3 R);

    /**
   * @brief pose3WithZeroTranslation sets to zero the translation part
   */
    gtsam::Values pose3WithZeroTranslation(gtsam::Values rotations);


    /** @brief initializeVectorValues use the keys from the input Values and set vector to zero(6) */
    gtsam::VectorValues initializeVectorValues(gtsam::Values rotations);

    /**
   * @brief initializeZeroRotation iterates over rotations and puts zero vector
   */
    gtsam::VectorValues
    initializeZeroRotation(gtsam::Values subInitials);

    /**
   * @brief rowMajorVectorValues iterates over rotations and converts each into row-major vector
   */
    gtsam::VectorValues
    rowMajorVectorValues(gtsam::Values subInitials);

    /**
   * @brief retractPose3Global adds delta to initial in global frame
   */
    gtsam::Values retractPose3Global(gtsam::Values initial, gtsam::VectorValues delta);

    /**
   * @brief buildLinearOrientationGraph generates a linear orientation graph given the nonlinear factor graph and uses the between noise
   */
    gtsam::GaussianFactorGraph buildLinearOrientationGraph(const gtsam::NonlinearFactorGraph& g, bool useBetweenNoise = false);


    /**
   * @brief loadSubgraphs loads the subgraphs in the dataPath directory
   * @param numSubgraphs is the num of subgraphs in the directory
   * @param dataPath is the data path of the directory
   */
    std::pair <std::vector<gtsam::NonlinearFactorGraph>, std::vector<gtsam::Values> >
    loadSubgraphs(size_t numSubgraphs, std::string dataPath);


    /** @brief loadGraphWithPrior loads graph in dataFile and adds prior to the first pose */
    std::pair<gtsam::NonlinearFactorGraph, gtsam::Values>
    loadGraphWithPrior(std::string dataFile, const gtsam::SharedNoiseModel& priorModel);

    /** @brief convertToChordalGraph converts gtsam factor graph to chordal graph */
    gtsam::NonlinearFactorGraph
    convertToChordalGraph(gtsam::NonlinearFactorGraph graph, const gtsam::SharedNoiseModel &betweenNoise, bool useBetweenNoise=false);

    /** @brief centralizedEstimation performs two stage pose estimation in a centralized fashion */
    gtsam::Values centralizedEstimation(gtsam::NonlinearFactorGraph graph,
                                        const gtsam::SharedNoiseModel& betweenNoise,
                                        const gtsam::SharedNoiseModel& priorNoise,
                                        bool useBetweenNoise=false);

    /** @brief centralizedGNEstimation performs GN pose estimation in a centralized fashion */
    gtsam::Values centralizedGNEstimation(gtsam::NonlinearFactorGraph graph,
                                          const gtsam::SharedNoiseModel& betweenNoise,
                                          const gtsam::SharedNoiseModel& priorNoise, bool useBetweenNoise=false);


    /**
   * @brief convert between factor noise to chordal noise format
   * @param noise is shared noise model
   * @return converted noise
   */
    gtsam::SharedNoiseModel convertToChordalNoise(gtsam::SharedNoiseModel noise, gtsam::Matrix Rhat = gtsam::eye(3));


    /**
   * @brief convertToDiagonalNoise convert between rotation factor noise to diagonal noise format
   * @param noise
   * @return
   */
    gtsam::SharedDiagonal convertToDiagonalNoise(gtsam::SharedNoiseModel noise);

    /** @brief writeValuesAsTUM writes the optimized values in TUM format */
    void
    writeValuesAsTUM(gtsam::Values values, std::string filename);

  }

}
