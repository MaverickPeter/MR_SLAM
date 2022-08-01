#pragma once

#include <distributed_mapper/between_chordal_factor.h>
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
#define COMPARE_CENTRALIZED_DISTRIBUTED_VECTORVALUES(nr_robots, centralized, distributed, tol) \
  { \
  for(size_t i = 0; i < nr_robots; i++){ \
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
#define COMPARE_VALUES_DATASET(nr_robots, centralized, distributed, tol) \
  { \
  for(size_t i = 0; i < nr_robots; i++){ \
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
#define COMPARE_VALUES_SIMULATION(nr_robots, centralized, distributed, tol) \
  { \
  for(size_t i = 0; i < nr_robots; i++){ \
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
  namespace evaluation_utils{


    /**
   * @brief printKeys prints the keys corresponding to the values
   */
    void printKeys(const gtsam::Values& values);

    /**
   * @brief printKeys prints the keys corresponding to the factor graph
   */
    void printKeys(const gtsam::NonlinearFactorGraph& graph);


    /**
   * @brief rowMajorVector takes a rotation matrix converts it into a vector
   */
    gtsam::Vector rowMajorVector(const gtsam::Matrix3& R);

    /**
   * @brief pose3WithZeroTranslation sets to zero the translation part
   */
    gtsam::Values pose3WithZeroTranslation(const gtsam::Values& rotations);


    /** @brief initializeVectorValues use the keys from the input Values and set vector to zero(6) */
    gtsam::VectorValues initializeVectorValues(const gtsam::Values& rotations);

    /**
   * @brief initializeZeroRotation iterates over rotations and puts zero vector
   */
    gtsam::VectorValues
    initializeZeroRotation(const gtsam::Values& sub_initials);

    /**
   * @brief rowMajorVectorValues iterates over rotations and converts each into row-major vector
   */
    gtsam::VectorValues
    rowMajorVectorValues(const gtsam::Values& sub_initials);

    /**
   * @brief retractPose3Global adds delta to initial in global frame
   */
    gtsam::Values retractPose3Global(const gtsam::Values& initial, const gtsam::VectorValues& delta);

    /**
   * @brief retractPose3Global adds delta to initial in global frame with offset
   */
    gtsam::Values retractPose3GlobalWithOffset(const gtsam::Values& initial, const gtsam::VectorValues& delta, const gtsam::Point3& offset);

    /**
   * @brief buildLinearOrientationGraph generates a linear orientation graph given the nonlinear factor graph and uses the between noise
   */
    gtsam::GaussianFactorGraph buildLinearOrientationGraph(const gtsam::NonlinearFactorGraph& g, const bool& use_between_noise = false);


    /**
   * @brief loadSubgraphs loads the subgraphs in the data_path directory
   * @param num_subgraphs is the num of subgraphs in the directory
   * @param data_path is the data path of the directory
   */
    std::pair <std::vector<gtsam::NonlinearFactorGraph>, std::vector<gtsam::Values> >
    loadSubgraphs(const size_t& num_subgraphs, const std::string& data_path);


    /** @brief loadGraphWithPrior loads graph in data_file and adds prior to the first pose */
    std::pair<gtsam::NonlinearFactorGraph, gtsam::Values>
    loadGraphWithPrior(const std::string& data_file, const gtsam::SharedNoiseModel& prior_model);

    /** @brief convertToChordalGraph converts gtsam factor graph to chordal graph */
    gtsam::NonlinearFactorGraph
    convertToChordalGraph(const gtsam::NonlinearFactorGraph& graph, const gtsam::SharedNoiseModel &between_noise, const bool& use_between_noise=false);

    /** @brief centralizedEstimation performs two stage pose estimation in a centralized fashion */
    gtsam::Values centralizedEstimation(const gtsam::NonlinearFactorGraph& graph,
                                        const gtsam::SharedNoiseModel& between_noise,
                                        const gtsam::SharedNoiseModel& prior_noise,
                                        const bool& use_between_noise=false);

    /** @brief centralizedGNEstimation performs GN pose estimation in a centralized fashion */
    gtsam::Values centralizedGNEstimation(const gtsam::NonlinearFactorGraph& graph,
                                          const gtsam::SharedNoiseModel& between_noise,
                                          const gtsam::SharedNoiseModel& prior_noise, const bool& use_between_noise=false);


    /**
   * @brief convert between factor noise to chordal noise format
   * @param noise is shared noise model
   * @return converted noise
   */
    gtsam::SharedNoiseModel convertToChordalNoise(const gtsam::SharedNoiseModel& noise, const gtsam::Matrix& Rhat = gtsam::eye(3));


    /**
   * @brief convertToDiagonalNoise convert between rotation factor noise to diagonal noise format
   * @param noise
   * @return
   */
    gtsam::SharedDiagonal convertToDiagonalNoise(const gtsam::SharedNoiseModel& noise);

    /** @brief writeValuesAsTUM writes the optimized values in TUM format */
    void
    writeValuesAsTUM(const gtsam::Values& values, const std::string& filename);

    /**
     * @brief readFullGraph reads the full graph if it is present in the directory, otherwise creates it
     * @param nr_robots is the number of robots
     * @param graph_and_values_vec contains the graphs and initials of each robot
     */
    gtsam::GraphAndValues readFullGraph(const size_t& nr_robots, // number of robots
                                        const std::vector <gtsam::GraphAndValues>& graph_and_values_vec  // vector of all graphs and initials for each robot
    );

    /**
     *  @brief function to evaluate the resulting estimates of the optimization
     *  @returns a tuple containing the distributed cost, the centralized cost and the initial cost.
     */
    std::tuple<double, double, double> evaluateEstimates(const size_t& nr_robots,
                                                const gtsam::GraphAndValues& full_graph_and_values,
                                                const gtsam::noiseModel::Diagonal::shared_ptr& prior_model,
                                                const gtsam::noiseModel::Isotropic::shared_ptr& model,
                                                const bool& use_between_noise,
                                                const gtsam::Values& distributed_estimates,
                                                const bool& debug);
    /**
     *  @brief function to get centralized estimates
     *  @returns a pair with Centralized Two Stage estimates and Centralized Two Stage + Gauss Newton estimates
     */
    std::pair<gtsam::Values, gtsam::Values> centralizedEstimates(const gtsam::GraphAndValues &full_graph_and_values,
                                const gtsam::noiseModel::Diagonal::shared_ptr &prior_model,
                                const gtsam::noiseModel::Isotropic::shared_ptr &model,
                                const bool &use_between_noise); 
    /**
     * @brief readFullGraph reads the full graph if it is present in the directory, otherwise creates it
     * @param nr_robots is the number of robots
     * @param graph_and_values_vec contains the graphs and initials of each robot
     */
    gtsam::GraphAndValues readFullGraph(const size_t& nr_robots, // number of robots
                                 const std::vector <gtsam::GraphAndValues>& graph_and_values_vec  // vector of all graphs and initials for each robot
    );

    /**
     * @brief readFullGraph reads the full graph if it is present in the directory, otherwise creates it, reads in the order of the vector.
     * @param graph_and_values_vec contains the graphs and initials of each robot
     */
    gtsam::GraphAndValues readFullGraph(const std::vector <gtsam::GraphAndValues>& graph_and_values_vec  // vector of all graphs and initials for each robot
    );

    /**
     * @brief copyInitial copies the initial graph to optimized graph as a fall back option
     * @param nr_robots is the number of robots
     * @param data_dir is the directory containing the initial graph
     */
    void copyInitial(const size_t& nr_robots, const std::string& data_dir);


  }

}
