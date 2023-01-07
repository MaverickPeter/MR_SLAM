// Copyright (C) 2018 by Pierre-Yves Lajoie <lajoie.py@gmail.com>

#include "global_map/global_map.h"
#include "findClique.h"
#include <math.h>
#include <gtsam/inference/Symbol.h>
#include <iostream>
#include <sys/stat.h>
#include <sys/types.h> 

#ifdef LOG_DIR
#define DIR LOG_DIR
#else
#define DIR "/tmp/"
#endif

namespace global_map {

const std::string GlobalMap::LOG_DIRECTORY = std::string(DIR) + std::string("log/");
const std::string GlobalMap::CONSISTENCY_MATRIX_FILE_NAME = std::string(GlobalMap::LOG_DIRECTORY+"consistency_matrix");
const std::string GlobalMap::CONSISTENCY_LOOP_CLOSURES_FILE_NAME = std::string(GlobalMap::LOG_DIRECTORY+"consistent_loop_closures");


GlobalMap::GlobalMap(const robot_measurements::RobotLocalMap& robot1_local_map,
                const robot_measurements::RobotLocalMap& robot2_local_map,
                const robot_measurements::RobotMeasurements& interrobot_measurements,
                const double& pcm_threshold,
                const bool& use_heuristics):
                pairwise_consistency_(robot1_local_map.getTransforms(), robot2_local_map.getTransforms(), 
                            interrobot_measurements.getTransforms(), interrobot_measurements.getLoopClosures(),
                            robot1_local_map.getTrajectory(), robot2_local_map.getTrajectory(),
                            robot1_local_map.getNbDegreeFreedom(), pcm_threshold), use_heuristics_(use_heuristics) {}


std::pair<std::vector<int>, int> GlobalMap::pairwiseConsistencyMaximization() {
    // Compute consistency matrix
    Eigen::MatrixXi consistency_matrix = pairwise_consistency_.computeConsistentMeasurementsMatrix();

    char robot_id = gtsam::Symbol(pairwise_consistency_.getTransformsRobot1().start_id).chr();
    mkdir(GlobalMap::LOG_DIRECTORY.c_str(),S_IRUSR | S_IWUSR | S_IXUSR | S_IRWXG | S_IRWXO);

    std::string consistency_matrix_file = CONSISTENCY_MATRIX_FILE_NAME + "_" + robot_id + ".clq.mtx";

    graph_utils::printConsistencyGraph(consistency_matrix, consistency_matrix_file);

    // Compute maximum clique
    FMC::CGraphIO gio;
    gio.readGraph(consistency_matrix_file);
    int max_clique_size = 0;
    std::vector<int> max_clique_data;

    //max_clique_size = FMC::maxClique(gio, max_clique_size, max_clique_data);
    max_clique_size = FMC::maxCliqueHeu(gio, max_clique_data);

    // Print results
    std::string consistency_loop_closures_file = CONSISTENCY_LOOP_CLOSURES_FILE_NAME + "_" + robot_id + ".txt";
    graph_utils::printConsistentLoopClosures(pairwise_consistency_.getLoopClosures(), max_clique_data, consistency_loop_closures_file);

    int number_of_loop_closures_to_be_rejected = pairwise_consistency_.getLoopClosures().size() - max_clique_data.size();
    return std::make_pair(max_clique_data, number_of_loop_closures_to_be_rejected);
}

}