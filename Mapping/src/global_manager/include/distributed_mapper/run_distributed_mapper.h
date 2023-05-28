/**
 * @file run_distributed_mapper.h
 */
#pragma once
#include "distributed_mapper/distributed_mapper_utils.h"
#include <distributed_mapper/evaluation_utils.h>
#include <distributed_mapper/between_chordal_factor.h>

#include <boost/lexical_cast.hpp>
#include <boost/program_options.hpp>
#include <boost/random/mersenne_twister.hpp>
#include <boost/random/normal_distribution.hpp>
#include <boost/random/variate_generator.hpp>
#include <stdlib.h>     /* srand, rand */
#include <time.h>       /* time */
#include <utility>

using namespace std;
using namespace gtsam;

namespace distributed_mapper {
    /**
     * @brief function to run the distributed mapping
     */
    std::tuple<double, double, int> runDistributedMapper(const size_t& nr_robots, const string& log_dir, const string& data_dir, const string& trace_file, const bool& use_XY, const bool& use_OP,
                             const bool& debug, const noiseModel::Diagonal::shared_ptr& prior_model, const noiseModel::Isotropic::shared_ptr& model,
                             const size_t& max_iter, const double& rotation_estimate_change_threshold, const double& pose_estimate_change_threshold,
                             const double& gamma, const bool& use_flagged_init, const distributed_mapper::DistributedMapper::UpdateType& update_type,
                             const bool& use_between_noise,  const bool& use_chr_less_full_graph, const bool& use_landmarks, const double& pcm_threshold, const bool& use_covariance,
                             const bool& use_PCM, const bool& use_heuristics);
}