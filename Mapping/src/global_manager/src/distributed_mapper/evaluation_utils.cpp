#include "distributed_mapper/evaluation_utils.h"

using namespace std;
using namespace gtsam;

namespace distributed_mapper {

static const Matrix I9 = eye(9);
static const Vector zero9 = Vector::Zero(9);
static const Matrix zero33 = Matrix::Zero(3, 3);
static const Key key_anchor = symbol('Z', 9999999);

namespace evaluation_utils{
  //*****************************************************************************
  Vector rowMajorVector(const Matrix3& R) {
    return (Vector(9) << R(0, 0), R(0, 1), R(0, 2),/*  */ R(1, 0), R(1, 1), R(1, 2), /*  */ R(2, 0), R(2, 1), R(2,
                                                                                                                2)).finished();
  }

  //*****************************************************************************
  void printKeys(const Values& values) {
    cout << "Keys: ";
    for (const Values::ConstKeyValuePair &key_value: values) {
      Key key = key_value.key;
      cout << symbolChr(key) << symbolIndex(key) << " ";
    }
    cout << endl;
  }

  //*****************************************************************************
  void printKeys(const NonlinearFactorGraph& graph) {
    cout << "Factor Keys: ";
    for (size_t k = 0; k < graph.size(); k++) {
      KeyVector keys = graph.at(k)->keys();
      if (keys.size() == 2) {
        cout << "(" << symbolChr(keys.at(0)) << symbolIndex(keys.at(0)) << "," <<
             symbolChr(keys.at(1)) << symbolIndex(keys.at(1)) << ") ";
      }
    }
    cout << endl;
  }

  //*****************************************************************************
  Values pose3WithZeroTranslation(const Values& rotations) {
    Values poses;
    for (const Values::ConstKeyValuePair &key_value: rotations) {
      Key key = key_value.key;
      Pose3 pose(rotations.at<Rot3>(key), zero(3));
      poses.insert(key, pose);
    }
    return poses;
  }

  //*****************************************************************************
  VectorValues initializeVectorValues(const Values& rotations) {
    VectorValues vector_values;
    for (const Values::ConstKeyValuePair &key_value: rotations) {
      Key key = key_value.key;
      vector_values.insert(key, zero(6));
    }
    return vector_values;
  }

  //*****************************************************************************
  VectorValues
  initializeZeroRotation(const Values& sub_initials) {
    VectorValues sub_initials_vector_value;
    for (const Values::ConstKeyValuePair &key_value: sub_initials) {
      Vector r = zero(9);
      sub_initials_vector_value.insert(key_value.key, r);
    }
    return sub_initials_vector_value;
  }

  //*****************************************************************************
  VectorValues
  rowMajorVectorValues(const Values& sub_initials) {
    VectorValues sub_initials_vector_value;
    for (const Values::ConstKeyValuePair &key_value: sub_initials) {
      Pose3 pose = sub_initials.at<Pose3>(key_value.key);
      Matrix3 R = pose.rotation().matrix();
      Vector r = rowMajorVector(R);
      sub_initials_vector_value.insert(key_value.key, r);
    }
    return sub_initials_vector_value;
  }

  //*****************************************************************************
  Values retractPose3Global(const Values& initial, const VectorValues& delta) {
    Values estimate;
    for (const Values::ConstKeyValuePair &key_value: initial) {
      Key key = key_value.key;
      Vector6 delta_pose = delta.at(key);
      Rot3 R = initial.at<Pose3>(key).rotation().retract(delta_pose.head(3));
      Point3 t = initial.at<Pose3>(key).translation() + Point3(delta_pose.tail(3));
      estimate.insert(key, Pose3(R, t));
    }
    return estimate;
  }

  //*****************************************************************************
  Values retractPose3GlobalWithOffset(const Values& initial, const VectorValues& delta, const gtsam::Point3& offset) {
    Values estimate;
    for (const Values::ConstKeyValuePair &key_value: initial) {
      Key key = key_value.key;
      Vector6 delta_pose = delta.at(key);
      Rot3 R = initial.at<Pose3>(key).rotation().retract(delta_pose.head(3));
      Point3 t_initial = initial.at<Pose3>(key).translation();
      Point3 t = Point3(delta_pose.tail(3)) + offset;
      estimate.insert(key, Pose3(R, t));
    }
    return estimate;
  }

  //*****************************************************************************
  pair<vector<NonlinearFactorGraph>, vector<Values> >
  loadSubgraphs(const size_t& num_subgraphs, const string& data_path) {
    vector<NonlinearFactorGraph> sub_graphs;
    vector<Values> sub_initials;

    for (size_t i = 0; i < num_subgraphs; i++) { // for each robot
      string data_file_i = data_path + boost::lexical_cast<string>(i) + ".g2o";
      GraphAndValues graph_and_values_i = readG2o(data_file_i, true);

      sub_graphs.push_back(*(graph_and_values_i.first));
      sub_initials.push_back(*(graph_and_values_i.second));
      cout << "Variables in ith subgraph: " << graph_and_values_i.second->size() << endl;
    }

    return make_pair(sub_graphs, sub_initials);
  }

  //*****************************************************************************
  pair<NonlinearFactorGraph, Values>
  loadGraphWithPrior(const string& data_file, const SharedNoiseModel &prior_model) {

    GraphAndValues graph_and_values = readG2o(data_file, true);
    NonlinearFactorGraph graph_centralized = *(graph_and_values.first);
    Values initial_centralized = *(graph_and_values.second);

    // Add prior
    NonlinearFactorGraph graph_with_prior = graph_centralized;
    graph_with_prior.add(PriorFactor<Pose3>(0, Pose3(), prior_model));

    return make_pair(graph_with_prior, initial_centralized);
  }

  //*****************************************************************************

  NonlinearFactorGraph
  convertToChordalGraph(const NonlinearFactorGraph& graph,
                        const SharedNoiseModel &between_noise,
                        const bool& use_between_noise) {
    NonlinearFactorGraph cen_FG;
    for (size_t k = 0; k < graph.size(); k++) {
      boost::shared_ptr<BetweenFactor<Pose3> > factor =
          boost::dynamic_pointer_cast<BetweenFactor<Pose3> >(graph[k]);
      if (factor) {
        Key key1 = factor->keys().at(0);
        Key key2 = factor->keys().at(1);
        Pose3 measured = factor->measured();

        if (use_between_noise) {
          // Convert noise model to chordal factor noise
          SharedNoiseModel chordal_noise = evaluation_utils::convertToChordalNoise(factor->get_noiseModel());
          cen_FG.add(BetweenChordalFactor<Pose3>(key1, key2, measured, chordal_noise));
        } else {
          cen_FG.add(BetweenChordalFactor<Pose3>(key1, key2, measured, between_noise));
        }
      }
    }
    return cen_FG;
  }

  /* ************************************************************************* */
  GaussianFactorGraph
  buildLinearOrientationGraph(const NonlinearFactorGraph &g, const bool& use_between_noise) {

    GaussianFactorGraph linear_graph;
    SharedDiagonal model = noiseModel::Unit::Create(9);

    for (const boost::shared_ptr<NonlinearFactor> &factor: g) {
      Matrix3 Rij;

      boost::shared_ptr<BetweenFactor<Pose3> > pose3_between =
          boost::dynamic_pointer_cast<BetweenFactor<Pose3> >(factor);
      if (pose3_between)
        Rij = pose3_between->measured().rotation().matrix();
      else
        std::cout << "Error in buildLinearOrientationGraph" << std::endl;

      // if using between noise, use the factor noise model converted to a conservative diagonal estimate
      if (use_between_noise) {
        model = evaluation_utils::convertToDiagonalNoise(pose3_between->get_noiseModel());
      }

      const FastVector<Key> &keys = factor->keys();
      Key key1 = keys[0], key2 = keys[1];
      Matrix M9 = Matrix::Zero(9, 9);
      M9.block(0, 0, 3, 3) = Rij;
      M9.block(3, 3, 3, 3) = Rij;
      M9.block(6, 6, 3, 3) = Rij;

      linear_graph.add(key1, -I9, key2, M9, zero9, model);
    }
    // prior on the anchor orientation
    SharedDiagonal prior_model = noiseModel::Unit::Create(9);
    linear_graph.add(key_anchor,
                    I9,
                    (Vector(9) << 1.0, 0.0, 0.0,/*  */ 0.0, 1.0, 0.0, /*  */ 0.0, 0.0, 1.0).finished(),
                    prior_model);
    return linear_graph;
  }

  //*****************************************************************************
  Values
  centralizedEstimation(const NonlinearFactorGraph& graph,
                        const SharedNoiseModel &between_noise,
                        const SharedNoiseModel &prior_noise,
                        const bool& use_between_noise) {

    // STEP 1: solve for rotations first, using chordal relaxation (centralized)
    // This is the "expected" solution
    // pose3_graph only contains between factors, as the priors are converted to between factors on an anchor key
    NonlinearFactorGraph pose3_graph = InitializePose3::buildPose3graph(graph);
    // this will also put a prior on the anchor
    GaussianFactorGraph centralized_linear_graph = buildLinearOrientationGraph(pose3_graph, use_between_noise);
    VectorValues rot_est_centralized = centralized_linear_graph.optimize();
    Values cen_rot = InitializePose3::normalizeRelaxedRotations(rot_est_centralized);
    Values initial = pose3WithZeroTranslation(cen_rot);

    // STEP 2: solve for poses using initial estimate from rotations
    // Linearize pose3 factor graph, using chordal formulation
    NonlinearFactorGraph cen_FG;
    for (size_t k = 0; k < graph.size(); k++) {
      boost::shared_ptr<BetweenFactor<Pose3> > factor =
          boost::dynamic_pointer_cast<BetweenFactor<Pose3> >(graph[k]);
      if (factor) {
        Key key1 = factor->keys().at(0);
        Key key2 = factor->keys().at(1);
        Pose3 measured = factor->measured();

        if (use_between_noise) {
          // Convert noise model to chordal factor noise
          SharedNoiseModel chordal_noise = evaluation_utils::convertToChordalNoise(factor->get_noiseModel());
          cen_FG.add(BetweenChordalFactor<Pose3>(key1, key2, measured, chordal_noise));
        } else {
          cen_FG.add(BetweenChordalFactor<Pose3>(key1, key2, measured, between_noise));
        }
      }
    }

    NonlinearFactorGraph chordal_graph_without_prior = cen_FG.clone();

    // First key
    Key first_key = KeyVector(initial.keys()).at(0);
    cen_FG.add(PriorFactor<Pose3>(first_key, initial.at<Pose3>(first_key), prior_noise));

    Values estimate = initial;
    for (size_t iter = 0; iter < 1; iter++) {
      GaussianFactorGraph cen_GFG = *(cen_FG.linearize(estimate));
      VectorValues cen_pose_vector_values = cen_GFG.optimize(); // optimize
      estimate = retractPose3Global(estimate, cen_pose_vector_values);
    }

    //std::cout << "Centralized Two Stage Error: " << chordal_graph_without_prior.error(estimate) << std::endl;
    return estimate;

  }

  //*****************************************************************************
  Values
  centralizedGNEstimation(const NonlinearFactorGraph& graph,
                          const SharedNoiseModel &between_noise,
                          const SharedNoiseModel &prior_noise,
                          const bool& use_between_noise) {

    // STEP 1: solve for rotations first, using chordal relaxation (centralized)
    // This is the "expected" solution
    // pose3_graph only contains between factors, as the priors are converted to between factors on an anchor key
    NonlinearFactorGraph pose3_graph = InitializePose3::buildPose3graph(graph);

    // this will also put a prior on the anchor
    GaussianFactorGraph centralized_linear_graph = InitializePose3::buildLinearOrientationGraph(pose3_graph);
    VectorValues rot_est_centralized = centralized_linear_graph.optimize();
    Values cen_rot = InitializePose3::normalizeRelaxedRotations(rot_est_centralized);
    Values initial = pose3WithZeroTranslation(cen_rot);

    // STEP 2: solve for poses using initial estimate from rotations
    // Linearize pose3 factor graph, using chordal formulation
    NonlinearFactorGraph cen_FG;
    for (size_t k = 0; k < graph.size(); k++) {
      boost::shared_ptr<BetweenFactor<Pose3> > factor =
          boost::dynamic_pointer_cast<BetweenFactor<Pose3> >(graph[k]);
      if (factor) {
        Key key1 = factor->keys().at(0);
        Key key2 = factor->keys().at(1);
        Pose3 measured = factor->measured();

        if (use_between_noise) {
          // Convert noise model to chordal factor noise
          Rot3 rotation = cen_rot.at<Rot3>(key1);
          SharedNoiseModel
              chordal_noise = evaluation_utils::convertToChordalNoise(factor->get_noiseModel(), rotation.matrix());
          cen_FG.add(BetweenChordalFactor<Pose3>(key1, key2, measured, chordal_noise));
        } else {
          cen_FG.add(BetweenChordalFactor<Pose3>(key1, key2, measured, between_noise));
        }
      }
    }

    NonlinearFactorGraph chordal_graph_without_prior = cen_FG.clone();

    // First key
    // Add prior
    Key first_key = KeyVector(initial.keys()).at(0);
    NonlinearFactor::shared_ptr first_key_prior(new PriorFactor<Pose3>(first_key, initial.at<Pose3>(first_key), prior_noise));
    cen_FG.push_back(first_key_prior);

    Values chordal_GN = initial;
    for (size_t iter = 0; iter < 200; iter++) {
      GaussianFactorGraph chordal_GFG = *(cen_FG.linearize(chordal_GN));
      VectorValues chordal_vector_values = chordal_GFG.optimize(); // optimize
      chordal_GN = retractPose3Global(chordal_GN, chordal_vector_values);
    }

    // std::cout << "Centralized Two Stage + GN Error: " << chordal_graph_without_prior.error(chordal_GN) << std::endl;

    return chordal_GN;
  }

  //*****************************************************************************
  SharedNoiseModel convertToChordalNoise(const SharedNoiseModel& noise, const Matrix& Rhat) {

    // Converted chordal covariance
    Matrix converted_chordal = Matrix::Zero(12, 12);

    // Extract square root information matrix
    SharedGaussian gaussian_noise = boost::dynamic_pointer_cast<noiseModel::Gaussian>(noise);
    Matrix R = gaussian_noise->R();
    Matrix C = gtsam::inverse(trans(R) * R); // get covariance from square root information

    // convert rotation to a conservative isotropic noise model
    double max_C = DBL_MIN;
    for (size_t i = 0; i < 3; i++) {
      if (C(i, i) > max_C)
        max_C = C(i, i);
    }

    converted_chordal(0, 0) = max_C;
    converted_chordal(1, 1) = max_C;
    converted_chordal(2, 2) = max_C;
    converted_chordal(3, 3) = max_C;
    converted_chordal(4, 4) = max_C;
    converted_chordal(5, 5) = max_C;
    converted_chordal(6, 6) = max_C;
    converted_chordal(7, 7) = max_C;
    converted_chordal(8, 8) = max_C;

    // Translation noise is kept as it is
    Matrix translation = C.block(3, 3, 3, 3) + 0.01 * gtsam::eye(3, 3);
    converted_chordal.block(9, 9, 3, 3) = Rhat * translation * trans(Rhat);

    SharedNoiseModel chordal_noise = noiseModel::Diagonal::Gaussian::Covariance(converted_chordal);
    return chordal_noise;
  }

  //*****************************************************************************
  SharedDiagonal convertToDiagonalNoise(const SharedNoiseModel& noise) {

    // Extract square root information matrix
    SharedGaussian gaussian_noise = boost::dynamic_pointer_cast<noiseModel::Gaussian>(noise);
    Matrix R = gaussian_noise->R();
    Matrix C = gtsam::inverse(trans(R) * R); // get covariance from square root information

    // convert rotation to a conservative isotropic noise model
    double max_C = DBL_MIN;
    for (size_t i = 0; i < 3; i++) {
      if (C(i, i) > max_C)
        max_C = C(i, i);
    }

    SharedDiagonal chordal_noise = noiseModel::Diagonal::Sigmas(repeat(9, max_C));
    return chordal_noise;
  }

  //*****************************************************************************
  void
  writeValuesAsTUM(const gtsam::Values& values, const std::string& filename) {

    // Open filestream
    fstream file_stream(filename.c_str(), fstream::out);

    for (const Values::ConstKeyValuePair &key_value: values) {
      Key key = key_value.key;

      // Since TUM rgbd tools are used to compare trajectory poses, do not process object landmarks
      if (isupper(symbolChr(key)))
        continue;

      long unsigned int index = (long unsigned int) symbolIndex(key);
      Pose3 pose = values.at<Pose3>(key);
      Quaternion quat = pose.rotation().toQuaternion();
      Point3 trans = pose.translation();

      char output_time[18];
      sprintf(output_time, "%010lu.000000", index);

      file_stream << output_time << " " << trans.x() << " " << trans.y() << " " << trans.z() << " "
                 << quat.x() << " " << quat.y() << " " << quat.z() << " " << quat.w() << endl;
    }

    // Close filestream
    file_stream.close();
  }

  void copyInitial(const size_t& nr_robots, const std::string& data_dir) {
    cout << "Copying initial to optimized" << endl;
    for (size_t robot = 0; robot < nr_robots; robot++) {
      string data_file_i = data_dir + boost::lexical_cast<string>(robot) + ".g2o";
      GraphAndValues graph_and_value_g2o = readG2o(data_file_i, true);
      NonlinearFactorGraph graph = *(graph_and_value_g2o.first);
      Values initial = *(graph_and_value_g2o.second);

      // Write optimized full graph
      string dist_optimized_i = data_dir + boost::lexical_cast<string>(robot) + "_optimized.g2o";
      writeG2o(graph, initial, dist_optimized_i);
    }
  }


  GraphAndValues readFullGraph(const size_t& nr_robots, // number of robots
                               const vector<GraphAndValues>& graph_and_values_vec  // vector of all graphs and initials for each robot
  ) {
    //std::cout << "Creating full_graph by combining subgraphs." << std::endl;

    // Combined graph and Values
    NonlinearFactorGraph::shared_ptr combined_graph(new NonlinearFactorGraph);
    Values::shared_ptr combined_values(new Values);

    // Iterate over each robot
    for (size_t robot = 0; robot < nr_robots; robot++) {

      // Load graph and initial
      NonlinearFactorGraph graph = *(graph_and_values_vec[robot].first);
      Values initial = *(graph_and_values_vec[robot].second);

      // Iterate over initial and push it to the combined_values, each initial value is present only once
      for (const Values::ConstKeyValuePair &key_value: initial) {
        Key key = key_value.key;
        if (!combined_values->exists(key))
          combined_values->insert(key, initial.at<Pose3>(key));
      }

      // Iterate over the graph and push the factor if it is not already present in combined_graph
      for (size_t ksub = 0; ksub < graph.size(); ksub++) { //for each factor in the new subgraph
        bool is_present = false;
        for (size_t k = 0; k < combined_graph->size(); k++) {

          boost::shared_ptr<BetweenFactor<Pose3>> factor_sub =
              boost::dynamic_pointer_cast<BetweenFactor<Pose3> >(graph.at(ksub));
          Key factor_sub_key1 = factor_sub->key1();
          Key factor_sub_key2 = factor_sub->key2();

          boost::shared_ptr<BetweenFactor<Pose3>> factor_combined =
              boost::dynamic_pointer_cast<BetweenFactor<Pose3> >(combined_graph->at(k));
          Key factor_combined_key1 = factor_combined->key1();
          Key factor_combined_key2 = factor_combined->key2();

          // values don't match exactly that's why check with keys as well
          if (factor_combined->equals(*factor_sub) ||
              ((factor_sub_key1 == factor_combined_key1) && (factor_sub_key2 == factor_combined_key2))) {
            is_present = true;
            break;
          }
        }
        if (is_present == false) // we insert the factor
          combined_graph->add(graph.at(ksub));
      }
    }

    // Return graph and values
    return make_pair(combined_graph, combined_values);
  }


  GraphAndValues readFullGraph(const vector<GraphAndValues>& graph_and_values_vec  // vector of all graphs and initials for each robot
  ) {
    //std::cout << "Creating full_graph by combining subgraphs." << std::endl;

    // Combined graph and Values
    NonlinearFactorGraph::shared_ptr combined_graph(new NonlinearFactorGraph);
    Values::shared_ptr combined_values(new Values);

    // Iterate over each robot
    for (size_t robot = 0; robot < graph_and_values_vec.size(); robot++) {

      // Load graph and initial
      NonlinearFactorGraph graph = *(graph_and_values_vec[robot].first);
      Values initial = *(graph_and_values_vec[robot].second);

      // Iterate over initial and push it to the combined_values, each initial value is present only once
      for (const Values::ConstKeyValuePair &key_value: initial) {
        Key key = key_value.key;
        if (!combined_values->exists(key))
          combined_values->insert(key, initial.at<Pose3>(key));
      }

      // Iterate over the graph and push the factor if it is not already present in combined_graph
      for (size_t ksub = 0; ksub < graph.size(); ksub++) { //for each factor in the new subgraph
        bool is_present = false;
        for (size_t k = 0; k < combined_graph->size(); k++) {

          boost::shared_ptr<BetweenFactor<Pose3>> factor_sub =
              boost::dynamic_pointer_cast<BetweenFactor<Pose3> >(graph.at(ksub));
          Key factor_sub_key1 = factor_sub->key1();
          Key factor_sub_key2 = factor_sub->key2();

          boost::shared_ptr<BetweenFactor<Pose3>> factor_combined =
              boost::dynamic_pointer_cast<BetweenFactor<Pose3> >(combined_graph->at(k));
          Key factor_combined_key1 = factor_combined->key1();
          Key factor_combined_key2 = factor_combined->key2();

          // values don't match exactly that's why check with keys as well
          if (factor_combined->equals(*factor_sub) ||
              ((factor_sub_key1 == factor_combined_key1) && (factor_sub_key2 == factor_combined_key2))) {
            is_present = true;
            break;
          }
        }
        if (is_present == false) // we insert the factor
          combined_graph->add(graph.at(ksub));
      }
    }

    // Return graph and values
    return make_pair(combined_graph, combined_values);
  }

  std::tuple<double, double, double> evaluateEstimates(const size_t &nr_robots,
                                              const gtsam::GraphAndValues &full_graph_and_values,
                                              const gtsam::noiseModel::Diagonal::shared_ptr &prior_model,
                                              const gtsam::noiseModel::Isotropic::shared_ptr &model,
                                              const bool &use_between_noise,
                                              const gtsam::Values &distributed_estimates,
                                              const bool& debug) {
    ////////////////////////////////////////////////////////////////////////////////
    // Extract full graph and add prior
    ////////////////////////////////////////////////////////////////////////////////
    NonlinearFactorGraph full_graph = *(full_graph_and_values.first);
    Values full_initial = *(full_graph_and_values.second);

    // Add prior
    NonlinearFactorGraph full_graph_with_prior = full_graph.clone();
    Key prior_key = KeyVector(full_initial.keys()).at(0);
    NonlinearFactor::shared_ptr prior(
        new PriorFactor<Pose3>(prior_key, full_initial.at<Pose3>(prior_key), prior_model));
    full_graph_with_prior.push_back(prior);

    ////////////////////////////////////////////////////////////////////////////////
    // Chordal Graph
    ////////////////////////////////////////////////////////////////////////////////
    NonlinearFactorGraph chordal_graph = convertToChordalGraph(
        full_graph, model, use_between_noise);

    ////////////////////////////////////////////////////////////////////////////////
    // Initial Error
    ////////////////////////////////////////////////////////////////////////////////
    double initial_error = chordal_graph.error(full_initial);
    if (debug) {
      std::cout << "Initial Error: " << initial_error << std::endl;
    }

    ////////////////////////////////////////////////////////////////////////////////
    // Centralized Two Stage
    ////////////////////////////////////////////////////////////////////////////////
    Values centralized = centralizedEstimation(full_graph_with_prior,
                                                                   model, prior_model,
                                                                   use_between_noise);

    //std::string dataset_file_name = "log/datasets/centralized.g2o";
    //gtsam::writeG2o(full_graph_with_prior, centralized, dataset_file_name);
    double centralized_error = chordal_graph.error(centralized);
    if (debug) {
      std::cout << "Centralized Two Stage Error: " << centralized_error << std::endl;
    }

    ////////////////////////////////////////////////////////////////////////////////
    // Centralized Two Stage + Gauss Newton
    ////////////////////////////////////////////////////////////////////////////////
    Values chordal_GN = centralizedGNEstimation(full_graph_with_prior,
                                                                   model, prior_model,
                                                                   use_between_noise);
    if (debug) {                                                               
      std::cout << "Centralized Two Stage + GN Error: " << chordal_graph.error(chordal_GN) << std::endl;
    }

    ////////////////////////////////////////////////////////////////////////////////
    // Distributed Error
    ////////////////////////////////////////////////////////////////////////////////
    double distributed_error = chordal_graph.error(distributed_estimates);
    if (debug) {
      std::cout << "Distributed Error: " << distributed_error << std::endl;
    }

    return std::make_tuple(centralized_error, distributed_error, initial_error);
  }

  std::pair<Values, Values> centralizedEstimates(const gtsam::GraphAndValues &full_graph_and_values,
                                const gtsam::noiseModel::Diagonal::shared_ptr &prior_model,
                                const gtsam::noiseModel::Isotropic::shared_ptr &model,
                                const bool &use_between_noise) {
    ////////////////////////////////////////////////////////////////////////////////
    // Extract full graph and add prior
    ////////////////////////////////////////////////////////////////////////////////
    NonlinearFactorGraph full_graph = *(full_graph_and_values.first);
    Values full_initial = *(full_graph_and_values.second);

    // Add prior
    NonlinearFactorGraph full_graph_with_prior = full_graph.clone();
    Key prior_key = KeyVector(full_initial.keys()).at(0);
    NonlinearFactor::shared_ptr prior(
        new PriorFactor<Pose3>(prior_key, full_initial.at<Pose3>(prior_key), prior_model));
    full_graph_with_prior.push_back(prior);

    ////////////////////////////////////////////////////////////////////////////////
    // Centralized Two Stage
    ////////////////////////////////////////////////////////////////////////////////
    Values centralized = centralizedEstimation(full_graph_with_prior,
                                                model, prior_model,
                                                use_between_noise);

    ////////////////////////////////////////////////////////////////////////////////
    // Centralized Two Stage + Gauss Newton
    ////////////////////////////////////////////////////////////////////////////////
    Values chordal_GN = centralizedGNEstimation(full_graph_with_prior,
                                                model, prior_model,
                                                use_between_noise);

    return std::make_pair(centralized, chordal_GN);
  }
}
}

