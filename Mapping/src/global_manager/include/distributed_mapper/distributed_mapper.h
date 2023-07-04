#pragma once

#include <distributed_mapper/evaluation_utils.h>
#include <distributed_mapper/between_chordal_factor.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/nonlinear/GaussNewtonOptimizer.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/linear/GaussianFactorGraph.h>
#include <gtsam/linear/VectorValues.h>
#include <gtsam/base/Value.h>
#include <gtsam/base/timing.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Rot3.h>
#include <gtsam/slam/dataset.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/InitializePose3.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/inference/graph.h>

/** @ingroup distributed_mapper */
namespace distributed_mapper{


// Static Consts
static const gtsam::Matrix I9 = gtsam::eye(9);
static const gtsam::Vector zero9 = gtsam::Vector::Zero(9);
static const size_t max_iter_ = 1000;
static const gtsam::Key key_anchor_ = gtsam::symbol('Z', 9999999);

/**
 * @brief The DistributedMapper class runs distributed mapping algorithm
 *  for each robot separately. Each robot has access to it's subgraph and neighboring measurements
 */
class DistributedMapper{

  public:
    /**
     * @brief DistributedMapper constructor
     */
    DistributedMapper(const char& robotName, const bool& use_chr_less_full_graph = false, const bool& use_flagged_init = false){
      // Config
      verbosity_ = SILENT;
      robotName_ = robotName;
      rotation_noise_model_ = gtsam::noiseModel::Isotropic::Variance(9, 1);
      pose_noise_model_ = gtsam::noiseModel::Isotropic::Variance(12, 1);
      graph_ = gtsam::NonlinearFactorGraph();
      chordal_graph_ = gtsam::NonlinearFactorGraph();
      rot_subgraph_ = gtsam::GaussianFactorGraph();
      initial_ = gtsam::Values();
      neighbors_ = gtsam::Values();
      loopclosures_symbols_ = std::vector<std::pair<gtsam::Symbol, gtsam::Symbol>>();
      rotation_error_trace_ = std::vector<double>();
      pose_error_trace_ = std::vector<double>();
      rotation_estimate_change_trace_ = std::vector<double>();
      pose_estimate_change_trace_ = std::vector<double>();
      centralized_values_ = gtsam::Values();
      use_flagged_init_ = use_flagged_init;
      use_chr_less_full_graph_ = use_chr_less_full_graph;
      update_type_ = incUpdate;
      gamma_ = 1.0f;
      use_between_noise_ = false;
      use_landmarks_ = false;
      latest_change_ = DBL_MAX;
    }

    /**
     * Reinitialize the optimizer.
     */
    void reinit() {
      // Config
      rotation_noise_model_ = gtsam::noiseModel::Isotropic::Variance(9, 1);
      pose_noise_model_ = gtsam::noiseModel::Isotropic::Variance(12, 1);
      graph_ = gtsam::NonlinearFactorGraph();
      chordal_graph_ = gtsam::NonlinearFactorGraph();
      rot_subgraph_ = gtsam::GaussianFactorGraph();
      initial_ = gtsam::Values();
      neighbors_ = gtsam::Values();
      loopclosures_symbols_ = std::vector<std::pair<gtsam::Symbol, gtsam::Symbol>>();
      rotation_error_trace_ = std::vector<double>();
      pose_error_trace_ = std::vector<double>();
      rotation_estimate_change_trace_ = std::vector<double>();
      pose_estimate_change_trace_ = std::vector<double>();
      centralized_values_ = gtsam::Values();
      update_type_ = incUpdate;
      gamma_ = 1.0f;
      use_between_noise_ = false;
      use_landmarks_ = false;
      latest_change_ = DBL_MAX;
    }

    /**
     * Reset latest change control attribute
     */
    void resetLatestChange() {
      latest_change_ = DBL_MAX;
    }

    /** Set the flag whether to use landmarks or not */
    void setUseLandmarksFlag(const bool& use_landmarks){
      use_landmarks_ = use_landmarks;
    }

    /** Set the flag whether to use between noise or not */
    void setUseBetweenNoiseFlag(const bool& use_between_noise){
      use_between_noise_ = use_between_noise;
    }

    /** updateType differetiates between Distributed Jacobi/Jacobi OverRelaxation (postUpdate)
     *  and Gauss-Seidel/Successive OverRelaxation (incUpdate) */
    enum UpdateType{postUpdate, incUpdate};

    /** @brief setUpdateType sets the update type */
    void setUpdateType(const UpdateType& update_type){update_type_ = update_type;}

    /** @brief setGamma sets the gamma value for over relaxation methods
     *  Distributed Jacobi: update_type_ = postUpdate, gamma = 1
     *  Gauss Seidel: update_type_ = incUpdate, gamma = 1
     *  Jacobi Overrelax: update_type_ = postUpdate, gamma != 1
     *  Succ Overrelax: update_type_ = incUpdate, gamma != 1
     */
    void setGamma(const double& gamma){gamma_ = gamma;}

    /**
    * @brief createSubgraphInnerAndSepEdges splits the input subgraph into inner edge factors and seperator edge ids
    * @param subgraph is the input Nonlinear factor graph
    * @return pair of subgraphInnerEdge and subgraphsSepEdgesId
    */
    std::pair< gtsam::NonlinearFactorGraph, std::vector<size_t> >
    createSubgraphInnerAndSepEdges(const gtsam::NonlinearFactorGraph& subgraph);

    /**
     * @brief loadSubgraphsAndCreateSubgraphEdges loads the subgraph graphAndValues and creates inner and loopclosure edges
     * @param graphAndValues contains the current subgraph and loopclosure edges
     */
    void loadSubgraphAndCreateSubgraphEdge(const gtsam::GraphAndValues& graph_and_values);

    /** @brief createLinearOrientationGraph creates orientation graph for distributed rotation estimation */
    void createLinearOrientationGraph();

    /** @brief addPriorToSubgraph adds prior to a subgraph "id" at particular symbol "sym"  */
    void
    addPrior(const gtsam::Symbol& sym,  const gtsam::Pose3& prior_pose, const gtsam::SharedNoiseModel& prior_model){
      gtsam::NonlinearFactor::shared_ptr factor(new gtsam::PriorFactor<gtsam::Pose3>(sym, prior_pose, prior_model));
      graph_.push_back(factor);
      inner_edges_.add(factor);
      chordal_graph_.add(factor);
      // recreate orientation graph of inner edges, this time with prior (included in inner_edges_)
      createLinearOrientationGraph();
    }

    /** @brief removePrior removes the prior factor in the graph  */
    void
    removePrior(){
      for(size_t k=0; k < graph_.size(); k++){
        if(!graph_.at(k))continue;
        gtsam::KeyVector keys = graph_.at(k)->keys();
        if (keys.size() != 2){
          boost::shared_ptr<gtsam::PriorFactor<gtsam::Pose3> > pose3_prior =
              boost::dynamic_pointer_cast<gtsam::PriorFactor<gtsam::Pose3> >(graph_.at(k));
          if (pose3_prior){
            graph_.remove(k);
            break;
          }
        }
      }
      for(size_t k=0; k < inner_edges_.size(); k++){
        if(!inner_edges_.at(k))continue;
        gtsam::KeyVector keys = inner_edges_.at(k)->keys();
        if (keys.size() != 2){
          boost::shared_ptr<gtsam::PriorFactor<gtsam::Pose3> > pose3_prior =
              boost::dynamic_pointer_cast<gtsam::PriorFactor<gtsam::Pose3> >(inner_edges_.at(k));
          if (pose3_prior){
            inner_edges_.remove(k);
            chordal_graph_.remove(k);
            break;
          }
        }
      }      
      createLinearOrientationGraph(); // linear orientation graph
    }

    /** @brief chordalFactorGraph generates a graph of BetweenChordalFactors using innerEdges required for distributed pose estimation*/
    void chordalFactorGraph();

    /** @brief estimateRotation estimates rotation using distributed mapping algorithm */
    void estimateRotation();

    /** @brief estimatePoses estimates poses using distributed mapping given initial rotation estimates */
    void estimatePoses();

    /** @brief estimateRotation followed by estimatePoses */
    void optimize();

    /** @brief innerEdges returns the inner edges  */
    gtsam::NonlinearFactorGraph innerEdges(){ return inner_edges_; }

    /** @brief loopclosureEdges returns indices of loopclosure edges  */
    std::vector<size_t> loopclosureEdge(){ return loopclosure_edge_ids_; }

    /** @brief subgraphs */
    gtsam::NonlinearFactorGraph currentGraph(){ return graph_; }

    /** @brief neighbors returns the neighboring values  */
    gtsam::Values neighbors() {return neighbors_;}

    /** @brief neighbors returns the loopclosures factor symbols  */
    std::vector<std::pair<gtsam::Symbol, gtsam::Symbol>> separatorsSymbols() {return loopclosures_symbols_;}

    /** @brief erase loopclosures factor symbols  */
    void eraseseparatorsSymbols(const std::pair<gtsam::Symbol, gtsam::Symbol>& symbols) {
      auto it = std::find(loopclosures_symbols_.begin(), loopclosures_symbols_.end(), symbols);
      if (it != loopclosures_symbols_.end()) {
        loopclosures_symbols_.erase(it);
      }
    }

    /** @brief allows to remove a factor from the graph.
     *  @param index is the factor index in the graph
     */
    void removeFactor(const int& index){
        graph_.remove(index);
    }

    /** @brief allows to erase a factor from the graph.
     *  @param index is the factor index in the graph
     */
    void eraseFactor(const int& index){
        graph_.erase(graph_.begin()+index);
    }

    /** @brief allows to erase a loopclosure ID.
     *  @param id is the id to be removed
     */
    void setloopclosureIds(const std::vector<size_t>& ids){
        loopclosure_edge_ids_ = ids;
    }

    /** @brief allows to erase a loopclosure ID.
     *  @param id is the id to be removed
     */
    void eraseloopclosureId(const int& id){
        loopclosure_edge_ids_.erase(find(loopclosure_edge_ids_.begin(), loopclosure_edge_ids_.end(), id));
    }

    /**
     * @brief insertValue updates inneer nodes with the new value
     * @param sym is symbol
     * @param pose is the pose
     */
    void insertValue(const gtsam::Key& key, const gtsam::Pose3& pose){
      // Update the value if symbol already exists
      if(initial_.exists(key)){
        initial_.update(key, pose);
      }
      else{
        initial_.insert(key, pose);
      }
      linearized_rotation_ = evaluation_utils::rowMajorVectorValues(initial_);
    }

    /**removePrior
     * @brief updateGraph adds new factor to the graph
     * @param factor is the input factor
     */
    void addFactor(const gtsam::NonlinearFactor::shared_ptr& factor){
      graph_.push_back(factor);
      gtsam::KeyVector keys = factor->keys();
      if(gtsam::symbolChr(keys.at(0)) == gtsam::symbolChr(keys.at(1))){
        inner_edges_.push_back(factor);

        // Chordal factor
        boost::shared_ptr<gtsam::BetweenFactor<gtsam::Pose3> > between =
            boost::dynamic_pointer_cast<gtsam::BetweenFactor<gtsam::Pose3> >(factor);
        gtsam::Key key1 = between->keys().at(0);
        gtsam::Key key2 = between->keys().at(1);
        gtsam::Pose3 measured = between->measured();
        chordal_graph_.add(gtsam::BetweenChordalFactor<gtsam::Pose3>(key1, key2, measured, pose_noise_model_));

        // Linear orientation graph
        createLinearOrientationGraph(); // TODO: Rebuilds entire linear orientation graph everytime a factor is added
      }
      else{
        loopclosure_edge_ids_.push_back(graph_.size() -1);
      }

      // Clear traces
      rotation_error_trace_.clear();
      pose_error_trace_.clear();
    }

    /**
     * @brief updateNeighbor updates neighboring nodes with the new value
     * @param sym is symbol
     * @param pose is the pose
     */
    void updateNeighbor(const gtsam::Key& key, const gtsam::Pose3& pose){
      // Update the value if symbol already exists
      if(neighbors_.exists(key)){
        neighbors_.update(key, pose);
      }
      else{
        neighbors_.insert(key, pose);
        neighbors_linearized_poses_.insert(key, gtsam::zero(6));
        gtsam::Matrix3 R = pose.rotation().matrix();
        gtsam::Vector r = evaluation_utils::rowMajorVector(R);
        neighbors_linearized_rotations_.insert(key, r);
      }
    }

    /**
     * @brief updateNeighborLinearizedPoses updates neighboring node vector_values with the new vector_values
     * @param sym is symbol
     * @param vector_value is the new vector_value
     */
    void updateNeighborLinearizedPoses(const gtsam::Key& key, const gtsam::Vector& vector_value){
      // Update the value if symbol already exists
      if(neighbors_linearized_poses_.exists(key)){
        neighbors_linearized_poses_.at(key) = vector_value;
        neighbors_updated_[key] = true;
      }
    }

    /**
     * @brief updateNeighborLinearizedRotations updates neighboring node vector_values with the new vector_values
     * @param sym is symbol
     * @param vector_value is the new vector_value
     */
    void updateNeighborLinearizedRotations(const gtsam::Key& key, const gtsam::Vector& vector_value){
      // Update the value if symbol already exists
      if(neighbors_linearized_rotations_.exists(key)){
        neighbors_linearized_rotations_.at(key) = vector_value;
        neighbors_updated_[key] = true;
      }
    }

    /** @brief linearizedPoses returns the linearized poses */
    gtsam::VectorValues linearizedPoses(){ return linearized_poses_;}

    /** @brief linearizedPosesAt returns the current pose estimate at sym */
    gtsam::Vector linearizedPosesAt(const gtsam::Key& key){ return linearized_poses_.at(key); }

    /** @brief retractPose3Global performs global retraction using linearizedPoses and initial */
    void retractPose3Global(){
      initial_ = evaluation_utils::retractPose3Global(initial_, linearized_poses_);
    }

    /** @brief retractPose3Global performs global retraction using linearizedPoses and initial */
    void retractPose3GlobalWithOffset(const gtsam::Point3& offset){
      initial_ = evaluation_utils::retractPose3GlobalWithOffset(initial_, linearized_poses_, offset);
    }

    /** @brief linearizedRotationAt returns the current rotation estimate at sym */
    gtsam::Vector linearizedRotationAt(const gtsam::Key& key){ return linearized_rotation_.at(key); }

    /** @brief returns *latest* linear rotation estimate for neighbors */
    gtsam::Vector neighborsLinearizedRotationsAt(const gtsam::Key& key){ return neighbors_linearized_rotations_.at(key); }


    /** @brief convertLinearizedRotationToPoses iterates over linearized rotations and convert them to poses with zero translation  */
    void convertLinearizedRotationToPoses(){
      gtsam::Values rotValue = gtsam::InitializePose3::normalizeRelaxedRotations(linearized_rotation_);
      initial_ = evaluation_utils::pose3WithZeroTranslation(rotValue);
      linearized_poses_ = evaluation_utils::initializeVectorValues(initial_); // Init linearized poses
      dist_GFG_ = *(chordal_graph_.linearize(initial_));

      // Initial error
      //double error = dist_GFG_.error(linearized_poses_);
      //pose_error_trace_.push_back(error);

    }

    /** @brief estimateAt returns the current estimate at sym */
    gtsam::Pose3 estimateAt(const gtsam::Key& key){ return initial_.at<gtsam::Pose3>(key); }

    /** @brief returns the current estimate */
    gtsam::Values currentEstimate(){ return initial_; }

    /** @brief returns the current estimate */
    int numberOfPosesInCurrentEstimate(){ return initial_.size(); }

    /** @brief returns the robot name */
    char robotName(){ return robotName_; }

    /** @brief getConvertedEstimate converts the current estimate of the subgraph to follow global symbolChr-less indexing */
    gtsam::Values getConvertedEstimate(const gtsam::Values& initial){
      gtsam::Values converted_estimate;
      for(gtsam::Values::ConstKeyValuePair key_value: initial) {
        gtsam::Symbol key = key_value.key;
        if(use_chr_less_full_graph_){
          int index = gtsam::symbolIndex(key);
          converted_estimate.insert(index, initial.at<gtsam::Pose3>(key));
        }
        else{
          converted_estimate.insert(key, initial.at<gtsam::Pose3>(key));
        }
      }
      return converted_estimate;
    }


    /**
     * @brief updateEstimate updates linearizedRotation according to gamma and old linearized rotation.
     */
    void updateRotation(){
      for(gtsam::VectorValues::KeyValuePair& key_value: new_linearized_rotation_){
        gtsam::Key key = key_value.first;
        if(!linearized_rotation_.exists(key)){
          linearized_rotation_.insert(key, new_linearized_rotation_.at(key));
        }
        else{
          linearized_rotation_.at(key) = (1-gamma_)*linearized_rotation_.at(key) + gamma_*new_linearized_rotation_.at(key);
        }
      }
    }

    /**
     * @brief updatePoses updes linearized_poses_ according to gamma and old linearized poses.
     */
    void updatePoses(){
      for(gtsam::VectorValues::KeyValuePair& key_value: new_linearized_poses_){
        gtsam::Key key = key_value.first;
        if(!linearized_poses_.exists(key)){
          linearized_poses_.insert(key, new_linearized_poses_.at(key));
        }
        else{
          linearized_poses_.at(key) = (1-gamma_)*linearized_poses_.at(key) + gamma_*new_linearized_poses_.at(key);
        }
      }
    }


    /** Verbosity defines the verbosity levels */
    enum Verbosity{SILENT, // Print or output nothing
                   ERROR, // Save error traces
                   DEBUG // Also print debug statements
                  };


    /** @brief setVerbosity sets the verbosity level */
    void setVerbosity(const Verbosity& verbosity){verbosity_ = verbosity;}

    /** @brief trace returns the trace */
    std::pair<std::vector<double>, std::vector<double> > trace()
    {return std::make_pair(rotation_error_trace_, pose_error_trace_);}

    /** @brief trace returns the trace */
    std::pair<double, double> latestError()
    {
      double rotation_error = 0;
      double pose_error = 0;
      if (!rotation_error_trace_.empty()) {
        rotation_error = rotation_error_trace_.back();
      }
      if (!pose_error_trace_.empty()) {
        pose_error = pose_error_trace_.back();
      }
      return std::make_pair(rotation_error, pose_error);
    }

    /** @brief traceEstimateChange returns the trace of change in estimate */
    std::pair<std::vector<double>, std::vector<double> > traceEstimateChange()
    {return std::make_pair(rotation_estimate_change_trace_, pose_estimate_change_trace_);}

    /** @brief log centralized estimate error for plotting */
    std::pair<double, double> logCentralizedError(const gtsam::Values& centralized){
      centralized_values_.clear();
      for(const gtsam::Values::KeyValuePair& key_value: initial_) {
        gtsam::Symbol key = key_value.key;
        int index = gtsam::symbolIndex(key);
        gtsam::Symbol new_key(robotName_, index);
        gtsam::Pose3 estimate;
        if(centralized.exists(index)){
          estimate = centralized.at<gtsam::Pose3>(index);
        }
        else{
          estimate = centralized.at<gtsam::Pose3>(key);
          }
        if(use_landmarks_){
            centralized_values_.insert(key, estimate);
          }
        else{
            centralized_values_.insert(new_key, estimate);
          }
      }
      return std::make_pair(inner_edges_.error(centralized_values_),
                            inner_edges_.error(initial_));
      }

    /** @brief latestChange returns the latest change in estimate */
    double latestChange(){
      return latest_change_;
    }

    /**
    * @brief updateRobotInitialized
    * @param robot
    * @param flag
    */
    void updateNeighboringRobotInitialized(const char& robot, const bool& flag){
      if(neighboring_robots_initialized_.find(robot) != neighboring_robots_initialized_.end()){
        neighboring_robots_initialized_.at(robot) = flag;
      }
      else{
        neighboring_robots_initialized_.insert(std::make_pair(robot, flag));
      }
    }

    /**
    * @brief clearNeighboringRobotInit
    */
    void clearNeighboringRobotInit(){
      neighboring_robots_initialized_.clear();
    }

    /**
    * @brief getNeighboringRobotsInit
    */
    std::map<char, bool> getNeighboringRobotsInit(){
      return neighboring_robots_initialized_;
    }

    /**
     * @brief isRobotInitialized
     * @return
     */
    bool isRobotInitialized(){
      return robot_initialized_;
    }

    /**
     * @brief updateInitialized
     * @param flag
     */
    void updateInitialized(const bool& flag){
      robot_initialized_ = flag;
    }

    /**
     * @brief setFlaggedInit
     * @param flag
     */
    void setFlaggedInit(const bool& flag){
      use_flagged_init_ = flag;
    }

    /**
     * @brief getNeighboringChars returns the set of neighboring robot symbol chars
     */
    std::set<char> getNeighboringChars(){
      return neighbor_chars_;
    }

    bool use_chr_less_full_graph_; // full graph pose have no key characters if it is true.

    // UpdateType and Gamma
    UpdateType update_type_;
    double gamma_;


  protected:
    bool debug_; // Debug flag
    gtsam::noiseModel::Diagonal::shared_ptr rotation_noise_model_;
    gtsam::noiseModel::Isotropic::shared_ptr pose_noise_model_;

    char robotName_;// Key for each robot
    gtsam::NonlinearFactorGraph graph_; // subgraph corresponding to each robot
    gtsam::Values initial_; // subinitials corresponding to each robot        
    gtsam::NonlinearFactorGraph inner_edges_; // edges involving keys from a single robot (exclude loopclosure edges)
    std::vector<size_t>  loopclosure_edge_ids_; // for each robot stores the position of the factors corresponding to loopclosure edges
    gtsam::Values neighbors_; // contains keys of all the neighboring robots
    std::vector<std::pair<gtsam::Symbol, gtsam::Symbol>> loopclosures_symbols_; // contains keys of all the loopclosures
    std::set<char> neighbor_chars_; // contains neighboring robot symbols
    double latest_change_; // Latest change in estimate, stopping condition
    bool use_between_noise_; // To use the between factor noise instead of isotropic unit noise during pose estimation

    // Cached values and graph required for fast optimization and communication
    gtsam::VectorValues linearized_rotation_; // contains vector values of rotation of internal nodes
    gtsam::VectorValues new_linearized_rotation_; // contains vector values of rotation of internal nodes after current iteration
    gtsam::VectorValues neighbors_linearized_rotations_; // contains vector values of all the neighboring robots for distributed estimation
    std::map<gtsam::Key, bool> neighbors_updated_;
    gtsam::VectorValues linearized_poses_; // contains vector values of poses of internal nodes
    gtsam::VectorValues new_linearized_poses_; // contains vector values of poses of internal nodes after current iteration
    gtsam::VectorValues neighbors_linearized_poses_; // contains vector values of all the neighboring robots for distributed estimation
    gtsam::NonlinearFactorGraph chordal_graph_; // edges involving keys from a single robot represented using BetweenChordalFactors
    gtsam::GaussianFactorGraph rot_subgraph_; // linear orientation graph required for distributed rotation estimation
    gtsam::GaussianFactorGraph dist_GFG_; // Gaussian factor graph initialized before distributed pose estimation

    // Initialization
    std::map<char, bool> neighboring_robots_initialized_; // contains boolean flag to check if a robot is initialized or not
    bool robot_initialized_; // flag to check if this robot is initialized or not
    bool use_flagged_init_; // flagged initialization

    // Landmarks
    bool use_landmarks_; // use landmarks -- landmarks are given symbols as upper case of robot name, for eg: if robot is 'a', landmark will be 'A'

    // Verbosity
    std::vector <double> rotation_error_trace_; // error trace
    std::vector <double> pose_error_trace_; // error trace
    std::vector <double> rotation_estimate_change_trace_; // change in estimate trace
    std::vector <double> pose_estimate_change_trace_; // change in estimate trace
    double centralized_error_; // log it for plotting
    gtsam::Values centralized_values_; // centralized estimate converted to the estimate format of this graph
    Verbosity verbosity_; // Verbosity level
};

}
