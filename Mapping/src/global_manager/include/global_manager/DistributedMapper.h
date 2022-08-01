#pragma once

#include <global_manager/MultiRobotUtils.h>
#include <global_manager/BetweenChordalFactor.h>
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

namespace distributed_mapper{


// Static Consts
static const gtsam::Matrix I9 = gtsam::eye(9);
static const gtsam::Vector zero9 = gtsam::Vector::Zero(9);
static const size_t maxIter_ = 1000;
static const gtsam::Key keyAnchor = gtsam::symbol('Z', 9999999);

/**
 * @brief The DistributedMapper class runs distributed mapping algorithm
 *  for each robot separately. Each robot has access to it's subgraph and neighboring measurements
 */
class DistributedMapper{

  public:
    /**
     * @brief DistributedMapper constructor
     */
    DistributedMapper(char robotName, bool useChrLessFullGraph = false, bool useFlaggedInit = false){
      // Config
      verbosity_ = ERROR;
      robotName_ = robotName;
      rotationNoiseModel_ = gtsam::noiseModel::Isotropic::Variance(9, 1);
      poseNoiseModel_ = gtsam::noiseModel::Isotropic::Variance(12, 1);
      graph_ = gtsam::NonlinearFactorGraph();
      chordalGraph_ = gtsam::NonlinearFactorGraph();
      rotSubgraph_ = gtsam::GaussianFactorGraph();
      initial_ = gtsam::Values();
      neighbors_ = gtsam::Values();
      rotationErrorTrace_ = std::vector<double>();
      poseErrorTrace_ = std::vector<double>();
      rotationEstimateChangeTrace_ = std::vector<double>();
      poseEstimateChangeTrace_ = std::vector<double>();
      centralizedValues_ = gtsam::Values();
      useFlaggedInit_ = useFlaggedInit;
      useChrLessFullGraph_ = useChrLessFullGraph;
      updateType_ = incUpdate;
      gamma_ = 1.0f;
      useBetweenNoise_ = false;
      useLandmarks_ = false;
      latestChange_ = DBL_MAX;
    }


    /** Set the flag whether to use landmarks or not */
    void setUseLandmarksFlag(bool useLandmarks){
      useLandmarks_ = useLandmarks;
    }

    /** Set the flag whether to use between noise or not */
    void setUseBetweenNoiseFlag(bool useBetweenNoise){
      useBetweenNoise_ = useBetweenNoise;
    }

    /** updateType differetiates between Distributed Jacobi/Jacobi OverRelaxation (postUpdate)
     *  and Gauss-Seidel/Successive OverRelaxation (incUpdate) */
    enum UpdateType{postUpdate, incUpdate};

    /** @brief setUpdateType sets the update type */
    void setUpdateType(UpdateType updateType){updateType_ = updateType;}

    /** @brief setGamma sets the gamma value for over relaxation methods
     *  Distributed Jacobi: updateType_ = postUpdate, gamma = 1
     *  Gauss Seidel: updateType_ = incUpdate, gamma = 1
     *  Jacobi Overrelax: updateType_ = postUpdate, gamma != 1
     *  Succ Overrelax: updateType_ = incUpdate, gamma != 1
     */
    void setGamma(double gamma){gamma_ = gamma;}

    /**
    * @brief createSubgraphInnerAndSepEdges splits the input subgraph into inner edge factors and seperator edge ids
    * @param subgraph is the input Nonlinear factor graph
    * @return pair of subgraphInnerEdge and subgraphsSepEdgesId
    */
    std::pair< gtsam::NonlinearFactorGraph, std::vector<size_t> >
    createSubgraphInnerAndSepEdges(const gtsam::NonlinearFactorGraph& subgraph);

    /**
     * @brief loadSubgraphsAndCreateSubgraphEdges loads the subgraph graphAndValues and creates inner and separator edges
     * @param graphAndValues contains the current subgraph and separator edges
     */
    void loadSubgraphAndCreateSubgraphEdge(gtsam::GraphAndValues graphAndValues);

    /** @brief createLinearOrientationGraph creates orientation graph for distributed rotation estimation */
    void createLinearOrientationGraph();

    /** @brief addPriorToSubgraph adds prior to a subgraph "id" at particular symbol "sym"  */
    void
    addPrior(gtsam::Symbol sym,  gtsam::Pose3 priorPose, const gtsam::SharedNoiseModel& priorModel){
      gtsam::NonlinearFactor::shared_ptr factor(new gtsam::PriorFactor<gtsam::Pose3>(sym, priorPose, priorModel));
      graph_.push_back(factor);
      innerEdges_.add(factor);
      chordalGraph_.add(factor);
      // recreate orientation graph of inner edges, this time with prior (included in innerEdges_)
      createLinearOrientationGraph();
    }

    /** @brief removePrior removes the prior factor in the graph  */
    void
    removePrior(){
      for(size_t k=0; k < graph_.size(); k++){
        if(!graph_.at(k))continue;
        gtsam::KeyVector keys = graph_.at(k)->keys();
        if (keys.size() != 2){
          boost::shared_ptr<gtsam::PriorFactor<gtsam::Pose3> > pose3Prior =
              boost::dynamic_pointer_cast<gtsam::PriorFactor<gtsam::Pose3> >(graph_.at(k));
          if (pose3Prior){
            graph_.remove(k);
            innerEdges_.remove(k);
            chordalGraph_.remove(k);
            createLinearOrientationGraph(); // linear orientation graph
            break;
          }
        }
      }
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
    gtsam::NonlinearFactorGraph innerEdges(){ return innerEdges_; }

    /** @brief separatorEdges returns indices of separator edges  */
    std::vector<size_t> seperatorEdge(){ return separatorEdgeIds_; }

    /** @brief subgraphs */
    gtsam::NonlinearFactorGraph currentGraph(){ return graph_; }

    /** @brief neighbors returns the neighboring values  */
    gtsam::Values neighbors() {return neighbors_;}

    /**
     * @brief insertValue updates inneer nodes with the new value
     * @param sym is symbol
     * @param pose is the pose
     */
    void insertValue(gtsam::Key key, gtsam::Pose3 pose){
      // Update the value if symbol already exists
      if(initial_.exists(key)){
        initial_.update(key, pose);
      }
      else{
        initial_.insert(key, pose);
      }
      linearizedRotation_ = multirobot_util::rowMajorVectorValues(initial_);
    }

    /**removePrior
     * @brief updateGraph adds new factor to the graph
     * @param factor is the input factor
     */
    void addFactor(gtsam::NonlinearFactor::shared_ptr& factor){
      graph_.push_back(factor);
      gtsam::KeyVector keys = factor->keys();
      if(gtsam::symbolChr(keys.at(0)) == gtsam::symbolChr(keys.at(1))){
        innerEdges_.push_back(factor);

        // Chordal factor
        boost::shared_ptr<gtsam::BetweenFactor<gtsam::Pose3> > between =
            boost::dynamic_pointer_cast<gtsam::BetweenFactor<gtsam::Pose3> >(factor);
        gtsam::Key key1 = between->keys().at(0);
        gtsam::Key key2 = between->keys().at(1);
        gtsam::Pose3 measured = between->measured();
        chordalGraph_.add(gtsam::BetweenChordalFactor<gtsam::Pose3>(key1, key2, measured, poseNoiseModel_));

        // Linear orientation graph
        createLinearOrientationGraph(); // TODO: Rebuilds entire linear orientation graph everytime a factor is added
      }
      else{
        separatorEdgeIds_.push_back(graph_.size() -1);
      }

      // Clear traces
      rotationErrorTrace_.clear();
      poseErrorTrace_.clear();
    }

    /**
     * @brief updateNeighbor updates neighboring nodes with the new value
     * @param sym is symbol
     * @param pose is the pose
     */
    void updateNeighbor(gtsam::Key key, gtsam::Pose3 pose){
      // Update the value if symbol already exists
      if(neighbors_.exists(key)){
        neighbors_.update(key, pose);
      }
      else{
        neighbors_.insert(key, pose);
        neighborsLinearizedPoses_.insert(key, gtsam::zero(6));
        gtsam::Matrix3 R = pose.rotation().matrix();
        gtsam::Vector r = multirobot_util::rowMajorVector(R);
        neighborsLinearizedRotations_.insert(key, r);
      }
    }

    /**
     * @brief updateNeighborLinearizedPoses updates neighboring node vectorValues with the new vectorValues
     * @param sym is symbol
     * @param vectorValue is the new vectorValue
     */
    void updateNeighborLinearizedPoses(gtsam::Key key, gtsam::Vector vectorValue){
      // Update the value if symbol already exists
      if(neighborsLinearizedPoses_.exists(key)){
        neighborsLinearizedPoses_.at(key) = vectorValue;
      }
      else{
        neighborsLinearizedPoses_.insert(key, vectorValue);
      }
    }

    /**
     * @brief updateNeighborLinearizedRotations updates neighboring node vectorValues with the new vectorValues
     * @param sym is symbol
     * @param vectorValue is the new vectorValue
     */
    void updateNeighborLinearizedRotations(gtsam::Key key, gtsam::Vector vectorValue){
      // Update the value if symbol already exists
      if(neighborsLinearizedRotations_.exists(key)){
        neighborsLinearizedRotations_.at(key) = vectorValue;
      }
      else{
        neighborsLinearizedRotations_.insert(key, vectorValue);
      }
    }

    /** @brief linearizedPoses returns the linearized poses */
    gtsam::VectorValues linearizedPoses(){ return linearizedPoses_;}

    /** @brief linearizedPosesAt returns the current pose estimate at sym */
    gtsam::Vector linearizedPosesAt(gtsam::Key key){ return linearizedPoses_.at(key); }

    /** @brief retractPose3Global performs global retraction using linearizedPoses and initial */
    void retractPose3Global(){
      initial_ = multirobot_util::retractPose3Global(initial_, linearizedPoses_);
    }

    /** @brief linearizedRotationAt returns the current rotation estimate at sym */
    gtsam::Vector linearizedRotationAt(gtsam::Key key){ return linearizedRotation_.at(key); }

    /** @brief returns *latest* linear rotation estimate for neighbors */
    gtsam::Vector neighborsLinearizedRotationsAt(gtsam::Key key){ return neighborsLinearizedRotations_.at(key); }


    /** @brief convertLinearizedRotationToPoses iterates over linearized rotations and convert them to poses with zero translation  */
    void convertLinearizedRotationToPoses(){
      gtsam::Values rotValue = gtsam::InitializePose3::normalizeRelaxedRotations(linearizedRotation_);
      initial_ = multirobot_util::pose3WithZeroTranslation(rotValue);
      linearizedPoses_ = multirobot_util::initializeVectorValues(initial_); // Init linearized poses
      distGFG_ = *(chordalGraph_.linearize(initial_));

      // Initial error
      //double error = distGFG_.error(linearizedPoses_);
      //poseErrorTrace_.push_back(error);

    }

    /** @brief estimateAt returns the current estimate at sym */
    gtsam::Pose3 estimateAt(gtsam::Key key){ return initial_.at<gtsam::Pose3>(key); }

    /** @brief returns the current estimate */
    gtsam::Values currentEstimate(){ return initial_; }

    /** @brief returns the robot name */
    char robotName(){ return robotName_; }

    /** @brief getConvertedEstimate converts the current estimate of the subgraph to follow global symbolChr-less indexing */
    gtsam::Values getConvertedEstimate(gtsam::Values initial){
      gtsam::Values converted_estimate;
      for(const gtsam::Values::KeyValuePair& key_value: initial) {
        gtsam::Symbol key = key_value.key;
        if(useChrLessFullGraph_){
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
      for(gtsam::VectorValues::KeyValuePair& key_value: newLinearizedRotation_){
        gtsam::Key key = key_value.first;
        if(!linearizedRotation_.exists(key)){
          linearizedRotation_.insert(key, newLinearizedRotation_.at(key));
        }
        else{
          linearizedRotation_.at(key) = (1-gamma_)*linearizedRotation_.at(key) + gamma_*newLinearizedRotation_.at(key);
        }
      }
    }

    /**
     * @brief updatePoses updes linearizedPoses_ according to gamma and old linearized poses.
     */
    void updatePoses(){
      for(gtsam::VectorValues::KeyValuePair& key_value: newLinearizedPoses_){
        gtsam::Key key = key_value.first;
        if(!linearizedPoses_.exists(key)){
          linearizedPoses_.insert(key, newLinearizedPoses_.at(key));
        }
        else{
          linearizedPoses_.at(key) = (1-gamma_)*linearizedPoses_.at(key) + gamma_*newLinearizedPoses_.at(key);
        }
      }
    }


    /** Verbosity defines the verbosity levels */
    enum Verbosity{SILENT, // Print or output nothing
                   ERROR, // Save error traces
                   DEBUG // Also print debug statements
                  };


    /** @brief setVerbosity sets the verbosity level */
    void setVerbosity(Verbosity verbosity){verbosity_ = verbosity;}

    /** @brief trace returns the trace */
    std::pair<std::vector<double>, std::vector<double> > trace()
    {return std::make_pair(rotationErrorTrace_, poseErrorTrace_);}

    /** @brief traceEstimateChange returns the trace of change in estimate */
    std::pair<std::vector<double>, std::vector<double> > traceEstimateChange()
    {return std::make_pair(rotationEstimateChangeTrace_, poseEstimateChangeTrace_);}

    /** @brief log centralized estimate error for plotting */
    std::pair<double, double> logCentralizedError(gtsam::Values centralized){
      centralizedValues_.clear();
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
        if(useLandmarks_){
            centralizedValues_.insert(key, estimate);
          }
        else{
            centralizedValues_.insert(new_key, estimate);
          }
      }
      return std::make_pair(innerEdges_.error(centralizedValues_),
                            innerEdges_.error(initial_));
      }

    /** @brief latestChange returns the latest change in estimate */
    double latestChange(){
      return latestChange_;
    }

    /**
    * @brief updateRobotInitialized
    * @param robot
    * @param flag
    */
    void updateNeighboringRobotInitialized(char robot, bool flag){
      if(neighboringRobotsInitialized_.find(robot) != neighboringRobotsInitialized_.end()){
        neighboringRobotsInitialized_.at(robot) = flag;
      }
      else{
        neighboringRobotsInitialized_.insert(std::make_pair(robot, flag));
      }
    }

    /**
    * @brief clearNeighboringRobotInit
    */
    void clearNeighboringRobotInit(){
      neighboringRobotsInitialized_.clear();
    }

    /**
     * @brief isRobotInitialized
     * @return
     */
    bool isRobotInitialized(){
      return robotInitialized_;
    }

    /**
     * @brief updateInitialized
     * @param flag
     */
    void updateInitialized(bool flag){
      robotInitialized_ = flag;
    }

    /**
     * @brief setFlaggedInit
     * @param flag
     */
    void setFlaggedInit(bool flag){
      useFlaggedInit_ = flag;
    }

    /**
     * @brief getNeighboringChars returns the set of neighboring robot symbol chars
     */
    std::set<char> getNeighboringChars(){
      return neighborChars_;
    }

    bool useChrLessFullGraph_; // full graph pose have no key characters if it is true.

    // UpdateType and Gamma
    UpdateType updateType_;
    double gamma_;


  protected:
    bool debug_; // Debug flag
    gtsam::noiseModel::Diagonal::shared_ptr rotationNoiseModel_;
    gtsam::noiseModel::Isotropic::shared_ptr poseNoiseModel_;

    char robotName_;// Key for each robot
    gtsam::NonlinearFactorGraph graph_; // subgraph corresponding to each robot
    gtsam::Values initial_; // subinitials corresponding to each robot        
    gtsam::NonlinearFactorGraph innerEdges_; // edges involving keys from a single robot (exclude separator edges)
    std::vector<size_t>  separatorEdgeIds_; // for each robot stores the position of the factors corresponding to separator edges
    gtsam::Values neighbors_; // contains keys of all the neighboring robots
    std::set<char> neighborChars_; // contains neighboring robot symbols
    double latestChange_; // Latest change in estimate, stopping condition
    bool useBetweenNoise_; // To use the between factor noise instead of isotropic unit noise during pose estimation

    // Cached values and graph required for fast optimization and communication
    gtsam::VectorValues linearizedRotation_; // contains vector values of rotation of internal nodes
    gtsam::VectorValues newLinearizedRotation_; // contains vector values of rotation of internal nodes after current iteration
    gtsam::VectorValues neighborsLinearizedRotations_; // contains vector values of all the neighboring robots for distributed estimation
    gtsam::VectorValues linearizedPoses_; // contains vector values of poses of internal nodes
    gtsam::VectorValues newLinearizedPoses_; // contains vector values of poses of internal nodes after current iteration
    gtsam::VectorValues neighborsLinearizedPoses_; // contains vector values of all the neighboring robots for distributed estimation
    gtsam::NonlinearFactorGraph chordalGraph_; // edges involving keys from a single robot represented using BetweenChordalFactors
    gtsam::GaussianFactorGraph rotSubgraph_; // linear orientation graph required for distributed rotation estimation
    gtsam::GaussianFactorGraph distGFG_; // Gaussian factor graph initialized before distributed pose estimation

    // Initialization
    std::map<char, bool> neighboringRobotsInitialized_; // contains boolean flag to check if a robot is initialized or not
    bool robotInitialized_; // flag to check if this robot is initialized or not
    bool useFlaggedInit_; // flagged initialization

    // Landmarks
    bool useLandmarks_; // use landmarks -- landmarks are given symbols as upper case of robot name, for eg: if robot is 'a', landmark will be 'A'

    // Verbosity
    std::vector <double> rotationErrorTrace_; // error trace
    std::vector <double> poseErrorTrace_; // error trace
    std::vector <double> rotationEstimateChangeTrace_; // change in estimate trace
    std::vector <double> poseEstimateChangeTrace_; // change in estimate trace
    double centralizedError_; // log it for plotting
    gtsam::Values centralizedValues_; // centralized estimate converted to the estimate format of this graph
    Verbosity verbosity_; // Verbosity level
};

}
