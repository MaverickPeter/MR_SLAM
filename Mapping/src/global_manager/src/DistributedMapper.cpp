#include <global_manager/DistributedMapper.h>


using namespace std;
using namespace gtsam;

namespace distributed_mapper{
//*****************************************************************************
pair<NonlinearFactorGraph, vector<size_t> >
DistributedMapper::createSubgraphInnerAndSepEdges(const NonlinearFactorGraph& subgraph){

  // we create the NFG including only the inner edges
  NonlinearFactorGraph subgraphInnerEdge;
  vector<size_t> subgraphsSepEdgesId;
  neighbors_.clear();

  for(size_t k=0; k < subgraph.size(); k++){ // this loops over the factors in subgraphs[i]
    // Continue if factor does not exist
    if(!subgraph.at(k))continue;
    // Separate in subgraphs[i] edges that are in the interior
    // from separator edges, which connect vertices in different subgraphs
    KeyVector keys = subgraph.at(k)->keys();

    if (keys.size() != 2){
      subgraphInnerEdge.push_back(subgraph.at(k));
      continue;
    }
    Symbol key0 = keys.at(0);
    Symbol key1 = keys.at(1);

    char robot0 = symbolChr(key0);
    char robot1 = symbolChr(key1);

    if (robot0 == robot1 || (useLandmarks_ && robot1 == toupper(robot0))){ // keys from the same subgraph
      if(verbosity_ >= DEBUG) cout << "Factor connecting (intra): " << robot0 << " " << symbolIndex(key0) << " " <<  robot1 << " " << symbolIndex(key1) << endl;
      subgraphInnerEdge.push_back(subgraph.at(k)); // the edge is not a separator, but belongs to the interior of the subgraph
    }
    else{
      // Add it as a prior factor using the current estimate from the other graph
      if(verbosity_ >= DEBUG) cout << "Factor connecting (extra): " << robot0 << " " << symbolIndex(key0) << " " <<  robot1 << " " << symbolIndex(key1) << endl;
      subgraphsSepEdgesId.push_back(k); // TODO: allocate this

      // Neighbors data structure
      if(robot0 == robotName_ || (useLandmarks_ && robot0 == toupper(robotName_))){
        if(!neighbors_.exists(key1))
          neighbors_.insert(key1, Pose3());

        if(!neighborChars_.count(robot1))
          neighborChars_.insert(robot1);
      }
      else{
        if(!neighbors_.exists(key0))
          neighbors_.insert(key0, Pose3());

        if(!neighborChars_.count(robot0))
          neighborChars_.insert(robot0);
      }
    }
  }

  // Convert neighbor values into row major vector values
  neighborsLinearizedRotations_ = multirobot_util::rowMajorVectorValues(neighbors_);
  neighborsLinearizedPoses_ = multirobot_util::initializeVectorValues(neighbors_);

  return make_pair(subgraphInnerEdge,subgraphsSepEdgesId);
}

//*****************************************************************************
void
DistributedMapper::loadSubgraphAndCreateSubgraphEdge(GraphAndValues graphAndValues){
  graph_ = *(graphAndValues.first);
  initial_ = *(graphAndValues.second);    

  // Convert initial values into row major vector values
  linearizedRotation_ = multirobot_util::rowMajorVectorValues(initial_);

  // create a nonlinear factor graph with inner edges and store slots of separators
  pair<NonlinearFactorGraph, vector<size_t> > subGraphEdge = createSubgraphInnerAndSepEdges(graph_);
  innerEdges_ = subGraphEdge.first;
  separatorEdgeIds_ = subGraphEdge.second;

  // Internal cached graphs for distributed estimations
  createLinearOrientationGraph(); // linear orientation graph with inner edges
  chordalFactorGraph(); // nonlinear chordal pose3graph with inner edges
}

//*****************************************************************************
void
DistributedMapper::createLinearOrientationGraph(){
  // Preallocate
  NonlinearFactorGraph pose3Graph = InitializePose3::buildPose3graph(innerEdges_);
  rotSubgraph_ = multirobot_util::buildLinearOrientationGraph(pose3Graph, useBetweenNoise_);
}


//*****************************************************************************
void
DistributedMapper::estimateRotation(){
  // Rotation vector estimated using Chordal relaxation

  // Inner edges for the current subgraph
  gtsam::GaussianFactorGraph rotSubgraph = rotSubgraph_.clone();

  // push back to rotgraph_i separator edges as priors
  for(size_t s = 0 ; s < separatorEdgeIds_.size(); s++){ // for each separator
    // | rj - Mij * ri | = | Mij * ri - rj |, with Mij = diag(Rij',Rij',Rij')
    // |Ab - b| -> Jacobian(key,A,b)

    size_t sepSlot =  separatorEdgeIds_[s];
    boost::shared_ptr<BetweenFactor<Pose3> > pose3Between =
        boost::dynamic_pointer_cast<BetweenFactor<Pose3> >(graph_.at(sepSlot));

    Pose3 relativePose = pose3Between->measured();
    Matrix3 R01t = relativePose.rotation().transpose().matrix();
    Matrix M9 = Matrix::Zero(9,9);
    M9.block(0,0,3,3) = R01t;
    M9.block(3,3,3,3) = R01t;
    M9.block(6,6,3,3) = R01t;
    KeyVector keys = pose3Between->keys();
    Symbol key0 = keys.at(0);
    Symbol key1 = keys.at(1);
    char robot0 = symbolChr(key0);
    char robot1 = symbolChr(key1);

    // Landmarks use Upper case robot symbol
    if(useLandmarks_){
        robot0 = tolower(robot0);
        robot1 = tolower(robot1);
      }

    // if using between noise, use the factor noise model converted to a conservative diagonal estimate
    SharedDiagonal model = rotationNoiseModel_;
    if(useBetweenNoise_){
        model = multirobot_util::convertToDiagonalNoise(pose3Between->get_noiseModel());
      }

    if(robot0 == robotName_){ // robot i owns the first key
      if(!useFlaggedInit_ || neighboringRobotsInitialized_[robot1]){ // if use flagged initialization and robot sharing the edge is already optimized
        Vector r1 = neighborsLinearizedRotations_.at(key1);
        rotSubgraph.add(key0, M9, r1, model);
      }
    }
    else if(robot1 == robotName_){ // robot i owns the second key
      if(!useFlaggedInit_ || neighboringRobotsInitialized_[robot0]){ // if use flagged initialization and robot sharing the edge is already optimized
        Vector r0 = neighborsLinearizedRotations_.at(key0);
        Vector M9_r0 = M9*r0;
        rotSubgraph.add(key1, I9, M9_r0, model);
      }
    }
    else{
      cout << "robot0 != robotNames[i] and robot1 != robotNames[i]: " <<
              robot0 << " " << robot1 << " " << endl;
      exit(1);
    }
  }

  // Solve the LFG
  newLinearizedRotation_ = rotSubgraph.optimize();

  // Log it
  if(verbosity_ >= ERROR){
    if(linearizedRotation_.size() == newLinearizedRotation_.size()){
      latestChange_ = newLinearizedRotation_.subtract(linearizedRotation_).norm();
      rotationEstimateChangeTrace_.push_back(latestChange_);
    }

    double error = rotSubgraph.error(newLinearizedRotation_);
    rotationErrorTrace_.push_back(error);
  }  
}

//*****************************************************************************
void
DistributedMapper::chordalFactorGraph(){
  chordalGraph_ = gtsam::NonlinearFactorGraph(); // Clear the previous graph
  for(size_t k = 0; k < innerEdges_.size(); k++){
      boost::shared_ptr<BetweenFactor<Pose3> > factor =
          boost::dynamic_pointer_cast<BetweenFactor<Pose3> >(innerEdges_[k]);
      if(factor){
          Key key1 = factor->keys().at(0);
          Key key2 = factor->keys().at(1);
          Pose3 measured = factor->measured();
          if(useBetweenNoise_){
              // Convert noise model to chordal factor noise
              SharedNoiseModel chordalNoise = multirobot_util::convertToChordalNoise(factor->get_noiseModel());
              //chordalNoise->print("Chordal Noise: \n");
              chordalGraph_.add(BetweenChordalFactor<Pose3>(key1, key2, measured, chordalNoise));
            }
          else{
              chordalGraph_.add(BetweenChordalFactor<Pose3>(key1, key2, measured, poseNoiseModel_));
            }
        }
      else{
          chordalGraph_.add(innerEdges_[k]);
        }
    }
}

//*****************************************************************************
void
DistributedMapper::estimatePoses(){

  if(verbosity_ >= DEBUG)
    cout << "========= each robot normalizes rotations & linearize pose graph of inner edges " << endl;
  
  // Gaussian factor graph for i_th subgraph using BetweenChordalFactor
  // Compute linearization point from rotation estimate (we need to project to SO(3))
  GaussianFactorGraph distGFG = distGFG_.clone();

  // push back to distGFG_i separator edges as priors
  for(size_t s = 0 ; s < separatorEdgeIds_.size(); s++){ // for each separator
    // | rj - Mij * ri | = | Mij * ri - rj |, with Mij = diag(Rij',Rij',Rij')
    // |Ab - b| -> Jacobian(key,A,b)

    size_t sepSlot =  separatorEdgeIds_[s];
    boost::shared_ptr<BetweenFactor<Pose3> > pose3Between =
        boost::dynamic_pointer_cast<BetweenFactor<Pose3> >(graph_.at(sepSlot));

    // Construct between chordal factor corresponding to separator edges
    KeyVector keys = pose3Between->keys();
    Symbol key0 = keys.at(0);
    Symbol key1 = keys.at(1);
    char robot0 = symbolChr(key0);
    char robot1 = symbolChr(key1);
    Pose3 measured = pose3Between->measured();

    BetweenChordalFactor<Pose3> betweenChordalFactor(key0, key1, measured, poseNoiseModel_);

    // Pre allocate jacobian matrices
    Matrix M0 = Matrix::Zero(12,6);
    Matrix M1 = Matrix::Zero(12,6);

    // Landmarks use Upper case robot symbol
    if(useLandmarks_){
        robot0 = tolower(robot0);
        robot1 = tolower(robot1);
      }

    if(robot0 == robotName_){ // robot i owns the first key
        if(!useFlaggedInit_ || neighboringRobotsInitialized_[robot1]){ // if use flagged initialization and robot sharing the edge is already optimized
            Vector error = betweenChordalFactor.evaluateError(initial_.at<Pose3>(key0), neighbors_.at<Pose3>(key1), M0, M1);
            // Robot i owns the first key_i, on which we put a prior
            Matrix A = M0;
            Vector b = -(M1 * neighborsLinearizedPoses_.at(key1) + error);
            if(useBetweenNoise_){
                Rot3 rotation = initial_.at<Pose3>(key0).rotation();
                SharedNoiseModel chordalNoise = multirobot_util::convertToChordalNoise(pose3Between->get_noiseModel(), rotation.matrix());
                chordalNoise->WhitenSystem(A, b);
              }
            distGFG.add(key0, A, b, poseNoiseModel_);
          }
      }
    else if(robot1 == robotName_){ // robot i owns the second key
        if(!useFlaggedInit_ || neighboringRobotsInitialized_[robot0]){ // if use flagged initialization and robot sharing the edge is already optimized            
            Vector error = betweenChordalFactor.evaluateError(neighbors_.at<Pose3>(key0), initial_.at<Pose3>(key1), M0, M1);
            // Robot i owns the second key_i, on which we put a prior
            Matrix A = M1;
            Vector b = -(M0 * neighborsLinearizedPoses_.at(key0) + error);
            if(useBetweenNoise_){
                Rot3 rotation = neighbors_.at<Pose3>(key0).rotation();
                SharedNoiseModel chordalNoise = multirobot_util::convertToChordalNoise(pose3Between->get_noiseModel(), rotation.matrix());
                chordalNoise->WhitenSystem(A, b);
              }
            distGFG.add(key1, A, b, poseNoiseModel_);
          }
      }
    else{
        cout << "robot0 != robotNames[i] and robot1 != robotNames[i]: " <<
                robot0 << " " << robot1 << " " << endl;
      exit(1);
    }
  }

  // Solve the LFG
  newLinearizedPoses_ = distGFG.optimize();

  // Log it
  if(verbosity_ >= ERROR){
    if(linearizedPoses_.size() == newLinearizedPoses_.size()){
      latestChange_ = newLinearizedPoses_.subtract(linearizedPoses_).norm();
      poseEstimateChangeTrace_.push_back(latestChange_);
    }

    double error = distGFG.error(newLinearizedPoses_);
    poseErrorTrace_.push_back(error);
  }
}

//*****************************************************************************

void DistributedMapper::optimize(){
  // Estimate Rotation
  estimateRotation();

  // Update Estimate
  updateRotation();

  // Convert it to poses
  convertLinearizedRotationToPoses();

  // Pose estimation
  estimatePoses();

  // Update Poses
  updatePoses();

  // Retrace
  retractPose3Global();
}

}
