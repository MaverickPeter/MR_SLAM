#include "distributed_mapper/distributed_mapper.h"


using namespace std;
using namespace gtsam;

namespace distributed_mapper{
//*****************************************************************************
pair<NonlinearFactorGraph, vector<size_t> >
DistributedMapper::createSubgraphInnerAndSepEdges(const NonlinearFactorGraph& subgraph){

  // we create the NFG including only the inner edges
  NonlinearFactorGraph subgraph_inner_edge;
  vector<size_t> subgraphs_sep_edges_id;
  neighbors_.clear();
  neighbors_updated_.clear();
  loopclosures_symbols_.clear();

  for(size_t k=0; k < subgraph.size(); k++){ // this loops over the factors in subgraphs[i]
    // Continue if factor does not exist
    if(!subgraph.at(k))continue;
    // Separate in subgraphs[i] edges that are in the interior
    // from loopclosure edges, which connect vertices in different subgraphs
    KeyVector keys = subgraph.at(k)->keys();

    if (keys.size() != 2){
      subgraph_inner_edge.push_back(subgraph.at(k));
      continue;
    }
    Symbol key0 = keys.at(0);
    Symbol key1 = keys.at(1);
    size_t key_bits = sizeof(Key) * 8;
    size_t chr_bits = sizeof(unsigned char) * 8;
    size_t index_bits = key_bits - chr_bits;
    Key chr_mask = Key(UCHAR_MAX)  << index_bits; // For some reason, std::numeric_limits<unsigned char>::max() fails
    Key index_mask = ~chr_mask;

    char robot0 = symbolChr(key0);
    char robot1 = symbolChr(key1);

    if (robot0 == robot1 || (use_landmarks_ && robot1 == toupper(robot0))){ // keys from the same subgraph
      if(verbosity_ >= DEBUG) cout << "Factor connecting (intra): " << robot0 << " " << symbolIndex(key0) << " " <<  robot1 << " " << symbolIndex(key1) << endl;
      subgraph_inner_edge.push_back(subgraph.at(k)); // the edge is not a loopclosure, but belongs to the interior of the subgraph
    }
    else{
      // Add it as a prior factor using the current estimate from the other graph
      if(verbosity_ >= DEBUG) cout << "Factor connecting (extra): " << robot0 << " " << symbolIndex(key0) << " " <<  robot1 << " " << symbolIndex(key1) << endl;
      subgraphs_sep_edges_id.push_back(k); // TODO: allocate this

      // Neighbors data structure
      if(robot0 == robotName_ || (use_landmarks_ && robot0 == toupper(robotName_))){
        if(!neighbors_.exists(key1)) {
          neighbors_.insert(key1, Pose3());
          loopclosures_symbols_.emplace_back(std::make_pair(key1, key0));
        }
        if(!neighbor_chars_.count(robot1)) {
          neighbor_chars_.insert(robot1);
        }
      }
      else{
        if(!neighbors_.exists(key0)) {
          neighbors_.insert(key0, Pose3());
          loopclosures_symbols_.emplace_back(std::make_pair(key0, key1));
        }
        if(!neighbor_chars_.count(robot0)) {
          neighbor_chars_.insert(robot0);
        }
      }
    }
  }

  // Convert neighbor values into row major vector values
  for (auto neighbor_value : evaluation_utils::rowMajorVectorValues(neighbors_)){
    neighbors_linearized_rotations_.tryInsert(neighbor_value.first, neighbor_value.second);
    neighbors_updated_.insert(std::make_pair(neighbor_value.first, false));
  }

  for (auto neighbor_value : evaluation_utils::initializeVectorValues(neighbors_)){
    neighbors_linearized_poses_.tryInsert(neighbor_value.first, neighbor_value.second);
  }

  return make_pair(subgraph_inner_edge,subgraphs_sep_edges_id);
}

//*****************************************************************************
void
DistributedMapper::loadSubgraphAndCreateSubgraphEdge(const GraphAndValues& graph_and_values){
  graph_ = *(graph_and_values.first);
  initial_ = *(graph_and_values.second);

      // Convert initial values into row major vector values
  linearized_rotation_ = evaluation_utils::rowMajorVectorValues(initial_);

  // create a nonlinear factor graph with inner edges and store slots of loopclosures
  pair<NonlinearFactorGraph, vector<size_t> > subgraph_edge = createSubgraphInnerAndSepEdges(graph_);

  inner_edges_ = subgraph_edge.first;
  loopclosure_edge_ids_ = subgraph_edge.second;

  // Internal cached graphs for distributed estimations
  createLinearOrientationGraph(); // linear orientation graph with inner edges
  chordalFactorGraph(); // nonlinear chordal pose3_graph with inner edges

}

//*****************************************************************************
void
DistributedMapper::createLinearOrientationGraph(){
  // Preallocate
  NonlinearFactorGraph pose3_graph = InitializePose3::buildPose3graph(inner_edges_);
  rot_subgraph_ = evaluation_utils::buildLinearOrientationGraph(pose3_graph, use_between_noise_);
}


//*****************************************************************************
void
DistributedMapper::estimateRotation(){
  // Rotation vector estimated using Chordal relaxation

  // Inner edges for the current subgraph
  gtsam::GaussianFactorGraph rot_subgraph = rot_subgraph_.clone();

  // push back to rotgraph_i loopclosure edges as priors
  for(size_t s = 0 ; s < loopclosure_edge_ids_.size(); s++){ // for each loopclosure
    // | rj - Mij * ri | = | Mij * ri - rj |, with Mij = diag(Rij',Rij',Rij')
    // |Ab - b| -> Jacobian(key,A,b)

    size_t sep_slot =  loopclosure_edge_ids_[s];
    boost::shared_ptr<BetweenFactor<Pose3> > pose3_between =
        boost::dynamic_pointer_cast<BetweenFactor<Pose3> >(graph_.at(sep_slot));

    Pose3 relative_pose = pose3_between->measured();
    Matrix3 R01t = relative_pose.rotation().transpose().matrix();
    Matrix M9 = Matrix::Zero(9,9);
    M9.block(0,0,3,3) = R01t;
    M9.block(3,3,3,3) = R01t;
    M9.block(6,6,3,3) = R01t;
    KeyVector keys = pose3_between->keys();
    Symbol key0 = keys.at(0);
    Symbol key1 = keys.at(1);
    char robot0 = symbolChr(key0);
    char robot1 = symbolChr(key1);

    // Landmarks use Upper case robot symbol
    if(use_landmarks_){
        robot0 = tolower(robot0);
        robot1 = tolower(robot1);
      }

    // if using between noise, use the factor noise model converted to a conservative diagonal estimate
    SharedDiagonal model = rotation_noise_model_;
    if(use_between_noise_){
        model = evaluation_utils::convertToDiagonalNoise(pose3_between->get_noiseModel());
      }

    if(robot0 == robotName_){ // robot i owns the first key
      if((!use_flagged_init_ || neighboring_robots_initialized_[robot1]) && neighbors_updated_[key1]) { // if use flagged initialization and robot sharing the edge is already optimized
        Vector r1 = neighbors_linearized_rotations_.at(key1);
        rot_subgraph.add(key0, M9, r1, model);
      }
    }
    else if(robot1 == robotName_){ // robot i owns the second key
      if((!use_flagged_init_ || neighboring_robots_initialized_[robot0]) && neighbors_updated_[key0]){ // if use flagged initialization and robot sharing the edge is already optimized
        Vector r0 = neighbors_linearized_rotations_.at(key0);
        Vector M9_r0 = M9*r0;
        rot_subgraph.add(key1, I9, M9_r0, model);
      }
    }
    else{
      cout << "robot0 != robotNames[i] and robot1 != robotNames[i]: " <<
              robot0 << " " << robot1 << " " << endl;
      exit(1);
    }
  }

  // Solve the LFG
  new_linearized_rotation_ = rot_subgraph.optimize();

  // Log it
  if(verbosity_ >= ERROR){
    if(linearized_rotation_.size() == new_linearized_rotation_.size()){
      latest_change_ = new_linearized_rotation_.subtract(linearized_rotation_).norm();
      rotation_estimate_change_trace_.push_back(latest_change_);
    }

    double error = rot_subgraph.error(new_linearized_rotation_);
    rotation_error_trace_.push_back(error);
  }  
}

//*****************************************************************************
void
DistributedMapper::chordalFactorGraph(){
  chordal_graph_ = gtsam::NonlinearFactorGraph(); // Clear the previous graph
  for(size_t k = 0; k < inner_edges_.size(); k++){
      boost::shared_ptr<BetweenFactor<Pose3> > factor =
          boost::dynamic_pointer_cast<BetweenFactor<Pose3> >(inner_edges_[k]);
      if(factor){
          Key key1 = factor->keys().at(0);
          Key key2 = factor->keys().at(1);
          Pose3 measured = factor->measured();
          if(use_between_noise_){
              // Convert noise model to chordal factor noise
              SharedNoiseModel chordal_noise = evaluation_utils::convertToChordalNoise(factor->get_noiseModel());
              //chordal_noise->print("Chordal Noise: \n");
              chordal_graph_.add(BetweenChordalFactor<Pose3>(key1, key2, measured, chordal_noise));
            }
          else{
              chordal_graph_.add(BetweenChordalFactor<Pose3>(key1, key2, measured, pose_noise_model_));
            }
        }
      else{
          chordal_graph_.add(inner_edges_[k]);
        }
    }
}

//*****************************************************************************
void
DistributedMapper::estimatePoses(){

  if(verbosity_ >= DEBUG)
    cout << "========= each robot normalizes rotations & linearize pose graph of inner edges " << endl;
  // Gaussian factor graph for i_th subgraph using between_chordal_factor
  // Compute linearization point from rotation estimate (we need to project to SO(3))
  GaussianFactorGraph dist_GFG = dist_GFG_.clone();

  // push back to dist_GFG_i loopclosure edges as priors
  for(size_t s = 0 ; s < loopclosure_edge_ids_.size(); s++){ // for each loopclosure
    // | rj - Mij * ri | = | Mij * ri - rj |, with Mij = diag(Rij',Rij',Rij')
    // |Ab - b| -> Jacobian(key,A,b)

    size_t sep_slot =  loopclosure_edge_ids_[s];
    boost::shared_ptr<BetweenFactor<Pose3> > pose3_between =
        boost::dynamic_pointer_cast<BetweenFactor<Pose3> >(graph_.at(sep_slot));

    // Construct between chordal factor corresponding to loopclosure edges
    KeyVector keys = pose3_between->keys();
    Symbol key0 = keys.at(0);
    Symbol key1 = keys.at(1);
    char robot0 = symbolChr(key0);
    char robot1 = symbolChr(key1);
    Pose3 measured = pose3_between->measured();

    BetweenChordalFactor<Pose3> between_chordal_factor(key0, key1, measured, pose_noise_model_);

    // Pre allocate jacobian matrices
    Matrix M0 = Matrix::Zero(12,6);
    Matrix M1 = Matrix::Zero(12,6);

    // Landmarks use Upper case robot symbol
    if(use_landmarks_){
        robot0 = tolower(robot0);
        robot1 = tolower(robot1);
      }

    if(robot0 == robotName_){ // robot i owns the first key
        if((!use_flagged_init_ || neighboring_robots_initialized_[robot1]) && neighbors_updated_[key1]){ // if use flagged initialization and robot sharing the edge is already optimized
            Vector error = between_chordal_factor.evaluateError(initial_.at<Pose3>(key0), neighbors_.at<Pose3>(key1), M0, M1);
            // Robot i owns the first key_i, on which we put a prior
            Matrix A = M0;
            Vector b = -(M1 * neighbors_linearized_poses_.at(key1) + error);
            if(use_between_noise_){
                Rot3 rotation = initial_.at<Pose3>(key0).rotation();
                SharedNoiseModel chordal_noise = evaluation_utils::convertToChordalNoise(pose3_between->get_noiseModel(), rotation.matrix());
                chordal_noise->WhitenSystem(A, b);
              }
            dist_GFG.add(key0, A, b, pose_noise_model_);
          }
      }
    else if(robot1 == robotName_){ // robot i owns the second key
        if((!use_flagged_init_ || neighboring_robots_initialized_[robot0]) && neighbors_updated_[key0]){ // if use flagged initialization and robot sharing the edge is already optimized            
            Vector error = between_chordal_factor.evaluateError(neighbors_.at<Pose3>(key0), initial_.at<Pose3>(key1), M0, M1);
            // Robot i owns the second key_i, on which we put a prior
            Matrix A = M1;
            Vector b = -(M0 * neighbors_linearized_poses_.at(key0) + error);
            if(use_between_noise_){
                Rot3 rotation = neighbors_.at<Pose3>(key0).rotation();
                SharedNoiseModel chordal_noise = evaluation_utils::convertToChordalNoise(pose3_between->get_noiseModel(), rotation.matrix());
                chordal_noise->WhitenSystem(A, b);
              }
            dist_GFG.add(key1, A, b, pose_noise_model_);
        }
      }
    else{
        cout << "robot0 != robotNames[i] and robot1 != robotNames[i]: " <<
                robot0 << " " << robot1 << " " << endl;
      exit(1);
    }
  }

  // Solve the LFG
  new_linearized_poses_ = dist_GFG.optimize();

  // Log it
  if(verbosity_ >= ERROR){
    if(linearized_poses_.size() == new_linearized_poses_.size()){
      latest_change_ = new_linearized_poses_.subtract(linearized_poses_).norm();
      pose_estimate_change_trace_.push_back(latest_change_);
    }

    double error = dist_GFG.error(new_linearized_poses_);
    pose_error_trace_.push_back(error);
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
