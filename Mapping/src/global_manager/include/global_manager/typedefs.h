#pragma once
#define PCL_NO_PRECOMPILE

// STL
#include <vector>

// Eigen
#include <Eigen/Core>
#include <Eigen/Geometry>

// PCL
#include <pcl/io/pcd_io.h>
#include <pcl/pcl_macros.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/correspondence.h>
#include <pcl/point_cloud.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/common.h>
#include <pcl/registration/icp.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/visualization/point_cloud_color_handlers.h>
#include <pcl/filters/passthrough.h>


// gtsam
#include <gtsam/slam/InitializePose3.h>
#include <gtsam/slam/dataset.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/nonlinear/GaussNewtonOptimizer.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/linear/VectorValues.h>
#include <gtsam/base/Value.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Rot3.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/ISAM2.h>

// point type
#include "PointXYZT.hpp"
#include "PointXYZIRPYT.hpp"

// kdtree
#include "nanoflann.hpp"
#include "KDTreeVectorOfVectorsAdaptor.h"


using namespace std;
using namespace gtsam;

namespace global_manager
{
// setup some typedefs for working with RGB pointclouds

// basic pointclouds definitions
typedef PointXYZT PointT;
typedef pcl::PointXYZI PointTI;
typedef pcl::PointXYZINormal PointTIN;
typedef pcl::PointCloud<PointTI> PointCloudI;
typedef pcl::PointCloud<PointTI>::Ptr PointCloudIPtr;
typedef pcl::PointCloud<PointTIN> PointCloudIN;
typedef pcl::PointCloud<PointTIN>::Ptr PointCloudINPtr;
typedef pcl::PointCloud<PointT> PointCloud;
typedef pcl::PointCloud<PointT>::Ptr PointCloudPtr;
typedef pcl::PointCloud<PointT>::ConstPtr PointCloudConstPtr;

// keyframe pointclouds
typedef std::vector<PointCloudIPtr> KeyFrameVec;

// timestamp
typedef std::vector<ros::Time> TimeStampVec;

// FFT result of disco storage
typedef std::vector<float> DiSCO;
typedef std::pair<std::vector<float>, std::vector<float>> DiSCOFFT;
typedef std::vector<DiSCOFFT> DiSCOFFTVec;

// ortho image for DPCN
typedef std::vector<cv::Mat> OrthoVector;

// submap and disco type
typedef std::vector<PointCloudPtr> SubMapVec;
typedef std::vector<std::vector<float>> DiSCOVec;
typedef KDTreeVectorOfVectorsAdaptor<DiSCOVec, float> KDTree;

// factor graph
typedef std::vector<GraphAndValues> GraphAndValuesVec;
typedef NonlinearFactorGraph::shared_ptr GraphPtr;
typedef Values::shared_ptr ValuesPtr;

// trajectory storage
typedef std::vector<Eigen::Isometry3f, Eigen::aligned_allocator<Eigen::Isometry3f>> TrajVec;

// normals as separate pointcloud
typedef pcl::Normal NormalT;
typedef pcl::PointCloud<NormalT> SurfaceNormals;
typedef pcl::PointCloud<NormalT>::Ptr SurfaceNormalsPtr;
typedef pcl::PointCloud<NormalT>::ConstPtr SurfaceNormalsConstPtr;

// local descriptors for registration
typedef pcl::PCLPointCloud2 LocalDescriptors;
typedef pcl::PCLPointCloud2::Ptr LocalDescriptorsPtr;
typedef pcl::PCLPointCloud2::ConstPtr LocalDescriptorsConstPtr;

// correspondences
using pcl::Correspondences;
using pcl::CorrespondencesPtr;

// color handler for visualisation
typedef pcl::visualization::PointCloudColorHandlerCustom<PointT> ColorHandlerT;


}  // namespace global_manager
