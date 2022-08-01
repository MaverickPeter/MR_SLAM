/*
 * SensorProcessorBase.cpp
 *
 *  Created on: Jun 6, 2014
 *      Author: Péter Fankhauser, Hannes Keller
 *   Institute: ETH Zurich, ANYbotics
 */

#include <elevation_mapping/sensor_processors/SensorProcessorBase.hpp>

//PCL

#include <pcl/io/pcd_io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/pcl_base.h>
#include <pcl/common/transforms.h>
#include <pcl/common/io.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/extract_indices.h>
//#include <pcl/gpu/containers/device_memory.h>
//TF
#include <tf_conversions/tf_eigen.h>

// STL
#include <limits>
#include <math.h>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>

int Process_points(int *mapindex, float *point_x, float *point_y, float *point_z, float *point_var, float *point_x_ts, float *point_y_ts, float *point_z_ts, Eigen::Matrix4f Transform, int point_num, double relativeLowerThreshold, double relativeUpperThreshold, float min_r, float beam_a, float beam_c, Eigen::RowVector3f sensorJacobian, Eigen::Matrix3f rotationVariance, Eigen::Matrix3f C_SB_transpose, Eigen::RowVector3f P_mul_C_BM_transpose, Eigen::Matrix3f B_r_BS_skew);
namespace elevation_mapping {

SensorProcessorBase::SensorProcessorBase(ros::NodeHandle& nodeHandle, tf::TransformListener& transformListener)
    : nodeHandle_(nodeHandle),
      transformListener_(transformListener),
      ignorePointsUpperThreshold_(std::numeric_limits<double>::infinity()),
      ignorePointsLowerThreshold_(-std::numeric_limits<double>::infinity())
{
  pcl::console::setVerbosityLevel(pcl::console::L_ERROR);
	transformationSensorToMap_.setIdentity();
	transformListenerTimeout_.fromSec(1.0);
}

SensorProcessorBase::~SensorProcessorBase() {}

bool SensorProcessorBase::readParameters()
{
  nodeHandle_.param("sensor_frame_id", sensorFrameId_, std::string("/sensor")); // TODO Fail if parameters are not found.
  nodeHandle_.param("robot_base_frame_id", robotBaseFrameId_, std::string("/robot"));
  nodeHandle_.param("map_frame_id", mapFrameId_, std::string("/map"));

  double minUpdateRate;
  nodeHandle_.param("min_update_rate", minUpdateRate, 2.0);
  transformListenerTimeout_.fromSec(1.0 / minUpdateRate);
  ROS_ASSERT(!transformListenerTimeout_.isZero());

  nodeHandle_.param("sensor_processor/ignore_points_above", ignorePointsUpperThreshold_, std::numeric_limits<double>::infinity());
  nodeHandle_.param("sensor_processor/ignore_points_below", ignorePointsLowerThreshold_, -std::numeric_limits<double>::infinity());
  return true;
}

bool SensorProcessorBase::process(
		const pcl::PointCloud<Anypoint>::ConstPtr pointCloudInput,
		const pcl::PointCloud<Anypoint>::Ptr pointCloudMapFrame,
    int *point_colorR,
    int *point_colorG,
    int *point_colorB,
		int *point_index,
    float* point_intensity,
    float *point_height,
    float *point_var)
{
  ros::Time timeStamp;
  timeStamp.fromNSec(1000 * pointCloudInput->header.stamp);
  ros::Time begin_time = ros::Time::now ();
 
  //if (!updateTransformations(timeStamp)) return false;
  //pcl::gpu::DeviceArray<Anypoint> cloud_device;
  pcl::PointCloud<Anypoint> Points;
  Points = *pointCloudInput;
	pcl::PointCloud<Anypoint>::Ptr pointCloudSensorFrame(new pcl::PointCloud<Anypoint>);
	//transformPointCloud(pointCloudInput, pointCloudSensorFrame, sensorFrameId_);
  pointCloudSensorFrame = Points.makeShared();

  cleanPointCloud(pointCloudSensorFrame);  //除去NAN值
 
  if (!GPUPointCloudprocess(pointCloudSensorFrame, pointCloudMapFrame, mapFrameId_, point_colorR, point_colorG, point_colorB, point_index, point_intensity, point_height, point_var)) return false; //TF变化

	return true;
}

//监听各项TF
bool SensorProcessorBase::updateTransformations(const ros::Time& timeStamp)
{
  try {
    transformListener_.waitForTransform(sensorFrameId_, mapFrameId_, timeStamp, ros::Duration(1.0));
    
    tf::StampedTransform transformTf;
    // transformListener_.lookupTransform(mapFrameId_, sensorFrameId_, ros::Time(0), transformTf);
    transformListener_.lookupTransform(mapFrameId_, sensorFrameId_, ros::Time(0), M2StransformTf);  // ros::Time(0)
    poseTFToEigen(M2StransformTf, transformationSensorToMap_);
    
    // transformListener_.lookupTransform(robotBaseFrameId_, sensorFrameId_, ros::Time(0), transformTf);  // TODO Why wrong direction?
    transformListener_.lookupTransform(robotBaseFrameId_, sensorFrameId_, ros::Time(0), transformTf);  // TODO Why wrong direction?
    Eigen::Affine3d transform;
    poseTFToEigen(transformTf, transform);
    rotationBaseToSensor_.setMatrix(transform.rotation().matrix());
    translationBaseToSensorInBaseFrame_.toImplementation() = transform.translation();
    
    // transformListener_.lookupTransform(mapFrameId_, robotBaseFrameId_, ros::Time(0), transformTf);  // TODO Why wrong direction?
    transformListener_.lookupTransform(mapFrameId_, robotBaseFrameId_, ros::Time(0), transformTf);  // TODO Why wrong direction?
    poseTFToEigen(transformTf, transform);
    rotationMapToBase_.setMatrix(transform.rotation().matrix());
    translationMapToBaseInMapFrame_.toImplementation() = transform.translation();
    return true;
  } catch (tf::TransformException &ex) {
    ROS_ERROR("%s", ex.what());
    return false;
  }
}

bool SensorProcessorBase::GPUPointCloudprocess(
		pcl::PointCloud<Anypoint>::Ptr pointCloudSensorframe,
		pcl::PointCloud<Anypoint>::Ptr pointCloudTransformed,
		const std::string& targetFrame, 
    int *point_colorR,
    int *point_colorG,
    int *point_colorB,
    int *point_index,
    float* point_intensity,
    float *point_height,
    float *point_var)
{
  ros::Time timeStamp;
  timeStamp.fromNSec(1000 * pointCloudSensorframe->header.stamp);
  const std::string inputFrameId(pointCloudSensorframe->header.frame_id);

  tf::StampedTransform transformTf;
  try {
    transformListener_.waitForTransform(targetFrame, inputFrameId, timeStamp, ros::Duration(0.5));
    transformListener_.lookupTransform(targetFrame, inputFrameId, timeStamp, transformTf);
  } catch (tf::TransformException &ex) {
    ROS_ERROR("%s", ex.what());
    return false;
  }

  ros::Time begin_time = ros::Time::now ();
  int point_num = pointCloudSensorframe->size();

	float point_x[point_num];
	float point_y[point_num];
	float point_z[point_num];
  float point_x_ts[point_num];
	float point_y_ts[point_num];
	
	for (size_t i = 0; i < point_num; ++i) {
		auto& point = pointCloudSensorframe->points[i];
    point_x[i] = point.x;
		point_y[i] = point.y;
		point_z[i] = point.z;
    point_colorR[i] = point.r;
    point_colorG[i] = point.g;
    point_colorB[i] = point.b;
    point_intensity[i] = point.intensity;
	} 

  Eigen::Affine3d transform;
 
  poseTFToEigen(transformTf, transform);
  
  Eigen::Matrix4f Transform;
  
  for (int i = 0; i < 4; i ++)
    for (int j = 0; j < 4; j ++)
      Transform(i, j) = transform(i, j);
  // std::cout << "process Timestamp::"  << timeStamp << std::endl;
  //std::cout << "transform::"  << Transform << std::endl;
  double t1;
  double relativeLowerThreshold = translationMapToBaseInMapFrame_.z() + ignorePointsLowerThreshold_;
  double relativeUpperThreshold = translationMapToBaseInMapFrame_.z() + ignorePointsUpperThreshold_;
  pcl::PointCloud<Anypoint> Ps;
  pcl::PointCloud<Anypoint> Ps_ts;

  float min_r;
  float beam_a; 
  float beam_c; 
  Eigen::RowVector3f sensorJacobian; 
  Eigen::Matrix3f rotationVariance;
  Eigen::Matrix3f C_SB_transpose;
  Eigen::RowVector3f P_mul_C_BM_transpose; 
  Eigen::Matrix3f B_r_BS_skew; 
  /* 
  float point_var[point_num];
  float point_var_value[point_num];
  int mapindex[point_num];
   */

  rotationVariance << 0, 0, 0,
                      0, 0, 0,
                      0, 0, 0;

  readcomputerparam(&min_r, &beam_a, &beam_c, &sensorJacobian, &C_SB_transpose, &P_mul_C_BM_transpose, &B_r_BS_skew);

  Process_points(point_index, point_x, point_y, point_z, point_var, point_x_ts, point_y_ts, point_height, Transform, point_num, relativeLowerThreshold, relativeUpperThreshold, min_r, beam_a, beam_c, sensorJacobian, rotationVariance, C_SB_transpose, P_mul_C_BM_transpose, B_r_BS_skew);

	return true;
}


bool SensorProcessorBase::transformPointCloud(
		pcl::PointCloud<Anypoint>::ConstPtr pointCloud,
		pcl::PointCloud<Anypoint>::Ptr pointCloudTransformed,
		const std::string& targetFrame)
{
  ros::Time timeStamp;
  timeStamp.fromNSec(1000 * pointCloud->header.stamp);
  const std::string inputFrameId(pointCloud->header.frame_id);

  tf::StampedTransform transformTf;
  //tf::TransformListener& transformListener_;
  try {
    transformListener_.waitForTransform(targetFrame, inputFrameId, timeStamp, ros::Duration(1.0));
    transformListener_.lookupTransform(targetFrame, inputFrameId, timeStamp, transformTf);
  } catch (tf::TransformException &ex) {
    ROS_ERROR("%s", ex.what());
    return false;
  }

  Eigen::Affine3d transform;
  poseTFToEigen(transformTf, transform);
  pcl::transformPointCloud(*pointCloud, *pointCloudTransformed, transform.cast<float>());
  pointCloudTransformed->header.frame_id = targetFrame;

	ROS_DEBUG("Point cloud transformed to frame %s for time stamp %f.", targetFrame.c_str(),
			ros::Time(pointCloudTransformed->header.stamp).toSec());
	return true;
}

void SensorProcessorBase::removePointsOutsideLimits(
    pcl::PointCloud<Anypoint>::ConstPtr reference, std::vector<pcl::PointCloud<Anypoint>::Ptr>& pointClouds)
{
  if (!std::isfinite(ignorePointsLowerThreshold_) && !std::isfinite(ignorePointsUpperThreshold_)) return;
  ROS_DEBUG("Limiting point cloud to the height interval of [%f, %f] relative to the robot base.", ignorePointsLowerThreshold_, ignorePointsUpperThreshold_);

  pcl::PassThrough<Anypoint> passThroughFilter(true);
  passThroughFilter.setInputCloud(reference);
  passThroughFilter.setFilterFieldName("z"); // TODO: Should this be configurable?
  double relativeLowerThreshold = translationMapToBaseInMapFrame_.z() + ignorePointsLowerThreshold_;
  double relativeUpperThreshold = translationMapToBaseInMapFrame_.z() + ignorePointsUpperThreshold_;
  passThroughFilter.setFilterLimits(relativeLowerThreshold, relativeUpperThreshold);
  pcl::IndicesPtr insideIndeces(new std::vector<int>);
  passThroughFilter.filter(*insideIndeces);
  for (auto& pointCloud : pointClouds) {
    pcl::ExtractIndices<Anypoint> extractIndicesFilter;
    extractIndicesFilter.setInputCloud(pointCloud);
    extractIndicesFilter.setIndices(insideIndeces);
    pcl::PointCloud<Anypoint> tempPointCloud;
    extractIndicesFilter.filter(tempPointCloud);
    pointCloud->swap(tempPointCloud);
  }

  ROS_DEBUG("removePointsOutsideLimits() reduced point cloud to %i points.", (int) pointClouds[0]->size());
}


void SensorProcessorBase::readcomputerparam(float *min_r, float *beam_a, float *beam_c, Eigen::RowVector3f *sensorJacobian, Eigen::Matrix3f *C_SB_transpose, Eigen::RowVector3f *P_mul_C_BM_transpose, Eigen::Matrix3f *B_r_BS_skew)
{
	const Eigen::RowVector3f projectionVector = Eigen::RowVector3f::UnitZ();

	// Sensor Jacobian (J_s).
	*sensorJacobian = projectionVector * (rotationMapToBase_.transposed() * rotationBaseToSensor_.transposed()).toImplementation().cast<float>();

	// Robot rotation covariance matrix (Sigma_q).
	//*rotationVariance = robotPoseCovariance.bottomRightCorner(3, 3).cast<float>();

	// Preparations for robot rotation Jacobian (J_q) to minimize computation for every point in point cloud.
	const Eigen::Matrix3f C_BM_transpose = rotationMapToBase_.transposed().toImplementation().cast<float>();
	*P_mul_C_BM_transpose = projectionVector * C_BM_transpose;
	*C_SB_transpose = rotationBaseToSensor_.transposed().toImplementation().cast<float>();
	*B_r_BS_skew = kindr::getSkewMatrixFromVector(Eigen::Vector3f(translationBaseToSensorInBaseFrame_.toImplementation().cast<float>()));

	*min_r = sensorParameters_.at("min_radius");
	*beam_c = sensorParameters_.at("beam_constant");
	*beam_a = sensorParameters_.at("beam_angle");

}

} /* namespace elevation_mapping */

