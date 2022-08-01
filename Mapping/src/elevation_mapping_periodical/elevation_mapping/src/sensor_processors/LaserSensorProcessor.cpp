/*
 * KinectSensorProcessor.cpp
 *
 *  Created on: Feb 5, 2014
 *      Author: Péter Fankhauser
 *   Institute: ETH Zurich, ANYbotics
 */

#include <elevation_mapping/sensor_processors/LaserSensorProcessor.hpp>

#include <pcl/filters/passthrough.h>
#include <vector>
#include <limits>
#include <string>

//int func(float *h_result, float *point_x, float *point_y, float *point_z, int point_num, float min_r, float beam_a, float beam_c, Eigen::RowVector3f sensorJacobian, Eigen::Matrix3f rotationVariance, Eigen::Matrix3f C_SB_transpose, Eigen::RowVector3f P_mul_C_BM_transpose, Eigen::Matrix3f B_r_BS_skew);

namespace elevation_mapping {

/*!
 * Anisotropic laser range sensor model:
 * standardDeviationInBeamDirection = minRadius
 * standardDeviationOfBeamRadius = beamConstant + beamAngle * measurementDistance
 *
 * Taken from: Pomerleau, F.; Breitenmoser, A; Ming Liu; Colas, F.; Siegwart, R.,
 * "Noise characterization of depth sensors for surface inspections,"
 * International Conference on Applied Robotics for the Power Industry (CARPI), 2012.
 */

LaserSensorProcessor::LaserSensorProcessor(ros::NodeHandle& nodeHandle, tf::TransformListener& transformListener)
    : SensorProcessorBase(nodeHandle, transformListener)
{

}

LaserSensorProcessor::~LaserSensorProcessor()
{

}

bool LaserSensorProcessor::readParameters()
{
  SensorProcessorBase::readParameters();
  nodeHandle_.param("sensor_processor/min_radius", sensorParameters_["min_radius"], 0.0);
  nodeHandle_.param("sensor_processor/beam_angle", sensorParameters_["beam_angle"], 0.0);
  nodeHandle_.param("sensor_processor/beam_constant", sensorParameters_["beam_constant"], 0.0);
  return true;
}

bool LaserSensorProcessor::cleanPointCloud(const pcl::PointCloud<Anypoint>::Ptr pointCloud)
{
  pcl::PointCloud<Anypoint> tempPointCloud;
  std::vector<int> indices;
  pcl::removeNaNFromPointCloud(*pointCloud, tempPointCloud, indices);
  tempPointCloud.is_dense = true;
  pointCloud->swap(tempPointCloud);
  ROS_DEBUG("ElevationMap: cleanPointCloud() reduced point cloud to %i points.", static_cast<int>(pointCloud->size()));
  return true;
}



bool LaserSensorProcessor::computeVariances(
		const pcl::PointCloud<Anypoint>::ConstPtr pointCloud,
		const Eigen::Matrix<double, 6, 6>& robotPoseCovariance,
		Eigen::VectorXf& variances)
{
	variances.resize(pointCloud->size());

	// Projection vector (P).
	const Eigen::RowVector3f projectionVector = Eigen::RowVector3f::UnitZ();

	// Sensor Jacobian (J_s).
	const Eigen::RowVector3f sensorJacobian = projectionVector * (rotationMapToBase_.transposed() * rotationBaseToSensor_.transposed()).toImplementation().cast<float>();

	// Robot rotation covariance matrix (Sigma_q).
	Eigen::Matrix3f rotationVariance = robotPoseCovariance.bottomRightCorner(3, 3).cast<float>();

	// Preparations for robot rotation Jacobian (J_q) to minimize computation for every point in point cloud.
	const Eigen::Matrix3f C_BM_transpose = rotationMapToBase_.transposed().toImplementation().cast<float>();
	const Eigen::RowVector3f P_mul_C_BM_transpose = projectionVector * C_BM_transpose;
	const Eigen::Matrix3f C_SB_transpose = rotationBaseToSensor_.transposed().toImplementation().cast<float>();
	const Eigen::Matrix3f B_r_BS_skew = kindr::getSkewMatrixFromVector(Eigen::Vector3f(translationBaseToSensorInBaseFrame_.toImplementation().cast<float>()));

	float min_r = sensorParameters_.at("min_radius");
	float beam_c = sensorParameters_.at("beam_constant");
	float beam_a = sensorParameters_.at("beam_angle");
    
	std::cout << "methold2-----------------------------:" << std::endl;
	std::cout << "min_r is:" << min_r << std::endl;
	std::cout << "beam_a is:" << beam_a << std::endl;
	std::cout << "beam_c is:" << beam_c << std::endl;
	std::cout << "sensorJacobian is:" << sensorJacobian << std::endl;
	std::cout << "rotationVariance is:" << rotationVariance << std::endl;
	std::cout << "C_SB_transpose is:" << C_SB_transpose << std::endl;
	std::cout << "P_mul_C_BM_transpose is:" << P_mul_C_BM_transpose << std::endl;
	std::cout << "B_r_BS_skew is:" << B_r_BS_skew << std::endl;

	int point_num = pointCloud->size();
	float point_x[point_num];
	float point_y[point_num];
	float point_z[point_num];
    float cuda_var[point_num];
   
	for (size_t i = 0; i < point_num; ++i) {
		auto& point = pointCloud->points[i];
        point_x[i] = point.x;
		point_y[i] = point.y;
		point_z[i] = point.z;
	} 
	
	//Eigen::Vector3f pointVector(point.x, point.y, point.z); // S_r_SP
	//cloud_device.upload(Cloud.points);
	//
	//func();
	
    ros::Time begin_time = ros::Time::now ();
	//func(cuda_var, point_x, point_y, point_z, point_num, min_r, beam_a, beam_c, sensorJacobian, rotationVariance, C_SB_transpose, P_mul_C_BM_transpose, B_r_BS_skew);
  	double t2 = (ros::Time::now () - begin_time).toSec ();
	  
	std::cout << "laser var88:" << cuda_var[88]<<std::endl;
    std::cout << "laser var1321:" << cuda_var[1321]<<std::endl;
	std::cout << "cuda_var:" << point_num <<std::endl;
	for(size_t i = 0; i < point_num; ++i){
		if(variances(i) != cuda_var[i])
		{
			std::cout << i <<"!!!!!!!!!!!!!!!!!!!!!" <<std::endl;
		    std::cout << variances(i) << std::endl;
			std::cout << cuda_var[i] << std::endl;
		}
		variances(i) = cuda_var[i];
	}
	
	/*
  for (size_t i = 0; i < pointCloud->size(); ++i) {
		// For every point in point cloud.

		// Preparation.
		auto& point = pointCloud->points[i];
		Eigen::Vector3f pointVector(point.x, point.y, point.z); // S_r_SP
		float heightVariance = 0.0; // sigma_p

		// Measurement distance.
		float measurementDistance = pointVector.norm();

		// Compute sensor covariance matrix (Sigma_S) with sensor model.
		float varianceNormal = pow(sensorParameters_.at("min_radius"), 2);
		float varianceLateral = pow(sensorParameters_.at("beam_constant") + sensorParameters_.at("beam_angle") * measurementDistance, 2);
		Eigen::Matrix3f sensorVariance = Eigen::Matrix3f::Zero();
		sensorVariance.diagonal() << varianceLateral, varianceLateral, varianceNormal;

		// Robot rotation Jacobian (J_q).
		const Eigen::Matrix3f C_SB_transpose_times_S_r_SP_skew = kindr::getSkewMatrixFromVector(Eigen::Vector3f(C_SB_transpose * pointVector));
		Eigen::RowVector3f rotationJacobian = P_mul_C_BM_transpose * (C_SB_transpose_times_S_r_SP_skew + B_r_BS_skew);

		// Measurement variance for map (error propagation law).
		heightVariance = rotationJacobian * rotationVariance * rotationJacobian.transpose();
		heightVariance += sensorJacobian * sensorVariance * sensorJacobian.transpose();

		// Copy to list.
		variances(i) = heightVariance;
	}
	*/
	return true;
}

} /* namespace */
