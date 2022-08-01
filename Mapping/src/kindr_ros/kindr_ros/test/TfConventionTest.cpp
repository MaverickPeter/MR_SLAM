/*
 * tf_kindr_test.cpp
 *
 *  Created on: Jun 17, 2016
 *      Author: Christian Gehring
 */

#include <gtest/gtest.h>
#include <kindr/Core>
#include <kindr/rotations/gtest_rotations.hpp>
#include <tf/tf.h>

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////

namespace kindr {

template<>
class RotationConversion<tf::Quaternion, tf::Vector3, double> {
  typedef tf::Quaternion Rotation;
  typedef tf::Vector3 Vector;
 public:

  inline static void convertToOtherRotation(Rotation& out, const kindr::RotationQuaternion<double>& in) {
    kindr::RotationQuaternion<double> in2 = in;
    out = tf::Quaternion(in2.x(), in2.y(), in2.z(), in2.w());
  }

  inline static void convertToKindr(kindr::RotationQuaternion<double>& out, Rotation& in) {
    out = kindr::RotationQuaternion<double>(in.w(), in.x(), in.y(), in.z());
  }

  inline static void convertToVelocityVector(Vector& out, Rotation& rot, const Eigen::Matrix<double,3,1>& in) {
    out = tf::Vector3(in.x(), in.y(), in.z());
  }

  inline static void concatenate(Rotation& res, const Rotation& rot1, const Rotation& rot2) {
    res = rot2*rot1;
  }

  inline static void getRotationMatrixFromRotation(Eigen::Matrix3d& rotationMatrix, const Rotation& quaternion) {
    tf::Matrix3x3 tfRotationmatrix;
    tfRotationmatrix.setRotation(quaternion);
    for (int i=0; i<3; i++) {
      rotationMatrix(i, 0) = tfRotationmatrix.getRow(i).x();
      rotationMatrix(i, 1) = tfRotationmatrix.getRow(i).y();
      rotationMatrix(i, 2) = tfRotationmatrix.getRow(i).z();
    }
  }

  inline static void rotateVector(Eigen::Matrix<double,3,1>& A_r, const Rotation& rotationBToA, const Eigen::Matrix<double,3,1>& B_r) {
    tf::Vector3 A_v = tf::quatRotate(rotationBToA, tf::Vector3(B_r.x(), B_r.y(), B_r.z()));
    A_r.x() = A_v.x();
    A_r.y() = A_v.y();
    A_r.z() = A_v.z();
  }

};


template<>
class RotationConversion<tf::Matrix3x3, tf::Vector3, double> {
  typedef tf::Matrix3x3 Rotation;
  typedef tf::Vector3 Vector;
 public:

  inline static void convertToOtherRotation(Rotation& out, const kindr::RotationQuaternion<double>& in) {
    kindr::RotationQuaternion<double> in2 = in;
    out.setRotation(tf::Quaternion(in2.x(), in2.y(), in2.z(), in2.w()));
  }

  inline static void convertToKindr(kindr::RotationQuaternion<double>& out, Rotation& matrix) {
    tf::Quaternion quat;
    matrix.getRotation(quat);
    out = kindr::RotationQuaternion<double>(quat.w(), quat.x(), quat.y(), quat.z());
  }

  inline static void convertToOtherVelocityVector(Vector& out, Rotation& rot, const Eigen::Matrix<double,3,1>& in) {
    out = tf::Vector3(in.x(), in.y(), in.z());
  }

  inline static void concatenate(Rotation& res, const Rotation& rot1, const Rotation& rot2) {
    res = rot2*rot1;
  }

  inline static void getRotationMatrixFromRotation(Eigen::Matrix3d& rotationMatrix, const Rotation& matrix) {
    for (int i=0; i<3; i++) {
      rotationMatrix(i, 0) = matrix.getRow(i).x();
      rotationMatrix(i, 1) = matrix.getRow(i).y();
      rotationMatrix(i, 2) = matrix.getRow(i).z();
    }
  }

  inline static void rotateVector(Eigen::Matrix<double,3,1>& A_r, const Rotation& rotationBToA, const Eigen::Matrix<double,3,1>& B_r) {
    tf::Vector3 B_v(B_r.x(), B_r.y(), B_r.z());
    tf::Vector3 A_v = rotationBToA*B_v;
    A_r.x() = A_v.x();
    A_r.y() = A_v.y();
    A_r.z() = A_v.z();
  }
};


} // namespace kindr
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////

TEST(TfConventionTest, Concatenation) {
  kindr::ConventionTest<tf::Quaternion, tf::Vector3, double>::testConcatenation();
  kindr::ConventionTest<tf::Matrix3x3, tf::Vector3, double>::testConcatenation();
}

TEST(TfConventionTest, Rotation) {
  kindr::ConventionTest<tf::Quaternion, tf::Vector3, double>::testRotationMatrix();
  kindr::ConventionTest<tf::Matrix3x3, tf::Vector3, double>::testRotationMatrix();
}

//TEST(TfConventionTest, BoxPlus) {
//  kindr::ConventionTest<tf::Quaternion, tf::Vector3, double>::testBoxPlus();
//}

TEST(TfConventionTest, GeometricalInterpretation) {
  kindr::ConventionTest<tf::Quaternion, tf::Vector3, double>::testGeometricalInterpretation();
  kindr::ConventionTest<tf::Matrix3x3, tf::Vector3, double>::testGeometricalInterpretation();
}
