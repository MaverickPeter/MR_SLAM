/*
 * Copyright (c) Koji Terada
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

/** \author Wim Meeussen */


#include <tf2_eigen/tf2_eigen.h>
#include <gtest/gtest.h>
#include <tf2/convert.h>

TEST(TfEigen, ConvertVector3dStamped)
{
  const tf2::Stamped<Eigen::Vector3d> v(Eigen::Vector3d(1,2,3), ros::Time(5), "test");

  tf2::Stamped<Eigen::Vector3d> v1;
  geometry_msgs::PointStamped p1;
  tf2::convert(v, p1);
  tf2::convert(p1, v1);

  EXPECT_EQ(v, v1);
}

TEST(TfEigen, ConvertVector3d)
{
  const Eigen::Vector3d v(1,2,3);

  Eigen::Vector3d v1;
  geometry_msgs::Point p1;
  tf2::convert(v, p1);
  tf2::convert(p1, v1);

  EXPECT_EQ(v, v1);
}

TEST(TfEigen, ConvertQuaterniondStamped)
{
  const tf2::Stamped<Eigen::Quaterniond> v(Eigen::Quaterniond(1,2,3,4), ros::Time(5), "test");

  tf2::Stamped<Eigen::Quaterniond> v1;
  geometry_msgs::QuaternionStamped p1;
  tf2::convert(v, p1);
  tf2::convert(p1, v1);

  EXPECT_EQ(v.frame_id_, v1.frame_id_);
  EXPECT_EQ(v.stamp_, v1.stamp_);
  EXPECT_EQ(v.w(), v1.w());
  EXPECT_EQ(v.x(), v1.x());
  EXPECT_EQ(v.y(), v1.y());
  EXPECT_EQ(v.z(), v1.z());
}

TEST(TfEigen, ConvertQuaterniond)
{
  const Eigen::Quaterniond v(1,2,3,4);

  Eigen::Quaterniond v1;
  geometry_msgs::Quaternion p1;
  tf2::convert(v, p1);
  tf2::convert(p1, v1);

  EXPECT_EQ(v.w(), v1.w());
  EXPECT_EQ(v.x(), v1.x());
  EXPECT_EQ(v.y(), v1.y());
  EXPECT_EQ(v.z(), v1.z());
}

TEST(TfEigen, TransformQuaterion) {
 const tf2::Stamped<Eigen::Quaterniond> in(Eigen::Quaterniond(Eigen::AngleAxisd(1, Eigen::Vector3d::UnitX())), ros::Time(5), "test");
 const Eigen::Isometry3d iso(Eigen::AngleAxisd(M_PI/2, Eigen::Vector3d::UnitY()));
 const Eigen::Affine3d affine(iso);
 const tf2::Stamped<Eigen::Quaterniond> expected(Eigen::Quaterniond(Eigen::AngleAxisd(1, Eigen::Vector3d::UnitZ())), ros::Time(10), "expected");

 geometry_msgs::TransformStamped trafo = tf2::eigenToTransform(affine);
 trafo.header.stamp = ros::Time(10);
 trafo.header.frame_id = "expected";

 tf2::Stamped<Eigen::Quaterniond> out;
 tf2::doTransform(in, out, trafo);

 EXPECT_TRUE(out.isApprox(expected));
 EXPECT_EQ(expected.stamp_, out.stamp_);
 EXPECT_EQ(expected.frame_id_, out.frame_id_);

 // the same using Isometry
 trafo = tf2::eigenToTransform(iso);
 trafo.header.stamp = ros::Time(10);
 trafo.header.frame_id = "expected";
 tf2::doTransform(in, out, trafo);

 EXPECT_TRUE(out.isApprox(expected));
 EXPECT_EQ(expected.stamp_, out.stamp_);
 EXPECT_EQ(expected.frame_id_, out.frame_id_);
}

TEST(TfEigen, ConvertAffine3dStamped)
{
  const Eigen::Affine3d v_nonstamped(Eigen::Translation3d(1,2,3) * Eigen::AngleAxis<double>(1, Eigen::Vector3d::UnitX()));
  const tf2::Stamped<Eigen::Affine3d> v(v_nonstamped, ros::Time(42), "test_frame");

  tf2::Stamped<Eigen::Affine3d> v1;
  geometry_msgs::PoseStamped p1;
  tf2::convert(v, p1);
  tf2::convert(p1, v1);

  EXPECT_EQ(v.translation(), v1.translation());
  EXPECT_EQ(v.rotation(), v1.rotation());
  EXPECT_EQ(v.frame_id_, v1.frame_id_);
  EXPECT_EQ(v.stamp_, v1.stamp_);
}

TEST(TfEigen, ConvertIsometry3dStamped)
{
  const Eigen::Isometry3d v_nonstamped(Eigen::Translation3d(1,2,3) * Eigen::AngleAxis<double>(1, Eigen::Vector3d::UnitX()));
  const tf2::Stamped<Eigen::Isometry3d> v(v_nonstamped, ros::Time(42), "test_frame");

  tf2::Stamped<Eigen::Isometry3d> v1;
  geometry_msgs::PoseStamped p1;
  tf2::convert(v, p1);
  tf2::convert(p1, v1);

  EXPECT_EQ(v.translation(), v1.translation());
  EXPECT_EQ(v.rotation(), v1.rotation());
  EXPECT_EQ(v.frame_id_, v1.frame_id_);
  EXPECT_EQ(v.stamp_, v1.stamp_);
}

TEST(TfEigen, ConvertAffine3d)
{
  const Eigen::Affine3d v(Eigen::Translation3d(1,2,3) * Eigen::AngleAxis<double>(1, Eigen::Vector3d::UnitX()));

  Eigen::Affine3d v1;
  geometry_msgs::Pose p1;
  tf2::convert(v, p1);
  tf2::convert(p1, v1);

  EXPECT_EQ(v.translation(), v1.translation());
  EXPECT_EQ(v.rotation(), v1.rotation());
}

TEST(TfEigen, ConvertIsometry3d)
{
  const Eigen::Isometry3d v(Eigen::Translation3d(1,2,3) * Eigen::AngleAxis<double>(1, Eigen::Vector3d::UnitX()));

  Eigen::Isometry3d v1;
  geometry_msgs::Pose p1;
  tf2::convert(v, p1);
  tf2::convert(p1, v1);

  EXPECT_EQ(v.translation(), v1.translation());
  EXPECT_EQ(v.rotation(), v1.rotation());
}

TEST(TfEigen, ConvertTransform)
{
  Eigen::Matrix4d tm;

  double alpha = M_PI/4.0;
  double theta = M_PI/6.0;
  double gamma = M_PI/12.0;

  tm << cos(theta)*cos(gamma),-cos(theta)*sin(gamma),sin(theta), 1, //
  cos(alpha)*sin(gamma)+sin(alpha)*sin(theta)*cos(gamma),cos(alpha)*cos(gamma)-sin(alpha)*sin(theta)*sin(gamma),-sin(alpha)*cos(theta), 2, //
  sin(alpha)*sin(gamma)-cos(alpha)*sin(theta)*cos(gamma),cos(alpha)*sin(theta)*sin(gamma)+sin(alpha)*cos(gamma),cos(alpha)*cos(theta), 3, //
  0, 0, 0, 1;

  Eigen::Affine3d T(tm);

  geometry_msgs::TransformStamped msg = tf2::eigenToTransform(T);
  Eigen::Affine3d Tback = tf2::transformToEigen(msg);

  EXPECT_TRUE(T.isApprox(Tback));
  EXPECT_TRUE(tm.isApprox(Tback.matrix()));

  // same for Isometry
  Eigen::Isometry3d I(tm);

  msg = tf2::eigenToTransform(T);
  Eigen::Isometry3d Iback = tf2::transformToEigen(msg);

  EXPECT_TRUE(I.isApprox(Iback));
  EXPECT_TRUE(tm.isApprox(Iback.matrix()));
}



int main(int argc, char **argv){
  testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
