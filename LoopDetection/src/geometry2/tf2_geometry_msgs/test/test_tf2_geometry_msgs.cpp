/*
 * Copyright (c) 2008, Willow Garage, Inc.
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
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
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


#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>
#include <ros/ros.h>
#include <gtest/gtest.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

tf2_ros::Buffer* tf_buffer;
geometry_msgs::TransformStamped t;
static const double EPS = 1e-3;


TEST(TfGeometry, Frame)
{
  geometry_msgs::PoseStamped v1;
  v1.pose.position.x = 1;
  v1.pose.position.y = 2;
  v1.pose.position.z = 3;
  v1.pose.orientation.x = 1;
  v1.header.stamp = ros::Time(2);
  v1.header.frame_id = "A";

  // simple api
  geometry_msgs::PoseStamped v_simple = tf_buffer->transform(v1, "B", ros::Duration(2.0));
  EXPECT_NEAR(v_simple.pose.position.x, -9, EPS);
  EXPECT_NEAR(v_simple.pose.position.y, 18, EPS);
  EXPECT_NEAR(v_simple.pose.position.z, 27, EPS);
  EXPECT_NEAR(v_simple.pose.orientation.x, 0.0, EPS);
  EXPECT_NEAR(v_simple.pose.orientation.y, 0.0, EPS);
  EXPECT_NEAR(v_simple.pose.orientation.z, 0.0, EPS);
  EXPECT_NEAR(v_simple.pose.orientation.w, 1.0, EPS);
  
  // advanced api
  geometry_msgs::PoseStamped v_advanced = tf_buffer->transform(v1, "B", ros::Time(2.0),
							      "A", ros::Duration(3.0));
  EXPECT_NEAR(v_advanced.pose.position.x, -9, EPS);
  EXPECT_NEAR(v_advanced.pose.position.y, 18, EPS);
  EXPECT_NEAR(v_advanced.pose.position.z, 27, EPS);
  EXPECT_NEAR(v_advanced.pose.orientation.x, 0.0, EPS);
  EXPECT_NEAR(v_advanced.pose.orientation.y, 0.0, EPS);
  EXPECT_NEAR(v_advanced.pose.orientation.z, 0.0, EPS);
  EXPECT_NEAR(v_advanced.pose.orientation.w, 1.0, EPS);
}

TEST(TfGeometry, PoseWithCovarianceStamped)
{
  geometry_msgs::PoseWithCovarianceStamped v1;
  v1.pose.pose.position.x = 1;
  v1.pose.pose.position.y = 2;
  v1.pose.pose.position.z = 3;
  v1.pose.pose.orientation.x = 1;
  v1.header.stamp = ros::Time(2);
  v1.header.frame_id = "A";
  v1.pose.covariance[0] = 1;
  v1.pose.covariance[7] = 1;
  v1.pose.covariance[14] = 1;
  v1.pose.covariance[21] = 1;
  v1.pose.covariance[28] = 1;
  v1.pose.covariance[35] = 1;
  
  // simple api
  const geometry_msgs::PoseWithCovarianceStamped v_simple = tf_buffer->transform(v1, "B", ros::Duration(2.0));
  EXPECT_NEAR(v_simple.pose.pose.position.x, -9, EPS);
  EXPECT_NEAR(v_simple.pose.pose.position.y, 18, EPS);
  EXPECT_NEAR(v_simple.pose.pose.position.z, 27, EPS);
  EXPECT_NEAR(v_simple.pose.pose.orientation.x, 0.0, EPS);
  EXPECT_NEAR(v_simple.pose.pose.orientation.y, 0.0, EPS);
  EXPECT_NEAR(v_simple.pose.pose.orientation.z, 0.0, EPS);
  EXPECT_NEAR(v_simple.pose.pose.orientation.w, 1.0, EPS);
  
  // no rotation in this transformation, so no change to covariance
  EXPECT_NEAR(v_simple.pose.covariance[0], 1.0, EPS);
  EXPECT_NEAR(v_simple.pose.covariance[7], 1.0, EPS);
  EXPECT_NEAR(v_simple.pose.covariance[14], 1.0, EPS);
  EXPECT_NEAR(v_simple.pose.covariance[21], 1.0, EPS);
  EXPECT_NEAR(v_simple.pose.covariance[28], 1.0, EPS);
  EXPECT_NEAR(v_simple.pose.covariance[35], 1.0, EPS);
  
  // advanced api
  const geometry_msgs::PoseWithCovarianceStamped v_advanced = tf_buffer->transform(v1, "B", ros::Time(2.0),
							      "A", ros::Duration(3.0));
  EXPECT_NEAR(v_advanced.pose.pose.position.x, -9, EPS);
  EXPECT_NEAR(v_advanced.pose.pose.position.y, 18, EPS);
  EXPECT_NEAR(v_advanced.pose.pose.position.z, 27, EPS);
  EXPECT_NEAR(v_advanced.pose.pose.orientation.x, 0.0, EPS);
  EXPECT_NEAR(v_advanced.pose.pose.orientation.y, 0.0, EPS);
  EXPECT_NEAR(v_advanced.pose.pose.orientation.z, 0.0, EPS);
  EXPECT_NEAR(v_advanced.pose.pose.orientation.w, 1.0, EPS);
  
  // no rotation in this transformation, so no change to covariance
  EXPECT_NEAR(v_advanced.pose.covariance[0], 1.0, EPS);
  EXPECT_NEAR(v_advanced.pose.covariance[7], 1.0, EPS);
  EXPECT_NEAR(v_advanced.pose.covariance[14], 1.0, EPS);
  EXPECT_NEAR(v_advanced.pose.covariance[21], 1.0, EPS);
  EXPECT_NEAR(v_advanced.pose.covariance[28], 1.0, EPS);
  EXPECT_NEAR(v_advanced.pose.covariance[35], 1.0, EPS);
  
  /** now add rotation to transform to test the effect on covariance **/
  
  // rotate pi/2 radians about x-axis
  geometry_msgs::TransformStamped t_rot;
  t_rot.transform.rotation = tf2::toMsg(tf2::Quaternion(tf2::Vector3(1,0,0), M_PI/2));
  t_rot.header.stamp = ros::Time(2.0);
  t_rot.header.frame_id = "A";
  t_rot.child_frame_id = "rotated";
  tf_buffer->setTransform(t_rot, "rotation_test");
  
  // need to put some covariance in the matrix
  v1.pose.covariance[1] = 1;
  v1.pose.covariance[6] = 1;
  v1.pose.covariance[12] = 1;
  
  // perform rotation
  const geometry_msgs::PoseWithCovarianceStamped v_rotated = tf_buffer->transform(v1, "rotated", ros::Duration(2.0));

  // the covariance matrix should now be transformed
  EXPECT_NEAR(v_rotated.pose.covariance[0], 1.0, EPS);
  EXPECT_NEAR(v_rotated.pose.covariance[1], 0.0, EPS);
  EXPECT_NEAR(v_rotated.pose.covariance[2],-1.0, EPS);
  EXPECT_NEAR(v_rotated.pose.covariance[6], 1.0, EPS);
  EXPECT_NEAR(v_rotated.pose.covariance[7], 1.0, EPS);
  EXPECT_NEAR(v_rotated.pose.covariance[8], 0.0, EPS);
  EXPECT_NEAR(v_rotated.pose.covariance[12],-1.0, EPS);
  EXPECT_NEAR(v_rotated.pose.covariance[13], 0.0, EPS);
  EXPECT_NEAR(v_rotated.pose.covariance[14], 1.0, EPS);
  
  // set buffer back to original transform
  tf_buffer->setTransform(t, "test");
}
  
TEST(TfGeometry, Transform)
{
  geometry_msgs::TransformStamped v1;
  v1.transform.translation.x = 1;
  v1.transform.translation.y = 2;
  v1.transform.translation.z = 3;
  v1.transform.rotation.x = 1;
  v1.header.stamp = ros::Time(2);
  v1.header.frame_id = "A";

  // simple api
  geometry_msgs::TransformStamped v_simple = tf_buffer->transform(v1, "B", ros::Duration(2.0));
  EXPECT_NEAR(v_simple.transform.translation.x, -9, EPS);
  EXPECT_NEAR(v_simple.transform.translation.y, 18, EPS);
  EXPECT_NEAR(v_simple.transform.translation.z, 27, EPS);
  EXPECT_NEAR(v_simple.transform.rotation.x, 0.0, EPS);
  EXPECT_NEAR(v_simple.transform.rotation.y, 0.0, EPS);
  EXPECT_NEAR(v_simple.transform.rotation.z, 0.0, EPS);
  EXPECT_NEAR(v_simple.transform.rotation.w, 1.0, EPS);
  

  // advanced api
  geometry_msgs::TransformStamped v_advanced = tf_buffer->transform(v1, "B", ros::Time(2.0),
							      "A", ros::Duration(3.0));
  EXPECT_NEAR(v_advanced.transform.translation.x, -9, EPS);
  EXPECT_NEAR(v_advanced.transform.translation.y, 18, EPS);
  EXPECT_NEAR(v_advanced.transform.translation.z, 27, EPS);
  EXPECT_NEAR(v_advanced.transform.rotation.x, 0.0, EPS);
  EXPECT_NEAR(v_advanced.transform.rotation.y, 0.0, EPS);
  EXPECT_NEAR(v_advanced.transform.rotation.z, 0.0, EPS);
  EXPECT_NEAR(v_advanced.transform.rotation.w, 1.0, EPS);
}

TEST(TfGeometry, Vector)
{
  geometry_msgs::Vector3Stamped v1, res;
  v1.vector.x = 1;
  v1.vector.y = 2;
  v1.vector.z = 3;
  v1.header.stamp = ros::Time(2.0);
  v1.header.frame_id = "A";

  // simple api
  geometry_msgs::Vector3Stamped v_simple = tf_buffer->transform(v1, "B", ros::Duration(2.0));
  EXPECT_NEAR(v_simple.vector.x, 1, EPS);
  EXPECT_NEAR(v_simple.vector.y, -2, EPS);
  EXPECT_NEAR(v_simple.vector.z, -3, EPS);

  // advanced api
  geometry_msgs::Vector3Stamped v_advanced = tf_buffer->transform(v1, "B", ros::Time(2.0),
								 "A", ros::Duration(3.0));
  EXPECT_NEAR(v_advanced.vector.x, 1, EPS);
  EXPECT_NEAR(v_advanced.vector.y, -2, EPS);
  EXPECT_NEAR(v_advanced.vector.z, -3, EPS);
}


TEST(TfGeometry, Point)
{
  geometry_msgs::PointStamped v1, res;
  v1.point.x = 1;
  v1.point.y = 2;
  v1.point.z = 3;
  v1.header.stamp = ros::Time(2.0);
  v1.header.frame_id = "A";

  // simple api
  geometry_msgs::PointStamped v_simple = tf_buffer->transform(v1, "B", ros::Duration(2.0));
  EXPECT_NEAR(v_simple.point.x, -9, EPS);
  EXPECT_NEAR(v_simple.point.y, 18, EPS);
  EXPECT_NEAR(v_simple.point.z, 27, EPS);

  // advanced api
  geometry_msgs::PointStamped v_advanced = tf_buffer->transform(v1, "B", ros::Time(2.0),
								 "A", ros::Duration(3.0));
  EXPECT_NEAR(v_advanced.point.x, -9, EPS);
  EXPECT_NEAR(v_advanced.point.y, 18, EPS);
  EXPECT_NEAR(v_advanced.point.z, 27, EPS);
}

TEST(TfGeometry, doTransformPoint)
{
  geometry_msgs::Point v1, res;
  v1.x = 2;
  v1.y = 1;
  v1.z = 3;

  geometry_msgs::TransformStamped trafo;
  trafo.transform.translation.x = -1;
  trafo.transform.translation.y = 2;
  trafo.transform.translation.z = -3;
  trafo.transform.rotation = tf2::toMsg(tf2::Quaternion(tf2::Vector3(0,0,1), -M_PI / 2.0));

  tf2::doTransform(v1, res, trafo);

  EXPECT_NEAR(res.x, 0, EPS);
  EXPECT_NEAR(res.y, 0, EPS);
  EXPECT_NEAR(res.z, 0, EPS);
}

TEST(TfGeometry, doTransformQuaterion)
{
  geometry_msgs::Quaternion v1, res;
  v1.w = 1;

  geometry_msgs::TransformStamped trafo;
  trafo.transform.translation.x = -1;
  trafo.transform.translation.y = 2;
  trafo.transform.translation.z = -3;
  trafo.transform.rotation = tf2::toMsg(tf2::Quaternion(tf2::Vector3(0,0,1), -M_PI / 2.0));

  tf2::doTransform(v1, res, trafo);

  EXPECT_NEAR(res.x, trafo.transform.rotation.x, EPS);
  EXPECT_NEAR(res.y, trafo.transform.rotation.y, EPS);
  EXPECT_NEAR(res.z, trafo.transform.rotation.z, EPS);
  EXPECT_NEAR(res.w, trafo.transform.rotation.w, EPS);
}

TEST(TfGeometry, doTransformPose)
{
  geometry_msgs::Pose v1, res;
  v1.position.x = 2;
  v1.position.y = 1;
  v1.position.z = 3;
  v1.orientation.w = 1;

  geometry_msgs::TransformStamped trafo;
  trafo.transform.translation.x = -1;
  trafo.transform.translation.y = 2;
  trafo.transform.translation.z = -3;
  trafo.transform.rotation = tf2::toMsg(tf2::Quaternion(tf2::Vector3(0,0,1), -M_PI / 2.0));

  tf2::doTransform(v1, res, trafo);

  EXPECT_NEAR(res.position.x, 0, EPS);
  EXPECT_NEAR(res.position.y, 0, EPS);
  EXPECT_NEAR(res.position.z, 0, EPS);

  EXPECT_NEAR(res.orientation.x, trafo.transform.rotation.x, EPS);
  EXPECT_NEAR(res.orientation.y, trafo.transform.rotation.y, EPS);
  EXPECT_NEAR(res.orientation.z, trafo.transform.rotation.z, EPS);
  EXPECT_NEAR(res.orientation.w, trafo.transform.rotation.w, EPS);
}

TEST(TfGeometry, doTransformVector3)
{
  geometry_msgs::Vector3 v1, res;
  v1.x = 2;
  v1.y = 1;
  v1.z = 3;

  geometry_msgs::TransformStamped trafo;
  trafo.transform.translation.x = -1;
  trafo.transform.translation.y = 2;
  trafo.transform.translation.z = -3;
  trafo.transform.rotation = tf2::toMsg(tf2::Quaternion(tf2::Vector3(0,0,1), -M_PI / 2.0));

  tf2::doTransform(v1, res, trafo);

  EXPECT_NEAR(res.x, 1, EPS);
  EXPECT_NEAR(res.y, -2, EPS);
  EXPECT_NEAR(res.z, 3, EPS);
}

TEST(TfGeometry, doTransformWrench)
{
 geometry_msgs::Wrench v1, res;
 v1.force.x = 2;
 v1.force.y = 1;
 v1.force.z = 3;
 v1.torque.x = 2;
 v1.torque.y = 1;
 v1.torque.z = 3;

 geometry_msgs::TransformStamped trafo;
 trafo.transform.translation.x = -1;
 trafo.transform.translation.y = 2;
 trafo.transform.translation.z = -3;
 trafo.transform.rotation = tf2::toMsg(tf2::Quaternion(tf2::Vector3(0,0,1), -M_PI / 2.0));

 tf2::doTransform(v1, res, trafo);
 EXPECT_NEAR(res.force.x, 1, EPS);
 EXPECT_NEAR(res.force.y, -2, EPS);
 EXPECT_NEAR(res.force.z, 3, EPS);

 EXPECT_NEAR(res.torque.x, 1, EPS);
 EXPECT_NEAR(res.torque.y, -2, EPS);
 EXPECT_NEAR(res.torque.z, 3, EPS);
}

int main(int argc, char **argv){
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "test");
  ros::NodeHandle n;

  tf_buffer = new tf2_ros::Buffer();
  tf_buffer->setUsingDedicatedThread(true);
  
  // populate buffer
  t.transform.translation.x = 10;
  t.transform.translation.y = 20;
  t.transform.translation.z = 30;
  t.transform.rotation.x = 1;
  t.header.stamp = ros::Time(2.0);
  t.header.frame_id = "A";
  t.child_frame_id = "B";
  tf_buffer->setTransform(t, "test");

  int ret = RUN_ALL_TESTS();
  delete tf_buffer;
  return ret;
}





