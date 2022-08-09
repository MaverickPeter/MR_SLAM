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


#include <tf2_bullet/tf2_bullet.h>
#include <tf2_ros/buffer.h>
#include <ros/ros.h>
#include <gtest/gtest.h>
#include <tf2/convert.h>

tf2_ros::Buffer* tf_buffer;
static const double EPS = 1e-3;

TEST(TfBullet, Transform)
{
  tf2::Stamped<btTransform> v1(btTransform(btQuaternion(1,0,0,0), btVector3(1,2,3)), ros::Time(2.0), "A");

  // simple api
  btTransform v_simple = tf_buffer->transform(v1, "B", ros::Duration(2.0));
  EXPECT_NEAR(v_simple.getOrigin().getX(), -9, EPS);
  EXPECT_NEAR(v_simple.getOrigin().getY(), 18, EPS);
  EXPECT_NEAR(v_simple.getOrigin().getZ(), 27, EPS);

  // advanced api
  btTransform v_advanced = tf_buffer->transform(v1, "B", ros::Time(2.0),
					       "B", ros::Duration(3.0));
  EXPECT_NEAR(v_advanced.getOrigin().getX(), -9, EPS);
  EXPECT_NEAR(v_advanced.getOrigin().getY(), 18, EPS);
  EXPECT_NEAR(v_advanced.getOrigin().getZ(), 27, EPS);
}



TEST(TfBullet, Vector)
{
  tf2::Stamped<btVector3>  v1(btVector3(1,2,3), ros::Time(2.0), "A");

  // simple api
  btVector3 v_simple = tf_buffer->transform(v1, "B", ros::Duration(2.0));
  EXPECT_NEAR(v_simple.getX(), -9, EPS);
  EXPECT_NEAR(v_simple.getY(), 18, EPS);
  EXPECT_NEAR(v_simple.getZ(), 27, EPS);

  // advanced api
  btVector3 v_advanced = tf_buffer->transform(v1, "B", ros::Time(2.0),
  					     "B", ros::Duration(3.0));
  EXPECT_NEAR(v_advanced.getX(), -9, EPS);
  EXPECT_NEAR(v_advanced.getY(), 18, EPS);
  EXPECT_NEAR(v_advanced.getZ(), 27, EPS);
}




int main(int argc, char **argv){
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "test");
  ros::NodeHandle n;

  tf_buffer = new tf2_ros::Buffer();

  // populate buffer
  geometry_msgs::TransformStamped t;
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
