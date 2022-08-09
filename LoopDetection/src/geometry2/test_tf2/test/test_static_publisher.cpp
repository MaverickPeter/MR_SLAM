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

#include <gtest/gtest.h>
#include <tf2/buffer_core.h>
#include "tf2/exceptions.h"
#include <tf2_ros/static_transform_broadcaster.h>
#include <ros/ros.h>
#include "rostest/permuter.h"

#include "tf2_ros/transform_listener.h"

TEST(StaticTransformPublisher, a_b_different_times)
{
  tf2_ros::Buffer mB;
  tf2_ros::TransformListener tfl(mB);
  EXPECT_TRUE(mB.canTransform("a", "b", ros::Time(), ros::Duration(1.0)));
  EXPECT_TRUE(mB.canTransform("a", "b", ros::Time(100), ros::Duration(1.0)));
  EXPECT_TRUE(mB.canTransform("a", "b", ros::Time(1000), ros::Duration(1.0)));
};

TEST(StaticTransformPublisher, a_c_different_times)
{
  tf2_ros::Buffer mB;
  tf2_ros::TransformListener tfl(mB);
  EXPECT_TRUE(mB.canTransform("a", "c", ros::Time(), ros::Duration(1.0)));
  EXPECT_TRUE(mB.canTransform("a", "c", ros::Time(100), ros::Duration(1.0)));
  EXPECT_TRUE(mB.canTransform("a", "c", ros::Time(1000), ros::Duration(1.0)));
};

TEST(StaticTransformPublisher, a_d_different_times)
{
  tf2_ros::Buffer mB;
  tf2_ros::TransformListener tfl(mB);
  geometry_msgs::TransformStamped ts;
  ts.transform.rotation.w = 1;
  ts.header.frame_id = "c";
  ts.header.stamp = ros::Time(10.0);
  ts.child_frame_id = "d";

  // make sure listener has populated
  EXPECT_TRUE(mB.canTransform("a", "c", ros::Time(), ros::Duration(1.0)));
  EXPECT_TRUE(mB.canTransform("a", "c", ros::Time(100), ros::Duration(1.0)));
  EXPECT_TRUE(mB.canTransform("a", "c", ros::Time(1000), ros::Duration(1.0)));

  mB.setTransform(ts, "authority");
  //printf("%s\n", mB.allFramesAsString().c_str());
  EXPECT_TRUE(mB.canTransform("c", "d", ros::Time(10), ros::Duration(0)));

  EXPECT_TRUE(mB.canTransform("a", "d", ros::Time(), ros::Duration(0)));
  EXPECT_FALSE(mB.canTransform("a", "d", ros::Time(1), ros::Duration(0)));
  EXPECT_TRUE(mB.canTransform("a", "d", ros::Time(10), ros::Duration(0)));
  EXPECT_FALSE(mB.canTransform("a", "d", ros::Time(100), ros::Duration(0)));
};

TEST(StaticTransformPublisher, multiple_parent_test)
{
  tf2_ros::Buffer mB;
  tf2_ros::TransformListener tfl(mB);
  tf2_ros::StaticTransformBroadcaster stb;
  geometry_msgs::TransformStamped ts;
  ts.transform.rotation.w = 1;
  ts.header.frame_id = "c";
  ts.header.stamp = ros::Time(10.0);
  ts.child_frame_id = "d";

  stb.sendTransform(ts);

  // make sure listener has populated
  EXPECT_TRUE(mB.canTransform("a", "d", ros::Time(), ros::Duration(1.0)));
  EXPECT_TRUE(mB.canTransform("a", "d", ros::Time(100), ros::Duration(1.0)));
  EXPECT_TRUE(mB.canTransform("a", "d", ros::Time(1000), ros::Duration(1.0)));

  // Publish new transform with child 'd', should replace old one in static tf
  ts.header.frame_id = "new_parent";
  stb.sendTransform(ts);
  ts.child_frame_id = "other_child";
  stb.sendTransform(ts);
  ts.child_frame_id = "other_child2";
  stb.sendTransform(ts);

  EXPECT_TRUE(mB.canTransform("new_parent", "d", ros::Time(), ros::Duration(1.0)));
  EXPECT_TRUE(mB.canTransform("new_parent", "other_child", ros::Time(), ros::Duration(1.0)));
  EXPECT_TRUE(mB.canTransform("new_parent", "other_child2", ros::Time(), ros::Duration(1.0)));
  EXPECT_FALSE(mB.canTransform("a", "d", ros::Time(), ros::Duration(1.0)));
};

TEST(StaticTransformPublisher, tf_from_param_server_valid)
{
  // This TF is loaded from the parameter server; ensure it is valid.
  tf2_ros::Buffer mB;
  tf2_ros::TransformListener tfl(mB);
  EXPECT_TRUE(mB.canTransform("robot_calibration", "world", ros::Time(), ros::Duration(1.0)));
  EXPECT_TRUE(mB.canTransform("robot_calibration", "world", ros::Time(100), ros::Duration(1.0)));
  EXPECT_TRUE(mB.canTransform("robot_calibration", "world", ros::Time(1000), ros::Duration(1.0)));
}

int main(int argc, char **argv){
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "tf_unittest");
  return RUN_ALL_TESTS();
}
