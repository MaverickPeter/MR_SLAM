/*********************************************************************
*
* Software License Agreement (BSD License)
*
*  Copyright (c) 2009, Willow Garage, Inc.
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of Willow Garage, Inc. nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*
* Author: Eitan Marder-Eppstein
*********************************************************************/
#include <gtest/gtest.h>
#include <tf2/convert.h>
#include <tf2_kdl/tf2_kdl.h>
#include <tf2_bullet/tf2_bullet.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_eigen/tf2_eigen.h>
#include <ros/time.h>

TEST(tf2Convert, kdlToBullet)
{
  double epsilon = 1e-9;

  tf2::Stamped<btVector3> b(btVector3(1,2,3), ros::Time(), "my_frame");

  tf2::Stamped<btVector3> b1 = b;
  tf2::Stamped<KDL::Vector> k1;
  tf2::convert(b1, k1);

  tf2::Stamped<btVector3> b2;
  tf2::convert(k1, b2);

  EXPECT_EQ(b.frame_id_, b2.frame_id_);
  EXPECT_NEAR(b.stamp_.toSec(), b2.stamp_.toSec(), epsilon);
  EXPECT_NEAR(b.x(), b2.x(), epsilon);
  EXPECT_NEAR(b.y(), b2.y(), epsilon);
  EXPECT_NEAR(b.z(), b2.z(), epsilon);


  EXPECT_EQ(b1.frame_id_, b2.frame_id_);
  EXPECT_NEAR(b1.stamp_.toSec(), b2.stamp_.toSec(), epsilon);
  EXPECT_NEAR(b1.x(), b2.x(), epsilon);
  EXPECT_NEAR(b1.y(), b2.y(), epsilon);
  EXPECT_NEAR(b1.z(), b2.z(), epsilon);
}

TEST(tf2Convert, kdlBulletROSConversions)
{
  double epsilon = 1e-9;

  tf2::Stamped<btVector3> b1(btVector3(1,2,3), ros::Time(), "my_frame"), b2, b3, b4;
  geometry_msgs::PointStamped r1, r2, r3;
  tf2::Stamped<KDL::Vector> k1, k2, k3;

  // Do bullet -> self -> bullet -> KDL -> self -> KDL -> ROS -> self -> ROS -> KDL -> bullet -> ROS -> bullet
  tf2::convert(b1, b1);
  tf2::convert(b1, b2);
  tf2::convert(b2, k1);
  tf2::convert(k1, k1);
  tf2::convert(k1, k2);
  tf2::convert(k2, r1);
  tf2::convert(r1, r1);
  tf2::convert(r1, r2);
  tf2::convert(r2, k3);
  tf2::convert(k3, b3);
  tf2::convert(b3, r3);
  tf2::convert(r3, b4);

  EXPECT_EQ(b1.frame_id_, b4.frame_id_);
  EXPECT_NEAR(b1.stamp_.toSec(), b4.stamp_.toSec(), epsilon);
  EXPECT_NEAR(b1.x(), b4.x(), epsilon);
  EXPECT_NEAR(b1.y(), b4.y(), epsilon);
  EXPECT_NEAR(b1.z(), b4.z(), epsilon);
} 

TEST(tf2Convert, ConvertTf2Quaternion)
{
  tf2::Quaternion tq(1,2,3,4);
  Eigen::Quaterniond eq;
  tf2::convert(tq, eq);

  EXPECT_EQ(tq.w(), eq.w());
  EXPECT_EQ(tq.x(), eq.x());
  EXPECT_EQ(tq.y(), eq.y());
  EXPECT_EQ(tq.z(), eq.z());
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

