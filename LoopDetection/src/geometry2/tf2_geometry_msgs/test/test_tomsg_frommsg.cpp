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
#include <gtest/gtest.h>

static const double EPS = 1e-6;

tf2::Vector3 get_tf2_vector()
{
  return tf2::Vector3(1.0, 2.0, 3.0);
}

geometry_msgs::Vector3& value_initialize(geometry_msgs::Vector3 &m1)
{
  m1.x = 1;
  m1.y = 2;
  m1.z = 3;
  return m1;
}

std_msgs::Header& value_initialize(std_msgs::Header & h)
{
  h.stamp = ros::Time(10);
  h.frame_id = "foobar";
  return h;
}

geometry_msgs::Vector3Stamped& value_initialize(geometry_msgs::Vector3Stamped &m1)
{
  value_initialize(m1.header);
  value_initialize(m1.vector);
  return m1;
}

geometry_msgs::Point& value_initialize(geometry_msgs::Point &m1)
{
  m1.x = 1;
  m1.y = 2;
  m1.z = 3;
  return m1;
}

geometry_msgs::PointStamped& value_initialize(geometry_msgs::PointStamped &m1)
{
  value_initialize(m1.header);
  value_initialize(m1.point);
  return m1;
}

geometry_msgs::Quaternion & value_initialize(geometry_msgs::Quaternion &m1)
{
  m1.x = 0;
  m1.y = 0;
  m1.z = 0.7071067811;
  m1.w = 0.7071067811;
  return m1;
}

geometry_msgs::QuaternionStamped& value_initialize(geometry_msgs::QuaternionStamped &m1)
{
  value_initialize(m1.header);
  value_initialize(m1.quaternion);
  return m1;
}

geometry_msgs::Pose & value_initialize(geometry_msgs::Pose & m1)
{
  value_initialize(m1.position);
  value_initialize(m1.orientation);
  return m1;
}

geometry_msgs::PoseStamped& value_initialize(geometry_msgs::PoseStamped &m1)
{
  value_initialize(m1.header);
  value_initialize(m1.pose);
  return m1;
}

geometry_msgs::Transform & value_initialize(geometry_msgs::Transform & m1)
{
  value_initialize(m1.translation);
  value_initialize(m1.rotation);
  return m1;
}

geometry_msgs::TransformStamped& value_initialize(geometry_msgs::TransformStamped &m1)
{
  value_initialize(m1.header);
  value_initialize(m1.transform);
  return m1;
}

void expect_near(const std_msgs::Header & h1, const std_msgs::Header & h2)
{
  EXPECT_NEAR(h1.stamp.toSec(), h2.stamp.toSec(), EPS);
  EXPECT_STREQ(h1.frame_id.c_str(), h2.frame_id.c_str());
}

/*
 * Vector3
 */
void expect_near(const geometry_msgs::Vector3 & v1, const tf2::Vector3 & v2)
{
  EXPECT_NEAR(v1.x, v2.x(), EPS);
  EXPECT_NEAR(v1.y, v2.y(), EPS);
  EXPECT_NEAR(v1.z, v2.z(), EPS);
}

void expect_near(const geometry_msgs::Vector3 & v1, const geometry_msgs::Vector3 & v2)
{
  EXPECT_NEAR(v1.x, v2.x, EPS);
  EXPECT_NEAR(v1.y, v2.y, EPS);
  EXPECT_NEAR(v1.z, v2.z, EPS);
}

void expect_near(const tf2::Vector3 & v1, const tf2::Vector3 & v2)
{
  EXPECT_NEAR(v1.x(), v2.x(), EPS);
  EXPECT_NEAR(v1.y(), v2.y(), EPS);
  EXPECT_NEAR(v1.z(), v2.z(), EPS);
}

void expect_near(const geometry_msgs::Vector3Stamped & p1, const geometry_msgs::Vector3Stamped & p2)
{
  expect_near(p1.header, p2.header);
  expect_near(p1.vector, p2.vector);
}

/*
 * Point
 */
void expect_near(const geometry_msgs::Point & p1, const tf2::Vector3 & v2)
{
  EXPECT_NEAR(p1.x, v2.x(), EPS);
  EXPECT_NEAR(p1.y, v2.y(), EPS);
  EXPECT_NEAR(p1.z, v2.z(), EPS);
}

void expect_near(const geometry_msgs::Point & p1, const geometry_msgs::Point & v2)
{
  EXPECT_NEAR(p1.x, v2.x, EPS);
  EXPECT_NEAR(p1.y, v2.y, EPS);
  EXPECT_NEAR(p1.z, v2.z, EPS);
}

void expect_near(const geometry_msgs::PointStamped & p1, const geometry_msgs::PointStamped & p2)
{
  expect_near(p1.header, p2.header);
  expect_near(p1.point, p2.point);
}


/*
 * Quaternion
 */
void expect_near(const geometry_msgs::Quaternion & q1, const tf2::Quaternion & v2)
{
  EXPECT_NEAR(q1.x, v2.x(), EPS);
  EXPECT_NEAR(q1.y, v2.y(), EPS);
  EXPECT_NEAR(q1.z, v2.z(), EPS);
}

void expect_near(const geometry_msgs::Quaternion & q1, const geometry_msgs::Quaternion & v2)
{
  EXPECT_NEAR(q1.x, v2.x, EPS);
  EXPECT_NEAR(q1.y, v2.y, EPS);
  EXPECT_NEAR(q1.z, v2.z, EPS);
}

void expect_near(const geometry_msgs::QuaternionStamped & p1, const geometry_msgs::QuaternionStamped & p2)
{
  expect_near(p1.header, p2.header);
  expect_near(p1.quaternion, p2.quaternion);
}

/*
 * Pose
 */
void expect_near(const geometry_msgs::Pose & p, const tf2::Transform & t)
{
  expect_near(p.position, t.getOrigin());
  expect_near(p.orientation, t.getRotation());
}

void expect_near(const geometry_msgs::Pose & p1, const geometry_msgs::Pose & p2)
{
  expect_near(p1.position, p2.position);
  expect_near(p1.orientation, p2.orientation);
}

void expect_near(const geometry_msgs::PoseStamped & p1, const geometry_msgs::PoseStamped & p2)
{
  expect_near(p1.header, p2.header);
  expect_near(p1.pose, p2.pose);
}

/*
 * Transform
 */
void expect_near(const geometry_msgs::Transform & p, const tf2::Transform & t)
{
  expect_near(p.translation, t.getOrigin());
  expect_near(p.rotation, t.getRotation());
}

void expect_near(const geometry_msgs::Transform & p1, const geometry_msgs::Transform & p2)
{
  expect_near(p1.translation, p2.translation);
  expect_near(p1.rotation, p2.rotation);
}

void expect_near(const geometry_msgs::TransformStamped & p1, const geometry_msgs::TransformStamped & p2)
{
  expect_near(p1.header, p2.header);
  expect_near(p1.transform, p2.transform);
}

/*
 * Stamped templated expect_near
 */

template <typename T>
void expect_near(const tf2::Stamped<T>& s1, const tf2::Stamped<T>& s2)
{
 expect_near((T)s1, (T)s2);
}

/*********************
 * Tests
 *********************/
 
TEST(tf2_geometry_msgs, Vector3)
{
  geometry_msgs::Vector3 m1;
  value_initialize(m1);
  tf2::Vector3 v1;
  fromMsg(m1, v1);
  SCOPED_TRACE("m1 v1");
  expect_near(m1, v1);
  geometry_msgs::Vector3 m2 = toMsg(v1);
  SCOPED_TRACE("m1 m2");
  expect_near(m1, m2);
}

TEST(tf2_geometry_msgs, Point)
{
  geometry_msgs::Point m1;
  value_initialize(m1);
  tf2::Vector3 v1;
  SCOPED_TRACE("m1 v1");
  fromMsg(m1, v1);
  expect_near(m1, v1);
  geometry_msgs::Point m2 = toMsg(v1, m2);
  SCOPED_TRACE("m1 m2");
  expect_near(m1, m2);
}

TEST(tf2_geometry_msgs, Quaternion)
{
  geometry_msgs::Quaternion m1;
  value_initialize(m1);
  tf2::Quaternion q1;
  SCOPED_TRACE("m1 q1");
  fromMsg(m1, q1);
  expect_near(m1, q1);
  geometry_msgs::Quaternion m2 = toMsg(q1);
  SCOPED_TRACE("m1 m2");
  expect_near(m1, m2);
}

TEST(tf2_geometry_msgs, Pose)
{
  geometry_msgs::Pose m1;
  value_initialize(m1);
  tf2::Transform t1;
  fromMsg(m1, t1);
  SCOPED_TRACE("m1 t1");
  expect_near(m1, t1);
  geometry_msgs::Pose m2 = toMsg(t1, m2);
  SCOPED_TRACE("m1 m2");
  expect_near(m1, m2);
}

TEST(tf2_geometry_msgs, Transform)
{
  geometry_msgs::Transform m1;
  value_initialize(m1);
  tf2::Transform t1;
  fromMsg(m1, t1);
  SCOPED_TRACE("m1 t1");
  expect_near(m1, t1);
  geometry_msgs::Transform m2 = toMsg(t1);
  SCOPED_TRACE("m1 m2");
  expect_near(m1, m2);
}

TEST(tf2_geometry_msgs, Vector3Stamped)
{
  geometry_msgs::Vector3Stamped m1;
  value_initialize(m1);
  tf2::Stamped<tf2::Vector3> v1;
  fromMsg(m1, v1);
  SCOPED_TRACE("m1 v1");
  // expect_near(m1, v1);
  geometry_msgs::Vector3Stamped m2;
  m2 = toMsg(v1);
  SCOPED_TRACE("m1 m2");
  expect_near(m1, m2);
}

TEST(tf2_geometry_msgs, PointStamped)
{
  geometry_msgs::PointStamped m1;
  value_initialize(m1);
  tf2::Stamped<tf2::Vector3> v1;
  fromMsg(m1, v1);
  SCOPED_TRACE("m1 v1");
  // expect_near(m1, v1); //TODO implement cross verification explicityly
  geometry_msgs::PointStamped m2;
  m2 = toMsg(v1, m2);
  SCOPED_TRACE("m1 m2");
  expect_near(m1, m2);
}

TEST(tf2_geometry_msgs, QuaternionStamped)
{
  geometry_msgs::QuaternionStamped m1;
  value_initialize(m1);
  tf2::Stamped<tf2::Quaternion> v1;
  fromMsg(m1, v1);
  SCOPED_TRACE("m1 v1");
  // expect_near(m1, v1); //TODO implement cross verification explicityly
  geometry_msgs::QuaternionStamped m2;
  m2 = tf2::toMsg(v1);
  SCOPED_TRACE("m1 m2");
  expect_near(m1, m2);
}

TEST(tf2_geometry_msgs, PoseStamped)
{
  geometry_msgs::PoseStamped m1;
  value_initialize(m1);
  tf2::Stamped<tf2::Transform> v1;
  SCOPED_TRACE("m1 v1");
  fromMsg(m1, v1);
  // expect_near(m1, v1); //TODO implement cross verification explicityly
  geometry_msgs::PoseStamped m2;
  m2 = tf2::toMsg(v1, m2);
  SCOPED_TRACE("m1 m2");
  expect_near(m1, m2);
}

TEST(tf2_geometry_msgs, TransformStamped)
{
  geometry_msgs::TransformStamped m1;
  value_initialize(m1);
  tf2::Stamped<tf2::Transform> v1;
  fromMsg(m1, v1);
  SCOPED_TRACE("m1 v1");
  // expect_near(m1, v1);
  geometry_msgs::TransformStamped m2;
  m2 = tf2::toMsg(v1);
  SCOPED_TRACE("m1 m2");
  expect_near(m1, m2);
}




int main(int argc, char **argv){
  testing::InitGoogleTest(&argc, argv);
  int ret = RUN_ALL_TESTS();
  return ret;
}
