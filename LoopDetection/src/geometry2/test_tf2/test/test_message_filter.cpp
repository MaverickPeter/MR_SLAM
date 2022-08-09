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

/** \author Josh Faust */


#include <tf2_ros/message_filter.h>
#include <tf2/buffer_core.h>
#include <geometry_msgs/PointStamped.h>
#include <boost/bind.hpp>
#include <boost/scoped_ptr.hpp>

#include "ros/ros.h"
#include "ros/callback_queue.h"

#include <gtest/gtest.h>

using namespace tf2;
using namespace tf2_ros;

class Notification
{
public:
  Notification(int expected_count) :
    count_(0), expected_count_(expected_count), failure_count_(0)
  {
  }

  void notify(const geometry_msgs::PointStamped::ConstPtr& message)
  {
    ++count_;
  }

  void failure(const geometry_msgs::PointStamped::ConstPtr& message, FilterFailureReason reason)
  {
    ++failure_count_;
  }

  int count_;
  int expected_count_;
  int failure_count_;
};

TEST(MessageFilter, noTransforms)
{
  BufferCore bc;
  Notification n(1);
  MessageFilter<geometry_msgs::PointStamped> filter(bc, "frame1", 1, 0);
  filter.registerCallback(boost::bind(&Notification::notify, &n, _1));

  geometry_msgs::PointStampedPtr msg(new geometry_msgs::PointStamped);
  msg->header.stamp = ros::Time(1);
  msg->header.frame_id = "frame2";
  filter.add(msg);

  EXPECT_EQ(0, n.count_);
}

TEST(MessageFilter, noTransformsSameFrame)
{
  BufferCore bc;
  Notification n(1);
  MessageFilter<geometry_msgs::PointStamped> filter(bc, "frame1", 1, 0);
  filter.registerCallback(boost::bind(&Notification::notify, &n, _1));

  geometry_msgs::PointStampedPtr msg(new geometry_msgs::PointStamped);
  msg->header.stamp = ros::Time(1);
  msg->header.frame_id = "frame1";
  filter.add(msg);

  EXPECT_EQ(1, n.count_);
}

geometry_msgs::TransformStamped createTransform(Quaternion q, Vector3 v, ros::Time stamp, const std::string& frame1, const std::string& frame2)
{
  geometry_msgs::TransformStamped t;
  t.header.frame_id = frame1;
  t.child_frame_id = frame2;
  t.header.stamp = stamp;
  t.transform.translation.x = v.x();
  t.transform.translation.y = v.y();
  t.transform.translation.z = v.z();
  t.transform.rotation.x = q.x();
  t.transform.rotation.y = q.y();
  t.transform.rotation.z = q.z();
  t.transform.rotation.w = q.w();
  return t;
}

TEST(MessageFilter, preexistingTransforms)
{
  BufferCore bc;
  Notification n(1);
  MessageFilter<geometry_msgs::PointStamped> filter(bc, "frame1", 1, 0);
  filter.registerCallback(boost::bind(&Notification::notify, &n, _1));

  ros::Time stamp(1);
  bc.setTransform(createTransform(Quaternion(0,0,0,1), Vector3(1,2,3), stamp, "frame1", "frame2"), "me");

  geometry_msgs::PointStampedPtr msg(new geometry_msgs::PointStamped);
  msg->header.stamp = stamp;
  msg->header.frame_id = "frame2";

  filter.add(msg);

  EXPECT_EQ(1, n.count_);
}

TEST(MessageFilter, postTransforms)
{
  BufferCore bc;
  Notification n(1);
  MessageFilter<geometry_msgs::PointStamped> filter(bc, "frame1", 1, 0);
  filter.registerCallback(boost::bind(&Notification::notify, &n, _1));

  ros::Time stamp(1);

  geometry_msgs::PointStampedPtr msg(new geometry_msgs::PointStamped);
  msg->header.stamp = stamp;
  msg->header.frame_id = "frame2";

  filter.add(msg);

  EXPECT_EQ(0, n.count_);

  bc.setTransform(createTransform(Quaternion(0,0,0,1), Vector3(1,2,3), stamp, "frame1", "frame2"), "me");

  EXPECT_EQ(1, n.count_);
}

TEST(MessageFilter, queueSize)
{
  BufferCore bc;
  Notification n(10);
  MessageFilter<geometry_msgs::PointStamped> filter(bc, "frame1", 10, 0);
  filter.registerCallback(boost::bind(&Notification::notify, &n, _1));
  filter.registerFailureCallback(boost::bind(&Notification::failure, &n, _1, _2));

  ros::Time stamp(1);

  for (int i = 0; i < 20; ++i)
  {
    geometry_msgs::PointStampedPtr msg(new geometry_msgs::PointStamped);
    msg->header.stamp = stamp;
    msg->header.frame_id = "frame2";

    filter.add(msg);
  }

  EXPECT_EQ(0, n.count_);
  EXPECT_EQ(10, n.failure_count_);

  bc.setTransform(createTransform(Quaternion(0,0,0,1), Vector3(1,2,3), stamp, "frame1", "frame2"), "me");

  EXPECT_EQ(10, n.count_);
}

TEST(MessageFilter, setTargetFrame)
{
  BufferCore bc;
  Notification n(1);
  MessageFilter<geometry_msgs::PointStamped> filter(bc, "frame1", 1, 0);
  filter.registerCallback(boost::bind(&Notification::notify, &n, _1));
  filter.setTargetFrame("frame1000");

  ros::Time stamp(1);
  bc.setTransform(createTransform(Quaternion(0,0,0,1), Vector3(1,2,3), stamp, "frame1000", "frame2"), "me");

  geometry_msgs::PointStampedPtr msg(new geometry_msgs::PointStamped);
  msg->header.stamp = stamp;
  msg->header.frame_id = "frame2";

  filter.add(msg);

  EXPECT_EQ(1, n.count_);
}


TEST(MessageFilter, multipleTargetFrames)
{
  BufferCore bc;
  Notification n(1);
  MessageFilter<geometry_msgs::PointStamped> filter(bc, "", 1, 0);
  filter.registerCallback(boost::bind(&Notification::notify, &n, _1));

  std::vector<std::string> target_frames;
  target_frames.push_back("frame1");
  target_frames.push_back("frame2");
  filter.setTargetFrames(target_frames);

  ros::Time stamp(1);
  bc.setTransform(createTransform(Quaternion(0,0,0,1), Vector3(1,2,3), stamp, "frame1", "frame3"), "me");

  geometry_msgs::PointStampedPtr msg(new geometry_msgs::PointStamped);
  msg->header.stamp = stamp;
  msg->header.frame_id = "frame3";
  filter.add(msg);

  EXPECT_EQ(0, n.count_); // frame1->frame3 exists, frame2->frame3 does not (yet)

  //ros::Time::setNow(ros::Time::now() + ros::Duration(1.0));

  bc.setTransform(createTransform(Quaternion(0,0,0,1), Vector3(1,2,3), stamp, "frame1", "frame2"), "me");

  EXPECT_EQ(1, n.count_); // frame2->frame3 now exists
}

TEST(MessageFilter, tolerance)
{
  ros::Duration offset(0.2);
  BufferCore bc;
  Notification n(1);
  MessageFilter<geometry_msgs::PointStamped> filter(bc, "frame1", 1, 0);
  filter.registerCallback(boost::bind(&Notification::notify, &n, _1));
  filter.setTolerance(offset);

  ros::Time stamp(1);
  bc.setTransform(createTransform(Quaternion(0,0,0,1), Vector3(1,2,3), stamp, "frame1", "frame2"), "me");

  geometry_msgs::PointStampedPtr msg(new geometry_msgs::PointStamped);
  msg->header.stamp = stamp;
  msg->header.frame_id = "frame2";
  filter.add(msg);

  EXPECT_EQ(0, n.count_); //No return due to lack of space for offset

  bc.setTransform(createTransform(Quaternion(0,0,0,1), Vector3(1,2,3), stamp + (offset * 1.1), "frame1", "frame2"), "me");

  EXPECT_EQ(1, n.count_); // Now have data for the message published earlier

  msg->header.stamp = stamp + offset;
  filter.add(msg);

  EXPECT_EQ(1, n.count_); // Latest message is off the end of the offset
}

TEST(MessageFilter, outTheBackFailure)
{
  BufferCore bc;
  Notification n(1);
  MessageFilter<geometry_msgs::PointStamped> filter(bc, "frame1", 1, 0);
  filter.registerFailureCallback(boost::bind(&Notification::failure, &n, _1, _2));

  ros::Time stamp(1);
  bc.setTransform(createTransform(Quaternion(0,0,0,1), Vector3(1,2,3), stamp, "frame1", "frame2"), "me");
  bc.setTransform(createTransform(Quaternion(0,0,0,1), Vector3(1,2,3), stamp + ros::Duration(10000), "frame1", "frame2"), "me");

  geometry_msgs::PointStampedPtr msg(new geometry_msgs::PointStamped);
  msg->header.stamp = stamp;
  msg->header.frame_id = "frame2";
  filter.add(msg);

  EXPECT_EQ(1, n.failure_count_);
}

TEST(MessageFilter, outTheBackFailure2)
{
  BufferCore bc;
  Notification n(1);
  MessageFilter<geometry_msgs::PointStamped> filter(bc, "frame1", 1, 0);
  filter.registerFailureCallback(boost::bind(&Notification::failure, &n, _1, _2));

  ros::Time stamp(1);

  geometry_msgs::PointStampedPtr msg(new geometry_msgs::PointStamped);
  msg->header.stamp = stamp;
  msg->header.frame_id = "frame2";
  filter.add(msg);

  EXPECT_EQ(0, n.count_);
  EXPECT_EQ(0, n.failure_count_);

  bc.setTransform(createTransform(Quaternion(0,0,0,1), Vector3(1,2,3), stamp + ros::Duration(10000), "frame1", "frame2"), "me");

  EXPECT_EQ(1, n.failure_count_);
}

TEST(MessageFilter, emptyFrameIDFailure)
{
  BufferCore bc;
  Notification n(1);
  MessageFilter<geometry_msgs::PointStamped> filter(bc, "frame1", 1, 0);
  filter.registerFailureCallback(boost::bind(&Notification::failure, &n, _1, _2));

  geometry_msgs::PointStampedPtr msg(new geometry_msgs::PointStamped);
  msg->header.frame_id = "";
  filter.add(msg);

  EXPECT_EQ(1, n.failure_count_);
}

TEST(MessageFilter, callbackQueue)
{
  BufferCore bc;
  Notification n(1);
  ros::CallbackQueue queue;
  MessageFilter<geometry_msgs::PointStamped> filter(bc, "frame1", 1, &queue);
  filter.registerCallback(boost::bind(&Notification::notify, &n, _1));

  geometry_msgs::PointStampedPtr msg(new geometry_msgs::PointStamped);
  msg->header.stamp = ros::Time(1);
  msg->header.frame_id = "frame1";
  filter.add(msg);

  EXPECT_EQ(0, n.count_);

  queue.callAvailable();

  EXPECT_EQ(1, n.count_);
}


int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);

  int ret = RUN_ALL_TESTS();

  return ret;
}
