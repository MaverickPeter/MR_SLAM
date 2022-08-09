/*
 * Copyright (c) 2014, Open Source Robotics Foundation
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

#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <message_filters/subscriber.h>
#include <ros/ros.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/message_filter.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

#include <gtest/gtest.h>
#include <thread>
#include <chrono>


void spin_for_a_second()
{
  ros::spinOnce();
  for (uint8_t i = 0; i < 10; ++i)
  {
    std::this_thread::sleep_for(std::chrono::microseconds(100));
    ros::spinOnce();
  }
}

bool filter_callback_fired = false;
void filter_callback(const geometry_msgs::PointStamped& msg)
{
  filter_callback_fired = true;
}

TEST(tf2_ros_message_filter, multiple_frames_and_time_tolerance)
{
  ros::NodeHandle nh;
  message_filters::Subscriber<geometry_msgs::PointStamped> sub;
  sub.subscribe(nh, "point", 10);

  tf2_ros::Buffer buffer;
  tf2_ros::TransformListener tfl(buffer);
  tf2_ros::MessageFilter<geometry_msgs::PointStamped> filter(buffer, "map", 10, nh);
  filter.connectInput(sub);
  filter.registerCallback(&filter_callback);
  // Register multiple target frames
  std::vector<std::string> frames;
  frames.push_back("odom");
  frames.push_back("map");
  filter.setTargetFrames(frames);
  // Set a non-zero time tolerance
  filter.setTolerance(ros::Duration(1, 0));

  // Publish static transforms so the frame transformations will always be valid
  tf2_ros::StaticTransformBroadcaster tfb;
  geometry_msgs::TransformStamped map_to_odom;
  map_to_odom.header.stamp = ros::Time(0, 0);
  map_to_odom.header.frame_id = "map";
  map_to_odom.child_frame_id = "odom";
  map_to_odom.transform.translation.x = 0.0;
  map_to_odom.transform.translation.y = 0.0;
  map_to_odom.transform.translation.z = 0.0;
  map_to_odom.transform.rotation.x = 0.0;
  map_to_odom.transform.rotation.y = 0.0;
  map_to_odom.transform.rotation.z = 0.0;
  map_to_odom.transform.rotation.w = 1.0;
  tfb.sendTransform(map_to_odom);

  geometry_msgs::TransformStamped odom_to_base;
  odom_to_base.header.stamp = ros::Time(0, 0);
  odom_to_base.header.frame_id = "odom";
  odom_to_base.child_frame_id = "base";
  odom_to_base.transform.translation.x = 0.0;
  odom_to_base.transform.translation.y = 0.0;
  odom_to_base.transform.translation.z = 0.0;
  odom_to_base.transform.rotation.x = 0.0;
  odom_to_base.transform.rotation.y = 0.0;
  odom_to_base.transform.rotation.z = 0.0;
  odom_to_base.transform.rotation.w = 1.0;
  tfb.sendTransform(odom_to_base);

  // Publish a Point message in the "base" frame
  ros::Publisher pub = nh.advertise<geometry_msgs::PointStamped>("point", 10);
  geometry_msgs::PointStamped point;
  point.header.stamp = ros::Time::now();
  point.header.frame_id = "base";
  pub.publish(point);

  // make sure it arrives
  spin_for_a_second();

  // The filter callback should have been fired because all required transforms are available
  ASSERT_TRUE(filter_callback_fired);
}

int main(int argc, char **argv){
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "tf2_ros_message_filter");
  return RUN_ALL_TESTS();
}
