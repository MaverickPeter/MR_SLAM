/*
 * Copyright (c) 2020, Open Source Robotics Foundation, Inc.
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

#include "tf2/transform_datatypes.h"

#include <string>


TEST(Stamped, assignment)
{
  tf2::Stamped<std::string> first("foobar", ros::Time(0), "my_frame_id");
  tf2::Stamped<std::string> second("baz", ros::Time(0), "my_frame_id");

  EXPECT_NE(second, first);
  second = first;
  EXPECT_EQ(second, first);
}

TEST(Stamped, setData)
{
  tf2::Stamped<std::string> first("foobar", ros::Time(0), "my_frame_id");
  tf2::Stamped<std::string> second("baz", ros::Time(0), "my_frame_id");

  EXPECT_NE(second, first);
  second.setData("foobar");
  EXPECT_EQ(second, first);
}

TEST(Stamped, copy_constructor)
{
  tf2::Stamped<std::string> first("foobar", ros::Time(0), "my_frame_id");
  tf2::Stamped<std::string> second(first);

  EXPECT_EQ(second, first);
}

TEST(Stamped, default_constructor)
{
  tf2::Stamped<std::string> first("foobar", ros::Time(0), "my_frame_id");
  tf2::Stamped<std::string> second;

  EXPECT_NE(second, first);
}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  ros::Time::init();
  return RUN_ALL_TESTS();
}
