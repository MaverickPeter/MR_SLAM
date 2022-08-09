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
#include <ros/time.h>
#include "tf2/LinearMath/Vector3.h"
#include "tf2/exceptions.h"

TEST(tf2, setTransformFail)
{
  tf2::BufferCore tfc;
  geometry_msgs::TransformStamped st;
  EXPECT_FALSE(tfc.setTransform(st, "authority1"));

}

TEST(tf2, setTransformValid)
{
  tf2::BufferCore tfc;
  geometry_msgs::TransformStamped st;
  st.header.frame_id = "foo";
  st.header.stamp = ros::Time(1.0);
  st.child_frame_id = "child";
  st.transform.rotation.w = 1;
  EXPECT_TRUE(tfc.setTransform(st, "authority1"));

}

TEST(tf2, setTransformInvalidQuaternion)
{
  tf2::BufferCore tfc;
  geometry_msgs::TransformStamped st;
  st.header.frame_id = "foo";
  st.header.stamp = ros::Time(1.0);
  st.child_frame_id = "child";
  st.transform.rotation.w = 0;
  EXPECT_FALSE(tfc.setTransform(st, "authority1"));

}

TEST(tf2_lookupTransform, LookupException_Nothing_Exists)
{
  tf2::BufferCore tfc;
  EXPECT_THROW(tfc.lookupTransform("a", "b", ros::Time().fromSec(1.0)), tf2::LookupException);

}

TEST(tf2_canTransform, Nothing_Exists)
{
  tf2::BufferCore tfc;
  EXPECT_FALSE(tfc.canTransform("a", "b", ros::Time().fromSec(1.0)));

  std::string error_msg = std::string();
  EXPECT_FALSE(tfc.canTransform("a", "b", ros::Time().fromSec(1.0), &error_msg));
  ASSERT_STREQ(error_msg.c_str(), "canTransform: target_frame a does not exist. canTransform: source_frame b does not exist.");

}

TEST(tf2_lookupTransform, LookupException_One_Exists)
{
  tf2::BufferCore tfc;
  geometry_msgs::TransformStamped st;
  st.header.frame_id = "foo";
  st.header.stamp = ros::Time(1.0);
  st.child_frame_id = "child";
  st.transform.rotation.w = 1;
  EXPECT_TRUE(tfc.setTransform(st, "authority1"));
  EXPECT_THROW(tfc.lookupTransform("foo", "bar", ros::Time().fromSec(1.0)), tf2::LookupException);

}

TEST(tf2_canTransform, One_Exists)
{
  tf2::BufferCore tfc;
  geometry_msgs::TransformStamped st;
  st.header.frame_id = "foo";
  st.header.stamp = ros::Time(1.0);
  st.child_frame_id = "child";
  st.transform.rotation.w = 1;
  EXPECT_TRUE(tfc.setTransform(st, "authority1"));
  EXPECT_FALSE(tfc.canTransform("foo", "bar", ros::Time().fromSec(1.0)));
}

TEST(tf2_chainAsVector, chain_v_configuration)
{
  tf2::BufferCore mBC;
  tf2::TestBufferCore tBC;

  geometry_msgs::TransformStamped st;
  st.header.stamp = ros::Time(0);
  st.transform.rotation.w = 1;

  st.header.frame_id = "a";
  st.child_frame_id = "b";
  mBC.setTransform(st, "authority1");

  st.header.frame_id = "b";
  st.child_frame_id = "c";
  mBC.setTransform(st, "authority1");

  st.header.frame_id = "a";
  st.child_frame_id = "d";
  mBC.setTransform(st, "authority1");

  st.header.frame_id = "d";
  st.child_frame_id = "e";
  mBC.setTransform(st, "authority1");

  std::vector<std::string> chain;
  
  
  mBC._chainAsVector( "c", ros::Time(), "c", ros::Time(), "c", chain);
  EXPECT_EQ( 0, chain.size());

  mBC._chainAsVector( "a", ros::Time(), "c", ros::Time(), "c", chain);
  EXPECT_EQ( 3, chain.size());
  if( chain.size() >= 1 ) EXPECT_EQ( chain[0], "c" );
  if( chain.size() >= 2 ) EXPECT_EQ( chain[1], "b" );
  if( chain.size() >= 3 ) EXPECT_EQ( chain[2], "a" );

  mBC._chainAsVector( "c", ros::Time(), "a", ros::Time(), "c", chain);
  EXPECT_EQ( 3, chain.size());
  if( chain.size() >= 1 ) EXPECT_EQ( chain[0], "a" );
  if( chain.size() >= 2 ) EXPECT_EQ( chain[1], "b" );
  if( chain.size() >= 3 ) EXPECT_EQ( chain[2], "c" );

  mBC._chainAsVector( "a", ros::Time(), "c", ros::Time(), "a", chain);
  EXPECT_EQ( 3, chain.size());
  if( chain.size() >= 1 ) EXPECT_EQ( chain[0], "c" );
  if( chain.size() >= 2 ) EXPECT_EQ( chain[1], "b" );
  if( chain.size() >= 3 ) EXPECT_EQ( chain[2], "a" );

  mBC._chainAsVector( "c", ros::Time(), "a", ros::Time(), "a", chain);
  EXPECT_EQ( 3, chain.size());
  if( chain.size() >= 1 ) EXPECT_EQ( chain[0], "a" );
  if( chain.size() >= 2 ) EXPECT_EQ( chain[1], "b" );
  if( chain.size() >= 3 ) EXPECT_EQ( chain[2], "c" );
  
  mBC._chainAsVector( "c", ros::Time(), "e", ros::Time(), "c", chain);

  EXPECT_EQ( 5, chain.size());
  if( chain.size() >= 1 ) EXPECT_EQ( chain[0], "e" );
  if( chain.size() >= 2 ) EXPECT_EQ( chain[1], "d" );
  if( chain.size() >= 3 ) EXPECT_EQ( chain[2], "a" );
  if( chain.size() >= 4 ) EXPECT_EQ( chain[3], "b" );
  if( chain.size() >= 5 ) EXPECT_EQ( chain[4], "c" );

  mBC._chainAsVector( "c", ros::Time(), "e", ros::Time(), "a", chain);

  EXPECT_EQ( 5, chain.size());
  if( chain.size() >= 1 ) EXPECT_EQ( chain[0], "e" );
  if( chain.size() >= 2 ) EXPECT_EQ( chain[1], "d" );
  if( chain.size() >= 3 ) EXPECT_EQ( chain[2], "a" );
  if( chain.size() >= 4 ) EXPECT_EQ( chain[3], "b" );
  if( chain.size() >= 5 ) EXPECT_EQ( chain[4], "c" );

  mBC._chainAsVector( "c", ros::Time(), "e", ros::Time(), "e", chain);

  EXPECT_EQ( 5, chain.size());
  if( chain.size() >= 1 ) EXPECT_EQ( chain[0], "e" );
  if( chain.size() >= 2 ) EXPECT_EQ( chain[1], "d" );
  if( chain.size() >= 3 ) EXPECT_EQ( chain[2], "a" );
  if( chain.size() >= 4 ) EXPECT_EQ( chain[3], "b" );
  if( chain.size() >= 5 ) EXPECT_EQ( chain[4], "c" );
}

TEST(tf2_walkToTopParent, walk_i_configuration)
{
  tf2::BufferCore mBC;
  tf2::TestBufferCore tBC;

  geometry_msgs::TransformStamped st;
  st.header.stamp = ros::Time(0);
  st.transform.rotation.w = 1;

  st.header.frame_id = "a";
  st.child_frame_id = "b";
  mBC.setTransform(st, "authority1");

  st.header.frame_id = "b";
  st.child_frame_id = "c";
  mBC.setTransform(st, "authority1");

  st.header.frame_id = "c";
  st.child_frame_id = "d";
  mBC.setTransform(st, "authority1");

  st.header.frame_id = "d";
  st.child_frame_id = "e";
  mBC.setTransform(st, "authority1");

  std::vector<tf2::CompactFrameID> id_chain;
  tBC._walkToTopParent(mBC, ros::Time(), mBC._lookupFrameNumber("a"), mBC._lookupFrameNumber("e"), 0, &id_chain);

  EXPECT_EQ(5, id_chain.size() );
  if( id_chain.size() >= 1 ) EXPECT_EQ("e", tBC._lookupFrameString(mBC, id_chain[0]));
  if( id_chain.size() >= 2 ) EXPECT_EQ("d", tBC._lookupFrameString(mBC, id_chain[1]));
  if( id_chain.size() >= 3 ) EXPECT_EQ("c", tBC._lookupFrameString(mBC, id_chain[2]));
  if( id_chain.size() >= 4 ) EXPECT_EQ("b", tBC._lookupFrameString(mBC, id_chain[3]));
  if( id_chain.size() >= 5 ) EXPECT_EQ("a", tBC._lookupFrameString(mBC, id_chain[4]));

  id_chain.clear();
  tBC._walkToTopParent(mBC,  ros::Time(), mBC._lookupFrameNumber("e"), mBC._lookupFrameNumber("a"), 0, &id_chain);

  EXPECT_EQ(5, id_chain.size() );
  if( id_chain.size() >= 1 ) EXPECT_EQ("a", tBC._lookupFrameString(mBC, id_chain[0]));
  if( id_chain.size() >= 2 ) EXPECT_EQ("b", tBC._lookupFrameString(mBC, id_chain[1]));
  if( id_chain.size() >= 3 ) EXPECT_EQ("c", tBC._lookupFrameString(mBC, id_chain[2]));
  if( id_chain.size() >= 4 ) EXPECT_EQ("d", tBC._lookupFrameString(mBC, id_chain[3]));
  if( id_chain.size() >= 5 ) EXPECT_EQ("e", tBC._lookupFrameString(mBC, id_chain[4]));

}

TEST(tf2_walkToTopParent, walk_v_configuration)
{
  tf2::BufferCore mBC;
  tf2::TestBufferCore tBC;

  geometry_msgs::TransformStamped st;
  st.header.stamp = ros::Time(0);
  st.transform.rotation.w = 1;

  // st.header.frame_id = "aaa";
  // st.child_frame_id = "aa";
  // mBC.setTransform(st, "authority1");
  // 
  // st.header.frame_id = "aa";
  // st.child_frame_id = "a";
  // mBC.setTransform(st, "authority1");  

  st.header.frame_id = "a";
  st.child_frame_id = "b";
  mBC.setTransform(st, "authority1");

  st.header.frame_id = "b";
  st.child_frame_id = "c";
  mBC.setTransform(st, "authority1");

  st.header.frame_id = "a";
  st.child_frame_id = "d";
  mBC.setTransform(st, "authority1");

  st.header.frame_id = "d";
  st.child_frame_id = "e";
  mBC.setTransform(st, "authority1");

  std::vector<tf2::CompactFrameID> id_chain;
  tBC._walkToTopParent(mBC,  ros::Time(), mBC._lookupFrameNumber("e"), mBC._lookupFrameNumber("c"), 0, &id_chain);

  EXPECT_EQ(5, id_chain.size());
  if( id_chain.size() >= 1 ) EXPECT_EQ("c", tBC._lookupFrameString(mBC, id_chain[0]));
  if( id_chain.size() >= 2 ) EXPECT_EQ("b", tBC._lookupFrameString(mBC, id_chain[1]));
  if( id_chain.size() >= 3 ) EXPECT_EQ("a", tBC._lookupFrameString(mBC, id_chain[2]));
  if( id_chain.size() >= 4 ) EXPECT_EQ("d", tBC._lookupFrameString(mBC, id_chain[3]));
  if( id_chain.size() >= 5 ) EXPECT_EQ("e", tBC._lookupFrameString(mBC, id_chain[4]));

  tBC._walkToTopParent(mBC,  ros::Time(), mBC._lookupFrameNumber("c"), mBC._lookupFrameNumber("e"), 0, &id_chain);
  EXPECT_EQ(5, id_chain.size());
  if( id_chain.size() >= 1 ) EXPECT_EQ("e", tBC._lookupFrameString(mBC, id_chain[0]));
  if( id_chain.size() >= 2 ) EXPECT_EQ("d", tBC._lookupFrameString(mBC, id_chain[1]));
  if( id_chain.size() >= 3 ) EXPECT_EQ("a", tBC._lookupFrameString(mBC, id_chain[2]));
  if( id_chain.size() >= 4 ) EXPECT_EQ("b", tBC._lookupFrameString(mBC, id_chain[3]));
  if( id_chain.size() >= 5 ) EXPECT_EQ("c", tBC._lookupFrameString(mBC, id_chain[4]));

  tBC._walkToTopParent(mBC,  ros::Time(), mBC._lookupFrameNumber("a"), mBC._lookupFrameNumber("e"), 0, &id_chain);
  EXPECT_EQ( id_chain.size(), 3 );
  if( id_chain.size() >= 1 ) EXPECT_EQ("e", tBC._lookupFrameString(mBC, id_chain[0]));
  if( id_chain.size() >= 2 ) EXPECT_EQ("d", tBC._lookupFrameString(mBC, id_chain[1]));
  if( id_chain.size() >= 3 ) EXPECT_EQ("a", tBC._lookupFrameString(mBC, id_chain[2]));

  tBC._walkToTopParent(mBC,  ros::Time(), mBC._lookupFrameNumber("e"), mBC._lookupFrameNumber("a"), 0, &id_chain);
  EXPECT_EQ( id_chain.size(), 3 );
  if( id_chain.size() >= 1 ) EXPECT_EQ("a", tBC._lookupFrameString(mBC, id_chain[0]));
  if( id_chain.size() >= 2 ) EXPECT_EQ("d", tBC._lookupFrameString(mBC, id_chain[1]));
  if( id_chain.size() >= 3 ) EXPECT_EQ("e", tBC._lookupFrameString(mBC, id_chain[2]));

  tBC._walkToTopParent(mBC,  ros::Time(), mBC._lookupFrameNumber("e"), mBC._lookupFrameNumber("d"), 0, &id_chain);
  EXPECT_EQ( id_chain.size(), 2 );
  if( id_chain.size() >= 1 ) EXPECT_EQ("d", tBC._lookupFrameString(mBC, id_chain[0]));
  if( id_chain.size() >= 2 ) EXPECT_EQ("e", tBC._lookupFrameString(mBC, id_chain[1]));

  tBC._walkToTopParent(mBC,  ros::Time(), mBC._lookupFrameNumber("d"), mBC._lookupFrameNumber("e"), 0, &id_chain);
  EXPECT_EQ( id_chain.size(), 2 );
  if( id_chain.size() >= 1 ) EXPECT_EQ("e", tBC._lookupFrameString(mBC, id_chain[0]));
  if( id_chain.size() >= 2 ) EXPECT_EQ("d", tBC._lookupFrameString(mBC, id_chain[1]));
}


int main(int argc, char **argv){
  testing::InitGoogleTest(&argc, argv);
  ros::Time::init(); //needed for ros::TIme::now()
  return RUN_ALL_TESTS();
}
