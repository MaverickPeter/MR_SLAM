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

#include <cmath>
#include <gtest/gtest.h>
#include <tf2/buffer_core.h>
#include "tf2/exceptions.h"
#include <ros/ros.h>
#include "LinearMath/btVector3.h"
#include "LinearMath/btTransform.h"
#include "rostest/permuter.h"

void setIdentity(geometry_msgs::Transform& trans)
{
  trans.translation.x = 0;
  trans.translation.y = 0;
  trans.translation.z = 0;
  trans.rotation.x = 0;
  trans.rotation.y = 0;
  trans.rotation.z = 0;
  trans.rotation.w = 1;
}


void push_back_i(std::vector<std::string>& children, std::vector<std::string>& parents,
                 std::vector<double>& dx, std::vector<double>& dy)
{
  /*
     "a"
     v   (1,0)
     "b"
     v   (1,0)
     "c"
  */

  children.push_back("b");
  parents.push_back("a");
  dx.push_back(1.0);
  dy.push_back(0.0);
  children.push_back("c");
  parents.push_back("b");
  dx.push_back(1.0);
  dy.push_back(0.0);
}


void push_back_y(std::vector<std::string>& children, std::vector<std::string>& parents,
                 std::vector<double>& dx, std::vector<double>& dy)
{
    /*
      "a"
      v  (1,0)
      "b" ------(0,1)-----> "d"
      v  (1,0)              v  (0,1)
      "c"                   "e"
    */
    // a>b
    children.push_back("b");
    parents.push_back("a");
    dx.push_back(1.0);
    dy.push_back(0.0);
     // b>c
    children.push_back("c");
    parents.push_back("b");
    dx.push_back(1.0);
    dy.push_back(0.0);
     // b>d
    children.push_back("d");
    parents.push_back("b");
    dx.push_back(0.0);
    dy.push_back(1.0);
     // d>e
    children.push_back("e");
    parents.push_back("d");
    dx.push_back(0.0);
    dy.push_back(1.0);
}

void push_back_v(std::vector<std::string>& children, std::vector<std::string>& parents,
                 std::vector<double>& dx, std::vector<double>& dy)
{
  /*
    "a" ------(0,1)-----> "f"
    v  (1,0)              v  (0,1)
    "b"                   "g"
    v  (1,0)
    "c"
  */
  // a>b
  children.push_back("b");
  parents.push_back("a");
  dx.push_back(1.0);
  dy.push_back(0.0);
  // b>c
  children.push_back("c");
  parents.push_back("b");
  dx.push_back(1.0);
  dy.push_back(0.0);
  // a>f
  children.push_back("f");
  parents.push_back("a");
  dx.push_back(0.0);
  dy.push_back(1.0);
  // f>g
  children.push_back("g");
  parents.push_back("f");
  dx.push_back(0.0);
  dy.push_back(1.0);

}

void push_back_1(std::vector<std::string>& children, std::vector<std::string>& parents,
                 std::vector<double>& dx, std::vector<double>& dy)
{
  children.push_back("2");
  parents.push_back("1");
  dx.push_back(1.0);
  dy.push_back(0.0);
}

void setupTree(tf2::BufferCore& mBC, const std::string& mode, const ros::Time & time, const ros::Duration& interpolation_space = ros::Duration())
{
  ROS_DEBUG("Clearing Buffer Core for new test setup");
  mBC.clear();

  ROS_DEBUG("Setting up test tree for formation %s", mode.c_str());

  std::vector<std::string> children;
  std::vector<std::string> parents;
  std::vector<double> dx, dy;

  if (mode == "i")
  {
    push_back_i(children, parents, dx, dy);
  }
  else if (mode == "y")
  {
    push_back_y(children, parents, dx, dy);
  }

  else if (mode == "v")
  {
    push_back_v(children, parents, dx, dy);
  }

  else if (mode == "ring_45")
  {
    /* Form a ring of transforms at every 45 degrees on the unit circle.  */

    std::vector<std::string> frames;



    frames.push_back("a");
    frames.push_back("b");
    frames.push_back("c");
    frames.push_back("d");
    frames.push_back("e");
    frames.push_back("f");
    frames.push_back("g");
    frames.push_back("h");
    frames.push_back("i");

    for (uint8_t iteration = 0; iteration < 2; ++iteration)
    {
      double direction = 1;
      std::string frame_prefix;
      if (iteration == 0)
      {
        frame_prefix = "inverse_";
        direction = -1;
      }
      else
        frame_prefix ="";
      for (uint64_t i = 1; i <  frames.size(); i++)
      {
        geometry_msgs::TransformStamped ts;
        setIdentity(ts.transform);
        ts.transform.translation.x = direction * ( sqrt(2)/2 - 1);
        ts.transform.translation.y = direction * sqrt(2)/2;
        ts.transform.rotation.x = 0;
        ts.transform.rotation.y = 0;
        ts.transform.rotation.z = sin(direction * M_PI/8);
        ts.transform.rotation.w = cos(direction * M_PI/8);
        if (time > ros::Time() + (interpolation_space * .5))
          ts.header.stamp = time - (interpolation_space * .5);
        else
          ts.header.stamp = ros::Time();

        ts.child_frame_id = frame_prefix + frames[i];
        if (i > 1)
          ts.header.frame_id = frame_prefix + frames[i-1];
        else
          ts.header.frame_id = frames[i-1];

        EXPECT_TRUE(mBC.setTransform(ts, "authority"));
        if (interpolation_space > ros::Duration())
        {
          ts.header.stamp = time + interpolation_space * .5;
          EXPECT_TRUE(mBC.setTransform(ts, "authority"));

        }
      }
    }
    return; // nonstandard setup return before standard executinog
  }
  else if (mode == "1")
  {
    push_back_1(children, parents, dx, dy);

  }
  else if (mode =="1_v")
  {
    push_back_1(children, parents, dx, dy);
    push_back_v(children, parents, dx, dy);
  }
  else
    EXPECT_FALSE("Undefined mode for tree setup.  Test harness improperly setup.");


  /// Standard
  for (uint64_t i = 0; i <  children.size(); i++)
  {
    geometry_msgs::TransformStamped ts;
    setIdentity(ts.transform);
    ts.transform.translation.x = dx[i];
    ts.transform.translation.y = dy[i];
    if (time > ros::Time() + (interpolation_space * .5))
      ts.header.stamp = time - (interpolation_space * .5);
    else
      ts.header.stamp = ros::Time();

    ts.header.frame_id = parents[i];
    ts.child_frame_id = children[i];
    EXPECT_TRUE(mBC.setTransform(ts, "authority"));
    if (interpolation_space > ros::Duration())
    {
      ts.header.stamp = time + interpolation_space * .5;
      EXPECT_TRUE(mBC.setTransform(ts, "authority"));

    }
  }
}


TEST(BufferCore_setTransform, NoInsertOnSelfTransform)
{
  tf2::BufferCore mBC;
  geometry_msgs::TransformStamped tranStamped;
  setIdentity(tranStamped.transform);
  tranStamped.header.stamp = ros::Time().fromNSec(10.0);
  tranStamped.header.frame_id = "same_frame";
  tranStamped.child_frame_id = "same_frame";
  EXPECT_FALSE(mBC.setTransform(tranStamped, "authority"));
}

TEST(BufferCore_setTransform, NoInsertWithNan)
{
  tf2::BufferCore mBC;
  geometry_msgs::TransformStamped tranStamped;
  setIdentity(tranStamped.transform);
  tranStamped.header.stamp = ros::Time().fromNSec(10.0);
  tranStamped.header.frame_id = "same_frame";
  tranStamped.child_frame_id = "other_frame";
  EXPECT_TRUE(mBC.setTransform(tranStamped, "authority"));
  tranStamped.transform.translation.x = std::nan("");
  EXPECT_TRUE(std::isnan(tranStamped.transform.translation.x));
  EXPECT_FALSE(mBC.setTransform(tranStamped, "authority"));

}

TEST(BufferCore_setTransform, NoInsertWithNoFrameID)
{
  tf2::BufferCore mBC;
  geometry_msgs::TransformStamped tranStamped;
  setIdentity(tranStamped.transform);
  tranStamped.header.stamp = ros::Time().fromNSec(10.0);
  tranStamped.header.frame_id = "same_frame";
  tranStamped.child_frame_id = "";
  EXPECT_FALSE(mBC.setTransform(tranStamped, "authority"));
  tranStamped.child_frame_id = "/";
  EXPECT_FALSE(mBC.setTransform(tranStamped, "authority"));

}

TEST(BufferCore_setTransform, NoInsertWithNoParentID)
{
  tf2::BufferCore mBC;
  geometry_msgs::TransformStamped tranStamped;
  setIdentity(tranStamped.transform);
  tranStamped.header.stamp = ros::Time().fromNSec(10.0);
  tranStamped.header.frame_id = "";
  tranStamped.child_frame_id = "some_frame";
  EXPECT_FALSE(mBC.setTransform(tranStamped, "authority"));

  tranStamped.header.frame_id = "/";
  EXPECT_FALSE(mBC.setTransform(tranStamped, "authority"));
}

/*
TEST(tf, ListOneInverse)
{
  unsigned int runs = 4;
  double epsilon = 1e-6;
  seed_rand();

  tf::Transformer mTR(true);
  std::vector<double> xvalues(runs), yvalues(runs), zvalues(runs);
  for ( uint64_t i = 0; i < runs ; i++ )
  {
    xvalues[i] = 10.0 * ((double) rand() - (double)RAND_MAX /2.0) /(double)RAND_MAX;
    yvalues[i] = 10.0 * ((double) rand() - (double)RAND_MAX /2.0) /(double)RAND_MAX;
    zvalues[i] = 10.0 * ((double) rand() - (double)RAND_MAX /2.0) /(double)RAND_MAX;

    StampedTransform tranStamped (btTransform(btQuaternion(0,0,0,1), btVector3(xvalues[i],yvalues[i],zvalues[i])), ros::Time().fromNSec(10 + i),  "my_parent", "child");
    mTR.setTransform(tranStamped);
  }

  //  std::cout << mTR.allFramesAsString() << std::endl;
  //  std::cout << mTR.chainAsString("child", 0, "my_parent2", 0, "my_parent2") << std::endl;

  for ( uint64_t i = 0; i < runs ; i++ )

  {
    Stamped<Pose> inpose (btTransform(btQuaternion(0,0,0,1), btVector3(0,0,0)), ros::Time().fromNSec(10 + i), "child");

    try{
    Stamped<Pose> outpose;
    outpose.setIdentity(); //to make sure things are getting mutated
    mTR.transformPose("my_parent",inpose, outpose);
    EXPECT_NEAR(outpose.getOrigin().x(), xvalues[i], epsilon);
    EXPECT_NEAR(outpose.getOrigin().y(), yvalues[i], epsilon);
    EXPECT_NEAR(outpose.getOrigin().z(), zvalues[i], epsilon);
    }
    catch (tf::TransformException & ex)
    {
      std::cout << "TransformExcepion got through!!!!! " << ex.what() << std::endl;
      bool exception_improperly_thrown = true;
      EXPECT_FALSE(exception_improperly_thrown);
    }
  }

}

TEST(tf, ListTwoInverse)
{
  unsigned int runs = 4;
  double epsilon = 1e-6;
  seed_rand();

  tf::Transformer mTR(true);
  std::vector<double> xvalues(runs), yvalues(runs), zvalues(runs);
  for ( unsigned int i = 0; i < runs ; i++ )
  {
    xvalues[i] = 10.0 * ((double) rand() - (double)RAND_MAX /2.0) /(double)RAND_MAX;
    yvalues[i] = 10.0 * ((double) rand() - (double)RAND_MAX /2.0) /(double)RAND_MAX;
    zvalues[i] = 10.0 * ((double) rand() - (double)RAND_MAX /2.0) /(double)RAND_MAX;

    StampedTransform tranStamped(btTransform(btQuaternion(0,0,0,1), btVector3(xvalues[i],yvalues[i],zvalues[i])), ros::Time().fromNSec(10 + i),  "my_parent", "child");
    mTR.setTransform(tranStamped);
    StampedTransform tranStamped2(btTransform(btQuaternion(0,0,0,1), btVector3(xvalues[i],yvalues[i],zvalues[i])), ros::Time().fromNSec(10 + i),  "child", "grandchild");
    mTR.setTransform(tranStamped2);
  }

  //  std::cout << mTR.allFramesAsString() << std::endl;
  //  std::cout << mTR.chainAsString("child", 0, "my_parent2", 0, "my_parent2") << std::endl;

  for ( unsigned int i = 0; i < runs ; i++ )

  {
    Stamped<Pose> inpose (btTransform(btQuaternion(0,0,0,1), btVector3(0,0,0)), ros::Time().fromNSec(10 + i), "grandchild");

    try{
    Stamped<Pose> outpose;
    outpose.setIdentity(); //to make sure things are getting mutated
    mTR.transformPose("my_parent",inpose, outpose);
    EXPECT_NEAR(outpose.getOrigin().x(), 2*xvalues[i], epsilon);
    EXPECT_NEAR(outpose.getOrigin().y(), 2*yvalues[i], epsilon);
    EXPECT_NEAR(outpose.getOrigin().z(), 2*zvalues[i], epsilon);
    }
    catch (tf::TransformException & ex)
    {
      std::cout << "TransformExcepion got through!!!!! " << ex.what() << std::endl;
      bool exception_improperly_thrown = true;
      EXPECT_FALSE(exception_improperly_thrown);
    }
  }

}


TEST(tf, ListOneForward)
{
  unsigned int runs = 4;
  double epsilon = 1e-6;
  seed_rand();

  tf::Transformer mTR(true);
  std::vector<double> xvalues(runs), yvalues(runs), zvalues(runs);
  for ( uint64_t i = 0; i < runs ; i++ )
  {
    xvalues[i] = 10.0 * ((double) rand() - (double)RAND_MAX /2.0) /(double)RAND_MAX;
    yvalues[i] = 10.0 * ((double) rand() - (double)RAND_MAX /2.0) /(double)RAND_MAX;
    zvalues[i] = 10.0 * ((double) rand() - (double)RAND_MAX /2.0) /(double)RAND_MAX;

    StampedTransform tranStamped(btTransform(btQuaternion(0,0,0,1), btVector3(xvalues[i],yvalues[i],zvalues[i])), ros::Time().fromNSec(10 + i),  "my_parent", "child");
    mTR.setTransform(tranStamped);
  }

  //  std::cout << mTR.allFramesAsString() << std::endl;
  //  std::cout << mTR.chainAsString("child", 0, "my_parent2", 0, "my_parent2") << std::endl;

  for ( uint64_t i = 0; i < runs ; i++ )

  {
    Stamped<Pose> inpose (btTransform(btQuaternion(0,0,0,1), btVector3(0,0,0)), ros::Time().fromNSec(10 + i), "my_parent");

    try{
    Stamped<Pose> outpose;
    outpose.setIdentity(); //to make sure things are getting mutated
    mTR.transformPose("child",inpose, outpose);
    EXPECT_NEAR(outpose.getOrigin().x(), -xvalues[i], epsilon);
    EXPECT_NEAR(outpose.getOrigin().y(), -yvalues[i], epsilon);
    EXPECT_NEAR(outpose.getOrigin().z(), -zvalues[i], epsilon);
    }
    catch (tf::TransformException & ex)
    {
      std::cout << "TransformExcepion got through!!!!! " << ex.what() << std::endl;
      bool exception_improperly_thrown = true;
      EXPECT_FALSE(exception_improperly_thrown);
    }
  }

}

TEST(tf, ListTwoForward)
{
  unsigned int runs = 4;
  double epsilon = 1e-6;
  seed_rand();

  tf::Transformer mTR(true);
  std::vector<double> xvalues(runs), yvalues(runs), zvalues(runs);
  for ( unsigned int i = 0; i < runs ; i++ )
  {
    xvalues[i] = 10.0 * ((double) rand() - (double)RAND_MAX /2.0) /(double)RAND_MAX;
    yvalues[i] = 10.0 * ((double) rand() - (double)RAND_MAX /2.0) /(double)RAND_MAX;
    zvalues[i] = 10.0 * ((double) rand() - (double)RAND_MAX /2.0) /(double)RAND_MAX;

    StampedTransform tranStamped(btTransform(btQuaternion(0,0,0,1), btVector3(xvalues[i],yvalues[i],zvalues[i])), ros::Time().fromNSec(10 + i),  "my_parent", "child");
    mTR.setTransform(tranStamped);
    StampedTransform tranStamped2(btTransform(btQuaternion(0,0,0,1), btVector3(xvalues[i],yvalues[i],zvalues[i])), ros::Time().fromNSec(10 + i),  "child", "grandchild");
    mTR.setTransform(tranStamped2);
  }

  //  std::cout << mTR.allFramesAsString() << std::endl;
  //  std::cout << mTR.chainAsString("child", 0, "my_parent2", 0, "my_parent2") << std::endl;

  for ( unsigned int i = 0; i < runs ; i++ )

  {
    Stamped<Pose> inpose (btTransform(btQuaternion(0,0,0,1), btVector3(0,0,0)), ros::Time().fromNSec(10 + i), "my_parent");

    try{
    Stamped<Pose> outpose;
    outpose.setIdentity(); //to make sure things are getting mutated
    mTR.transformPose("grandchild",inpose, outpose);
    EXPECT_NEAR(outpose.getOrigin().x(), -2*xvalues[i], epsilon);
    EXPECT_NEAR(outpose.getOrigin().y(), -2*yvalues[i], epsilon);
    EXPECT_NEAR(outpose.getOrigin().z(), -2*zvalues[i], epsilon);
    }
    catch (tf::TransformException & ex)
    {
      std::cout << "TransformExcepion got through!!!!! " << ex.what() << std::endl;
      bool exception_improperly_thrown = true;
      EXPECT_FALSE(exception_improperly_thrown);
    }
  }

}

TEST(tf, TransformThrougRoot)
{
  unsigned int runs = 4;
  double epsilon = 1e-6;
  seed_rand();

  tf::Transformer mTR(true);
  std::vector<double> xvalues(runs), yvalues(runs), zvalues(runs);
  for ( unsigned int i = 0; i < runs ; i++ )
  {
    xvalues[i] = 10.0 * ((double) rand() - (double)RAND_MAX /2.0) /(double)RAND_MAX;
    yvalues[i] = 10.0 * ((double) rand() - (double)RAND_MAX /2.0) /(double)RAND_MAX;
    zvalues[i] = 10.0 * ((double) rand() - (double)RAND_MAX /2.0) /(double)RAND_MAX;

    StampedTransform tranStamped(btTransform(btQuaternion(0,0,0,1), btVector3(xvalues[i],yvalues[i],zvalues[i])), ros::Time().fromNSec(1000 + i*100),  "my_parent", "childA");
    mTR.setTransform(tranStamped);
    StampedTransform tranStamped2(btTransform(btQuaternion(0,0,0,1), btVector3(xvalues[i],yvalues[i],zvalues[i])), ros::Time().fromNSec(1000 + i*100),  "my_parent", "childB");
    mTR.setTransform(tranStamped2);
  }

  //  std::cout << mTR.allFramesAsString() << std::endl;
  //  std::cout << mTR.chainAsString("child", 0, "my_parent2", 0, "my_parent2") << std::endl;

  for ( unsigned int i = 0; i < runs ; i++ )

  {
    Stamped<Pose> inpose (btTransform(btQuaternion(0,0,0,1), btVector3(0,0,0)), ros::Time().fromNSec(1000 + i*100), "childA");

    try{
    Stamped<Pose> outpose;
    outpose.setIdentity(); //to make sure things are getting mutated
    mTR.transformPose("childB",inpose, outpose);
    EXPECT_NEAR(outpose.getOrigin().x(), 0*xvalues[i], epsilon);
    EXPECT_NEAR(outpose.getOrigin().y(), 0*yvalues[i], epsilon);
    EXPECT_NEAR(outpose.getOrigin().z(), 0*zvalues[i], epsilon);
    }
    catch (tf::TransformException & ex)
    {
      std::cout << "TransformExcepion got through!!!!! " << ex.what() << std::endl;
      bool exception_improperly_thrown = true;
      EXPECT_FALSE(exception_improperly_thrown);
    }
  }

}

TEST(tf, TransformThroughNO_PARENT)
{
  unsigned int runs = 4;
  double epsilon = 1e-6;
  seed_rand();

  tf::Transformer mTR(true);
  std::vector<double> xvalues(runs), yvalues(runs), zvalues(runs);
  for ( unsigned int i = 0; i < runs ; i++ )
  {
    xvalues[i] = 10.0 * ((double) rand() - (double)RAND_MAX /2.0) /(double)RAND_MAX;
    yvalues[i] = 10.0 * ((double) rand() - (double)RAND_MAX /2.0) /(double)RAND_MAX;
    zvalues[i] = 10.0 * ((double) rand() - (double)RAND_MAX /2.0) /(double)RAND_MAX;

    StampedTransform tranStamped(btTransform(btQuaternion(0,0,0,1), btVector3(xvalues[i],yvalues[i],zvalues[i])), ros::Time().fromNSec(10 + i),  "my_parentA", "childA");
    mTR.setTransform(tranStamped);
    StampedTransform tranStamped2(btTransform(btQuaternion(0,0,0,1), btVector3(xvalues[i],yvalues[i],zvalues[i])), ros::Time().fromNSec(10 + i),  "my_parentB", "childB");
    mTR.setTransform(tranStamped2);
  }

  //  std::cout << mTR.allFramesAsString() << std::endl;
  //  std::cout << mTR.chainAsString("child", 0, "my_parent2", 0, "my_parent2") << std::endl;

  for ( unsigned int i = 0; i < runs ; i++ )

  {
    Stamped<btTransform> inpose (btTransform(btQuaternion(0,0,0,1), btVector3(0,0,0)), ros::Time().fromNSec(10 + i), "childA");
    bool exception_thrown = false;

    try{
    Stamped<btTransform> outpose;
    outpose.setIdentity(); //to make sure things are getting mutated
    mTR.transformPose("childB",inpose, outpose);
    EXPECT_NEAR(outpose.getOrigin().x(), 0*xvalues[i], epsilon);
    EXPECT_NEAR(outpose.getOrigin().y(), 0*yvalues[i], epsilon);
    EXPECT_NEAR(outpose.getOrigin().z(), 0*zvalues[i], epsilon);
    }
    catch (tf::TransformException & ex)
    {
      exception_thrown = true;
    }
    EXPECT_TRUE(exception_thrown);
  }

}

*/


TEST(BufferCore_lookupTransform, i_configuration)
{
  double epsilon = 1e-6;



  rostest::Permuter permuter;

  std::vector<ros::Time> times;
  times.push_back(ros::Time(1.0));
  times.push_back(ros::Time(10.0));
  times.push_back(ros::Time(0.0));
  ros::Time eval_time;
  permuter.addOptionSet(times, &eval_time);

  std::vector<ros::Duration> durations;
  durations.push_back(ros::Duration(1.0));
  durations.push_back(ros::Duration(0.001));
  durations.push_back(ros::Duration(0.1));
  ros::Duration interpolation_space;
  //  permuter.addOptionSet(durations, &interpolation_space);

  std::vector<std::string> frames;
  frames.push_back("a");
  frames.push_back("b");
  frames.push_back("c");
  std::string source_frame;
  permuter.addOptionSet(frames, &source_frame);

  std::string target_frame;
  permuter.addOptionSet(frames, &target_frame);

  while  (permuter.step())
  {

    tf2::BufferCore mBC;
    setupTree(mBC, "i", eval_time, interpolation_space);

    geometry_msgs::TransformStamped outpose = mBC.lookupTransform(source_frame, target_frame, eval_time);
    //printf("source_frame %s target_frame %s time %f\n", source_frame.c_str(), target_frame.c_str(), eval_time.toSec());
    EXPECT_EQ(outpose.header.stamp, eval_time);
    EXPECT_EQ(outpose.header.frame_id, source_frame);
    EXPECT_EQ(outpose.child_frame_id, target_frame);
    EXPECT_NEAR(outpose.transform.translation.y, 0, epsilon);
    EXPECT_NEAR(outpose.transform.translation.z, 0, epsilon);
    EXPECT_NEAR(outpose.transform.rotation.x, 0, epsilon);
    EXPECT_NEAR(outpose.transform.rotation.y, 0, epsilon);
    EXPECT_NEAR(outpose.transform.rotation.z, 0, epsilon);
    EXPECT_NEAR(outpose.transform.rotation.w, 1, epsilon);

    //Zero distance
    if (source_frame == target_frame)
    {
      EXPECT_NEAR(outpose.transform.translation.x, 0, epsilon);
    }
    else if ((source_frame == "a" && target_frame =="b") ||
             (source_frame == "b" && target_frame =="c"))
    {
      EXPECT_NEAR(outpose.transform.translation.x, 1, epsilon);
    }
    else if ((source_frame == "b" && target_frame =="a") ||
             (source_frame == "c" && target_frame =="b"))
    {
      EXPECT_NEAR(outpose.transform.translation.x, -1, epsilon);
    }
    else if (source_frame == "a" && target_frame =="c")
    {
      EXPECT_NEAR(outpose.transform.translation.x, 2, epsilon);
    }
    else if (source_frame == "c" && target_frame =="a")
    {
      EXPECT_NEAR(outpose.transform.translation.x, -2, epsilon);
    }
    else
    {
      EXPECT_FALSE("i configuration: Shouldn't get here");
      printf("source_frame %s target_frame %s time %f\n", source_frame.c_str(), target_frame.c_str(), eval_time.toSec());
    }

  }
}

/* Check 1 result return false if test parameters unmet */
bool check_1_result(const geometry_msgs::TransformStamped& outpose, const std::string& source_frame, const std::string& target_frame, const ros::Time& eval_time, double epsilon)
{
  //printf("source_frame %s target_frame %s time %f\n", source_frame.c_str(), target_frame.c_str(), eval_time.toSec());
  EXPECT_EQ(outpose.header.stamp, eval_time);
  EXPECT_EQ(outpose.header.frame_id, source_frame);
  EXPECT_EQ(outpose.child_frame_id, target_frame);
  EXPECT_NEAR(outpose.transform.translation.y, 0, epsilon);
  EXPECT_NEAR(outpose.transform.translation.z, 0, epsilon);
  EXPECT_NEAR(outpose.transform.rotation.x, 0, epsilon);
  EXPECT_NEAR(outpose.transform.rotation.y, 0, epsilon);
  EXPECT_NEAR(outpose.transform.rotation.z, 0, epsilon);
  EXPECT_NEAR(outpose.transform.rotation.w, 1, epsilon);

  //Zero distance
  if (source_frame == target_frame)
  {
    EXPECT_NEAR(outpose.transform.translation.x, 0, epsilon);
  }
  else if (source_frame == "1" && target_frame =="2")
  {
    EXPECT_NEAR(outpose.transform.translation.x, 1, epsilon);
  }
  else if (source_frame == "2" && target_frame =="1")
  {
    EXPECT_NEAR(outpose.transform.translation.x, -1, epsilon);
  }
  else
  {
    //printf("source_frame %s target_frame %s time %f\n", source_frame.c_str(), target_frame.c_str(), eval_time.toSec());
    return false;
  }
  return true;
}

/* Check v result return false if test parameters unmet */
bool check_v_result(const geometry_msgs::TransformStamped& outpose, const std::string& source_frame, const std::string& target_frame, const ros::Time& eval_time, double epsilon)
{
  //printf("source_frame %s target_frame %s time %f\n", source_frame.c_str(), target_frame.c_str(), eval_time.toSec());
  EXPECT_EQ(outpose.header.stamp, eval_time);
  EXPECT_EQ(outpose.header.frame_id, source_frame);
  EXPECT_EQ(outpose.child_frame_id, target_frame);
  EXPECT_NEAR(outpose.transform.translation.z, 0, epsilon);
  EXPECT_NEAR(outpose.transform.rotation.x, 0, epsilon);
  EXPECT_NEAR(outpose.transform.rotation.y, 0, epsilon);
  EXPECT_NEAR(outpose.transform.rotation.z, 0, epsilon);
  EXPECT_NEAR(outpose.transform.rotation.w, 1, epsilon);

  //Zero distance
  if (source_frame == target_frame)
  {
    EXPECT_NEAR(outpose.transform.translation.x, 0, epsilon);
  }
  else if ((source_frame == "a" && target_frame =="b") ||
           (source_frame == "b" && target_frame =="c"))
  {
    EXPECT_NEAR(outpose.transform.translation.x, 1, epsilon);
    EXPECT_NEAR(outpose.transform.translation.y, 0, epsilon);
  }
  else if ((source_frame == "b" && target_frame =="a") ||
           (source_frame == "c" && target_frame =="b"))
  {
    EXPECT_NEAR(outpose.transform.translation.x, -1, epsilon);
    EXPECT_NEAR(outpose.transform.translation.y, 0, epsilon);
  }
  else if ((source_frame == "a" && target_frame =="f") ||
           (source_frame == "f" && target_frame =="g"))
  {
    EXPECT_NEAR(outpose.transform.translation.x, 0, epsilon);
    EXPECT_NEAR(outpose.transform.translation.y, 1, epsilon);
  }
  else if ((source_frame == "f" && target_frame =="a") ||
           (source_frame == "g" && target_frame =="f"))
  {
    EXPECT_NEAR(outpose.transform.translation.x, 0, epsilon);
    EXPECT_NEAR(outpose.transform.translation.y, -1, epsilon);
  }
  else if (source_frame == "a" && target_frame =="g")
  {
    EXPECT_NEAR(outpose.transform.translation.x, 0, epsilon);
    EXPECT_NEAR(outpose.transform.translation.y, 2, epsilon);
  }
  else if (source_frame == "g" && target_frame =="a")
  {
    EXPECT_NEAR(outpose.transform.translation.x, 0, epsilon);
    EXPECT_NEAR(outpose.transform.translation.y, -2, epsilon);
  }
  else if (source_frame == "a" && target_frame =="c")
  {
    EXPECT_NEAR(outpose.transform.translation.x, 2, epsilon);
    EXPECT_NEAR(outpose.transform.translation.y, 0, epsilon);
  }
  else if (source_frame == "c" && target_frame =="a")
  {
    EXPECT_NEAR(outpose.transform.translation.x, -2, epsilon);
    EXPECT_NEAR(outpose.transform.translation.y, 0, epsilon);
  }
  else if (source_frame == "b" && target_frame =="f")
  {
    EXPECT_NEAR(outpose.transform.translation.x, -1, epsilon);
    EXPECT_NEAR(outpose.transform.translation.y, 1, epsilon);
  }
  else if (source_frame == "f" && target_frame =="b")
  {
    EXPECT_NEAR(outpose.transform.translation.x, 1, epsilon);
    EXPECT_NEAR(outpose.transform.translation.y, -1, epsilon);
  }
  else if (source_frame == "c" && target_frame =="f")
  {
    EXPECT_NEAR(outpose.transform.translation.x, -2, epsilon);
    EXPECT_NEAR(outpose.transform.translation.y, 1, epsilon);
  }
  else if (source_frame == "f" && target_frame =="c")
  {
    EXPECT_NEAR(outpose.transform.translation.x, 2, epsilon);
    EXPECT_NEAR(outpose.transform.translation.y, -1, epsilon);
  }
  else if (source_frame == "b" && target_frame =="g")
  {
    EXPECT_NEAR(outpose.transform.translation.x, -1, epsilon);
    EXPECT_NEAR(outpose.transform.translation.y, 2, epsilon);
  }
  else if (source_frame == "g" && target_frame =="b")
  {
    EXPECT_NEAR(outpose.transform.translation.x, 1, epsilon);
    EXPECT_NEAR(outpose.transform.translation.y, -2, epsilon);
  }
  else if (source_frame == "c" && target_frame =="g")
  {
    EXPECT_NEAR(outpose.transform.translation.x, -2, epsilon);
    EXPECT_NEAR(outpose.transform.translation.y, 2, epsilon);
  }
  else if (source_frame == "g" && target_frame =="c")
  {
    EXPECT_NEAR(outpose.transform.translation.x, 2, epsilon);
    EXPECT_NEAR(outpose.transform.translation.y, -2, epsilon);
  }
  else
  {
    //printf("source_frame %s target_frame %s time %f\n", source_frame.c_str(), target_frame.c_str(), eval_time.toSec());
    return false;
  }
  return true;
}

/* Check v result return false if test parameters unmet */
bool check_y_result(const geometry_msgs::TransformStamped& outpose, const std::string& source_frame, const std::string& target_frame, const ros::Time& eval_time, double epsilon)
{
  //printf("source_frame %s target_frame %s time %f\n", source_frame.c_str(), target_frame.c_str(), eval_time.toSec());
  EXPECT_EQ(outpose.header.stamp, eval_time);
  EXPECT_EQ(outpose.header.frame_id, source_frame);
  EXPECT_EQ(outpose.child_frame_id, target_frame);
  EXPECT_NEAR(outpose.transform.translation.z, 0, epsilon);
  EXPECT_NEAR(outpose.transform.rotation.x, 0, epsilon);
  EXPECT_NEAR(outpose.transform.rotation.y, 0, epsilon);
  EXPECT_NEAR(outpose.transform.rotation.z, 0, epsilon);
  EXPECT_NEAR(outpose.transform.rotation.w, 1, epsilon);

  //Zero distance
  if (source_frame == target_frame)
  {
    EXPECT_NEAR(outpose.transform.translation.x, 0, epsilon);
  }
  else if ((source_frame == "a" && target_frame =="b") ||
           (source_frame == "b" && target_frame =="c"))
  {
    EXPECT_NEAR(outpose.transform.translation.x, 1, epsilon);
    EXPECT_NEAR(outpose.transform.translation.y, 0, epsilon);
  }
  else if ((source_frame == "b" && target_frame =="a") ||
           (source_frame == "c" && target_frame =="b"))
  {
    EXPECT_NEAR(outpose.transform.translation.x, -1, epsilon);
    EXPECT_NEAR(outpose.transform.translation.y, 0, epsilon);
  }
  else if ((source_frame == "b" && target_frame =="d") ||
           (source_frame == "d" && target_frame =="e"))
  {
    EXPECT_NEAR(outpose.transform.translation.x, 0, epsilon);
    EXPECT_NEAR(outpose.transform.translation.y, 1, epsilon);
  }
  else if ((source_frame == "d" && target_frame =="b") ||
           (source_frame == "e" && target_frame =="d"))
  {
    EXPECT_NEAR(outpose.transform.translation.x, 0, epsilon);
    EXPECT_NEAR(outpose.transform.translation.y, -1, epsilon);
  }
  else if (source_frame == "b" && target_frame =="e")
  {
    EXPECT_NEAR(outpose.transform.translation.x, 0, epsilon);
    EXPECT_NEAR(outpose.transform.translation.y, 2, epsilon);
  }
  else if (source_frame == "e" && target_frame =="b")
  {
    EXPECT_NEAR(outpose.transform.translation.x, 0, epsilon);
    EXPECT_NEAR(outpose.transform.translation.y, -2, epsilon);
  }
  else if (source_frame == "a" && target_frame =="c")
  {
    EXPECT_NEAR(outpose.transform.translation.x, 2, epsilon);
    EXPECT_NEAR(outpose.transform.translation.y, 0, epsilon);
  }
  else if (source_frame == "c" && target_frame =="a")
  {
    EXPECT_NEAR(outpose.transform.translation.x, -2, epsilon);
    EXPECT_NEAR(outpose.transform.translation.y, 0, epsilon);
  }
  else if (source_frame == "a" && target_frame =="d")
  {
    EXPECT_NEAR(outpose.transform.translation.x, 1, epsilon);
    EXPECT_NEAR(outpose.transform.translation.y, 1, epsilon);
  }
  else if (source_frame == "d" && target_frame =="a")
  {
    EXPECT_NEAR(outpose.transform.translation.x, -1, epsilon);
    EXPECT_NEAR(outpose.transform.translation.y, -1, epsilon);
  }
  else if (source_frame == "c" && target_frame =="d")
  {
    EXPECT_NEAR(outpose.transform.translation.x, -1, epsilon);
    EXPECT_NEAR(outpose.transform.translation.y, 1, epsilon);
  }
  else if (source_frame == "d" && target_frame =="c")
  {
    EXPECT_NEAR(outpose.transform.translation.x, 1, epsilon);
    EXPECT_NEAR(outpose.transform.translation.y, -1, epsilon);
  }
  else if (source_frame == "a" && target_frame =="e")
  {
    EXPECT_NEAR(outpose.transform.translation.x, 1, epsilon);
    EXPECT_NEAR(outpose.transform.translation.y, 2, epsilon);
  }
  else if (source_frame == "e" && target_frame =="a")
  {
    EXPECT_NEAR(outpose.transform.translation.x, -1, epsilon);
    EXPECT_NEAR(outpose.transform.translation.y, -2, epsilon);
  }
  else if (source_frame == "c" && target_frame =="e")
  {
    EXPECT_NEAR(outpose.transform.translation.x, -1, epsilon);
    EXPECT_NEAR(outpose.transform.translation.y, 2, epsilon);
  }
  else if (source_frame == "e" && target_frame =="c")
  {
    EXPECT_NEAR(outpose.transform.translation.x, 1, epsilon);
    EXPECT_NEAR(outpose.transform.translation.y, -2, epsilon);
  }
  else
  {
    //printf("source_frame %s target_frame %s time %f\n", source_frame.c_str(), target_frame.c_str(), eval_time.toSec());
    return false;
  }
  return true;
}


TEST(BufferCore_lookupTransform, one_link_configuration)
{
  double epsilon = 1e-6;



  rostest::Permuter permuter;

  std::vector<ros::Time> times;
  times.push_back(ros::Time(1.0));
  times.push_back(ros::Time(10.0));
  times.push_back(ros::Time(0.0));
  ros::Time eval_time;
  permuter.addOptionSet(times, &eval_time);

  std::vector<ros::Duration> durations;
  durations.push_back(ros::Duration(1.0));
  durations.push_back(ros::Duration(0.001));
  durations.push_back(ros::Duration(0.1));
  ros::Duration interpolation_space;
  //  permuter.addOptionSet(durations, &interpolation_space);

  std::vector<std::string> frames;
  frames.push_back("1");
  frames.push_back("2");
  std::string source_frame;
  permuter.addOptionSet(frames, &source_frame);

  std::string target_frame;
  permuter.addOptionSet(frames, &target_frame);

  while  (permuter.step())
  {

    tf2::BufferCore mBC;
    setupTree(mBC, "1", eval_time, interpolation_space);

    geometry_msgs::TransformStamped outpose = mBC.lookupTransform(source_frame, target_frame, eval_time);

    EXPECT_TRUE(check_1_result(outpose, source_frame, target_frame, eval_time, epsilon));
  }
}


TEST(BufferCore_lookupTransform, v_configuration)
{
  double epsilon = 1e-6;



  rostest::Permuter permuter;

  std::vector<ros::Time> times;
  times.push_back(ros::Time(1.0));
  times.push_back(ros::Time(10.0));
  times.push_back(ros::Time(0.0));
  ros::Time eval_time;
  permuter.addOptionSet(times, &eval_time);

  std::vector<ros::Duration> durations;
  durations.push_back(ros::Duration(1.0));
  durations.push_back(ros::Duration(0.001));
  durations.push_back(ros::Duration(0.1));
  ros::Duration interpolation_space;
  //  permuter.addOptionSet(durations, &interpolation_space);

  std::vector<std::string> frames;
  frames.push_back("a");
  frames.push_back("b");
  frames.push_back("c");
  frames.push_back("f");
  frames.push_back("g");
  std::string source_frame;
  permuter.addOptionSet(frames, &source_frame);

  std::string target_frame;
  permuter.addOptionSet(frames, &target_frame);

  while  (permuter.step())
  {

    tf2::BufferCore mBC;
    setupTree(mBC, "v", eval_time, interpolation_space);

    geometry_msgs::TransformStamped outpose = mBC.lookupTransform(source_frame, target_frame, eval_time);

    EXPECT_TRUE(check_v_result(outpose, source_frame, target_frame, eval_time, epsilon));
  }
}


TEST(BufferCore_lookupTransform, y_configuration)
{
  double epsilon = 1e-6;



  rostest::Permuter permuter;

  std::vector<ros::Time> times;
  times.push_back(ros::Time(1.0));
  times.push_back(ros::Time(10.0));
  times.push_back(ros::Time(0.0));
  ros::Time eval_time;
  permuter.addOptionSet(times, &eval_time);

  std::vector<ros::Duration> durations;
  durations.push_back(ros::Duration(1.0));
  durations.push_back(ros::Duration(0.001));
  durations.push_back(ros::Duration(0.1));
  ros::Duration interpolation_space;
  //  permuter.addOptionSet(durations, &interpolation_space);

  std::vector<std::string> frames;
  frames.push_back("a");
  frames.push_back("b");
  frames.push_back("c");
  frames.push_back("d");
  frames.push_back("e");
  std::string source_frame;
  permuter.addOptionSet(frames, &source_frame);

  std::string target_frame;
  permuter.addOptionSet(frames, &target_frame);

  while  (permuter.step())
  {

    tf2::BufferCore mBC;
    setupTree(mBC, "y", eval_time, interpolation_space);

    geometry_msgs::TransformStamped outpose = mBC.lookupTransform(source_frame, target_frame, eval_time);

    EXPECT_TRUE(check_y_result(outpose, source_frame, target_frame, eval_time, epsilon));
  }
}

TEST(BufferCore_lookupTransform, multi_configuration)
{
  double epsilon = 1e-6;



  rostest::Permuter permuter;

  std::vector<ros::Time> times;
  times.push_back(ros::Time(1.0));
  times.push_back(ros::Time(10.0));
  times.push_back(ros::Time(0.0));
  ros::Time eval_time;
  permuter.addOptionSet(times, &eval_time);

  std::vector<ros::Duration> durations;
  durations.push_back(ros::Duration(1.0));
  durations.push_back(ros::Duration(0.001));
  durations.push_back(ros::Duration(0.1));
  ros::Duration interpolation_space;
  //  permuter.addOptionSet(durations, &interpolation_space);

  std::vector<std::string> frames;
  frames.push_back("1");
  frames.push_back("2");
  frames.push_back("a");
  frames.push_back("b");
  frames.push_back("c");
  frames.push_back("f");
  frames.push_back("g");
  std::string source_frame;
  permuter.addOptionSet(frames, &source_frame);

  std::string target_frame;
  permuter.addOptionSet(frames, &target_frame);

  while  (permuter.step())
  {

    tf2::BufferCore mBC;
    setupTree(mBC, "1_v", eval_time, interpolation_space);

    if (mBC.canTransform(source_frame, target_frame, eval_time))
    {
      geometry_msgs::TransformStamped outpose = mBC.lookupTransform(source_frame, target_frame, eval_time);

      if ((source_frame == "1" || source_frame =="2") && (target_frame =="1" || target_frame == "2"))
        EXPECT_TRUE(check_1_result(outpose, source_frame, target_frame, eval_time, epsilon));
      else if ((source_frame == "a" || source_frame == "b" || source_frame == "c" || source_frame == "f" || source_frame == "g") &&
               (target_frame == "a" || target_frame == "b" || target_frame == "c" || target_frame == "f" || target_frame == "g"))
        EXPECT_TRUE(check_v_result(outpose, source_frame, target_frame, eval_time, epsilon));
      else
        EXPECT_FALSE("Frames unhandled");
    }
    else
      EXPECT_TRUE(((source_frame == "a" || source_frame =="b" || source_frame == "c" || source_frame == "f" || source_frame == "g") &&
                   (target_frame == "1" || target_frame == "2") )
                  ||
                  ((target_frame == "a" || target_frame =="b" || target_frame == "c" || target_frame == "f" || target_frame == "g") &&
                   (source_frame == "1" || source_frame == "2"))
                  );

  }
}

#define CHECK_QUATERNION_NEAR(_q1, _x, _y, _z, _w, _epsilon)                 \
	   {                        											 \
	   btQuaternion q1(_q1.x, _q1.y, _q1.z, _q1.w);                          \
       btQuaternion q2(_x, _y, _z, _w);                                      \
       double angle = q1.angle(q2);                                          \
	   EXPECT_TRUE(fabs(angle) < _epsilon || fabs(angle - M_PI) < _epsilon); \
	   }

#define CHECK_TRANSFORMS_NEAR(_out, _expected, _eps)       																							            	            \
	EXPECT_NEAR(_out.transform.translation.x, _expected.getOrigin().x(), epsilon); 											              			            				\
	EXPECT_NEAR(_out.transform.translation.y, _expected.getOrigin().y(), epsilon); 															                        			\
	EXPECT_NEAR(_out.transform.translation.z, _expected.getOrigin().z(), epsilon); 													            	             				\
	CHECK_QUATERNION_NEAR(_out.transform.rotation, _expected.getRotation().x(), _expected.getRotation().y(), _expected.getRotation().z(), _expected.getRotation().w(), _eps);


// Simple test with compound transform
TEST(BufferCore_lookupTransform, compound_xfm_configuration)
{
	/*
	 * Frames
	 *
	 * root->a
	 *
	 * root->b->c->d
	 *
	 */

	double epsilon = 2e-5; // Larger epsilon for interpolation values

    tf2::BufferCore mBC;

    geometry_msgs::TransformStamped tsa;
    tsa.header.frame_id = "root";
    tsa.child_frame_id  = "a";
    tsa.transform.translation.x = 1.0;
    tsa.transform.translation.y = 1.0;
    tsa.transform.translation.z = 1.0;
    btQuaternion q1;
    q1.setEuler(0.25, .5, .75);
    tsa.transform.rotation.x = q1.x();
    tsa.transform.rotation.y = q1.y();
    tsa.transform.rotation.z = q1.z();
    tsa.transform.rotation.w = q1.w();
    EXPECT_TRUE(mBC.setTransform(tsa, "authority"));

    geometry_msgs::TransformStamped tsb;
    tsb.header.frame_id = "root";
    tsb.child_frame_id  = "b";
    tsb.transform.translation.x = -1.0;
    tsb.transform.translation.y =  0.0;
    tsb.transform.translation.z = -1.0;
    btQuaternion q2;
    q2.setEuler(1.0, 0.25, 0.5);
    tsb.transform.rotation.x = q2.x();
    tsb.transform.rotation.y = q2.y();
    tsb.transform.rotation.z = q2.z();
    tsb.transform.rotation.w = q2.w();
    EXPECT_TRUE(mBC.setTransform(tsb, "authority"));

    geometry_msgs::TransformStamped tsc;
    tsc.header.frame_id = "b";
    tsc.child_frame_id  = "c";
    tsc.transform.translation.x =  0.0;
    tsc.transform.translation.y =  2.0;
    tsc.transform.translation.z =  0.5;
    btQuaternion q3;
    q3.setEuler(0.25, .75, 1.25);
    tsc.transform.rotation.x = q3.x();
    tsc.transform.rotation.y = q3.y();
    tsc.transform.rotation.z = q3.z();
    tsc.transform.rotation.w = q3.w();
    EXPECT_TRUE(mBC.setTransform(tsc, "authority"));

    geometry_msgs::TransformStamped tsd;
    tsd.header.frame_id = "c";
    tsd.child_frame_id  = "d";
    tsd.transform.translation.x =  0.5;
    tsd.transform.translation.y =  -1;
    tsd.transform.translation.z =  1.5;
    btQuaternion q4;
    q4.setEuler(-0.5, 1.0, -.75);
    tsd.transform.rotation.x = q4.x();
    tsd.transform.rotation.y = q4.y();
    tsd.transform.rotation.z = q4.z();
    tsd.transform.rotation.w = q4.w();
    EXPECT_TRUE(mBC.setTransform(tsd, "authority"));

    btTransform ta, tb, tc, td, expected_ab, expected_bc, expected_cb, expected_ac, expected_ba, expected_ca, expected_ad, expected_da, expected_bd, expected_db, expected_rootd, expected_rootc;
    ta.setOrigin(btVector3(1.0,  1.0,  1.0));
    ta.setRotation(q1);
    tb.setOrigin(btVector3(-1.0, 0.0, -1.0));
    tb.setRotation(q2);
    tc.setOrigin(btVector3(0.0, 2.0, 0.5));
    tc.setRotation(q3);
    td.setOrigin(btVector3(0.5, -1, 1.5));
    td.setRotation(q4);


    expected_ab = ta.inverse() * tb;
    expected_ac = ta.inverse() * tb * tc;
    expected_ad = ta.inverse() * tb * tc * td;
    expected_cb = tc.inverse();
    expected_bc = tc;
    expected_bd = tc * td;
    expected_db = expected_bd.inverse();
    expected_ba = tb.inverse() * ta;
    expected_ca = tc.inverse() * tb.inverse() * ta;
    expected_da = td.inverse() * tc.inverse() * tb.inverse() * ta;
    expected_rootd = tb * tc * td;
    expected_rootc = tb * tc;

    // root -> b -> c
    geometry_msgs::TransformStamped out_rootc = mBC.lookupTransform("root", "c", ros::Time());
    CHECK_TRANSFORMS_NEAR(out_rootc, expected_rootc, epsilon);

    // root -> b -> c -> d
    geometry_msgs::TransformStamped out_rootd = mBC.lookupTransform("root", "d", ros::Time());
    CHECK_TRANSFORMS_NEAR(out_rootd, expected_rootd, epsilon);

    // a <- root -> b
    geometry_msgs::TransformStamped out_ab = mBC.lookupTransform("a", "b", ros::Time());
    CHECK_TRANSFORMS_NEAR(out_ab, expected_ab, epsilon);

    geometry_msgs::TransformStamped out_ba = mBC.lookupTransform("b", "a", ros::Time());
    CHECK_TRANSFORMS_NEAR(out_ba, expected_ba, epsilon);

    // a <- root -> b -> c
    geometry_msgs::TransformStamped out_ac = mBC.lookupTransform("a", "c", ros::Time());
    CHECK_TRANSFORMS_NEAR(out_ac, expected_ac, epsilon);

    geometry_msgs::TransformStamped out_ca = mBC.lookupTransform("c", "a", ros::Time());
    CHECK_TRANSFORMS_NEAR(out_ca, expected_ca, epsilon);

    // a <- root -> b -> c -> d
    geometry_msgs::TransformStamped out_ad = mBC.lookupTransform("a", "d", ros::Time());
    CHECK_TRANSFORMS_NEAR(out_ad, expected_ad, epsilon);

    geometry_msgs::TransformStamped out_da = mBC.lookupTransform("d", "a", ros::Time());
    CHECK_TRANSFORMS_NEAR(out_da, expected_da, epsilon);

    // b -> c
    geometry_msgs::TransformStamped out_cb = mBC.lookupTransform("c", "b", ros::Time());
    CHECK_TRANSFORMS_NEAR(out_cb, expected_cb, epsilon);

    geometry_msgs::TransformStamped out_bc = mBC.lookupTransform("b", "c", ros::Time());
    CHECK_TRANSFORMS_NEAR(out_bc, expected_bc, epsilon);

    // b -> c -> d
    geometry_msgs::TransformStamped out_bd = mBC.lookupTransform("b", "d", ros::Time());
    CHECK_TRANSFORMS_NEAR(out_bd, expected_bd, epsilon);

    geometry_msgs::TransformStamped out_db = mBC.lookupTransform("d", "b", ros::Time());
    CHECK_TRANSFORMS_NEAR(out_db, expected_db, epsilon);
}

// Time varying transforms, testing interpolation
TEST(BufferCore_lookupTransform, helix_configuration)
{
	double epsilon = 2e-5; // Larger epsilon for interpolation values

    tf2::BufferCore mBC;

    ros::Time     t0        = ros::Time() + ros::Duration(10);
    ros::Duration step      = ros::Duration(0.05);
    ros::Duration half_step = ros::Duration(0.025);
    ros::Time     t1        = t0 + ros::Duration(5.0);

    /*
     * a->b->c
     *
     * b.z = vel * (t - t0)
     * c.x = cos(theta * (t - t0))
     * c.y = sin(theta * (t - t0))
     *
     * a->d
     *
     * d.z = 2 * cos(theta * (t - t0))
     * a->d transforms are at half-step between a->b->c transforms
     */

    double theta = 0.25;
    double vel   = 1.0;

    for (ros::Time t = t0; t <= t1; t += step)
    {
    	ros::Time t2 = t + half_step;
    	double dt  = (t - t0).toSec();
    	double dt2 = (t2 - t0).toSec();

        geometry_msgs::TransformStamped ts;
        ts.header.frame_id = "a";
        ts.header.stamp    = t;
        ts.child_frame_id  = "b";
        ts.transform.translation.z = vel * dt;
        ts.transform.rotation.w = 1.0;
        EXPECT_TRUE(mBC.setTransform(ts, "authority"));

        geometry_msgs::TransformStamped ts2;
        ts2.header.frame_id = "b";
        ts2.header.stamp    = t;
        ts2.child_frame_id  = "c";
        ts2.transform.translation.x = cos(theta * dt);
        ts2.transform.translation.y = sin(theta * dt);
        btQuaternion q;
        q.setEuler(0,0,theta*dt);
        ts2.transform.rotation.z = q.z();
        ts2.transform.rotation.w = q.w();
        EXPECT_TRUE(mBC.setTransform(ts2, "authority"));

        geometry_msgs::TransformStamped ts3;
        ts3.header.frame_id = "a";
        ts3.header.stamp    = t2;
        ts3.child_frame_id  = "d";
        ts3.transform.translation.z = cos(theta * dt2);
        ts3.transform.rotation.w = 1.0;
        EXPECT_TRUE(mBC.setTransform(ts3, "authority"));
    }


    for (ros::Time t = t0 + half_step; t < t1; t += step)
    {
    	ros::Time t2 = t + half_step;
    	double dt  = (t - t0).toSec();
    	double dt2 = (t2 - t0).toSec();

        geometry_msgs::TransformStamped out_ab = mBC.lookupTransform("a", "b", t);
        EXPECT_NEAR(out_ab.transform.translation.z, vel * dt, epsilon);

        geometry_msgs::TransformStamped out_ac = mBC.lookupTransform("a", "c", t);
        EXPECT_NEAR(out_ac.transform.translation.x, cos(theta * dt), epsilon);
        EXPECT_NEAR(out_ac.transform.translation.y, sin(theta * dt), epsilon);
        EXPECT_NEAR(out_ac.transform.translation.z, vel * dt, 		 epsilon);
        btQuaternion q;
        q.setEuler(0,0,theta*dt);
        CHECK_QUATERNION_NEAR(out_ac.transform.rotation, 0, 0, q.z(), q.w(), epsilon);

        geometry_msgs::TransformStamped out_ad = mBC.lookupTransform("a", "d", t);
        EXPECT_NEAR(out_ad.transform.translation.z, cos(theta * dt), epsilon);

        geometry_msgs::TransformStamped out_cd = mBC.lookupTransform("c", "d", t2);
        EXPECT_NEAR(out_cd.transform.translation.x, -1,           			      epsilon);
        EXPECT_NEAR(out_cd.transform.translation.y,  0,  			              epsilon);
        EXPECT_NEAR(out_cd.transform.translation.z, cos(theta * dt2) - vel * dt2, epsilon);
        btQuaternion mq;
        mq.setEuler(0,0,-theta*dt2);
        CHECK_QUATERNION_NEAR(out_cd.transform.rotation, 0, 0, mq.z(), mq.w(), epsilon);
    }

    // Advanced API
    for (ros::Time t = t0 + half_step; t < t1; t += (step + step))
    {
    	ros::Time t2 = t + step;
    	double dt  = (t - t0).toSec();
    	double dt2 = (t2 - t0).toSec();

        geometry_msgs::TransformStamped out_cd2 = mBC.lookupTransform("c", t, "d", t2, "a");
        EXPECT_NEAR(out_cd2.transform.translation.x, -1,           			      epsilon);
        EXPECT_NEAR(out_cd2.transform.translation.y,  0,  			              epsilon);
        EXPECT_NEAR(out_cd2.transform.translation.z, cos(theta * dt2) - vel * dt, epsilon);
        btQuaternion mq2;
        mq2.setEuler(0,0,-theta*dt);
        CHECK_QUATERNION_NEAR(out_cd2.transform.rotation, 0, 0, mq2.z(), mq2.w(), epsilon);
    }
}


TEST(BufferCore_lookupTransform, ring_45_configuration)
{
  double epsilon = 1e-6;
  rostest::Permuter permuter;

  std::vector<ros::Time> times;
  times.push_back(ros::Time(1.0));
  times.push_back(ros::Time(10.0));
  times.push_back(ros::Time(0.0));
  ros::Time eval_time;
  permuter.addOptionSet(times, &eval_time);

  std::vector<ros::Duration> durations;
  durations.push_back(ros::Duration(1.0));
  durations.push_back(ros::Duration(0.001));
  durations.push_back(ros::Duration(0.1));
  ros::Duration interpolation_space;
  //  permuter.addOptionSet(durations, &interpolation_space);

  std::vector<std::string> frames;
  frames.push_back("a");
  frames.push_back("b");
  frames.push_back("c");
  frames.push_back("d");
  frames.push_back("e");
  frames.push_back("f");
  frames.push_back("g");
  frames.push_back("h");
  frames.push_back("i");
  /*  frames.push_back("inverse_b");
  frames.push_back("inverse_c");
  frames.push_back("inverse_d");
  frames.push_back("inverse_e");
  frames.push_back("inverse_f");
  frames.push_back("inverse_g");
  frames.push_back("inverse_h");
  frames.push_back("inverse_i");*/
  std::string source_frame;
  permuter.addOptionSet(frames, &source_frame);

  std::string target_frame;
  permuter.addOptionSet(frames, &target_frame);

  while  (permuter.step())
  {

    tf2::BufferCore mBC;
    setupTree(mBC, "ring_45", eval_time, interpolation_space);

    geometry_msgs::TransformStamped outpose = mBC.lookupTransform(source_frame, target_frame, eval_time);


    //printf("source_frame %s target_frame %s time %f\n", source_frame.c_str(), target_frame.c_str(), eval_time.toSec());
    EXPECT_EQ(outpose.header.stamp, eval_time);
    EXPECT_EQ(outpose.header.frame_id, source_frame);
    EXPECT_EQ(outpose.child_frame_id, target_frame);



    //Zero distance or all the way
    if (source_frame == target_frame               ||
        (source_frame == "a" && target_frame == "i") ||
        (source_frame == "i" && target_frame == "a") ||
        (source_frame == "a" && target_frame == "inverse_i") ||
        (source_frame == "inverse_i" && target_frame == "a") )
    {
      //printf ("here %s %s\n", source_frame.c_str(), target_frame.c_str());
      EXPECT_NEAR(outpose.transform.translation.x, 0, epsilon);
      EXPECT_NEAR(outpose.transform.translation.y, 0, epsilon);
      EXPECT_NEAR(outpose.transform.translation.z, 0, epsilon);
      EXPECT_NEAR(outpose.transform.rotation.x, 0, epsilon);
      EXPECT_NEAR(outpose.transform.rotation.y, 0, epsilon);
      EXPECT_NEAR(outpose.transform.rotation.z, 0, epsilon);
      EXPECT_NEAR(fabs(outpose.transform.rotation.w), 1, epsilon);
    }
    // Chaining 1
    else if ((source_frame == "a" && target_frame =="b") ||
             (source_frame == "b" && target_frame =="c") ||
             (source_frame == "c" && target_frame =="d") ||
             (source_frame == "d" && target_frame =="e") ||
             (source_frame == "e" && target_frame =="f") ||
             (source_frame == "f" && target_frame =="g") ||
             (source_frame == "g" && target_frame =="h") ||
             (source_frame == "h" && target_frame =="i")
             )
    {
      EXPECT_NEAR(outpose.transform.translation.x, sqrt(2)/2 - 1, epsilon);
      EXPECT_NEAR(outpose.transform.translation.y, sqrt(2)/2 , epsilon);
      EXPECT_NEAR(outpose.transform.translation.z, 0, epsilon);
      EXPECT_NEAR(outpose.transform.rotation.x, 0, epsilon);
      EXPECT_NEAR(outpose.transform.rotation.y, 0, epsilon);
      EXPECT_NEAR(outpose.transform.rotation.z, sin(M_PI/8), epsilon);
      EXPECT_NEAR(outpose.transform.rotation.w, cos(M_PI/8), epsilon);
    }
    // Inverse Chaining 1
    else if ((source_frame == "b" && target_frame =="a") ||
             (source_frame == "c" && target_frame =="b") ||
             (source_frame == "d" && target_frame =="c") ||
             (source_frame == "e" && target_frame =="d") ||
             (source_frame == "f" && target_frame =="e") ||
             (source_frame == "g" && target_frame =="f") ||
             (source_frame == "h" && target_frame =="g") ||
             (source_frame == "i" && target_frame =="h")
             )
    {
      EXPECT_NEAR(outpose.transform.translation.x, sqrt(2)/2 - 1, epsilon);
      EXPECT_NEAR(outpose.transform.translation.y, -sqrt(2)/2 , epsilon);
      EXPECT_NEAR(outpose.transform.translation.z, 0, epsilon);
      EXPECT_NEAR(outpose.transform.rotation.x, 0, epsilon);
      EXPECT_NEAR(outpose.transform.rotation.y, 0, epsilon);
      EXPECT_NEAR(outpose.transform.rotation.z, sin(-M_PI/8), epsilon);
      EXPECT_NEAR(outpose.transform.rotation.w, cos(-M_PI/8), epsilon);
    }
    // Chaining 2
    else if ((source_frame == "a" && target_frame =="c") ||
             (source_frame == "b" && target_frame =="d") ||
             (source_frame == "c" && target_frame =="e") ||
             (source_frame == "d" && target_frame =="f") ||
             (source_frame == "e" && target_frame =="g") ||
             (source_frame == "f" && target_frame =="h") ||
             (source_frame == "g" && target_frame =="i")
             )
    {
      EXPECT_NEAR(outpose.transform.translation.x, -1, epsilon);
      EXPECT_NEAR(outpose.transform.translation.y, 1 , epsilon);
      EXPECT_NEAR(outpose.transform.translation.z, 0, epsilon);
      EXPECT_NEAR(outpose.transform.rotation.x, 0, epsilon);
      EXPECT_NEAR(outpose.transform.rotation.y, 0, epsilon);
      EXPECT_NEAR(outpose.transform.rotation.z, sin(M_PI/4), epsilon);
      EXPECT_NEAR(outpose.transform.rotation.w, cos(M_PI/4), epsilon);
    }
    // Inverse Chaining 2
    else if ((source_frame == "c" && target_frame =="a") ||
             (source_frame == "d" && target_frame =="b") ||
             (source_frame == "e" && target_frame =="c") ||
             (source_frame == "f" && target_frame =="d") ||
             (source_frame == "g" && target_frame =="e") ||
             (source_frame == "h" && target_frame =="f") ||
             (source_frame == "i" && target_frame =="g")
             )
    {
      EXPECT_NEAR(outpose.transform.translation.x, -1, epsilon);
      EXPECT_NEAR(outpose.transform.translation.y, -1 , epsilon);
      EXPECT_NEAR(outpose.transform.translation.z, 0, epsilon);
      EXPECT_NEAR(outpose.transform.rotation.x, 0, epsilon);
      EXPECT_NEAR(outpose.transform.rotation.y, 0, epsilon);
      EXPECT_NEAR(outpose.transform.rotation.z, sin(-M_PI/4), epsilon);
      EXPECT_NEAR(outpose.transform.rotation.w, cos(-M_PI/4), epsilon);
    }
    // Chaining 3
    else if ((source_frame == "a" && target_frame =="d") ||
             (source_frame == "b" && target_frame =="e") ||
             (source_frame == "c" && target_frame =="f") ||
             (source_frame == "d" && target_frame =="g") ||
             (source_frame == "e" && target_frame =="h") ||
             (source_frame == "f" && target_frame =="i")
             )
    {
      EXPECT_NEAR(outpose.transform.translation.x, -1 - sqrt(2)/2, epsilon);
      EXPECT_NEAR(outpose.transform.translation.y, sqrt(2)/2 , epsilon);
      EXPECT_NEAR(outpose.transform.translation.z, 0, epsilon);
      EXPECT_NEAR(outpose.transform.rotation.x, 0, epsilon);
      EXPECT_NEAR(outpose.transform.rotation.y, 0, epsilon);
      EXPECT_NEAR(outpose.transform.rotation.z, sin(M_PI*3/8), epsilon);
      EXPECT_NEAR(outpose.transform.rotation.w, cos(M_PI*3/8), epsilon);
    }
    // Inverse Chaining 3
    else if ((target_frame == "a" && source_frame =="d") ||
             (target_frame == "b" && source_frame =="e") ||
             (target_frame == "c" && source_frame =="f") ||
             (target_frame == "d" && source_frame =="g") ||
             (target_frame == "e" && source_frame =="h") ||
             (target_frame == "f" && source_frame =="i")
             )
    {
      EXPECT_NEAR(outpose.transform.translation.x, -1 - sqrt(2)/2, epsilon);
      EXPECT_NEAR(outpose.transform.translation.y, - sqrt(2)/2 , epsilon);
      EXPECT_NEAR(outpose.transform.translation.z, 0, epsilon);
      EXPECT_NEAR(outpose.transform.rotation.x, 0, epsilon);
      EXPECT_NEAR(outpose.transform.rotation.y, 0, epsilon);
      EXPECT_NEAR(outpose.transform.rotation.z, sin(-M_PI*3/8), epsilon);
      EXPECT_NEAR(outpose.transform.rotation.w, cos(-M_PI*3/8), epsilon);
    }
    // Chaining 4
    else if ((source_frame == "a" && target_frame =="e") ||
             (source_frame == "b" && target_frame =="f") ||
             (source_frame == "c" && target_frame =="g") ||
             (source_frame == "d" && target_frame =="h") ||
             (source_frame == "e" && target_frame =="i")
             )
    {
      EXPECT_NEAR(outpose.transform.translation.x, -2, epsilon);
      EXPECT_NEAR(outpose.transform.translation.y, 0 , epsilon);
      EXPECT_NEAR(outpose.transform.translation.z, 0, epsilon);
      EXPECT_NEAR(outpose.transform.rotation.x, 0, epsilon);
      EXPECT_NEAR(outpose.transform.rotation.y, 0, epsilon);
      EXPECT_NEAR(outpose.transform.rotation.z, sin(M_PI/2), epsilon);
      EXPECT_NEAR(outpose.transform.rotation.w, cos(M_PI/2), epsilon);
    }
    // Inverse Chaining 4
    else if ((target_frame == "a" && source_frame =="e") ||
             (target_frame == "b" && source_frame =="f") ||
             (target_frame == "c" && source_frame =="g") ||
             (target_frame == "d" && source_frame =="h") ||
             (target_frame == "e" && source_frame =="i")
             )
    {
      EXPECT_NEAR(outpose.transform.translation.x, -2, epsilon);
      EXPECT_NEAR(outpose.transform.translation.y, 0 , epsilon);
      EXPECT_NEAR(outpose.transform.translation.z, 0, epsilon);
      EXPECT_NEAR(outpose.transform.rotation.x, 0, epsilon);
      EXPECT_NEAR(outpose.transform.rotation.y, 0, epsilon);
      EXPECT_NEAR(outpose.transform.rotation.z, sin(-M_PI/2), epsilon);
      EXPECT_NEAR(outpose.transform.rotation.w, cos(-M_PI/2), epsilon);
    }
    // Chaining 5
    else if ((source_frame == "a" && target_frame =="f") ||
             (source_frame == "b" && target_frame =="g") ||
             (source_frame == "c" && target_frame =="h") ||
             (source_frame == "d" && target_frame =="i")
             )
    {
      EXPECT_NEAR(outpose.transform.translation.x, -1 - sqrt(2) /2, epsilon);
      EXPECT_NEAR(outpose.transform.translation.y, - sqrt(2) /2 , epsilon);
      EXPECT_NEAR(outpose.transform.translation.z, 0, epsilon);
      EXPECT_NEAR(outpose.transform.rotation.x, 0, epsilon);
      EXPECT_NEAR(outpose.transform.rotation.y, 0, epsilon);
      EXPECT_NEAR(outpose.transform.rotation.z, sin(M_PI*5/8), epsilon);
      EXPECT_NEAR(outpose.transform.rotation.w, cos(M_PI*5/8), epsilon);
    }
    // Inverse Chaining 5
    else if ((target_frame == "a" && source_frame =="f") ||
             (target_frame == "b" && source_frame =="g") ||
             (target_frame == "c" && source_frame =="h") ||
             (target_frame == "d" && source_frame =="i")
             )
    {
      EXPECT_NEAR(outpose.transform.translation.x, -1 - sqrt(2)/2, epsilon);
      EXPECT_NEAR(outpose.transform.translation.y, sqrt(2)/2 , epsilon);
      EXPECT_NEAR(outpose.transform.translation.z, 0, epsilon);
      EXPECT_NEAR(outpose.transform.rotation.x, 0, epsilon);
      EXPECT_NEAR(outpose.transform.rotation.y, 0, epsilon);
      EXPECT_NEAR(outpose.transform.rotation.z, sin(-M_PI*5/8), epsilon);
      EXPECT_NEAR(outpose.transform.rotation.w, cos(-M_PI*5/8), epsilon);
    }
    // Chaining 6
    else if ((source_frame == "a" && target_frame =="g") ||
             (source_frame == "b" && target_frame =="h") ||
             (source_frame == "c" && target_frame =="i")
             )
    {
      EXPECT_NEAR(outpose.transform.translation.x, -1, epsilon);
      EXPECT_NEAR(outpose.transform.translation.y, -1 , epsilon);
      EXPECT_NEAR(outpose.transform.translation.z, 0, epsilon);
      EXPECT_NEAR(outpose.transform.rotation.x, 0, epsilon);
      EXPECT_NEAR(outpose.transform.rotation.y, 0, epsilon);
      EXPECT_NEAR(outpose.transform.rotation.z, sin(M_PI*6/8), epsilon);
      EXPECT_NEAR(outpose.transform.rotation.w, cos(M_PI*6/8), epsilon);
    }
    // Inverse Chaining 6
    else if ((target_frame == "a" && source_frame =="g") ||
             (target_frame == "b" && source_frame =="h") ||
             (target_frame == "c" && source_frame =="i")
             )
    {
      EXPECT_NEAR(outpose.transform.translation.x, -1, epsilon);
      EXPECT_NEAR(outpose.transform.translation.y, 1 , epsilon);
      EXPECT_NEAR(outpose.transform.translation.z, 0, epsilon);
      EXPECT_NEAR(outpose.transform.rotation.x, 0, epsilon);
      EXPECT_NEAR(outpose.transform.rotation.y, 0, epsilon);
      EXPECT_NEAR(outpose.transform.rotation.z, sin(-M_PI*6/8), epsilon);
      EXPECT_NEAR(outpose.transform.rotation.w, cos(-M_PI*6/8), epsilon);
    }
    // Chaining 7
    else if ((source_frame == "a" && target_frame =="h") ||
             (source_frame == "b" && target_frame =="i")
             )
    {
      EXPECT_NEAR(outpose.transform.translation.x, sqrt(2)/2 - 1, epsilon);
      EXPECT_NEAR(outpose.transform.translation.y, -sqrt(2)/2 , epsilon);
      EXPECT_NEAR(outpose.transform.translation.z, 0, epsilon);
      EXPECT_NEAR(outpose.transform.rotation.x, 0, epsilon);
      EXPECT_NEAR(outpose.transform.rotation.y, 0, epsilon);
      EXPECT_NEAR(outpose.transform.rotation.z, sin(M_PI*7/8), epsilon);
      EXPECT_NEAR(outpose.transform.rotation.w, cos(M_PI*7/8), epsilon);
    }
    // Inverse Chaining 7
    else if ((target_frame == "a" && source_frame =="h") ||
             (target_frame == "b" && source_frame =="i")
             )
    {
      EXPECT_NEAR(outpose.transform.translation.x, sqrt(2)/2 - 1, epsilon);
      EXPECT_NEAR(outpose.transform.translation.y, sqrt(2)/2 , epsilon);
      EXPECT_NEAR(outpose.transform.translation.z, 0, epsilon);
      EXPECT_NEAR(outpose.transform.rotation.x, 0, epsilon);
      EXPECT_NEAR(outpose.transform.rotation.y, 0, epsilon);
      EXPECT_NEAR(outpose.transform.rotation.z, sin(-M_PI*7/8), epsilon);
      EXPECT_NEAR(outpose.transform.rotation.w, cos(-M_PI*7/8), epsilon);
    }
    else
    {
      EXPECT_FALSE("Ring_45 testing Shouldn't get here");
      printf("source_frame %s target_frame %s time %f\n", source_frame.c_str(), target_frame.c_str(), eval_time.toSec());
    }

  }
}

TEST(BufferCore_lookupTransform, invalid_arguments)
{
  tf2::BufferCore mBC;

  setupTree(mBC, "i", ros::Time(1.0));

  EXPECT_NO_THROW(mBC.lookupTransform("b", "a", ros::Time()));

  //Empty frame_id
  EXPECT_THROW(mBC.lookupTransform("", "a", ros::Time()), tf2::InvalidArgumentException);
  EXPECT_THROW(mBC.lookupTransform("b", "", ros::Time()), tf2::InvalidArgumentException);

  //frame_id with /
  EXPECT_THROW(mBC.lookupTransform("/b", "a", ros::Time()), tf2::InvalidArgumentException);
  EXPECT_THROW(mBC.lookupTransform("b", "/a", ros::Time()), tf2::InvalidArgumentException);

};

TEST(BufferCore_canTransform, invalid_arguments)
{
  tf2::BufferCore mBC;

  setupTree(mBC, "i", ros::Time(1.0));

  EXPECT_TRUE(mBC.canTransform("b", "a", ros::Time()));


  //Empty frame_id
  EXPECT_FALSE(mBC.canTransform("", "a", ros::Time()));
  EXPECT_FALSE(mBC.canTransform("b", "", ros::Time()));

  //frame_id with /
  EXPECT_FALSE(mBC.canTransform("/b", "a", ros::Time()));
  EXPECT_FALSE(mBC.canTransform("b", "/a", ros::Time()));

};

struct TransformableHelper
{
  TransformableHelper()
  : called(false)
  {}

  void callback(tf2::TransformableRequestHandle request_handle, const std::string& target_frame, const std::string& source_frame,
          ros::Time time, tf2::TransformableResult result)
  {
    called = true;
  }

  bool called;
};

TEST(BufferCore_transformableCallbacks, alreadyTransformable)
{
  tf2::BufferCore b;
  TransformableHelper h;

  geometry_msgs::TransformStamped t;
  t.header.stamp = ros::Time(1);
  t.header.frame_id = "a";
  t.child_frame_id = "b";
  t.transform.rotation.w = 1.0;
  b.setTransform(t, "me");

  tf2::TransformableCallbackHandle cb_handle = b.addTransformableCallback(boost::bind(&TransformableHelper::callback, &h, _1, _2, _3, _4, _5));
  EXPECT_EQ(b.addTransformableRequest(cb_handle, "a", "b", ros::Time(1)), 0U);
}

TEST(BufferCore_transformableCallbacks, waitForNewTransform)
{
  tf2::BufferCore b;
  TransformableHelper h;
  tf2::TransformableCallbackHandle cb_handle = b.addTransformableCallback(boost::bind(&TransformableHelper::callback, &h, _1, _2, _3, _4, _5));
  EXPECT_GT(b.addTransformableRequest(cb_handle, "a", "b", ros::Time(10)), 0U);

  geometry_msgs::TransformStamped t;
  for (uint32_t i = 1; i <= 10; ++i)
  {
    t.header.stamp = ros::Time(i);
    t.header.frame_id = "a";
    t.child_frame_id = "b";
    t.transform.rotation.w = 1.0;
    b.setTransform(t, "me");

    if (i < 10)
    {
      ASSERT_FALSE(h.called);
    }
    else
    {
      ASSERT_TRUE(h.called);
    }
  }
}

TEST(BufferCore_transformableCallbacks, waitForOldTransform)
{
  tf2::BufferCore b;
  TransformableHelper h;
  tf2::TransformableCallbackHandle cb_handle = b.addTransformableCallback(boost::bind(&TransformableHelper::callback, &h, _1, _2, _3, _4, _5));
  EXPECT_GT(b.addTransformableRequest(cb_handle, "a", "b", ros::Time(1)), 0U);

  geometry_msgs::TransformStamped t;
  for (uint32_t i = 10; i > 0; --i)
  {
    t.header.stamp = ros::Time(i);
    t.header.frame_id = "a";
    t.child_frame_id = "b";
    t.transform.rotation.w = 1.0;
    b.setTransform(t, "me");

    if (i > 1)
    {
      ASSERT_FALSE(h.called);
    }
    else
    {
      ASSERT_TRUE(h.called);
    }
  }
}

/*
TEST(tf, Exceptions)
{

 tf::Transformer mTR(true);


 Stamped<btTransform> outpose;

 //connectivity when no data
 EXPECT_FALSE(mTR.canTransform("parent", "me", ros::Time().fromNSec(10000000)));
 try
 {
   mTR.transformPose("parent",Stamped<Pose>(btTransform(btQuaternion(0,0,0,1), btVector3(0,0,0)), ros::Time().fromNSec(10000000) , "me"), outpose);
   EXPECT_FALSE("ConnectivityException Not Thrown");
 }
 catch ( tf::LookupException &ex)
 {
   EXPECT_TRUE("Lookupgh Exception Caught");
 }
 catch (tf::TransformException& ex)
 {
   printf("%s\n",ex.what());
   EXPECT_FALSE("Other Exception Caught");
 }

 mTR.setTransform( StampedTransform(btTransform(btQuaternion(0,0,0,1), btVector3(0,0,0)), ros::Time().fromNSec(100000), "parent", "me"));

 //Extrapolation not valid with one value
 EXPECT_FALSE(mTR.canTransform("parent", "me", ros::Time().fromNSec(200000)));
 try
 {
   mTR.transformPose("parent",Stamped<Pose>(btTransform(btQuaternion(0,0,0,1), btVector3(0,0,0)), ros::Time().fromNSec(200000) , "me"), outpose);
   EXPECT_TRUE("ExtrapolationException Not Thrown");
 }
 catch ( tf::ExtrapolationException &ex)
 {
   EXPECT_TRUE("Extrapolation Exception Caught");
 }
 catch (tf::TransformException& ex)
 {
   printf("%s\n",ex.what());
   EXPECT_FALSE("Other Exception Caught");
 }


 mTR.setTransform( StampedTransform (btTransform(btQuaternion(0,0,0,1), btVector3(0,0,0)), ros::Time().fromNSec(300000), "parent", "me"));

 //NO Extration when Interpolating
 //inverse list
 EXPECT_TRUE(mTR.canTransform("parent", "me", ros::Time().fromNSec(200000)));
 try
 {
   mTR.transformPose("parent",Stamped<Pose>(btTransform(btQuaternion(0,0,0,1), btVector3(0,0,0)), ros::Time().fromNSec(200000) , "me"), outpose);
   EXPECT_TRUE("ExtrapolationException Not Thrown");
 }
 catch ( tf::ExtrapolationException &ex)
 {
   EXPECT_FALSE("Extrapolation Exception Caught");
 }
 catch (tf::TransformException& ex)
 {
   printf("%s\n",ex.what());
   EXPECT_FALSE("Other Exception Caught");
 }



 //forward list
 EXPECT_TRUE(mTR.canTransform("me", "parent", ros::Time().fromNSec(200000)));
 try
 {
   mTR.transformPose("me",Stamped<Pose>(btTransform(btQuaternion(0,0,0,1), btVector3(0,0,0)), ros::Time().fromNSec(200000) , "parent"), outpose);
   EXPECT_TRUE("ExtrapolationException Not Thrown");
 }
 catch ( tf::ExtrapolationException &ex)
 {
   EXPECT_FALSE("Extrapolation Exception Caught");
 }
 catch (tf::TransformException& ex)
 {
   printf("%s\n",ex.what());
   EXPECT_FALSE("Other Exception Caught");
 }


 //Extrapolating backwards
 //inverse list
 EXPECT_FALSE(mTR.canTransform("parent", "me", ros::Time().fromNSec(1000)));
 try
 {
   mTR.transformPose("parent",Stamped<Pose> (btTransform(btQuaternion(0,0,0,1), btVector3(0,0,0)), ros::Time().fromNSec(1000) , "me"), outpose);
   EXPECT_FALSE("ExtrapolationException Not Thrown");
 }
 catch ( tf::ExtrapolationException &ex)
 {
   EXPECT_TRUE("Extrapolation Exception Caught");
 }
 catch (tf::TransformException& ex)
 {
   printf("%s\n",ex.what());
   EXPECT_FALSE("Other Exception Caught");
 }
 //forwards list
 EXPECT_FALSE(mTR.canTransform("me", "parent", ros::Time().fromNSec(1000)));
 try
 {
   mTR.transformPose("me",Stamped<Pose> (btTransform(btQuaternion(0,0,0,1), btVector3(0,0,0)), ros::Time().fromNSec(1000) , "parent"), outpose);
   EXPECT_FALSE("ExtrapolationException Not Thrown");
 }
 catch ( tf::ExtrapolationException &ex)
 {
   EXPECT_TRUE("Extrapolation Exception Caught");
 }
 catch (tf::TransformException& ex)
 {
   printf("%s\n",ex.what());
   EXPECT_FALSE("Other Exception Caught");
 }



 // Test extrapolation inverse and forward linkages FORWARD

 //inverse list
 EXPECT_FALSE(mTR.canTransform("parent", "me", ros::Time().fromNSec(350000)));
 try
 {
   mTR.transformPose("parent", Stamped<Pose> (btTransform(btQuaternion(0,0,0,1), btVector3(0,0,0)), ros::Time().fromNSec(350000) , "me"), outpose);
   EXPECT_FALSE("ExtrapolationException Not Thrown");
 }
 catch ( tf::ExtrapolationException &ex)
 {
   EXPECT_TRUE("Extrapolation Exception Caught");
 }
 catch (tf::TransformException& ex)
 {
   printf("%s\n",ex.what());
   EXPECT_FALSE("Other Exception Caught");
 }

 //forward list
 EXPECT_FALSE(mTR.canTransform("parent", "me", ros::Time().fromNSec(350000)));
 try
 {
   mTR.transformPose("me", Stamped<Pose> (btTransform(btQuaternion(0,0,0,1), btVector3(0,0,0)), ros::Time().fromNSec(350000) , "parent"), outpose);
   EXPECT_FALSE("ExtrapolationException Not Thrown");
 }
 catch ( tf::ExtrapolationException &ex)
 {
   EXPECT_TRUE("Extrapolation Exception Caught");
 }
 catch (tf::TransformException& ex)
 {
   printf("%s\n",ex.what());
   EXPECT_FALSE("Other Exception Caught");
 }




}



TEST(tf, NoExtrapolationExceptionFromParent)
{
  tf::Transformer mTR(true, ros::Duration().fromNSec(1000000));



  mTR.setTransform(  StampedTransform (btTransform(btQuaternion(0,0,0,1), btVector3(0,0,0)), ros::Time().fromNSec(1000), "parent", "a"));
  mTR.setTransform(  StampedTransform (btTransform(btQuaternion(0,0,0,1), btVector3(0,0,0)), ros::Time().fromNSec(10000),  "parent", "a"));


  mTR.setTransform(  StampedTransform (btTransform(btQuaternion(0,0,0,1), btVector3(0,0,0)), ros::Time().fromNSec(1000),  "parent", "b"));
  mTR.setTransform(  StampedTransform (btTransform(btQuaternion(0,0,0,1), btVector3(0,0,0)), ros::Time().fromNSec(10000),  "parent", "b"));

  mTR.setTransform(  StampedTransform (btTransform(btQuaternion(0,0,0,1), btVector3(0,0,0)), ros::Time().fromNSec(1000),  "parent's parent", "parent"));
  mTR.setTransform(  StampedTransform (btTransform(btQuaternion(0,0,0,1), btVector3(0,0,0)), ros::Time().fromNSec(1000),  "parent's parent's parent", "parent's parent"));

  mTR.setTransform(  StampedTransform (btTransform(btQuaternion(0,0,0,1), btVector3(0,0,0)), ros::Time().fromNSec(10000),  "parent's parent", "parent"));
  mTR.setTransform(  StampedTransform (btTransform(btQuaternion(0,0,0,1), btVector3(0,0,0)), ros::Time().fromNSec(10000),  "parent's parent's parent", "parent's parent"));

  Stamped<Point> output;

  try
  {
    mTR.transformPoint( "b", Stamped<Point>(Point(1,1,1), ros::Time().fromNSec(2000), "a"), output);
  }
  catch (ExtrapolationException &ex)
  {
    EXPECT_FALSE("Shouldn't have gotten this exception");
  }



};



TEST(tf, ExtrapolationFromOneValue)
{
  tf::Transformer mTR(true, ros::Duration().fromNSec(1000000));



  mTR.setTransform(  StampedTransform (btTransform(btQuaternion(0,0,0,1), btVector3(0,0,0)), ros::Time().fromNSec(1000),  "parent", "a"));

  mTR.setTransform(  StampedTransform (btTransform(btQuaternion(0,0,0,1), btVector3(0,0,0)), ros::Time().fromNSec(1000),  "parent's parent", "parent"));


  Stamped<Point> output;

  bool excepted = false;
  //Past time
  try
  {
    mTR.transformPoint( "parent", Stamped<Point>(Point(1,1,1), ros::Time().fromNSec(10), "a"), output);
  }
  catch (ExtrapolationException &ex)
  {
    excepted = true;
  }

  EXPECT_TRUE(excepted);

  excepted = false;
  //Future one element
  try
  {
    mTR.transformPoint( "parent", Stamped<Point>(Point(1,1,1), ros::Time().fromNSec(100000), "a"), output);
  }
  catch (ExtrapolationException &ex)
  {
    excepted = true;
  }

  EXPECT_TRUE(excepted);

  //Past multi link
  excepted = false;
  try
  {
    mTR.transformPoint( "parent's parent", Stamped<Point>(Point(1,1,1), ros::Time().fromNSec(1), "a"), output);
  }
  catch (ExtrapolationException &ex)
  {
    excepted = true;
  }

  EXPECT_TRUE(excepted);

  //Future case multi link
  excepted = false;
  try
  {
    mTR.transformPoint( "parent's parent", Stamped<Point>(Point(1,1,1), ros::Time().fromNSec(10000), "a"), output);
  }
  catch (ExtrapolationException &ex)
  {
    excepted = true;
  }

  EXPECT_TRUE(excepted);

  mTR.setTransform(  StampedTransform (btTransform(btQuaternion(0,0,0,1), btVector3(0,0,0)), ros::Time().fromNSec(20000),  "parent", "a"));

  excepted = false;
  try
  {
    mTR.transformPoint( "parent", Stamped<Point>(Point(1,1,1), ros::Time().fromNSec(10000), "a"), output);
  }
  catch (ExtrapolationException &ex)
  {
    excepted = true;
  }

  EXPECT_FALSE(excepted);

};



TEST(tf, getLatestCommonTime)
{
  tf::Transformer mTR(true);
  mTR.setTransform(  StampedTransform (btTransform(btQuaternion(0,0,0,1), btVector3(0,0,0)), ros::Time().fromNSec(1000),  "parent", "a"));
  mTR.setTransform(  StampedTransform (btTransform(btQuaternion(0,0,0,1), btVector3(0,0,0)), ros::Time().fromNSec(2000),  "parent's parent", "parent"));

  //simple case
  ros::Time t;
  mTR.getLatestCommonTime("a", "parent's parent", t, NULL);
  EXPECT_EQ(t, ros::Time().fromNSec(1000));

  //no connection
  EXPECT_EQ(tf::LOOKUP_ERROR, mTR.getLatestCommonTime("a", "not valid", t, NULL));
  EXPECT_EQ(t, ros::Time());

  //testing with update
  mTR.setTransform(  StampedTransform (btTransform(btQuaternion(0,0,0,1), btVector3(0,0,0)), ros::Time().fromNSec(3000),  "parent", "a"));
  mTR.getLatestCommonTime("a", "parent's parent",t, NULL);
  EXPECT_EQ(t, ros::Time().fromNSec(2000));

  //longer chain
  mTR.setTransform(  StampedTransform (btTransform(btQuaternion(0,0,0,1), btVector3(0,0,0)), ros::Time().fromNSec(4000),  "parent", "b"));
  mTR.setTransform(  StampedTransform (btTransform(btQuaternion(0,0,0,1), btVector3(0,0,0)), ros::Time().fromNSec(3000),  "b", "c"));
  mTR.setTransform(  StampedTransform (btTransform(btQuaternion(0,0,0,1), btVector3(0,0,0)), ros::Time().fromNSec(9000),  "c", "d"));
  mTR.setTransform(  StampedTransform (btTransform(btQuaternion(0,0,0,1), btVector3(0,0,0)), ros::Time().fromNSec(5000),  "f", "e"));

  //shared parent
  mTR.getLatestCommonTime("a", "b",t, NULL);
  EXPECT_EQ(t, ros::Time().fromNSec(3000));

  //two degrees
  mTR.getLatestCommonTime("a", "c", t, NULL);
  EXPECT_EQ(t, ros::Time().fromNSec(3000));
  //reversed
  mTR.getLatestCommonTime("c", "a", t, NULL);
  EXPECT_EQ(t, ros::Time().fromNSec(3000));

  //three degrees
  mTR.getLatestCommonTime("a", "d", t, NULL);
  EXPECT_EQ(t, ros::Time().fromNSec(3000));
  //reversed
  mTR.getLatestCommonTime("d", "a", t, NULL);
  EXPECT_EQ(t, ros::Time().fromNSec(3000));

  //disconnected tree
  mTR.getLatestCommonTime("e", "f", t, NULL);
  EXPECT_EQ(t, ros::Time().fromNSec(5000));
  //reversed order
  mTR.getLatestCommonTime("f", "e", t, NULL);
  EXPECT_EQ(t, ros::Time().fromNSec(5000));


  mTR.setExtrapolationLimit(ros::Duration().fromNSec(20000));

  //check timestamps resulting
  tf::Stamped<tf::Point> output, output2;
  try
  {
    mTR.transformPoint( "parent", Stamped<Point>(Point(1,1,1), ros::Time(), "b"), output);
    mTR.transformPoint( "a", ros::Time(),Stamped<Point>(Point(1,1,1), ros::Time(), "b"), "c",  output2);
  }
  catch (tf::TransformException &ex)
  {
    printf("%s\n", ex.what());
    EXPECT_FALSE("Shouldn't get this Exception");
  }

  EXPECT_EQ(output.stamp_, ros::Time().fromNSec(4000));
  EXPECT_EQ(output2.stamp_, ros::Time().fromNSec(3000));


  //zero length lookup zero time
  ros::Time now1 = ros::Time::now();
  ros::Time time_output;
  mTR.getLatestCommonTime("a", "a", time_output, NULL);
  EXPECT_LE(now1.toSec(), time_output.toSec());
  EXPECT_LE(time_output.toSec(), ros::Time::now().toSec());


}

TEST(tf, RepeatedTimes)
{
  Transformer mTR;
  mTR.setTransform(  StampedTransform (btTransform(btQuaternion(1,0,0), btVector3(0,0,0)), ros::Time().fromNSec(4000),  "parent", "b"));
  mTR.setTransform(  StampedTransform (btTransform(btQuaternion(1,1,0), btVector3(0,0,0)), ros::Time().fromNSec(4000),  "parent", "b"));

  tf::StampedTransform  output;
  try{
    mTR.lookupTransform("parent", "b" , ros::Time().fromNSec(4000), output);
    EXPECT_TRUE(!std::isnan(output.getOrigin().x()));
    EXPECT_TRUE(!std::isnan(output.getOrigin().y()));
    EXPECT_TRUE(!std::isnan(output.getOrigin().z()));
    EXPECT_TRUE(!std::isnan(output.getRotation().x()));
    EXPECT_TRUE(!std::isnan(output.getRotation().y()));
    EXPECT_TRUE(!std::isnan(output.getRotation().z()));
    EXPECT_TRUE(!std::isnan(output.getRotation().w()));
  }
  catch (...)
  {
    EXPECT_FALSE("Excetion improperly thrown");
  }


}

TEST(tf, frameExists)
{
  Transformer mTR;

  // test with fully qualified name
  EXPECT_FALSE(mTR.frameExists("/b"));;
  EXPECT_FALSE(mTR.frameExists("/parent"));
  EXPECT_FALSE(mTR.frameExists("/other"));
  EXPECT_FALSE(mTR.frameExists("/frame"));

  //test with resolveping
  EXPECT_FALSE(mTR.frameExists("b"));;
  EXPECT_FALSE(mTR.frameExists("parent"));
  EXPECT_FALSE(mTR.frameExists("other"));
  EXPECT_FALSE(mTR.frameExists("frame"));

  mTR.setTransform(  StampedTransform (btTransform(btQuaternion(1,0,0), btVector3(0,0,0)), ros::Time().fromNSec(4000),  "/parent", "/b"));

  // test with fully qualified name
  EXPECT_TRUE(mTR.frameExists("/b"));
  EXPECT_TRUE(mTR.frameExists("/parent"));
  EXPECT_FALSE(mTR.frameExists("/other"));
  EXPECT_FALSE(mTR.frameExists("/frame"));

  //Test with resolveping
  EXPECT_TRUE(mTR.frameExists("b"));
  EXPECT_TRUE(mTR.frameExists("parent"));
  EXPECT_FALSE(mTR.frameExists("other"));
  EXPECT_FALSE(mTR.frameExists("frame"));

  mTR.setTransform(  StampedTransform (btTransform(btQuaternion(1,1,0), btVector3(0,0,0)), ros::Time().fromNSec(4000),  "/frame", "/other"));

  // test with fully qualified name
  EXPECT_TRUE(mTR.frameExists("/b"));
  EXPECT_TRUE(mTR.frameExists("/parent"));
  EXPECT_TRUE(mTR.frameExists("/other"));
  EXPECT_TRUE(mTR.frameExists("/frame"));

  //Test with resolveping
  EXPECT_TRUE(mTR.frameExists("b"));
  EXPECT_TRUE(mTR.frameExists("parent"));
  EXPECT_TRUE(mTR.frameExists("other"));
  EXPECT_TRUE(mTR.frameExists("frame"));

}

TEST(tf, resolve)
{
  //no prefix
  EXPECT_STREQ("/id", tf::resolve("","id").c_str());
  //prefix w/o /
  EXPECT_STREQ("/asdf/id", tf::resolve("asdf","id").c_str());
  //prefix w /
  EXPECT_STREQ("/asdf/id", tf::resolve("/asdf","id").c_str());
  // frame_id w / -> no prefix
  EXPECT_STREQ("/id", tf::resolve("asdf","/id").c_str());
  // frame_id w / -> no prefix
  EXPECT_STREQ("/id", tf::resolve("/asdf","/id").c_str());

}

TEST(tf, canTransform)
{
  Transformer mTR;

  //confirm zero length list disconnected will return true
  EXPECT_TRUE(mTR.canTransform("some_frame","some_frame", ros::Time()));
  EXPECT_TRUE(mTR.canTransform("some_frame","some_frame", ros::Time::now()));

  //Create a two link tree between times 10 and 20
  for (int i = 10; i < 20; i++)
  {
    mTR.setTransform(  StampedTransform (btTransform(btQuaternion(1,0,0), btVector3(0,0,0)), ros::Time().fromSec(i),  "parent", "child"));
    mTR.setTransform(  StampedTransform (btTransform(btQuaternion(1,0,0), btVector3(0,0,0)), ros::Time().fromSec(i),  "parent", "other_child"));
  }

  // four different timestamps related to tf state
  ros::Time zero_time = ros::Time().fromSec(0);
  ros::Time old_time = ros::Time().fromSec(5);
  ros::Time valid_time = ros::Time().fromSec(15);
  ros::Time future_time = ros::Time().fromSec(25);


  //confirm zero length list disconnected will return true
  EXPECT_TRUE(mTR.canTransform("some_frame","some_frame", zero_time));
  EXPECT_TRUE(mTR.canTransform("some_frame","some_frame", old_time));
  EXPECT_TRUE(mTR.canTransform("some_frame","some_frame", valid_time));
  EXPECT_TRUE(mTR.canTransform("some_frame","some_frame", future_time));

  // Basic API Tests

  //Valid data should pass
  EXPECT_TRUE(mTR.canTransform("child", "parent", valid_time));
  EXPECT_TRUE(mTR.canTransform("child", "other_child", valid_time));

  //zero data should pass
  EXPECT_TRUE(mTR.canTransform("child", "parent", zero_time));
  EXPECT_TRUE(mTR.canTransform("child", "other_child", zero_time));

  //Old data should fail
  EXPECT_FALSE(mTR.canTransform("child", "parent", old_time));
  EXPECT_FALSE(mTR.canTransform("child", "other_child", old_time));

  //Future data should fail
  EXPECT_FALSE(mTR.canTransform("child", "parent", future_time));
  EXPECT_FALSE(mTR.canTransform("child", "other_child", future_time));

  //Same Frame should pass for all times
  EXPECT_TRUE(mTR.canTransform("child", "child", zero_time));
  EXPECT_TRUE(mTR.canTransform("child", "child", old_time));
  EXPECT_TRUE(mTR.canTransform("child", "child", valid_time));
  EXPECT_TRUE(mTR.canTransform("child", "child", future_time));

  // Advanced API Tests

  // Source = Fixed
  //zero data in fixed frame should pass
  EXPECT_TRUE(mTR.canTransform("child", zero_time, "parent", valid_time, "child"));
  EXPECT_TRUE(mTR.canTransform("child", zero_time, "other_child", valid_time, "child"));
  //Old data in fixed frame should pass
  EXPECT_TRUE(mTR.canTransform("child", old_time, "parent", valid_time, "child"));
  EXPECT_TRUE(mTR.canTransform("child", old_time, "other_child", valid_time, "child"));
  //valid data in fixed frame should pass
  EXPECT_TRUE(mTR.canTransform("child", valid_time, "parent", valid_time, "child"));
  EXPECT_TRUE(mTR.canTransform("child", valid_time, "other_child", valid_time, "child"));
  //future data in fixed frame should pass
  EXPECT_TRUE(mTR.canTransform("child", future_time, "parent", valid_time, "child"));
  EXPECT_TRUE(mTR.canTransform("child", future_time, "other_child", valid_time, "child"));

  //transforming through fixed into the past
  EXPECT_FALSE(mTR.canTransform("child", valid_time, "parent", old_time, "child"));
  EXPECT_FALSE(mTR.canTransform("child", valid_time, "other_child", old_time, "child"));
  //transforming through fixed into the future
  EXPECT_FALSE(mTR.canTransform("child", valid_time, "parent", future_time, "child"));
  EXPECT_FALSE(mTR.canTransform("child", valid_time, "other_child", future_time, "child"));

  // Target = Fixed
  //zero data in fixed frame should pass
  EXPECT_TRUE(mTR.canTransform("child", zero_time, "parent", valid_time, "parent"));
  //Old data in fixed frame should pass
  EXPECT_FALSE(mTR.canTransform("child", old_time, "parent", valid_time, "parent"));
  //valid data in fixed frame should pass
  EXPECT_TRUE(mTR.canTransform("child", valid_time, "parent", valid_time, "parent"));
  //future data in fixed frame should pass
  EXPECT_FALSE(mTR.canTransform("child", future_time, "parent", valid_time, "parent"));

  //transforming through fixed into the zero
  EXPECT_TRUE(mTR.canTransform("child", valid_time, "parent", zero_time, "parent"));
  //transforming through fixed into the past
  EXPECT_TRUE(mTR.canTransform("child", valid_time, "parent", old_time, "parent"));
  //transforming through fixed into the valid
  EXPECT_TRUE(mTR.canTransform("child", valid_time, "parent", valid_time, "parent"));
  //transforming through fixed into the future
  EXPECT_TRUE(mTR.canTransform("child", valid_time, "parent", future_time, "parent"));

}

TEST(tf, lookupTransform)
{
  Transformer mTR;
  //Create a two link tree between times 10 and 20
  for (int i = 10; i < 20; i++)
  {
    mTR.setTransform(  StampedTransform (btTransform(btQuaternion(1,0,0), btVector3(0,0,0)), ros::Time().fromSec(i),  "parent", "child"));
    mTR.setTransform(  StampedTransform (btTransform(btQuaternion(1,0,0), btVector3(0,0,0)), ros::Time().fromSec(i),  "parent", "other_child"));
  }

  // four different timestamps related to tf state
  ros::Time zero_time = ros::Time().fromSec(0);
  ros::Time old_time = ros::Time().fromSec(5);
  ros::Time valid_time = ros::Time().fromSec(15);
  ros::Time future_time = ros::Time().fromSec(25);

  //output
  tf::StampedTransform output;

  // Basic API Tests

  try
  {
    //confirm zero length list disconnected will return true
    mTR.lookupTransform("some_frame","some_frame", zero_time, output);
    mTR.lookupTransform("some_frame","some_frame", old_time, output);
    mTR.lookupTransform("some_frame","some_frame", valid_time, output);
    mTR.lookupTransform("some_frame","some_frame", future_time, output);
    mTR.lookupTransform("child","child", future_time, output);
    mTR.lookupTransform("other_child","other_child", future_time, output);

    //Valid data should pass
    mTR.lookupTransform("child", "parent", valid_time, output);
    mTR.lookupTransform("child", "other_child", valid_time, output);

    //zero data should pass
    mTR.lookupTransform("child", "parent", zero_time, output);
    mTR.lookupTransform("child", "other_child", zero_time, output);
  }
  catch (tf::TransformException &ex)
  {
    printf("Exception improperly thrown: %s", ex.what());
    EXPECT_FALSE("Exception thrown");
  }
  try
  {
    //Old data should fail
    mTR.lookupTransform("child", "parent", old_time, output);
    EXPECT_FALSE("Exception should have been thrown");
  }
  catch (tf::TransformException)
  {
    EXPECT_TRUE("Exception Thrown Correctly");
  }
  try {
    //Future data should fail
    mTR.lookupTransform("child", "parent", future_time, output);
    EXPECT_FALSE("Exception should have been thrown");
  }
  catch (tf::TransformException)
  {
    EXPECT_TRUE("Exception Thrown Correctly");
  }

  try {
    //Same Frame should pass for all times
    mTR.lookupTransform("child", "child", zero_time, output);
    mTR.lookupTransform("child", "child", old_time, output);
    mTR.lookupTransform("child", "child", valid_time, output);
    mTR.lookupTransform("child", "child", future_time, output);

    // Advanced API Tests

    // Source = Fixed
    //zero data in fixed frame should pass
    mTR.lookupTransform("child", zero_time, "parent", valid_time, "child", output);
    mTR.lookupTransform("child", zero_time, "other_child", valid_time, "child", output);
    //Old data in fixed frame should pass
    mTR.lookupTransform("child", old_time, "parent", valid_time, "child", output);
    mTR.lookupTransform("child", old_time, "other_child", valid_time, "child", output);
    //valid data in fixed frame should pass
    mTR.lookupTransform("child", valid_time, "parent", valid_time, "child", output);
    mTR.lookupTransform("child", valid_time, "other_child", valid_time, "child", output);
    //future data in fixed frame should pass
    mTR.lookupTransform("child", future_time, "parent", valid_time, "child", output);
    mTR.lookupTransform("child", future_time, "other_child", valid_time, "child", output);
  }
  catch (tf::TransformException &ex)
  {
    printf("Exception improperly thrown: %s", ex.what());
    EXPECT_FALSE("Exception incorrectly thrown");
  }

  try {
    //transforming through fixed into the past
    mTR.lookupTransform("child", valid_time, "parent", old_time, "child", output);
    EXPECT_FALSE("Exception should have been thrown");
  }
  catch (tf::TransformException)
  {
    EXPECT_TRUE("Exception Thrown Correctly");
  }

  try {
    //transforming through fixed into the future
    mTR.lookupTransform("child", valid_time, "parent", future_time, "child", output);
    EXPECT_FALSE("Exception should have been thrown");
  }
  catch (tf::TransformException)
  {
    EXPECT_TRUE("Exception Thrown Correctly");
  }

  try {
    // Target = Fixed
    //zero data in fixed frame should pass
    mTR.lookupTransform("child", zero_time, "parent", valid_time, "parent", output);
    //valid data in fixed frame should pass
    mTR.lookupTransform("child", valid_time, "parent", valid_time, "parent", output);
  }
  catch (tf::TransformException &ex)
  {
    printf("Exception improperly thrown: %s", ex.what());
    EXPECT_FALSE("Exception incorrectly thrown");
  }

  try {
  //Old data in fixed frame should pass
  mTR.lookupTransform("child", old_time, "parent", valid_time, "parent", output);
      EXPECT_FALSE("Exception should have been thrown");
  }
  catch (tf::TransformException)
  {
    EXPECT_TRUE("Exception Thrown Correctly");
  }
  try {
    //future data in fixed frame should pass
    mTR.lookupTransform("child", future_time, "parent", valid_time, "parent", output);
    EXPECT_FALSE("Exception should have been thrown");
  }
  catch (tf::TransformException)
  {
    EXPECT_TRUE("Exception Thrown Correctly");
  }

  try {
    //transforming through fixed into the zero
    mTR.lookupTransform("child", valid_time, "parent", zero_time, "parent", output);
    //transforming through fixed into the past
    mTR.lookupTransform("child", valid_time, "parent", old_time, "parent", output);
    //transforming through fixed into the valid
    mTR.lookupTransform("child", valid_time, "parent", valid_time, "parent", output);
    //transforming through fixed into the future
    mTR.lookupTransform("child", valid_time, "parent", future_time, "parent", output);
  }
  catch (tf::TransformException &ex)
  {
    printf("Exception improperly thrown: %s", ex.what());
    EXPECT_FALSE("Exception improperly thrown");
  }


  //make sure zero goes to now for zero length
  try
  {
    ros::Time now1 = ros::Time::now();

    mTR.lookupTransform("a", "a", ros::Time(),output);
    EXPECT_LE(now1.toSec(), output.stamp_.toSec());
    EXPECT_LE(output.stamp_.toSec(), ros::Time::now().toSec());
  }
  catch (tf::TransformException &ex)
  {
    printf("Exception improperly thrown: %s", ex.what());
    EXPECT_FALSE("Exception improperly thrown");
  }

}


TEST(tf, getFrameStrings)
{
  Transformer mTR;


  mTR.setTransform(  StampedTransform (btTransform(btQuaternion(0,0,0,1), btVector3(0,0,0)), ros::Time().fromNSec(4000),  "/parent", "/b"));
  std::vector <std::string> frames_string;
  mTR.getFrameStrings(frames_string);
  ASSERT_EQ(frames_string.size(), (unsigned)2);
  EXPECT_STREQ(frames_string[0].c_str(), std::string("/b").c_str());
  EXPECT_STREQ(frames_string[1].c_str(), std::string("/parent").c_str());


  mTR.setTransform(  StampedTransform (btTransform(btQuaternion(0,0,0,1), btVector3(0,0,0)), ros::Time().fromNSec(4000),  "/frame", "/other"));

  mTR.getFrameStrings(frames_string);
  ASSERT_EQ(frames_string.size(), (unsigned)4);
  EXPECT_STREQ(frames_string[0].c_str(), std::string("/b").c_str());
  EXPECT_STREQ(frames_string[1].c_str(), std::string("/parent").c_str());
  EXPECT_STREQ(frames_string[2].c_str(), std::string("/other").c_str());
  EXPECT_STREQ(frames_string[3].c_str(), std::string("/frame").c_str());

}

bool expectInvalidQuaternion(tf::Quaternion q)
{
  try
  {
    tf::assertQuaternionValid(q);
    printf("this should have thrown\n");
    return false;
  }
  catch (tf::InvalidArgument &ex)
  {
    return true;
  }
  catch  (...)
  {
    printf("A different type of exception was expected\n");
    return false;
  }
  return false;
}

bool expectValidQuaternion(tf::Quaternion q)
{
  try
  {
    tf::assertQuaternionValid(q);
  }
  catch (tf::TransformException &ex)
  {
    return false;
  }
  return true;
}

bool expectInvalidQuaternion(geometry_msgs::Quaternion q)
{
  try
  {
    tf::assertQuaternionValid(q);
    printf("this should have thrown\n");
    return false;
  }
  catch (tf::InvalidArgument &ex)
  {
    return true;
  }
  catch  (...)
  {
    printf("A different type of exception was expected\n");
    return false;
  }
  return false;
}

bool expectValidQuaternion(geometry_msgs::Quaternion q)
{
  try
  {
    tf::assertQuaternionValid(q);
  }
  catch (tf::TransformException &ex)
  {
    return false;
  }
  return true;
}


TEST(tf, assertQuaternionValid)
{
  tf::Quaternion q(1,0,0,0);
  EXPECT_TRUE(expectValidQuaternion(q));
  q.setX(0);
  EXPECT_TRUE(expectInvalidQuaternion(q));
  q.setY(1);
  EXPECT_TRUE(expectValidQuaternion(q));
  q.setZ(1);
  EXPECT_TRUE(expectInvalidQuaternion(q));
  q.setY(0);
  EXPECT_TRUE(expectValidQuaternion(q));
  q.setW(1);
  EXPECT_TRUE(expectInvalidQuaternion(q));
  q.setZ(0);
  EXPECT_TRUE(expectValidQuaternion(q));
  q.setZ(sqrt(2.0)/2.0);
  EXPECT_TRUE(expectInvalidQuaternion(q));
  q.setW(sqrt(2.0)/2.0);
  EXPECT_TRUE(expectValidQuaternion(q));

  q.setZ(sqrt(2.0)/2.0 + 0.01);
  EXPECT_TRUE(expectInvalidQuaternion(q));

  q.setZ(sqrt(2.0)/2.0 - 0.01);
  EXPECT_TRUE(expectInvalidQuaternion(q));

  EXPECT_THROW(tf::assertQuaternionValid(q), tf::InvalidArgument);
  //    Waiting for gtest 1.1 or later
  //  EXPECT_NO_THROW(tf::assertQuaternionValid(q));
  //q.setX(0);
  //EXPECT_THROW(tf::assertQuaternionValid(q), tf::InvalidArgument);
  //q.setY(1);
  //EXPECT_NO_THROW(tf::assertQuaternionValid(q));

}
TEST(tf, assertQuaternionMsgValid)
{
  geometry_msgs::Quaternion q;
  q.x = 1;//others zeroed to start

  EXPECT_TRUE(expectValidQuaternion(q));
  q.x = 0;
  EXPECT_TRUE(expectInvalidQuaternion(q));
  q.y = 1;
  EXPECT_TRUE(expectValidQuaternion(q));
  q.z = 1;
  EXPECT_TRUE(expectInvalidQuaternion(q));
  q.y = 0;
  EXPECT_TRUE(expectValidQuaternion(q));
  q.w = 1;
  EXPECT_TRUE(expectInvalidQuaternion(q));
  q.z = 0;
  EXPECT_TRUE(expectValidQuaternion(q));
  q.z = sqrt(2.0)/2.0;
  EXPECT_TRUE(expectInvalidQuaternion(q));
  q.w = sqrt(2.0)/2.0;
  EXPECT_TRUE(expectValidQuaternion(q));

  q.z = sqrt(2.0)/2.0 + 0.01;
  EXPECT_TRUE(expectInvalidQuaternion(q));

  q.z = sqrt(2.0)/2.0 - 0.01;
  EXPECT_TRUE(expectInvalidQuaternion(q));


  //    Waiting for gtest 1.1 or later
  //  EXPECT_NO_THROW(tf::assertQuaternionValid(q));
  //q.x = 0);
  //EXPECT_THROW(tf::assertQuaternionValid(q), tf::InvalidArgument);
  //q.y = 1);
  //EXPECT_NO_THROW(tf::assertQuaternionValid(q));

}


TEST(tf2_stamped, OperatorEqualEqual)
{
  btTransform transform0, transform1, transform0a;
  transform0.setIdentity();
  transform0a.setIdentity();
  transform1.setIdentity();
  transform1.setOrigin(btVector3(1, 0, 0));
  tf2::StampedTransform stamped_transform_reference(transform0a, ros::Time(), "frame_id", "child_frame_id");
  tf2::StampedTransform stamped_transform0A(transform0, ros::Time(), "frame_id", "child_frame_id");
  EXPECT_TRUE(stamped_transform0A == stamped_transform_reference); // Equal
  tf2::StampedTransform stamped_transform0B(transform0, ros::Time(), "frame_id_not_equal", "child_frame_id");
  EXPECT_FALSE(stamped_transform0B == stamped_transform_reference); // Different Frame id
  tf2::StampedTransform stamped_transform0C(transform0, ros::Time(1.0), "frame_id", "child_frame_id");
  EXPECT_FALSE(stamped_transform0C == stamped_transform_reference); // Different Time
  tf2::StampedTransform stamped_transform0D(transform0, ros::Time(1.0), "frame_id_not_equal", "child_frame_id");
  EXPECT_FALSE(stamped_transform0D == stamped_transform_reference); // Different frame id and time
  tf2::StampedTransform stamped_transform0E(transform1, ros::Time(), "frame_id_not_equal", "child_frame_id");
  EXPECT_FALSE(stamped_transform0E == stamped_transform_reference); // Different transform, frame id
  tf2::StampedTransform stamped_transform0F(transform1, ros::Time(1.0), "frame_id", "child_frame_id");
  EXPECT_FALSE(stamped_transform0F == stamped_transform_reference); // Different transform, time
  tf2::StampedTransform stamped_transform0G(transform1, ros::Time(1.0), "frame_id_not_equal", "child_frame_id");
  EXPECT_FALSE(stamped_transform0G == stamped_transform_reference); // Different transform, frame id and time
  tf2::StampedTransform stamped_transform0H(transform1, ros::Time(), "frame_id", "child_frame_id");
  EXPECT_FALSE(stamped_transform0H == stamped_transform_reference); // Different transform


  //Different child_frame_id
  tf2::StampedTransform stamped_transform1A(transform0, ros::Time(), "frame_id", "child_frame_id2");
  EXPECT_FALSE(stamped_transform1A == stamped_transform_reference); // Equal
  tf2::StampedTransform stamped_transform1B(transform0, ros::Time(), "frame_id_not_equal", "child_frame_id2");
  EXPECT_FALSE(stamped_transform1B == stamped_transform_reference); // Different Frame id
  tf2::StampedTransform stamped_transform1C(transform0, ros::Time(1.0), "frame_id", "child_frame_id2");
  EXPECT_FALSE(stamped_transform1C == stamped_transform_reference); // Different Time
  tf2::StampedTransform stamped_transform1D(transform0, ros::Time(1.0), "frame_id_not_equal", "child_frame_id2");
  EXPECT_FALSE(stamped_transform1D == stamped_transform_reference); // Different frame id and time
  tf2::StampedTransform stamped_transform1E(transform1, ros::Time(), "frame_id_not_equal", "child_frame_id2");
  EXPECT_FALSE(stamped_transform1E == stamped_transform_reference); // Different transform, frame id
  tf2::StampedTransform stamped_transform1F(transform1, ros::Time(1.0), "frame_id", "child_frame_id2");
  EXPECT_FALSE(stamped_transform1F == stamped_transform_reference); // Different transform, time
  tf2::StampedTransform stamped_transform1G(transform1, ros::Time(1.0), "frame_id_not_equal", "child_frame_id2");
  EXPECT_FALSE(stamped_transform1G == stamped_transform_reference); // Different transform, frame id and time
  tf2::StampedTransform stamped_transform1H(transform1, ros::Time(), "frame_id", "child_frame_id2");
  EXPECT_FALSE(stamped_transform1H == stamped_transform_reference); // Different transform

}

TEST(tf2_stamped, OperatorEqual)
{
  btTransform pose0, pose1, pose0a;
  pose0.setIdentity();
  pose1.setIdentity();
  pose1.setOrigin(btVector3(1, 0, 0));
  tf2::Stamped<btTransform> stamped_pose0(pose0, ros::Time(), "frame_id");
  tf2::Stamped<btTransform> stamped_pose1(pose1, ros::Time(1.0), "frame_id_not_equal");
  EXPECT_FALSE(stamped_pose1 == stamped_pose0);
  stamped_pose1 = stamped_pose0;
  EXPECT_TRUE(stamped_pose1 == stamped_pose0);

}
  */
int main(int argc, char **argv){
  testing::InitGoogleTest(&argc, argv);
  ros::Time::init(); //needed for ros::TIme::now()
  ros::init(argc, argv, "tf_unittest");
  return RUN_ALL_TESTS();
}
