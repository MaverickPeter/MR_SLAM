/*
 * Copyright (c) 2012, Willow Garage, Inc.
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

#include <OGRE/OgreVector3.h>
#include <OGRE/OgreSceneNode.h>
#include <OGRE/OgreSceneManager.h>

#include <rviz/ogre_helpers/arrow.h>
#include <rviz/ogre_helpers/billboard_line.h>
#include <rviz/ogre_helpers/movable_text.h>

#include "kindr_rviz_plugins/VectorAtPositionVisual.hpp"


namespace kindr_rviz_plugins {

VectorAtPositionVisual::VectorAtPositionVisual(Ogre::SceneManager* scene_manager, Ogre::SceneNode* parent_node)
: vector_(Ogre::Vector3::UNIT_Z),
  length_(0.0),
  lengthScalingFactor_(1.0),
  widthScalingFactor_(1.0),
  showText_(true),
  showTorque_(false),
  color_(Ogre::ColourValue(0,0,0,1))
{
  scene_manager_ = scene_manager;

  // Ogre::SceneNode s form a tree, with each node storing the
  // transform (position and orientation) of itself relative to its
  // parent. Ogre does the math of combining those transforms when it
  // is time to render.
  //
  // Here we create a node to store the pose of the VectorAtPosition's header frame
  // relative to the RViz fixed frame.
  scene_node_frame_ = parent_node->createChildSceneNode();
  scene_node_arrow_ = scene_node_frame_->createChildSceneNode();
  scene_node_circle_ = scene_node_frame_->createChildSceneNode();
  scene_node_text_ = scene_node_arrow_->createChildSceneNode();
  scene_node_text_->setPosition(Ogre::Vector3(0, 0, -0.05));

  // We create the arrow object within the frame node so that we can
  // set its position and direction relative to its header frame.
  arrow_.reset(new rviz::Arrow(scene_manager_, scene_node_arrow_, 0.8f, 0.07f, 0.2f, 0.15f));
  circle_.reset(new rviz::BillboardLine(scene_manager_, scene_node_circle_));
  circleArrow_.reset(new rviz::Arrow( scene_manager_, scene_node_circle_ ));
}

VectorAtPositionVisual::~VectorAtPositionVisual()
{
  // Destroy the frame node since we don't need it anymore.
  scene_manager_->destroySceneNode(scene_node_frame_);
  scene_manager_->destroySceneNode(scene_node_arrow_);
  scene_manager_->destroySceneNode(scene_node_circle_);
  scene_manager_->destroySceneNode(scene_node_text_);
}

void VectorAtPositionVisual::setMessage(const kindr_msgs::VectorAtPosition::ConstPtr& msg)
{
  // Check if type is torque
  showTorque_ = msg->type == msg->TYPE_TORQUE;

  // Set the name of the arrow.
  name_ = msg->name;

  // Convert the geometry_msgs::Vector3 to an Ogre::Vector3. Set torque orientation.
  vector_ = Ogre::Vector3(msg->vector.x, msg->vector.y, msg->vector.z);

  // Set the position of the arrow.
  arrow_->setPosition(Ogre::Vector3(0,0,0));

  // Set the orientation of the arrow to match the direction of the vector.
  arrow_->setDirection(vector_);

  // Find the magnitude of the vector.
  length_ = vector_.length();

  // update the scaling
  updateScaling();

  // update the color
  updateColor();

  // update the text
  updateText();

  // show circle arrow only if type is torque
  scene_node_circle_->setVisible(showTorque_);

}

void VectorAtPositionVisual::setArrowPosition(const Ogre::Vector3& position)
{
  scene_node_frame_->setPosition(position);
}

void VectorAtPositionVisual::setArrowOrientation(const Ogre::Quaternion& orientation)
{
  scene_node_frame_->setOrientation(orientation);
}

void VectorAtPositionVisual::setScalingFactors(float lengthScalingFactor, float widthScalingFactor)
{
  lengthScalingFactor_ = lengthScalingFactor;
  widthScalingFactor_ = widthScalingFactor;
  updateScaling();
}

void VectorAtPositionVisual::setColor(const Ogre::ColourValue& color)
{
  color_ = color;
  updateColor();
}

void VectorAtPositionVisual::setShowText(bool showText)
{
  showText_ = showText;
  updateText();
}

void VectorAtPositionVisual::updateScaling()
{
  // Scale the arrow's thickness in each dimension along with its length and scaling factors.
  arrow_->setScale(Ogre::Vector3(lengthScalingFactor_ * length_, widthScalingFactor_, widthScalingFactor_));

  // Scale the circle arrow
  if(showTorque_) {
    // Get orientation of arrow in scene
    Ogre::Vector3 axis_z(0,0,1);
    Ogre::Quaternion orientation = axis_z.getRotationTo(lengthScalingFactor_ * vector_);
    if ( orientation.isNaN() ) {orientation = Ogre::Quaternion::IDENTITY;}

    // Set position and direction of the arrowhead
    circleArrow_->set(0, widthScalingFactor_*0.1, widthScalingFactor_*0.1*1.0, widthScalingFactor_*0.1*2.0);
    circleArrow_->setPosition(orientation * Ogre::Vector3(std::fabs(lengthScalingFactor_)*length_/4, 0, std::fabs(lengthScalingFactor_)*length_/2));
    circleArrow_->setDirection(orientation * Ogre::Vector3(0,1,0));

    // Add circular arrow points
    circle_->clear();
    circle_->setLineWidth(widthScalingFactor_*0.05);
    for (int i = 4; i <= 32; i++) {
        Ogre::Vector3 point = Ogre::Vector3((std::fabs(lengthScalingFactor_)*length_/4)*cos(i*2*M_PI/32),
                                            (std::fabs(lengthScalingFactor_)*length_/4)*sin(i*2*M_PI/32),
                                             std::fabs(lengthScalingFactor_)*length_/2);
        circle_->addPoint(orientation * point);
    }
  }
}

void VectorAtPositionVisual::updateColor()
{
  arrow_->setColor(color_);
  if(showTorque_) {
    circle_->setColor(color_.r, color_.g, color_.b, color_.a);
    circleArrow_->setColor(color_.r, color_.g, color_.b, color_.a);
  }
}

void VectorAtPositionVisual::updateText()
{
  // add description text if available
  const std::string textString = showText_? name_ : "";
  text_.reset(new rviz::MovableText(textString));
  text_->setCharacterHeight(0.1);
  text_->setTextAlignment(rviz::MovableText::H_CENTER, rviz::MovableText::V_BELOW);
  scene_node_text_->attachObject(text_.get());
}

} // kindr_rviz_plugins
