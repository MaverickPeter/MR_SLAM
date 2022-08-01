#pragma once

#ifndef Q_MOC_RUN
#include <vector>

#include <boost/circular_buffer.hpp>

#include <rviz/message_filter_display.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>
#endif

#include <OGRE/OgreSceneNode.h>
#include <OGRE/OgreSceneManager.h>

#include <tf/transform_listener.h>

#include <rviz/visualization_manager.h>
#include <rviz/properties/bool_property.h>
#include <rviz/properties/int_property.h>
#include <rviz/properties/float_property.h>
#include <rviz/properties/color_property.h>
#include <rviz/properties/parse_color.h>
#include <rviz/frame_manager.h>

#include "multi_dof_joint_trajectory_rviz_plugins/MultiDOFJointTrajectoryPointConnectionVisual.hpp"
#include "multi_dof_joint_trajectory_rviz_plugins/MultiDOFJointTrajectoryPointVisual.hpp"


namespace multi_dof_joint_trajectory_rviz_plugins {

class MultiDOFJointTrajectoryDisplay: public rviz::MessageFilterDisplay<trajectory_msgs::MultiDOFJointTrajectory>
{
Q_OBJECT
public:
  MultiDOFJointTrajectoryDisplay();
  virtual ~MultiDOFJointTrajectoryDisplay();


protected:
  virtual void onInitialize();
  virtual void reset();

Q_SIGNALS:
  void updateTrajectorySignal();

private Q_SLOTS:
  void setShowConnection();
  void setShowTransformRotation();
  void setShowVelocityLinear();
  void setShowVelocityAngular();
  void setShowAccelerationLinear();
  void setShowAccelerationAngular();

  void setSizeTransformRotation();
  void setDiameterArrows();
  void setScaleVelocityLinear();
  void setScaleVelocityAngular();
  void setScaleAccelerationLinear();
  void setScaleAccelerationAngular();

  void setAlpha();
  void setColorConnection();
  void setColorVelocityLinear();
  void setColorVelocityAngular();
  void setColorAccelerationLinear();
  void setColorAccelerationAngular();

  void setFontSize();
  void setShowText();

  void setHistoryLength();

  void updateTrajectory();

private:
  void processMessage(const trajectory_msgs::MultiDOFJointTrajectory::ConstPtr& msg);

  void updateShowConnection();
  void updateShowTransformRotation();
  void updateShowVelocityLinear();
  void updateShowVelocityAngular();
  void updateShowAccelerationLinear();
  void updateShowAccelerationAngular();

  void updateSizeTransformRotation();
  void updateDiameterArrows();
  void updateScaleVelocityLinear();
  void updateScaleVelocityAngular();
  void updateScaleAccelerationLinear();
  void updateScaleAccelerationAngular();

  void updateColorConnection();
  void updateAlphaTransformRotation();
  void updateColorVelocityLinear();
  void updateColorVelocityAngular();
  void updateColorAccelerationLinear();
  void updateColorAccelerationAngular();

  void updateFontSize();
  void updateShowText();

  boost::circular_buffer<std::vector<boost::shared_ptr<MultiDOFJointTrajectoryPointVisual>>> visuals_points_;
  boost::circular_buffer<std::vector<boost::shared_ptr<MultiDOFJointTrajectoryPointConnectionVisual>>> visuals_connections_;

  rviz::BoolProperty* property_show_connection_;
  rviz::BoolProperty* property_show_transform_rotation_;
  rviz::BoolProperty* property_show_velocity_linear_;
  rviz::BoolProperty* property_show_velocity_angular_;
  rviz::BoolProperty* property_show_acceleration_linear_;
  rviz::BoolProperty* property_show_acceleration_angular_;

  rviz::FloatProperty* property_size_transform_rotation_;
  rviz::FloatProperty* property_diameter_arrows_;
  rviz::FloatProperty* property_scale_velocity_linear_;
  rviz::FloatProperty* property_scale_velocity_angular_;
  rviz::FloatProperty* property_scale_acceleration_linear_;
  rviz::FloatProperty* property_scale_acceleration_angular_;

  rviz::ColorProperty* property_color_connection_;
  rviz::ColorProperty* property_color_velocity_linear_;
  rviz::ColorProperty* property_color_velocity_angular_;
  rviz::ColorProperty* property_color_acceleration_linear_;
  rviz::ColorProperty* property_color_acceleration_angular_;

  rviz::FloatProperty* property_alpha_;

  rviz::FloatProperty* property_font_size_;
  rviz::BoolProperty* property_show_text_;

  rviz::IntProperty* property_history_length_;

  bool show_connection_;
  bool show_transform_rotation_;
  bool show_velocity_linear_;
  bool show_velocity_angular_;
  bool show_acceleration_linear_;
  bool show_acceleration_angular_;

  float size_transform_rotation_;
  float diameter_arrows_;
  float scale_velocity_linear_;
  float scale_velocity_angular_;
  float scale_acceleration_linear_;
  float scale_acceleration_angular_;

  float alpha_;
  Ogre::ColourValue color_connection_;
  Ogre::ColourValue color_velocity_linear_;
  Ogre::ColourValue color_velocity_angular_;
  Ogre::ColourValue color_acceleration_linear_;
  Ogre::ColourValue color_acceleration_angular_;

  float font_size_;
  bool show_text_;

  trajectory_msgs::MultiDOFJointTrajectory::ConstPtr current_trajectory_;
};

} // multi_dof_joint_trajectory_rviz_plugins
