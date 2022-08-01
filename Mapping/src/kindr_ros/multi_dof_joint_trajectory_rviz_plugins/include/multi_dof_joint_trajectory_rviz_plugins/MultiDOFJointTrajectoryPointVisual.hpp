#pragma once

#include <vector>

#include <OGRE/OgreVector3.h>
#include <OGRE/OgreSceneNode.h>
#include <OGRE/OgreSceneManager.h>

#include <rviz/ogre_helpers/arrow.h>
#include <rviz/ogre_helpers/axes.h>
#include <rviz/ogre_helpers/movable_text.h>

#include <rviz/default_plugin/markers/text_view_facing_marker.h>

#include <trajectory_msgs/MultiDOFJointTrajectoryPoint.h>


namespace multi_dof_joint_trajectory_rviz_plugins {

class MultiDOFJointTrajectoryPointVisual
{
public:
  MultiDOFJointTrajectoryPointVisual(
      Ogre::SceneManager* scene_manager,
      Ogre::SceneNode* parent_node,
      const trajectory_msgs::MultiDOFJointTrajectoryPoint& msg,
      bool show_transform_rotation,
      bool show_velocity_linear,
      bool show_velocity_angular,
      bool show_acceleration_linear,
      bool show_acceleration_angular,
      float size_transform_rotation,
      float diameter_arrows,
      float scale_velocity_linear,
      float scale_velocity_angular,
      float scale_acceleration_linear,
      float scale_acceleration_angular,
      float alpha_transform_rotatation,
      const Ogre::ColourValue& color_velocity_linear,
      const Ogre::ColourValue& color_velocity_angular,
      const Ogre::ColourValue& color_acceleration_linear,
      const Ogre::ColourValue& color_acceleration_angular,
      const std::vector<std::string>& captions,
      float font_size,
      bool show_text);
  virtual ~MultiDOFJointTrajectoryPointVisual();

  void setShowTransformRotation(bool visible);
  void setShowVelocityLinear(bool visible);
  void setShowVelocityAngular(bool visible);
  void setShowAccelerationLinear(bool visible);
  void setShowAccelerationAngular(bool visible);

  void setSizeTransformRotation(float size);
  void setDiametersArrows(float diameter);
  void setScaleVelocityLinear(float scale);
  void setScaleVelocityAngular(float scale);
  void setScaleAccelerationLinear(float scale);
  void setScaleAccelerationAngular(float scale);

  void setAlphaTransformRotation(float alpha);
  void setColorVelocityLinear(const Ogre::ColourValue& color);
  void setColorVelocityAngular(const Ogre::ColourValue& color);
  void setColorAccelerationLinear(const Ogre::ColourValue& color);
  void setColorAccelerationAngular(const Ogre::ColourValue& color);

  void setCaptions(const std::vector<std::string>& captions);
  void setFontSize(float font_size);
  void setShowText(bool show_text);


private:
  void updateSizeTransformRotation();
  void updateDiametersArrows();
  void updateScaleVelocityLinear();
  void updateScaleVelocityAngular();
  void updateScaleAccelerationLinear();
  void updateScaleAccelerationAngular();

  void updateAlphaTransformRotation();
  void updateColorVelocityLinear();
  void updateColorVelocityAngular();
  void updateColorAccelerationLinear();
  void updateColorAccelerationAngular();

  void updateCaptions();
  void updateFontSize();
  void updateShowText();

  Ogre::ColourValue getColor(const Ogre::ColourValue& color, bool visible);
  float getCharacterHeight();

  Ogre::SceneManager* scene_manager_;

  std::vector<Ogre::SceneNode*> transforms_position_;
  std::vector<boost::shared_ptr<rviz::Axes>> transforms_rotation_;
  std::vector<boost::shared_ptr<rviz::Arrow>> velocities_linear_;
  std::vector<boost::shared_ptr<rviz::Arrow>> velocities_angular_;
  std::vector<boost::shared_ptr<rviz::Arrow>> accelerations_linear_;
  std::vector<boost::shared_ptr<rviz::Arrow>> accelerations_angular_;
  std::vector<boost::shared_ptr<rviz::MovableText>> texts_;

  std::vector<double> velocities_linear_absolute_;
  std::vector<double> velocities_angular_absolute_;
  std::vector<double> accelerations_linear_absolute_;
  std::vector<double> accelerations_angular_absolute_;

  bool show_transform_rotation_;
  bool show_velocity_linear_;
  bool show_velocity_angular_;
  bool show_acceleration_linear_;
  bool show_acceleration_angular_;

  const float axis_radius_per_size_;

  float size_transform_rotation_;
  float diameter_arrows_;
  float scale_velocity_linear_;
  float scale_velocity_angular_;
  float scale_acceleration_linear_;
  float scale_acceleration_angular_;

  float alpha_transform_rotatation_;
  Ogre::ColourValue color_velocity_linear_;
  Ogre::ColourValue color_velocity_angular_;
  Ogre::ColourValue color_acceleration_linear_;
  Ogre::ColourValue color_acceleration_angular_;

  std::vector<std::string> captions_;
  float font_size_;
  bool show_text_;
};

} // multi_dof_joint_trajectory_rviz_plugins

