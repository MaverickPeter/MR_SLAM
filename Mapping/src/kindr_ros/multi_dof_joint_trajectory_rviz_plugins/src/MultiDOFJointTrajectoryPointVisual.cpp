#include <cassert>

#include "multi_dof_joint_trajectory_rviz_plugins/MultiDOFJointTrajectoryPointVisual.hpp"


namespace multi_dof_joint_trajectory_rviz_plugins {

MultiDOFJointTrajectoryPointVisual::MultiDOFJointTrajectoryPointVisual(
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
    bool show_text)
: scene_manager_(scene_manager),
  axis_radius_per_size_(0.1),
  show_transform_rotation_(show_transform_rotation),
  show_velocity_linear_(show_velocity_linear),
  show_velocity_angular_(show_velocity_angular),
  show_acceleration_linear_(show_acceleration_linear),
  show_acceleration_angular_(show_acceleration_angular),
  size_transform_rotation_(size_transform_rotation),
  diameter_arrows_(diameter_arrows),
  scale_velocity_linear_(scale_velocity_linear),
  scale_velocity_angular_(scale_velocity_angular),
  scale_acceleration_linear_(scale_acceleration_linear),
  scale_acceleration_angular_(scale_acceleration_angular),
  alpha_transform_rotatation_(alpha_transform_rotatation),
  color_velocity_linear_(color_velocity_linear),
  color_velocity_angular_(color_velocity_angular),
  color_acceleration_linear_(color_acceleration_linear),
  color_acceleration_angular_(color_acceleration_angular),
  captions_(captions),
  font_size_(font_size),
  show_text_(show_text)
{
  // check vector lengths
  assert((msg.transforms.size()    == captions.size())                                  && "ERROR: MultiDOFJointTrajectoryPointVisual: captions and transforms vectors do not have the same length.");
  assert((msg.velocities.size()    == captions.size() || msg.velocities.size()    == 0) && "ERROR: MultiDOFJointTrajectoryPointVisual: velocity vector has invalid length.");
  assert((msg.accelerations.size() == captions.size() || msg.accelerations.size() == 0) && "ERROR: MultiDOFJointTrajectoryPointVisual: acceleration vector has invalid length.");

  const bool message_contains_velocities    = (msg.velocities.size()    == msg.transforms.size());
  const bool message_contains_accelerations = (msg.accelerations.size() == msg.transforms.size());

  // loop through all joints
  for (unsigned int i = 0; i < msg.transforms.size(); i++)
  {
    // transform position
    transforms_position_.push_back(parent_node->createChildSceneNode());
    transforms_position_.back()->setPosition(msg.transforms[i].translation.x, msg.transforms[i].translation.y, msg.transforms[i].translation.z);

    // transform rotation
    const Ogre::Quaternion orientation(msg.transforms[i].rotation.w, msg.transforms[i].rotation.x, msg.transforms[i].rotation.y, msg.transforms[i].rotation.z);
    transforms_rotation_.push_back(boost::shared_ptr<rviz::Axes>(new rviz::Axes(scene_manager_, transforms_position_.back(), size_transform_rotation, axis_radius_per_size_*size_transform_rotation)));
    transforms_rotation_.back()->setOrientation(orientation);

    // define common variables
    Ogre::Vector3 vector;

    if (message_contains_velocities)
    {
      // velocity linear
      vector = Ogre::Vector3(msg.velocities[i].linear.x, msg.velocities[i].linear.y, msg.velocities[i].linear.z);
      velocities_linear_absolute_.push_back(vector.length());
      velocities_linear_.push_back(boost::shared_ptr<rviz::Arrow>(new rviz::Arrow(
          scene_manager,
          transforms_position_.back(),
          0.8*scale_velocity_linear_*velocities_linear_absolute_.back(),
          diameter_arrows_,
          0.2*scale_velocity_linear_*velocities_linear_absolute_.back(),
          2.0*diameter_arrows_)));
      velocities_linear_.back()->setDirection(vector);

      // velocity angular
      vector = Ogre::Vector3(msg.velocities[i].angular.x, msg.velocities[i].angular.y, msg.velocities[i].angular.z);
      velocities_angular_absolute_.push_back(vector.length());
      velocities_angular_.push_back(boost::shared_ptr<rviz::Arrow>(new rviz::Arrow(
          scene_manager,
          transforms_position_.back(),
          0.8*scale_velocity_angular_*velocities_angular_absolute_.back(),
          diameter_arrows_,
          0.2*scale_velocity_angular_*velocities_angular_absolute_.back(),
          2.0*diameter_arrows_)));
      velocities_angular_.back()->setDirection(vector);
    }

    if (message_contains_accelerations)
    {
      // acceleration linear
      vector = Ogre::Vector3(msg.accelerations[i].linear.x, msg.accelerations[i].linear.y, msg.accelerations[i].linear.z);
      accelerations_linear_absolute_.push_back(vector.length());
      accelerations_linear_.push_back(boost::shared_ptr<rviz::Arrow>(new rviz::Arrow(
          scene_manager,
          transforms_position_.back(),
          0.8*scale_acceleration_linear_*accelerations_linear_absolute_.back(),
          diameter_arrows_,
          0.2*scale_acceleration_linear_*accelerations_linear_absolute_.back(),
          2.0*diameter_arrows_)));
      accelerations_linear_.back()->setDirection(vector);

      // acceleration angular
      vector = Ogre::Vector3(msg.accelerations[i].angular.x, msg.accelerations[i].angular.y, msg.accelerations[i].angular.z);
      accelerations_angular_absolute_.push_back(vector.length());
      accelerations_angular_.push_back(boost::shared_ptr<rviz::Arrow>(new rviz::Arrow(
          scene_manager,
          transforms_position_.back(),
          0.8*scale_acceleration_angular_*accelerations_angular_absolute_.back(),
          diameter_arrows_,
          0.2*scale_acceleration_angular_*accelerations_angular_absolute_.back(),
          2.0*diameter_arrows_)));
      accelerations_angular_.back()->setDirection(vector);
    }

    // text
    auto text = boost::shared_ptr<rviz::MovableText>(new rviz::MovableText(captions_[i]));
    text->setCharacterHeight(getCharacterHeight());
    texts_.push_back(text);
    texts_.back()->setTextAlignment(rviz::MovableText::H_CENTER, rviz::MovableText::V_BELOW);
    transforms_position_.back()->attachObject(texts_.back().get());
  }

  // update all colors
  updateAlphaTransformRotation();
  updateColorVelocityLinear();
  updateColorVelocityAngular();
  updateColorAccelerationLinear();
  updateColorAccelerationAngular();
}

MultiDOFJointTrajectoryPointVisual::~MultiDOFJointTrajectoryPointVisual()
{
  for (unsigned int i = 0; i < transforms_position_.size(); i++)
  {
    scene_manager_->destroySceneNode(transforms_position_[i]);
  }
}

void MultiDOFJointTrajectoryPointVisual::setShowTransformRotation(bool visible)
{
  show_transform_rotation_ = visible;
  updateAlphaTransformRotation();
}

void MultiDOFJointTrajectoryPointVisual::setShowVelocityLinear(bool visible)
{
  show_velocity_linear_ = visible;
  updateColorVelocityLinear();
}

void MultiDOFJointTrajectoryPointVisual::setShowVelocityAngular(bool visible)
{
  show_velocity_angular_ = visible;
  updateColorVelocityAngular();
}

void MultiDOFJointTrajectoryPointVisual::setShowAccelerationLinear(bool visible)
{
  show_acceleration_linear_ = visible;
  updateColorAccelerationLinear();
}

void MultiDOFJointTrajectoryPointVisual::setShowAccelerationAngular(bool visible)
{
  show_acceleration_angular_ = visible;
  updateColorAccelerationAngular();
}

void MultiDOFJointTrajectoryPointVisual::setSizeTransformRotation(float size)
{
  size_transform_rotation_ = size;
  updateSizeTransformRotation();
}

void MultiDOFJointTrajectoryPointVisual::setDiametersArrows(float diameter)
{
  diameter_arrows_ = diameter;
  updateDiametersArrows();
}

void MultiDOFJointTrajectoryPointVisual::setScaleVelocityLinear(float scale)
{
  scale_velocity_linear_ = scale;
  updateScaleVelocityLinear();
}

void MultiDOFJointTrajectoryPointVisual::setScaleVelocityAngular(float scale)
{
  scale_velocity_angular_ = scale;
  updateScaleVelocityAngular();
}

void MultiDOFJointTrajectoryPointVisual::setScaleAccelerationLinear(float scale)
{
  scale_acceleration_linear_ = scale;
  updateScaleAccelerationLinear();
}

void MultiDOFJointTrajectoryPointVisual::setScaleAccelerationAngular(float scale)
{
  scale_acceleration_angular_ = scale;
  updateScaleAccelerationAngular();
}

void MultiDOFJointTrajectoryPointVisual::setAlphaTransformRotation(float alpha)
{
  alpha_transform_rotatation_ = alpha;
  updateAlphaTransformRotation();
}

void MultiDOFJointTrajectoryPointVisual::setColorVelocityLinear(const Ogre::ColourValue& color)
{
  color_velocity_linear_ = color;
  updateColorVelocityLinear();
}

void MultiDOFJointTrajectoryPointVisual::setColorVelocityAngular(const Ogre::ColourValue& color)
{
  color_velocity_angular_ = color;
  updateColorVelocityAngular();
}

void MultiDOFJointTrajectoryPointVisual::setColorAccelerationLinear(const Ogre::ColourValue& color)
{
  color_acceleration_linear_ = color;
  updateColorAccelerationLinear();
}

void MultiDOFJointTrajectoryPointVisual::setColorAccelerationAngular(const Ogre::ColourValue& color)
{
  color_acceleration_angular_ = color;
  updateColorAccelerationAngular();
}

void MultiDOFJointTrajectoryPointVisual::setCaptions(const std::vector<std::string>& captions)
{
  // check vector lengths
  assert(captions_.size() == captions.size() && "ERROR: MultiDOFJointTrajectoryPointVisual: old and new captions vectors do not have the same length.");

  captions_ = captions;
  updateCaptions();
}

void MultiDOFJointTrajectoryPointVisual::setFontSize(float font_size)
{
  font_size_ = font_size;
  updateFontSize();
}

void MultiDOFJointTrajectoryPointVisual::setShowText(bool show_text)
{
  show_text_ = show_text;
  updateShowText();
}

void MultiDOFJointTrajectoryPointVisual::updateSizeTransformRotation()
{
  for (unsigned int i = 0; i < transforms_rotation_.size(); i++)
  {
    transforms_rotation_[i]->set(size_transform_rotation_, axis_radius_per_size_*size_transform_rotation_);
  }
  updateAlphaTransformRotation(); // colors are reset by set()
}

void MultiDOFJointTrajectoryPointVisual::updateDiametersArrows()
{
  updateScaleVelocityLinear();
  updateScaleVelocityAngular();
  updateScaleAccelerationLinear();
  updateScaleAccelerationAngular();
}

void MultiDOFJointTrajectoryPointVisual::updateScaleVelocityLinear()
{
  for (unsigned int i = 0; i < velocities_linear_.size(); i++)
  {
    velocities_linear_[i]->set(
        0.8*scale_velocity_linear_*velocities_linear_absolute_[i],
        diameter_arrows_,
        0.2*scale_velocity_linear_*velocities_linear_absolute_[i],
        2.0*diameter_arrows_);
  }
}

void MultiDOFJointTrajectoryPointVisual::updateScaleVelocityAngular()
{
  for (unsigned int i = 0; i < velocities_angular_.size(); i++)
  {
    velocities_angular_[i]->set(
        0.8*scale_velocity_angular_*velocities_angular_absolute_[i],
        diameter_arrows_,
        0.2*scale_velocity_angular_*velocities_angular_absolute_[i],
        2.0*diameter_arrows_);
  }
}

void MultiDOFJointTrajectoryPointVisual::updateScaleAccelerationLinear()
{
  for (unsigned int i = 0; i < accelerations_linear_.size(); i++)
  {
    accelerations_linear_[i]->set(
        0.8*scale_acceleration_linear_*accelerations_linear_absolute_[i],
        diameter_arrows_,
        0.2*scale_acceleration_linear_*accelerations_linear_absolute_[i],
        2.0*diameter_arrows_);
  }
}

void MultiDOFJointTrajectoryPointVisual::updateScaleAccelerationAngular()
{
  for (unsigned int i = 0; i < accelerations_angular_.size(); i++)
  {
    accelerations_angular_[i]->set(
        0.8*scale_acceleration_angular_*accelerations_angular_absolute_[i],
        diameter_arrows_,
        0.2*scale_acceleration_angular_*accelerations_angular_absolute_[i],
        2.0*diameter_arrows_);
  }
}

void MultiDOFJointTrajectoryPointVisual::updateAlphaTransformRotation()
{
  Ogre::ColourValue color;
  for (unsigned int i = 0; i < transforms_rotation_.size(); i++)
  {
    color = transforms_rotation_[i]->getDefaultXColor();
    color.a = show_transform_rotation_*alpha_transform_rotatation_;
    transforms_rotation_[i]->setXColor(color);
    color = transforms_rotation_[i]->getDefaultYColor();
    color.a = show_transform_rotation_*alpha_transform_rotatation_;
    transforms_rotation_[i]->setYColor(color);
    color = transforms_rotation_[i]->getDefaultZColor();
    color.a = show_transform_rotation_*alpha_transform_rotatation_;
    transforms_rotation_[i]->setZColor(color);
  }
}

void MultiDOFJointTrajectoryPointVisual::updateColorVelocityLinear()
{
  for (unsigned int i = 0; i < velocities_linear_.size(); i++)
  {
    velocities_linear_[i]->setColor(getColor(color_velocity_linear_, show_velocity_linear_));
  }
}

void MultiDOFJointTrajectoryPointVisual::updateColorVelocityAngular()
{
  for (unsigned int i = 0; i < velocities_angular_.size(); i++)
  {
    velocities_angular_[i]->setColor(getColor(color_velocity_angular_, show_velocity_angular_));
  }
}

void MultiDOFJointTrajectoryPointVisual::updateColorAccelerationLinear()
{
  for (unsigned int i = 0; i < accelerations_linear_.size(); i++)
  {
    accelerations_linear_[i]->setColor(getColor(color_acceleration_linear_, show_acceleration_linear_));
  }
}

void MultiDOFJointTrajectoryPointVisual::updateColorAccelerationAngular()
{
  for (unsigned int i = 0; i < accelerations_angular_.size(); i++)
  {
    accelerations_angular_[i]->setColor(getColor(color_acceleration_angular_, show_acceleration_angular_));
  }
}

void MultiDOFJointTrajectoryPointVisual::updateCaptions()
{
  for (unsigned int i = 0; i < texts_.size(); i++)
  {
    texts_[i]->setCaption(captions_[i]);
  }
}

void MultiDOFJointTrajectoryPointVisual::updateFontSize()
{
  double character_height = getCharacterHeight();
  for (unsigned int i = 0; i < texts_.size(); i++)
  {
    texts_[i]->setCharacterHeight(character_height);
  }
}

void MultiDOFJointTrajectoryPointVisual::updateShowText()
{
  double character_height = getCharacterHeight();
  for (unsigned int i = 0; i < texts_.size(); i++)
  {
    texts_[i]->setCharacterHeight(character_height);
  }
}

Ogre::ColourValue MultiDOFJointTrajectoryPointVisual::getColor(const Ogre::ColourValue& color, bool visible)
{
  if (!visible)
  {
    Ogre::ColourValue color_invisible = color;
    color_invisible.a = 0;
    return color_invisible;
  }
  else
  {
    return color;
  }
}

float MultiDOFJointTrajectoryPointVisual::getCharacterHeight()
{
  if (show_text_)
  {
    return font_size_;
  }
  else
  {
    return 0;
  }
}

} // multi_dof_joint_trajectory_rviz_plugins

