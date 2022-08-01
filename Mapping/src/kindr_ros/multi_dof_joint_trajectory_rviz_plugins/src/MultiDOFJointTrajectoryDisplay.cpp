#include <iomanip>

#include "multi_dof_joint_trajectory_rviz_plugins/MultiDOFJointTrajectoryDisplay.hpp"


namespace multi_dof_joint_trajectory_rviz_plugins {

MultiDOFJointTrajectoryDisplay::MultiDOFJointTrajectoryDisplay()
: show_connection_(true),
  show_transform_rotation_(true),
  show_velocity_linear_(true),
  show_velocity_angular_(true),
  show_acceleration_linear_(true),
  show_acceleration_angular_(true),
  size_transform_rotation_(0.2),
  diameter_arrows_(0.05),
  scale_velocity_linear_(1.0),
  scale_velocity_angular_(1.0),
  scale_acceleration_linear_(1.0),
  scale_acceleration_angular_(1.0),
  alpha_(1.0),
  color_connection_(Ogre::ColourValue(1.0, 1.0, 1.0, alpha_)), // white
  color_velocity_linear_(Ogre::ColourValue(0.4, 0.0, 0.0, alpha_)), // dark red
  color_velocity_angular_(Ogre::ColourValue(0.0, 0.4, 0.0, alpha_)), // dark green
  color_acceleration_linear_(Ogre::ColourValue(1.0, 1.0, 0.0, alpha_)), // yellow
  color_acceleration_angular_(Ogre::ColourValue(0.75, 0.0, 0.75, alpha_)), // purple
  font_size_(0.05),
  show_text_(true)
{
  connect(this, SIGNAL(updateTrajectorySignal()), this, SLOT(updateTrajectory()));

  property_show_connection_ = new rviz::BoolProperty(
      "Show Connections", show_connection_,
      "Enable or disable connections rendering.",
      this, SLOT(setShowConnection()));

  property_show_transform_rotation_ = new rviz::BoolProperty(
      "Show Transform Rotation", show_transform_rotation_,
      "Enable or disable rotation transforms rendering.",
      this, SLOT(setShowTransformRotation()));

  property_show_velocity_linear_ = new rviz::BoolProperty(
      "Show Velocity Linear", show_velocity_linear_,
      "Enable or disable linear velocities rendering.",
      this, SLOT(setShowVelocityLinear()));

  property_show_velocity_angular_ = new rviz::BoolProperty(
      "Show Velocity Angular", show_velocity_angular_,
      "Enable or disable angular velocities rendering.",
      this, SLOT(setShowVelocityAngular()));

  property_show_acceleration_linear_ = new rviz::BoolProperty(
      "Show Acceleration Linear", show_acceleration_linear_,
      "Enable or disable linear accelerations rendering.",
      this, SLOT(setShowAccelerationLinear()));

  property_show_acceleration_angular_ = new rviz::BoolProperty(
      "Show Acceleration Angular", show_acceleration_angular_,
      "Enable or disable angular accelerations rendering.",
      this, SLOT(setShowAccelerationAngular()));

  property_size_transform_rotation_ = new rviz::FloatProperty(
      "Size Transform Rotation", size_transform_rotation_,
      "Size of the axes of the rotation transforms.",
      this, SLOT(setSizeTransformRotation()));
  property_size_transform_rotation_->setMin(0);

  property_diameter_arrows_ = new rviz::FloatProperty(
      "Diameter Arrows", diameter_arrows_,
      "Diameter of the arrows.",
      this, SLOT(setDiameterArrows()));
  property_diameter_arrows_->setMin(0);

  property_scale_velocity_linear_ = new rviz::FloatProperty(
      "Scale Velocity Linear", scale_velocity_linear_,
      "Scale of the linear velocities.",
      this, SLOT(setScaleVelocityLinear()));

  property_scale_velocity_angular_ = new rviz::FloatProperty(
      "Scale Velocity Angular", scale_velocity_angular_,
      "Scale of the angular velocities.",
      this, SLOT(setScaleVelocityAngular()));

  property_scale_acceleration_linear_ = new rviz::FloatProperty(
      "Scale Acceleration Linear", scale_acceleration_linear_,
      "Scale of the linear accelerations.",
      this, SLOT(setScaleAccelerationLinear()));

  property_scale_acceleration_angular_ = new rviz::FloatProperty(
      "Scale Acceleration Angular", scale_acceleration_angular_,
      "Scale of the angular accelerations.",
      this, SLOT(setScaleAccelerationAngular()));

  property_color_connection_ = new rviz::ColorProperty(
      "Color Connection", rviz::ogreToQt(color_connection_),
      "Color of connection lines.",
      this, SLOT(setColorConnection()));

  property_color_velocity_linear_ = new rviz::ColorProperty(
      "Color Velocity Linear", rviz::ogreToQt(color_velocity_linear_),
      "Color of the linear velocities.",
      this, SLOT(setColorVelocityLinear()));

  property_color_velocity_angular_ = new rviz::ColorProperty(
      "Color Velocity Angular", rviz::ogreToQt(color_velocity_angular_),
      "Color of the angular velocities.",
      this, SLOT(setColorVelocityAngular()));

  property_color_acceleration_linear_ = new rviz::ColorProperty(
      "Color Acceleration Linear", rviz::ogreToQt(color_acceleration_linear_),
      "Color of the linear accelerations.",
      this, SLOT(setColorAccelerationLinear()));

  property_color_acceleration_angular_ = new rviz::ColorProperty(
      "Color Acceleration Angular", rviz::ogreToQt(color_acceleration_angular_),
      "Color of the angular accelerations.",
      this, SLOT(setColorAccelerationAngular()));

  property_alpha_ = new rviz::FloatProperty(
      "Alpha", alpha_,
      "0 is fully transparent, 1.0 is fully opaque.",
      this, SLOT(setAlpha()));
  property_alpha_->setMin(0);
  property_alpha_->setMax(1);

  property_font_size_ = new rviz::FloatProperty(
      "Font Size", font_size_,
      "Size of the font.",
      this, SLOT(setFontSize()));
  property_font_size_->setMin(0);

  property_show_text_ = new rviz::BoolProperty(
      "Show Caption", show_text_,
      "Enable or disable text rendering.",
      this, SLOT(setShowText()));

  property_history_length_ = new rviz::IntProperty(
      "History Length", 1,
      "Number of prior measurements to display.",
      this, SLOT(setHistoryLength()));
  property_history_length_->setMin(1);
  property_history_length_->setMax(100000);
}

MultiDOFJointTrajectoryDisplay::~MultiDOFJointTrajectoryDisplay()
{

}

void MultiDOFJointTrajectoryDisplay::onInitialize()
{
  MFDClass::onInitialize();
  setHistoryLength();
}

void MultiDOFJointTrajectoryDisplay::reset()
{
  MFDClass::reset();
  visuals_points_.clear();
  visuals_connections_.clear();
}

void MultiDOFJointTrajectoryDisplay::setShowConnection()
{
  show_connection_ = property_show_connection_->getBool();
  updateShowConnection();
}

void MultiDOFJointTrajectoryDisplay::setShowTransformRotation()
{
  show_transform_rotation_ = property_show_transform_rotation_->getBool();
  updateShowTransformRotation();
}

void MultiDOFJointTrajectoryDisplay::setShowVelocityLinear()
{
  show_velocity_linear_ = property_show_velocity_linear_->getBool();
  updateShowVelocityLinear();
}

void MultiDOFJointTrajectoryDisplay::setShowVelocityAngular()
{
  show_velocity_angular_ = property_show_velocity_angular_->getBool();
  updateShowVelocityAngular();
}

void MultiDOFJointTrajectoryDisplay::setShowAccelerationLinear()
{
  show_acceleration_linear_ = property_show_acceleration_linear_->getBool();
  updateShowAccelerationLinear();
}

void MultiDOFJointTrajectoryDisplay::setShowAccelerationAngular()
{
  show_acceleration_angular_ = property_show_acceleration_angular_->getBool();
  updateShowAccelerationAngular();
}

void MultiDOFJointTrajectoryDisplay::setSizeTransformRotation()
{
  size_transform_rotation_ = property_size_transform_rotation_->getFloat();
  updateSizeTransformRotation();
}

void MultiDOFJointTrajectoryDisplay::setDiameterArrows()
{
  diameter_arrows_ = property_diameter_arrows_->getFloat();
  updateDiameterArrows();
}

void MultiDOFJointTrajectoryDisplay::setScaleVelocityLinear()
{
  scale_velocity_linear_ = property_scale_velocity_linear_->getFloat();
  updateScaleVelocityLinear();
}

void MultiDOFJointTrajectoryDisplay::setScaleVelocityAngular()
{
  scale_velocity_angular_ = property_scale_velocity_angular_->getFloat();
  updateScaleVelocityAngular();
}

void MultiDOFJointTrajectoryDisplay::setScaleAccelerationLinear()
{
  scale_acceleration_linear_ = property_scale_acceleration_linear_->getFloat();
  updateScaleAccelerationLinear();
}

void MultiDOFJointTrajectoryDisplay::setScaleAccelerationAngular()
{
  scale_acceleration_angular_ = property_scale_acceleration_angular_->getFloat();
  updateScaleAccelerationAngular();
}

void MultiDOFJointTrajectoryDisplay::setAlpha()
{
  alpha_ = property_alpha_->getFloat();
  color_connection_.a = alpha_;
  color_velocity_linear_.a = alpha_;
  color_velocity_angular_.a = alpha_;
  color_acceleration_linear_.a = alpha_;
  color_acceleration_angular_.a = alpha_;
  updateColorConnection();
  updateAlphaTransformRotation();
  updateColorVelocityLinear();
  updateColorVelocityAngular();
  updateColorAccelerationLinear();
  updateColorAccelerationAngular();
}

void MultiDOFJointTrajectoryDisplay::setColorConnection()
{
  color_connection_ = rviz::qtToOgre(property_color_connection_->getColor());
  color_connection_.a = property_alpha_->getFloat();
  updateColorConnection();
}

void MultiDOFJointTrajectoryDisplay::setColorVelocityLinear()
{
  color_velocity_linear_ = rviz::qtToOgre(property_color_velocity_linear_->getColor());
  color_velocity_linear_.a = property_alpha_->getFloat();
  updateColorVelocityLinear();
}

void MultiDOFJointTrajectoryDisplay::setColorVelocityAngular()
{
  color_velocity_angular_ = rviz::qtToOgre(property_color_velocity_angular_->getColor());
  color_velocity_angular_.a = property_alpha_->getFloat();
  updateColorVelocityAngular();
}

void MultiDOFJointTrajectoryDisplay::setColorAccelerationLinear()
{
  color_acceleration_linear_ = rviz::qtToOgre(property_color_acceleration_linear_->getColor());
  color_acceleration_linear_.a = property_alpha_->getFloat();
  updateColorAccelerationLinear();
}

void MultiDOFJointTrajectoryDisplay::setColorAccelerationAngular()
{
  color_acceleration_angular_ = rviz::qtToOgre(property_color_acceleration_angular_->getColor());
  color_acceleration_angular_.a = property_alpha_->getFloat();
  updateColorAccelerationAngular();
}

void MultiDOFJointTrajectoryDisplay::setFontSize()
{
  font_size_ = property_font_size_->getFloat();
  updateFontSize();
}

void MultiDOFJointTrajectoryDisplay::setShowText()
{
  show_text_ = property_show_text_->getBool();
  updateShowText();
}


void MultiDOFJointTrajectoryDisplay::setHistoryLength()
{
  visuals_points_.rset_capacity(property_history_length_->getInt());
  visuals_connections_.rset_capacity(property_history_length_->getInt());
}

void MultiDOFJointTrajectoryDisplay::updateTrajectory() {
    show_connection_           = property_show_connection_->getBool();
    show_transform_rotation_   = property_show_transform_rotation_->getBool();
    show_velocity_linear_      = property_show_velocity_linear_->getBool();
    show_velocity_angular_     = property_show_velocity_angular_->getBool();
    show_acceleration_linear_  = property_show_acceleration_linear_->getBool();
    show_acceleration_angular_ = property_show_acceleration_angular_->getBool();

    size_transform_rotation_    = property_size_transform_rotation_->getFloat();
    scale_velocity_linear_      = property_scale_velocity_linear_->getFloat();
    scale_velocity_angular_     = property_scale_velocity_angular_->getFloat();
    scale_acceleration_linear_  = property_scale_acceleration_linear_->getFloat();
    scale_acceleration_angular_ = property_scale_acceleration_angular_->getFloat();

    color_connection_           = rviz::qtToOgre(property_color_connection_->getColor());
    color_velocity_linear_      = rviz::qtToOgre(property_color_velocity_linear_->getColor());
    color_velocity_angular_     = rviz::qtToOgre(property_color_velocity_angular_->getColor());
    color_acceleration_linear_  = rviz::qtToOgre(property_color_acceleration_linear_->getColor());
    color_acceleration_angular_ = rviz::qtToOgre(property_color_acceleration_angular_->getColor());
    alpha_ = property_alpha_->getFloat();
    color_connection_.a           = alpha_;
    color_velocity_linear_.a      = alpha_;
    color_velocity_angular_.a     = alpha_;
    color_acceleration_linear_.a  = alpha_;
    color_acceleration_angular_.a = alpha_;

    std::vector<std::vector<std::string>> captions;
    for (unsigned int i = 0; i < current_trajectory_->points.size(); i++)
    {
        std::vector<std::string> caption_point;
        for (unsigned int j = 0; j < current_trajectory_->joint_names.size(); j++)
        {
            std::stringstream ss;
            ss << current_trajectory_->joint_names[j] << ": t" << i << " = " << current_trajectory_->points[i].time_from_start.toSec() << "s";
            caption_point.push_back(ss.str());
        }
        captions.push_back(caption_point);
    }

    font_size_ = property_font_size_->getFloat();
    show_text_ = property_show_text_->getBool();

    std::vector<boost::shared_ptr<MultiDOFJointTrajectoryPointVisual>> visuals_points;
    std::vector<boost::shared_ptr<MultiDOFJointTrajectoryPointConnectionVisual>> visuals_connections;

    trajectory_msgs::MultiDOFJointTrajectoryPoint last_point;
    trajectory_msgs::MultiDOFJointTrajectoryPoint current_point = current_trajectory_->points[0];

    // add first point
    visuals_points.push_back(boost::shared_ptr<MultiDOFJointTrajectoryPointVisual>(new MultiDOFJointTrajectoryPointVisual(
            context_->getSceneManager(),
            scene_node_,
            current_point,
            show_transform_rotation_,
            show_velocity_linear_,
            show_velocity_angular_,
            show_acceleration_linear_,
            show_acceleration_angular_,
            size_transform_rotation_,
            diameter_arrows_,
            scale_velocity_linear_,
            scale_velocity_angular_,
            scale_acceleration_linear_,
            scale_acceleration_angular_,
            alpha_,
            color_velocity_linear_,
            color_velocity_angular_,
            color_acceleration_linear_,
            color_acceleration_angular_,
            captions[0],
            font_size_,
            show_text_)));

    // add second to last points and connections to predecessors
    for (unsigned int i = 1; i < current_trajectory_->points.size(); i++)
    {
        // go one pose further
        last_point = current_point;
        current_point = current_trajectory_->points[i];

        // add edge to predecessor
        visuals_connections.push_back(boost::shared_ptr<MultiDOFJointTrajectoryPointConnectionVisual>(new MultiDOFJointTrajectoryPointConnectionVisual(context_->getSceneManager(),
                                                                                                                                                       scene_node_,
                                                                                                                                                       last_point,
                                                                                                                                                       current_point,
                                                                                                                                                       show_connection_,
                                                                                                                                                       color_connection_)));

        // add pose
        visuals_points.push_back(boost::shared_ptr<MultiDOFJointTrajectoryPointVisual>(new MultiDOFJointTrajectoryPointVisual(
                context_->getSceneManager(),
                scene_node_,
                current_point,
                show_transform_rotation_,
                show_velocity_linear_,
                show_velocity_angular_,
                show_acceleration_linear_,
                show_acceleration_angular_,
                size_transform_rotation_,
                diameter_arrows_,
                scale_velocity_linear_,
                scale_velocity_angular_,
                scale_acceleration_linear_,
                scale_acceleration_angular_,
                alpha_,
                color_velocity_linear_,
                color_velocity_angular_,
                color_acceleration_linear_,
                color_acceleration_angular_,
                captions[i],
                font_size_,
                show_text_)));
    }

    visuals_points_.push_back(visuals_points);
    visuals_connections_.push_back(visuals_connections);
}

void MultiDOFJointTrajectoryDisplay::processMessage(const trajectory_msgs::MultiDOFJointTrajectory::ConstPtr& msg)
{
  current_trajectory_ = msg;
  Q_EMIT updateTrajectorySignal();
}

void MultiDOFJointTrajectoryDisplay::updateShowConnection()
{
  for(size_t i = 0; i < visuals_connections_.size(); i++)
  {
    for (unsigned int j = 0; j < visuals_connections_[i].size(); j++)
    {
      visuals_connections_[i][j]->setShowConnection(show_connection_);
    }
  }
}

void MultiDOFJointTrajectoryDisplay::updateShowTransformRotation()
{
  for(size_t i = 0; i < visuals_points_.size(); i++)
  {
    for (unsigned int j = 0; j < visuals_points_[i].size(); j++)
    {
      visuals_points_[i][j]->setShowTransformRotation(show_transform_rotation_);
    }
  }
}

void MultiDOFJointTrajectoryDisplay::updateShowVelocityLinear()
{
  for(size_t i = 0; i < visuals_points_.size(); i++)
  {
    for (unsigned int j = 0; j < visuals_points_[i].size(); j++)
    {
      visuals_points_[i][j]->setShowVelocityLinear(show_velocity_linear_);
    }
  }
}

void MultiDOFJointTrajectoryDisplay::updateShowVelocityAngular()
{
  for(size_t i = 0; i < visuals_points_.size(); i++)
  {
    for (unsigned int j = 0; j < visuals_points_[i].size(); j++)
    {
      visuals_points_[i][j]->setShowVelocityAngular(show_velocity_angular_);
    }
  }
}

void MultiDOFJointTrajectoryDisplay::updateShowAccelerationLinear()
{
  for(size_t i = 0; i < visuals_points_.size(); i++)
  {
    for (unsigned int j = 0; j < visuals_points_[i].size(); j++)
    {
      visuals_points_[i][j]->setShowAccelerationLinear(show_acceleration_linear_);
    }
  }
}

void MultiDOFJointTrajectoryDisplay::updateShowAccelerationAngular()
{
  for(size_t i = 0; i < visuals_points_.size(); i++)
  {
    for (unsigned int j = 0; j < visuals_points_[i].size(); j++)
    {
      visuals_points_[i][j]->setShowAccelerationAngular(show_acceleration_angular_);
    }
  }
}

void MultiDOFJointTrajectoryDisplay::updateSizeTransformRotation()
{
  for(size_t i = 0; i < visuals_points_.size(); i++)
  {
    for (unsigned int j = 0; j < visuals_points_[i].size(); j++)
    {
      visuals_points_[i][j]->setSizeTransformRotation(size_transform_rotation_);
    }
  }
}

void MultiDOFJointTrajectoryDisplay::updateDiameterArrows()
{
  for(size_t i = 0; i < visuals_points_.size(); i++)
  {
    for (unsigned int j = 0; j < visuals_points_[i].size(); j++)
    {
      visuals_points_[i][j]->setDiametersArrows(diameter_arrows_);
    }
  }
}

void MultiDOFJointTrajectoryDisplay::updateScaleVelocityLinear()
{
  for(size_t i = 0; i < visuals_points_.size(); i++)
  {
    for (unsigned int j = 0; j < visuals_points_[i].size(); j++)
    {
      visuals_points_[i][j]->setScaleVelocityLinear(scale_velocity_linear_);
    }
  }
}

void MultiDOFJointTrajectoryDisplay::updateScaleVelocityAngular()
{
  for(size_t i = 0; i < visuals_points_.size(); i++)
  {
    for (unsigned int j = 0; j < visuals_points_[i].size(); j++)
    {
      visuals_points_[i][j]->setScaleVelocityAngular(scale_velocity_angular_);
    }
  }
}

void MultiDOFJointTrajectoryDisplay::updateScaleAccelerationLinear()
{
  for(size_t i = 0; i < visuals_points_.size(); i++)
  {
    for (unsigned int j = 0; j < visuals_points_[i].size(); j++)
    {
      visuals_points_[i][j]->setScaleAccelerationLinear(scale_acceleration_linear_);
    }
  }
}

void MultiDOFJointTrajectoryDisplay::updateScaleAccelerationAngular()
{
  for(size_t i = 0; i < visuals_points_.size(); i++)
  {
    for (unsigned int j = 0; j < visuals_points_[i].size(); j++)
    {
      visuals_points_[i][j]->setScaleAccelerationAngular(scale_acceleration_angular_);
    }
  }
}

void MultiDOFJointTrajectoryDisplay::updateColorConnection()
{
  for(size_t i = 0; i < visuals_connections_.size(); i++)
  {
    for (unsigned int j = 0; j < visuals_connections_[i].size(); j++)
    {
      visuals_connections_[i][j]->setColor(color_connection_);
    }
  }
}

void MultiDOFJointTrajectoryDisplay::updateAlphaTransformRotation()
{
  for(size_t i = 0; i < visuals_points_.size(); i++)
  {
    for (unsigned int j = 0; j < visuals_points_[i].size(); j++)
    {
      visuals_points_[i][j]->setAlphaTransformRotation(alpha_);
    }
  }
}

void MultiDOFJointTrajectoryDisplay::updateColorVelocityLinear()
{
  for(size_t i = 0; i < visuals_points_.size(); i++)
  {
    for (unsigned int j = 0; j < visuals_points_[i].size(); j++)
    {
      visuals_points_[i][j]->setColorVelocityLinear(color_velocity_linear_);
    }
  }
}

void MultiDOFJointTrajectoryDisplay::updateColorVelocityAngular()
{
  for(size_t i = 0; i < visuals_points_.size(); i++)
  {
    for (unsigned int j = 0; j < visuals_points_[i].size(); j++)
    {
      visuals_points_[i][j]->setColorVelocityAngular(color_velocity_angular_);
    }
  }
}

void MultiDOFJointTrajectoryDisplay::updateColorAccelerationLinear()
{
  for(size_t i = 0; i < visuals_points_.size(); i++)
  {
    for (unsigned int j = 0; j < visuals_points_[i].size(); j++)
    {
      visuals_points_[i][j]->setColorAccelerationLinear(color_acceleration_linear_);
    }
  }
}

void MultiDOFJointTrajectoryDisplay::updateColorAccelerationAngular()
{
  for(size_t i = 0; i < visuals_points_.size(); i++)
  {
    for (unsigned int j = 0; j < visuals_points_[i].size(); j++)
    {
      visuals_points_[i][j]->setColorAccelerationAngular(color_acceleration_angular_);
    }
  }
}

void MultiDOFJointTrajectoryDisplay::updateFontSize()
{
  for(size_t i = 0; i < visuals_points_.size(); i++)
  {
    for (unsigned int j = 0; j < visuals_points_[i].size(); j++)
    {
      visuals_points_[i][j]->setFontSize(font_size_);
    }
  }
}

void MultiDOFJointTrajectoryDisplay::updateShowText()
{
  for(size_t i = 0; i < visuals_points_.size(); i++)
  {
    for (unsigned int j = 0; j < visuals_points_[i].size(); j++)
    {
      visuals_points_[i][j]->setShowText(show_text_);
    }
  }
}

} // multi_dof_joint_trajectory_rviz_plugins

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(multi_dof_joint_trajectory_rviz_plugins::MultiDOFJointTrajectoryDisplay, rviz::Display)
