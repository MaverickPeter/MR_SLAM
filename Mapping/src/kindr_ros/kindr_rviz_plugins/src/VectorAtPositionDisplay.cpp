// ogre
#include <OGRE/OgreSceneNode.h>
#include <OGRE/OgreSceneManager.h>

// tf
#include <tf/transform_listener.h>

// rviz
#include <rviz/visualization_manager.h>
#include <rviz/properties/color_property.h>
#include <rviz/properties/float_property.h>
#include <rviz/properties/int_property.h>
#include <rviz/frame_manager.h>

// kindr rviz plugins
#include "kindr_rviz_plugins/VectorAtPositionDisplay.hpp"
#include "kindr_rviz_plugins/VectorAtPositionVisual.hpp"


namespace kindr_rviz_plugins {

// The constructor must have no arguments, so we can't give the
// constructor the parameters it needs to fully initialize.
VectorAtPositionDisplay::VectorAtPositionDisplay()
: lengthScale_(1.0f),
  widthScale_(1.0f),
  showText_(true),
  color_(Ogre::ColourValue::Black),
  alpha_(1.0)
{
  connect(this, SIGNAL(updateVectorAtPositionSignal()), this, SLOT(updateVectorAtPosition()));

  length_scale_property_ = new rviz::FloatProperty("Length scale", 1.0,
                                                   "Scale of the length of the vector.",
                                                   this, SLOT(updateScale()));

  width_scale_property_ = new rviz::FloatProperty("Width scale", 1.0,
                                                  "Scale of the width of the vector.",
                                                  this, SLOT(updateScale()));
  width_scale_property_->setMin(0);

  show_text_property_ = new rviz::BoolProperty("Show text", true,
                                              "Enable or disable text rendering.",
                                              this, SLOT(updateShowText()));

  color_property_ = new rviz::ColorProperty("Color", QColor(0, 0, 0),
                                            "Color to draw the vector (if not defined by vector type).",
                                            this, SLOT(updateColorAndAlpha()));

  alpha_property_ = new rviz::FloatProperty("Alpha", 1.0,
                                            "0 is fully transparent, 1.0 is fully opaque.",
                                            this, SLOT(updateColorAndAlpha()));

  history_length_property_ = new rviz::IntProperty("History Length", 1,
                                                   "Number of prior measurements to display.",
                                                   this, SLOT(updateHistoryLength()));
  history_length_property_->setMin(1);
  history_length_property_->setMax(100000);
}

// After the top-level rviz::Display::initialize() does its own setup,
// it calls the subclass's onInitialize() function. This is where we
// instantiate all the workings of the class. We make sure to also
// call our immediate super-class's onInitialize() function, since it
// does important stuff setting up the message filter.
//
//  Note that "MFDClass" is a typedef of
// ``MessageFilterDisplay<message type>``, to save typing that long
// templated class name every time you need to refer to the
// superclass.
void VectorAtPositionDisplay::onInitialize()
{
  MFDClass::onInitialize();
  updateHistoryLength();
}

VectorAtPositionDisplay::~VectorAtPositionDisplay()
{
}

// Clear the visuals by deleting their objects.
void VectorAtPositionDisplay::reset()
{
  MFDClass::reset();
  visuals_.clear();
}

// Set the scale.
void VectorAtPositionDisplay::updateScale()
{
  lengthScale_ = length_scale_property_->getFloat();
  widthScale_ = width_scale_property_->getFloat();

  for(size_t i = 0; i < visuals_.size(); i++)
  {
    visuals_[i]->setScalingFactors(lengthScale_, widthScale_);
  }
}

// Show text.
void VectorAtPositionDisplay::updateShowText()
{
  showText_ = show_text_property_->getBool();

  for(size_t i = 0; i < visuals_.size(); i++)
  {
    visuals_[i]->setShowText(showText_);
  }
}

// Set the current color and alpha values for each visual.
void VectorAtPositionDisplay::updateColorAndAlpha()
{
  color_ = color_property_->getOgreColor();
  color_.a = alpha_property_->getFloat();

  for(size_t i = 0; i < visuals_.size(); i++)
  {
    visuals_[i]->setColor(color_);
  }
}

// Set the number of past visuals to show.
void VectorAtPositionDisplay::updateHistoryLength()
{
  visuals_.rset_capacity(history_length_property_->getInt());
}

void VectorAtPositionDisplay::updateVectorAtPosition() {

  // Here we call the rviz::FrameManager to get the transform from the
  // fixed frame to the frame in the header of this VectorAtPosition message.
  Ogre::Vector3 positionFixedToPositionFrameInFixedFrame;
  Ogre::Vector3 positionFixedToArrowInFixedFrame;
  Ogre::Quaternion orientationArrowFrameToFixedFrame;
  Ogre::Quaternion orientationPositionFrameToFixedFrame;

  // Check if the position has an empty or the same frame as the vector
  if (current_vector_at_position_->position_frame_id.empty() || current_vector_at_position_->position_frame_id == current_vector_at_position_->header.frame_id)
  {
    // Arrow and position frame coincide.

    // Get arrow position and orientation
    if(!context_->getFrameManager()->getTransform(current_vector_at_position_->header.frame_id,
                                                  current_vector_at_position_->header.stamp,
                                                  positionFixedToPositionFrameInFixedFrame,
                                                  orientationArrowFrameToFixedFrame))
    {
      ROS_ERROR("Error transforming from frame '%s' to frame '%s'", current_vector_at_position_->position_frame_id.c_str(), qPrintable(fixed_frame_));
      return;
    }

    orientationPositionFrameToFixedFrame = orientationArrowFrameToFixedFrame;
  }
  else
  {
    // Get arrow position
    if(!context_->getFrameManager()->getTransform(current_vector_at_position_->position_frame_id,
                                                  current_vector_at_position_->header.stamp,
                                                  positionFixedToPositionFrameInFixedFrame,
                                                  orientationPositionFrameToFixedFrame))
    {
      ROS_ERROR("Error transforming from frame '%s' to frame '%s'", current_vector_at_position_->position_frame_id.c_str(), qPrintable(fixed_frame_));
      return;
    }

    // Get arrow orientation
    Ogre::Vector3 dummyPosition;
    if(!context_->getFrameManager()->getTransform(current_vector_at_position_->header.frame_id,
                                                  current_vector_at_position_->header.stamp,
                                                  dummyPosition,
                                                  orientationArrowFrameToFixedFrame))
    {
      ROS_ERROR("Error transforming from frame '%s' to frame '%s'", current_vector_at_position_->header.frame_id.c_str(), qPrintable(fixed_frame_));
      return;
    }

  }

  Ogre::Matrix3 rotMat;
  orientationPositionFrameToFixedFrame.ToRotationMatrix(rotMat);
  positionFixedToArrowInFixedFrame = positionFixedToPositionFrameInFixedFrame
                                   + rotMat*Ogre::Vector3(current_vector_at_position_->position.x,
                                                          current_vector_at_position_->position.y,
                                                          current_vector_at_position_->position.z);

  // We are keeping a circular buffer of visual pointers. This gets
  // the next one, or creates and stores it if the buffer is not full
  boost::shared_ptr<VectorAtPositionVisual> visual;
  if(visuals_.full())
  {
    visual = visuals_.front();
  }
  else
  {
    visual.reset(new VectorAtPositionVisual(context_->getSceneManager(), scene_node_));
  }

  // Now set or update the contents of the chosen visual.
  visual->setMessage(current_vector_at_position_);
  visual->setArrowPosition(positionFixedToArrowInFixedFrame); // position is taken from position in msg
  visual->setArrowOrientation(orientationArrowFrameToFixedFrame); // orientation is taken from vector in msg
  visual->setScalingFactors(lengthScale_, widthScale_);
  visual->setShowText(showText_);
  visual->setColor(color_);

  // And send it to the end of the circular buffer
  visuals_.push_back(visual);
}

// This is our callback to handle an incoming message.
void VectorAtPositionDisplay::processMessage(const kindr_msgs::VectorAtPosition::ConstPtr& msg)
{
  current_vector_at_position_ = msg;
  Q_EMIT updateVectorAtPositionSignal();
}

} // kindr_rviz_plugins


// Tell pluginlib about this class. It is important to do this in
// global scope, outside our package's namespace.
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(kindr_rviz_plugins::VectorAtPositionDisplay, rviz::Display)
