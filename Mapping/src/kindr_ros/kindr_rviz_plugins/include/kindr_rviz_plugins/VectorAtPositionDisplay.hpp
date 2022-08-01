#pragma once

#ifndef Q_MOC_RUN
// boost
#include <boost/circular_buffer.hpp>

//OGRE
#include <OGRE/OgreSceneNode.h>
#include <OGRE/OgreSceneManager.h>

// rviz
#include <rviz/message_filter_display.h>

// kindr ros
#include <kindr_msgs/VectorAtPosition.h>
#endif


namespace Ogre {
class SceneNode;
} // Ogre

namespace rviz {
class ColorProperty;
class FloatProperty;
class IntProperty;
} // rviz


// All the source in this plugin is in its own namespace. This is not
// required but is good practice.
namespace kindr_rviz_plugins {

class VectorAtPositionVisual;

// Here we declare our new subclass of rviz::Display. Every display
// which can be listed in the "Displays" panel is a subclass of
// rviz::Display.
//
// VectorAtPositionDisplay will show a 3D arrow showing the direction and magnitude
// of the IMU acceleration vector. The base of the arrow will be at
// the frame listed in the header of the VectorAtPosition message, and the
// direction of the arrow will be relative to the orientation of that
// frame. It will also optionally show a history of recent
// acceleration vectors, which will be stored in a circular buffer.
//
// The VectorAtPositionDisplay class itself just implements the circular buffer,
// editable parameters, and Display subclass machinery. The visuals
// themselves are represented by a separate class, VectorAtPositionVisual. The
// idiom for the visuals is that when the objects exist, they appear
// in the scene, and when they are deleted, they disappear.
class VectorAtPositionDisplay: public rviz::MessageFilterDisplay<kindr_msgs::VectorAtPosition>
{
Q_OBJECT
public:
  // Constructor. pluginlib::ClassLoader creates instances by calling
  // the default constructor, so make sure you have one.
  VectorAtPositionDisplay();
  virtual ~VectorAtPositionDisplay();

  // Overrides of protected virtual functions from Display. As much
  // as possible, when Displays are not enabled, they should not be
  // subscribed to incoming data and should not show anything in the
  // 3D view. These functions are where these connections are made
  // and broken.
protected:
  virtual void onInitialize();

  // A helper to clear this display back to the initial state.
  virtual void reset();

Q_SIGNALS:
  void updateVectorAtPositionSignal();

  // These Qt slots get connected to signals indicating changes in the user-editable properties.
private Q_SLOTS:
  void updateScale();
  void updateShowText();
  void updateColorAndAlpha();
  void updateHistoryLength();

  void updateVectorAtPosition();

  // Function to handle an incoming ROS message.
private:
  void processMessage(const kindr_msgs::VectorAtPosition::ConstPtr& msg);

  // Storage for the list of visuals. It is a circular buffer where
  // data gets popped from the front (oldest) and pushed to the back (newest)
  boost::circular_buffer<boost::shared_ptr<VectorAtPositionVisual> > visuals_;

  // User-editable property variables.
  rviz::FloatProperty* length_scale_property_;
  rviz::FloatProperty* width_scale_property_;
  rviz::BoolProperty* show_text_property_;
  rviz::ColorProperty* color_property_;
  rviz::FloatProperty* alpha_property_;
  rviz::IntProperty* history_length_property_;

  // Storage of user editable values
  float lengthScale_;
  float widthScale_;
  bool showText_;
  Ogre::ColourValue color_;
  float alpha_;

  kindr_msgs::VectorAtPosition::ConstPtr current_vector_at_position_;
};

} // kindr_rviz_plugins
