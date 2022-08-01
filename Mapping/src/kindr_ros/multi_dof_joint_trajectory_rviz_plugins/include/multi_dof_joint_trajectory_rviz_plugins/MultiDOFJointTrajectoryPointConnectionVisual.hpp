#pragma once

#include <OGRE/OgreVector3.h>
#include <OGRE/OgreSceneNode.h>
#include <OGRE/OgreSceneManager.h>

#include <rviz/ogre_helpers/line.h>
#include <rviz/ogre_helpers/movable_text.h>

#include <rviz/default_plugin/markers/text_view_facing_marker.h>

#include <trajectory_msgs/MultiDOFJointTrajectoryPoint.h>


namespace multi_dof_joint_trajectory_rviz_plugins {

class MultiDOFJointTrajectoryPointConnectionVisual
{
public:
  MultiDOFJointTrajectoryPointConnectionVisual(
      Ogre::SceneManager* scene_manager,
      Ogre::SceneNode* parent_node,
      const trajectory_msgs::MultiDOFJointTrajectoryPoint& from,
      const trajectory_msgs::MultiDOFJointTrajectoryPoint& to,
      float show_connection,
      const Ogre::ColourValue& color);
  virtual ~MultiDOFJointTrajectoryPointConnectionVisual();

  void setShowConnection(bool visible);
  void setColor(const Ogre::ColourValue& color);

private:
  void updateShowConnection();
  void updateColor();

  Ogre::SceneManager* scene_manager_;

  Ogre::SceneNode* scene_node_;

  std::vector<boost::shared_ptr<rviz::Line>> lines_;

  float show_connection_;
  Ogre::ColourValue color_;
};

} // multi_dof_joint_trajectory_rviz_plugins

