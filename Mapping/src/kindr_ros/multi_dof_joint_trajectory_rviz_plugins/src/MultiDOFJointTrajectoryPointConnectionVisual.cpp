#include "multi_dof_joint_trajectory_rviz_plugins/MultiDOFJointTrajectoryPointConnectionVisual.hpp"


namespace multi_dof_joint_trajectory_rviz_plugins {

MultiDOFJointTrajectoryPointConnectionVisual::MultiDOFJointTrajectoryPointConnectionVisual(
    Ogre::SceneManager* scene_manager,
    Ogre::SceneNode* parent_node,
    const trajectory_msgs::MultiDOFJointTrajectoryPoint& from,
    const trajectory_msgs::MultiDOFJointTrajectoryPoint& to,
    float show_connection,
    const Ogre::ColourValue& color)
: scene_manager_(scene_manager),
  show_connection_(show_connection),
  color_(color)
{
  // check vector lengths
  assert(from.transforms.size() == to.transforms.size() && "ERROR: MultiDOFJointTrajectoryPointConnectionVisual: from and to vectors do not have the same length.");

  // scene node
  scene_node_ = parent_node->createChildSceneNode();

  // lines
  for (unsigned int i = 0; i < from.transforms.size(); i++)
  {
    lines_.push_back(boost::shared_ptr<rviz::Line>(new rviz::Line(scene_manager_, scene_node_)));
    const Ogre::Vector3 fromVector(from.transforms[i].translation.x, from.transforms[i].translation.y, from.transforms[i].translation.z);
    const Ogre::Vector3 toVector(to.transforms[i].translation.x, to.transforms[i].translation.y, to.transforms[i].translation.z);
    lines_.back()->setPoints(fromVector, toVector);
    updateColor();
    lines_.back()->setVisible(show_connection_);
  }
}

MultiDOFJointTrajectoryPointConnectionVisual::~MultiDOFJointTrajectoryPointConnectionVisual()
{
  scene_manager_->destroySceneNode(scene_node_);
}

void MultiDOFJointTrajectoryPointConnectionVisual::setShowConnection(bool visible)
{
  show_connection_ = visible;
  updateShowConnection();
}

void MultiDOFJointTrajectoryPointConnectionVisual::setColor(const Ogre::ColourValue& color)
{
  color_ = color;
  updateColor();
}

void MultiDOFJointTrajectoryPointConnectionVisual::updateShowConnection()
{
  for (unsigned int i = 0; i < lines_.size(); i++)
  {
    lines_[i]->setVisible(show_connection_);
  }
}

void MultiDOFJointTrajectoryPointConnectionVisual::updateColor()
{
  for (unsigned int i = 0; i < lines_.size(); i++)
  {
    lines_[i]->setColor(color_);
  }
}

} // multi_dof_joint_trajectory_rviz_plugins

