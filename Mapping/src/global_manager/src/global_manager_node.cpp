#include <global_manager/global_manager.h>

#include <pcl_ros/point_cloud.h>
#include <ros/assert.h>
#include <ros/console.h>
#include <tf2_eigen/tf2_eigen.h>


int main(int argc, char** argv)
{
  ros::init(argc, argv, "global_manager");

  ros::NodeHandle private_nh("~");

  std::string logger_level;
  private_nh.param<std::string>("logger_level", logger_level, "Info");

  if(logger_level == "Info"){
    // this package is still in development -- start wil debugging enabled
    if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME,
                                      ros::console::levels::Info)) {
      ros::console::notifyLoggerLevelsChanged();
    }
  }else if(logger_level == "Debug"){
    // this package is still in development -- start wil debugging enabled
    if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME,
                                      ros::console::levels::Debug)) {
      ros::console::notifyLoggerLevelsChanged();
    }
  }else if(logger_level == "Error"){
    // this package is still in development -- start wil debugging enabled
    if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME,
                                      ros::console::levels::Error)) {
      ros::console::notifyLoggerLevelsChanged();
    }
  }else if(logger_level == "Warn"){
    // this package is still in development -- start wil debugging enabled
    if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME,
                                      ros::console::levels::Warn)) {
      ros::console::notifyLoggerLevelsChanged();
    }
  }

  global_manager::GlobalManager global_manager_node(private_nh);
  boost::thread GlobalManager(&global_manager::GlobalManager::discoveryThread, &global_manager_node);
  boost::thread MapComposingThread(&global_manager::GlobalManager::mapComposingThread, &global_manager_node);
  boost::thread LoopClousreThread(&global_manager::GlobalManager::loopClosingThread, &global_manager_node);
  boost::thread TFPublishThread(&global_manager::GlobalManager::publishTFThread, &global_manager_node);
  boost::thread PoseGraphPublishThread(&global_manager::GlobalManager::publishPoseGraphThread, &global_manager_node);
  boost::thread GeometryCheckPublishThread(&global_manager::GlobalManager::geometryCheckThread, &global_manager_node);

  // use all threads for spinning
  ros::MultiThreadedSpinner spinner;
  spinner.spin();

  return 0;
}
