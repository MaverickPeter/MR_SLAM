cmake_minimum_required(VERSION 2.8.3)
project(pyslam_msgs)


find_package(catkin REQUIRED COMPONENTS
  message_generation
  geometry_msgs
  std_msgs
  sensor_msgs
  nav_msgs
  message_runtime
)

add_message_files(
  DIRECTORY msg
  FILES
  Loop.msg
  SubMap.msg
)


generate_messages(
  DEPENDENCIES
  geometry_msgs
  sensor_msgs  
  std_msgs
  nav_msgs
)


catkin_package(
  CATKIN_DEPENDS 
  message_runtime 
  message_generation
  geometry_msgs
  sensor_msgs
  std_msgs  
  nav_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)



