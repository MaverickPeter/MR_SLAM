cmake_minimum_required(VERSION 2.8.3)
project(dislam_msgs)


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
  SubMap.msg
  DiSCO.msg
  InterPR.msg
  InterPRs.msg
  Loop.msg
  Loops.msg
)

add_service_files(
  FILES
  GetInitMap.srv
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



