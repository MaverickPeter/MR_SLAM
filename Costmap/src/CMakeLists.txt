cmake_minimum_required(VERSION 3.0.2)
project(pointMap_layer)

## Compile as C++11, supported in ROS Kinetic and newer
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
# add_compile_options(-std=c++11)
find_package(PCL REQUIRED)
## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  dynamic_reconfigure roscpp costmap_2d 
  grid_map_core grid_map_ros grid_map_msgs rqt_gui rqt_gui_py
   rviz pcl_ros
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES simple_layers
#  CATKIN_DEPENDS costmap_2d dynamic_reconfigure roscpp
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

## Declare a C++ library
add_library(pointMap_layer
  src/pointMap_layer.cpp
)
target_link_libraries(pointMap_layer ${catkin_LIBRARIES} ${PCL_LIBRARIES})
target_compile_options(pointMap_layer PRIVATE -std=c++11)


add_library(elevationMap_layer 
  src/elevationMap_layer.cpp)
target_link_libraries(elevationMap_layer ${catkin_LIBRARIES} ${PCL_LIBRARIES})
target_compile_options(elevationMap_layer PRIVATE -std=c++11)

#############
## Install ##
#############
install(FILES
  costmap_plugins.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(
  TARGETS
  pointMap_layer  
  elevationMap_layer  
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)
