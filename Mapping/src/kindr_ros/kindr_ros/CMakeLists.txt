cmake_minimum_required(VERSION 2.8.3)
project(kindr_ros)

add_definitions(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
   geometry_msgs tf
)

# Attempt to find catkinized kindr
find_package(kindr QUIET)
if(NOT kindr_FOUND)
  # Attempt to find package-based kindr
  find_package(PkgConfig REQUIRED)
  pkg_check_modules(kindr kindr REQUIRED)
endif()

find_package(Eigen3 REQUIRED)

catkin_package(
  INCLUDE_DIRS include $ENV{EIGEN3_INCLUDE_DIR}
#  LIBRARIES kindr_ros
  CATKIN_DEPENDS geometry_msgs tf
  DEPENDS kindr
)

include_directories(include)
include_directories(${EIGEN3_INCLUDE_DIR})
include_directories(${kindr_INCLUDE_DIRS})
include_directories(${catkin_INCLUDE_DIRS})

#############
## Testing ##
#############

catkin_add_gtest(${PROJECT_NAME}-test 
    test/test_main.cpp
    test/RosGeometryMsgPhysicalQuantitiesTest.cpp
    test/RosGeometryMsgRotationTest.cpp
    test/RosGeometryMsgPoseTest.cpp
    test/RosTfPoseTest.cpp
    test/TfConventionTest.cpp
)
if(TARGET ${PROJECT_NAME}-test)
   target_link_libraries(${PROJECT_NAME}-test ${catkin_LIBRARIES})
endif()

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
