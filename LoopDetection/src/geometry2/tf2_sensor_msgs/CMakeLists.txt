cmake_minimum_required(VERSION 3.0.2)
project(tf2_sensor_msgs)

find_package(catkin REQUIRED COMPONENTS cmake_modules sensor_msgs tf2_ros tf2)
find_package(Boost COMPONENTS thread REQUIRED)

# Finding Eigen is somewhat complicated because of our need to support Ubuntu
# all the way back to saucy.  First we look for the Eigen3 cmake module
# provided by the libeigen3-dev on newer Ubuntu.  If that fails, then we
# fall-back to the version provided by cmake_modules, which is a stand-in.
find_package(Eigen3 QUIET)
if(NOT EIGEN3_FOUND)
  find_package(cmake_modules REQUIRED)
  find_package(Eigen REQUIRED)
  set(EIGEN3_INCLUDE_DIRS ${EIGEN_INCLUDE_DIRS})
endif()

# Note that eigen 3.2 (on Ubuntu Wily) only provides EIGEN3_INCLUDE_DIR,
# not EIGEN3_INCLUDE_DIRS, so we have to set the latter from the former.
if(NOT EIGEN3_INCLUDE_DIRS)
  set(EIGEN3_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR})
endif()

catkin_package(
   INCLUDE_DIRS include
   CATKIN_DEPENDS sensor_msgs tf2_ros tf2
   DEPENDS EIGEN3
)


include_directories(include
   ${catkin_INCLUDE_DIRS}
   ${EIGEN3_INCLUDE_DIRS}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

catkin_python_setup()

if(CATKIN_ENABLE_TESTING)
  catkin_add_nosetests(test/test_tf2_sensor_msgs.py)

  find_package(catkin REQUIRED COMPONENTS geometry_msgs sensor_msgs rostest tf2_ros tf2)

  add_executable(test_tf2_sensor_msgs_cpp EXCLUDE_FROM_ALL test/test_tf2_sensor_msgs.cpp)
  target_link_libraries(test_tf2_sensor_msgs_cpp ${catkin_LIBRARIES} ${GTEST_LIBRARIES})


  if(TARGET tests)
    add_dependencies(tests test_tf2_sensor_msgs_cpp)
  endif()
  add_rostest(${CMAKE_CURRENT_SOURCE_DIR}/test/test.launch)

endif()
