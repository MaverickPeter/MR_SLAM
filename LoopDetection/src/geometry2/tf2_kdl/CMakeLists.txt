cmake_minimum_required(VERSION 3.0.2)
project(tf2_kdl)

find_package(orocos_kdl)
find_package(catkin REQUIRED COMPONENTS cmake_modules tf2 tf2_ros tf2_msgs)

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
  INCLUDE_DIRS include ${EIGEN3_INCLUDE_DIRS}
  DEPENDS EIGEN3 orocos_kdl
)


catkin_python_setup()
link_directories(${orocos_kdl_LIBRARY_DIRS})

include_directories(include ${catkin_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIRS} ${GTEST_INCLUDE_DIRS})


install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

catkin_install_python(PROGRAMS scripts/test.py
         DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)


if(CATKIN_ENABLE_TESTING)

  find_package(catkin REQUIRED COMPONENTS rostest tf2 tf2_ros tf2_msgs)

  add_executable(test_kdl EXCLUDE_FROM_ALL test/test_tf2_kdl.cpp)
  find_package(Threads)
  target_include_directories(test_kdl PUBLIC ${orocos_kdl_INCLUDE_DIRS})
  target_link_libraries(test_kdl ${catkin_LIBRARIES} ${GTEST_LIBRARIES} ${orocos_kdl_LIBRARIES} ${GTEST_LIBRARIES} ${CMAKE_THREAD_LIBS_INIT})

  add_rostest(${CMAKE_CURRENT_SOURCE_DIR}/test/test.launch)
  add_rostest(${CMAKE_CURRENT_SOURCE_DIR}/test/test_python.launch)

  if(TARGET tests)
    add_dependencies(tests test_kdl)
  endif()

endif()
