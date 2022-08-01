cmake_minimum_required(VERSION 2.8.3)
project(multi_dof_joint_trajectory_rviz_plugins)

# Set compiler flags
set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")

find_package(catkin REQUIRED COMPONENTS
   rviz
   trajectory_msgs
)

catkin_package(
  INCLUDE_DIRS
    include
  LIBRARIES
    ${PROJECT_NAME}
  CATKIN_DEPENDS
    rviz
    trajectory_msgs
)

link_directories(${catkin_LIBRARY_DIRS})

## This plugin includes Qt widgets, so we must include Qt like so:
if(rviz_QT_VERSION VERSION_GREATER "5.0.0")
  find_package(Qt5Widgets REQUIRED)
else()
  find_package(Qt4 COMPONENTS QtCore QtGui REQUIRED)
  include(${QT_USE_FILE})
endif()

## I prefer the Qt signals and slots to avoid defining "emit", "slots",
## etc because they can conflict with boost signals, so define QT_NO_KEYWORDS here.
add_definitions(-DQT_NO_KEYWORDS)

## Here we specify which header files need to be run through "moc",
## Qt's meta-object compiler.
set(multi_dof_joint_trajectory_rviz_plugins_HDRS_QT
  include/multi_dof_joint_trajectory_rviz_plugins/MultiDOFJointTrajectoryDisplay.hpp
)
set(multi_dof_joint_trajectory_rviz_plugins_HDRS
  include/multi_dof_joint_trajectory_rviz_plugins/MultiDOFJointTrajectoryPointConnectionVisual.hpp
  include/multi_dof_joint_trajectory_rviz_plugins/MultiDOFJointTrajectoryPointVisual.hpp
)

set(multi_dof_joint_trajectory_rviz_plugins_SRCS
  src/MultiDOFJointTrajectoryDisplay.cpp
  src/MultiDOFJointTrajectoryPointConnectionVisual.cpp
  src/MultiDOFJointTrajectoryPointVisual.cpp
)

if(rviz_QT_VERSION VERSION_GREATER "5.0.0")
  qt5_wrap_cpp(multi_dof_joint_trajectory_rviz_plugins_MOCS
    ${multi_dof_joint_trajectory_rviz_plugins_HDRS_QT}
  )
else()
  qt4_wrap_cpp(multi_dof_joint_trajectory_rviz_plugins_MOCS
    ${multi_dof_joint_trajectory_rviz_plugins_HDRS_QT}
  )
endif()

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
  ${multi_dof_joint_trajectory_rviz_plugins_SRCS}
  ${multi_dof_joint_trajectory_rviz_plugins_HDRS_QT}
  ${multi_dof_joint_trajectory_rviz_plugins_HDRS}
  ${multi_dof_joint_trajectory_rviz_plugins_MOCS}
)

target_link_libraries(${PROJECT_NAME}
  ${QT_LIBRARIES} ${catkin_LIBRARIES}
)

if(rviz_QT_VERSION VERSION_GREATER "5.0.0")
  target_link_libraries(${PROJECT_NAME} Qt5::Widgets)
else()
  target_link_libraries(${PROJECT_NAME} ${QT_QTCORE_LIBRARY} ${QT_QTGUI_LIBRARY})
endif()

## Install rules

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(FILES plugin_description.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY icons
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
