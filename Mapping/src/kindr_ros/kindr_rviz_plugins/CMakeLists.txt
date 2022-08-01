cmake_minimum_required(VERSION 2.8.3)
project(kindr_rviz_plugins)

# Set compiler flags
set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")

find_package(catkin REQUIRED COMPONENTS
  rviz
  kindr_msgs
)

catkin_package(
  INCLUDE_DIRS
    include
  LIBRARIES
    ${PROJECT_NAME}
  CATKIN_DEPENDS
    rviz
    kindr_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
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
set(kindr_rviz_plugins_HDRS_QT
  include/kindr_rviz_plugins/VectorAtPositionDisplay.hpp
)
set(kindr_rviz_plugins_HDRS
  include/kindr_rviz_plugins/VectorAtPositionVisual.hpp
)

## Here we specify the list of source files, including the output of
## the previous command which is stored in ``${MOC_FILES}``.
set(kindr_rviz_plugins_SRCS
  src/VectorAtPositionDisplay.cpp
  src/VectorAtPositionVisual.cpp
)

if(rviz_QT_VERSION VERSION_GREATER "5.0.0")
  qt5_wrap_cpp(kindr_rviz_plugins_MOCS ${kindr_rviz_plugins_HDRS_QT})
else()
  qt4_wrap_cpp(kindr_rviz_plugins_MOCS ${kindr_rviz_plugins_HDRS_QT})
endif()

add_library(${PROJECT_NAME}
  ${kindr_rviz_plugins_SRCS}
  ${kindr_rviz_plugins_HDRS_QT}
  ${kindr_rviz_plugins_HDRS}
  ${kindr_rviz_plugins_MOCS}
)

add_dependencies(${PROJECT_NAME}
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(${PROJECT_NAME}
  ${QT_LIBRARIES}
  ${catkin_LIBRARIES}
  pthread
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
