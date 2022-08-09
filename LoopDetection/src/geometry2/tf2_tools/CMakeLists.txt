cmake_minimum_required(VERSION 3.0.2)
project(tf2_tools)

find_package(catkin REQUIRED COMPONENTS tf2
                                         tf2_msgs
                                         tf2_ros
)

catkin_package(
   CATKIN_DEPENDS tf2
                  tf2_msgs
                  tf2_ros)

install(PROGRAMS scripts/view_frames.py scripts/echo.py
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
