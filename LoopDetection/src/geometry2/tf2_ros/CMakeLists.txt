cmake_minimum_required(VERSION 3.0.2)
project(tf2_ros)

if(NOT ANDROID)
set(TF2_PY tf2_py)
endif()

find_package(catkin REQUIRED COMPONENTS
  actionlib
  actionlib_msgs
  geometry_msgs
  message_filters
  roscpp
  rosgraph
  rospy
  tf2
  tf2_msgs
  ${TF2_PY}
)
find_package(Boost REQUIRED COMPONENTS thread)

catkin_python_setup()

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS
  actionlib
  actionlib_msgs
  geometry_msgs
  message_filters
  roscpp
  rosgraph
  tf2
  tf2_msgs
  ${TF2_PY}
)

include_directories(include ${catkin_INCLUDE_DIRS})

# tf2_ros library
add_library(${PROJECT_NAME}
  src/buffer.cpp
  src/transform_listener.cpp
  src/buffer_client.cpp
  src/buffer_server.cpp
  src/transform_broadcaster.cpp
  src/static_transform_broadcaster.cpp
)
add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

# buffer_server executable
add_executable(${PROJECT_NAME}_buffer_server src/buffer_server_main.cpp)
add_dependencies(${PROJECT_NAME}_buffer_server ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_buffer_server
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
)
set_target_properties(${PROJECT_NAME}_buffer_server
  PROPERTIES OUTPUT_NAME buffer_server
)

# static_transform_publisher
add_executable(${PROJECT_NAME}_static_transform_publisher
  src/static_transform_broadcaster_program.cpp
)
add_dependencies(${PROJECT_NAME}_static_transform_publisher ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_static_transform_publisher
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
)
set_target_properties(${PROJECT_NAME}_static_transform_publisher
  PROPERTIES OUTPUT_NAME static_transform_publisher
)

# Install library
install(TARGETS
  ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

# Install executable
install(TARGETS
  ${PROJECT_NAME}_buffer_server ${PROJECT_NAME}_static_transform_publisher
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)


# Tests
if(CATKIN_ENABLE_TESTING)

# new requirements for testing
find_package(catkin REQUIRED COMPONENTS
  actionlib
  actionlib_msgs
  geometry_msgs
  message_filters
  roscpp
  rosgraph
  rospy
  rostest
  tf2
  tf2_msgs
  ${TF2_PY}
)



# tf2_ros_test_listener
add_executable(${PROJECT_NAME}_test_listener EXCLUDE_FROM_ALL test/listener_unittest.cpp)
add_dependencies(${PROJECT_NAME}_test_listener ${catkin_EXPORTED_TARGETS})
add_executable(${PROJECT_NAME}_test_time_reset EXCLUDE_FROM_ALL test/time_reset_test.cpp)
add_dependencies(${PROJECT_NAME}_test_time_reset ${catkin_EXPORTED_TARGETS})
add_executable(${PROJECT_NAME}_test_message_filter EXCLUDE_FROM_ALL test/message_filter_test.cpp)
add_dependencies(${PROJECT_NAME}_test_message_filter ${catkin_EXPORTED_TARGETS})

target_link_libraries(${PROJECT_NAME}_test_listener
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${GTEST_LIBRARIES}
)

target_link_libraries(${PROJECT_NAME}_test_time_reset
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${GTEST_LIBRARIES}
)

target_link_libraries(${PROJECT_NAME}_test_message_filter
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${GTEST_LIBRARIES}
)

add_dependencies(tests ${PROJECT_NAME}_test_listener)
add_dependencies(tests ${PROJECT_NAME}_test_time_reset)
add_dependencies(tests ${PROJECT_NAME}_test_message_filter)

add_rostest(test/transform_listener_unittest.launch)
add_rostest(test/transform_listener_time_reset_test.launch)
add_rostest(test/message_filter_test.launch)

endif()
