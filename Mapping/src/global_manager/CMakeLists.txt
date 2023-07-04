cmake_minimum_required(VERSION 3.1)
project(global_manager)

set(CMAKE_BUILD_TYPE "RELEASE")

add_definitions(-w)
add_definitions(${GLEW_DEFINITIONS})
add_definitions(-msse -msse2 -msse3 -msse4 -msse4.1 -msse4.2)
set(CMAKE_CXX_FLAGS "-msse -msse2 -msse3 -msse4 -msse4.1 -msse4.2")
set(CMAKE_CXX_STANDARD 11)

# Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  fast_gicp
  roscpp
  pcl_ros
  sensor_msgs
  std_msgs
  dislam_msgs
  cv_bridge
  tf2_eigen
  tf_conversions
  tf2_ros
)

find_package(OpenMP)
if (OPENMP_FOUND)
    set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif()

find_library(VGICP_CUDA_FOUND NAMES fast_vgicp_cuda)
message(STATUS "*********************VGICP_CUDA_FOUND:" ${VGICP_CUDA_FOUND})
if(VGICP_CUDA_FOUND)
  add_definitions(-DUSE_VGICP_CUDA)
endif()


if(NOT DEFINED CMAKE_SUPPRESS_DEVELOPER_WARNINGS)
     set(CMAKE_SUPPRESS_DEVELOPER_WARNINGS 1 CACHE INTERNAL "No dev warnings")
endif()
find_package(PCL REQUIRED COMPONENTS core features filters io registration visualization)
add_definitions(${PCL_DEFINITIONS})

# Find GTSAM
find_package(GTSAM REQUIRED)
find_package(FFTW3 REQUIRED)
find_package(CUDA 10 REQUIRED)


#find_package(Yaml-cpp REQUIRED)
#include_directories(${YAML_CPP_INCLUDE_DIR})
#link_directories(${YAML_CPP_LIBRARY_DIRS})

# include_directories(${GTSAM_INCLUDE_DIR})
link_directories(${GTSAM_LIBRARY_DIRS})

# include_directories(${Open3D_INCLUDE_DIR})
# link_directories(${Open3D_LIBRARY_DIRS})
#message("GTSAM Include dir: ${GTSAM_INCLUDE_DIR}")
#message("YAML_CPP Include dir: ${YAML_CPP_INCLUDE_DIR}")
###########################################################################
# Find Boost
find_package(Boost COMPONENTS serialization filesystem timer system program_options thread REQUIRED)
include_directories(${Boost_INCLUDE_DIRS})
link_libraries(${Boost_LIBRARIES})
#message("Boost Libraries: ${Boost_LIBRARIES}")

# workaround issues on debian
# see: https://github.com/ros-perception/perception_pcl/issues/139
if(NOT "${PCL_LIBRARIES}" STREQUAL "")
  list(REMOVE_ITEM PCL_LIBRARIES
    "vtkGUISupportQt"
    "vtkGUISupportQtOpenGL"
    "vtkGUISupportQtSQL"
    "vtkGUISupportQtWebkit"
    "vtkViewsQt"
    "vtkRenderingQt")
endif()

# There is a bug in the Ubuntu Artful (17.10) version of the VTK package,
# where it includes /usr/include/*-linux-gnu/freetype2 in the include
# directories (which doesn't exist).  This filters down to the PCL_INCLUDE_DIRS,
# and causes downstream projects trying to use these libraries to fail to
# configure properly.  Here we remove those bogus entries so that downstream
# consumers of this package succeed.
if(NOT "${PCL_INCLUDE_DIRS}" STREQUAL "")
  foreach(item ${PCL_INCLUDE_DIRS})
    string(REGEX MATCH "/usr/include/.*-linux-gnu/freetype2" item ${item})
    if(item)
      list(REMOVE_ITEM PCL_INCLUDE_DIRS ${item})
    endif()
  endforeach()
endif()

# C++14 is supported since ROS Melodic
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

################################################
## Declare ROS messages, services and actions ##
################################################
# we don't have any

###################################
## catkin specific configuration ##
###################################
catkin_package(
  INCLUDE_DIRS
    include
  CATKIN_DEPENDS
    submap_msg
    sensor_msgs
    std_msgs
  # DEPENDS Open3D
)

###################################
## Cuda ICP ##
###################################

# add_subdirectory(cudaICP)
# set (EXTRA_LIBS ${EXTRA_LIBS} cudaICP)


###########
## Build ##
###########

# Specify additional locations of header files
include_directories(
  include
  include/distributed_mapper
  include/global_manager
  include/pairwise_consistency_maximization
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${CUDA_INCLUDE_DIRS}
  # ${Open3D_INCLUDE_DIRS}
)

###########################################################################
# Pairwise Consistency Maximization

# Overall include directories
include_directories(
  include
  include/pairwise_consistency_maximization
  ${CMAKE_CURRENT_SOURCE_DIR}/src/pairwise_consistency_maximization/third_parties/fast_max-clique_finder/src/
)

# Graph utils library
add_library(graph_utils
        src/pairwise_consistency_maximization/graph_utils/graph_utils_functions.cpp
        ${GTSAM_INCLUDE_DIR}
)
target_link_libraries(graph_utils
        gtsam
)

# Robot local map library
add_library(robot_measurements
        src/pairwise_consistency_maximization/robot_measurements/robot_measurements.cpp
        src/pairwise_consistency_maximization/robot_measurements/interrobot_measurements.cpp
        src/pairwise_consistency_maximization/robot_measurements/robot_local_map.cpp
        )
target_link_libraries(robot_measurements
        graph_utils
        )

# Paiwise Consistency computation library
add_library(pairwise_consistency
   src/pairwise_consistency_maximization/pairwise_consistency/pairwise_consistency.cpp
)
target_link_libraries(pairwise_consistency
       robot_measurements
)

# Maximum Clique Solver
add_library(fast_max-clique_finder
    src/pairwise_consistency_maximization/third_parties/fast_max-clique_finder/src/findClique.h
    src/pairwise_consistency_maximization/third_parties/fast_max-clique_finder/src/graphIO.h
    src/pairwise_consistency_maximization/third_parties/fast_max-clique_finder/src/findClique.cpp
    src/pairwise_consistency_maximization/third_parties/fast_max-clique_finder/src/findCliqueHeu.cpp 
    src/pairwise_consistency_maximization/third_parties/fast_max-clique_finder/src/utils.cpp 
    src/pairwise_consistency_maximization/third_parties/fast_max-clique_finder/src/graphIO.cpp
)
target_compile_options(fast_max-clique_finder PRIVATE -w)

# Global map solver library
add_library(global_map_solver
   src/pairwise_consistency_maximization/global_map/global_map.cpp
)
target_link_libraries(global_map_solver
   graph_utils
   pairwise_consistency
   fast_max-clique_finder
)

# Distributed PCM library
add_library(distributed_pcm
        src/pairwise_consistency_maximization/distributed_pcm/distributed_pcm.cpp
)
target_link_libraries(distributed_pcm
        global_map_solver
)

# Distributed Mapper
add_library (distributed_mapper
        include/distributed_mapper/run_distributed_mapper.h
        src/distributed_mapper/distributed_mapper.cpp
        src/distributed_mapper/evaluation_utils.cpp
        include/distributed_mapper/distributed_mapper_utils.h
        src/distributed_mapper/distributed_mapper_utils.cpp
        src/distributed_mapper/run_distributed_mapper.cpp
        )

target_link_libraries(distributed_mapper
        ${Boost_LIBRARIES}
        gtsam
        distributed_pcm)


add_library(global_manager SHARED
  src/global_manager.cpp
  src/kdtree.cpp
  src/pose_graph_tool.cpp
  ${CUDA_INCLUDE_DIRS}
  # ${OPEN3D_INCLUDE_DIRS}
  # src/MultiRobotUtils.cpp
  # src/DistributedMapper.cpp
)


add_dependencies(global_manager ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(global_manager ${catkin_LIBRARIES} ${PCL_LIBRARIES} gtsam distributed_mapper fftw3)

add_executable(global_manager_node
  src/global_manager_node.cpp
)
add_dependencies(global_manager_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(global_manager_node global_manager )
