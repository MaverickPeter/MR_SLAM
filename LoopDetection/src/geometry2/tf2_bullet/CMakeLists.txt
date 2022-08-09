cmake_minimum_required(VERSION 3.0.2)
project(tf2_bullet)

find_package(PkgConfig REQUIRED)

set(bullet_FOUND 0)
pkg_check_modules(bullet bullet)
if(NOT bullet_FOUND)
    # windows build bullet3 doesn't come with pkg-config by default and it only comes with CMake config files
    # so pkg_check_modules will fail
    find_package(bullet REQUIRED)

    # https://github.com/bulletphysics/bullet3/blob/master/BulletConfig.cmake.in
    set(bullet_INCLUDE_DIRS "${BULLET_INCLUDE_DIRS}")
endif()

find_package(catkin REQUIRED COMPONENTS geometry_msgs tf2)

include_directories(include ${bullet_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})

catkin_package(INCLUDE_DIRS include ${bullet_INCLUDE_DIRS})

install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})


if(CATKIN_ENABLE_TESTING)

catkin_add_gtest(test_bullet test/test_tf2_bullet.cpp)
target_link_libraries(test_bullet ${catkin_LIBRARIES} ${GTEST_LIBRARIES})

endif()
