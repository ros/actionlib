include(${CMAKE_CURRENT_BINARY_DIR}/package.cmake)


# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)
include(${actionlib_msgs_SOURCE_DIR}/cmake/rosbuild2.cmake)
#set the default path for built executables to the "bin" directory
#set the default path for built libraries to the "lib" directory
rosbuild_add_library(${PROJECT_NAME} src/connection_monitor.cpp src/goal_id_generator.cpp)
rosbuild_add_boost_directories()
include_directories(${PROJECT_SOURCE_DIR}/msg/cpp/)
rosbuild_link_boost(${PROJECT_NAME} thread)

add_subdirectory(test EXCLUDE_FROM_ALL)


