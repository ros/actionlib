cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)

rosbuild_init()
# rosbuild_genmsg()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)

#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

rosbuild_add_library(${PROJECT_NAME} src/connection_monitor.cpp)

include_directories(${PROJECT_SOURCE_DIR}/msg/cpp/)
rosbuild_add_executable(ref_server test/ref_server.cpp)

rosbuild_add_executable(simple_execute_ref_server test/simple_execute_ref_server.cpp)

rosbuild_add_executable(simple_client_test test/simple_client_test.cpp)
rosbuild_add_gtest_build_flags(simple_client_test)
target_link_libraries(simple_client_test ${PROJECT_NAME})

rosbuild_add_rostest(${CMAKE_CURRENT_SOURCE_DIR}/test/ref_server_test.launch)
rosbuild_add_rostest(${CMAKE_CURRENT_SOURCE_DIR}/test/simple_execute_ref_server_test.launch)

add_subdirectory(test EXCLUDE_FROM_ALL)
