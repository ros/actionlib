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
rosbuild_genmsg()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)

#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

rosbuild_add_library(${PROJECT_NAME} src/connection_monitor.cpp
                                     src/goal_id_generator.cpp)

include_directories(${PROJECT_SOURCE_DIR}/msg/cpp/)
rosbuild_add_executable(ref_server test/ref_server.cpp)
target_link_libraries(ref_server ${PROJECT_NAME})

rosbuild_add_executable(simple_execute_ref_server test/simple_execute_ref_server.cpp)
target_link_libraries(simple_execute_ref_server ${PROJECT_NAME})

rosbuild_add_executable(simple_client_test test/simple_client_test.cpp)
rosbuild_add_gtest_build_flags(simple_client_test)
target_link_libraries(simple_client_test ${PROJECT_NAME})

rosbuild_add_executable(simple_client_wait_test test/simple_client_wait_test.cpp)
rosbuild_add_gtest_build_flags(simple_client_wait_test)
target_link_libraries(simple_client_wait_test ${PROJECT_NAME})

rosbuild_add_executable(simple_client_allocator_test test/simple_client_allocator_test.cpp)
rosbuild_add_gtest_build_flags(simple_client_allocator_test)
target_link_libraries(simple_client_allocator_test ${PROJECT_NAME})

rosbuild_add_gtest(destruction_guard_test test/destruction_guard_test.cpp)
#rosbuild_add_gtest_build_flags(destruction_guard_test)
target_link_libraries(destruction_guard_test ${PROJECT_NAME})

rosbuild_add_rostest(${CMAKE_CURRENT_SOURCE_DIR}/test/ref_server_test.launch)
rosbuild_add_rostest(${CMAKE_CURRENT_SOURCE_DIR}/test/simple_execute_ref_server_test.launch)
rosbuild_add_rostest(${CMAKE_CURRENT_SOURCE_DIR}/test/test_python_simple_client.launch)
rosbuild_add_rostest(${CMAKE_CURRENT_SOURCE_DIR}/test/test_cpp_simple_client_allocator.launch)

rosbuild_add_rostest(${CMAKE_CURRENT_SOURCE_DIR}/test/test_imports.launch)

rosbuild_add_rostest(${CMAKE_CURRENT_SOURCE_DIR}/test/test_python_server_components.launch)
rosbuild_add_rostest(${CMAKE_CURRENT_SOURCE_DIR}/test/test_python_server.launch)
rosbuild_add_rostest(${CMAKE_CURRENT_SOURCE_DIR}/test/test_python_server2.launch)
rosbuild_add_rostest(${CMAKE_CURRENT_SOURCE_DIR}/test/test_python_simple_server.launch)



add_subdirectory(test EXCLUDE_FROM_ALL)
