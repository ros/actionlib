cmake_minimum_required(VERSION 2.4.6)

set(ROSPACK_MAKEDIST true)

if(ROSBUILD)
  include(rosbuild.cmake)
  return()
endif()
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)
rosbuild_find_ros_package(actionlib_msgs)
include(${actionlib_msgs_PACKAGE_PATH}/cmake/actionbuild.cmake)
genaction()

rosbuild_init()
rosbuild_genmsg()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)

#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

rosbuild_add_library(${PROJECT_NAME} src/connection_monitor.cpp
                                     src/goal_id_generator.cpp)

include_directories(${PROJECT_SOURCE_DIR}/msg/cpp/)

rosbuild_add_executable(add_two_ints_client test/add_two_ints_client.cpp)
target_link_libraries(add_two_ints_client ${PROJECT_NAME})

rosbuild_add_executable(add_two_ints_server test/add_two_ints_server.cpp)
target_link_libraries(add_two_ints_server ${PROJECT_NAME})

rosbuild_add_executable(ref_server test/ref_server.cpp)
target_link_libraries(ref_server ${PROJECT_NAME})

rosbuild_add_executable(server_goal_handle_destruction test/server_goal_handle_destruction.cpp)
rosbuild_add_gtest_build_flags(server_goal_handle_destruction)
rosbuild_declare_test(server_goal_handle_destruction)
target_link_libraries(server_goal_handle_destruction ${PROJECT_NAME})

rosbuild_add_executable(simple_execute_ref_server test/simple_execute_ref_server.cpp)
target_link_libraries(simple_execute_ref_server ${PROJECT_NAME})

rosbuild_add_executable(simple_client_test test/simple_client_test.cpp)
rosbuild_add_gtest_build_flags(simple_client_test)
rosbuild_declare_test(simple_client_test)
target_link_libraries(simple_client_test ${PROJECT_NAME})

rosbuild_add_executable(simple_client_wait_test test/simple_client_wait_test.cpp)
rosbuild_add_gtest_build_flags(simple_client_wait_test)
rosbuild_declare_test(simple_client_wait_test)
target_link_libraries(simple_client_wait_test ${PROJECT_NAME})

rosbuild_add_executable(simple_client_allocator_test test/simple_client_allocator_test.cpp)
rosbuild_add_gtest_build_flags(simple_client_allocator_test)
rosbuild_declare_test(simple_client_allocator_test)
target_link_libraries(simple_client_allocator_test ${PROJECT_NAME})

rosbuild_add_executable(action_client_destruction_test test/action_client_destruction_test.cpp)
rosbuild_add_gtest_build_flags(action_client_destruction_test)
rosbuild_declare_test(action_client_destruction_test)
target_link_libraries(action_client_destruction_test ${PROJECT_NAME})

rosbuild_add_gtest(destruction_guard_test test/destruction_guard_test.cpp)
target_link_libraries(destruction_guard_test ${PROJECT_NAME})

rosbuild_add_executable(test_cpp_simple_client_cancel_crash test/test_cpp_simple_client_cancel_crash.cpp)
rosbuild_add_gtest_build_flags(test_cpp_simple_client_cancel_crash)
rosbuild_declare_test(test_cpp_simple_client_cancel_crash)
target_link_libraries(test_cpp_simple_client_cancel_crash ${PROJECT_NAME})

rosbuild_add_rostest(${CMAKE_CURRENT_SOURCE_DIR}/test/ref_server_test.launch)
rosbuild_add_rostest(${CMAKE_CURRENT_SOURCE_DIR}/test/simple_execute_ref_server_test.launch)
rosbuild_add_rostest(${CMAKE_CURRENT_SOURCE_DIR}/test/test_python_simple_client.launch)
rosbuild_add_rostest(${CMAKE_CURRENT_SOURCE_DIR}/test/test_cpp_simple_client_allocator.launch)
rosbuild_add_rostest(${CMAKE_CURRENT_SOURCE_DIR}/test/test_cpp_action_client_destruction.launch)
rosbuild_add_rostest(${CMAKE_CURRENT_SOURCE_DIR}/test/test_server_goal_handle_destruction.launch)

rosbuild_add_rostest(${CMAKE_CURRENT_SOURCE_DIR}/test/test_cpp_simple_client_cancel_crash.launch)

rosbuild_add_rostest(${CMAKE_CURRENT_SOURCE_DIR}/test/test_imports.launch)

rosbuild_add_rostest(${CMAKE_CURRENT_SOURCE_DIR}/test/test_python_server_components.launch)
rosbuild_add_rostest(${CMAKE_CURRENT_SOURCE_DIR}/test/test_python_server.launch)
rosbuild_add_rostest(${CMAKE_CURRENT_SOURCE_DIR}/test/test_python_server2.launch)
rosbuild_add_rostest(${CMAKE_CURRENT_SOURCE_DIR}/test/test_python_server3.launch)
rosbuild_add_rostest(${CMAKE_CURRENT_SOURCE_DIR}/test/test_python_simple_server.launch)


# New Testing format
rosbuild_add_executable(exercise_simple_client test/exercise_simple_client.cpp)
rosbuild_add_gtest_build_flags(exercise_simple_client)
target_link_libraries(exercise_simple_client ${PROJECT_NAME})

# New tests, using the TestRequest action
rosbuild_add_rostest(${CMAKE_CURRENT_SOURCE_DIR}/test/test_exercise_simple_clients.launch)

# Append to CPACK_SOURCE_IGNORE_FILES a semicolon-separated list of
# directories (or patterns, but directories should suffice) that should 
# be excluded from the distro.  This is not the place to put things that
# should be ignored everywhere, like "build" directories; that happens in
# rosbuild/rosbuild.cmake.  Here should be listed packages that aren't
# ready for inclusion in a distro.
#
# This list is combined with the list in rosbuild/rosbuild.cmake.  Note
# that CMake 2.6 may be required to ensure that the two lists are combined
# properly.  CMake 2.4 seems to have unpredictable scoping rules for such
# variables.
#list(APPEND CPACK_SOURCE_IGNORE_FILES /core/experimental)

rosbuild_make_distribution(1.7.0)
