cmake_minimum_required(VERSION 2.8)
project(actionlib)
find_package(catkin)
catkin_project(actionlib
  LIBRARIES actionlib
  INCLUDE_DIRS include
  )

find_package(ROS COMPONENTS
  rostime roscpp_serialization roscpp_traits cpp_common # serialization stuff
  roscpp rosconsole xmlrpcpp #roscpp stuff
  actionlib_msgs rosunit rostest)

include_directories(include)
include_directories(${ROS_INCLUDE_DIRS})

# actionlib library
add_library(actionlib SHARED src/connection_monitor.cpp
                      src/goal_id_generator.cpp)

target_link_libraries(actionlib ${ROS_LIBRARIES})

catkin_export_python(src)

# action files
add_action_files( DIRECTORY action FILES Test.action TestRequest.action TwoInts.action )

generate_messages(DEPENDENCIES std_msgs actionlib_msgs)
add_dependencies(actionlib actionlib_gencpp)

install(TARGETS ${PROJECT_NAME}
        LIBRARY DESTINATION lib)
install(DIRECTORY include/
        DESTINATION include
        FILES_MATCHING PATTERN "*.h"
        PATTERN ".svn" EXCLUDE)
install(FILES manifest.xml stack.xml
        DESTINATION share/${PROJECT_NAME})

# tests
find_package(GTest)
include_directories(${GTEST_INCLUDE_DIRS})

add_executable(actionlib-add_two_ints_server EXCLUDE_FROM_ALL test/add_two_ints_server.cpp)
target_link_libraries(actionlib-add_two_ints_server ${PROJECT_NAME} ${GTEST_LIBRARIES})

add_executable(actionlib-ref_server EXCLUDE_FROM_ALL test/ref_server.cpp)
target_link_libraries(actionlib-ref_server ${PROJECT_NAME})

add_executable(actionlib-simple_client_test EXCLUDE_FROM_ALL test/simple_client_test.cpp)
target_link_libraries(actionlib-simple_client_test ${PROJECT_NAME} ${GTEST_LIBRARIES})

add_executable(actionlib-simple_execute_ref_server EXCLUDE_FROM_ALL test/simple_execute_ref_server.cpp)
target_link_libraries(actionlib-simple_execute_ref_server ${PROJECT_NAME})

add_executable(actionlib-server_goal_handle_destruction EXCLUDE_FROM_ALL test/server_goal_handle_destruction.cpp)
target_link_libraries(actionlib-server_goal_handle_destruction ${PROJECT_NAME} ${GTEST_LIBRARIES})

add_executable(actionlib-simple_client_wait_test EXCLUDE_FROM_ALL test/simple_client_wait_test.cpp)
target_link_libraries(actionlib-simple_client_wait_test ${PROJECT_NAME} ${GTEST_LIBRARIES})

add_executable(actionlib-simple_client_allocator_test EXCLUDE_FROM_ALL test/simple_client_allocator_test.cpp)
target_link_libraries(actionlib-simple_client_allocator_test ${PROJECT_NAME} ${GTEST_LIBRARIES})

add_executable(actionlib-action_client_destruction_test EXCLUDE_FROM_ALL test/action_client_destruction_test.cpp)
target_link_libraries(actionlib-action_client_destruction_test ${PROJECT_NAME} ${GTEST_LIBRARIES})

add_executable(actionlib-test_cpp_simple_client_cancel_crash EXCLUDE_FROM_ALL test/test_cpp_simple_client_cancel_crash.cpp)
target_link_libraries(actionlib-test_cpp_simple_client_cancel_crash ${PROJECT_NAME} ${GTEST_LIBRARIES})

add_executable(actionlib-exercise_simple_client EXCLUDE_FROM_ALL test/exercise_simple_client.cpp)
target_link_libraries(actionlib-exercise_simple_client ${PROJECT_NAME} ${GTEST_LIBRARIES})

add_dependencies(tests
  actionlib-add_two_ints_server
  actionlib-ref_server
  actionlib-simple_client_test
  actionlib-simple_execute_ref_server
  actionlib-server_goal_handle_destruction
  actionlib-simple_client_wait_test
  actionlib-simple_client_allocator_test
  actionlib-action_client_destruction_test
  actionlib-test_cpp_simple_client_cancel_crash
  actionlib-exercise_simple_client
  )

add_rostest(${CMAKE_CURRENT_SOURCE_DIR}/test/ref_server_test.launch)
add_rostest(${CMAKE_CURRENT_SOURCE_DIR}/test/simple_execute_ref_server_test.launch)
add_rostest(${CMAKE_CURRENT_SOURCE_DIR}/test/test_python_simple_client.launch)
add_rostest(${CMAKE_CURRENT_SOURCE_DIR}/test/test_cpp_simple_client_allocator.launch)
add_rostest(${CMAKE_CURRENT_SOURCE_DIR}/test/test_cpp_action_client_destruction.launch)
add_rostest(${CMAKE_CURRENT_SOURCE_DIR}/test/test_server_goal_handle_destruction.launch)
add_rostest(${CMAKE_CURRENT_SOURCE_DIR}/test/test_cpp_simple_client_cancel_crash.launch)
add_rostest(${CMAKE_CURRENT_SOURCE_DIR}/test/test_imports.launch)
add_rostest(${CMAKE_CURRENT_SOURCE_DIR}/test/test_python_server_components.launch)
add_rostest(${CMAKE_CURRENT_SOURCE_DIR}/test/test_python_server.launch)
add_rostest(${CMAKE_CURRENT_SOURCE_DIR}/test/test_python_server2.launch)
add_rostest(${CMAKE_CURRENT_SOURCE_DIR}/test/test_python_server3.launch)
add_rostest(${CMAKE_CURRENT_SOURCE_DIR}/test/test_python_simple_server.launch)
add_rostest(${CMAKE_CURRENT_SOURCE_DIR}/test/test_exercise_simple_clients.launch)

add_gtest(actionlib-destruction_guard_test test/destruction_guard_test.cpp)
target_link_libraries(actionlib-destruction_guard_test ${PROJECT_NAME})
