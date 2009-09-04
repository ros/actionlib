cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

rospack(actionlib_tutorials)

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#uncomment if you have defined messages
genmsg()
#uncomment if you have defined services
#gensrv()

add_subdirectory(simple_action_servers)
add_subdirectory(simple_action_clients)



#common commands for building c++ executables and libraries
#rospack_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#rospack_add_boost_directories()
#rospack_link_boost(${PROJECT_NAME} thread)
#rospack_add_executable(example examples/example.cpp)
#target_link_libraries(example ${PROJECT_NAME})
