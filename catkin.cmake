cmake_minimum_required(VERSION 2.8)
project(actionlib)

find_package(catkin)
find_package(actionlib_msgs)
find_package(PythonLibs)

include_directories(include)

add_library(actionlib src/connection_monitor.cpp
                      src/goal_id_generator.cpp)

target_link_libraries(actionlib ${PYTHON_LIBRARIES})

install_cmake_infrastructure(actionlib
  VERSION 0.0.0
  LIBRARIES actionlib
  INCLUDE_DIRS include
  )
