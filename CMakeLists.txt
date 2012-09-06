cmake_minimum_required(VERSION 2.8)
project(actionlib)
find_package(catkin REQUIRED COMPONENTS
  rostime roscpp_serialization roscpp_traits cpp_common # serialization stuff
  roscpp rosconsole xmlrpcpp #roscpp stuff
  actionlib_msgs rosunit rostest)
find_package(Boost REQUIRED COMPONENTS thread)
catkin_stack()
catkin_project(actionlib
  LIBRARIES actionlib
  INCLUDE_DIRS include
  )

include_directories(include ${catkin_INCLUDE_DIRS})
link_directories(${catkin_LIBRARY_DIRS})

add_library(actionlib src/connection_monitor.cpp src/goal_id_generator.cpp)
target_link_libraries(actionlib ${catkin_LIBRARIES} ${Boost_LIBRARIES})

catkin_python_setup()

add_action_files(DIRECTORY action FILES Test.action TestRequest.action TwoInts.action)
generate_messages(DEPENDENCIES std_msgs actionlib_msgs)
add_dependencies(actionlib actionlib_gencpp)

install(PROGRAMS tools/axclient.py tools/axserver.py tools/dynamic_action.py tools/library.py
  DESTINATION ${CATKIN_PROJECT_BIN_DESTINATION}) 
install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PROJECT_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PROJECT_LIB_DESTINATION})
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PROJECT_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h")

add_subdirectory(test)
