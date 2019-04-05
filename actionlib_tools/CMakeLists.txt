cmake_minimum_required(VERSION 2.8.3)
project(actionlib_tools)

find_package(catkin REQUIRED)

catkin_package()

catkin_install_python(PROGRAMS scripts/axclient.py scripts/axserver.py scripts/dynamic_action.py scripts/library.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
