cmake_minimum_required(VERSION 3.0.2)
project(actionlib_tools)

find_package(catkin REQUIRED)

catkin_python_setup()

catkin_package()

catkin_install_python(PROGRAMS
  scripts/axclient.py
  scripts/axserver.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
