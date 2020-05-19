cmake_minimum_required(VERSION 3.0.2)
project(actionlib_tools)

find_package(catkin REQUIRED)

catkin_package()

catkin_install_python(PROGRAMS
  # These scripts require Python 2 dependency wx
  # scripts/axclient.py
  # scripts/axserver.py
  scripts/dynamic_action.py
  scripts/library.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
