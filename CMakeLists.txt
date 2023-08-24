cmake_minimum_required(VERSION 3.0.2)
project(ttc_calculation)

find_package(catkin REQUIRED COMPONENTS
  rospy
  nav_msgs
  tf
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

install(PROGRAMS
  scripts/ttc_calculation_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(PROGRAMS
  scripts/ttc_calculation_multi_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
