
cmake_minimum_required(VERSION 3.0.2)
project(traffic_light)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  morai_msgs
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

install(
  DIRECTORY traffic_light/
  DESTINATION ${CATKIN_PACKAGE_PYTHON_DESTINATION}/traffic_light
  FILES_MATCHING PATTERN "*.py"
)

install(PROGRAMS
  scripts/traffic_light_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)