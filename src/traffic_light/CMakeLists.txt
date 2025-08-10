
cmake_minimum_required(VERSION 3.0.2)
project(traffic_light)

## Find catkin macros and libraries. This also finds other
## catkin packages automatically via ``package.xml``.
find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
)

## Declare a catkin package.  We do not need to export any
## libraries or include directories for this simple Python‑only package,
## but the call must exist to correctly generate package metadata.
catkin_package()

## Specify additional locations of header files.  Although this
## package is purely Python, including the catkin include directories
## here maintains consistency with other packages in this repository.
include_directories(
  ${catkin_INCLUDE_DIRS}
)

## Install Python modules.  The ``traffic_light`` folder contains
## pure‑Python helper classes (e.g., perception and decision logic).
install(
  DIRECTORY traffic_light/
  DESTINATION ${CATKIN_PACKAGE_PYTHON_DESTINATION}/traffic_light
  FILES_MATCHING PATTERN "*.py"
)

## Install executable scripts.  These scripts are marked executable and
## will be placed into ``devel/lib/${PROJECT_NAME}`` during a devel build
## and ``lib/${PROJECT_NAME}`` in an installed workspace.  The node
## ``traffic_light_node.py`` sets up the ROS publishers/subscribers and
## ties together the perception and decision modules.
install(PROGRAMS
  scripts/traffic_light_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Install launch files.  Launch files allow users to start the node
## and set parameters such as the subscribed traffic light topic.  They
## belong under ``share/${PROJECT_NAME}/launch`` in an installed tree.
install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)