cmake_minimum_required(VERSION 3.0.2)
project(control)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

## 노드 스크립트 설치
install(PROGRAMS
  scripts/controller_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## 런치 파일 설치
install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)
