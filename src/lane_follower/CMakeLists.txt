cmake_minimum_required(VERSION 3.0.2)
project(lane_follower)

## catkin
find_package(catkin REQUIRED COMPONENTS
  rospy
  cv_bridge
  sensor_msgs
  std_msgs
)

catkin_package()

## include
include_directories(
  ${catkin_INCLUDE_DIRS}
)

## 1) Python 모듈 설치
install(
  DIRECTORY lane_follower/
  DESTINATION ${CATKIN_PACKAGE_PYTHON_DESTINATION}/lane_follower
  FILES_MATCHING PATTERN "*.py"
)

## 2) 노드 스크립트 설치
install(PROGRAMS
  scripts/lane_follower_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## 3) 런치 파일 설치
install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)
