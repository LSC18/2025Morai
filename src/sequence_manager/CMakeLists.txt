cmake_minimum_required(VERSION 3.0.2)
project(sequence_manager)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
)

catkin_package()

# 파이썬 실행 스크립트 설치
catkin_install_python(PROGRAMS
  scripts/sequence_manager_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# launch 파일 설치
install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
