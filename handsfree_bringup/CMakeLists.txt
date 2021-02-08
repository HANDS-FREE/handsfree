cmake_minimum_required(VERSION 2.8.3)
project(handsfree_bringup)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

install(DIRECTORY demo robot_para rviz scripts
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

