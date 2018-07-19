cmake_minimum_required(VERSION 2.8.3)
project(handsfree_smach)

find_package(catkin REQUIRED COMPONENTS
  actionlib
  geometry_msgs
  nav_msgs
  roscpp
  rospy
  smach
  smach_ros
  std_msgs
  tf
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

