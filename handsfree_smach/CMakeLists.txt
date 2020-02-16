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

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(PROGRAMS
  script/__init__.py
  script/laser_detection.py
  script/LinearMove.py
  script/RadianTurn.py
  script/smach_concurrence.py
  script/smach_patrol.py
  script/smach_preemption.py
  script/smach_serial_square.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
