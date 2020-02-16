cmake_minimum_required(VERSION 2.8.3)

project(handsfree_vision)

find_package(catkin REQUIRED)

catkin_package()

install(DIRECTORY cartographer fake_laser rgbdslam rviz
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

