cmake_minimum_required(VERSION 2.8.3)

project(handsfree_camera)

find_package(catkin REQUIRED)

catkin_package()

install(DIRECTORY launch monocular rgb-d rviz
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

