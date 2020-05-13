cmake_minimum_required(VERSION 2.8.3)
project(handsfree_gazebo)

find_package(catkin REQUIRED)

catkin_package()

install(DIRECTORY config launch map rviz urdf worlds
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

