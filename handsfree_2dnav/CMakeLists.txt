cmake_minimum_required(VERSION 2.8.3)
project(handsfree_2dnav)

find_package(catkin REQUIRED)

catkin_package()

install(DIRECTORY launch map move_base_config rviz
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

