cmake_minimum_required(VERSION 2.8.3)
project(handsfree_dataset)

find_package(catkin REQUIRED)

catkin_package()

install(DIRECTORY rosbags videos
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

