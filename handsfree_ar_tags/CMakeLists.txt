cmake_minimum_required(VERSION 2.8.3)

project(handsfree_ar_tags)

find_package(catkin REQUIRED)

catkin_package()

install(DIRECTORY config data launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(FILES ar_tags.rviz readme
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(PROGRAMS
  nodes/ar_follower.py
  nodes/ar_tags_cog.py
  nodes/head_tracker.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

