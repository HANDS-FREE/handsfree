cmake_minimum_required(VERSION 2.8.3)
project(handsfree_controller)

find_package(catkin REQUIRED COMPONENTS
  controller_interface
  nav_msgs
  realtime_tools
  roscpp
  tf
  handsfree_hw
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
)

include_directories(
    include ${catkin_INCLUDE_DIRS}
    INCLUDE_DIRS include/handsfree_controller
)

add_library(base_simple_controller src/base_simple_controller.cpp)
target_link_libraries(base_simple_controller ${catkin_LIBRARIES})

add_library(omni_triangle_controller src/omni_triangle_controller.cpp src/odometry.cpp src/speed_limiter.cpp)
target_link_libraries(omni_triangle_controller ${catkin_LIBRARIES})

install(FILES base_simple_controller_plugins.xml omni_triangle_controller_plugins.xml
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY include
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(TARGETS base_simple_controller omni_triangle_controller
	ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
	LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
	RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
