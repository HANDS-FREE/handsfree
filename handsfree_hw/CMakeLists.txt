cmake_minimum_required(VERSION 2.8.3)
project(handsfree_hw)

find_package(catkin REQUIRED COMPONENTS
  controller_manager
  geometry_msgs
  hardware_interface
  realtime_tools
  roscpp
  std_msgs
  handsfree_msgs
)

add_definitions(-std=c++11)

catkin_package(   
  INCLUDE_DIRS include
#  LIBRARIES handsfree_hw
#  CATKIN_DEPENDS controller_manager geometry_msgs hardware_interface realtime_tools roscpp std_msgs
#  DEPENDS system_lib
)

execute_process(COMMAND sh ${CMAKE_CURRENT_SOURCE_DIR}/env.sh)

include_directories(
  include 
  ${catkin_INCLUDE_DIRS}
  include/sdk/robolink/
  include/sdk/robot_abstract/
)

add_executable(handsfree_hw_node src/main.cpp src/transport_serial.cpp include/sdk/robolink/robolink.cpp include/sdk/robolink/state_machine.cpp src/hf_hw.cpp src/hf_hw_ros.cpp)

add_dependencies(handsfree_hw_node handsfree_msgs_gencpp)

target_link_libraries(handsfree_hw_node ${catkin_LIBRARIES})

install(FILES env.sh
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY config launch
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY include/handsfree_hw/ include/sdk
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(TARGETS handsfree_hw_node
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

