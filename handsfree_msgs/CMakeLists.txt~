cmake_minimum_required(VERSION 2.8.3)
project(handsfree_msgs)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  sensor_msgs
  std_msgs
)

 add_message_files(
   FILES
   robot_state.msg
   parameters.msg
 )


 add_service_files(
   FILES
   GetParamList.srv
   SetParamList.srv
 )

 generate_messages(
   DEPENDENCIES
   sensor_msgs   std_msgs
 )

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES handsfree_msgs
  CATKIN_DEPENDS message_generation sensor_msgs std_msgs 
  DEPENDS system_lib
)


include_directories(
  ${catkin_INCLUDE_DIRS}
)
