cmake_minimum_required(VERSION 2.8.3)
project(handsfree_msgs)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  std_msgs
)

 add_message_files(
   FILES
   robot_state.msg
   robot_time.msg
   parameters.msg
   dissensor.msg
   PoseEuler.msg
   arm_joints_state.msg
   arm_state.msg
 )


 add_service_files(
   FILES
   GetParamList.srv
   SetParamList.srv
   SpecialTurn.srv
   SpecialMove.srv
 )

 generate_messages(
   DEPENDENCIES
   std_msgs
 )

catkin_package(
  CATKIN_DEPENDS message_runtime 
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)
