#!/bin/bash

rosbag record -O subset /base_scan \
/handsfree/robot_state  \
/mobile_base/joint_states  \
/mobile_base/mobile_base_controller/cmd_vel \
/mobile_base/mobile_base_controller/odom  \
/mobile_base/pitch_position_controller/command \
/mobile_base/yaw_position_controller/command  \
/rosout \
/rosout_agg \
/scan \
/handsfree/imu \
/tf \
/tf_static \
/map \

