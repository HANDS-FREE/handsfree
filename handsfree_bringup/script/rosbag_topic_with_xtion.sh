#!/bin/bash

rosrun dynamic_reconfigure dynparam set /camera/driver data_skip 3

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
/tf \
/tf_static \
/camera/depth/camera_info \
/camera/rgb/camera_info \
/camera/rgb/image_raw \
/camera/depth/image_raw \






