#!/bin/bash

#安装HandsFree ROS包
echo 请确保已经成功安装indigo
sleep 1
mkdir -p /opt/ros_handsfree/src
git clone https://github.com/HANDS-FREE/handsfree.git /opt/ros_handsfree/src
cd /opt/ros_handsfree/src/Documentation
sh environment_config.sh -y -y
sh rbx1-prereq.sh
sh rbx2-prereq.sh
cd ../handsfree_hw/src/
find . -name 'main.cpp' | xargs perl -pi -e 's|/home/kedou/ros_workspace/mobile_robot_ws/src/handsfree/handsfree_hw|/opt/ros_handsfree/src/handsfree_hw|g'
cd /opt/ros_handsfree/
catkin_make
echo "source /opt/ros_handsfree/devel/setup.sh" >> ~/.bashrc
echo HandsFree 相关程序已经安装完毕
echo 安装路径为：/opt/ros_handsfree/src

