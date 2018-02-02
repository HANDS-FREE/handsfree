#!/bin/bash

#安装HandsFree ROS包
echo 请确保已经成功安装indigo
sleep 1
mkdir -p ~/ros_handsfree/src
git clone https://github.com/HANDS-FREE/handsfree.git ~/ros_handsfree/src
cd ~/ros_handsfree/src/Documentation
sh environment_config.sh -y -y
sh rbx1-prereq.sh
sh rbx2-prereq.sh
cd ../handsfree_hw
pathofhf=`pwd`
cd src/
sed -i "s|/home/kedou/ros_workspace/mobile_robot_ws/src/handsfree/handsfree_hw|$pathofhf|" main.cpp
cd ~/ros_handsfree/
catkin_make
echo "source ~/ros_handsfree/devel/setup.sh" >> ~/.bashrc
echo HandsFree 相关程序已经安装完毕
echo 安装路径为：~/ros_handsfree/src

