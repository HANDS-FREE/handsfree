#!/bin/bash

sudo apt-get update
sudo apt-get install -y git
mkdir -p ~/handsfree/handsfree_ros_ws/src/
cd ~/handsfree/handsfree_ros_ws/src/
echo 设置 HandsFree 安装路径为： ~/handsfree
git clone https://github.com/HANDS-FREE/handsfree.git
cd ~/handsfree/handsfree_ros_ws/src/handsfree/Documentation
sudo cp ./script/sources_ubuntu_14_04.list /etc/apt/sources.list
sudo apt-get update
echo 设置 HandsFree 开发环境
sh ./script/auto_set_ubuntu14.04.sh
echo 安装 ROS 基本环境
sh ./script/ros_indigo_base.sh
echo 安装 ROS 扩展环境
sh ./script/ros_indigo_ext.sh

echo 嵌入式开发库 OpenRE 
cd ~/handsfree
git clone https://github.com/HANDS-FREE/OpenRE.git

echo 安装 HandsFree ROS 
cd ~/handsfree/handsfree_ros_ws/src/
catkin_init_workspace
cd ./handsfree/handsfree_ros_ws
catkin_make
echo "source ~/handsfree/handsfree_ros_ws/devel/setup.sh" >> ~/.bashrc
source ~/.bashrc
echo HandsFree 相关程序已经安装完毕
echo 安装路径为：~/handsfree
