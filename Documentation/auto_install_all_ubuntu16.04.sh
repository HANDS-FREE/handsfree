#!/bin/bash

sudo apt-get update
sudo apt-get install -y git
mkdir -p ~/handsfree/handsfree_ros_ws/
cd ~/handsfree/handsfree_ros_ws/
echo 设置 HandsFree 安装路径为： ~/handsfree
git clone https://github.com/HANDS-FREE/handsfree.git src
sleep 1
cd ~/handsfree/handsfree_ros_ws/src/Documentation/script
sudo cp /etc/apt/sources.list /etc/apt/sources.list.backup
sudo cp ./sources_ubuntu_16_04.list /etc/apt/sources.list
sudo apt-get update
echo 设置 HandsFree 开发环境
sleep 1
sh ./auto_set_ubuntu16.04.sh
echo 安装 ROS 基本环境
sleep 1
sh ./ros_kinetic_base.sh
echo 安装 ROS 扩展环境
sleep 1
sh ./ros_kinetic_ext.sh

echo 安装 HandsFree ROS 
source ~/.bashrc 
sleep 1
cd ~/handsfree/handsfree_ros_ws/src/
catkin_init_workspace
sleep 1
cd ~/handsfree/handsfree_ros_ws
catkin_make
echo "source ~/handsfree/handsfree_ros_ws/devel/setup.sh" >> ~/.bashrc
source ~/.bashrc
echo HandsFree 相关程序已经安装完毕
echo 安装路径为：~/handsfree

sudo cp /opt/ros/kinetic/share/laser_filters/laser_filters_plugins.xml /opt/ros/kinetic/share/laser_filters_jsk_patch/

cd ~/handsfree/handsfree_ros_ws/src/Documentation/script
sudo cp ./60-persistent-serial.rules /etc/udev/rules.d/
sudo udevadm control --reload
