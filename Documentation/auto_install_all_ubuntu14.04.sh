#!/bin/bash

cd ~
wget https://raw.githubusercontent.com/HANDS-FREE/handsfree/master/Documentation/script/sources_ubuntu_14_04.list
echo 更新系统源列表
sudo cp /etc/apt/sources.list /etc/apt/sources.list.backup
sudo cp ./sources_ubuntu_14_04.list /etc/apt/sources.list
sudo apt-get update
sleep 3

sudo apt-get install -y git
sleep 3
source .bashrc
sleep 1
mkdir -p ~/handsfree/handsfree_ros_ws/src
cd ~/handsfree/handsfree_ros_ws/src
echo 设置 HandsFree ROS安装路径为：~/handsfree/handsfree_ros_ws/src
echo 下载Github上HandsFree的代码............
git clone https://github.com/HANDS-FREE/handsfree.git 
echo 代码下载完毕...........

echo 设置 HandsFree 开发环境
sleep 3
cd ~/handsfree/handsfree_ros_ws/src/handsfree/Documentation/script/
#bash ./ubuntu_14.04_base.sh
echo 安装 ROS 基本环境
sleep 3
bash ./ros_indigo_base.sh
echo 安装 ROS 扩展环境
sleep 3
bash ./ros_indigo_ext.sh

echo 安装 HandsFree ROS 
sleep 3
source /opt/ros/indigo/setup.bash
cd ~/handsfree/handsfree_ros_ws/src/
catkin_init_workspace
sleep 3
cd ~/handsfree/handsfree_ros_ws
catkin_make
echo "source ~/handsfree/handsfree_ros_ws/devel/setup.sh" >> ~/.bashrc
source ~/.bashrc
echo HandsFree 相关程序已经安装完毕
echo 安装路径为：~/handsfree

echo 设置usb规则
cd ~/handsfree/handsfree_ros_ws/src/handsfree/Documentation/
bash set_usb_env.sh

sudo cp /opt/ros/indigo/share/laser_filters/laser_filters_plugins.xml /opt/ros/indigo/share/laser_filters_jsk_patch/
