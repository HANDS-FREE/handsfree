#!/bin/bash

mkdir -p ~/ros_handsfree/src
git clone https://github.com/HANDS-FREE/handsfree.git ~/ros_handsfree/src
cd ~/ros_handsfree/
catkin_make
echo "source ~/ros_handsfree/devel/setup.sh" >> ~/.bashrc
source ~/.bashrc
echo HandsFree 相关程序已经安装完毕
echo 安装路径为：~/ros_handsfree/src
