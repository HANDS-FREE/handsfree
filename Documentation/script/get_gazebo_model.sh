#!/bin/bash

mkdir -p ~/handsfree/
cd ~/handsfree/
echo 下载Gazebo模型
wget https://handsfree-mv.oss-cn-shenzhen.aliyuncs.com/handsfree_download/ros_course/gazebo_model.tar.bz2
sleep 3
tar -jxvf gazebo_model.tar.bz2

if grep -q "export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH"  ~/.bashrc 
then   
echo -e "\n" 
else 
echo "export GAZEBO_MODEL_PATH=\$GAZEBO_MODEL_PATH:/home/$USER/handsfree/gazebo_model/download:/home/$USER/handsfree/gazebo_model/others" >> ~/.bashrc
echo "export GAZEBO_RESOURCE_PATH=\$GAZEBO_RESOURCE_PATH" >> ~/.bashrc
fi
