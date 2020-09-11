#!/bin/bash

if grep -q "HANDSFREE_ROBOT_MODEL"  ~/.bashrc 
then   
echo -e "\n" 
else 
echo "#HANDSFREE_ROBOT_MODEL: support all HandsFree open source robots" >> ~/.bashrc
echo "#####Mini Series: mini mini_omni3 mini_mecanum4 mini_carlike" >> ~/.bashrc
echo "#####Stone Series: stone stone_v2 stone_v3 stone_v3_omni3" >> ~/.bashrc
echo "#####Giraffe Series: giraffe giraffe_v2" >> ~/.bashrc
echo "###请修改HANDSFREE_ROBOT_MODEL是你正在使用的机器人型号" >> ~/.bashrc
echo "export HANDSFREE_ROBOT_MODEL=stone_v3" >> ~/.bashrc
echo -e "\n" 
fi  

