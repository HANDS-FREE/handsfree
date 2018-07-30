#!/bin/bash

if grep -q "HANDSFREE_ROBOT_MODEL"  ~/.bashrc 
then   
echo -e "\n" 
else  
echo "### MODEL type [mini, stone_v2, stone_v3, giraffe]" >> ~/.bashrc
echo "export HANDSFREE_ROBOT_MODEL=stone_v3" >> ~/.bashrc
echo -e "\n" 
fi  

