#!/bin/bash

sudo cp ./script/usb_rules/handsfree-serial.rules /etc/udev/rules.d/
sudo cp ./script/usb_rules/56-orbbec-usb.rules /etc/udev/rules.d/56-orbbec-usb.rules
sudo udevadm control --reload-rules
sudo udevadm trigger --action=add
sudo service udev reload
sudo service udev restart

sudo usermod -a -G dialout $USER

sudo chmod 777 ../handsfree_ar_tags/nodes/*.py
sudo chmod 777 ../handsfree_smach/script/*.py
sudo chmod 777 ../handsfree_speech/nodes/*.py
sudo chmod 777 ../handsfree_teleop/scripts/*.py
sudo chmod 777 ../handsfree_tutorials/script/1_get_sensors/*.py
sudo chmod 777 ../handsfree_tutorials/script/2_base_control/*.py
sudo chmod 777 ../handsfree_tutorials/script/3_navigation/*.py
sudo chmod 777 ../handsfree_tutorials/script/4_visualization/*.py
sudo chmod 777 ../handsfree_tutorials/script/5_advance_app/*.py
sudo chmod 777 ../handsfree_tutorials/script/6_application/2_follow_me/*.py

