#!/bin/bash

sudo usermod -a -G dialout $USER

sudo apt-get install ros-indigo-turtlebot-bringup ros-indigo-turtlebot-create-desktop ros-indigo-openni-* ros-indigo-openni2-* ros-indigo-freenect-* ros-indigo-usb-cam ros-indigo-laser-* ros-indigo-hokuyo-node ros-indigo-audio-common gstreamer0.10-pocketsphinx ros-indigo-pocketsphinx ros-indigo-slam-gmapping ros-indigo-joystick-drivers python-rosinstall ros-indigo-orocos-kdl ros-indigo-python-orocos-kdl python-setuptools ros-indigo-dynamixel-motor-*  ros-indigo-depthimage-to-laserscan ros-indigo-arbotix-* ros-indigo-turtlebot-teleop ros-indigo-move-base ros-indigo-map-server ros-indigo-fake-localization ros-indigo-amcl git subversion mercurial

sudo apt-get install ros-indigo-ros-controllers

#libopencv-dev python-opencv  ros-indigo-vision-opencv
