#!/bin/bash

sudo usermod -a -G dialout $USER

sudo apt-get install ros-indigo-turtlebot-bringup ros-indigo-turtlebot-create-desktop ros-indigo-openni-* ros-indigo-openni2-* ros-indigo-freenect-* ros-indigo-usb-cam ros-indigo-laser-* ros-indigo-hokuyo-node ros-indigo-audio-common gstreamer0.10-pocketsphinx ros-indigo-pocketsphinx ros-indigo-slam-gmapping ros-indigo-joystick-drivers python-rosinstall ros-indigo-orocos-kdl ros-indigo-python-orocos-kdl python-setuptools ros-indigo-dynamixel-motor-*  ros-indigo-depthimage-to-laserscan ros-indigo-arbotix-* ros-indigo-turtlebot-teleop ros-indigo-move-base ros-indigo-map-server ros-indigo-fake-localization ros-indigo-amcl git subversion mercurial

sudo apt-get install ros-indigo-ros-controllers

#rbx1-prereq.sh
sudo apt-get install ros-indigo-turtlebot-bringup \
ros-indigo-turtlebot-create-desktop ros-indigo-openni-* \
ros-indigo-openni2-* ros-indigo-freenect-* ros-indigo-usb-cam \
ros-indigo-laser-* ros-indigo-hokuyo-node \
ros-indigo-audio-common gstreamer0.10-pocketsphinx \
ros-indigo-pocketsphinx ros-indigo-slam-gmapping \
ros-indigo-joystick-drivers python-rosinstall \
ros-indigo-orocos-kdl ros-indigo-python-orocos-kdl \
python-setuptools ros-indigo-dynamixel-motor-* \
libopencv-dev python-opencv ros-indigo-vision-opencv \
ros-indigo-depthimage-to-laserscan ros-indigo-arbotix-* \
ros-indigo-turtlebot-teleop ros-indigo-move-base \
ros-indigo-map-server ros-indigo-fake-localization \
ros-indigo-amcl git subversion mercurial

#rbx2-prereq.sh
sudo apt-get install ros-indigo-arbotix ros-indigo-openni-camera \
ros-indigo-dynamixel-motor ros-indigo-rosbridge-suite \
ros-indigo-mjpeg-server ros-indigo-rgbd-launch \
ros-indigo-moveit-full ros-indigo-moveit-ikfast \
ros-indigo-turtlebot-* ros-indigo-kobuki-* ros-indigo-moveit-python \
python-pygraph python-pygraphviz python-easygui \
mini-httpd ros-indigo-laser-pipeline ros-indigo-ar-track-alvar \
ros-indigo-laser-filters ros-indigo-hokuyo-node \
ros-indigo-depthimage-to-laserscan \
ros-indigo-gazebo-ros ros-indigo-gazebo-ros-pkgs \
ros-indigo-gazebo-msgs ros-indigo-gazebo-plugins \
ros-indigo-gazebo-ros-control ros-indigo-cmake-modules \
ros-indigo-kobuki-gazebo-plugins ros-indigo-kobuki-gazebo \
ros-indigo-smach ros-indigo-smach-ros ros-indigo-grasping-msgs \
ros-indigo-executive-smach ros-indigo-smach-viewer \
ros-indigo-robot-pose-publisher ros-indigo-tf2-web-republisher \
ros-indigo-move-base-msgs ros-indigo-fake-localization \
graphviz-dev libgraphviz-dev gv python-scipy liburdfdom-tools \
ros-indigo-laptop-battery-monitor ros-indigo-ar-track-alvar* \
ros-indigo-map-server ros-indigo-move-base* \
ros-indigo-simple-grasping

#libopencv-dev python-opencv  ros-kinetic-vision-opencv
