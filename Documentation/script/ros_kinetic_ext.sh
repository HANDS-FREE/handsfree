#!/bin/bash

install_command="sudo apt-get install -y"

#rbx1-prereq.sh
$install_command ros-kinetic-turtlebot-bringup \
 ros-kinetic-openni-* \
ros-kinetic-openni2-* ros-kinetic-freenect-*  \
ros-kinetic-laser-*  \
ros-kinetic-audio-common  \
 ros-kinetic-slam-gmapping \
ros-kinetic-joystick-drivers python-rosinstall \
ros-kinetic-orocos-kdl ros-kinetic-python-orocos-kdl \
python-setuptools ros-kinetic-dynamixel-motor-* \
libopencv-dev python-opencv ros-kinetic-vision-opencv \
ros-kinetic-depthimage-to-laserscan  \
ros-kinetic-turtlebot-teleop ros-kinetic-move-base \
ros-kinetic-map-server ros-kinetic-fake-localization \
ros-kinetic-amcl git subversion mercurial \
ros-kinetic-hokuyo3d

#rbx2-prereq.sh
$install_command ros-kinetic-openni-camera \
ros-kinetic-dynamixel-motor ros-kinetic-rosbridge-suite \
 ros-kinetic-rgbd-launch \
ros-kinetic-moveit  \
ros-kinetic-turtlebot-* ros-kinetic-kobuki-* ros-kinetic-moveit-python \
python-pygraph python-pygraphviz python-easygui \
mini-httpd ros-kinetic-laser-pipeline ros-kinetic-ar-track-alvar \
ros-kinetic-laser-filters  \
ros-kinetic-depthimage-to-laserscan ros-kinetic-shape-msgs \
ros-kinetic-gazebo-ros ros-kinetic-gazebo-ros-pkgs \
ros-kinetic-gazebo-msgs ros-kinetic-gazebo-plugins \
ros-kinetic-gazebo-ros-control ros-kinetic-cmake-modules \
ros-kinetic-kobuki-gazebo-plugins ros-kinetic-kobuki-gazebo \
ros-kinetic-smach ros-kinetic-smach-ros ros-kinetic-grasping-msgs \
ros-kinetic-executive-smach  \
ros-kinetic-robot-pose-publisher ros-kinetic-tf2-web-republisher \
ros-kinetic-move-base-msgs ros-kinetic-fake-localization \
graphviz-dev libgraphviz-dev gv python-scipy liburdfdom-tools \
ros-kinetic-laptop-battery-monitor ros-kinetic-ar-track-alvar* \
ros-kinetic-map-server ros-kinetic-move-base* \
ros-kinetic-simple-grasping ros-kinetic-manipulation-msgs

$install_command ros-kinetic-usb-cam ros-kinetic-ros-controllers ros-kinetic-driver-base  

source /opt/ros/kinetic/setup.bash
#libopencv-dev python-opencv  ros-kinetic-vision-opencv
