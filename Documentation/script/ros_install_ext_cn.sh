#!/bin/bash

function usage {
    # Print out usage of this script.
    echo >&2 "usage: $0 [ROS distro (default: kinetic)"
    echo >&2 "          [-h|--help] Print help message."
    exit 0
}

# Parse command line. If the number of argument differs from what is expected, call `usage` function.
OPT=`getopt -o h -l help -- $*`
if [ $# != 1 ]; then
    usage
fi
eval set -- $OPT
while [ -n "$1" ] ; do
    case $1 in
        -h|--help) usage ;;
        --) shift; break;;
        *) echo "Unknown option($1)"; usage;;
    esac
done

ROS_DISTRO=$1
ROS_DISTRO=${ROS_DISTRO:="kinetic"}
install_command="sudo apt-get install -y"
########################

if [ "$ROS_DISTRO" = "kinetic" ]
then

#rbx1-prereq.sh
$install_command ros-kinetic-turtlebot-bringup \
 ros-kinetic-openni-* \
ros-kinetic-openni2-* ros-kinetic-freenect-*  \
ros-kinetic-laser-*  \
gstreamer1.0-pocketsphinx ros-kinetic-audio-common libasound2 \
 ros-kinetic-slam-gmapping \
ros-kinetic-joystick-drivers python-rosinstall \
ros-kinetic-orocos-kdl ros-kinetic-python-orocos-kdl \
python-setuptools ros-kinetic-dynamixel-motor-* \
libopencv-dev python-opencv ros-kinetic-vision-opencv \
ros-kinetic-depthimage-to-laserscan  \
ros-kinetic-turtlebot-teleop ros-kinetic-move-base \
ros-kinetic-map-server ros-kinetic-fake-localization \
ros-kinetic-amcl git subversion mercurial \
ros-kinetic-hokuyo3d ros-kinetic-cob-perception-msgs

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

$install_command ros-kinetic-usb-cam ros-kinetic-ros-controllers ros-kinetic-driver-base ros-kinetic-imu-tools ros-kinetic-rviz-imu-plugin

fi

if [ "$ROS_DISTRO" = "melodic" ]
then

#rbx1-prereq.sh
$install_command ros-melodic-openni-* \
ros-melodic-openni2-* ros-melodic-freenect-*  \
ros-melodic-laser-*  \
gstreamer1.0-pocketsphinx ros-melodic-audio-common libasound2 \
 ros-melodic-slam-gmapping \
ros-melodic-joystick-drivers python-rosinstall \
ros-melodic-orocos-kdl ros-melodic-python-orocos-kdl \
python-setuptools \
libopencv-dev python-opencv ros-melodic-vision-opencv \
ros-melodic-depthimage-to-laserscan  \
ros-melodic-move-base \
ros-melodic-map-server ros-melodic-fake-localization \
ros-melodic-amcl git subversion mercurial \
ros-melodic-hokuyo3d ros-melodic-cob-perception-msgs ros-melodic-dwa-local-planner

#rbx2-prereq.sh
$install_command ros-melodic-openni-camera \
ros-melodic-rosbridge-suite \
ros-melodic-rgbd-launch \
ros-melodic-moveit  \
ros-melodic-kobuki-* ros-melodic-moveit-python \
python-pygraph python-pygraphviz python-easygui \
mini-httpd ros-melodic-laser-pipeline ros-melodic-ar-track-alvar \
ros-melodic-laser-filters  \
ros-melodic-depthimage-to-laserscan ros-melodic-shape-msgs \
ros-melodic-gazebo-ros ros-melodic-gazebo-ros-pkgs \
ros-melodic-gazebo-msgs ros-melodic-gazebo-plugins \
ros-melodic-gazebo-ros-control ros-melodic-cmake-modules \
ros-melodic-smach ros-melodic-smach-ros ros-melodic-grasping-msgs \
ros-melodic-executive-smach  \
ros-melodic-tf2-web-republisher \
ros-melodic-move-base-msgs ros-melodic-fake-localization \
graphviz-dev libgraphviz-dev gv python-scipy liburdfdom-tools \
ros-melodic-laptop-battery-monitor ros-melodic-ar-track-alvar* \
ros-melodic-map-server ros-melodic-move-base* \
ros-melodic-simple-grasping

$install_command ros-melodic-usb-cam ros-melodic-ros-controllers ros-melodic-driver-base ros-melodic-imu-tools ros-melodic-rviz-imu-plugin

fi

if [ "$ROS_DISTRO" = "noetic" ]
then

#rbx1-prereq.sh
$install_command ros-noetic-openni-* \
ros-noetic-openni2-* \
ros-noetic-laser-*  \
gstreamer1.0-pocketsphinx ros-noetic-audio-common libasound2 \
 ros-noetic-slam-gmapping \
ros-noetic-joystick-drivers \
libopencv-dev python3-opencv ros-noetic-vision-opencv \
ros-noetic-move-base \
ros-noetic-map-server ros-noetic-fake-localization \
ros-noetic-amcl git subversion mercurial \
ros-noetic-hokuyo3d ros-noetic-cob-perception-msgs ros-noetic-dwa-local-planner

#rbx2-prereq.sh
$install_command ros-noetic-rosbridge-suite \
ros-noetic-rgbd-launch \
ros-noetic-moveit  \
ros-noetic-kobuki-* ros-noetic-moveit-python \
python3-easygui \
mini-httpd ros-noetic-laser-pipeline  \
ros-noetic-laser-filters  \
ros-noetic-shape-msgs \
ros-noetic-gazebo-ros ros-noetic-gazebo-ros-pkgs \
ros-noetic-gazebo-msgs ros-noetic-gazebo-plugins \
ros-noetic-gazebo-ros-control ros-noetic-cmake-modules \
ros-noetic-smach ros-noetic-smach-ros ros-noetic-grasping-msgs \
ros-noetic-executive-smach  \
ros-noetic-move-base-msgs ros-noetic-fake-localization \
graphviz-dev libgraphviz-dev gv python3-scipy liburdfdom-tools \
ros-noetic-map-server ros-noetic-move-base* \

$install_command ros-noetic-usb-cam ros-noetic-ros-controllers ros-noetic-imu-tools ros-noetic-rviz-imu-plugin

fi

source /opt/ros/$ROS_DISTRO/setup.bash

