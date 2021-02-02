#!/bin/bash -eu

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

version=`lsb_release -sc`
echo ""
echo "INSTALLING ROS USING quick_ros_install --------------------------------"
echo ""
echo "Checking the Ubuntu version"
case $version in
  "trusty" | "xenial" | "bionic" | "focal")
  ;;
  *)
    echo "ERROR: This script will only work on Ubuntu Trusty(14.04) / Xenial(16.04) / Bionic(18.04) / Focal(20.04). Exit."
    exit 0
esac

relesenum=`grep DISTRIB_DESCRIPTION /etc/*-release | awk -F 'Ubuntu ' '{print $2}' | awk -F ' LTS' '{print $1}'`
if [ "$relesenum" = "14.04.2" ]
then
  echo "Your ubuntu version is $relesenum"
  echo "Intstall the libgl1-mesa-dev-lts-utopic package to solve the dependency issues for the ROS installation specifically on $relesenum"
  sudo apt-get install -y libgl1-mesa-dev-lts-utopic
else
  echo "Your ubuntu version is $relesenum"
fi

echo "Add the ROS repository"
case $ROS_DISTRO in
  "indigo" | "kinetic" | "melodic")
    if [ ! -e /etc/apt/sources.list.d/ros-latest.list ]; then
      sudo sh -c "echo \"deb http://mirrors.ustc.edu.cn/ros/ubuntu ${version} main\" > /etc/apt/sources.list.d/ros-latest.list"
    fi
esac
case $ROS_DISTRO in
  "noetic")
    if [ ! -e /etc/apt/sources.list.d/ros-latest.list ]; then
      sudo sh -c "echo \"deb http://mirrors.tuna.tsinghua.edu.cn/ros/ubuntu ${version} main\" > /etc/apt/sources.list.d/ros-latest.list"
    fi
esac

echo "Download the ROS keys"
roskey=`apt-key list | grep "ROS Builder"` && true # make sure it returns true
if [ -z "$roskey" ]; then
  echo "No ROS key, adding"
  sudo apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net:80 --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
fi

echo "Updating packages"
sudo apt-get update

echo "Installing ROS"

sudo apt install -y ros-$ROS_DISTRO-desktop-full
case $ROS_DISTRO in
  "indigo" | "kinetic" | "melodic")
    echo "sudo apt install -y python-rosdep python-rosinstall python-rosinstall-generator python-wstool build-essential"
    sudo apt install -y python-rosdep python-rosinstall python-rosinstall-generator python-wstool build-essential
esac
case $ROS_DISTRO in
  "noetic")
    echo "apt install python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool build-essential"
    sudo apt install python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool build-essential
esac


echo "Environment setup"
echo "source /opt/ros/$ROS_DISTRO/setup.bash" >> ~/.bashrc
source ~/.bashrc

# Only init if it has not already been done before
if [ ! -e /etc/ros/rosdep/sources.list.d/20-default.list ]; then
  sudo rosdep init
fi
rosdep update

echo "Done installing ROS"

exit 0
