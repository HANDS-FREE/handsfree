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
########################

relesenum=`grep DISTRIB_DESCRIPTION /etc/*-release | awk -F 'Ubuntu ' '{print $2}' | awk -F ' LTS' '{print $1}'`
echo "Your ubuntu version is $relesenum"

echo "更新系统源列表"
cd ~
version=`lsb_release -sc`
echo "Checking the Ubuntu version"
case $version in
  "trusty" | "xenial" | "bionic" | "focal")
  ;;
  *)
    echo "ERROR: This script will only work on Ubuntu Trusty(14.04) / Xenial(16.04) / Bionic(18.04) / Focal(20.04). Exit."
    exit 0
esac
sudo cp /etc/apt/sources.list /etc/apt/sources.list.backup
echo "使用163源"
echo "
deb http://mirrors.163.com/ubuntu/ ${version} main restricted universe multiverse
deb http://mirrors.163.com/ubuntu/ ${version}-security main restricted universe multiverse
deb http://mirrors.163.com/ubuntu/ ${version}-updates main restricted universe multiverse
deb http://mirrors.163.com/ubuntu/ ${version}-proposed main restricted universe multiverse
deb http://mirrors.163.com/ubuntu/ ${version}-backports main restricted universe multiverse
deb-src http://mirrors.163.com/ubuntu/ ${version} main restricted universe multiverse
deb-src http://mirrors.163.com/ubuntu/ ${version}-security main restricted universe multiverse
deb-src http://mirrors.163.com/ubuntu/ ${version}-updates main restricted universe multiverse
deb-src http://mirrors.163.com/ubuntu/ ${version}-proposed main restricted universe multiverse
deb-src http://mirrors.163.com/ubuntu/ ${version}-backports main restricted universe multiverse
 " | sudo tee /etc/apt/sources.list
sleep 1
sudo apt-get update
sleep 1
sudo apt-get purge libappstream3
sleep 1
sudo apt-get update
sleep 1

########################
echo "下载HandsFree ROS代码"
sudo apt-get install -y git
sleep 3
source .bashrc
sleep 1
mkdir -p ~/handsfree/handsfree_ros_ws/src
cd ~/handsfree/handsfree_ros_ws/src
echo "设置 HandsFree ROS安装路径为：~/handsfree/handsfree_ros_ws/src"
echo "下载Gitee上HandsFree的代码............"
git clone https://gitee.com/HANDS-FREE/handsfree.git
echo "代码下载完毕..........."

########################
echo "设置 HandsFree 开发环境"
sleep 3
cd ~/handsfree/handsfree_ros_ws/src/handsfree/Documentation/script/
echo "安装 ROS 基本环境"
sleep 3
bash ./ros_install_cn.sh $ROS_DISTRO
echo "安装 ROS 扩展环境"
sleep 3
bash ./ros_install_ext_cn.sh $ROS_DISTRO

########################
echo "安装 HandsFree ROS"
sleep 3
cd 
source .bashrc
source /opt/ros/$ROS_DISTRO/setup.bash
sleep 1

if [ "$ROS_DISTRO" = "noetic" ]
then
cd ~/handsfree/handsfree_ros_ws/src/handsfree
git checkout noetic
fi

sleep 1
cd ~/handsfree/handsfree_ros_ws/src/
catkin_init_workspace
sleep 3
cd ~/handsfree/handsfree_ros_ws
echo " " >> ~/.bashrc
echo "source ~/handsfree/handsfree_ros_ws/devel/setup.sh" >> ~/.bashrc
catkin_make
source ~/.bashrc
echo "HandsFree 相关程序已经安装完毕"
echo "安装路径为：~/handsfree"

########################
echo "设置usb规则"
cd ~/handsfree/handsfree_ros_ws/src/handsfree/Documentation/
bash set_usb_env.sh

cp ~/handsfree/handsfree_ros_ws/src/handsfree/Documentation/script/get_gazebo_model.sh ~/handsfree/

