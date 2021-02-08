#!/usr/bin/env bash

#ps a |grep ros|grep -v grep|awk "{print $1}"|xargs kill -9
#ps -ef |grep ros|grep -v grep|awk "{print $2}"|xargs kill -9
#

red="\033[1;31m"
green="\033[1;33m"
yellow="\033[1;33m"
color_end="\033[0m"

ps3=`ls /dev/input/js*| wc -l`

source ~/.bashrc

if [ $ps3 -gt 1 ];then
    echo -e $green"    检测到" $p3 "个手柄"$color_end
    sleep .3
    echo -e $green"    程序可能运行不成功，请确保控制机器人的 ps3 手柄映射的 js0"$color_end
fi


if ls /dev/input/js0; then
    echo -e $yellow"    检测到手柄"$color_end
    sleep .3
else
    echo -e $red"    未检测到手柄设备，请检查是否插上手柄的 usb 接受器"$color_end
    sleep .3
    echo -e ""
    sleep .3
    echo -e "3 秒后将自动关闭该终端"
    sleep 3
    exit
fi

if ls /dev/HFRobot; then
    echo -e $yellow"    检测到机器人底盘"$color_end
    sleep .4
else
    echo -e $red"    没有检测到机器人底盘"$color_end
    sleep .4
    echo -e "3 秒后将自动关闭该终端"
    sleep 3
    exit
fi

if rosnode list | grep "/handsfree_hw_node"; then
    echo -e "    handsfree_hw_node 已经打开"
    sleep .3
else
    echo -e "    handsfree_hw_node 没有打开，现在将为你打开..."
    sleep .3
    roslaunch handsfree_hw handsfree_hw.launch&
    sleep 3
fi

roslaunch handsfree_hw ps3_teleop.launch&
sleep 3
echo ""
sleep .3
echo ""
sleep .3
echo -e $green"[提示] 现在您可以打开 ps3 手柄开始遥控了！"$color_end
sleep .3
echo ""
sleep .3
echo -e $green"[提示] 如果手柄失效，请检查急停开关是否打卡，是否有多个 js 设备"$color_end
sleep .3
echo ""
sleep .3
echo -e $green"[提示] 请不要通过鼠标直接关掉该终端！！！"$color_end
sleep .3
echo ""
sleep .3
echo ""
sleep .3
read -p `echo -e $red"按-Entrt-键退出..."$color_end`
ps -ef |grep launch|grep -v grep|awk '{print $2}'|xargs kill -9
ps -ef |grep opt|grep -v grep|awk '{print $2}'|xargs kill -9
ps -ef |grep .log|grep -v grep|awk '{print $2}'|xargs kill -9
sleep .3
