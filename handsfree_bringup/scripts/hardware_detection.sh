#!/usr/bin/env bash

#ps a |grep ros|grep -v grep|awk '{print $1}'|xargs kill -9
#ps -ef |grep ros|grep -v grep|awk '{print $2}'|xargs kill -9
ps -ef |grep launch|grep -v grep|awk '{print $2}'|xargs kill -9
ps -ef |grep opt|grep -v grep|awk '{print $2}'|xargs kill -9
ps -ef |grep .log|grep -v grep|awk '{print $2}'|xargs kill -9

red="\033[1;31m"
green="\033[1;33m"
yellow="\033[1;33m"
color_end="\033[0m"

fail_sum=0
success_sum=0
fail=":"
success=":"

hardware1="底盘设备,"
hardware1_1="Octopus底盘设备,"
hardware2="雷达设备（前）,"
hardware2_1="雷达设备（后）,"
hardware3="Asus 相机设备,"
hardware4="机械臂设备,"

chassis="off"
lidar="off"
camera="off"
arm="off"

rviz="off"

source ~/.bashrc

echo -e ""
sleep .3
echo -e "检测到您软件中定义的器人型号为"$yellow$HANDSFREE_ROBOT_MODEL$color_end
sleep .3
echo -e ""
sleep .3
echo -e "现在将开始搜索机器人所连接的硬件设备"
sleep 1
echo -e ""
sleep .3
if ls /dev/HFRobotOctopus; then
    echo -e "    "$yellow"检测到机器人Octopus底盘设备..."$color_end
    let success_sum+=1
    success=$success""$hardware1_1
    chassis="on"
    rviz="on"
else
    echo -e "    "$red"未能检测到机器人Octopus底盘设备..."$color_end
    let fail_sum+=1
    fail=$fail""$hardware1_1
fi
sleep .3
echo -e ""
sleep .3
if ls /dev/HFRobot; then
    echo -e "    "$yellow"检测到机器人底盘设备..."$color_end
    let success_sum+=1
    success=$success""$hardware1
    chassis="on"
    rviz="on"
else
    echo -e "    "$red"未能检测到机器人底盘设备..."$color_end
    let fail_sum+=1
    fail=$fail""$hardware1
fi

sleep .3
echo -e ""
sleep .3
if ls /dev/rplidar; then
    echo -e "    "$yellow"检测到机器人雷达设备...（前）"$color_end
    let success_sum+=1
    success=$success""$hardware2
    lidar="on"
    rviz="on"
else
    echo -e "    "$red"未检测到机器人雷达设备...（前）"$color_end
    let fail_sum+=1
    fail=$fail""$hardware2
fi

sleep .3
echo -e ""
sleep .3
if ls /dev/rplidar2; then
    echo -e "    "$yellow"检测到机器人雷达设备...（后）"$color_end
    let success_sum+=1
    success=$success""$hardware2_1
    lidar="on"
    rviz="on"
else
    echo -e "    "$red"未检测到机器人雷达设备...（后）"$color_end
    let fail_sum+=1
    fail=$fail""$hardware2_1
fi

sleep .3
echo -e ""
sleep .3
if lsusb |grep  ASUS; then
    echo -e "    "$yellow"检测到机器人摄像机设备..."$color_end
    let success_sum+=1
    success=$success""$hardware3
    camera="on"
    rviz="on"
else
    echo -e "    "$red"未检测到机器人摄像机设备..."$color_end
    let fail_sum+=1
    fail=$fail""$hardware3
fi

sleep .3

echo -e ""
sleep .3
if ls /dev/HFRobotArm; then
    echo -e "    "$yellow"检测到机器人机械臂设备..."$color_end
    let success_sum+=1
    success=$success""$hardware4
    arm="on"
    rviz="on"
else
    echo -e "    "$red"未检测到机器人机械臂设备..."$color_end
    let fail_sum+=1
    fail=$fail""$hardware4
fi
echo -e ""
sleep .3
echo -e " ================================================================"
sleep .3
echo -e "|"
sleep .3
echo -e "| 本次未成功检测到机器人的 "$fail_sum" 个硬件设备"
sleep .3
echo -e "|     分别为"$red$fail$color_end
sleep .3
echo -e "|"
sleep .3
echo -e "| 本次成功检测到机器人的 "$success_sum" 个硬件设备"
sleep .3
echo -e "|     分别为"$yellow$success$color_end
sleep .3
echo -e "|"
sleep .3
echo -e "| 未成功检测到设备的原因可能有以下几点:"
sleep .3
echo -e "|"
echo -e "|     1. 你所购买的机器人型号不包含这些硬件设备"
sleep .3
echo -e "|     2. 机器人的连接电脑的 usb 插口可能松动了"
sleep .3
echo -e "|     3. ..." 
sleep .3
echo -e "|"
sleep .3
echo -e " ================================================================"
sleep .3
echo -e "|                                                                |"
sleep .3
echo -e "|               https://edu.taobotics.com/wiki                   |"
sleep .3
echo -e "|                                                                |"
sleep .3
main(){
    echo -e " 提示框========================================================== "
    sleep .3
    echo -e "|                                                                |"
    sleep .3
    echo -e "|    你可以通过下面选项打开 rviz 可视化界面直观测试或退出该程序"
    sleep .3
    echo -e "|                                                                |"
    sleep .3
    echo -e "|    1. 打开 rviz 可视化界面"
    sleep .3
    echo -e "|    2. 退出终端"
    sleep .3
    echo -e "|                                                                |"
    sleep .3
    echo -e "|                                                                |"
    sleep .3
    echo -e "|"$green"[提示] 请不要通过鼠标直接关掉该终端！！！"$color_end
    echo -e " ================================================================"
    sleep .3
    echo && stty erase ^? && read -p "请选择"" 1 ""或"" 2 ""两个选项后，按回车:" num 
        case "$num" in
            1)
            if [ $chassis = "on" ];then
                sleep 1
                roslaunch handsfree_hw handsfree_hw.launch&
            fi
            if [ $lidar = "on" ];then
                sleep 3
                roslaunch rplidar_ros rplidar.launch&
            fi
            if [ $camera = "on" ];then
                sleep 3
                roslaunch handsfree_camera xtion.launch&
            fi
            if [ $arm = "on" ];then
                sleep 3
                roslaunch handsfree_v6_plus_arm_hw handsfree_v6_plus_arm_hw.launch&
            fi
            if [ $rviz = "on" ];then
                sleep 3
                rosrun rviz rviz -d `rospack find handsfree_bringup`/rviz/hardware_detection.rviz&
                sleep 3
                echo ""
                read -p `echo -e $red"按-Entrt-键退出..."$color_end`
                ps -ef |grep launch|grep -v grep|awk '{print $2}'|xargs kill -9
                ps -ef |grep opt|grep -v grep|awk '{print $2}'|xargs kill -9
                ps -ef |grep .log|grep -v grep|awk '{print $2}'|xargs kill -9
                sleep .3
            else
                echo ""
                sleep .3
                echo ""
                sleep .3
                echo ""
                sleep .3
                echo -e $red"什么都没有检测到，请检查硬件设备是否连接"$color_end
                sleep .3
                echo ""
                read -p `echo -e $red"按-Entrt-键退出..."$color_end`
            fi
            ;;
            2)
            #ps a |grep opt|grep -v grep|awk '{print $1}'|xargs kill -9
            ps -ef |grep launch|grep -v grep|awk '{print $2}'|xargs kill -9
            ps -ef |grep opt|grep -v grep|awk '{print $2}'|xargs kill -9
            ps -ef |grep .log|grep -v grep|awk '{print $2}'|xargs kill -9
            killall -9 bash
            ;;
            *)
            clear
            echo ""
            read -p `echo -e $red"按-Entrt-键退出..."$color_end`
            ;;
        esac
}
main $*
