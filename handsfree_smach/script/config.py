#!/usr/bin/env python 
# -*- coding: utf-8 -*-
#robot_state_pkg_path = os.path.abspath(os.path.join(os.path.dirname("__file__"), os.path.pardir))
#用于加载自己写的模块,请将它改为自己的相应模块

robot_state_pkg_path = '/home/huangtairan/catkin_ws/src/basketball_strage/scripts/'


#设置直线速度

linear_move_speed = 0.5


#设置靠近球时以及在机器人坐标下的速度
low_linear_speed = 0.25
#设置插值直线移动阈值
high_speed_stop_tolerance = 0.01
# 设置低速以及在机器人坐标系下的移动阈值
low_speed_move_stop_tolerance = 0.01
#设置转弯的速度

high_turn_angular_speed = 9


# 低速、在机器人坐标系下转动速度
low_turn_angular_speed = 0.15
#弧度值 代表高速停止的阈值
high_turn_angular_stop_tolerance = 0.02
#弧度值 低速、在机器人坐标系下转动阈值
low_turn_angular_stop_tolerance = 0.05



