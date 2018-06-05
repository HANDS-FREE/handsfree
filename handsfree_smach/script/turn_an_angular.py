#!/usr/bin/env python 
# -*- coding: utf-8 -*-
#机器人旋转的函数，暂时将角度矫正去掉（防止不必要的时间浪费）

import config
import math
import sys
sys.path.append(config.robot_state_pkg_path)
import robot_state_pkg.get_robot_position as robot_state
import rospy
import geometry_msgs.msg as g_msgs

class turn_an_angular(object):
    def __init__(self):
        rospy.loginfo('[robot_move_pkg]->move_an_angular is initial')
        self.robot_state = robot_state.robot_position_state()
        self.speed = config.high_turn_angular_speed
        self.rate = rospy.Rate(200)
        self.stop_tolerance = config.high_turn_angular_stop_tolerance
        self.cmd_move_pub = rospy.Publisher('/cmd_move' , g_msgs.Twist , queue_size=100)

    #发送急停速度，使机器人转动停止
    def brake(self):#停止时的回调函数
        move_velocity = g_msgs.Twist()
        move_velocity.linear.x = 0
        move_velocity.linear.y = 0
        move_velocity.angular.z = 0
        self.cmd_move_pub.publish(move_velocity)

    def turn_to(self , yaw = 0.0):
        rospy.on_shutdown(self.brake)
        move_velocity = g_msgs.Twist()
        start_w = current_w = self.robot_state.get_robot_current_w()
        w_has_moved = 0.0
        w_speed = 0.0
        flag = 0.0
        #规定一个角度跳跃的方式
        angular_has_changed = False
        #目标角度的变换
        current_goal = yaw + start_w
        if current_goal < -math.pi:
            current_goal = 2*math.pi+current_goal
        elif current_goal > math.pi:
            current_goal = current_goal-2*math.pi

        #判断角速度的方向
        if(yaw > 0):
            w_speed = -1 * self.speed
        else:
            w_speed = self.speed

        #由于角度数据只有-pi～pi，所以需要判断起始角+目标角是否会发生跳变
        if current_goal > math.pi:
            flag = 1
        elif current_goal < -math.pi:
            flag = 2
        else:
            flag = 0
        while not rospy.is_shutdown():
            #角度在正常的角度范围
            if(flag == 0):
                current_w = self.robot_state.get_robot_current_w()
                w_has_moved = abs(start_w - current_w)
                move_velocity.angular.z = w_speed   
                if w_has_moved>abs(yaw):
                    self.brake()
                    print abs(current_w-current_goal)*180/math.pi
                    current_w = self.robot_state.get_robot_current_w()
                    #if abs(current_w-current_goal) > 3*math.pi/180:
                        #turn_an_angular().turn_to(current_goal - current_w)
                    break

            #角度变换到大于pi的范围
            if(flag == 1):
                current_w = self.robot_state.get_robot_current_w()
                #判断角度是否已经超越现在的范围了
                if(current_w < 0):
                    angular_has_changed = True
                if(angular_has_changed == True):
                    current_w = 2*math.pi + self.robot_state.get_robot_current_w()
                move_velocity.angular.z = w_speed  
                w_has_moved = abs(current_w - start_w)
                if w_has_moved>abs(yaw):
                    self.brake()
                    print abs(current_w-current_goal)*180/math.pi
                    current_w = self.robot_state.get_robot_current_w()
                    #if abs(current_w-current_goal) > 3*math.pi/180:
                        #turn_an_angular().turn_to(current_goal - current_w)
                    break

            #角度变换到小于-pi的范围        
            if(flag == 2):
                current_w = self.robot_state.get_robot_current_w()
                #判断角度是否已经超越现在的范围了
                if(current_w > 0):
                    angular_has_changed = True
                if(angular_has_changed == True):
                    current_w = self.robot_state.get_robot_current_w() - 2*math.pi
                move_velocity.angular.z = w_speed  
                w_has_moved = abs(current_w - start_w)
                if w_has_moved>abs(yaw):
                    self.brake()
                    print abs(current_w-current_goal)*180/math.pi
                    current_w = self.robot_state.get_robot_current_w()
                    #if abs(current_w-current_goal) > 3*math.pi/180:
                        #turn_an_angular().turn_to(current_goal - current_w)
                    break
            
            self.cmd_move_pub.publish(move_velocity)
            self.rate.sleep()
            
if __name__ == '__main__':
    rospy.init_node('turn_angular')
    test = turn_an_angular()
    test.turn_to(-math.pi/2)

sys.path.remove(config.robot_state_pkg_path)
