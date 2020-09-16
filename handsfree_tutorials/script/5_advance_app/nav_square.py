#!/usr/bin/env python
# coding:utf-8 


""" nav_square.py - Version 1.1 2013-12-20

    A basic demo of the using odometry data to move the robot
    along a square trajectory.

    Created for the Pi Robot Project: http://www.pirobot.org
    Copyright (c) 2012 Patrick Goebel.  All rights reserved.

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.5
    
    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details at:
    
    http://www.gnu.org/licenses/gpl.html
      
"""

import rospy
from geometry_msgs.msg import Twist, Point, Quaternion
import tf
from math import radians, copysign, sqrt, pow, pi
import PyKDL


def quat_to_angle(quat):
    rot = PyKDL.Rotation.Quaternion(quat.x, quat.y, quat.z, quat.w)
    return rot.GetRPY()[2]
        
def normalize_angle(angle):
    res = angle
    while res > pi:
        res -= 2.0 * pi
    while res < -pi:
        res += 2.0 * pi
    return res


class NavSquare():
    def __init__(self):
        
        # 设置rospy在终止节点时执行的关闭函数
        rospy.on_shutdown(self.shutdown)
        
        # 获取参数
        self.goal_distance = rospy.get_param("~goal_distance", 1.0)      # meters
        self.goal_angle = radians(rospy.get_param("~goal_angle", 90))    # degrees converted to radians
        self.linear_speed = rospy.get_param("~linear_speed", 0.2)        # meters per second
        self.angular_speed = rospy.get_param("~angular_speed", 0.7)      # radians per second
        self.angular_tolerance = radians(rospy.get_param("~angular_tolerance", 2)) # degrees to radians
        self.velocity_topic = rospy.get_param('~vel_topic', '/mobile_base/mobile_base_controller/cmd_vel')
        self.rate_pub = rospy.get_param('~velocity_pub_rate', 20)
        self.rate = rospy.Rate(self.rate_pub)
        
        # 初始化发布速度的topic
        self.cmd_vel = rospy.Publisher(self.velocity_topic, Twist, queue_size=1)
         
        
        self.base_frame = rospy.get_param('~base_frame', '/base_link')#set the base_frame
        self.odom_frame = rospy.get_param('~odom_frame', '/odom')#set the odom_frame
        # 初始化tf
        self.tf_listener = tf.TransformListener()
        
        rospy.sleep(2)

        try:
            self.tf_listener.waitForTransform(self.base_frame,
                                              self.odom_frame,
                                              rospy.Time(0),
                                              rospy.Duration(5.0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            rospy.logerr('tf catch exception when robot waiting for transform......')
            exit(-1)
            
        # 循环四次画出正方形
        for i in range(4):
            
            self.Linear_motion()

            # 再进行旋转之前时机器人停止
            move_cmd = Twist()
            self.cmd_vel.publish(move_cmd)
            rospy.sleep(1.0)
            
            self.Rotational_motion()

            move_cmd = Twist()
            self.cmd_vel.publish(move_cmd)
            rospy.sleep(1.0)
            
        #结束时使机器人停止
        self.cmd_vel.publish(Twist())

    '''
    线性运动的具体工作
    '''
    def Linear_motion(self):
        
        position = Point()
        move_cmd = Twist()
        move_cmd.linear.x = self.linear_speed
        
        # Get the starting position values     
        (position, rotation) = self.get_odom()
        
        x_start = position.x
        y_start = position.y
        
        distance = 0
        # Enter the loop to move along a side
        while distance < self.goal_distance and not rospy.is_shutdown():
            # Publish the Twist message      
            self.cmd_vel.publish(move_cmd)     
            self.rate.sleep()
    
            # Get the current position
            (position, rotation) = self.get_odom()
            # Compute the Euclidean distance from the start
            distance = sqrt(pow((position.x - x_start), 2) + 
                            pow((position.y - y_start), 2))

    '''
    旋转运动的具体工作
    '''  
    def Rotational_motion(self):
        
        (position, rotation) = self.get_odom()
        move_cmd = Twist()
        move_cmd.angular.z = self.angular_speed #赋值旋转角度
        
        # Track the last angle measured
        last_angle = rotation
        turn_angle = 0
        
        # Begin the rotation
        while abs(turn_angle + self.angular_tolerance) < abs(self.goal_angle) and not rospy.is_shutdown():
            # Publish the Twist message and sleep 1 cycle         
            self.cmd_vel.publish(move_cmd)
            self.rate.sleep()
            
            # Get the current rotation
            (position, rotation) = self.get_odom()
            
            # Compute the amount of rotation since the last lopp
            delta_angle = normalize_angle(rotation - last_angle)
            
            turn_angle += delta_angle
            last_angle = rotation


    '''
     获取odom和base_link之间的当前变换
    '''
    def get_odom(self):
        
        try:
            (trans, rot)  = self.tf_listener.lookupTransform(self.odom_frame, self.base_frame, rospy.Time(0))
        except (tf.Exception, tf.ConnectivityException, tf.LookupException):
            rospy.loginfo("TF Exception")
            return

        return (Point(*trans), quat_to_angle(Quaternion(*rot)))
            
    '''
    当关闭节点时会运行的函数

    '''
    def shutdown(self):
        rospy.loginfo("Stopping the robot...")
        self.cmd_vel.publish(Twist())
        rospy.sleep(1)

if __name__ == '__main__':
    try:
        rospy.init_node('nav_square', anonymous=False)
        NavSquare()
    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation terminated.")
