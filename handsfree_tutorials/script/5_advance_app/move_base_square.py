#!/usr/bin/env python
# coding:utf-8 

""" move_base_square.py - Version 1.1 2013-12-20

    Command a robot to move in a square using move_base actions..

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
    
    http://www.gnu.org/licenses/gpl.htmlPoint
      
"""

import rospy
import actionlib
from actionlib_msgs.msg import *
from geometry_msgs.msg import Pose, Point, Quaternion, Twist
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from tf.transformations import quaternion_from_euler
from visualization_msgs.msg import Marker
from math import radians, pi

class MoveBaseSquare():
    def __init__(self):

        
        rospy.on_shutdown(self.shutdown)
        self.goal_finish = 0
        # 正方形的边长
        square_size = rospy.get_param("~square_size", 1.0) # meters
        
        self.pose_x = rospy.get_param('~start_pose_x',10.75)
        self.pose_y = rospy.get_param('~start_pose_y',14.72)
        self.pose_z = rospy.get_param('~start_pose_z',0.0)

        self.quat_x = rospy.get_param('~start_quat_x',0.0)
        self.quat_y = rospy.get_param('~start_quat_y',0.0)
        self.quat_z = rospy.get_param('~start_quat_z',-0.013)
        self.quat_w = rospy.get_param('~start_quat_w',0.999)

        
        #存储四个角的角度
        quaternions = list()
        euler_angles = (pi/2, pi, 3*pi/2, 0)
        
        #将角度都转化为四元数
        for angle in euler_angles:
            q_angle = quaternion_from_euler(0, 0, angle, axes='sxyz')
            q_angle[0] += self.quat_x; q_angle[1] += self.quat_y; q_angle[2] += self.quat_z; q_angle[3] += self.quat_w
            q = Quaternion(*q_angle)
            quaternions.append(q)
        
        # 存储要到达四个点的坐标
        waypoints = list()
        waypoints.append(Pose(Point(square_size + self.pose_x, self.pose_y, self.pose_z), quaternions[0]))
        waypoints.append(Pose(Point(square_size + self.pose_x, square_size + self.pose_y, self.pose_z), quaternions[1]))
        waypoints.append(Pose(Point(self.pose_x, square_size + self.pose_y, self.pose_z), quaternions[2]))
        waypoints.append(Pose(Point(self.pose_x, self.pose_y, self.pose_z), quaternions[3]))
        
        # 初始化RVIZ上的可视化markers
        self.init_markers()
        
        # 将四个点标记出来      
        for waypoint in waypoints:           
            p = Point()
            p = waypoint.position
            self.markers.points.append(p)
            
        # 定义一个发布速度的节点（只会在最后停止机器人的时候用）
        self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=5)
        
        # 订阅 move_base action server 信息
        self.move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        
        rospy.loginfo("Waiting for move_base action server...")
        
        # 等待至move_base action server 信息可获得
        self.move_base.wait_for_server()
        
        rospy.loginfo("Connected to move base server")
        rospy.loginfo("Starting navigation test")

        i = 0
        #循环四次
        while i < 4 and not rospy.is_shutdown():
            self.goal_finish = 0
            # Update the marker display
            self.marker_pub.publish(self.markers)
            
            #设置目标
            goal = MoveBaseGoal()
            goal.target_pose.header.frame_id = 'map'
            goal.target_pose.header.stamp = rospy.Time.now()
            goal.target_pose.pose = waypoints[i]
            
            # 开始导航
            self.move(goal)
            
            i += 1
        
    def move(self, goal):
        # 发送goal
        self.move_base.send_goal(goal,self.donecb,self.activecb,self.feedbackcb)
        self.wait_for_done()#等待导航完毕
                    
    def init_markers(self):
        # 设置makers
        marker_scale = 0.2
        marker_lifetime = 0 # 0 is forever
        marker_ns = 'waypoints'
        marker_id = 0
        marker_color = {'r': 1.0, 'g': 0.7, 'b': 1.0, 'a': 1.0}
        
        # 定义marker发布节点
        self.marker_pub = rospy.Publisher('waypoint_markers', Marker, queue_size=5)
        
        # 初始化
        self.markers = Marker()
        self.markers.ns = marker_ns
        self.markers.id = marker_id
        self.markers.type = Marker.CUBE_LIST
        self.markers.action = Marker.ADD
        self.markers.lifetime = rospy.Duration(marker_lifetime)
        self.markers.scale.x = marker_scale
        self.markers.scale.y = marker_scale
        self.markers.color.r = marker_color['r']
        self.markers.color.g = marker_color['g']
        self.markers.color.b = marker_color['b']
        self.markers.color.a = marker_color['a']
        
        self.markers.header.frame_id = 'odom'
        self.markers.header.stamp = rospy.Time.now()
        self.markers.points = list()

    def donecb(self, status, result):
        #一旦目标到达，调用此函数
        rospy.loginfo("navigation done!")
        self.goal_finish = 1

    def activecb(self):
        #一旦机器人开始运动。调用此函数
        rospy.loginfo("Goal just went active")
    
    def feedbackcb(self,feedback):
        #每过一定时间，就会调用此函数，显示机器人当前状态
        pass
        #rospy.loginfo("navigation feedback\r\n%s"%feedback)

    def wait_for_done(self):
        rate = rospy.Rate(10) 
        while 1 :
            if self.goal_finish == 1 :
                break
            rate.sleep()

    def shutdown(self):
        rospy.loginfo("Stopping the robot...")
        rospy.sleep(2)
        # Stop the robot
        self.cmd_vel_pub.publish(Twist())
        rospy.sleep(1)

if __name__ == '__main__':
    try:
        rospy.init_node('nav_test', anonymous=False)
        MoveBaseSquare()
    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation test finished.")
