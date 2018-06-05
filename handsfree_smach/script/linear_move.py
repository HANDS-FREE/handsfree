#!/usr/bin/env python
# -*- coding: UTF-8 -*-
# 采用慢速启动，慢速停止的策略，由于机器人底盘在启动时只能同时启动三个轮子，所以暂时采用这种方法

import rospy
import roslib
import tf
import actionlib
import math
import PyKDL
from actionlib_msgs.msg import *
from move_base_msgs.msg import *
from tf.transformations import *
from geometry_msgs.msg import *

class linearMove(object):
    def __init__(self):
        rospy.loginfo('the linear srv move init ok')

    def execute(self):
        rospy.on_shutdown(self.shutdown)
        rospy.loginfo('the distance you want to move is %s'%self.test_distance)
        self.rate = 100
        r = rospy.Rate(self.rate)
        self.speed = rospy.get_param('~speed',0.45)
        self.tolerance = rospy.get_param('~tolerance', 0.1)
        self.cmd_vel = rospy.Publisher('/mobile_base/mobile_base_controller/cmd_vel',Twist,queue_size=5)
        self.base_frame = rospy.get_param('~base_frame', '/base_link')
        self.odom_frame = rospy.get_param('~odom_frame', '/odom')
        self.tf_listener = tf.TransformListener()
        rospy.sleep(2)
        self.tf_listener.waitForTransform(self.odom_frame,self.base_frame,rospy.Time(),rospy.Duration(60.0))
        rospy.loginfo("start test!!!!")
        #define a bianliang
        self.flag = rospy.get_param('~flag', True)
        #tf get position
        self.position = Point()
        self.position = self.get_position()
        x_start = self.position.x
        y_start = self.position.y

        #publish cmd_vel
        move_cmd = Twist()
        while not rospy.is_shutdown():
            move_cmd = Twist()
            if self.flag:
                self.position = self.get_position()
                distance = float(math.sqrt(pow((self.position.x - x_start), 2) +
                                pow((self.position.y - y_start), 2)))
                if self.test_distance > 0:
                    error = float(distance - self.test_distance)
                else:
                    error = - float(distance + self.test_distance)
                if not self.flag or abs(error) < self.tolerance:
                    self.flag = False
                else:
                    move_cmd.linear.x = math.copysign(self.speed, -1 * error)
            else:
                self.position = self.get_position()
                x_start = self.position.x
                y_start = self.position.y
            if move_cmd.linear.x:
                self.cmd_vel.publish(move_cmd)
            else:
                break
            r.sleep()
        self.cmd_vel.publish(Twist())
    def get_position(self):
        try:
            (trans,rot) = self.tf_listener.lookupTransform(self.odom_frame, self.base_frame, rospy.Time(0))
        except (tf.Exception, tf.ConnectivityException, tf.lookupException):
            rospy.loginfo("TF Exception")
            return

        return Point(*trans)

    def shutdown(self):
        rospy.loginfo("Stopping the robot...")
        self.cmd_vel.publish(Twist())
        rospy.sleep(1)

    def move(self,dis):
        #rospy.loginfo('the distance you want to move is : %s'%dis)
        self.test_distance = dis
        self.execute()


if __name__ == '__main__':
    rospy.init_node('linear_move')
    move_cmd = linearMove()
    move_cmd.move(1)
