#!/usr/bin/env python
#coding=UTF-8

import rospy
import tf
from tf.transformations import *
from sensor_msgs.msg import Imu

def callback(data):
    #这个函数是tf中的,可以将四元数转成欧拉角
    (r,p,y) = tf.transformations.euler_from_quaternion((data.orientation.x,data.orientation.y,data.orientation.z,data.orientation.w))
    #由于是弧度制，下面将其改成角度制看起来更方便
    rospy.loginfo("Pitch = %f,Roll = %f, Yaw = %f",r*180/3.1415926,p*180/3.1415926,y*180/3.1415926)

def get_imu():
    rospy.init_node('get_imu', anonymous=True)
    rospy.Subscriber("/handsfree/imu", Imu, callback) #接受topic名称
    rospy.spin()

if __name__ == '__main__':
    get_imu()
