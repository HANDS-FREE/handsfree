#!/usr/bin/env python
#coding=UTF-8 

import rospy
from sensor_msgs.msg import LaserScan

def callback(data):
    rospy.loginfo("the angle_min is %f",data.angle_min) #打印一些信息
    rospy.loginfo("the angle_max is %f",data.angle_max)
    rospy.loginfo("the scan data is ")
    for i in range(0, len(data.ranges)):
        print data.ranges[i],
    print "\n"

def sub_state():
    rospy.init_node('sub_scan', anonymous=True)
    rospy.Subscriber("/scan", LaserScan, callback) #接受topic
    rospy.spin()

if __name__ == '__main__':
    sub_state()
