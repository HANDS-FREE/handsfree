#!/usr/bin/env python
#coding=UTF-8 

import rospy
from handsfree_msgs.msg import robot_state

def callback(data): #回调函数
    rospy.loginfo("the embedded system_time: %fus",data.system_time) #下位机系统时间 
    rospy.loginfo("the embedded cpu temperature is: %f",data.cpu_temperature) #cpu温度
    rospy.loginfo("the battery voltage is: %f",data.battery_voltage) #电池电压
    rospy.loginfo("the battery power remain is: percent %f",data.power_remain*100) #电池电量剩余
    rospy.loginfo("——————————————————————————————————————— \n\r") 

def sub_state():
    rospy.init_node('sub_state', anonymous=True)
    rospy.Subscriber("/handsfree/robot_state", robot_state, callback) #想要接受的主题名字
    rospy.spin()

if __name__ == '__main__':
    sub_state()
