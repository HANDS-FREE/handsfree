#!/usr/bin/env python
#coding=UTF-8

import cv2
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError #ros转opencv需要用到这个cv桥

def callback(data):
    try:
        cv_image = CvBridge().imgmsg_to_cv2(data, "bgr8") #使用cv_bridge将其转化为mat类型
    except CvBridgeError as e:
        print(e)
    (rows,cols,channels) = cv_image.shape
    cv2.imshow("Image window", cv_image) #显示出图像
    cv2.waitKey(30) #30ms 后播放下一帧

def get_photo():
    rospy.init_node('get_photo', anonymous=True)
    rospy.Subscriber("/camera/rgb/image_raw", Image, callback) #接受topic名称
    rospy.spin() 
   
if __name__ == '__main__':
    get_photo()
