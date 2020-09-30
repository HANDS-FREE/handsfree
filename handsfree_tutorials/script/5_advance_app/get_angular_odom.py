#! /usr/bin/python
## coding:utf-8 
import PyKDL
import rospy
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from math import *
import threading
import os
import subprocess
import yaml

#四元数转欧拉角
def quat_to_angle(quat):
  rot = PyKDL.Rotation.Quaternion(quat.x, quat.y, quat.z, quat.w)
  return rot.GetRPY()[2]
def normalize_angle(angle):
  res = angle
  while res > pi:
    res -= 2.0*pi
  while res < -pi:
    res += 2.0*pi
  return res


class CalibrateRobot1:
  def __init__(self):
    self.lock = threading.Lock()#创建锁
    self.odom_frame = rospy.get_param('~odom_frame', '/mobile_base/mobile_base_controller/odom')
    self.sub_imu = rospy.Subscriber(self.odom_frame, Odometry, self.imu_cb)
    self.last_imu_angle = 0
    self.imu_angle = 0
    while not rospy.is_shutdown():
      rospy.sleep(0.3)
      rospy.loginfo("imu_angle:"+str((self.imu_angle)*180/pi))


  def imu_cb(self, msg):
    with self.lock:#自动进行上锁，等语句运行完毕，自动解锁
      angle = quat_to_angle(msg.pose.pose.orientation)
      self.imu_angle = angle
      self.imu_time = msg.header.stamp

def main():
  rospy.init_node('scan_to_angle1')
  CalibrateRobot1()


if __name__ == '__main__':
  main()
