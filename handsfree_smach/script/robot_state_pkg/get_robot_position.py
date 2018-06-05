#! /usr/bin/env python
#coding:utf-8
# author = rescuer_liao
# get the current position of robot
#!/usr/bin/env python


#Team Unware Basketball Robot nwpu
#获取机器人当前在全局坐标系中的X，Y值以及机器人的方向值（单位为弧度值）
#均是通过tf_transform实现

#author=liao-zhihan
#first_debug_date:2016-01-26
#first_test_on_robot_date:2016-03
#第一次测试通过


import rospy
import roslib
import tf
import math
import tf.transformations as tf_transfer
import geometry_msgs.msg as g_msgs


class robot_position_state(object):
    def __init__(self):
        self.base_frame = rospy.get_param("base_frame_name","base_link")
        self.odom_frame = rospy.get_param("odom_frame_name","odom")
        self.tf_listener = tf.TransformListener()
        
        #进行tf的初始化，若失败请检查odom到bask_link的TF
    	self.tf_listener = tf.TransformListener()

        rospy.logwarn("[robot_state_pkg]->robot_position_state module is waiting for the tf between"
                      " %s and %s "%(self.base_frame , self.odom_frame))

        warn_time = 0
        wait_tf_start_time = rospy.Time.now()
        while not rospy.is_shutdown():
            is_tf_ok = self.tf_listener.canTransform(self.odom_frame,self.base_frame,rospy.Time())
            current_time = rospy.Time.now()
            if is_tf_ok:
                rospy.logerr('[robot_state_pkg]->robot_position_state module the transform between '
                              '%s and %s is ok!!'%(self.odom_frame , self.base_frame))
                break
            if current_time.to_sec()-wait_tf_start_time.to_sec() >= 10.0 and warn_time == 0:
                warn_time += 1


########################注：以下坐标获取均是通过tf实现####################################

	#获取机器人当前的X，Y轴与方向
    def get_robot_current_x_y_w(self):
         t = self.tf_listener.getLatestCommonTime(self.base_frame, self.odom_frame)
         position, quaternion = self.tf_listener.lookupTransform(self.odom_frame , self.base_frame,t)

         roll,pitch,yaw = tf.transformations.euler_from_quaternion(quaternion)
         return (position[0],position[1],yaw)

#获取机器人当前的X，Y值
    def get_robot_current_x_y(self):
        x , y , yaw = self.get_robot_current_x_y_w()
        return (x,y)

#获取机器人当前的X值
    def get_robot_current_x(self):
        x , y , yaw = self.get_robot_current_x_y_w()
        return x

#获取机器人当前的Y值
    def get_robot_current_y(self):
        x , y , yaw = self.get_robot_current_x_y_w()
        return y

#获取机器人当前的方向值
    def get_robot_current_w(self):
        x , y , yaw = self.get_robot_current_x_y_w()
        return yaw

if __name__ == '__main__':
    rospy.init_node('test')
    a = robot_position_state()
    s = a.get_robot_current_w()
    while True :
         s = a.get_robot_current_w()
	 print s
    
