#!/usr/bin/env python
# coding:utf-8 

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
        #两节点之间的循环次数
        self.num_of_cycles = rospy.get_param('~num_of_cycles',1)
        #下方构建如果没有传递相应位置参数的默认参数
        self.default_goal = {'goal1_point_x':0.0,'goal1_point_y':0.0,'goal1_point_z':0.0,
                            'goal1_quat_x':0.0,'goal1_quat_y':0.0,'goal1_quat_z':0.0,'goal1_quat_w':1.0, 
                            'goal2_point_x':0.0,'goal2_point_y':0.0,'goal2_point_z':0.0,
                            'goal2_quat_x':0.0,'goal2_quat_y':0.0,'goal2_quat_z':0.0,'goal2_quat_w':1.0}
        #下方获取整个私有节点内的所有参数，他们是以字典的方式存储
        self.goals_param = rospy.get_param("~",self.default_goal)
        #获取参数赋值
        self.goal1_point = Point()
        self.goal1_point.x = self.goals_param['goal1_point_x']
        self.goal1_point.y = self.goals_param['goal1_point_y']
        self.goal1_point.z = self.goals_param['goal1_point_z']
        self.goal1_quat = Quaternion()
        self.goal1_quat.x = self.goals_param['goal1_quat_x']
        self.goal1_quat.y = self.goals_param['goal1_quat_y']
        self.goal1_quat.z = self.goals_param['goal1_quat_z']
        self.goal1_quat.w = self.goals_param['goal1_quat_w']

        self.goal2_point = Point()
        self.goal2_point.x = self.goals_param['goal2_point_x']
        self.goal2_point.y = self.goals_param['goal2_point_y']
        self.goal2_point.z = self.goals_param['goal2_point_z']
        self.goal2_quat = Quaternion()
        self.goal2_quat.x = self.goals_param['goal2_quat_x']
        self.goal2_quat.y = self.goals_param['goal2_quat_y']
        self.goal2_quat.z = self.goals_param['goal2_quat_z']
        self.goal2_quat.w = self.goals_param['goal2_quat_w']

        # Subscribe to the move_base action server
        self.move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        rospy.loginfo("Waiting for move_base action server...")
        self.move_base.wait_for_server()
        rospy.loginfo("Connected to move base server")
        rospy.loginfo("Starting navigation test")
        
        goal = MoveBaseGoal()
        quaternion = Quaternion()

        for i in range(self.num_of_cycles):             
            #goal 1
            self.goal_finish = 0
            goal.target_pose.header.frame_id = 'map'
            goal.target_pose.header.stamp = rospy.Time.now()
            goal.target_pose.pose.position = self.goal1_point#赋值
            goal.target_pose.pose.orientation = self.goal1_quat
            
            #将goal发送出去
            self.move_base.send_goal(goal,self.donecb,self.activecb,self.feedbackcb)
            self.wait_for_done()#等待导航完毕
            
            self.goal_finish = 0
            #goal 2
            goal.target_pose.header.frame_id = 'map'
            goal.target_pose.header.stamp = rospy.Time.now()
            goal.target_pose.pose.position = self.goal2_point
            goal.target_pose.pose.orientation = self.goal2_quat

            self.move_base.send_goal(goal,self.donecb,self.activecb,self.feedbackcb)
            self.wait_for_done()
                    
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
        rospy.loginfo("navigation feedback\r\n%s"%feedback)

    def wait_for_done(self):
        rate = rospy.Rate(10) 
        while 1 :
            if self.goal_finish == 1 :
                break
            rate.sleep()

    def shutdown(self):
        rospy.loginfo("Stopping the robot...")
        rospy.sleep(2)

if __name__ == '__main__':
    try:
        rospy.init_node('set_two_goal', anonymous=False)
        MoveBaseSquare()
    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation test finished.")
