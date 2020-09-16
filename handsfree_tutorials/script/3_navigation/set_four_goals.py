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
        self.num_of_cycles = rospy.get_param('~num_of_cycles',2)
        #下方构建如果没有传递相应位置参数的默认参数
        self.default_goal = {'goal1_point_x':0.0,'goal1_point_y':0.0,'goal1_point_z':0.0,
                            'goal1_quat_x':0.0,'goal1_quat_y':0.0,'goal1_quat_z':0.0,'goal1_quat_w':1.0, 
                            'goal2_point_x':0.0,'goal2_point_y':0.0,'goal2_point_z':0.0,
                            'goal2_quat_x':0.0,'goal2_quat_y':0.0,'goal2_quat_z':0.0,'goal2_quat_w':1.0,
                            'goal3_point_x':0.0,'goal3_point_y':0.0,'goal3_point_z':0.0,
                            'goal3_quat_x':0.0,'goal3_quat_y':0.0,'goal3_quat_z':0.0,'goal3_quat_w':1.0,
                            'goal4_point_x':0.0,'goal4_point_y':0.0,'goal4_point_z':0.0,
                            'goal4_quat_x':0.0,'goal4_quat_y':0.0,'goal4_quat_z':0.0,'goal4_quat_w':1.0,
                            }
        #下方获取整个私有节点内的所有参数，他们是以字典的方式存储
        self.goals_param = rospy.get_param("~",self.default_goal)

        self.all_goals = [MoveBaseGoal()] *4 

        self.all_pose = [Pose()]*4
        #获取参数赋值
        self.all_pose[0] = Pose(
                                Point(self.goals_param['goal1_point_x'],
                                      self.goals_param['goal1_point_y'],
                                      self.goals_param['goal1_point_z']
                                     ),
                                Quaternion(self.goals_param['goal1_quat_x'],
                                           self.goals_param['goal1_quat_y'],
                                           self.goals_param['goal1_quat_z'],
                                           self.goals_param['goal1_quat_w']
                                          )
                             )
        
        self.all_pose[1] = Pose(
                                Point(self.goals_param['goal2_point_x'],
                                      self.goals_param['goal2_point_y'],
                                      self.goals_param['goal2_point_z']
                                     ),
                                Quaternion(self.goals_param['goal2_quat_x'],
                                           self.goals_param['goal2_quat_y'],
                                           self.goals_param['goal2_quat_z'],
                                           self.goals_param['goal2_quat_w']
                                          )
                             )
        self.all_pose[2] = Pose(
                                Point(self.goals_param['goal3_point_x'],
                                      self.goals_param['goal3_point_y'],
                                      self.goals_param['goal3_point_z']
                                     ),
                                Quaternion(self.goals_param['goal3_quat_x'],
                                           self.goals_param['goal3_quat_y'],
                                           self.goals_param['goal3_quat_z'],
                                           self.goals_param['goal3_quat_w']
                                          )
                             )
        self.all_pose[3] = Pose(
                                Point(self.goals_param['goal4_point_x'],
                                      self.goals_param['goal4_point_y'],
                                      self.goals_param['goal4_point_z']
                                     ),
                                Quaternion(self.goals_param['goal4_quat_x'],
                                           self.goals_param['goal4_quat_y'],
                                           self.goals_param['goal4_quat_z'],
                                           self.goals_param['goal4_quat_w']
                                          )
                             )
        i = 0
        for pose in self.all_pose:
            goal = MoveBaseGoal()
            goal.target_pose.header.frame_id = '/map'
            goal.target_pose.header.stamp = rospy.Time.now()
            goal.target_pose.pose = pose
            self.all_goals[i] = goal
            i = i+1
            
        
        # Subscribe to the move_base action server
        self.move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        rospy.loginfo("Waiting for move_base action server...")
        self.move_base.wait_for_server()
        rospy.loginfo("Connected to move base server")
        rospy.loginfo("Starting navigation test")
        
        goal = MoveBaseGoal()
        quaternion = Quaternion()

        for i in range(self.num_of_cycles):             
            for j in range(4):
                self.goal_finish = 0
                self.move_base.send_goal(self.all_goals[j],self.donecb,self.activecb,self.feedbackcb)
                #rospy.loginfo("goals: %d   %s ",j,self.all_goals[j])
                self.wait_for_done()#等待导航完毕

        self.move_base.send_goal(self.all_goals[0],self.donecb,self.activecb,self.feedbackcb)
        #rospy.loginfo("goals: %d   %s ",j,self.all_goals[j])
        self.wait_for_done()#等待导航完毕
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

if __name__ == '__main__':
    try:
        rospy.init_node('set_two_goal', anonymous=False)
        MoveBaseSquare()
    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation test finished.")
