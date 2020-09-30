#!/usr/bin/env python


import rospy
import actionlib
from actionlib_msgs.msg import *
from geometry_msgs.msg import Pose, Point, Quaternion, Twist
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from tf.transformations import quaternion_from_euler
from visualization_msgs.msg import Marker
from math import radians, pi

class MoveBaseSquare(object):
    def __init__(self):
        self.pose_x = rospy.get_param('~goal_pose_x',0.0)
        self.pose_y = rospy.get_param('~goal_pose_y',0.0)
        self.pose_z = rospy.get_param('~goal_pose_z',0.0)

        self.quat_x = rospy.get_param('~goal_quat_x',0.0)
        self.quat_y = rospy.get_param('~goal_quat_y',0.0)
        self.quat_z = rospy.get_param('~goal_quat_z',0.0)
        self.quat_w = rospy.get_param('~goal_quat_w',1.0)

        self.goal_finish = 0

        rospy.on_shutdown(self.shutdown)
        
        # Subscribe to the move_base action server
        self.move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        rospy.loginfo("Waiting for move_base action server...")
        # Wait 60 seconds for the action server to become available
        self.move_base.wait_for_server()
        rospy.loginfo("Connected to move base server")
        rospy.loginfo("Starting navigation test")

        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = '/map'
        goal.target_pose.header.stamp = rospy.Time.now()
            
        quaternion = Quaternion()
        quaternion.x = self.quat_x
        quaternion.y = self.quat_y
        quaternion.z = self.quat_z
        quaternion.w = self.quat_w
        goal.target_pose.pose.position.x = self.pose_x 
        goal.target_pose.pose.position.y = self.pose_y
        goal.target_pose.pose.position.z = self.pose_z 
        goal.target_pose.pose.orientation = quaternion
            
        self.move_base.send_goal(goal,self.donecb,self.activecb,self.feedbackcb)
        rate = rospy.Rate(10) 
        while 1 :
            if self.goal_finish == 1 :
                break
            rate.sleep()

    def shutdown(self):
        rospy.loginfo("Stopping the robot...")
        #self.move_base.cancel_goal()
        rospy.sleep(2)

    def donecb(self, status, result):
        rospy.loginfo("navigation done!")
        self.goal_finish = 1


    def activecb(self):
        rospy.loginfo("Goal just went active")
    
    def feedbackcb(self,feedback):
        pass
        rospy.loginfo("navigation feedback\r\n%s"%feedback)

if __name__ == '__main__':
    try:
        rospy.init_node('set_goal', anonymous=False)
        MoveBaseSquare()
    except (rospy.ROSInterruptException,rospy.ROSException,rospy.ROSInternalException):
        pass
