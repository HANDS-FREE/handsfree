#!/usr/bin/env python

import tf.transformations
import math
import smach
import rospy
import actionlib
import linear_move
import smach_ros
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

def gen_new_goal(frame_id, x, y, next_x, next_y):
    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = frame_id
    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.pose.position.x = x
    goal.target_pose.pose.position.y = y
    radian = (math.atan2(next_y-y, next_x-x)%(math.pi*2))
    quaternion = tf.transformations.quaternion_from_euler(0, 0, radian)
    goal.target_pose.pose.orientation.x = quaternion[0]
    goal.target_pose.pose.orientation.y = quaternion[1]
    goal.target_pose.pose.orientation.z = quaternion[2]
    goal.target_pose.pose.orientation.w = quaternion[3]
    return goal

class MoveBack(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['finish'])
    def execute(self, ud):
        goal = gen_new_goal('base_link', -0.5, 0)
        client_movebase.send_goal(goal)
        result = client_movebase.get_result()
        return 'finish'

class MovePoint(smach.State):
    def __init__(self, x, y, n_x, n_y):
        smach.State.__init__(self, outcomes=['next','failed','disable'])
        self._x = x
        self._y = y
        self._n_x = n_x
        self._n_y = n_y
    def execute(self, ud):
        goal = gen_new_goal('map', self._x, self._y, self._n_x, self._n_y)
        client_movebase.send_goal(goal)
        server_available = client_movebase.wait_for_result()
        if server_available:
            result = client_movebase.get_result()
            if result:
                return 'next'
            else:
                return 'failed'
        else:
            return 'disable'

if __name__ == '__main__':
    try:
        rospy.init_node('HandsFree_smach_demo')
        client_linearmove = linear_move.linearMove()
        client_movebase = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        client_movebase.wait_for_server()
        # rospy.loginfo("smach demo start")
        # goal = gen_new_goal('map',2)
        # client_movebase.send_goal(goal)
        # client_movebase.wait_for_result()
        # client_movebase.get_result()
        points = [[-7.23173017996, -4.42954061488],   #point 1
                  [-6.38450322146,  -8.49156316549],  #point 2
                  [-0.828087909847, -5.76186700931],  #point 3
                  [6.73188562237, -3.83891025409],    #point 4
                  [6.17357818599, -0.452238380699],   #point 5
                  [0.28653942132, -2.64845397718]]    #point 6
        point_size = len(points)
        smach_demo = smach.StateMachine(outcomes=['success', 'failed'])
        name_target_point = ""
        name_next_point = ""
        name_moveback_state = ""
        with smach_demo:
            for point_target in range(0, point_size, 1):
                point_next_target = (point_target+1)%point_size
                name_target_point = 'POINT%d'%point_target
                name_next_point = 'POINT%d'%(point_next_target)
                name_moveback_state = 'BACK%d'%point_target
                smach.StateMachine.add(name_target_point, MovePoint(points[point_target][0],
                                                                    points[point_target][1],
                                                                    points[point_next_target][0],
                                                                    points[point_next_target][1]),
                                       transitions={'next':name_next_point,
                                                    'failed':name_moveback_state,
                                                    'disable':'failed'})
                smach.StateMachine.add(name_moveback_state, MoveBack(),
                                       transitions={'finish':name_target_point})
        smach_demo.execute()
    except rospy.ROSInterruptException:
        rospy.logerr("ros interruption exception")





