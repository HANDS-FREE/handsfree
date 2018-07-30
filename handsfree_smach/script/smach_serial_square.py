#!/usr/bin/env python

import math
import smach
import rospy
import LinearMove
import RadianTurn


class MoveADistance(smach.State):
    def __init__(self, t_dis=0.0):
        smach.State.__init__(self, outcomes=['arrived'])
        self.__t_dis = t_dis

    def execute(self, ud):
        server_linear_move.move_to_target(dis_to_move=self.__t_dis)
        return 'arrived'


class TurnARadian(smach.State):
    def __init__(self, t_rad=0.0):
        smach.State.__init__(self, outcomes=["arrived"])
        self.__t_rad = t_rad

    def execute(self, ud):
        server_radian_turn.turn_to_target(radian_to_turn=self.__t_rad)
        return 'arrived'


if __name__ == '__main__':
    rospy.init_node('smach_serial_demo1', log_level=rospy.DEBUG)
    name_robot = rospy.get_param('~name_robot', 'robot_0')
    server_linear_move = LinearMove.LinearMove(name_robot)
    server_radian_turn = RadianTurn.RadianTurn(name_robot)
    smach_serial_demo1 = smach.StateMachine(outcomes=['walk_finished'])
    with smach_serial_demo1:
        smach.StateMachine.add('turn_right',
                               TurnARadian(-math.pi/2),
                               transitions={'arrived': 'forward_4_meters'})
        smach.StateMachine.add('forward_4_meters',
                               MoveADistance(4.0),
                               transitions={'arrived': 'turn_left'})
        smach.StateMachine.add('turn_left',
                               TurnARadian(math.pi/2),
                               transitions={'arrived': 'forward_4_meters'})
    smach_serial_demo1.execute()
    rospy.loginfo("HandsFree smach_serial_1 finish!")
