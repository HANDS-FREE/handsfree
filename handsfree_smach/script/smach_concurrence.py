#!/usr/bin/env python

import math
from hgext.convert.cvsps import logentry

import smach
import rospy
import LinearMove


class Robot1Move(smach.State):
    """
    A state to operate robot_1
    """
    def __init__(self, target_dis=0.0):
        smach.State.__init__(self, outcomes='arrived')
        self.__target_dis = target_dis

    def execute(self, ud):
        server_robot1_move.move_to_target(dis_to_move=self.__target_dis)
        return 'arrived'


class Robot2Move(smach.State):
    """
    A state to operate robot_2
    """
    def __init__(self, target_dis = 0.0):
        smach.State.__init__(self, outcomes='arrived')
        self.__target_dis = target_dis

    def execute(self, ud):
        server_robot2_move.move_to_target(dis_to_move=self.__target_dis)
        return 'arrived'


def gen_con_machine(robot1_t_dis, robot2_t_dis):
    """
    A generator to gen concurrence sub statemachine
    :param: robot1_t_dis: the distance wanna robot1 to move
    :type: float
    :param: robot2_t_dis: the distance wanna robot2 to move
    :type: float
    :return:
    """
    cc = smach.Concurrence(outcomes = ['both_arrived'],
                           default_outcome = 'both_arrived',
                           outcome_map = {'both_arrived': {'ROBOT1': 'arrived',
                                                           'ROBOT2': 'arrived'}})
    with cc:
        smach.Concurrence.add('ROBOT1', Robot1Move(robot1_t_dis))
        smach.Concurrence.add('ROBOT2', Robot2Move(robot2_t_dis))
    return cc


if __name__ == '__main__':
    rospy.init_node('smach_concurrence_demo')
    name_robot1 = rospy.get_param('~name_robot1', 'robot_0')
    name_robot2 = rospy.get_param('~name_robot2', 'robot_1')
    server_robot1_move = LinearMove.LinearMove(name_robot1)
    server_robot2_move = LinearMove.LinearMove(name_robot2)
    smach_root = smach.StateMachine(outcomes=['finish'])
    state1 = gen_con_machine(1, 3)
    state2 = gen_con_machine(3, 1)
    state3 = gen_con_machine(-1, -3)
    state4 = gen_con_machine(-3, -1)
    with smach_root:
        smach.StateMachine.add('state1',
                               state1,
                               transitions={'both_arrived': 'state2'})
        smach.StateMachine.add('state2',
                               state2,
                               transitions={'both_arrived': 'state3'})
        smach.StateMachine.add('state3',
                               state3,
                               transitions={'both_arrived': 'state4'})
        smach.StateMachine.add('state4',
                               state4,
                               transitions={'both_arrived': 'state1'})
    smach_root.execute()
    rospy.loginfo("HandsFree smach_concurrence demo finish!")
