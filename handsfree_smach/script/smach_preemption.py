#!/usr/bin/env python

import rospy
import math
import smach
import smach_ros
import LinearMove
import RadianTurn
import std_msgs.msg as std_msgs

dis_to_move = 4
dis_need_move = dis_to_move


class MoveADistance(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['arrived', 'preempted'])
        self.__move_step = 0.5  # move 0.5 meter every step

    def execute(self, ud):
        global  dis_need_move
        while dis_need_move - self.__move_step > 0.0:
            if self.preempt_requested():
                # find obstacle in front of robot
                self.service_preempt()
                return 'preempted'
            server_linear_move.move_to_target(dis_to_move=self.__move_step)
            dis_need_move -= self.__move_step

        if dis_need_move > 0.0:
            # after the obstacle in front of robot has been moved, robot continue moving
            if self.preempt_requested():
                # find obstacle in front of robot
                self.service_preempt()
                return 'preempted'
            server_linear_move.move_to_target(dis_to_move=dis_need_move)
        return 'arrived'


class TurnARadian(smach.State):
    def __init__(self, target_rad=0.0):
        smach.State.__init__(self,
                             outcomes=['arrived'])
        self.__t_rad = target_rad

    def execute(self, ud):
        global dis_need_move, dis_to_move
        server_radian_turn.turn_to_target(radian_to_turn=self.__t_rad)
        dis_need_move = dis_to_move    # prepare for next linear move
        return 'arrived'


class StayInSpace(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['stopped', 'preempted'])

    def execute(self, ud):
        if self.preempt_requested():
            # find obstacle in front of robot has been moved, so robot start moving
            self.service_preempt()
            return 'preempted'
        server_linear_move.brake()
        rospy.sleep(0.5)
        return 'stopped'


def child_term_cb_cc_move(outcome_map):
    if outcome_map['need_robot_stop'] == 'invalid':
        return True
    if outcome_map['move_forward'] == 'arrived':
        return True
    return False


def out_cb_cc_move(outcome_map):
    if outcome_map['move_forward'] == 'preempted':
        return 'need_stop'
    if outcome_map['move_forward'] == 'arrived':
        return 'arrived'


def child_term_cb_cc_stop(outcome_map):
    if outcome_map['need_robot_move'] == 'invalid':
        return True
    if outcome_map['stop'] == 'stopped':
        return True
    return False


def out_cb_cc_stop(outcome_map):
    if outcome_map['stop'] == 'preempted':
        return 'start_move'
    if outcome_map['stop'] == 'stopped':
        return 'still_stop'


def obstacle_monitor_cb(ud, msg):
    rospy.loginfo(msg.data)
    if msg.data is True:
        return False  # has obstacle in front of robot, invalid
    else:
        return True # no obstacle in front of robot, valid


if __name__ == '__main__':
    rospy.init_node('smach_preemption_demo', log_level=rospy.DEBUG)
    name_robot = rospy.get_param('~name_robot', 'robot_0')
    server_linear_move = LinearMove.LinearMove(name_robot)
    server_radian_turn = RadianTurn.RadianTurn(name_robot)
    concurrence_move = smach.Concurrence(outcomes=['arrived', 'need_stop'],
                                         default_outcome='need_stop',
                                         child_termination_cb=child_term_cb_cc_move,
                                         outcome_cb=out_cb_cc_move)
    with concurrence_move:
        smach.Concurrence.add('move_forward', MoveADistance())
        smach.Concurrence.add('need_robot_stop', smach_ros.MonitorState('/stop_robot',
                                                                        std_msgs.Bool,
                                                                        obstacle_monitor_cb))

    concurrence_stop = smach.Concurrence(outcomes=['still_stop', 'start_move'],
                                         default_outcome='still_stop',
                                         child_termination_cb=child_term_cb_cc_stop,
                                         outcome_cb=out_cb_cc_stop)
    with concurrence_stop:
        smach.Concurrence.add('stop', StayInSpace())
        smach.Concurrence.add('need_robot_move', smach_ros.MonitorState('/start_robot',
                                                                        std_msgs.Bool,
                                                                        obstacle_monitor_cb))

    sm_root = smach.StateMachine(outcomes=['finished'])
    with sm_root:
        smach.StateMachine.add('turn_right',
                               TurnARadian(-math.pi/2),
                               transitions={'arrived': 'linear_move'})

        smach.StateMachine.add('linear_move',
                               concurrence_move,
                               transitions={'arrived': 'turn_left',
                                            'need_stop': 'stop_in_space'})

        smach.StateMachine.add('turn_left',
                               TurnARadian(math.pi/2),
                               transitions={'arrived': 'linear_move'})

        smach.StateMachine.add('stop_in_space',
                               concurrence_stop,
                               transitions={'start_move': 'linear_move',
                                            'still_stop': 'stop_in_space'})
    sm_root.execute()
    rospy.loginfo('demo preemption finished!!!!!!')
