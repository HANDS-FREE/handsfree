#!/usr/bin/env python

import tf
import math
import rospy
import geometry_msgs.msg

class LinearMove(object):
    def __init__(self):
        self.frame_base = rospy.get_param('~base_frame', '/base_link')
        self.frame_odom = rospy.get_param('~odom_frame', '/odom')
        self.velocity_topic = rospy.get_param('~vel_topic', '/mobile_base/mobile_base_controller/cmd_vel')
        self.tolerance_distance = rospy.get_param('~tolerance', 0.08) # m
        self.speed_linear = rospy.get_param('~speed_linear', 0.1) # m/s
        self.rate_pub = rospy.get_param('~velocity_pub_rate', 10)
        self.rate = rospy.Rate(self.rate_pub)
        self.vel_pub = rospy.Publisher(self.velocity_topic,
                                       geometry_msgs.msg.Twist,
                                       queue_size=1)
        self.tf_listener = tf.TransformListener()
        rospy.on_shutdown(self.brake)
        try:
            self.tf_listener.waitForTransform(self.frame_odom,
                                              self.frame_base,
                                              rospy.Time(0),
                                              rospy.Duration(5.0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            rospy.logerr('tf catch exception when robot waiting for transform......')
            exit(-1)

    def move_to_target(self, dis_to_move=0):
        """
        to make robot move forward/back dis_to_move meters
        :param: dis_to_move: the distance make robot move, >0 means move forward, <0 means move back
        :type: float
        :return:
        """
        self.robot_start_pos = self.__get_robot_pos()
        rospy.logdebug("****************************************************************************")
        rospy.logdebug("robot current position is x = %f, y = %f, try to move forward/back %f Meter"
                       %(self.robot_start_pos.x, self.robot_start_pos.y, dis_to_move))
        rospy.logdebug("****************************************************************************")
        while self.__is_robot_arrived(dis_to_move) is not True:
            self.__move_robot(direction=(1 if dis_to_move > 0 else -1))
            self.rate.sleep()
        self.brake()  # we have arrived the target position, so stop robot
        rospy.loginfo('arrived the target point')

    def __is_robot_arrived(self, dis_to_move):
        """
        to check has the robot arrived target position
        :param: dis_to_move: the distance thar robot needs to move forward/back
        :type: float
        :return: False --- robot has not arrived the target position
                 True --- robot has arrived the target position
        """
        robot_cur_pos = self.__get_robot_pos()
        dis_moved = math.sqrt(math.pow((robot_cur_pos.x - self.robot_start_pos.x), 2) +
                                    math.pow((robot_cur_pos.y - self.robot_start_pos.y), 2))
        dis_need_move = math.fabs(dis_to_move) - dis_moved
        return False if math.fabs(dis_need_move) > self.tolerance_distance else True

    def __move_robot(self, direction=1):
        """
        send velocity to robot according to the direction
        :param: direction: when direction = 1: make robot move forward
                when direction = -1: make robot move back
        :type: int
        :return:
        """
        move_cmd = geometry_msgs.msg.Twist()
        move_cmd.linear.x = math.copysign(self.speed_linear, direction)
        self.vel_pub.publish(move_cmd)

    def __get_robot_pos(self):
        """
        to get current position(x,y,z) of robot
        :return: A geometry_msgs.msg.Point type store robot's position (x,y,z)
        """
        try:
            (trans, rot) = self.tf_listener.lookupTransform(self.frame_odom,
                                                            self.frame_base,
                                                            rospy.Time(0))
        except (tf.Exception, tf.ConnectivityException, tf.LookupException):
            rospy.logerr('tf catch exception when robot looking up transform')
            exit(-1)
        return geometry_msgs.msg.Point(*trans)

    def brake(self):
        """
        send command to stop the robot
        :return:
        """
        self.vel_pub.publish(geometry_msgs.msg.Twist())


if __name__ == '__main__':
    rospy.init_node('LinearMove')
    t_dis = rospy.get_param('~t_dis', 0.5) # m
    if t_dis == 0.0:
        rospy.logerr('no target distance set!')
        exit(-1)
    rospy.loginfo('try to move %f meters'%t_dis)
    LinearMove().move_to_target(dis_to_move=t_dis)

