#!/usr/bin/env python

import tf
import math
import rospy
import geometry_msgs.msg
import tf.transformations
import handsfree_msgs.srv as handsfree_srv

class RadianTurn(object):
    def __init__(self):
        self.frame_base = rospy.get_param('~base_frame', '/base_link')
        self.frame_odom = rospy.get_param('~odom_frame', '/odom')
        self.velocity_topic = rospy.get_param('~vel_topic', '/mobile_base/mobile_base_controller/cmd_vel')
        self.tolerance_radian = rospy.get_param('~tolerance', 0.03)  # 0.03 = 1.7/360*2*pi
        self.speed_radian = rospy.get_param('~speed_radian', 0.436)  # 0.436 = 25/360*2*pi
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
                                              rospy.Duration(60.0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            rospy.logerr('tf catch exception when robot waiting for transform')
            exit(-1)

    def turn_to_target(self, radian_to_turn=0.0):
        """
        to make robot turn target_radian radians
        :param: target_radian: the target radian that robot needs to turn
        :type: float
        :return:
        """
        # the range of yaw of odom is -pi ~ pi, we transform it to 0 ~ 2*pi
        robot_start_yaw = (self.__get_robot_pos() + math.pi*2) % (math.pi*2)

        # to avoid the radian_to_turn value is bigger than 2*pi or less than -2*pi
        target_yaw = math.copysign(math.fabs(radian_to_turn)%(math.pi*2), radian_to_turn)+robot_start_yaw

        # to transform the range
        target_yaw = (target_yaw + math.pi*2) % (math.pi*2)

        # to find the shortest direction to turn
        radian_to_move = target_yaw-robot_start_yaw
        if radian_to_move < -math.pi or math.pi < radian_to_move:
            direction = 1 if radian_to_move < -math.pi else -1
        else:
            direction = 1 if radian_to_move > 0 else -1
        self.brake()  # to stop robot first
        rospy.logdebug("****************************************************************************")
        rospy.logdebug("the robot's Yaw = %f, try to turn to Yaw = %f, the direction = %d"
                       %(robot_start_yaw, target_yaw, direction))
        rospy.logdebug("****************************************************************************")
        while self.__is_robot_arrived(target_yaw) is not True:
            self.__turn_robot(turn_direction=direction)
            self.rate.sleep()
        self.brake()
        rospy.loginfo('arrived the target radian!')

    def __is_robot_arrived(self, target_yaw):
        """
        to check has the robot arrived target position
        :param: target_yaw: the target yaw that robot needs turn to
        :type: float
        :return: False --- robot has not arrived the target position
                 True --- robot has arrived the target position
        """
        robot_cur_yaw = (self.__get_robot_pos()+math.pi*2)%(math.pi*2)
        return False if math.fabs(target_yaw-robot_cur_yaw)>self.tolerance_radian else True

    def __turn_robot(self, turn_direction=1):
        """
        send velocity command to robot according to the direction
        :param: direction: when direction = 1: make robot turn in clockwise
                when direction = -1: make robot turn in counterclockwise
        :type: int
        :return:
        """
        move_cmd = geometry_msgs.msg.Twist()
        move_cmd.angular.z = math.copysign(self.speed_radian, turn_direction)
        self.vel_pub.publish(move_cmd)

    def __get_robot_pos(self):
        """
        to get Yaw of robot
        :return:
        the Yaw value of the robot
        """
        try:
            trans, rot = self.tf_listener.lookupTransform(self.frame_odom,
                                                          self.frame_base,
                                                          rospy.Time(0))
        except (tf.Exception, tf.ConnectivityException, tf.LookupException):
            rospy.logerr('tf catch exception when robot looking up transform')
            exit(-1)
        return tf.transformations.euler_from_quaternion(rot)[2]

    def brake(self):
        """
        send command to make robot stop
        :return:
        """
        self.vel_pub.publish(geometry_msgs.msg.Twist())


def callback_turn_radian(req):
    turn_radian.turn_to_target(radian_to_turn=req.target_radian_turn)
    return handsfree_srv.SpecialTurnResponse(True)


if __name__ == '__main__':
    rospy.init_node('RadianTurn')
    turn_radian = RadianTurn()
    service_radian_turn = rospy.Service('turn_radian', handsfree_srv.SpecialTurn, callback_turn_radian)
    rospy.loginfo("********************************************************")
    rospy.loginfo("*           radian turn start successfully !           *")
    rospy.loginfo("********************************************************")
    rospy.spin()

