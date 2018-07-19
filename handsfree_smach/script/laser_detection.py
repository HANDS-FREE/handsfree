#!/usr/bin/env python


import tf
import math
import rospy
import std_msgs.msg as std_msgs
import sensor_msgs.msg as sensor_msgs


class ObstacleDetection(object):
    def __init__(self):
        self.__angle_check_min = rospy.get_param('~min_angle', -math.pi/12)  # the min angle we could to detect obstacle
        self.__angle_check_max = rospy.get_param('~max_angle', math.pi/12)   # the max angle we could to detect obstacle
        self.__obstacle_threshold_dis = rospy.get_param('~obstacle_threshold', 1.0)  # the distance we think need to stop robot
        self.__topic_laser = rospy.get_param('~topic_name_laser', '/robot_0/base_scan')
        self.__suber_laser = rospy.Subscriber(self.__topic_laser,
                                              sensor_msgs.LaserScan,
                                              self.__callback_laser_scan,
                                              queue_size=1)
        # publish True when find obstacle in front of robot
        self.__puber_stop = rospy.Publisher('/stop_robot',
                                                std_msgs.Bool,
                                                queue_size=1)
        self.__puber_start = rospy.Publisher('/start_robot',
                                             std_msgs.Bool,
                                             queue_size=1)

    def __callback_laser_scan(self, laser_msg):
        """
        the callback function for laser_scan
        :param:laser_msg: the laser msg we get from robot
        :type: sensor_msgs.msgs.laserScan
        :return: none
        """
        size_laser_points = len(laser_msg.ranges)
        angle_count = 0
        pub_data = std_msgs.Bool()
        pub_data.data = True
        for each_point in range(0, size_laser_points, 1):
            point_angle = laser_msg.angle_min + angle_count
            # this angle between angle_check_min and angle_check_max is the range we need to check
            if self.__angle_check_min <= point_angle <= self.__angle_check_max:
                if laser_msg.ranges[each_point] <= self.__obstacle_threshold_dis:
                    rospy.loginfo('obstacle find!!!!')
                    self.__puber_stop.publish(pub_data)
                    return   # need not to check out other points
            angle_count += laser_msg.angle_increment
        self.__puber_start.publish(pub_data)



if __name__ == '__main__':
    rospy.init_node('obstacle_detection')
    ObstacleDetection()
    rospy.spin()
