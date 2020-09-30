#!/usr/bin/env python  
import roslib
import rospy
import math
import tf
import geometry_msgs.msg

def getDistance(x1, y1, x2, y2):
    return math.sqrt((x2-x1)**2 + (y2-y1)**2)

if __name__ == '__main__':
    rospy.init_node('tf_turtle')

    listener = tf.TransformListener()


    turtle_vel = rospy.Publisher('cmd_vel_1', geometry_msgs.msg.Twist,queue_size=1)

    rate = rospy.Rate(10.0)
    while not rospy.is_shutdown():
        try:
            #now = rospy.Time.now() - rospy.Duration(5.0)
            #listener.waitForTransform("/base_footprint_1", "/base_footprint", now, rospy.Duration(1.0))
            (trans,rot) = listener.lookupTransform('/base_footprint_1', '/base_footprint', rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue
        
        dist = getDistance(0, 0, trans[0], trans[1])
        print trans[0], trans[1], dist
        if (dist>0.5):
            angular = 2 * math.atan2(trans[1], trans[0])
            linear = 0.2 * math.sqrt(trans[0] ** 2 + trans[1] ** 2)
            cmd = geometry_msgs.msg.Twist()
            cmd.linear.x = linear
            cmd.angular.z = angular
        else:
            cmd.linear.x = 0
            cmd.angular.z = 0
        turtle_vel.publish(cmd)
        rate.sleep()
