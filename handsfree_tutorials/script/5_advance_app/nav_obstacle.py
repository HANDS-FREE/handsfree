#!/usr/bin/env python
# coding:utf-8 

#################################################################################
# Copyright 2018 ROBOTIS CO., LTD.
#
# The robot moves in a straight line and stops when it encounters obstacles
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#################################################################################

# Authors: Gilbert #

import rospy
import math
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist


class Obstacle():
    def __init__(self):
        self.velocity_topic = rospy.get_param('~vel_topic', '/mobile_base/mobile_base_controller/cmd_vel')
        self.speed_linear = rospy.get_param('~speed_linear', 0.3)
        self.stop_distance = rospy.get_param('~stop_distance', 0.5)
        self.rate_pub = rospy.get_param('~velocity_pub_rate', 10)
        self.rate = rospy.Rate(self.rate_pub)


        self._cmd_pub = rospy.Publisher(self.velocity_topic, Twist, queue_size=1)
        self.obstacle()
        
    def get_scan(self):#接收scan topic并将其信息储存起来
        scan = rospy.wait_for_message('scan', LaserScan)
        scan_filter = []
       
        samples = len(scan.ranges)         
        
        if samples == 0:
            rospy.logerr('can`t find the scan')
        else:
            scan_filter.extend(scan.ranges)

        for i in range(samples):
            if scan_filter[i] == float('Inf') or math.isnan(scan_filter[i]):
                scan_filter[i] = 10        
        return scan_filter

    def obstacle(self):
        twist = Twist()
        turtlebot_moving = True

        while not rospy.is_shutdown():
            lidar_distances = self.get_scan()
            min_distance = min(lidar_distances)

            if min_distance < self.stop_distance:#当小于最小值时停止
                if turtlebot_moving:
                    twist.linear.x = 0.0
                    twist.angular.z = 0.0
                    self._cmd_pub.publish(twist)
                    turtlebot_moving = False
                    rospy.loginfo('Stop! Distance of the obstacle : %f', min_distance)
                    break
            else:
                twist.linear.x = self.speed_linear
                twist.angular.z = 0.0
                self._cmd_pub.publish(twist)
                turtlebot_moving = True
                rospy.loginfo('Distance of the obstacle : %f', min_distance)
            self.rate.sleep()

def main():
    rospy.init_node('move_obstacle')
    try:
        obstacle = Obstacle()
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    main()
