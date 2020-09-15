#!/usr/bin/env python

"""
  voice_cmd_vel.py is a simple demo of speech recognition.
  You can control a mobile base using commands found
  in the corpus file.

  Based on the voice_cmd_vel.py script by Michael Ferguson in
  the pocketsphinx ROS package.
  
  See http://www.ros.org/wiki/pocketsphinx
"""

import roslib; roslib.load_manifest('handsfree_speech')
import rospy
import math

from geometry_msgs.msg import Twist
from std_msgs.msg import String
from math import copysign

class voice_cmd_vel:

    def __init__(self):
        rospy.on_shutdown(self.cleanup)
        
        # Set a number of parameters affecting the robot's speed
        self.max_speed = rospy.get_param("~max_speed", 0.4)
        self.min_speed = rospy.get_param("~min_speed", 0.1)
        self.linear_increment = rospy.get_param("~linear_increment", 0.1)

        self.max_angular_speed = rospy.get_param("~max_angular_speed", 0.8)
        self.min_angular_speed = rospy.get_param("~min_angular_speed", 0.1)
        self.angular_increment = rospy.get_param("~angular_increment", 0.1)

        # Initial parameters
        self.speed = rospy.get_param("~start_speed", 0.1)
        self.angular_speed = rospy.get_param("~start_angular_speed", 0.2)
        
        # We don't have to run the script very fast
        self.rate = rospy.get_param("~rate", 5)
        r = rospy.Rate(self.rate)

        # Time, in seconds, for waiting correct command
        self.initial_wait_time = 5
        self.wait_time = self.initial_wait_time

        # A flag to determine whether or not TIAGo voice control
        self.TIAGo = False
        
        # Initialize the Twist message we will publish.
        self.cmd_vel = Twist()

        # Publish the Twist message to the cmd_vel topic
        self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=5)
        
        # Subscribe to the /recognizer/output topic to receive voice commands.
        rospy.Subscriber('/recognizer/output', String, self.speech_callback)

        
        # A mapping from keywords or phrases to commands
        self.keywords_to_command = {'backward': ['backward', 'go backward', 'move backward', 'back', 'move back'],
                                    'forward': ['forward', 'go forward', 'move forward'],
                                    'turn left': ['turn left', 'left'],
                                    'turn right': ['turn right', 'right'],
                                    'stop': ['stop', 'halt'],
                                    'faster': ['faster'],
                                    'slower': ['slower'],
                                    'quarter': ['quarter speed', 'quarter'],
                                    'half': ['half speed', 'half'],
                                    'full': ['full speed', 'full']}
        
        rospy.loginfo("Ready to receive voice commands")
        
        # We have to keep publishing the cmd_vel message if we want the robot to keep moving.
        while not rospy.is_shutdown():
            self.cmd_vel_pub.publish(self.cmd_vel)
            r.sleep()                       
            
    def get_command(self, data):
        # Attempt to match the recognized word or phrase to the 
        # keywords_to_command dictionary and return the appropriate
        # command
        for (command, keywords) in self.keywords_to_command.iteritems():
            for word in keywords:
                if data.find(word) > -1:
                    # Wait time for other command, in seconds
                    self.time1 = rospy.get_rostime()
                    self.wait_time = 10
                    return command
        
    def speech_callback(self, msg):
	rospy.loginfo('Moving the base through velocity commands')

        # If the user said TIAGo, set the flag as true 
        if msg.data == 'tiago':
            rospy.loginfo("TIAGo waiting command")
            self.TIAGo = True
            self.time1 = rospy.get_rostime()
            #rospy.loginfo("Current time1 %i", self.time1.secs)
            return

        elif self.get_command(msg.data) == 'stop':
            # Stop the robot!  Publish a Twist message consisting of all zeros.
            rospy.loginfo("Command: " + self.get_command(msg.data)) 
            self.cmd_vel = Twist()
            self.TIAGo = False
            self.wait_time = self.initial_wait_time
            return

        # If TIAGo voice control is true and not out of time
        # Get the motion command from the recognized phrase
        # If false, simply return without performing any action
        if self.TIAGo:
            self.time2 = rospy.get_rostime()
            #rospy.loginfo("Current time2 %i", self.time2.secs)
            if self.time2.secs < self.time1.secs+self.wait_time:
                command = self.get_command(msg.data)
            else:
                self.TIAGo = False
                self.wait_time = self.initial_wait_time
                return
        else:
            return
        
        # Log the command to the screen
        rospy.loginfo("Command: " + str(command))
             
        
        # The list of if-then statements should be fairly
        # self-explanatory
        if command == 'backward':
            self.cmd_vel.linear.x = -self.speed
            self.cmd_vel.angular.z = 0

        elif command == 'forward':    
            self.cmd_vel.linear.x = self.speed
            self.cmd_vel.angular.z = 0
            
        elif command == 'turn left':
            if self.cmd_vel.linear.x != 0:
                if self.cmd_vel.angular.z < self.angular_speed:
                    self.cmd_vel.angular.z += self.angular_increment
            else:        
                self.cmd_vel.angular.z = self.angular_speed
                
        elif command == 'turn right':    
            if self.cmd_vel.linear.x != 0:
                if self.cmd_vel.angular.z > -self.angular_speed:
                    self.cmd_vel.angular.z -= self.angular_increment
            else:        
                self.cmd_vel.angular.z = -self.angular_speed
   
        
        elif command == 'faster':
            if self.cmd_vel.linear.x != 0:
                if self.speed < self.max_speed:
                    self.speed += self.linear_increment
                    self.cmd_vel.linear.x = copysign(self.speed, self.cmd_vel.linear.x)

            if self.cmd_vel.angular.z != 0:
                if self.angular_speed < self.max_angular_speed:
                    self.angular_speed += self.angular_increment
                    self.cmd_vel.angular.z = copysign(self.angular_speed, self.cmd_vel.angular.z)
            
        elif command == 'slower':
            if self.cmd_vel.linear.x != 0:
                if self.speed > self.min_speed:
                    self.speed -= self.linear_increment
                    self.cmd_vel.linear.x = copysign(self.speed, self.cmd_vel.linear.x)

            if self.cmd_vel.angular.z != 0:
                if self.angular_speed > self.min_angular_speed:
                    self.angular_speed -= self.angular_increment
                    self.cmd_vel.angular.z = copysign(self.angular_speed, self.cmd_vel.angular.z)
                
        elif command in ['quarter', 'half', 'full']:
            if command == 'quarter':
                self.speed = copysign(self.max_speed / 4, self.speed)

            elif command == 'half':
                self.speed = copysign(self.max_speed / 2, self.speed)
            
            elif command == 'full':
                self.speed = copysign(self.max_speed, self.speed)
            
            if self.cmd_vel.linear.x != 0:
                self.cmd_vel.linear.x = copysign(self.speed, self.cmd_vel.linear.x)

            if self.cmd_vel.angular.z != 0:
                self.cmd_vel.angular.z = copysign(self.angular_speed, self.cmd_vel.angular.z)
                
        else:
            return

    def cleanup(self):
        # When shutting down be sure to stop the robot!
        twist = Twist()
        self.cmd_vel_pub.publish(twist)
        rospy.sleep(1)

if __name__=="__main__":
    rospy.init_node('voice_cmd_vel')
    try:
        voice_cmd_vel()
        # spin() simply keeps python from exiting
	# until this node is stopped
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("Moving the base through velocity commands terminated.")
