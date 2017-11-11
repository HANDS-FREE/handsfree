#!/usr/bin/env python

"""
    head_tracker.py - Version 2.0 2013-08-23
    
    Move the head to track an object pose published on the /target PoseStamped topic.
    
    Works with either the dynamixel_motor and abotix package.
    
    Created for the Pi Robot Project: http://www.pirobot.org
    Copyright (c) 2013 Patrick Goebel.  All rights reserved.

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.
    
    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details at:
    
    http://www.gnu.org/licenses/gpl.html
"""

import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64
from geometry_msgs.msg import PointStamped, Point, Pose, PoseStamped
from math import radians
import tf
import os, thread

# Initialize the node
rospy.init_node("head_tracker")

# Are we running in fake mode?
FAKE = rospy.get_param('~sim', False)

# For fake mode we use the arbotix controller package and position tracking
if FAKE:
    CONTROLLER_TYPE = 'arbotix'
    TRACKER_TYPE = 'position'
else:
    # Specify either 'dynamixel_motor' or 'arbotix' for the controller package
    CONTROLLER_TYPE = rospy.get_param('~controller_type', 'arbotix')
    
    # Specify either 'speed' or 'position' for the type of tracking
    TRACKER_TYPE = rospy.get_param('~tracker_type', 'speed')
    
# Import the appropriate services for the type of controller
if CONTROLLER_TYPE == 'arbotix':
    from arbotix_msgs.srv import *
else:
    from dynamixel_controllers.srv import *
    
class HeadTracker():
    def __init__(self):
        
        rospy.on_shutdown(self.shutdown)
        
        self.rate = rospy.get_param('~rate', 10)
        
        r = rospy.Rate(self.rate)
        
        self.tick = 1.0 / self.rate
            
        self.enable_target_search = rospy.get_param('~enable_target_search', False)
        
        # How long we are willing to wait (in seconds) before searching?
        self.search_target_timeout = rospy.get_param('~search_target_timeout', 10)
        
        # How long we are willing to wait (in seconds) before re-centering the servos?
        self.recenter_timeout = rospy.get_param('~recenter_timeout', 10)

        # What are the names of the pan and tilt joints in the list of dynamixels?
        self.head_pan_joint = rospy.get_param('~head_pan_joint', 'head_pan_joint')
        self.head_tilt_joint = rospy.get_param('~head_tilt_joint', 'head_tilt_joint')
        
        self.joints = [self.head_pan_joint, self.head_tilt_joint]
        
        # What is the name of the camera link?
        self.camera_link = rospy.get_param('~camera_link', 'camera_link')

        # Joint speeds are given in radians per second
        self.default_joint_speed = rospy.get_param('~default_joint_speed', 0.3)
        self.max_joint_speed = rospy.get_param('~max_joint_speed', 0.5)
        self.min_joint_speed = rospy.get_param('~min_joint_speed', 0.01)
        
        # Only update the speed if it differs by this much (rad/s) from the last update
        # If this is set too low (i.e. the updates are too fast), the servo can behave
        # unpredictably.
        self.speed_update_threshold = rospy.get_param('~speed_update_threshold', 0.1)
        
        # How far ahead or behind the target (in radians) should we aim for?
        self.lead_target_angle = rospy.get_param('~lead_target_angle', 0.5)
        
        # How far ahead or behind the target (in radians) should we aim for?
        self.max_lead_target_angle = rospy.get_param('~max_lead_target_angle', 0.5)
        
        # The pan/tilt thresholds indicate how far (in radians) the target needs to be off-center
        # before we make a movement.
        self.pan_threshold = rospy.get_param('~pan_threshold', 0.02)
        self.tilt_threshold = rospy.get_param('~tilt_threshold', 0.02)
        
        # The gain_pan and gain_tilt parameter determine how responsive the servo movements are.
        # If these are set too high, oscillation can result.
        self.gain_pan = rospy.get_param('~gain_pan', 1.5)
        self.gain_tilt = rospy.get_param('~gain_tilt', 1.5)
        
        # For simulated position tracking, setting the gain too high causes oscillations
        if FAKE and TRACKER_TYPE == 'position':
            self.gain_pan = 1.5
            self.gain_tilt = 1.5
        
        # Set limits on how far we can pan or tilt
        self.max_pan = rospy.get_param('~max_pan', radians(125))
        self.min_pan = rospy.get_param('~min_pan', radians(-125))
        self.max_tilt = rospy.get_param('~max_tilt', radians(90))
        self.min_tilt = rospy.get_param('~min_tilt', radians(-90))
        
        # Initialize the servo services and publishers
        self.init_servos()
        
        # Initialize tf listener
        self.tf = tf.TransformListener()

        # Allow tf to catch up        
        rospy.sleep(2)
        
        # Set a flag to indicate when the target has been lost
        self.target_visible = False
        
        # Set a timer to determine how long a target is no longer visible
        self.target_lost_time = 0.0
        
        # A flag to indicate whether we're in wait mode
        self.waiting = False
        
        # Get a lock for updating the self.move_cmd values
        self.lock = thread.allocate_lock()
                
        # Subscribe the the 'joint_states' topic so we can know how the joints are positioned
        rospy.loginfo("Subscribing to joint_states...")
        
        self.joint_state = JointState()
        
        rospy.Subscriber('joint_states', JointState, self.update_joint_state)
        
        # Wait until we actually have joint state values
        while self.joint_state == JointState():
            rospy.sleep(1)
            
        # Center the pan and tilt servos
        self.center_head_servos()
        
        # Initialize the pan and tilt speeds     
        if TRACKER_TYPE == 'position':
            pan_speed = tilt_speed = self.max_joint_speed
            
            self.set_servo_speed(self.head_pan_joint, pan_speed)
            self.set_servo_speed(self.head_tilt_joint, tilt_speed)
        else:
            pan_speed = tilt_speed = 0.0
        
        # Wait for the target topic to become alive
        rospy.loginfo("Waiting for target topic...")
        
        rospy.wait_for_message('target_topic', PoseStamped)
        
        # Subscribe to the target_pose topic queue no more than 1 message
        if CONTROLLER_TYPE == 'arbotix' and TRACKER_TYPE == 'position':
            rospy.Subscriber('target_topic', PoseStamped, self.update_joint_positions, queue_size=1)
        else:
            rospy.Subscriber('target_topic', PoseStamped, self.update_joint_speeds, queue_size=1)
        
        rospy.loginfo("Target messages detected. Starting tracker...")
                        
        while not rospy.is_shutdown():
            # Acquire the lock
            self.lock.acquire()
            
            try:
                # If we have lost the target, stop the servos incrementally for smoother tracking
                if not self.target_visible:
                    if not self.waiting:
                        pan_speed /= 1.1
                        tilt_speed /= 1.1
                        
                        # Keep track of how long the target is lost
                        self.target_lost_time += self.tick                        
                else:
                    pan_speed = self.pan_speed
                    tilt_speed = self.tilt_speed
                
                    self.target_visible = False
                    self.waiting = False
                    self.target_lost_time = 0.0
                
                # If the target is lost long enough, re-center the servos
                if self.target_lost_time > self.recenter_timeout:
                    rospy.loginfo("Cannot find target.")
                    self.center_head_servos()
                    
                    self.waiting = True
                    self.target_lost_time = 0.0
                    rospy.loginfo("Waiting for target to reappear...")
                    
                # If the target is lost for a bit, search for it
                elif self.enable_target_search and self.target_lost_time > self.search_target_timeout:
                    rospy.loginfo("Searching for target...")
                    self.search_for_target()
                    
                    self.target_lost_time += self.recenter_timeout
                    
                else:               
                    # Only update the pan speed if it differs enough from the last value
                    if TRACKER_TYPE == 'speed' and abs(self.last_pan_speed - pan_speed) > self.speed_update_threshold:
                        self.set_servo_speed(self.head_pan_joint, pan_speed)
                        self.last_pan_speed = pan_speed
                                            
                    # Update the pan position   
                    self.set_servo_position(self.head_pan_joint, self.pan_position)
    
                    # Only update the tilt speed if it differs enough from the last value
                    if TRACKER_TYPE == 'speed' and abs(self.last_tilt_speed - tilt_speed) > self.speed_update_threshold:
                        self.set_servo_speed(self.head_tilt_joint, tilt_speed)
                        self.last_tilt_speed = tilt_speed
                        
                    # Update the tilt position   
                    self.set_servo_position(self.head_tilt_joint, self.tilt_position)
                    
            finally:
                # Release the lock
                self.lock.release()
                                    
            r.sleep()
            
    def init_servos(self):
        # Create dictionaries to hold the speed, position and torque controllers
        self.servo_speed = dict()
        self.servo_position = dict()
        self.torque_enable = dict()

        # Connect to the set_speed services and define a position publisher for each servo
        rospy.loginfo("Waiting for joint controllers services...")
                
        for joint in sorted(self.joints):
            # The set_speed services
            set_speed_service = '/' + joint + '/set_speed'
            rospy.wait_for_service(set_speed_service)
            self.servo_speed[joint] = rospy.ServiceProxy(set_speed_service, SetSpeed, persistent=True)

            # Initialize the servo speed to the default_joint_speed
            self.servo_speed[joint](self.default_joint_speed)

            # The position controllers
            self.servo_position[joint] = rospy.Publisher('/' + joint + '/command', Float64, queue_size=5)
            
            # A service to relax (disable torque) a servo
            if CONTROLLER_TYPE == 'arbotix':
                torque_service = '/' + joint + '/relax'
                rospy.wait_for_service(torque_service) 
                self.torque_enable[joint] = rospy.ServiceProxy(torque_service, Relax)
                # Start the servo in the relaxed state
                self.torque_enable[joint]()
            else:
                torque_service = '/' + joint + '/torque_enable'
                rospy.wait_for_service(torque_service) 
                self.torque_enable[joint] = rospy.ServiceProxy(torque_service, TorqueEnable)
                self.torque_enable[joint](False)
        
        self.pan_position = 0
        self.tilt_position = 0
        self.pan_speed = 0.0
        self.tilt_speed = 0.0
        
        self.last_tilt_speed = 0
        self.last_pan_speed = 0
        
    def set_servo_speed(self, servo, speed):
        self.servo_speed[servo](speed)
        
    def set_servo_position(self, servo, position):
        self.servo_position[servo].publish(position)
        
    def update_joint_speeds(self, msg):
        # Acquire the lock
        self.lock.acquire()
        
        try:
            # If message is empty, return immediately
            if msg == PointStamped():
                return
            
            # If we get this far, the target is visible
            self.target_visible = True
    
            # Get position component of the message
            target = PointStamped()
            target.header.frame_id = msg.header.frame_id
            target.point = msg.pose.position
                    
            # Project the target point onto the camera link
            camera_target = self.tf.transformPoint(self.camera_link, target)
            
            # The virtual camera image is in the y-z plane
            pan = -camera_target.point.y
            tilt = -camera_target.point.z
            
            # Compute the distance to the target in the x direction
            distance = float(abs(camera_target.point.x))
            
            # Convert the pan and tilt values from meters to radians
            try:
                pan /= distance
                tilt /= distance
            except:
                # Check for exceptions (NaNs) and use minumum range as fallback
                pan /= 0.5
                tilt /= 0.5
                          
            # Get the current pan and tilt position
            try:
                current_pan = self.joint_state.position[self.joint_state.name.index(self.head_pan_joint)]
                current_tilt = self.joint_state.position[self.joint_state.name.index(self.head_tilt_joint)]
            except:
                return
                
            # Pan the camera only if the displacement of the target point exceeds the threshold 
            if abs(pan) > self.pan_threshold:
                # Set the pan speed proportion to the horizontal displacement of the target """
                self.pan_speed = trunc(min(self.max_joint_speed, max(self.min_joint_speed, self.gain_pan * abs(pan))), 2)
                   
                if pan > 0:
                    self.pan_position = max(self.min_pan, current_pan - self.lead_target_angle)
                else:
                    self.pan_position = min(self.max_pan, current_pan + self.lead_target_angle)
            else:
                self.pan_position = current_pan
                self.pan_speed = self.min_joint_speed
            
            # Tilt the camera only if the displacement of the target point exceeds the threshold
            if abs(tilt) > self.tilt_threshold:
                # Set the tilt speed proportion to the vertical displacement of the target
                self.tilt_speed = trunc(min(self.max_joint_speed, max(self.min_joint_speed, self.gain_tilt * abs(tilt))), 2)
                
                if tilt < 0:
                    self.tilt_position = max(self.min_tilt, current_tilt - self.lead_target_angle)
                else:
                    self.tilt_position = min(self.max_tilt, current_tilt + self.lead_target_angle)
    
            else:
                self.tilt_position = current_tilt
                self.tilt_speed = self.min_joint_speed
                
        finally:
            # Release the lock
            self.lock.release()
            
    def update_joint_positions(self, msg):
        # Acquire the lock
        self.lock.acquire()
        
        try:
            # Some publishers will continue to publish and empty message even when there is no 
            # point present.  In this case we want to return without setting the target_visible flag.
            if msg == PointStamped():
                return
            
            # If we get this far, the target is visible
            self.target_visible = True
    
            # We only need the position component of the target pose for tracking which can
            # be stored as a PointStamped() message.
            target = PointStamped()
            target.header.frame_id = msg.header.frame_id
            target.point = msg.pose.position
                    
            # Project the target point onto the camera link
            camera_target = self.tf.transformPoint(self.camera_link, target)
            
            # The virtual camera image is in the y-z plane
            pan = -camera_target.point.y
            tilt = -camera_target.point.z
            
            # Compute the distance to the target in the x direction
            distance = float(abs(camera_target.point.x))
            
            # Convert the pan and tilt values from meters to radians by dividing by the distance to the target.
            # Since the Kinect is or Xtion is blind to distance within 0.5 meters, check for an exception and
            # use 0.5 meters as a fall back.
            try:
                pan /= distance
                tilt /= distance
            except:
                pan /= 0.5
                tilt /= 0.5
                
            # Pan the camera only if the displacement of the target point exceeds the threshold.
            if abs(pan) > self.pan_threshold:
    
                current_pan = self.joint_state.position[self.joint_state.name.index(self.head_pan_joint)]
                
                delta_pan = min(self.max_lead_target_angle, 0.25 * self.gain_pan * abs(pan))
                    
                if pan > 0:
                    self.pan_position = max(self.min_pan, current_pan - delta_pan)
                else:
                    self.pan_position = min(self.max_pan, current_pan + delta_pan)
                    
            else:
                self.pan_position = max(self.min_pan, min(self.max_pan, pan))
            
            # Tilt the camera only if the displacement of the target point exceeds the threshold.
            if abs(tilt) > self.tilt_threshold:
                
                current_tilt = self.joint_state.position[self.joint_state.name.index(self.head_tilt_joint)]
                
                delta_tilt = min(self.max_lead_target_angle, 0.25 * self.gain_tilt * abs(tilt))
    
                if tilt < 0:
                    self.tilt_position = max(self.min_tilt, current_tilt - delta_tilt)
                else:
                    self.tilt_position = min(self.max_tilt, current_tilt + delta_tilt)
    
            else:
                self.tilt_position = max(self.min_tilt, min(self.max_tilt, tilt))
                
        finally:
            self.lock.release()
                        
    def search_for_target(self):
        # First pan one way with the head down a bit
        self.servo_speed[self.head_pan_joint](self.default_joint_speed)
        self.servo_speed[self.head_tilt_joint](self.default_joint_speed)   
            
        self.servo_position[self.head_pan_joint].publish(self.max_pan)
        self.servo_position[self.head_tilt_joint].publish(0.1)
        
        current_pan = self.joint_state.position[self.joint_state.name.index(self.head_pan_joint)]      
 
        while not self.target_visible and not rospy.is_shutdown() and current_pan < 0.9 * self.max_pan:
            self.servo_position[self.head_pan_joint].publish(self.max_pan)
            rospy.sleep(1)
            current_pan = self.joint_state.position[self.joint_state.name.index(self.head_pan_joint)]
        
        # Now pan the other way
        self.servo_position[self.head_pan_joint].publish(self.min_pan)
        self.servo_position[self.head_tilt_joint].publish(0.1)
        
        current_pan = self.joint_state.position[self.joint_state.name.index(self.head_pan_joint)]
             
        while not self.target_visible and not rospy.is_shutdown() and current_pan > 0.9 * self.min_pan:
            self.servo_position[self.head_pan_joint].publish(self.min_pan)
            rospy.sleep(1)
            current_pan = self.joint_state.position[self.joint_state.name.index(self.head_pan_joint)] 
            
    def center_head_servos(self):
        rospy.loginfo("Centering servos...")
        
        self.servo_speed[self.head_pan_joint](self.default_joint_speed)
        self.servo_speed[self.head_tilt_joint](self.default_joint_speed)
        
        current_tilt = self.joint_state.position[self.joint_state.name.index(self.head_tilt_joint)]
        current_pan = self.joint_state.position[self.joint_state.name.index(self.head_pan_joint)]

        while abs(current_tilt) > 0.05 or abs(current_pan) > 0.05:
            self.servo_position[self.head_pan_joint].publish(0)
            self.servo_position[self.head_tilt_joint].publish(0)
            
            rospy.sleep(0.5)
            
            current_tilt = self.joint_state.position[self.joint_state.name.index(self.head_tilt_joint)]
            current_pan = self.joint_state.position[self.joint_state.name.index(self.head_pan_joint)]    
                        
        self.set_servo_speed(self.head_pan_joint, 0)
        self.set_servo_speed(self.head_tilt_joint, 0)

                        
    def stop_servos(self):
        rospy.loginfo("Stopping servos...")
        
        current_tilt = self.joint_state.position[self.joint_state.name.index(self.head_tilt_joint)]
        current_pan = self.joint_state.position[self.joint_state.name.index(self.head_pan_joint)]

        self.servo_position[self.head_pan_joint].publish(current_tilt)
        self.servo_position[self.head_tilt_joint].publish(current_pan)
            
            
    def update_joint_state(self, msg):
        try:
            test = msg.name.index(self.head_pan_joint)
            self.joint_state = msg
        except:
            pass
        
    def shutdown(self):
        rospy.loginfo("Shutting down head tracking node...")
        self.center_head_servos()
        
        # Relax all servos to give them a rest.
        rospy.loginfo("Relaxing pan and tilt servos.")
        
        for servo in self.joints:
            if CONTROLLER_TYPE == 'arbotix':
                self.torque_enable[servo]()
            else:
                self.torque_enable[servo](False)
                    
def trunc(f, n):
    '''Truncates/pads a float f to n decimal places without rounding'''
    slen = len('%.*f' % (n, f))
    return float(str(f)[:slen])
                   
if __name__ == '__main__':
    try:
        HeadTracker()
    except rospy.ROSInterruptException:
        rospy.loginfo("Head tracking node terminated.")




