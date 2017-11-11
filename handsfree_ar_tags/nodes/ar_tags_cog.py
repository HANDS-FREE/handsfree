#!/usr/bin/env python

"""
    ar_tags_cog.py - Version 1.0 2013-11-10
    
    Find the COG of AR tags that are detected in the field of view and publish the
    result as a PoseStamped message on the /target_pose topic
    
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
from geometry_msgs.msg import Point, PoseStamped
from ar_track_alvar_msgs.msg import AlvarMarkers

class TagsCOG():
    def __init__(self):
        rospy.init_node("ar_tags_cog")
        
        # Read in an optional list of valid tag ids
        self.tag_ids = rospy.get_param('~tag_ids', None)
        
        # Publish the COG on the /target_pose topic as a PoseStamped message
        self.tag_pub = rospy.Publisher("target_pose", PoseStamped, queue_size=5)

        rospy.Subscriber("ar_pose_marker", AlvarMarkers, self.get_tags)
        
        rospy.loginfo("Publishing combined tag COG on topic /target_pose...")
                
    def get_tags(self, msg):
        # Initialize the COG as a PoseStamped message
        tag_cog = PoseStamped()
        
        # Get the number of markers
        n = len(msg.markers)
        
        # If no markers detected, just return
        if n == 0:
            return

        # Iterate through the tags and sum the x, y and z coordinates            
        for tag in msg.markers:
            
            # Skip any tags that are not in our list
            if self.tag_ids is not None and not tag.id in self.tag_ids:
                continue
            
            # Sum up the x, y and z position coordinates of all tags
            tag_cog.pose.position.x += tag.pose.pose.position.x
            tag_cog.pose.position.y += tag.pose.pose.position.y
            tag_cog.pose.position.z += tag.pose.pose.position.z
            
             # Compute the COG
            tag_cog.pose.position.x /= n
            tag_cog.pose.position.y /= n
            tag_cog.pose.position.z /= n
            
            # Give the tag a unit orientation
            tag_cog.pose.orientation.w = 1

            # Add a time stamp and frame_id
            tag_cog.header.stamp = rospy.Time.now()
            tag_cog.header.frame_id = msg.markers[0].header.frame_id

            # Publish the COG
            self.tag_pub.publish(tag_cog)      
  
if __name__ == '__main__':
    try:
        TagsCOG()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("AR Tag Tracker node terminated.")
