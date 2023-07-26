#!/usr/bin/env python

import rospy
import numpy as np
from sensor_msgs.msg import Joy
from geometry_msgs.msg import PoseStamped


engage = False

def button2_callback(msg):
    """
    Subscriber callback function
    Returns button press event for Button2 in 3DS Haptic Touch Device
    """
    global engage, anchor
    if msg.buttons[0] == 1:        
        engage=True
        rospy.loginfo('Robot-Slave: Engaged!')
    else:        
        engage=False
        rospy.loginfo('Robot-Slave: Disengaged!')
    anchor = False

def cp_callback(msg):
    """
    Subscriber callback function
    Returns the trajectory of haptic device while button is pressed
    """
    global engage, anchor, anchor_pose, start_time
    
    if engage == True:
        current_pose = msg.pose
        if anchor == False:             # Anchoring Event
            anchor_pose = current_pose
            start_time = msg.header.stamp
            rospy.loginfo(anchor_pose)
            rospy.loginfo(start_time)
            anchor = True
        else:
            # Add to relative position array
            rospy.loginfo(rel_position(current_pose.position, anchor_pose.position))
            # Add to relative orientation array
            # Add to time


def rel_position(cur, anch):
    """
    This function returns the relative position of point 'cur', wrt point 'anch'
    Precondition: cur and anch are Point msgs
    """
    return np.array([cur.x - anch.x, cur.y - anch.y, cur.z - anch.z])
    

if __name__ == '__main__':

    # Initialize the node
    rospy.init_node("desired_rel_pose_traj")

    # setup the subscribers
    sub1 = rospy.Subscriber("/arm/button2", Joy , callback=button2_callback)
    sub2 = rospy.Subscriber("/arm/measured_cp", PoseStamped , callback=cp_callback)
    
    rospy.spin()