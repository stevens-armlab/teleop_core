#!/usr/bin/env python

import rospy
import numpy as np
from sensor_msgs.msg import Joy
from geometry_msgs.msg import PoseStamped

POSE = []

def button1_callback(msg):
    """
    Subscriber callback function
    Returns button press event for Button1 in 3DS Haptic Touch Device
    """
    if msg.buttons[0] == 1:
        rospy.loginfo('Leader: Button1 Engaged!')
    else:
        rospy.loginfo('Leader: Button1 Disengaged!')

def button2_callback(msg):
    """
    Subscriber callback function
    Returns button press event for Button2 in 3DS Haptic Touch Device
    """
    if msg.buttons[0] == 1:
        rospy.loginfo('Leader: Button2 Engaged!')
    else:
        rospy.loginfo('Leader: Button2 Disengaged!')

def cp_callback(msg):
    """
    Subscriber callback function
    Returns the trajectory of haptic device while button is pressed
    """
    pass

if __name__ == '__main__':

    # Initialize the node
    rospy.init_node("teleop_cmd")

    # setup the subscribers
    sub1 = rospy.Subscriber("/arm/button1", Joy , callback=button1_callback)
    sub2 = rospy.Subscriber("/arm/button2", Joy , callback=button2_callback)
    sub3 = rospy.Subscriber("/arm/measured_cp", PoseStamped , callback=cp_callback)
    
    rospy.spin()    # Keeps the node running until killed