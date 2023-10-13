#!/usr/bin/env python

import rospy
import numpy as np
from sensor_msgs.msg import Joy
from geometry_msgs.msg import PoseStamped
import PyKDL
from tf_conversions import posemath

PRESS1 = None
PRESS2 = None
ANCH = None

def button1_callback(msg):
    """
    Subscriber callback function
    Returns button press event for Button1 in 3DS Haptic Touch Device
    """
    global PRESS1
    if msg.buttons[0] == 1:
        rospy.loginfo('Leader: Button1 Engaged!')
        PRESS1 = 1
    else:
        rospy.loginfo('Leader: Button1 Disengaged!')
        PRESS1 = 0
        ANCH = None

def button2_callback(msg):
    """
    Subscriber callback function
    Returns button press event for Button2 in 3DS Haptic Touch Device
    """
    global PRESS2
    if msg.buttons[0] == 1:
        rospy.loginfo('Leader: Button2 Engaged!')
        PRESS2 = 1
    else:
        rospy.loginfo('Leader: Button2 Disengaged!')
        PRESS2 = 0
        ANCH = None

def cp_callback(msg):
    """
    Subscriber callback function
    Returns the trajectory of haptic device while button is pressed
    """
    global ANCH
    if PRESS2 == 1:
        if ANCH is not None:
            f = posemath.fromMsg(msg.pose)
            rospy.loginfo(f.M)
        else:
            ANCH = posemath.fromMsg(msg.pose)

if __name__ == '__main__':

    # Initialize the node
    rospy.init_node("teleop_cmd")

    # setup the subscribers
    sub1 = rospy.Subscriber("/arm/button1", Joy , callback=button1_callback)
    sub2 = rospy.Subscriber("/arm/button2", Joy , callback=button2_callback)
    sub3 = rospy.Subscriber("/arm/measured_cp", PoseStamped , callback=cp_callback)
    
    rospy.spin()    # Keeps the node running until killed