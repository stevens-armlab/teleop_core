#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Joy
from geometry_msgs.msg import PoseStamped, Twist
from std_msgs.msg import Float32MultiArray
import PyKDL
from tf_conversions import posemath
import numpy as np

# Scaling Factor
cp_SF = 1
or_SF = 1

# Haptic Device Poses
LEFT_POSE = None

# Button Press
PRESS1_L = None
PRESS2_L = None

# Rotation Frames
VIEWER_L_HAPTIC = PyKDL.Frame(PyKDL.Rotation(1,0,0, 0,0,-1, 0,1,0))     # The Matrix that rewrites a vector in the haptic base frame, to a vector in the Viewer base frame
VIEWER_L_ROBOTBASE = PyKDL.Frame(PyKDL.Rotation(0,-1,0, 1,0,0, 0,0,1))

# Teleop Anchor event
ANCH_L = None
TCP0_L = None
TCP0_L_ANCH = None          # To be set during the anchoring event

# Manipulator current pose
TCP_L = None

def sub1_cb(msg):
    """
    Subscriber callback function
    Returns button press event for Button1 in 3DS Haptic Touch Device
    """
    global PRESS1_L
    if msg.buttons[0] == 1:
        PRESS1_L = 1
    else:
        PRESS1_L = 0

def sub2_cb(msg):
    """
    Subscriber callback function
    Returns button press event for Button2 in 3DS Haptic Touch Device
    """
    global PRESS2_L, ANCH_L
    if msg.buttons[0] == 1:
        PRESS2_L = 1
    else:
        PRESS2_L = 0
        ANCH_L = None

def sub3_cb(msg):
    """
    Subscriber callback function
    """
    global LEFT_POSE
    LEFT_POSE = posemath.fromMsg(msg.pose)

def sub4_cb(msg):
    """
    Subscriber callback function to get the pose of the franka arm
    Sets the TCP variable to a PyKDL frame of the Manipulators current pose
    """
    global TCP_L
    TCP_L = posemath.fromMsg(msg.pose)

def teleop():
    global ANCH_L, TCP0_L, TCP0_L_ANCH, LEFT_POSE, L_GRIP
    
    if PRESS2_L == 1:
        if ANCH_L is not None:
            # RELATIVE COMMANDED POSE [As represented in the Anchor frame]
            rel_cmd = ANCH_L.Inverse() * LEFT_POSE

            # RELATIVE COMMANDED POSE [As represented in the Robot TCP0 frame]
            command = TCP0_L_ANCH * rel_cmd * TCP0_L_ANCH.Inverse()
            command = TCP0_L * command

            # RESOLVED RATES
        else:
            TCP0_L = TCP_L
            ANCH_L = LEFT_POSE                   # Converts the pose message to a KDL frame
            
            haptic_L_anch = PyKDL.Frame(ANCH_L.M)               # The Matrix that can rewrite the relative commanded pose to the haptic base frame
            VIEWER_L_ANCH = VIEWER_L_HAPTIC * haptic_L_anch     # The Matrix that can rewrite the relative commanded pose to the Viewer base frame
            ROBOTBASE_L_TCP0 = PyKDL.Frame(TCP0_L.M)

            TCP0_L_ANCH = ROBOTBASE_L_TCP0.Inverse() * VIEWER_L_ROBOTBASE.Inverse() * VIEWER_L_ANCH

if __name__ == '__main__':
    try:
        # Initialize the node
        rospy.init_node("teleop_fr3")

        # Initialize the subscribers
        sub1 = rospy.Subscriber("/arm/button1", Joy , callback=sub1_cb)
        sub2 = rospy.Subscriber("/arm/button2", Joy , callback=sub2_cb)
        sub3 = rospy.Subscriber("/arm/measured_cp", PoseStamped , callback=sub3_cb)
        sub4 = rospy.Subscriber("/fr3_rel_cp", PoseStamped , callback=sub4_cb)

        rate = rospy.Rate(100)

        while not rospy.is_shutdown():
            teleop()
            rate.sleep()
        
    except rospy.ROSInterruptException:
        pass    