#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Joy
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float32MultiArray
import PyKDL
from tf_conversions import posemath
import numpy as np

# Haptic Device Poses
LEFT_POSE = None
RIGHT_POSE = None

# Button Press
PRESS1 = None
PRESS2 = None

# Rotation Frames
VIEWER_R_HAPTIC = PyKDL.Frame(PyKDL.Rotation(1,0,0, 0,0,-1, 0,1,0))     # The Matrix that rewrites a vector in the haptic base frame, to a vector in the Viewer base frame
# VIEWER_R_ROBOTBASE = PyKDL.Frame()
VIEWER_R_ROBOTBASE = PyKDL.Frame(PyKDL.Rotation(0,-1,0, 1,0,0, 0,0,1))

# Teleop Anchor event
ANCH = None
TCP0 = None
TCP0_R_ANCH = None          # To be set during the anchoring event

# Manipulator current pose
TCP = None
COUNT = None

def button1_callback(msg):
    """
    Subscriber callback function
    Returns button press event for Button1 in 3DS Haptic Touch Device
    """
    global PRESS1, ANCH
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
    global PRESS2, ANCH
    if msg.buttons[0] == 1:
        rospy.loginfo('Leader: Button2 Engaged!')
        PRESS2 = 1
    else:
        rospy.loginfo('Leader: Button2 Disengaged!')
        PRESS2 = 0
        ANCH = None

def sub4_callback(msg):
    """
    Subscriber callback function to get the pose of the underwater arm
    Sets the TCP variable to a PyKDL frame of the Manipulators current pose
    """
    global TCP
    rot = msg.data[-3:]
    angle = np.linalg.norm(rot)
    axis = rot  / angle
    axis = PyKDL.Vector(axis[0], axis[1], axis[2])

    TCP = PyKDL.Frame(PyKDL.Rotation.Rot(axis, angle), PyKDL.Vector(msg.data[0], msg.data[1], msg.data[2]))

def cp_callback(msg):
    """
    Subscriber callback function
    """    
    global LEFT_POSE
    LEFT_POSE = posemath.fromMsg(msg.pose)

def teleop():
    global ANCH, COUNT, TCP0, TCP0_R_ANCH, LEFT_POSE
    if PRESS2 == 1:
        if ANCH is not None:
            # COUNT += 1
            # if COUNT % 487 == 0: # Does the computation roughly every second (~ 487 Hz)
                
            # RELATIVE COMMANDED POSE [As represented in the Anchor frame]
            rel_cmd = ANCH.Inverse() * LEFT_POSE

            # RELATIVE COMMANDED POSE [As represented in the Robot TCP0 frame]
            command = TCP0_R_ANCH * rel_cmd * TCP0_R_ANCH.Inverse()
            command = TCP0 * command
            # Convert to Float32MultiArray
            rot = command.M.GetRot()
            array = Float32MultiArray()
            array.data = [command.p.x(), command.p.y(), command.p.z(), rot.x(), rot.y(), rot.z()]
            pub.publish(array)
            # else:
            #     pass
        else:
            TCP0 = TCP                                          
            ANCH = LEFT_POSE                   # Converts the pose message to a KDL frame
            
            haptic_R_anch = PyKDL.Frame(ANCH.M)                 # The Matrix that can rewrite the relative commanded pose to the haptic base frame
            VIEWER_R_ANCH = VIEWER_R_HAPTIC * haptic_R_anch     # The Matrix that can rewrite the relative commanded pose to the Viewer base frame
            ROBOTBASE_R_TCP0 = PyKDL.Frame(TCP0.M)

            TCP0_R_ANCH = ROBOTBASE_R_TCP0.Inverse() * VIEWER_R_ROBOTBASE.Inverse() * VIEWER_R_ANCH
            
            COUNT = 0

if __name__ == '__main__':

    # Initialize the node
    rospy.init_node("teleop_cmd")

    # Initialize the subscribers
    sub1 = rospy.Subscriber("/left/button1", Joy , callback=button1_callback)
    sub2 = rospy.Subscriber("/left/button2", Joy , callback=button2_callback)
    sub3 = rospy.Subscriber("/left/measured_cp", PoseStamped , callback=cp_callback)
    sub4 = rospy.Subscriber("/arm1/pose", Float32MultiArray, callback=sub4_callback)

    pub = rospy.Publisher('/Xd1', Float32MultiArray, queue_size=10)

    rate = rospy.Rate(100)

    while not rospy.is_shutdown():
        teleop()
        rate.sleep()