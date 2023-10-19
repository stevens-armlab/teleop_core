#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Joy
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float32MultiArray
import PyKDL
from tf_conversions import posemath
import numpy as np

# Scaling Factor
cp_SF = 1
or_SF = 0.5

# Haptic Device Poses
LEFT_POSE = None
RIGHT_POSE = None

# Button Press
PRESS1_L = None
PRESS2_L = None
PRESS1_R = None
PRESS2_R = None

# Rotation Frames
VIEWER_L_HAPTIC = PyKDL.Frame(PyKDL.Rotation(1,0,0, 0,0,-1, 0,1,0))     # The Matrix that rewrites a vector in the haptic base frame, to a vector in the Viewer base frame
VIEWER_L_ROBOTBASE = PyKDL.Frame(PyKDL.Rotation(0,-1,0, 1,0,0, 0,0,1))
VIEWER_R_HAPTIC = PyKDL.Frame(PyKDL.Rotation(1,0,0, 0,0,-1, 0,1,0))     # The Matrix that rewrites a vector in the haptic base frame, to a vector in the Viewer base frame
VIEWER_R_ROBOTBASE = PyKDL.Frame(PyKDL.Rotation(0,-1,0, 1,0,0, 0,0,1))

# Teleop Anchor event
ANCH_L = None
TCP0_L = None
TCP0_L_ANCH = None          # To be set during the anchoring event
ANCH_R = None
TCP0_R = None
TCP0_R_ANCH = None

# Manipulator current pose
TCP_L = None
TCP_R = None
COUNT = None

def sub1_cb(msg):
    """
    Subscriber callback function
    Returns button press event for Button1 in 3DS Haptic Touch Device
    """
    global PRESS1_L, ANCH_L
    if msg.buttons[0] == 1:
        rospy.loginfo('Leader: Button1_L Engaged!')
        PRESS1_L = 1
    else:
        rospy.loginfo('Leader: Button1_L Disengaged!')
        PRESS1_L = 0
        ANCH_L = None

def sub2_cb(msg):
    """
    Subscriber callback function
    Returns button press event for Button2 in 3DS Haptic Touch Device
    """
    global PRESS2_L, ANCH_L
    if msg.buttons[0] == 1:
        rospy.loginfo('Leader: Button2_L Engaged!')
        PRESS2_L = 1
    else:
        rospy.loginfo('Leader: Button2_L Disengaged!')
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
    Subscriber callback function
    Returns button press event for Button1 in 3DS Haptic Touch Device
    """
    global PRESS1_R, ANCH_R
    if msg.buttons[0] == 1:
        rospy.loginfo('Leader: Button1_R Engaged!')
        PRESS1_R = 1
    else:
        rospy.loginfo('Leader: Button1_R Disengaged!')
        PRESS1_R = 0
        ANCH_R = None

def sub5_cb(msg):
    """
    Subscriber callback function
    Returns button press event for Button2 in 3DS Haptic Touch Device
    """
    global PRESS2_R, ANCH_R
    if msg.buttons[0] == 1:
        rospy.loginfo('Leader: Button2_R Engaged!')
        PRESS2_R = 1
    else:
        rospy.loginfo('Leader: Button2_R Disengaged!')
        PRESS2_R = 0
        ANCH_R = None

def sub6_cb(msg):
    """
    Subscriber callback function
    """    
    global RIGHT_POSE
    RIGHT_POSE = posemath.fromMsg(msg.pose)

def sub7_cb(msg):
    """
    Subscriber callback function to get the pose of the underwater arm
    Sets the TCP variable to a PyKDL frame of the Manipulators current pose
    """
    global TCP_L
    rot = msg.data[-3:]
    angle = np.linalg.norm(rot)
    axis = rot  / angle
    axis = PyKDL.Vector(axis[0], axis[1], axis[2])

    TCP_L = PyKDL.Frame(PyKDL.Rotation.Rot(axis, angle), PyKDL.Vector(msg.data[0], msg.data[1], msg.data[2]))

def sub8_cb(msg):
    """
    Subscriber callback function to get the pose of the underwater arm
    Sets the TCP variable to a PyKDL frame of the Manipulators current pose
    """
    global TCP_R
    rot = msg.data[-3:]
    angle = np.linalg.norm(rot)
    axis = rot  / angle
    axis = PyKDL.Vector(axis[0], axis[1], axis[2])

    TCP_R = PyKDL.Frame(PyKDL.Rotation.Rot(axis, angle), PyKDL.Vector(msg.data[0], msg.data[1], msg.data[2]))

def scale(pose):
    """
    Applies the rotational and translational scaling factor to the relative command
    """
    tmp_rot = pose.M.GetRot()
    rot_ang = tmp_rot.Normalize()
    rot_axis = tmp_rot / rot_ang

    return PyKDL.Frame(PyKDL.Rotation.Rot(rot_axis, tmp_ang*or_SF), pose.p*cp_SF)

def teleop_L():
    global ANCH_L, TCP0_L, TCP0_L_ANCH, LEFT_POSE
    if PRESS2_L == 1:
        if ANCH_L is not None:
            
            # RELATIVE COMMANDED POSE [As represented in the Anchor frame]
            rel_cmd = ANCH_L.Inverse() * LEFT_POSE
            rel_cmd = scale(rel_cmd)

            # RELATIVE COMMANDED POSE [As represented in the Robot TCP0 frame]
            command = TCP0_L_ANCH * rel_cmd * TCP0_L_ANCH.Inverse()
            command = TCP0_L * command
            # Convert to Float32MultiArray
            rot = command.M.GetRot()
            array = Float32MultiArray()
            array.data = [command.p.x(), command.p.y(), command.p.z(), rot.x(), rot.y(), rot.z()]
            pub1.publish(array)
        else:
            TCP0_L = TCP_L                                          
            ANCH_L = LEFT_POSE                   # Converts the pose message to a KDL frame
            
            haptic_L_anch = PyKDL.Frame(ANCH_L.M)                 # The Matrix that can rewrite the relative commanded pose to the haptic base frame
            VIEWER_L_ANCH = VIEWER_L_HAPTIC * haptic_L_anch     # The Matrix that can rewrite the relative commanded pose to the Viewer base frame
            ROBOTBASE_L_TCP0 = PyKDL.Frame(TCP0_L.M)

            TCP0_L_ANCH = ROBOTBASE_L_TCP0.Inverse() * VIEWER_L_ROBOTBASE.Inverse() * VIEWER_L_ANCH
 
def teleop_R():
    global ANCH_R, TCP0_R, TCP0_R_ANCH, RIGHT_POSE
    if PRESS2_R == 1:
        if ANCH_R is not None:
            
            # RELATIVE COMMANDED POSE [As represented in the Anchor frame]
            rel_cmd = ANCH_R.Inverse() * RIGHT_POSE
            rel_cmd = scale(rel_cmd)

            # RELATIVE COMMANDED POSE [As represented in the Robot TCP0 frame]
            command = TCP0_R_ANCH * rel_cmd * TCP0_R_ANCH.Inverse()
            command = TCP0_R * command
            # Convert to Float32MultiArray
            rot = command.M.GetRot()
            array = Float32MultiArray()
            array.data = [command.p.x(), command.p.y(), command.p.z(), rot.x(), rot.y(), rot.z()]
            pub2.publish(array)
        else:
            TCP0_R = TCP_R                                          
            ANCH_R = RIGHT_POSE                   # Converts the pose message to a KDL frame
            
            haptic_R_anch = PyKDL.Frame(ANCH_R.M)                 # The Matrix that can rewrite the relative commanded pose to the haptic base frame
            VIEWER_R_ANCH = VIEWER_R_HAPTIC * haptic_R_anch     # The Matrix that can rewrite the relative commanded pose to the Viewer base frame
            ROBOTBASE_R_TCP0 = PyKDL.Frame(TCP0_R.M)

            TCP0_R_ANCH = ROBOTBASE_R_TCP0.Inverse() * VIEWER_R_ROBOTBASE.Inverse() * VIEWER_R_ANCH
 



if __name__ == '__main__':

    # Initialize the node
    rospy.init_node("teleop_cmd")

    # Initialize the subscribers
    sub1 = rospy.Subscriber("/left/button1", Joy , callback=sub1_cb)
    sub2 = rospy.Subscriber("/left/button2", Joy , callback=sub2_cb)
    sub3 = rospy.Subscriber("/left/measured_cp", PoseStamped , callback=sub3_cb)
    
    sub4 = rospy.Subscriber("/right/button1", Joy , callback=sub4_cb)
    sub5 = rospy.Subscriber("/right/button2", Joy , callback=sub5_cb)
    sub6 = rospy.Subscriber("/right/measured_cp", PoseStamped , callback=sub6_cb)

    sub7 = rospy.Subscriber("/arm1/pose", Float32MultiArray, callback=sub7_cb)
    sub8 = rospy.Subscriber("/arm2/pose", Float32MultiArray, callback=sub8_cb)

    pub1 = rospy.Publisher('/Xd1', Float32MultiArray, queue_size=10)
    pub2 = rospy.Publisher('/Xd2', Float32MultiArray, queue_size=10)

    rate = rospy.Rate(100)

    while not rospy.is_shutdown():
        teleop_L()
        teleop_R()
        rate.sleep()