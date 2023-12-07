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
mac_ANCH_L = None
mac_ANCH_R = None

# Manipulator current pose
TCP_L = None
TCP_R = None
COUNT = None

# Gripper States
L_GRIP = -1
R_GRIP = -1

def sub1_cb(msg):
    """
    Subscriber callback function
    Returns button press event for Button1 in 3DS Haptic Touch Device
    """
    global L_GRIP, PRESS1_L, mac_ANCH_L
    if msg.buttons[0] == 1:
        if PRESS2_L == 1:
            if L_GRIP == 1:
                L_GRIP = 0
            else:
                L_GRIP = 1
        PRESS1_L = 1
    else:
        PRESS1_L = 0
        mac_ANCH_L = None

def sub2_cb(msg):
    """
    Subscriber callback function
    Returns button press event for Button2 in 3DS Haptic Touch Device
    """
    global PRESS2_L, ANCH_L, L_GRIP
    if msg.buttons[0] == 1:
        # rospy.loginfo('Leader: Button2_L Engaged!')
        if PRESS1_L == 1:
            if L_GRIP == 1:
                L_GRIP = 0
            else:
                L_GRIP = 1
        PRESS2_L = 1
    else:
        # rospy.loginfo('Leader: Button2_L Disengaged!')
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
    global R_GRIP, PRESS1_R, mac_ANCH_R
    if msg.buttons[0] == 1:
        if PRESS2_R == 1:
            if R_GRIP == 1:
                R_GRIP = 0
            else:
                R_GRIP = 1
        PRESS1_R = 1
    else:
        PRESS1_R = 0
        mac_ANCH_R = None

def sub5_cb(msg):
    """
    Subscriber callback function
    Returns button press event for Button2 in 3DS Haptic Touch Device
    """
    global PRESS2_R, ANCH_R, R_GRIP
    if msg.buttons[0] == 1:
        # rospy.loginfo('Leader: Button2_R Engaged!')
        if PRESS1_R == 1:
            if R_GRIP == 1:
                R_GRIP = 0
            else:
                R_GRIP = 1
        PRESS2_R = 1
    else:
        # rospy.loginfo('Leader: Button2_R Disengaged!')
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
    if rot_ang != 0:
        tmp_rot = tmp_rot / rot_ang
    return PyKDL.Frame(PyKDL.Rotation.Rot(tmp_rot, rot_ang*or_SF), pose.p*cp_SF)

def teleop_L():
    global ANCH_L, TCP0_L, TCP0_L_ANCH, LEFT_POSE, L_GRIP
    if (PRESS2_L == 1) and (PRESS1_L == 0):
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
    global ANCH_R, TCP0_R, TCP0_R_ANCH, RIGHT_POSE, R_GRIP
    if (PRESS2_R == 1) and (PRESS1_R == 0):
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

def teleop_vehicle():

    global mac_ANCH_L, mac_ANCH_R
    if (PRESS1_R == 1) and (PRESS1_L == 1) and (PRESS2_L == 0) and (PRESS2_R == 0):
        if (mac_ANCH_L is None) or (mac_ANCH_R is None):
            # reset both anchors
            mac_ANCH_L = LEFT_POSE
            mac_ANCH_R = RIGHT_POSE

        else:
            # Find the resultant vector for the right and left arm in the vehicle frame
            left_cmd = list(VIEWER_L_ROBOTBASE.Inverse().M * VIEWER_L_HAPTIC.M * (LEFT_POSE.p - mac_ANCH_L.p))
            right_cmd = list(VIEWER_R_ROBOTBASE.Inverse().M * VIEWER_R_HAPTIC.M * (RIGHT_POSE.p - mac_ANCH_R.p))
            
            pilot(left_cmd, right_cmd)
    
    else:
        teleop_R()
        teleop_L()

def pilot(left, right):
    # Determines which direction dominates from each haptic command - 0:x, 1:y, 2:z, {in the robot frame}
    l_dir = dom_proj(left)
    r_dir = dom_proj(right)
    
    if l_dir == r_dir:
        cmd_vel = Twist()
        if (left[l_dir] > 0 and right[r_dir] > 0):
            if (l_dir == 0) and (abs(right[r_dir]) > 0.025) and (abs(left[l_dir]) > 0.025):      # Forward
                cmd_vel.linear.x = 0.5
                rospy.loginfo('Moving Front')
                # pub4.publish(cmd_vel)
            if (l_dir == 1) and (abs(right[r_dir]) > 0.050) and (abs(left[l_dir]) > 0.050):      # Strafe Left
                cmd_vel.linear.y = 0.5
                rospy.loginfo('Moving Left')
            if (l_dir == 2) and (abs(right[r_dir]) > 0.050) and (abs(left[l_dir]) > 0.050):      # Go Up
                cmd_vel.linear.z = 0.5
                rospy.loginfo('Moving Up')

        if (left[l_dir] < 0 and right[r_dir] < 0):
            if (l_dir == 0) and (abs(right[r_dir]) > 0.025) and (abs(left[l_dir]) > 0.025):      # Backward
                cmd_vel.linear.x = -0.5
                rospy.loginfo('Moving Back')
            if (l_dir == 1) and (abs(right[r_dir]) > 0.050) and (abs(left[l_dir]) > 0.050):      # Strafe Right
                cmd_vel.linear.y = -0.5
                rospy.loginfo('Moving Right')
            if (l_dir == 2) and (abs(right[r_dir]) > 0.050) and (abs(left[l_dir]) > 0.050):      # Go Down
                cmd_vel.linear.z = -0.5
                rospy.loginfo('Moving Down')

        if (left[l_dir] > 0 and right[r_dir] < 0):
            if (l_dir == 0) and (abs(right[r_dir]) > 0.025) and (abs(left[l_dir]) > 0.025):      # Turn Right
                cmd_vel.angular.z = -0.1
                rospy.loginfo('Turning Right')

        if (left[l_dir] < 0 and right[r_dir] > 0):
            if (l_dir == 0) and (abs(right[r_dir]) > 0.025) and (abs(left[l_dir]) > 0.025):      # Turn Left
                cmd_vel.angular.z = 0.1
                rospy.loginfo('Turning Left')

        pub4.publish(cmd_vel)

def dom_proj(ctrl):
    l = map(abs,ctrl)
    return l.index(max(l))

def gripper():
    """
    Publishes the desired gripper state
    1 : Close the gripper (Grasp)
    2 : Open the gripper (Release)
    """
    array = Float32MultiArray()
    array.data = [L_GRIP, R_GRIP]
    pub3.publish(array)


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
    pub3 = rospy.Publisher('/grippers', Float32MultiArray, queue_size=10)
    pub4 = rospy.Publisher('/rexrov/cmd_vel', Twist, queue_size=10)

    rate = rospy.Rate(100)

    while not rospy.is_shutdown():
        teleop_vehicle()
        gripper()
        rate.sleep()