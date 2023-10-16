#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Joy
from geometry_msgs.msg import PoseStamped
import PyKDL
from tf_conversions import posemath

PRESS1 = None
PRESS2 = None
ANCH = None
COUNT = None
VIEWER_R_ANCH = None
VIEWER_R_ROBOTBASE = None
ROBOTBASE_R_TCP0 = None     # This should be set at the anchoring moment

VIEWER_R_HAPTIC = PyKDL.Frame(PyKDL.Rotation(1,0,0, 0,0,-1, 0,1,0))     # The Matrix that rewrites a vector in the haptic base frame, to a vector in the Viewer base frame

def button1_callback(msg):
    """
    Subscriber callback function
    Returns button press event for Button1 in 3DS Haptic Touch Device
    """
    global PRESS1, ANCH, VIEWER_R_ANCH
    if msg.buttons[0] == 1:
        rospy.loginfo('Leader: Button1 Engaged!')
        PRESS1 = 1
    else:
        rospy.loginfo('Leader: Button1 Disengaged!')
        PRESS1 = 0
        ANCH = None
        VIEWER_R_ANCH = None

def button2_callback(msg):
    """
    Subscriber callback function
    Returns button press event for Button2 in 3DS Haptic Touch Device
    """
    global PRESS2, ANCH, VIEWER_R_ANCH
    if msg.buttons[0] == 1:
        rospy.loginfo('Leader: Button2 Engaged!')
        PRESS2 = 1
    else:
        rospy.loginfo('Leader: Button2 Disengaged!')
        PRESS2 = 0
        ANCH = None
        VIEWER_R_ANCH = None

def cp_callback(msg):
    """
    Subscriber callback function
    Returns the trajectory of haptic device while button is pressed

    The haptic device publishes this message at a freq of approximately 487 Hz
    """
    global ANCH, COUNT, VIEWER_R_ANCH
    if PRESS2 == 1:
        if ANCH is not None:
            COUNT += 1
            if COUNT % 487 == 0: # Does the computation roughly every second (~ 487 Hz)
                f = posemath.fromMsg(msg.pose)  # Converts the pose message to a KDL frame

                # RELATIVE COMMANDED POSE [As represented in the Anchor frame]
                rel_cmd = ANCH.Inverse() * f

                # RELATIVE COMMANDED POSE [As represented in the Viewer/Operator Base frame]
                viewer_rel_cmd = VIEWER_R_ANCH * rel_cmd * VIEWER_R_ANCH.Inverse()
                rospy.loginfo(viewer_rel_cmd)

            else:
                pass
        else:
            ANCH = posemath.fromMsg(msg.pose)                   # Converts the pose message to a KDL frame
            haptic_R_anch = PyKDL.Frame(ANCH.M)                 # The Matrix that can rewrite the relative commanded pose to the haptic base frame
            VIEWER_R_ANCH = VIEWER_R_HAPTIC * haptic_R_anch     # The Matrix that can rewrite the relative commanded pose to the Viewer base frame
            rospy.loginfo(VIEWER_R_ANCH)

            COUNT = 0

if __name__ == '__main__':

    # Initialize the node
    rospy.init_node("teleop_cmd")

    # setup the subscribers
    sub1 = rospy.Subscriber("/arm/button1", Joy , callback=button1_callback)
    sub2 = rospy.Subscriber("/arm/button2", Joy , callback=button2_callback)
    sub3 = rospy.Subscriber("/arm/measured_cp", PoseStamped , callback=cp_callback)

    rate = rospy.Rate(1)

    while not rospy.is_shutdown():
        # Do something
        rate.sleep()