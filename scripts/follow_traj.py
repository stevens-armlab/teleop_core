#!/usr/bin/env python

import argparse
import rospkg
import rospy
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import yaml

def load_config():
    """
    Returns a list consisting of the joint states trajectory from the config file
    """
    parser = argparse.ArgumentParser()
    parser.add_argument('--config', type=str, required=True, help="Path to the YAML configuration file")
    args = parser.parse_args()

    rospack = rospkg.RosPack()
    package_path = rospack.get_path("teleop_core")
    yaml_file_path = package_path + '/config/' + args.config + '.yaml'

    with open(yaml_file_path, 'r') as yaml_file:
        data = yaml.safe_load(yaml_file)

    return [point['Joint_Positions'] for point in data]

def publish_joint_trajectory():
    
    input_traj = load_config()
    joint_names = ["shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint", "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"]

    traj_msg = JointTrajectory()
    traj_msg.joint_names = joint_names

    rospy.init_node('follow_traj', anonymous=True)
    pub = rospy.Publisher('eff_joint_traj_controller/command', JointTrajectory, queue_size=10)
    
    dt = 0.025
    rate = rospy.Rate(1/dt)  # 40 Hz publishing rate

    i = 0
    while not rospy.is_shutdown():
        # Create trajectory points
        point1 = JointTrajectoryPoint()
        point2 = JointTrajectoryPoint()

        if i < len(input_traj) -1:
            point1.positions = input_traj[i]
            point2.positions = input_traj[i+1]

        else:
            point1.positions = input_traj[len(input_traj) -1]
            point2.positions = input_traj[len(input_traj) -1]

        traj_msg.points = [point1, point2]

        point1.time_from_start = rospy.Duration(0.0)  # Start time
        point2.time_from_start = rospy.Duration(dt)  # Time to reach this point in seconds

        traj_msg.header.stamp = rospy.Time.now()
        pub.publish(traj_msg)
        i+=1
        rate.sleep()

if __name__ == '__main__':
    try:
        publish_joint_trajectory()
    except rospy.ROSInterruptException:
        pass
