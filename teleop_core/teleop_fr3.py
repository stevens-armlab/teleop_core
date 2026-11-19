"""Single-arm teleop for the Franka Research 3 using one 3D Systems Touch.

Hold button 2 (clutch) to drive the arm; releasing it re-anchors on the next press.
Publishes geometry_msgs/Pose on ``go_to_pose``.
"""

import rclpy
from geometry_msgs.msg import Pose, PoseStamped
from rclpy.node import Node
from sensor_msgs.msg import Joy

from teleop_core import kdl_utils


class TeleopFR3(Node):

    def __init__(self):
        super().__init__('teleop_fr3')
        self.declare_parameter('rate', 10.0)
        self.declare_parameter('position_scale', 1.0)
        self.declare_parameter('orientation_scale', 1.0)
        self.declare_parameter('viewer_R_robotbase', [0.0, -1.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0])
        self.declare_parameter('haptic_pose_topic', '/arm/measured_cp')
        self.declare_parameter('button1_topic', '/arm/button1')
        self.declare_parameter('button2_topic', '/arm/button2')
        self.declare_parameter('robot_pose_topic', '/direct_transform')
        self.declare_parameter('command_topic', '/go_to_pose')

        gp = lambda n: self.get_parameter(n).value  # noqa: E731
        self.pos_sf = gp('position_scale')
        self.ori_sf = gp('orientation_scale')
        self.viewer_robotbase = kdl_utils.rotation_from_list(gp('viewer_R_robotbase'))

        self.haptic = None      # current haptic pose (PyKDL.Frame)
        self.tcp = None         # current robot pose (PyKDL.Frame)
        self.press1 = 0
        self.press2 = 0
        self.anchor = None      # haptic pose at the anchoring event
        self.tcp0 = None        # robot pose at the anchoring event
        self.tcp0_anchor = None

        self.create_subscription(Joy, gp('button1_topic'), self.button1_cb, 10)
        self.create_subscription(Joy, gp('button2_topic'), self.button2_cb, 10)
        self.create_subscription(PoseStamped, gp('haptic_pose_topic'), self.haptic_cb, 10)
        self.create_subscription(Pose, gp('robot_pose_topic'), self.robot_cb, 10)
        self.pub = self.create_publisher(Pose, gp('command_topic'), 50)
        self.create_timer(1.0 / gp('rate'), self.teleop)

    def button1_cb(self, msg):
        self.press1 = 1 if msg.buttons and msg.buttons[0] == 1 else 0

    def button2_cb(self, msg):
        self.press2 = 1 if msg.buttons and msg.buttons[0] == 1 else 0
        if not self.press2:
            self.anchor = None

    def haptic_cb(self, msg):
        self.haptic = kdl_utils.pose_to_frame(msg.pose)

    def robot_cb(self, msg):
        self.tcp = kdl_utils.pose_to_frame(msg)

    def teleop(self):
        if self.press2 != 1:
            return
        if self.haptic is None or self.tcp is None:
            self.get_logger().warn('Waiting for haptic and robot poses...',
                                   throttle_duration_sec=2.0)
            return
        if self.anchor is None:
            self.tcp0 = self.tcp
            self.anchor = self.haptic
            self.tcp0_anchor = kdl_utils.anchor_transform(
                self.anchor, self.tcp0, self.viewer_robotbase)
            return
        command = kdl_utils.relative_command(self.anchor, self.haptic, self.tcp0, self.tcp0_anchor,
                                             self.pos_sf, self.ori_sf)
        self.pub.publish(kdl_utils.frame_to_pose(command))


def main(args=None):
    rclpy.init(args=args)
    node = TeleopFR3()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
