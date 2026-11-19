"""Single-arm teleop for the xArm7 (RCM / minimally invasive surgery setup).

Button 1 toggles the RCM position lock (rotation only while locked).
Hold button 2 (clutch) to drive the arm. While not clutched, the current robot
pose is republished so the arm holds position.
Publishes geometry_msgs/PoseStamped on ``go_to_pose``.
"""

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from sensor_msgs.msg import Joy

from teleop_core import kdl_utils


class TeleopXArm7(Node):

    def __init__(self):
        super().__init__('teleop_xarm7')
        self.declare_parameter('rate', 100.0)
        self.declare_parameter('position_scale', 5.0)
        self.declare_parameter('orientation_scale', 1.0)
        # Identity = "Srikarran desk"; "Ehsan desk" = [-1,0,0, 0,-1,0, 0,0,1]
        self.declare_parameter('viewer_R_robotbase', [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0])
        self.declare_parameter('haptic_pose_topic', '/arm/measured_cp')
        self.declare_parameter('button1_topic', '/arm/button1')
        self.declare_parameter('button2_topic', '/arm/button2')
        self.declare_parameter('robot_pose_topic', '/rcm_pos')
        self.declare_parameter('command_topic', '/go_to_pose')

        gp = lambda n: self.get_parameter(n).value  # noqa: E731
        self.pos_sf = gp('position_scale')
        self.ori_sf = gp('orientation_scale')
        self.viewer_robotbase = kdl_utils.rotation_from_list(gp('viewer_R_robotbase'))

        self.haptic = None
        self.tcp = None
        self.robot_msg = None   # last robot PoseStamped (also supplies the header)
        self.rcm_locked = False
        self.command_button = 0
        self.anchor = None
        self.tcp0 = None
        self.tcp0_anchor = None

        self.create_subscription(Joy, gp('button1_topic'), self.button1_cb, 10)
        self.create_subscription(Joy, gp('button2_topic'), self.button2_cb, 10)
        self.create_subscription(PoseStamped, gp('haptic_pose_topic'), self.haptic_cb, 10)
        self.create_subscription(PoseStamped, gp('robot_pose_topic'), self.robot_cb, 10)
        self.pub = self.create_publisher(PoseStamped, gp('command_topic'), 50)
        self.create_timer(1.0 / gp('rate'), self.teleop)

    def button1_cb(self, msg):
        # The Touch driver publishes button messages on change, so a press toggles once.
        if msg.buttons and msg.buttons[0] == 1:
            self.rcm_locked = not self.rcm_locked
            self.get_logger().info('RCM position locked' if self.rcm_locked else 'RCM free to move')

    def button2_cb(self, msg):
        self.command_button = 1 if msg.buttons and msg.buttons[0] == 1 else 0
        if not self.command_button:
            self.anchor = None

    def haptic_cb(self, msg):
        self.haptic = kdl_utils.pose_to_frame(msg.pose)

    def robot_cb(self, msg):
        self.tcp = kdl_utils.pose_to_frame(msg.pose)
        self.robot_msg = msg

    def teleop(self):
        if self.robot_msg is None:
            self.get_logger().warn('Waiting for robot pose...', throttle_duration_sec=2.0)
            return
        if self.command_button != 1:
            self.pub.publish(self.robot_msg)
            return
        if self.haptic is None:
            self.get_logger().warn('Waiting for haptic pose...', throttle_duration_sec=2.0)
            return
        if self.anchor is None:
            self.tcp0 = self.tcp
            self.anchor = self.haptic
            self.tcp0_anchor = kdl_utils.anchor_transform(
                self.anchor, self.tcp0, self.viewer_robotbase)
            return
        command = kdl_utils.relative_command(self.anchor, self.haptic, self.tcp0, self.tcp0_anchor,
                                             self.pos_sf, self.ori_sf,
                                             lock_position=self.rcm_locked)
        out = PoseStamped()
        out.header.frame_id = self.robot_msg.header.frame_id
        out.header.stamp = self.get_clock().now().to_msg()
        out.pose = kdl_utils.frame_to_pose(command)
        self.get_logger().debug('Sending command')
        self.pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = TeleopXArm7()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
