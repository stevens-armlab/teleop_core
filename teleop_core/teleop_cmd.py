"""Bimanual UVMS (RexROV) teleop with two 3D Systems Touch devices (``left``/``right``).

Per device:
  * hold button 2 only        -> drive that arm (anchored relative motion)
  * press 1 while holding 2   -> toggle that side's gripper
Both devices, button 1 only -> pilot the vehicle by moving both styluses together.

Publishes:
  Xd1, Xd2   std_msgs/Float32MultiArray [x, y, z, rx, ry, rz] (rotation vector)
  grippers   std_msgs/Float32MultiArray [left, right]  (-1 unset, 1 grasp, 0 release)
  cmd_vel    geometry_msgs/Twist
"""

import PyKDL
import rclpy
from geometry_msgs.msg import PoseStamped, Twist
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_msgs.msg import Float32MultiArray

from teleop_core import kdl_utils

VIEWER_ROBOTBASE = PyKDL.Frame(PyKDL.Rotation(0, -1, 0, 1, 0, 0, 0, 0, 1))


class Side:
    """State for one haptic device / arm pair."""

    def __init__(self, name):
        self.name = name
        self.haptic = None
        self.tcp = None
        self.press1 = 0
        self.press2 = 0
        self.grip = -1
        self.anchor = None
        self.tcp0 = None
        self.tcp0_anchor = None
        self.vehicle_anchor = None

    def toggle_grip(self):
        self.grip = 0 if self.grip == 1 else 1

    def button1(self, msg):
        if msg.buttons and msg.buttons[0] == 1:
            if self.press2 == 1:
                self.toggle_grip()
            self.press1 = 1
        else:
            self.press1 = 0
            self.vehicle_anchor = None

    def button2(self, msg):
        if msg.buttons and msg.buttons[0] == 1:
            if self.press1 == 1:
                self.toggle_grip()
            self.press2 = 1
        else:
            self.press2 = 0
            self.anchor = None

    def haptic_cb(self, msg):
        self.haptic = kdl_utils.pose_to_frame(msg.pose)

    def arm_cb(self, msg):
        if len(msg.data) >= 6:
            self.tcp = kdl_utils.frame_from_rotvec(msg.data)


class TeleopCmd(Node):

    def __init__(self):
        super().__init__('teleop_cmd')
        self.declare_parameter('rate', 100.0)
        self.declare_parameter('position_scale', 2.0)
        self.declare_parameter('orientation_scale', 1.0)
        self.declare_parameter('cmd_vel_topic', '/rexrov/cmd_vel')

        self.pos_sf = self.get_parameter('position_scale').value
        self.ori_sf = self.get_parameter('orientation_scale').value

        self.left = Side('left')
        self.right = Side('right')
        self.arm_pubs = {}
        pairs = ((self.left, '/arm1/pose', '/Xd1'), (self.right, '/arm2/pose', '/Xd2'))
        for side, arm, xd in pairs:
            dev = '/' + side.name
            self.create_subscription(Joy, dev + '/button1', side.button1, 10)
            self.create_subscription(Joy, dev + '/button2', side.button2, 10)
            self.create_subscription(PoseStamped, dev + '/measured_cp', side.haptic_cb, 10)
            self.create_subscription(Float32MultiArray, arm, side.arm_cb, 10)
            self.arm_pubs[side.name] = self.create_publisher(Float32MultiArray, xd, 10)

        self.grip_pub = self.create_publisher(Float32MultiArray, '/grippers', 10)
        self.vel_pub = self.create_publisher(
            Twist, self.get_parameter('cmd_vel_topic').value, 10)
        self.create_timer(1.0 / self.get_parameter('rate').value, self.step)

    # ------------------------------------------------------------------ arms
    def teleop_arm(self, s):
        if not (s.press2 == 1 and s.press1 == 0):
            return
        if s.haptic is None or s.tcp is None:
            self.get_logger().warn(f'{s.name}: waiting for haptic and arm poses...',
                                   throttle_duration_sec=2.0)
            return
        if s.anchor is None:
            s.tcp0 = s.tcp
            s.anchor = s.haptic
            s.tcp0_anchor = kdl_utils.anchor_transform(s.anchor, s.tcp0, VIEWER_ROBOTBASE)
            return
        command = kdl_utils.relative_command(s.anchor, s.haptic, s.tcp0, s.tcp0_anchor,
                                             self.pos_sf, self.ori_sf)
        rot = command.M.GetRot()
        msg = Float32MultiArray()
        msg.data = [command.p.x(), command.p.y(), command.p.z(), rot.x(), rot.y(), rot.z()]
        self.arm_pubs[s.name].publish(msg)

    # --------------------------------------------------------------- vehicle
    def step(self):
        L, R = self.left, self.right
        vehicle_mode = (L.press1 == 1 and R.press1 == 1 and L.press2 == 0 and R.press2 == 0)
        if vehicle_mode and L.haptic is not None and R.haptic is not None:
            if L.vehicle_anchor is None or R.vehicle_anchor is None:
                L.vehicle_anchor = L.haptic
                R.vehicle_anchor = R.haptic
            else:
                self.pilot(self.vehicle_frame_delta(L), self.vehicle_frame_delta(R))
        elif not vehicle_mode:
            self.vel_pub.publish(Twist())
            self.teleop_arm(R)
            self.teleop_arm(L)
        self.publish_grippers()

    @staticmethod
    def vehicle_frame_delta(s):
        v = (VIEWER_ROBOTBASE.Inverse().M * kdl_utils.VIEWER_HAPTIC.M
             * (s.haptic.p - s.vehicle_anchor.p))
        return [v[0], v[1], v[2]]

    def pilot(self, left, right):
        # Dominant axis of each command in the vehicle frame: 0:x, 1:y, 2:z
        l_dir = dom_proj(left)
        r_dir = dom_proj(right)
        if l_dir != r_dir:
            return
        lv, rv = left[l_dir], right[r_dir]
        thresh = 0.025 if l_dir == 0 else 0.050
        big = abs(lv) > thresh and abs(rv) > thresh
        log = self.get_logger().info
        cmd = Twist()
        if big and lv > 0 and rv > 0:
            if l_dir == 0:
                cmd.linear.x = 0.5
                log('Moving Front')
            elif l_dir == 1:
                cmd.linear.y = 0.5
                log('Moving Left')
            else:
                cmd.linear.z = 0.3
                log('Moving Up')
        elif big and lv < 0 and rv < 0:
            if l_dir == 0:
                cmd.linear.x = -0.5
                log('Moving Back')
            elif l_dir == 1:
                cmd.linear.y = -0.5
                log('Moving Right')
            else:
                cmd.linear.z = -0.3
                log('Moving Down')
        elif big and l_dir == 0 and lv > 0 and rv < 0:
            cmd.angular.z = -0.03
            log('Turning Right')
        elif big and l_dir == 0 and lv < 0 and rv > 0:
            cmd.angular.z = 0.03
            log('Turning Left')
        self.vel_pub.publish(cmd)

    def publish_grippers(self):
        msg = Float32MultiArray()
        msg.data = [float(self.left.grip), float(self.right.grip)]
        self.grip_pub.publish(msg)


def dom_proj(ctrl):
    mags = [abs(c) for c in ctrl]
    return mags.index(max(mags))


def main(args=None):
    rclpy.init(args=args)
    node = TeleopCmd()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
