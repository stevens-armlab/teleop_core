"""PyKDL helpers shared by the teleop nodes.

Replaces ``tf_conversions.posemath`` (ROS 1 only) and holds the anchored
relative-motion math that every teleop node uses.
"""

import PyKDL
from geometry_msgs.msg import Pose

# Rewrites a vector in the haptic (Touch) base frame into the viewer frame.
VIEWER_HAPTIC = PyKDL.Frame(PyKDL.Rotation(1, 0, 0, 0, 0, -1, 0, 1, 0))


def rotation_from_list(values):
    """Build a frame (pure rotation) from 9 row-major rotation-matrix values."""
    if len(values) != 9:
        raise ValueError(f'expected 9 rotation-matrix values, got {len(values)}')
    return PyKDL.Frame(PyKDL.Rotation(*[float(v) for v in values]))


def pose_to_frame(pose):
    """geometry_msgs/Pose -> PyKDL.Frame (equivalent of posemath.fromMsg)."""
    p, q = pose.position, pose.orientation
    return PyKDL.Frame(PyKDL.Rotation.Quaternion(q.x, q.y, q.z, q.w),
                       PyKDL.Vector(p.x, p.y, p.z))


def frame_to_pose(frame):
    """PyKDL.Frame -> geometry_msgs/Pose (equivalent of posemath.toMsg)."""
    pose = Pose()
    pose.position.x, pose.position.y, pose.position.z = frame.p.x(), frame.p.y(), frame.p.z()
    qx, qy, qz, qw = frame.M.GetQuaternion()
    pose.orientation.x, pose.orientation.y = qx, qy
    pose.orientation.z, pose.orientation.w = qz, qw
    return pose


def frame_from_rotvec(data):
    """[x, y, z, rx, ry, rz] (axis-angle rotation vector) -> PyKDL.Frame."""
    pos = PyKDL.Vector(data[0], data[1], data[2])
    rot = PyKDL.Vector(data[3], data[4], data[5])
    angle = rot.Norm()
    if angle < 1e-12:
        return PyKDL.Frame(PyKDL.Rotation.Identity(), pos)
    return PyKDL.Frame(PyKDL.Rotation.Rot(rot, angle), pos)


def scale(frame, position_scale, orientation_scale, lock_position=False):
    """Scale the translation and rotation angle of a relative motion."""
    axis = frame.M.GetRot()
    angle = axis.Normalize()
    rot = PyKDL.Rotation.Rot(axis, angle * orientation_scale)
    if lock_position:
        return PyKDL.Frame(rot)
    return PyKDL.Frame(rot, frame.p * position_scale)


def anchor_transform(haptic_anchor, tcp0, viewer_robotbase, viewer_haptic=VIEWER_HAPTIC):
    """Rotation that maps a motion in the haptic anchor frame into the TCP0 frame.

    Computed once at the anchoring event (when the clutch button is pressed).
    """
    viewer_anchor = viewer_haptic * PyKDL.Frame(haptic_anchor.M)
    robotbase_tcp0 = PyKDL.Frame(tcp0.M)
    return robotbase_tcp0.Inverse() * viewer_robotbase.Inverse() * viewer_anchor


def relative_command(haptic_anchor, haptic_now, tcp0, tcp0_anchor,
                     position_scale=1.0, orientation_scale=1.0, lock_position=False):
    """Commanded robot pose = TCP0 composed with the scaled haptic motion since anchoring."""
    rel = haptic_anchor.Inverse() * haptic_now
    if lock_position or position_scale != 1.0 or orientation_scale != 1.0:
        rel = scale(rel, position_scale, orientation_scale, lock_position)
    return tcp0 * (tcp0_anchor * rel * tcp0_anchor.Inverse())
