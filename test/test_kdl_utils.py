import math

import PyKDL

from teleop_core import kdl_utils


def _frame(x, y, z, r, p, yaw):
    return PyKDL.Frame(PyKDL.Rotation.RPY(r, p, yaw), PyKDL.Vector(x, y, z))


def _close(a, b, tol=1e-9):
    return PyKDL.Equal(a, b, tol)


def test_pose_round_trip():
    f = _frame(0.1, -0.2, 0.3, 0.4, -0.5, 0.6)
    assert _close(kdl_utils.pose_to_frame(kdl_utils.frame_to_pose(f)), f)


def test_rotvec_round_trip():
    f = _frame(1.0, 2.0, 3.0, 0.1, 0.2, 0.3)
    r = f.M.GetRot()
    g = kdl_utils.frame_from_rotvec([f.p.x(), f.p.y(), f.p.z(), r.x(), r.y(), r.z()])
    assert _close(f, g)
    assert _close(kdl_utils.frame_from_rotvec([0, 0, 0, 0, 0, 0]), PyKDL.Frame())


def test_no_haptic_motion_commands_tcp0():
    anchor = _frame(0.05, 0.0, -0.02, 0.3, 0.1, -0.2)
    tcp0 = _frame(0.5, 0.1, 0.4, math.pi, 0.0, 0.2)
    viewer_rb = kdl_utils.rotation_from_list([0, -1, 0, 1, 0, 0, 0, 0, 1])
    t0a = kdl_utils.anchor_transform(anchor, tcp0, viewer_rb)
    cmd = kdl_utils.relative_command(anchor, anchor, tcp0, t0a, 5.0, 1.0)
    assert _close(cmd, tcp0)


def test_position_scaling_and_lock():
    rel = _frame(0.01, 0.02, -0.03, 0.1, 0.0, 0.0)
    scaled = kdl_utils.scale(rel, 2.0, 1.0)
    assert _close(scaled.p, rel.p * 2.0)
    assert _close(scaled.M, rel.M)
    locked = kdl_utils.scale(rel, 2.0, 1.0, lock_position=True)
    assert _close(locked.p, PyKDL.Vector())


def test_translation_maps_through_viewer_frames():
    # Haptic anchored at identity, robot TCP0 at identity, viewer->robotbase identity.
    # A +y move in the haptic frame is +z in the viewer frame (VIEWER_HAPTIC).
    anchor = PyKDL.Frame()
    tcp0 = PyKDL.Frame()
    t0a = kdl_utils.anchor_transform(anchor, tcp0, PyKDL.Frame())
    now = PyKDL.Frame(PyKDL.Vector(0.0, 0.01, 0.0))
    cmd = kdl_utils.relative_command(anchor, now, tcp0, t0a)
    assert _close(cmd.p, PyKDL.Vector(0.0, 0.0, 0.01))
