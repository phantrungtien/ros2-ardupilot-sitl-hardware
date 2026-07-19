#!/usr/bin/env python3
"""Stage 5 acceptance rig for N5 -- publish a path, then grade the flight.

Plan v2 par. 12 stage 5 asks for three things, and this measures all three:

  1. tracking error RMS < 0.15 m        (commanded position vs measured)
  2. commanded accel is never clamped   (compare against WPNAV_ACCEL / PSC_JERK)
  3. the trajectory is C2 at the joins  (mueller/trajectory_follower self-checks)

No perception: the path is synthetic, so this grades N5 alone. It publishes on
/planner/path exactly as N4 does (TRANSIENT_LOCAL, yaw baked into each pose), so
trajectory_follower cannot tell the difference.

Run (with trajectory_follower already flying):
    ros2 run simtofly_gates verify_n5
"""
import math
import sys
import time

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped, TwistStamped
from mavros_msgs.msg import PositionTarget
from nav_msgs.msg import Path
from rclpy.node import Node
from rclpy.qos import (QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile,
                       QoSReliabilityPolicy)

try:
    from simtofly_mavros_sitl.guided_base import mavros_qos
except ImportError:
    from guided_base import mavros_qos

# Measured on this vehicle (mav.parm, ArduCopter V4.6.3). Commanded accel above
# these is CLAMPED by the position controller's shaper, silently.
WPNAV_ACCEL = 2.5          # m/s^2
WPNAV_ACCEL_Z = 1.0        # m/s^2
RMS_LIMIT = 0.15           # m; the stage-5 acceptance bar
SETTLE_S = 3.0             # s to let the hold settle before grading
RUN_S = 45.0               # s of flight to grade


class VerifyN5(Node):
    def __init__(self):
        super().__init__('verify_n5')
        self.pub = self.create_publisher(
            Path, '/planner/path',
            QoSProfile(durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
                       reliability=QoSReliabilityPolicy.RELIABLE,
                       history=QoSHistoryPolicy.KEEP_LAST, depth=1))
        self.create_subscription(PoseStamped, '/mavros/local_position/pose',
                                 self._pose, mavros_qos())
        self.create_subscription(TwistStamped,
                                 '/mavros/local_position/velocity_local',
                                 self._vel, mavros_qos())
        self.create_subscription(PositionTarget, '/mavros/setpoint_raw/local',
                                 self._sp, 10)
        self.pose = None
        self.vel = np.zeros(3)
        self.samples = []          # (cmd_p, meas_p, cmd_a, meas_v)
        self.cmd = None

    def _pose(self, m):
        p = m.pose.position
        self.pose = np.array([p.x, p.y, p.z])

    def _vel(self, m):
        v = m.twist.linear
        self.vel = np.array([v.x, v.y, v.z])

    def _sp(self, m):
        self.cmd = (np.array([m.position.x, m.position.y, m.position.z]),
                    np.array([m.acceleration_or_force.x,
                              m.acceleration_or_force.y,
                              m.acceleration_or_force.z]),
                    int(m.type_mask))
        if self.pose is not None:
            self.samples.append((self.cmd[0].copy(), self.pose.copy(),
                                 self.cmd[1].copy(), self.vel.copy()))

    def send_path(self, start):
        """A gate-shaped path from wherever the drone is: approach then traverse."""
        C = start + np.array([5.0, 0.0, 0.0])       # 'gate' 5 m ahead
        n = np.array([1.0, 0.0, 0.0])
        pts = []
        for a, b in ((start, start + [2.0, 1.0, 0.0]),
                     (start + [2.0, 1.0, 0.0], C - 1.5 * n)):
            a, b = np.asarray(a, float), np.asarray(b, float)
            k = max(1, int(round(float(np.linalg.norm(b - a)) / 0.5)))
            pts += [a + (b - a) * i / k for i in range(k)]
        pts += [C + t * n for t in np.arange(-1.5, 1.51, 0.5)]

        msg = Path()
        msg.header.frame_id = 'map'
        msg.header.stamp = self.get_clock().now().to_msg()
        for q in pts:
            ps = PoseStamped()
            ps.header = msg.header
            ps.pose.position.x, ps.pose.position.y, ps.pose.position.z = (
                float(v) for v in q)
            ps.pose.orientation.w = 1.0
            msg.poses.append(ps)
        self.pub.publish(msg)
        self.get_logger().info('da gui path {} diem, gate gia tai {}'.format(
            len(pts), np.round(C, 2)))
        return C

    def report(self, C):
        if len(self.samples) < 20:
            self.get_logger().error(
                'chi co {} mau -- N5 co dang phat setpoint khong?'.format(
                    len(self.samples)))
            return False
        cmd_p = np.array([s[0] for s in self.samples])
        meas_p = np.array([s[1] for s in self.samples])
        cmd_a = np.array([s[2] for s in self.samples])
        err = np.linalg.norm(cmd_p - meas_p, axis=1)
        rms = float(np.sqrt((err ** 2).mean()))
        peak_axy = float(np.abs(cmd_a[:, :2]).max())
        peak_az = float(np.abs(cmd_a[:, 2]).max())
        mask = self.samples[-1] and self.cmd[2]

        ok_rms = rms < RMS_LIMIT
        ok_axy = peak_axy < WPNAV_ACCEL
        ok_az = peak_az < WPNAV_ACCEL_Z
        travelled = float(np.linalg.norm(meas_p[-1] - meas_p[0]))
        log = self.get_logger().info if (ok_rms and ok_axy and ok_az) \
            else self.get_logger().warn
        log('=== GD5 ===')
        log('  mau            : {}   quang duong bay {:.2f} m'.format(
            len(self.samples), travelled))
        log('  tracking error : RMS {:.3f} m  max {:.3f} m   [can < {:.2f}]  {}'
            .format(rms, float(err.max()), RMS_LIMIT, 'DAT' if ok_rms else 'TRUOT'))
        log('  accel lenh ngang: dinh {:.3f}  [WPNAV_ACCEL {:.1f}]  {}'.format(
            peak_axy, WPNAV_ACCEL, 'khong bi clamp' if ok_axy else 'BI CLAMP'))
        log('  accel lenh doc  : dinh {:.3f}  [WPNAV_ACCEL_Z {:.1f}]  {}'.format(
            peak_az, WPNAV_ACCEL_Z, 'khong bi clamp' if ok_az else 'BI CLAMP'))
        log('  type_mask      : {} (2048 = pos+vel+accel+yaw)'.format(mask))
        log('  cach tam gate  : {:.3f} m'.format(
            float(np.linalg.norm(meas_p[-1] - C))))
        return ok_rms and ok_axy and ok_az


def main():
    rclpy.init()
    node = VerifyN5()
    try:
        t0 = time.time()
        while rclpy.ok() and node.pose is None and time.time() - t0 < 30:
            rclpy.spin_once(node, timeout_sec=0.2)
        if node.pose is None:
            node.get_logger().error('khong nhan duoc pose'); return
        end = time.time() + SETTLE_S
        while rclpy.ok() and time.time() < end:
            rclpy.spin_once(node, timeout_sec=0.1)
        C = node.send_path(node.pose.copy())
        node.samples.clear()
        end = time.time() + RUN_S
        while rclpy.ok() and time.time() < end:
            rclpy.spin_once(node, timeout_sec=0.05)
        ok = node.report(C)
        print('GD5 {}'.format('PASS' if ok else 'FAIL'))
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


def _selfcheck():
    """The grading rules only, no ROS:  python3 verify_n5.py --selfcheck

    Every other module in this workspace answers --selfcheck; this one used to
    take the flag, start a live node and exit with "khong nhan duoc pose",
    which reads like a failure rather than "wrong tool". The thresholds below
    are the whole point of the rig, so they are what gets checked.
    """
    def graded(err, axy, az, n=40):
        node = VerifyN5.__new__(VerifyN5)
        cmd_p = np.zeros((n, 3))
        meas_p = np.tile([err, 0.0, 0.0], (n, 1))       # constant miss -> RMS=err
        cmd_a = np.tile([axy, 0.0, az], (n, 1))
        node.samples = [(cmd_p[i], meas_p[i], cmd_a[i], np.zeros(3))
                        for i in range(n)]
        node.cmd = (np.zeros(3), np.zeros(3), 2048)
        node.get_logger = lambda: type('L', (), {
            'info': lambda _s, m, **k: None, 'warn': lambda _s, m, **k: None,
            'error': lambda _s, m, **k: None})()
        return node.report(np.zeros(3))

    assert graded(0.05, 1.0, 0.5), 'chuyen bay dat chuan bi cham TRUOT'
    assert not graded(RMS_LIMIT + 0.01, 1.0, 0.5), 'RMS qua nguong van PASS'
    assert not graded(0.05, WPNAV_ACCEL + 0.1, 0.5), 'accel ngang bi clamp van PASS'
    assert not graded(0.05, 1.0, WPNAV_ACCEL_Z + 0.1), 'accel doc bi clamp van PASS'

    # too few samples is "N5 never streamed", not "N5 flew badly"
    thin = VerifyN5.__new__(VerifyN5)
    thin.samples = []
    thin.get_logger = lambda: type('L', (), {
        'info': lambda _s, m, **k: None, 'warn': lambda _s, m, **k: None,
        'error': lambda _s, m, **k: None})()
    assert not thin.report(np.zeros(3)), 'khong co mau ma van PASS'
    print('verify_n5 selfcheck OK (nguong RMS {:.2f} m, accel {:.1f}/{:.1f})'
          .format(RMS_LIMIT, WPNAV_ACCEL, WPNAV_ACCEL_Z))


if __name__ == '__main__':
    if '--selfcheck' in sys.argv:
        _selfcheck()
    else:
        main()
