#!/usr/bin/env python3
"""hold_and_watch -- take off, hold still facing the first gate, and MEASURE.

A verification rig for the perception + planner chain (N1..N4), not a mission.

Why it exists: mission_gates yaw-sweeps at 0.4 rad/s looking for a ring. Measured
live, that put the gate inside the frame for 1.3% of frames -- gate_detect saw it
183 times out of 13850 -- so gate_tracker only occasionally collected the 5
consistent detections it needs to CONFIRM, and the planner spent most of its life
waiting on an empty /gate/map. None of those nodes were broken; they were starved.

The course makes the fix trivial: gate_1 sits at (5, 0, 3.5) and the drone spawns
at the origin already facing +X. So there is nothing to search for. Take off to the
gate's own altitude, hold position, lock yaw to 0 (East, straight at the ring), and
the gate is in frame 100% of the time. Detection rate, tracker convergence and map
quality all stop being a function of luck, and what is left to measure is the thing
we actually want to know: how well N1/N2 locate a ring from a known standoff.

It prints, every REPORT_S seconds:
  - detections/s out of N1, so N1's TRUE hit rate is visible when nothing moves
  - the CONFIRMED gate centre from N2, in map frame
  - the drone->gate distance, and the error against the surveyed position

TRUTH is used ONLY for that error line -- never fed to any algorithm. The plan is
explicit that nothing may read gate poses from the simulator; this is the report
card, not an input.

--probe-accel answers the one question N5 is blocked on: does THIS ArduCopter
honour the acceleration field of SET_POSITION_TARGET_LOCAL_NED? See _probe_accel.

Run:  ros2 run simtofly_mavros_sitl hold_and_watch
      ros2 run simtofly_mavros_sitl hold_and_watch --probe-accel
      python3 hold_and_watch.py --selfcheck
"""
import math
import sys
import time

import numpy as np
import rclpy
from geometry_msgs.msg import PoseArray, PoseStamped, TwistStamped
from mavros_msgs.msg import PositionTarget

try:
    from simtofly_mavros_sitl.guided_base import GuidedNode, mavros_qos
except ImportError:                       # running the file directly
    from guided_base import GuidedNode, mavros_qos

HOLD_ALT = 3.5             # m; gate_1's own height, so the ring sits mid-frame
HOLD_YAW = 0.0             # rad ENU; 0 = East = +X = straight at gate_1
SETPOINT_HZ = 20.0         # ArduPilot GUIDED needs a CONTINUOUS stream, >= 10 Hz
REPORT_S = 3.0             # how often the measurement line is printed
# Surveyed position of gate_1 from drone_vision/worlds/gates_course.sdf. REPORTING
# ONLY -- see the module docstring.
TRUTH = np.array([5.0, 0.0, 3.5])

# --- accel probe (plan v2 R11 / stage 2) ------------------------------------
_IGN_ACC = (PositionTarget.IGNORE_AFX | PositionTarget.IGNORE_AFY
            | PositionTarget.IGNORE_AFZ)
# Pos+Vel+Accel+Yaw -- the mask the plan wants for N5. Ignores yaw_rate only.
MASK_PVA = PositionTarget.IGNORE_YAW_RATE
# Pos+Vel+Yaw -- identical EXCEPT the accel field is ignored. The control arm.
#
# The plan named 3520 as the fallback, but 3520 also sets IGNORE_YAW, so a 2048
# vs 3520 A/B changes yaw handling as well as accel and cannot attribute the
# difference to either. This pair differs in the accel bits and nothing else.
MASK_PV = MASK_PVA | _IGN_ACC
A_TEST = 2.0               # m/s^2 feed-forward. Deliberately UNDER WPNAV_ACCEL's
                           # 2.5 m/s^2 default: ask for more and the shaper clamps
                           # it, and a clamped null result is indistinguishable
                           # from an ignored one.
BURST_S = 2.0              # s streaming each arm
SETTLE_S = 3.0             # s back on plain position hold between arms
ROUNDS = 3
VERDICT_M = 0.10           # m; B must beat A by this much to call accel honoured


def yaw_to_quat(yaw):
    """ENU yaw -> (z, w) of a yaw-only quaternion (x = y = 0)."""
    return math.sin(yaw / 2.0), math.cos(yaw / 2.0)


class HoldAndWatch(GuidedNode):
    def __init__(self):
        super().__init__('hold_and_watch')
        self.sp_pub = self.create_publisher(
            PoseStamped, '/mavros/setpoint_position/local', 10)
        self.raw_pub = self.create_publisher(
            PositionTarget, '/mavros/setpoint_raw/local', 10)
        self.create_subscription(PoseArray, '/gate/detections', self._on_det, 10)
        self.create_subscription(PoseArray, '/gate/map', self._on_map, 10)
        self.create_subscription(
            TwistStamped, '/mavros/local_position/velocity_local',
            self._on_vel, mavros_qos())
        self.hold = None           # the point we are holding, set after takeoff
        self.n_det = 0             # detections since the last report
        self.gate = None           # newest CONFIRMED gate centre (map)
        self.vel = np.zeros(3)     # ENU, from MAVROS
        self.t_report = time.time()

    def _on_det(self, msg):
        self.n_det += len(msg.poses)

    def _on_vel(self, msg):
        v = msg.twist.linear
        self.vel = np.array([v.x, v.y, v.z])

    def _on_map(self, msg):
        if msg.poses:
            p = msg.poses[0].position
            self.gate = np.array([p.x, p.y, p.z])

    def setpoint(self):
        """The held pose, as a setpoint message."""
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        msg.pose.position.x, msg.pose.position.y, msg.pose.position.z = (
            float(v) for v in self.hold)
        msg.pose.orientation.z, msg.pose.orientation.w = yaw_to_quat(HOLD_YAW)
        return msg

    def report(self):
        dt = time.time() - self.t_report
        rate = self.n_det / dt if dt else 0.0
        self.n_det, self.t_report = 0, time.time()
        p = self.pose.pose.position
        drone = np.array([p.x, p.y, p.z])
        if self.gate is None:
            self.get_logger().info(
                'detect {:.1f}/s | CHUA co gate CONFIRMED | drone={}'.format(
                    rate, np.round(drone, 2)))
            return
        dist = float(np.linalg.norm(self.gate - drone))
        err = float(np.linalg.norm(self.gate - TRUTH))
        self.get_logger().info(
            'detect {:.1f}/s | gate={} | cach {:.2f} m | sai so vs truth {:.3f} m'
            .format(rate, np.round(self.gate, 3), dist, err))

    def _xyz(self):
        p = self.pose.pose.position
        return np.array([p.x, p.y, p.z])

    def raw_setpoint(self, mask, accel):
        """Hold `self.hold` at v=0 via setpoint_raw, with `accel` fed forward.

        ENU throughout -- MAVROS converts to NED behind FRAME_LOCAL_NED.
        """
        msg = PositionTarget()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        msg.coordinate_frame = PositionTarget.FRAME_LOCAL_NED
        msg.type_mask = mask
        msg.position.x, msg.position.y, msg.position.z = (
            float(v) for v in self.hold)
        msg.acceleration_or_force.x = float(accel[0])
        msg.acceleration_or_force.y = float(accel[1])
        msg.acceleration_or_force.z = float(accel[2])
        msg.yaw = float(HOLD_YAW)
        return msg          # velocity stays zero: hold still, ask accel to move it

    def _stream(self, mask, accel, secs):
        """Stream one mask for `secs`; return (peak excursion m, peak speed m/s)."""
        peak_d = peak_v = 0.0
        end = time.time() + secs
        period = 1.0 / SETPOINT_HZ
        while rclpy.ok() and time.time() < end:
            self.raw_pub.publish(self.raw_setpoint(mask, accel))
            tick = time.time() + period
            while rclpy.ok() and time.time() < tick:
                rclpy.spin_once(self, timeout_sec=period / 4.0)
            peak_d = max(peak_d, float(np.linalg.norm(self._xyz() - self.hold)))
            peak_v = max(peak_v, float(np.linalg.norm(self.vel)))
        return peak_d, peak_v

    def _probe_accel(self):
        """Decide whether ArduCopter honours the accel field (plan v2 R11).

        Both arms command the SAME hold position at v=0 and differ only in
        whether the accel bits are ignored. Arm B feeds forward A_TEST on +X.
        Position hold fights the feed-forward back, which is what keeps this
        bounded -- but if accel is honoured at all it shows up as a clear
        excursion; if it is dropped, B is indistinguishable from A.

        A null result is a RESULT: it selects the pos+vel fallback for N5.
        """
        self.get_logger().info(
            'PROBE accel: A=pos+vel (mask {}) vs B=pos+vel+accel (mask {}), '
            'a={} m/s2, {} vong'.format(MASK_PV, MASK_PVA, A_TEST, ROUNDS))
        gaps = []
        for i in range(ROUNDS):
            self._stream(MASK_PV, (0.0, 0.0, 0.0), SETTLE_S)
            a_d, a_v = self._stream(MASK_PV, (A_TEST, 0.0, 0.0), BURST_S)
            self._stream(MASK_PV, (0.0, 0.0, 0.0), SETTLE_S)
            b_d, b_v = self._stream(MASK_PVA, (A_TEST, 0.0, 0.0), BURST_S)
            gaps.append(b_d - a_d)
            self.get_logger().info(
                'vong {}: A lech {:.3f} m (v {:.2f}) | B lech {:.3f} m (v {:.2f})'
                ' | chenh {:+.3f} m'.format(i + 1, a_d, a_v, b_d, b_v, b_d - a_d))
        self._stream(MASK_PV, (0.0, 0.0, 0.0), SETTLE_S)

        gap = float(np.median(gaps))
        if gap > VERDICT_M:
            self.get_logger().info(
                'KET LUAN: ArduCopter CO nhan accel (chenh trung vi {:+.3f} m > '
                '{:.2f}). N5 dung mask {} (pos+vel+accel+yaw).'
                .format(gap, VERDICT_M, MASK_PVA))
        else:
            self.get_logger().warn(
                'KET LUAN: KHONG thay accel co tac dung (chenh trung vi {:+.3f} m '
                '<= {:.2f}). N5 phai dung fallback pos+vel, bo truong accel.'
                .format(gap, VERDICT_M))

    def run(self, probe=False):
        if not self.wait_for_services():
            self.get_logger().error('MAVROS services khong len'); return
        if not self.wait_for_connection():
            self.get_logger().error('FCU khong connect'); return
        self.get_logger().info('FCU connected -> GUIDED')
        if not self.set_mode('GUIDED'):
            self.get_logger().error('set GUIDED that bai'); return
        if not self.arm_with_retry():
            self.get_logger().error('arm that bai (prearm/EKF/GPS?)'); return
        self.get_logger().info('armed -> takeoff {:.1f} m'.format(HOLD_ALT))
        if not self.takeoff(HOLD_ALT):
            self.get_logger().error('takeoff bi tu choi'); return
        if not self.wait_for_altitude(HOLD_ALT, tol=0.3):
            self.get_logger().warn('khong dat do cao, van giu vi tri hien tai')

        p = self.pose.pose.position
        # Hold where we ended up, but at exactly the gate's altitude, so the ring
        # is centred vertically rather than wherever takeoff happened to stop.
        self.hold = np.array([p.x, p.y, HOLD_ALT])
        self.get_logger().info(
            'giu vi tri {} yaw=0 (nhin thang vao gate_1) -- Ctrl+C de dung'
            .format(np.round(self.hold, 2)))

        if probe:
            self._probe_accel()
            return

        period = 1.0 / SETPOINT_HZ
        while rclpy.ok():
            self.sp_pub.publish(self.setpoint())
            if time.time() - self.t_report >= REPORT_S:
                self.report()
            end = time.time() + period
            while rclpy.ok() and time.time() < end:
                rclpy.spin_once(self, timeout_sec=period / 4.0)


def _selfcheck():
    # yaw 0 must be the identity rotation: nose along +X (East), which is what
    # points the camera at gate_1. A sign slip here aims the drone 180 deg away.
    z, w = yaw_to_quat(0.0)
    assert (z, w) == (0.0, 1.0), (z, w)
    # and the round trip agrees with guided_base's decoder for a few angles
    try:
        from simtofly_mavros_sitl.guided_base import yaw_from_quat
    except ImportError:
        from guided_base import yaw_from_quat

    class Q:
        pass

    for yaw in (0.0, 0.5, -1.2, math.pi / 2):
        q = Q()
        q.x = q.y = 0.0
        q.z, q.w = yaw_to_quat(yaw)
        assert abs(yaw_from_quat(q) - yaw) < 1e-9, yaw

    # the surveyed gate really is straight ahead of the origin at hold altitude,
    # i.e. yaw 0 is the right thing to lock -- if the course moves, this fails
    assert abs(math.atan2(TRUTH[1], TRUTH[0]) - HOLD_YAW) < 1e-6, \
        'gate_1 khong con nam thang huong +X, HOLD_YAW phai doi'
    assert abs(TRUTH[2] - HOLD_ALT) < 1e-6, 'HOLD_ALT phai bang do cao gate_1'
    # --- accel probe masks. A sign/bit slip here silently tests nothing: both
    # arms would command the same thing and the probe would always report "no
    # effect", i.e. it would fake the very null result it exists to detect.
    P = PositionTarget
    pos_vel = (P.IGNORE_PX | P.IGNORE_PY | P.IGNORE_PZ
               | P.IGNORE_VX | P.IGNORE_VY | P.IGNORE_VZ)
    # neither arm may ignore position, velocity or yaw -- only accel may differ
    for name, mask in (('MASK_PVA', MASK_PVA), ('MASK_PV', MASK_PV)):
        assert not (mask & pos_vel), '{} bo qua pos/vel'.format(name)
        assert not (mask & P.IGNORE_YAW), '{} bo qua yaw'.format(name)
        assert mask & P.IGNORE_YAW_RATE, '{} phai bo qua yaw_rate'.format(name)
    assert not (MASK_PVA & _IGN_ACC), 'MASK_PVA phai DUNG accel'
    assert MASK_PV & _IGN_ACC == _IGN_ACC, 'MASK_PV phai bo qua ca 3 truc accel'
    # ...and differ in the accel bits and NOTHING else, or the A/B proves nothing
    assert MASK_PVA ^ MASK_PV == _IGN_ACC, (MASK_PVA, MASK_PV)
    assert MASK_PVA == 2048, MASK_PVA          # the mask plan v2 par. 5.1 names
    # a test accel above WPNAV_ACCEL would be clamped by the shaper, and a
    # clamped null is indistinguishable from an ignored one (plan v2 par. 5.3)
    assert A_TEST < 2.5, 'A_TEST phai duoi WPNAV_ACCEL mac dinh 2.5 m/s2'
    print('hold_and_watch self-check OK (yaw=0 -> +X, gate_1 {}; '
          'probe masks PVA={} PV={})'.format(TRUTH, MASK_PVA, MASK_PV))


def main():
    if '--selfcheck' in sys.argv:
        _selfcheck()
        return
    rclpy.init()
    node = HoldAndWatch()
    try:
        node.run(probe='--probe-accel' in sys.argv)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
