#!/usr/bin/env python3
"""GuidedNode: shared MAVROS/GUIDED plumbing for SITL mission scripts.

Extracted so new scripts (frame check, circular flight) reuse the proven
GUIDED -> arm -> takeoff flow instead of copy-pasting it.

NOTE: mission_simple.py is intentionally left on its own copy for now; it
will be migrated onto this base in a SEPARATE commit, after the circle
mission has been verified in SITL -- so a refactor never rides along with
a new feature.
"""

import math
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State, SysStatus
from mavros_msgs.srv import CommandBool, CommandTOL, SetMode


def yaw_from_quat(q):
    """ENU yaw (rad, 0=East, +CCW) from a geometry_msgs quaternion."""
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def mavros_qos():
    """BEST_EFFORT QoS profile matching MAVROS state/pose publishers."""
    return QoSProfile(
        reliability=ReliabilityPolicy.BEST_EFFORT,
        durability=DurabilityPolicy.VOLATILE,
        history=HistoryPolicy.KEEP_LAST,
        depth=10,
    )


# MAV_SYS_STATUS_PREARM_CHECK, from the MAV_SYS_STATUS_SENSOR enum. Not exposed
# as a named constant by mavros_msgs, so it is spelled out here once.
PREARM_CHECK_BIT = 0x10000000


def arm_gate(prearm_ok, waited, blind_after):
    """(may_arm, using_blind) -- may we send an arm command yet?

    Pure, so the self-check can pin it. Three inputs, three answers:
      prearm_ok True  -> the vehicle says its checks pass; go.
      prearm_ok False -> it says they do not; sending anything now buys a
                         rejection and nothing else, however long we have
                         waited. The caller's own timeout is the way out.
      prearm_ok None  -> no sys_status at all. Wait `blind_after` for one, then
                         fall back to blind retry: a firmware that never
                         advertises PREARM_CHECK must still be able to fly.
    """
    if prearm_ok is None:
        return (waited >= blind_after), (waited >= blind_after)
    return bool(prearm_ok), False


class GuidedNode(Node):
    """Base node with the common GUIDED-mode helpers.

    Subclasses inherit: /mavros/state and /mavros/local_position/pose
    subscriptions (BEST_EFFORT QoS), the arming / set_mode / takeoff
    service clients, and bounded helpers for connect / mode / arm /
    takeoff / altitude. Every wait is time-bounded so nothing hangs
    forever silently.
    """

    def __init__(self, node_name):
        super().__init__(node_name)
        qos = mavros_qos()
        self.state = State()
        self.pose = PoseStamped()
        self.prearm_ok = None       # None until sys_status arrives; see arm_when_ready
        self.create_subscription(State, '/mavros/state', self._state_cb, qos)
        self.create_subscription(
            SysStatus, '/mavros/sys_status', self._sys_cb, qos)
        self.create_subscription(
            PoseStamped, '/mavros/local_position/pose', self._pose_cb, qos)
        self.arming_client = self.create_client(
            CommandBool, '/mavros/cmd/arming')
        self.set_mode_client = self.create_client(SetMode, '/mavros/set_mode')
        self.takeoff_client = self.create_client(
            CommandTOL, '/mavros/cmd/takeoff')

    # ---------- callbacks ----------
    def _state_cb(self, msg):
        self.state = msg

    def _pose_cb(self, msg):
        self.pose = msg

    def _sys_cb(self, msg):
        # MAV_SYS_STATUS_PREARM_CHECK. ArduPilot advertises it in
        # sensors_present and flips the health bit the moment every prearm
        # check passes -- which is precisely the moment an arm command stops
        # being rejected. Verified from boot: bit False at 1.1 s, four
        # 'Arm: Gyros inconsistent' rejections at 2.9-9.0 s, bit True at
        # 10.0 s, armed at 11.0 s on the very next attempt.
        if msg.sensors_present & PREARM_CHECK_BIT:
            self.prearm_ok = bool(msg.sensors_health & PREARM_CHECK_BIT)

    @property
    def altitude(self):
        return self.pose.pose.position.z

    def heading_enu(self):
        """Current nose heading in ENU (rad, 0=East, +CCW)."""
        return yaw_from_quat(self.pose.pose.orientation)

    # ---------- helpers (all bounded) ----------
    def wait_for_services(self, timeout_sec=10.0):
        """Wait for arming/set_mode/takeoff services; True if all ready."""
        clients = (self.arming_client, self.set_mode_client,
                   self.takeoff_client)
        ready = True
        for client in clients:
            ready = client.wait_for_service(timeout_sec=timeout_sec) and ready
        return ready

    def wait_for_connection(self, timeout=60.0):
        """Block until /mavros/state reports connected, or timeout."""
        start = time.time()
        while rclpy.ok() and not self.state.connected:
            if time.time() - start > timeout:
                return False
            rclpy.spin_once(self, timeout_sec=0.1)
        return self.state.connected

    def set_mode(self, mode):
        """Request a flight mode; True if the FCU accepted the request."""
        req = SetMode.Request()
        req.custom_mode = mode
        fut = self.set_mode_client.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=5.0)
        return bool(fut.result() and fut.result().mode_sent)

    def arm_once(self):
        """Single arm attempt; True on success."""
        req = CommandBool.Request()
        req.value = True
        fut = self.arming_client.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=5.0)
        return bool(fut.result() and fut.result().success)

    def arm_with_retry(self, timeout=60.0, interval=2.0, blind_after=15.0,
                       mode='GUIDED'):
        """Wait until the vehicle reports itself armable, then arm.

        Right after the link connects, EKF/GPS/home are not ready and prearm
        rejects arming. This used to hammer an arm command every `interval`
        regardless and let the rejections pile up -- which is not merely
        untidy: each rejected command is a command-ack, and a burst of them
        trips the same double-satisfied std::promise that kills mavros_node
        with `std::future_error` (see docs/TROUBLESHOOTING.md). Two runs died
        that way, with dozens of 'Arm: Accels inconsistent' immediately above
        the abort in the log. Blocking the param plugin had fixed the OTHER
        route into that bug and did nothing for this one.

        So: ask first. PREARM_CHECK goes healthy exactly when the checks pass,
        so waiting on it costs no time at all -- measured, arming succeeded
        1.0 s after the flip, having sent zero rejected commands instead of
        four.

        `blind_after` is the escape hatch: if sys_status never arrives (an
        older firmware, or the prearm bit not advertised) prearm_ok stays None
        and we fall back to the old blind retry rather than never arming.

        `mode` is the mode to arm IN, confirmed against /mavros/state rather
        than assumed. Callers already request GUIDED before getting here, but
        set_mode() returns mode_sent -- the FCU accepted the REQUEST -- which is
        not the same as being in that mode: ArduPilot refuses GUIDED until it
        has a position estimate, exactly the window this function waits through.
        Arming anyway would arm the vehicle in STABILIZE, where a setpoint
        stream does nothing and the drone answers the sticks instead.
        """
        start = time.time()
        said_wait = said_blind = said_ready = False
        mode_at = 0.0
        while rclpy.ok() and time.time() - start < timeout:
            waited = time.time() - start
            ready, blind = arm_gate(self.prearm_ok, waited, blind_after)
            if blind and not said_blind:
                self.get_logger().warn(
                    'khong co /mavros/sys_status sau {:.0f}s -- quay ve thu '
                    'arm mu'.format(blind_after))
                said_blind = True
            if ready:
                if not said_ready:
                    self.get_logger().info(
                        'ready to arm ({:.1f}s)'.format(waited))
                    said_ready = True
                # Re-request the mode only if the vehicle is not already in it,
                # and at most once per interval: a set_mode is another command
                # whose ack lands in the same promise machinery this function
                # exists to keep quiet.
                if mode and self.state.mode != mode:
                    if time.time() - mode_at >= interval:
                        mode_at = time.time()
                        self.get_logger().warn(
                            'dang o mode {} -- xin lai {} truoc khi arm'.format(
                                self.state.mode or '?', mode))
                        self.set_mode(mode)
                    rclpy.spin_once(self, timeout_sec=0.1)
                    continue
                if self.arm_once():
                    return True
                end = time.time() + interval
                while rclpy.ok() and time.time() < end:
                    rclpy.spin_once(self, timeout_sec=0.1)
            else:
                if not said_wait:
                    self.get_logger().info('cho prearm xong roi moi arm...')
                    said_wait = True
                rclpy.spin_once(self, timeout_sec=0.1)
        return False

    def takeoff(self, altitude):
        """Send a GUIDED takeoff command; True if accepted."""
        req = CommandTOL.Request()
        req.altitude = float(altitude)
        fut = self.takeoff_client.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=5.0)
        return bool(fut.result() and fut.result().success)

    def wait_for_altitude(self, target_alt, tol=0.5, timeout=60.0):
        """Block until within tol of target_alt, or timeout (bounded)."""
        start = time.time()
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)
            if abs(self.altitude - target_alt) < tol:
                return True
            if time.time() - start > timeout:
                return False
        return False


def _selfcheck():
    """python3 guided_base.py --selfcheck -- the arming gate, no ROS."""
    # the vehicle says its checks pass: go, and not blindly
    assert arm_gate(True, 0.0, 15.0) == (True, False)
    assert arm_gate(True, 99.0, 15.0) == (True, False)

    # it says they do NOT: never send, however long we have waited. This is the
    # whole point -- every rejected arm command is another chance to trip the
    # promise bug in mavros_node, and waiting costs nothing (measured: armed
    # 1.0 s after the bit flipped).
    assert arm_gate(False, 0.0, 15.0) == (False, False)
    assert arm_gate(False, 600.0, 15.0) == (False, False), \
        'cho lau khong phai ly do de bo qua prearm'

    # no sys_status at all: hold briefly for one, then fall back to blind retry
    # so a firmware that never advertises PREARM_CHECK can still fly
    assert arm_gate(None, 0.0, 15.0) == (False, False)
    assert arm_gate(None, 14.9, 15.0) == (False, False)
    assert arm_gate(None, 15.0, 15.0) == (True, True)

    # the bit itself, against the MAV_SYS_STATUS_SENSOR enum
    assert PREARM_CHECK_BIT == 1 << 28
    healthy, sick = 1467088303, 1467088303 & ~PREARM_CHECK_BIT
    assert bool(healthy & PREARM_CHECK_BIT) and not bool(sick & PREARM_CHECK_BIT)

    print('guided_base selfcheck OK (cong arm: prearm-gated, mu sau 15s)')


if __name__ == '__main__':
    import sys
    if '--selfcheck' in sys.argv:
        _selfcheck()
