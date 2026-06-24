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
from mavros_msgs.msg import State
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
        self.create_subscription(State, '/mavros/state', self._state_cb, qos)
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

    def arm_with_retry(self, timeout=60.0, interval=2.0):
        """Retry arming until armable or timeout.

        Right after the MAVLink link connects, EKF/GPS/home are usually not
        ready and prearm checks reject arming. Retrying waits exactly as
        long as needed and arms the moment the vehicle becomes armable.
        """
        start = time.time()
        while rclpy.ok() and time.time() - start < timeout:
            if self.arm_once():
                return True
            end = time.time() + interval
            while rclpy.ok() and time.time() < end:
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
