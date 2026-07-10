#!/usr/bin/env python3
"""Fly the drone through the racing gates.

Streams ENU position setpoints on /mavros/setpoint_position/local toward
the current gate centre and advances to the next gate as soon as the drone
is within CAPTURE_RADIUS of it. Switching targets *before* arriving means
the drone cuts smoothly through each ring instead of stopping at it.

The gate list (GATES below) must match the gate <include> poses in
worlds/gates_course.sdf, so the rings the drone aims at are the rings you
see. The nose is pointed along each leg so it flies squarely through the
hole.

Frame is ENU (matches /mavros/local_position/pose): x=East, y=North,
z=Up, origin at take-off. The iris spawns in Gazebo at (0,0) with yaw
90 deg (nose +North), matching the MAVROS spawn heading ~90 deg, so the
Gazebo world frame is ENU too and a ring at Gazebo (x,y,z) is at the same
ENU (x,y,z) commanded here. (Verify visually on the first SITL run.)

Flow: wait FCU -> GUIDED -> arm (retry) -> takeoff -> stream through the
gates -> RTL. Leaving GUIDED (pilot takeover) or Ctrl+C -> RTL.

Run:
    ros2 launch simtofly_mavros_sitl sitl_mission.launch.py \\
        world:=gates_course.sdf mission:=mission_gates
"""

import math
import time

import rclpy
from geometry_msgs.msg import PoseStamped

from simtofly_mavros_sitl.guided_base import GuidedNode

# ===== the course: gate centres in ENU (East, North, Up) metres =====
# MUST match the gate <include> poses in worlds/gates_course.sdf. The drone
# takes off at START and flies through each gate in order. A demanding
# course to show the controller off: gates 14 m apart, a wide +/-9 m
# slalom with sharp heading reversals, and altitude weaving across many
# levels (2.0..6.5 m) so it must climb and descend between rings.
START = (0.0, 0.0)
GATES = [
    (12.0,   9.0, 2.5),
    (26.0,  -9.0, 4.0),
    (40.0,   9.0, 5.5),
    (54.0,  -9.0, 6.5),
    (68.0,   9.0, 5.0),
    (82.0,  -9.0, 3.5),
    (96.0,   9.0, 2.0),
    (110.0, -9.0, 3.5),
    (124.0,  9.0, 5.0),
    (138.0, -9.0, 6.0),
    (152.0,  6.0, 4.0),
    (166.0,  0.0, 2.5),
]

# ring opening radius (m); a pass whose closest approach to the centre is
# within this radius went through the hole. Must match gate_ring model.sdf.
RING_RADIUS = 1.5

CTRL_HZ = 20.0            # setpoint stream rate
CAPTURE_RADIUS = 1.0      # advance to next gate within this distance (m)
TAKEOFF_ALT = 3.0         # climb here before heading to gate 1 (m)
MAX_ALT = 30.0            # altitude ceiling (m) -> hover if exceeded


def yaw_to_quat(yaw):
    """ENU yaw (rad) -> geometry_msgs quaternion (about +Z)."""
    return (0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))


class GatesMission(GuidedNode):
    def __init__(self):
        super().__init__('gates_mission')
        self.sp_pub = self.create_publisher(
            PoseStamped, '/mavros/setpoint_position/local', 10)
        self.idx = 0                 # index of the gate we are heading to
        self.prev_xy = START         # start of the current leg (for yaw)
        self.active = False
        self.done = False
        self.reported = False
        # closest approach (m) to each gate centre over the whole flight --
        # the accuracy metric proving how precisely the controller flew.
        self.min_dist = [float('inf')] * len(GATES)
        self._warned_mode = False
        self._tick = 0
        self.timer = self.create_timer(1.0 / CTRL_HZ, self._control_loop)

    def _leg_yaw(self, gate):
        """Nose heading along the current leg (prev point -> this gate)."""
        return math.atan2(gate[1] - self.prev_xy[1],
                          gate[0] - self.prev_xy[0])

    def _publish(self, x, y, z, yaw):
        sp = PoseStamped()
        sp.header.stamp = self.get_clock().now().to_msg()
        sp.header.frame_id = 'map'
        sp.pose.position.x = float(x)   # +East
        sp.pose.position.y = float(y)   # +North
        sp.pose.position.z = float(z)   # +Up
        qx, qy, qz, qw = yaw_to_quat(yaw)
        sp.pose.orientation.x = qx
        sp.pose.orientation.y = qy
        sp.pose.orientation.z = qz
        sp.pose.orientation.w = qw
        self.sp_pub.publish(sp)

    def _control_loop(self):
        if not self.active or self.done:
            return

        # Pilot took over (left GUIDED) -> release, stop streaming.
        if self.state.mode != 'GUIDED':
            if not self._warned_mode:
                self.get_logger().warn(
                    'mode={} != GUIDED -> release control'
                    .format(self.state.mode))
                self._warned_mode = True
            return
        self._warned_mode = False

        gate = GATES[self.idx]
        self._publish(gate[0], gate[1], gate[2], self._leg_yaw(gate))

        px = self.pose.pose.position.x
        py = self.pose.pose.position.y
        pz = self.pose.pose.position.z
        # Track the closest approach to EVERY gate centre (not just the
        # active one) so the recorded accuracy is the true minimum distance,
        # independent of when we advance the target.
        for gi, gt in enumerate(GATES):
            d = math.sqrt((gt[0] - px) ** 2 + (gt[1] - py) ** 2
                          + (gt[2] - pz) ** 2)
            if d < self.min_dist[gi]:
                self.min_dist[gi] = d
            if gi == self.idx:
                dist = d

        self._tick += 1
        if self._tick % int(CTRL_HZ) == 0:   # ~1 Hz progress line
            print('  gate {}/{} | dist {:.1f}m | pos ({:.1f}, {:.1f}, {:.1f})'
                  .format(self.idx + 1, len(GATES), dist, px, py, pz))

        if dist < CAPTURE_RADIUS:
            print('  passed gate {}/{} (closest {:.2f}m)'
                  .format(self.idx + 1, len(GATES), self.min_dist[self.idx]))
            self.prev_xy = (gate[0], gate[1])
            self.idx += 1
            if self.idx >= len(GATES):
                self.done = True
                self.active = False
                print('\n✓ all gates cleared -> RTL')
                self._report()
                self.set_mode('RTL')

    def _report(self):
        """Print how precisely the drone flew through each gate centre --
        the closest-approach distance is the controller-accuracy proof."""
        if self.reported:
            return
        self.reported = True
        print('\n' + '=' * 56)
        print('CONTROL ACCURACY -- closest approach to each gate centre')
        print('=' * 56)
        cleared, seen = 0, []
        for i, d in enumerate(self.min_dist, start=1):
            if math.isinf(d):
                print('  gate {:2d}:   n/a  (not reached)'.format(i))
                continue
            seen.append(d)
            through = d < RING_RADIUS
            cleared += through
            print('  gate {:2d}: {:5.2f} m  {}'.format(
                i, d, 'through hole' if through else 'MISSED'))
        if seen:
            print('-' * 56)
            print('  {}/{} gates through the {:.1f} m opening | '
                  'mean {:.2f} m, worst {:.2f} m'
                  .format(cleared, len(self.min_dist), RING_RADIUS,
                          sum(seen) / len(seen), max(seen)))
        print('=' * 56)

    def startup(self):
        print('=' * 56)
        print('GATES: {} rings, capture {:.1f}m, takeoff {:.0f}m'
              .format(len(GATES), CAPTURE_RADIUS, TAKEOFF_ALT))
        print('=' * 56)

        if not self.wait_for_services():
            print('x services not available')
            return False
        print('Waiting for FCU connection...')
        if not self.wait_for_connection():
            print('x FCU not connected')
            return False
        print('✓ FCU connected')

        if not self.set_mode('GUIDED'):
            print('x could not set GUIDED')
            return False
        time.sleep(1)
        print('Arming (retry until prearm/EKF/GPS ready)...')
        if not self.arm_with_retry():
            print('x could not arm')
            return False
        print('✓ armed')
        time.sleep(2)
        if not self.takeoff(TAKEOFF_ALT):
            print('x takeoff rejected')
            return False
        if not self.wait_for_altitude(TAKEOFF_ALT):
            print('x did not reach takeoff altitude')
            return False
        print('✓ at {:.0f}m'.format(TAKEOFF_ALT))
        time.sleep(2)

        self.idx = 0
        self.prev_xy = START
        self.active = True
        print('✓ flying the gates (Ctrl+C -> RTL)\n')
        return True

    def safe_shutdown(self):
        try:
            # Show the accuracy of whatever gates were flown before exit.
            if any(not math.isinf(d) for d in self.min_dist):
                self._report()
            if self.state.armed and not self.done:
                print('\nRTL...')
                self.set_mode('RTL')
        except Exception:
            pass


def main():
    rclpy.init()
    node = GatesMission()
    try:
        if node.startup():
            rclpy.spin(node)
    except KeyboardInterrupt:
        print('\ninterrupted by user')
    finally:
        node.safe_shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
