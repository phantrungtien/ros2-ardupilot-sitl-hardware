#!/usr/bin/env python3
"""Fly the drone through racing ring(s) using ONLY the onboard camera.

CONTROL half. The VISION half (simtofly_vision/gate_detect.py) finds the orange
ring and publishes /gate/obs = [visible, cx_off, cy_off, fill, defo, dist];
dist is a monocular range estimate (2D detection + depth from the known ring
size). This node flies to the ring and through it.

Frame: ENU velocity + ABSOLUTE yaw on /mavros/setpoint_raw/local
(FRAME_LOCAL_NED) -- the convention verified in SITL by mission_circle
(velocity.x=+East, y=+North, z=+Up; yaw is an absolute ENU heading). We do NOT
use body-frame setpoints (their MAVROS sign convention was the bug that flew
the drone away from the ring).

How "toward the ring" is computed without the ring's world position: the
forward camera is welded level, so its optical axis points along the drone's
ENU heading. The ring's horizontal offset in the image (cx_off) is an angle off
that axis, so the ring's ENU bearing = heading + BEARING_SIGN*atan(cx*tan(HFOV/2)).
We then (a) yaw the nose to that bearing (keeps the ring in the forward camera),
and (b) command ENU velocity ALONG that bearing -- always toward the ring.

Phases per gate:
  SEARCH  -- no ring: sweep the yaw until one enters view.
  ALIGN   -- ring seen: face its bearing + fly toward it, decelerating with
             dist; creep until it is roughly centred so it stays in view.
  COMMIT  -- close (dist<=COMMIT_DIST) & centred: freeze the bearing and punch
             straight along it for a short burst (point-blank the image is
             unreliable). Ring cleared -> passed.
After the last gate: HOVER a few seconds, then RTL.

Flow: FCU -> GUIDED -> arm -> takeoff -> [per gate: SEARCH->ALIGN->COMMIT] ->
HOVER -> RTL. Leaving GUIDED or Ctrl+C -> RTL.

Run:
    ros2 launch simtofly_mavros_sitl gates_mission.launch.py
"""

import math
import time

import rclpy
from mavros_msgs.msg import PositionTarget
from std_msgs.msg import Float32MultiArray

from simtofly_mavros_sitl.circle_control import wrap_to_pi
from simtofly_mavros_sitl.guided_base import GuidedNode

NUM_GATES = 1            # rings to fly, in the order they are found (label = pass #)
TAKEOFF_ALT = 3.5        # climb here before searching (m) ~ ring centre height
CTRL_HZ = 20.0           # setpoint stream rate (ArduPilot needs a steady stream)
OBS_TIMEOUT = 0.4        # s; older /gate/obs -> treat ring as not visible

# camera geometry (must match gate_detect / model.sdf): image cx_off -> bearing.
HFOV = 2.0
HALF_TAN = math.tan(HFOV / 2.0)

# --- control tuning (SITL; retune on real hardware). ponytail: knobs. ---
VZ_KP, VZ_KI, VZ_KD = 1.0, 0.0, 0.05      # cy_off (-1..1) -> vertical vel (m/s)
VZ_MAX = 0.8             # m/s cap
SEARCH_YAW = 0.4         # rad/s yaw sweep when no ring is in view
CENTER_TOL = 0.15        # |cx_off|,|cy_off| below this = "centred"

# Approach speed = clamp(APPROACH_K*dist, FWD_CREEP, FWD_CRUISE): decelerate as
# the ring nears; also creep (not cruise) until the ring is roughly centred so
# the yaw slew keeps it in the forward camera.
APPROACH_K = 0.5         # m/s per m of range
FWD_CRUISE = 1.2         # m/s cap when centred and far
FWD_CREEP = 0.4          # m/s while still turning to face the ring
# Commit: once this close AND centred, punch straight through for a fixed burst.
COMMIT_DIST = 1.3        # m; range at which to commit to the punch-through
FWD_PUNCH = 1.6          # m/s straight through the hole
COMMIT_TIME = 1.6        # s of forward punch (clears the ~0.5 m hoop + drone)
COMMIT_MIN = 0.7         # s min punch before ring-loss counts as "through"
HOVER_SECS = 5.0         # hover this long after the last gate, then RTL

# First-run SITL-verify signs (flip if reversed):
BEARING_SIGN = -1.0   # ring to the RIGHT in image (cx>0) is CLOCKWISE = lower ENU yaw
VZ_SIGN = -1.0        # ring BELOW centre (cy>0) -> DESCEND (ENU vz is +Up)


def clamp(v, lo, hi):
    return lo if v < lo else hi if v > hi else v


def ring_bearing(heading, cx):
    """ENU bearing of the ring from the drone heading + image x-offset."""
    return wrap_to_pi(heading + BEARING_SIGN * math.atan(cx * HALF_TAN))


class PID:
    """Minimal PID with integral + output clamping (anti-windup)."""

    def __init__(self, kp, ki, kd, out_max, i_max=1.0):
        self.kp, self.ki, self.kd = kp, ki, kd
        self.out_max, self.i_max = out_max, i_max
        self.reset()

    def reset(self):
        self.i = 0.0
        self.prev = 0.0
        self.t = None

    def step(self, err, now):
        dt = 0.0 if self.t is None else max(now - self.t, 1e-3)
        self.t = now
        self.i = clamp(self.i + err * dt, -self.i_max, self.i_max)
        d = 0.0 if dt == 0.0 else (err - self.prev) / dt
        self.prev = err
        out = self.kp * err + self.ki * self.i + self.kd * d
        return clamp(out, -self.out_max, self.out_max)


# velocity + absolute-yaw mask (ignore position, accel, yaw_rate) -- as verified
# by mission_circle.
_MASK = (PositionTarget.IGNORE_PX | PositionTarget.IGNORE_PY
         | PositionTarget.IGNORE_PZ | PositionTarget.IGNORE_AFX
         | PositionTarget.IGNORE_AFY | PositionTarget.IGNORE_AFZ
         | PositionTarget.IGNORE_YAW_RATE)


class GatesMission(GuidedNode):
    def __init__(self):
        super().__init__('gates_mission')
        self.sp_pub = self.create_publisher(
            PositionTarget, '/mavros/setpoint_raw/local', 10)
        self.create_subscription(
            Float32MultiArray, '/gate/obs', self._obs_cb, 10)

        self.vz_pid = PID(VZ_KP, VZ_KI, VZ_KD, VZ_MAX)
        self.obs = [0.0] * 6     # [visible, cx, cy, fill, defo, dist]
        self.obs_t = 0.0
        self.active = False
        self.done = False
        self.phase = 'SEEK'      # SEEK -> COMMIT -> HOVER -> (RTL)
        self.gate_idx = 0        # 0-based index of the gate we are chasing
        self.search_yaw = 0.0    # swept absolute yaw target while searching
        self.commit_t = 0.0
        self.hover_t = 0.0
        self.cvx = self.cvy = self.cbear = 0.0   # frozen commit velocity/heading
        self._warned_mode = False
        self._state = ''
        self.timer = self.create_timer(1.0 / CTRL_HZ, self._control_loop)

    def _obs_cb(self, msg):
        if len(msg.data) >= 6:
            self.obs = list(msg.data[:6])
            self.obs_t = time.time()

    def _publish(self, vx, vy, vz, yaw, tag):
        sp = PositionTarget()
        sp.header.stamp = self.get_clock().now().to_msg()
        sp.coordinate_frame = PositionTarget.FRAME_LOCAL_NED
        sp.type_mask = _MASK
        sp.velocity.x = float(vx)          # +East  (verified)
        sp.velocity.y = float(vy)          # +North (verified)
        sp.velocity.z = float(vz)          # +Up
        sp.yaw = float(wrap_to_pi(yaw))    # absolute ENU heading
        self.sp_pub.publish(sp)
        if tag != self._state:
            print('  [gate {}/{}] {} v=({:+.2f},{:+.2f},{:+.2f}) yaw={:+.2f}'
                  .format(self.gate_idx + 1, NUM_GATES, tag, vx, vy, vz, yaw))
            self._state = tag

    def _control_loop(self):
        if not self.active or self.done:
            return
        if self.state.mode != 'GUIDED':          # pilot took over -> release
            if not self._warned_mode:
                self.get_logger().warn('mode={} != GUIDED -> release'
                                       .format(self.state.mode))
                self._warned_mode = True
            return
        self._warned_mode = False

        now = time.time()
        fresh = (now - self.obs_t) < OBS_TIMEOUT
        visible = fresh and self.obs[0] > 0.5
        _, cx, cy, fill, defo, dist = self.obs
        heading = self.heading_enu()

        # --- HOVER then RTL (after the last gate) ---
        if self.phase == 'HOVER':
            if now - self.hover_t < HOVER_SECS:
                self._publish(0.0, 0.0, 0.0, heading, 'HOVER')   # hold
            else:
                print('\n✓ {} gate(s) done, hover {:.0f}s -> RTL'
                      .format(NUM_GATES, HOVER_SECS))
                self.done = True
                self.active = False
                self.set_mode('RTL')
            return

        # --- COMMIT: punch straight through along the frozen bearing ---
        if self.phase == 'COMMIT':
            self._publish(self.cvx, self.cvy, 0.0, self.cbear, 'COMMIT')
            dt = now - self.commit_t
            if dt > COMMIT_TIME or (not visible and dt > COMMIT_MIN):
                print('\n✓ passed gate #{}/{}'.format(self.gate_idx + 1, NUM_GATES))
                self.gate_idx += 1
                if self.gate_idx < NUM_GATES:
                    self.phase = 'SEEK'                  # go find the next ring
                    self.search_yaw = heading
                    self.vz_pid.reset()
                else:
                    self.phase = 'HOVER'
                    self.hover_t = now
            return

        # --- SEEK: sweep the yaw until a ring is in view ---
        if not visible:
            self.vz_pid.reset()
            self.search_yaw = wrap_to_pi(self.search_yaw + SEARCH_YAW / CTRL_HZ)
            self._publish(0.0, 0.0, 0.0, self.search_yaw, 'SEARCH')
            return

        # --- ALIGN: face the ring's bearing and fly toward it in ENU ---
        bearing = ring_bearing(heading, cx)
        self.search_yaw = heading                     # keep in sync for re-SEEK
        vz = VZ_SIGN * self.vz_pid.step(cy, now)
        centred = abs(cx) < CENTER_TOL and abs(cy) < CENTER_TOL

        if 0.0 < dist <= COMMIT_DIST and centred:     # close + lined up -> commit
            self.cbear = bearing
            self.cvx = FWD_PUNCH * math.cos(bearing)
            self.cvy = FWD_PUNCH * math.sin(bearing)
            self.commit_t = now
            self.phase = 'COMMIT'
            self._publish(self.cvx, self.cvy, 0.0, bearing, 'COMMIT')
            return

        speed = clamp(APPROACH_K * dist, FWD_CREEP, FWD_CRUISE) if dist > 0 else FWD_CREEP
        if abs(cx) > 2.0 * CENTER_TOL:                # still turning to face -> creep
            speed = min(speed, FWD_CREEP)
        self._publish(speed * math.cos(bearing), speed * math.sin(bearing),
                      vz, bearing, 'ALIGN')

    def startup(self):
        print('=' * 56)
        print('GATES (vision+depth): {} ring(s) | search -> approach -> commit '
              '-> hover {:.0f}s -> RTL'.format(NUM_GATES, HOVER_SECS))
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
        print('✓ at {:.1f}m -> searching for the ring'.format(TAKEOFF_ALT))
        time.sleep(2)
        self.search_yaw = self.heading_enu()
        self.active = True
        print('✓ flying by camera (Ctrl+C -> RTL)\n')
        return True

    def safe_shutdown(self):
        try:
            if self.state.armed and not self.done:
                print('\nRTL...')
                self.set_mode('RTL')
        except Exception:
            pass


def _selfcheck():
    """Pure control-math check -- no ROS. Run: python3 mission_gates.py --selfcheck"""
    assert clamp(5, -1, 1) == 1 and clamp(-5, -1, 1) == -1 and clamp(0, -1, 1) == 0
    # PID drives a simple integrator (x += out*dt) to the target and clamps.
    pid = PID(0.5, 0.0, 0.0, out_max=10.0)
    x, dt = 0.0, 0.05
    for k in range(400):
        out = pid.step(1.0 - x, k * dt)
        assert abs(out) <= 10.0
        x += out * dt
    assert abs(1.0 - x) < 1e-2, x
    assert PID(100.0, 0, 0, out_max=0.8).step(9.9, 0.0) == 0.8
    # Bearing geometry: centred ring keeps heading; ring RIGHT (cx>0) turns the
    # nose clockwise (lower ENU yaw), ring LEFT turns it counter-clockwise.
    assert abs(ring_bearing(1.0, 0.0) - 1.0) < 1e-9
    assert ring_bearing(0.0, 1.0) < 0.0, 'ring-right should yaw clockwise'
    assert ring_bearing(0.0, -1.0) > 0.0, 'ring-left should yaw counter-clockwise'
    assert abs(ring_bearing(0.0, 1.0) + math.atan(HALF_TAN)) < 1e-9
    print('mission_gates self-check OK')


def main():
    import sys
    if '--selfcheck' in sys.argv:
        _selfcheck()
        return
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
