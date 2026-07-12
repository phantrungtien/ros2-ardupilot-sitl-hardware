#!/usr/bin/env python3
"""Fly the drone through racing ring(s) using ONLY the onboard D435i.

CONTROL half. The VISION half (simtofly_vision/gate_detect.py) finds the orange
ring and publishes /gate/obs = [visible, cx_off, cy_off, fill, defo, dist]; dist
is metres to the ring PLANE, ranged off the D435i depth stream. This node flies
to the ring and through it.

Frame: ENU velocity + ABSOLUTE yaw on /mavros/setpoint_raw/local
(FRAME_LOCAL_NED) -- the convention verified in SITL by mission_circle
(velocity.x=+East, y=+North, z=+Up; yaw is an absolute ENU heading). We do NOT
use body-frame setpoints (their MAVROS sign convention was the bug that flew the
drone away from the ring).

WHY THIS WORKS IN METRES, NOT PIXELS
------------------------------------
The ring's clear opening is 0.371 m and the drone is 0.321 m across the prop
tips, so there is 50 mm of margin -- and the ring has collision, so spending it
is a crash, not a warning. 50 mm is smaller than three biases that a pixel-space
controller cannot even see:

  * the colour camera sits 32.5 mm LEFT of the drone's centreline (it is bolted
    on the nose, and camera_link is the left IR imager -- see
    iris_with_d435i/model.sdf). Centring the ring in the IMAGE therefore parks
    the drone 32.5 mm off the ring's axis: 65% of the margin, gone for free.
  * the camera is welded rigid, so the drone's pitch tilts the sightline with it.
    5 deg of nose-down at commit range aims 123 mm low -- 2.5x the margin.
  * a tolerance in normalised pixels means a different number of millimetres at
    every range.

So instead of servoing cx/cy to zero, we reconstruct WHERE THE RING IS: turn
(cx, cy, dist) into metres in the drone's body frame (which is where the camera
offset lives), then rotate that by the drone's actual attitude quaternion into
ENU (which is where the pitch bias lives). Both biases cancel by construction
rather than by fudge factor, and every tolerance below is a real distance you
can compare against the 50 mm.

THE RING GOES BLIND BEFORE YOU REACH IT
---------------------------------------
The detector fits an ellipse to the ring's outer contour. Once the ring outgrows
the frame that contour is cut and the ring stops existing -- at full size, dead
ahead, metres from the props. The colour camera's vertical half-FOV is the binding
one, so this happens below BLIND_DIST = 1.26 m, and no tuning changes it.

Every distance in this file is built around that. COMMIT fires while the ring is
still WHOLE (>= FORCE_DIST), and the last stretch is flown on a world point rather
than on pixels. Losing the ring at close range is therefore not a failure -- it is
the expected end of the approach, and a cue to commit, never to go back to SEARCH.

Phases per gate:
  SEARCH  -- no ring, no lock: sweep the yaw until one enters view.
  ALIGN   -- ring seen: reconstruct it in ENU, face it, fly at it, decelerating
             with range; creep while still slewing onto it.
  HOLD    -- ring dropped out but we had it a moment ago and are still far: hold
             station with the nose on it and wait, instead of sweeping the whole
             approach away.
  COMMIT  -- close enough (and lined up to <= ALIGN_TOL, or simply at FORCE_DIST,
             or the ring went blind): fly the ring's WORLD point closed-loop off
             /mavros/local_position. Vision keeps refining that point while the
             ring is whole, then drops out and we fly on. Cross-track and height
             are corrected all the way through -- unlike a blind timed punch, which
             preserves whatever error it started with.
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
CTRL_HZ = 20.0           # setpoint stream rate (ArduPilot needs a steady stream)
OBS_TIMEOUT = 0.4        # s; older /gate/obs -> treat ring as not visible

# The drone spawns at world z = 0.195 (its landing gear), and MAVROS local z is
# measured from there -- so a 3.5 m "altitude" is world 3.695, i.e. 195 mm ABOVE
# the ring centre at world z = 3.5. 3.3 puts the nose on the ring instead.
TAKEOFF_ALT = 3.3        # m

# ----- camera, from realsense_d435i/model.sdf. MUST match gate_detect. -----
HFOV = 1.204277                                  # rad; the COLOUR sensor (69 deg)
W, H = 848, 480
TAN_H = math.tan(HFOV / 2.0)                     # 0.687; image x=+-1 -> +-this, per metre
TAN_V = TAN_H * H / W                            # 0.389; square pixels, so vfov follows

# Colour optical origin in the drone's body frame (X nose, Y left, Z up), from
# iris_with_d435i/model.sdf: d435i at (0.116, 0.0175, 0), colour +15 mm in Y.
CAM_X = 0.116            # m AHEAD of base_link -> the drone is further out than the camera reads
CAM_Y = 0.0325           # m LEFT of the centreline -> the bias that eats the margin
CAM_Z = 0.0

# ----- the margin this whole file exists to protect (gate_ring/model.sdf) -----
CLEAR_R = 0.371          # ring's clear opening radius
DRONE_R = 0.321          # drone half-width at the prop tips
RING_OUTER_R = 0.491     # 0.431 tube centreline + 0.06 tube -> what the camera sees
ALIGN_TOL = 0.03         # m; must be lined up this well to commit. < CLEAR_R - DRONE_R.

# THE RING GOES BLIND BEFORE YOU REACH IT, and that fact sets every distance below.
#
# gate_detect finds the ring by fitting an ellipse to its outer contour and checking
# the contour's area against the ellipse's. Once the ring outgrows the frame that
# contour is CUT, the area check fails, and the ring simply stops existing -- at
# full size, dead ahead, metres from the props.
#
# The binding limit is the colour camera's VERTICAL half-FOV (21.25 deg, the narrow
# one), so the ring is whole in frame only while
#
#     dist > RING_OUTER_R / TAN_V = 1.26 m
#
# Measured in SITL: last good frame dist=1.29 m, fill=0.60 -> major axis 509 px
# against an image 480 px tall. The next frame: "ring none".
#
# So EVERY commit threshold must clear this. The first cut of this file used
# COMMIT_DIST = 1.3 m of ENU range, i.e. dist <= 1.18 m -- BELOW the blind distance.
# The commit window was empty and COMMIT could never fire: the drone flew a perfect
# approach (miss = 0.021 m at 5 m), went blind at 1.26 m, fell back to SEARCH, and
# SEARCH stops dead and sweeps the yaw -- so it hovered in front of the gate and
# span. _selfcheck() now asserts the window is non-empty.
BLIND_DIST = RING_OUTER_R / TAN_V        # 1.26 m; nearer than this, no detection
COMMIT_DIST = 2.2        # m of ENU range; commit while the ring is still comfortably whole
FORCE_DIST = 1.6         # m; last chance. Commit on the best estimate we have rather
#                          than go blind still hunting for perfect alignment -- the
#                          COMMIT loop corrects cross-track and height anyway.
LOCK_HOLD = 2.0          # s; a ring seen this recently still counts as a lock
REFINE_A = 0.2           # low-pass on the ring's world point while it stays visible

# --- control tuning (SITL; retune on real hardware). ponytail: knobs. ---
VZ_KP, VZ_KI, VZ_KD = 1.0, 0.0, 0.05      # metres of height error -> m/s
VZ_MAX = 0.8             # m/s cap
SEARCH_YAW = 0.4         # rad/s yaw sweep when no ring is in view

# Approach speed = clamp(APPROACH_K*range, FWD_CREEP, FWD_CRUISE): decelerate as
# the ring nears; also creep (not cruise) while the ring is still well off the
# nose, so the yaw slew keeps it in the forward camera.
APPROACH_K = 0.5         # m/s per m of range
FWD_CRUISE = 1.2         # m/s cap when lined up and far
FWD_CREEP = 0.4          # m/s while still turning onto the ring
CREEP_ANG = 0.15         # rad (~9 deg) off the nose; above this, creep

FWD_PUNCH = 1.6          # m/s along the frozen approach axis
XT_KP = 2.0              # cross-track metres -> m/s of sideways correction
XT_MAX = 0.5             # m/s cap on that correction
PASS_MARGIN = 0.3        # m past the ring plane before calling it a pass
COMMIT_TIMEOUT = 5.0     # s watchdog only -- the pass is detected geometrically
HOVER_SECS = 5.0         # hover this long after the last gate, then RTL


def clamp(v, lo, hi):
    return lo if v < lo else hi if v > hi else v


def ring_body(cx, cy, dist):
    """(image offsets, range) -> the ring's centre in the drone's BODY frame (metres).

    Body frame is the SDF/ROS convention: X out the nose, Y left, Z up.

    The camera's optical frame has Z forward, X right, Y down, so a ring at
    normalised image offset (cx, cy) and range `dist` sits at
    (dist*cx*TAN_H) right and (dist*cy*TAN_V) down of the optical axis. Flip those
    into body axes and add where the camera actually is -- CAM_Y is the 32.5 mm
    that a pixel-centring controller silently gives away.
    """
    return (dist + CAM_X,
            -dist * cx * TAN_H + CAM_Y,
            -dist * cy * TAN_V + CAM_Z)


def body_to_enu(q, v):
    """Rotate a body-frame vector into ENU by the drone's attitude quaternion.

    v' = v + 2w(u x v) + 2u x (u x v). This is what cancels the pitch bias: the
    camera is welded to the airframe, so its sightline tilts with the drone, and
    the quaternion is the only honest way to undo that.
    """
    u = (q.x, q.y, q.z)
    w = q.w

    def cross(a, b):
        return (a[1] * b[2] - a[2] * b[1],
                a[2] * b[0] - a[0] * b[2],
                a[0] * b[1] - a[1] * b[0])

    uxv = cross(u, v)
    uxuxv = cross(u, uxv)
    return tuple(v[i] + 2.0 * w * uxv[i] + 2.0 * uxuxv[i] for i in range(3))


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
        self.ring_w = (0.0, 0.0, 0.0)   # the ring's point in ENU, as last seen
        self.lock_t = 0.0               # when we last saw it (0 = never)
        self.lock_bear = 0.0            # bearing to it when we last saw it
        self.ax = self.ay = 0.0         # frozen approach axis, unit horizontal
        self.cbear = 0.0                # frozen approach bearing
        self._warned_mode = False
        self._state = ''
        self.timer = self.create_timer(1.0 / CTRL_HZ, self._control_loop)

    def _obs_cb(self, msg):
        if len(msg.data) >= 6:
            self.obs = list(msg.data[:6])
            self.obs_t = time.time()

    def position(self):
        p = self.pose.pose.position
        return (p.x, p.y, p.z)

    def _publish(self, vx, vy, vz, yaw, tag, extra=''):
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
            print('  [gate {}/{}] {} v=({:+.2f},{:+.2f},{:+.2f}) yaw={:+.2f} {}'
                  .format(self.gate_idx + 1, NUM_GATES, tag, vx, vy, vz, yaw, extra))
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

        if self.phase == 'COMMIT':
            self._commit(now, heading)
            return

        sighting = self._sighting(now)

        # --- ALIGN: reconstruct the ring in ENU metres, then fly at it ---
        if sighting is not None:
            (dx, dy, dz), horiz, bearing, miss = sighting
            self.search_yaw = heading             # keep in sync for re-SEEK

            # Commit while the ring is still WHOLE in frame. FORCE_DIST fires even if
            # we never got inside ALIGN_TOL: going blind while still hunting for a
            # perfect line is strictly worse than committing on a good estimate, since
            # the COMMIT loop keeps correcting cross-track and height all the way in.
            if horiz <= FORCE_DIST or (horiz <= COMMIT_DIST and miss <= ALIGN_TOL):
                self._enter_commit(now, horiz, bearing,
                                   'miss={:.3f}m'.format(miss))
                return

            vz = self.vz_pid.step(dz, now)        # dz>0 = ring above -> climb
            speed = clamp(APPROACH_K * horiz, FWD_CREEP, FWD_CRUISE)
            if miss / horiz > CREEP_ANG:          # still slewing onto it -> creep
                speed = min(speed, FWD_CREEP)
            self._publish(speed * math.cos(bearing), speed * math.sin(bearing),
                          vz, bearing, 'ALIGN',
                          'd={:.2f}m miss={:.3f}m'.format(horiz, miss))
            return

        # --- ring not visible, but we had it a moment ago ---
        # Do NOT fall back to SEARCH here. Losing the ring at close range is not a
        # failure, it is the EXPECTED end of the approach: the ring outgrows the frame
        # (see BLIND_DIST). Treat it as the cue to commit on the point we already know.
        if self.lock_t and now - self.lock_t < LOCK_HOLD:
            px, py, _ = self.position()
            ex, ey = self.ring_w[0] - px, self.ring_w[1] - py
            horiz = math.hypot(ex, ey)
            if horiz <= COMMIT_DIST:
                self._enter_commit(now, horiz, math.atan2(ey, ex), 'ring went blind')
                return
            # Lost it further out -- a dropped frame, not the blind zone. Hold station
            # and keep the nose on it rather than sweeping the approach away.
            self._publish(0.0, 0.0, 0.0, self.lock_bear, 'HOLD',
                          'd={:.2f}m, waiting for re-acquire'.format(horiz))
            return

        # --- SEEK: no ring, no lock. Sweep the yaw until one enters view. ---
        self.vz_pid.reset()
        self.search_yaw = wrap_to_pi(self.search_yaw + SEARCH_YAW / CTRL_HZ)
        self._publish(0.0, 0.0, 0.0, self.search_yaw, 'SEARCH')

    def _sighting(self, now):
        """Fresh ring observation -> (enu_offset, horiz, bearing, miss), else None.

        Also refreshes the lock: the ring's world point, when we saw it, and where it
        was. Those survive the ring going blind, which is what COMMIT flies on.
        """
        if (now - self.obs_t) >= OBS_TIMEOUT or self.obs[0] <= 0.5:
            return None
        _, cx, cy, _, _, dist = self.obs
        x_b, y_b, z_b = ring_body(cx, cy, dist)
        dx, dy, dz = body_to_enu(self.pose.pose.orientation, (x_b, y_b, z_b))
        horiz = math.hypot(dx, dy)
        if horiz < 1e-3:                          # ring straight overhead: nonsense
            return None

        px, py, pz = self.position()
        seen = (px + dx, py + dy, pz + dz)
        if self.lock_t and now - self.lock_t < LOCK_HOLD:
            self.ring_w = tuple((1.0 - REFINE_A) * self.ring_w[i] + REFINE_A * seen[i]
                                for i in range(3))
        else:
            self.ring_w = seen                    # first sight: no history to blend
        bearing = math.atan2(dy, dx)
        self.lock_t = now
        self.lock_bear = bearing

        # Miss: how far off the drone's own axis the ring sits, in the two directions
        # the 50 mm margin is spent on -- lateral (where the camera offset lives) and
        # height (where the pitch bias lives).
        return (dx, dy, dz), horiz, bearing, math.hypot(y_b, dz)

    def _enter_commit(self, now, horiz, bearing, why):
        self.ax, self.ay = math.cos(bearing), math.sin(bearing)
        self.cbear = bearing
        self.commit_t = now
        self.phase = 'COMMIT'
        print('  [gate {}/{}] COMMIT at {:.2f}m ({}) -- margin {:.3f}m'
              .format(self.gate_idx + 1, NUM_GATES, horiz, why, CLEAR_R - DRONE_R))

    def _commit(self, now, heading):
        """Fly the ring's world point closed-loop until we are past it.

        No timer: the ring does not move, and /mavros/local_position keeps telling us
        where we are relative to it. Vision still refines the point WHILE the ring is
        whole in frame; past BLIND_DIST the ellipse fit is junk, so we stop listening
        and fly the point -- which is exactly the phase this design exists for.
        """
        if (now - self.obs_t) < OBS_TIMEOUT and self.obs[0] > 0.5:
            _, cx, cy, _, defo, dist = self.obs
            if dist > BLIND_DIST and defo > 0.8:      # whole in frame, and face-on
                dx, dy, dz = body_to_enu(self.pose.pose.orientation,
                                         ring_body(cx, cy, dist))
                px, py, pz = self.position()
                seen = (px + dx, py + dy, pz + dz)
                self.ring_w = tuple(
                    (1.0 - REFINE_A) * self.ring_w[i] + REFINE_A * seen[i]
                    for i in range(3))

        px, py, pz = self.position()
        ex, ey, ez = (self.ring_w[0] - px, self.ring_w[1] - py, self.ring_w[2] - pz)
        along = ex * self.ax + ey * self.ay        # + = ring still ahead
        cross = -ex * self.ay + ey * self.ax       # + = ring is left of the axis

        if along < -PASS_MARGIN:
            self._pass_gate(now, heading)
            return
        if now - self.commit_t > COMMIT_TIMEOUT:   # watchdog, not the mechanism
            self.get_logger().warn('commit watchdog -> hover')
            self.phase = 'HOVER'
            self.hover_t = now
            return

        xt = clamp(XT_KP * cross, -XT_MAX, XT_MAX)
        vx = FWD_PUNCH * self.ax - xt * self.ay    # forward + sideways correction
        vy = FWD_PUNCH * self.ay + xt * self.ax
        vz = clamp(VZ_KP * ez, -VZ_MAX, VZ_MAX)
        self._publish(vx, vy, vz, self.cbear, 'COMMIT',
                      'to-go={:+.2f}m xt={:+.3f}m dz={:+.3f}m'.format(along, cross, ez))

    def _pass_gate(self, now, heading):
        print('\n✓ passed gate #{}/{}'.format(self.gate_idx + 1, NUM_GATES))
        self.gate_idx += 1
        if self.gate_idx < NUM_GATES:
            self.phase = 'SEEK'                   # go find the next ring
            self.search_yaw = heading
            self.lock_t = 0.0                     # this ring's lock must not survive
            self.vz_pid.reset()
        else:
            self.phase = 'HOVER'
            self.hover_t = now

    def startup(self):
        print('=' * 60)
        print('GATES (D435i colour+depth): {} ring(s) | search -> align -> commit '
              '-> hover {:.0f}s -> RTL'.format(NUM_GATES, HOVER_SECS))
        print('clear radius {:.3f}m - drone {:.3f}m = {:.3f}m margin; commit at '
              'miss <= {:.3f}m'.format(CLEAR_R, DRONE_R, CLEAR_R - DRONE_R, ALIGN_TOL))
        print('=' * 60)
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


class _Q:
    """Bare quaternion for the self-check (no ROS message types needed)."""

    def __init__(self, w, x, y, z):
        self.w, self.x, self.y, self.z = w, x, y, z


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

    # The tolerance we commit on must fit inside the physical margin, or the ring
    # is a crash rather than a gate.
    assert ALIGN_TOL < CLEAR_R - DRONE_R

    # THE COMMIT WINDOW MUST NOT BE EMPTY. This is the regression test for the bug
    # that made the first SITL run hover in front of the gate and spin: the ring goes
    # undetectable at BLIND_DIST (1.26 m), but COMMIT_DIST was 1.3 m of ENU range --
    # i.e. dist <= 1.18 m, which the detector can never deliver. COMMIT could not fire
    # at any distance, and losing the ring dumped the drone into a yaw sweep.
    # Every commit threshold is an ENU range, so subtract CAM_X to compare with a
    # camera-measured dist.
    assert abs(BLIND_DIST - 1.262) < 0.005, BLIND_DIST
    assert COMMIT_DIST - CAM_X > BLIND_DIST, 'commit window empty'
    assert FORCE_DIST - CAM_X > BLIND_DIST, 'force-commit window empty'
    # And FORCE_DIST has to be the inner one, or it would pre-empt the aligned commit.
    assert FORCE_DIST < COMMIT_DIST

    # A ring that fills the frame vertically is exactly the blind case: at BLIND_DIST
    # its outer edge lands on the top and bottom of the image.
    assert abs(RING_OUTER_R / BLIND_DIST - TAN_V) < 1e-9

    # --- the camera offset: a ring dead-centre in the IMAGE is NOT dead ahead ---
    x_b, y_b, z_b = ring_body(0.0, 0.0, 5.0)
    assert abs(x_b - 5.116) < 1e-6, x_b        # camera sits ahead of base_link
    assert abs(y_b - 0.0325) < 1e-6, y_b       # ...and 32.5 mm left of it
    assert abs(z_b) < 1e-6, z_b
    # 32.5 mm is 65% of the 50 mm margin: centring on pixels alone would spend it.
    assert y_b > 0.6 * (CLEAR_R - DRONE_R)

    # Image right (cx>0) -> the ring is to the drone's RIGHT (body -Y).
    assert ring_body(0.5, 0.0, 5.0)[1] < 0.0
    # Image down (cy>0) -> the ring is BELOW the drone (body -Z).
    assert ring_body(0.0, 0.5, 5.0)[2] < 0.0
    # Edge of frame subtends exactly the half-FOV.
    assert abs(math.atan2(-ring_body(1.0, 0.0, 5.0)[1] + CAM_Y, 5.0) - HFOV / 2.0) < 1e-9

    # --- attitude: identity leaves the body vector alone ---
    v = (1.0, 2.0, 3.0)
    assert max(abs(a - b) for a, b in zip(body_to_enu(_Q(1, 0, 0, 0), v), v)) < 1e-9

    # Yaw 90 deg (nose to North): body +X must come out as ENU +Y.
    q = _Q(math.cos(math.pi / 4), 0, 0, math.sin(math.pi / 4))
    e = body_to_enu(q, (1.0, 0.0, 0.0))
    assert abs(e[0]) < 1e-9 and abs(e[1] - 1.0) < 1e-9, e

    # --- the pitch bias, which is the whole reason for the quaternion ---
    # Nose UP 10 deg (rotation about body +Y=left is nose-DOWN in FLU, so negate).
    th = math.radians(10.0)
    q = _Q(math.cos(-th / 2), 0.0, math.sin(-th / 2), 0.0)
    e = body_to_enu(q, (1.0, 0.0, 0.0))
    assert abs(e[2] - math.sin(th)) < 1e-9, e   # nose really is up

    # Ring centred in the image at 5 m while pitched 10 deg nose-up: it is NOT at
    # our altitude, it is 5*sin(10) = 0.87 m ABOVE us. A cy-only controller reads
    # this as "centred, hold height" and flies under the gate.
    _, _, dz = body_to_enu(q, ring_body(0.0, 0.0, 5.0))
    assert abs(dz - 5.116 * math.sin(th)) < 1e-6, dz
    assert dz > 0.8

    # And the bias that actually bites: 5 deg NOSE-DOWN at commit range aims low by
    # far more than the margin -- 114 mm against 50 mm.
    th = math.radians(5.0)
    q = _Q(math.cos(th / 2), 0.0, math.sin(th / 2), 0.0)   # nose down
    _, _, dz = body_to_enu(q, ring_body(0.0, 0.0, 1.3))
    assert dz < -(CLEAR_R - DRONE_R), dz
    assert abs(dz + (1.3 + CAM_X) * math.sin(th)) < 1e-6, dz
    assert abs(dz + 0.123) < 0.002, dz

    print('mission_gates self-check OK (margin {:.3f}m, commit tol {:.3f}m)'
          .format(CLEAR_R - DRONE_R, ALIGN_TOL))


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
