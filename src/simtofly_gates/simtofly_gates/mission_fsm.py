#!/usr/bin/env python3
"""N6 mission supervisor -- owns PASSED, the end of the mission, and safety.

Plan par. 6.2 draws N6 as an eight-state FSM. Six of those states are ALREADY
running code and are deliberately not rebuilt here:

    TAKEOFF          N5 trajectory_follower.run()  (stream -> GUIDED -> arm ->
                     takeoff -> settle, and it takes over an aircraft that is
                     already flying)
    SEARCH / PLAN    N4 gate_planner.pick_target() + needs_replan()
    FLY_TO_PRE       N5 stream(), Mueller segments off /planner/path
    ALIGN / TRAVERSE N5 too -- the P_pre/C/P_post anchors come baked into the
                     path N4 publishes, so "align then traverse" is one
                     continuous trajectory (par. 7 R10 fly-through)

Rebuilding them as an explicit state machine would replace a chain that already
measures 4/4 gates at <0.02 m with an untested copy. What was genuinely missing:

  1. CHECK_PASS done right. N4 currently calls a gate passed on n.(p - C) > 0,
     which par. 7 R3 says is wrong: drift wide past the rim and the gate is
     marked done without ever going through the hole, so the mission silently
     skips it. This node runs the R3 test -- sign change of s(t), interpolate
     the crossing, measure the off-axis miss THERE -- and publishes the verdict
     on /mission/passed, which N4 consumes instead of its own guess.
  2. An end. With every gate passed N4 logs "khong con muc tieu" and N5 holds
     the last point; the drone hovers until the battery says otherwise. Here it
     lands.
  3. The par. 6.3 envelope: geofence, pose watchdog, and not fighting a mode
     change that came from somewhere else.

Tracking-error ABORT (par. 6.3) stays in N5 -- it owns the commanded state that
the error is measured against; N5 already holds at 0.5 m.

Run:  ros2 run simtofly_gates mission_fsm
      ros2 run simtofly_gates mission_fsm --expect 4    (say how many gates the
                                          course HAS, so a run that never sees
                                          one of them says so instead of
                                          reporting the subset as a full course)
      python3 mission_fsm.py --selfcheck      (geometry + bookkeeping, no ROS)
"""
import sys
import time

import numpy as np
import rclpy

try:
    from simtofly_gates.gate_order import pick_gate
except ImportError:                       # running the file directly
    from gate_order import pick_gate
from geometry_msgs.msg import Pose, PoseArray
from rclpy.qos import (QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile,
                       QoSReliabilityPolicy)

try:
    from simtofly_mavros_sitl.guided_base import GuidedNode, mavros_qos
except ImportError:
    from guided_base import GuidedNode, mavros_qos

R_IN = 0.75             # m; ring hole radius
CLEAR_MIN = 0.25        # m; par. 12 stage 6 bar. A crossing closer to the rim
                        # than this counts as a graze, not a pass -- so N4 keeps
                        # the gate and flies it again.
KEY_R = 1.0             # m; same gate if two centres are within this. Course
                        # spacing is >= 4 m, so this cannot merge two gates.
GATE_TIMEOUT = 90.0     # s chasing ONE gate before giving up on it. Without
                        # this a gate that can never be threaded (bad pose,
                        # blocked approach) parks the mission forever, because
                        # the honest R3 test -- unlike the n.(p-C) one -- keeps
                        # the gate on the list after a bad crossing. Counted
                        # from when the gate became the TARGET, never from when
                        # it appeared on the map: the whole course confirms
                        # early, so map-age would time out gates 3 and 4 while
                        # the drone was still flying gate 1.
END_MAX = 75.0          # s after the last known gate is done, whatever is still
                        # in frame. A gate already passed can stay in view for
                        # ever, and without a ceiling the detection-aware quiet
                        # timer below would hold the drone up until the battery
                        # settled it.
                        # This number was sized to cover N5's closing sweep,
                        # which no longer exists -- so it is now just a generous
                        # ceiling with no measurement behind the exact value.
                        # Anything comfortably above END_QUIET would do.
END_QUIET = 20.0        # s with every known gate done and no new one confirmed
                        # -> the course is finished.
TICK_HZ = 10.0
POSE_TIMEOUT = 5.0      # s without /mavros/local_position/pose -> LAND (par. 6.3)
FENCE = (-40.0, 40.0, -40.0, 40.0, 0.0, 15.0)   # xmin xmax ymin ymax zmin zmax


def zaxis_from_quat(x, y, z, w):
    return np.array([2 * (x * z + w * y), 2 * (y * z - w * x),
                     1 - 2 * (x * x + y * y)])


def plane_crossing(prev_q, prev_s, q, s, C, n):
    """Where the segment prev_q->q pierces the gate plane, and how far off-axis.

    Returns (point, off-axis distance), or None when the segment stays on one
    side. n must be unit length. This is the par. 7 R3 test: measuring the miss
    at the current sample instead is wrong in both directions -- the drone can
    pass clean through and drift wide a moment later, or fly outside the ring
    and happen to line up when sampled.
    """
    if not (prev_s < 0.0 <= s or s < 0.0 <= prev_s):
        return None
    t = prev_s / (prev_s - s) if abs(prev_s - s) > 1e-12 else 0.0
    star = prev_q + t * (q - prev_q)
    rel = star - C
    return star, float(np.linalg.norm(rel - (n @ rel) * n))


def outside(p, fence=FENCE):
    """True when p is out of the soft geofence box (par. 6.3)."""
    x0, x1, y0, y1, z0, z1 = fence
    return not (x0 <= p[0] <= x1 and y0 <= p[1] <= y1 and z0 <= p[2] <= z1)


def _arg_int(flag, default):
    """`--flag N` off the command line. argv rather than a ROS parameter to match
    --selfcheck / --partial / --no-scan, which is how every other switch in this
    package is spelled."""
    argv = sys.argv
    if flag in argv and argv.index(flag) + 1 < len(argv):
        try:
            return int(argv[argv.index(flag) + 1])
        except ValueError:
            pass
    return default


class MissionFSM(GuidedNode):
    # states: INIT -> RUN -> DONE, plus ABORT from anywhere
    def __init__(self):
        super().__init__('mission_fsm')
        # PASSED is STATE, not a stream: N4 may start late or restart, and must
        # still learn which gates are already done. Latch it.
        self.pub = self.create_publisher(
            PoseArray, '/mission/passed',
            QoSProfile(durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
                       reliability=QoSReliabilityPolicy.RELIABLE,
                       history=QoSHistoryPolicy.KEEP_LAST, depth=1))
        self.create_subscription(PoseArray, '/gate/map', self._map, 10)
        # Raw detections too: a gate being LOOKED at is a reason to wait, and it
        # arrives ~5 sightings before the map hears about it.
        self.create_subscription(PoseArray, '/gate/detections',
                                 self._detect, 10)
        self.t_detect = 0.0
        self.t_all_done = 0.0   # when the last KNOWN gate was finished
        self.gates = {}         # key -> (centre, unit normal), as the DRONE believes
        self.home = None        # launch point; see gate_order.pick_gate
        self.prev = {}          # key -> (position, signed distance) last pose
        self.done = {}          # key -> reason string
        self.active = None      # key of the gate N4 is flying at right now
        self.t_active = 0.0     # when it became that
        self.t_pose = 0.0
        self.t_quiet = time.time()
        self.state = 'INIT'     # NOTE: shadows nothing -- GuidedNode keeps the
        self.fcu = None         # MAVLink State in .fcu here, see _state_cb
        self.mode_seen = None
        # How many gates the COURSE has, as opposed to how many the drone found.
        # Nothing on the wire carries this: /gate/map is the drone's own belief,
        # so a gate never confirmed is not in it, is never graded, and is never
        # missed -- "4/4 xong" on a three-gate map reads exactly like success.
        # Measured over four runs (2026-07-21), TWO of them flew three of four
        # gates and reported completion, on both worlds. The operator is the only
        # one who knows the real number, because they placed the gates.
        # 0 = unset = the old behaviour, so no existing run changes its verdict.
        self.expect = _arg_int('--expect', 0)

    # GuidedNode stores the MAVLink State in self.state; this node needs that
    # name for the mission state, so redirect it.
    def _state_cb(self, msg):
        self.fcu = msg

    def _pose_cb(self, msg):
        super()._pose_cb(msg)
        self.t_pose = time.time()
        p = msg.pose.position
        self._on_pose(np.array([p.x, p.y, p.z]))

    def _key(self, C):
        """Gate identity that survives the map being reordered.

        The index in /gate/map is NOT stable -- gate_tracker publishes the
        CONFIRMED subset of an append-ordered list, so a late confirmation
        inserts ahead of gates already being tracked and shifts every index
        after it. Position is stable; gates never move and sit metres apart.
        """
        for k, (C0, _) in self.gates.items():
            if float(np.linalg.norm(C - C0)) < KEY_R:
                return k
        return len(self.gates)

    def _map(self, m):
        for pose in m.poses:
            p, o = pose.position, pose.orientation
            C = np.array([p.x, p.y, p.z])
            n = zaxis_from_quat(o.x, o.y, o.z, o.w)
            k = self._key(C)
            if k not in self.gates:
                self.t_quiet = time.time()
                self.get_logger().info('vong moi #{} tai {}'.format(
                    k, np.round(C, 2)))
            self.gates[k] = (C, n / (np.linalg.norm(n) + 1e-12))

    def _on_pose(self, q):
        if self.home is None:
            self.home = q.copy()    # same latch as N4's on_pose: both must call
                                    # gate_order with the SAME launch point or
                                    # they can classify a gate differently
        for k, (C, n) in self.gates.items():
            if k in self.done:
                continue
            s = float(n @ (q - C))
            prev = self.prev.get(k)
            self.prev[k] = (q, s)
            if prev is None:
                continue                       # need two samples to see a sign change
            hit = plane_crossing(prev[0], prev[1], q, s, C, n)
            if hit is None:
                continue
            self._judge(k, C, *hit)

    def _judge(self, k, C, star, miss):
        clear = R_IN - miss
        if clear > CLEAR_MIN:
            self._finish(k, 'LOT (lech truc {:.3f} m, cach vanh {:.3f} m)'
                         .format(miss, clear))
            return
        # Crossed the plane but not cleanly through the hole. Do NOT mark it
        # done: leaving it on the list is what makes N4 turn around and try
        # again, and is the whole point of R3 over n.(p - C) > 0.
        self.get_logger().warn(
            'vong {} cat mat phang tai {} nhung TRUOT (lech {:.3f} m, vanh '
            '{:.2f} m) -- giu lai de bay lai'.format(
                k, np.round(star, 2), miss, R_IN))

    def _finish(self, k, reason):
        self.done[k] = reason
        self.t_quiet = time.time()
        self.get_logger().info('vong {} XONG: {}'.format(k, reason))
        self._publish()

    def _publish(self):
        msg = PoseArray()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        for k in sorted(self.done):
            pose = Pose()
            (pose.position.x, pose.position.y,
             pose.position.z) = (float(v) for v in self.gates[k][0])
            pose.orientation.w = 1.0
            msg.poses.append(pose)
        self.pub.publish(msg)

    def target(self, drone):
        """The gate N4 is flying at.

        Same call as N4's pick_target, on purpose: N6 does not tell N4 which
        gate to fly, it only tells it which ones are finished, so the timeout
        below only measures the right gate while both agree. They used to be
        two hand-copies of `min(left, key=distance)`; the rule lives in
        gate_order now so they cannot drift apart."""
        left = [k for k in self.gates if k not in self.done]
        if not left:
            return None
        i = pick_gate([self.gates[k][0] for k in left], drone,
                      self.home if self.home is not None else drone,
                      [C for C, _ in self.gates.values()])
        return left[i]

    def _detect(self, msg):
        if msg.poses:
            self.t_detect = time.time()

    def check_timeouts(self, now, drone):
        """Give up on the gate that has been the target for too long."""
        k = self.target(drone)
        if k != self.active:
            self.active, self.t_active = k, now
            if k is not None:
                self.get_logger().info('muc tieu -> vong {} @ {}'.format(
                    k, np.round(self.gates[k][0], 2)))
            return
        if k is not None and now - self.t_active > GATE_TIMEOUT:
            self._finish(k, 'BO QUA sau {:.0f} s khong lot duoc'
                         .format(GATE_TIMEOUT))

    def mission_over(self, now):
        """Every known gate done, and the map has been quiet long enough that
        no more are coming.

        "Quiet" counts DETECTIONS, not just map growth. A gate first seen late
        in the run needs CONFIRM_HITS sightings before it can reach the map at
        all, so a 20 s timer watching only the MAP calls the course over while
        N1 is still looking straight at a gate that is about to appear on it --
        which is how a gate that had been seen once still got dropped (measured,
        two runs of four).

        END_MAX is the hard ceiling, because a gate ALREADY passed can sit in
        frame indefinitely and would otherwise hold the drone up until the
        battery ended the argument.
        """
        if not (bool(self.gates) and len(self.done) == len(self.gates)):
            self.t_all_done = 0.0
            return False
        if not self.t_all_done:
            self.t_all_done = now
        quiet = max(self.t_quiet, self.t_detect)
        return (now - quiet > END_QUIET
                or now - self.t_all_done > END_MAX)

    def xyz(self):
        p = self.pose.pose.position
        return np.array([p.x, p.y, p.z])

    def abort_reason(self, now, p):
        if self.t_pose and now - self.t_pose > POSE_TIMEOUT:
            return 'mat /mavros/local_position/pose > {:.0f} s'.format(
                POSE_TIMEOUT)
        if self.t_pose and outside(p):
            return 'ra khoi geofence tai {}'.format(np.round(p, 1))
        return None

    def land(self, why, mode='LAND'):
        """End the flight. `mode` is LAND (descend where we are) or RTL (fly
        home first) -- see finish() for why the normal ending uses RTL."""
        self.get_logger().warn('{}: {}'.format(mode, why))
        other = 'RTL' if mode == 'LAND' else 'LAND'
        if not self.set_mode(mode):
            self.get_logger().error('set {} that bai -- thu {}'.format(mode, other))
            self.set_mode(other)
        self.state = 'DONE'

    def finish(self, why):
        """The course is done -- go home rather than landing here.

        LAND descends in place, and "in place" at the end of a course is a
        stand-off past the LAST gate, which is directly above that gate's
        stand: a 0.35 m base plate with a mast on it. Measured over four runs
        the drone came down between 0.4 m and on top of it, and touching the
        plate tips it -- ArduCopter then reports `Crash: Disarming AngErr=83>30`
        and cuts the motors. It survived the one clean run by 8 cm.

        RTL flies back to the launch point, which is open ground the drone
        already took off from, and lands there. Aborts keep plain LAND: with a
        dead pose estimate or a geofence breach, flying the length of the course
        is worse than coming down where we are.
        """
        self.land(why, mode='RTL')

    def tick(self):
        now = time.time()
        if self.state != 'RUN':
            if self.fcu is not None and self.fcu.armed and self.t_pose:
                self.state = 'RUN'
                self.mode_seen = self.fcu.mode
                self.get_logger().info(
                    'RUN -- giam sat {} vong'.format(len(self.gates)))
            else:
                self.get_logger().info('INIT: doi MAVROS armed + pose',
                                       throttle_duration_sec=5.0)
            return
        # Never fight ArduPilot's own failsafe (par. 6.3): if the mode moved
        # without us asking, say so and stop supervising rather than yanking it
        # back into GUIDED underneath a failsafe that had a reason.
        if self.fcu.mode != self.mode_seen:
            self.get_logger().warn('mode doi {} -> {} (khong phai do N6) -- '
                                   'dung giam sat'.format(self.mode_seen,
                                                          self.fcu.mode))
            self.mode_seen = self.fcu.mode
            if self.fcu.mode not in ('GUIDED', 'LAND', 'RTL'):
                self.state = 'DONE'
                return
        p = self.xyz()
        why = self.abort_reason(now, p)
        if why is not None:
            self.land(why)
            return
        self.check_timeouts(now, p)
        if self.mission_over(now):
            self.finish('het vong: {}/{} xong'.format(len(self.done),
                                                      len(self.gates)))
            for k in sorted(self.done):
                self.get_logger().info('  vong {}: {}'.format(k, self.done[k]))
            # ...and say plainly if the course had gates this run never met. The
            # landing itself is unchanged: holding position until a gate turns up
            # would keep a drone airborne on a dead course until the battery
            # decided the matter. What was wrong was the VERDICT, not the RTL.
            missing = self.expect - len(self.gates)
            if self.expect and missing > 0:
                self.get_logger().error(
                    'THIEU {} vong: san co {}, drone chi tung thay {}.'
                    ' Day KHONG phai mot chuyen bay tron ven.'
                    .format(missing, self.expect, len(self.gates)))

    def run(self):
        self._publish()                      # latch an empty list for N4 at once
        period = 1.0 / TICK_HZ
        while rclpy.ok() and self.state != 'DONE':
            self.tick()
            end = time.time() + period
            while rclpy.ok() and time.time() < end:
                rclpy.spin_once(self, timeout_sec=period / 4.0)


def main():
    rclpy.init()
    node = MissionFSM()
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    try:
        rclpy.shutdown()
    except Exception:
        pass


def _selfcheck():
    """Geometry + bookkeeping, no ROS:  python3 mission_fsm.py --selfcheck"""
    C, n = np.array([5.0, 0.0, 3.5]), np.array([1.0, 0.0, 0.0])

    def fly(pts):
        out, prev = [], None
        for q in map(np.asarray, pts):
            s = float(n @ (q - C))
            if prev is not None:
                hit = plane_crossing(prev[0], prev[1], q, s, C, n)
                if hit is not None:
                    out.append(hit[1])
            prev = (q, s)
        return out

    assert fly([[4.5, 0, 3.5], [5.5, 0, 3.5]])[0] < 1e-9          # dead centre
    # passes clean through, drifts 2 m wide AFTER the plane: grading at the
    # sample calls this a miss, interpolation must still see it centred
    assert fly([[4.9, 0, 3.5], [5.1, .2, 3.5], [6, 2, 3.5]])[0] < 0.11
    # mirror image: wide at the plane, lines up only after -> must read as wide
    assert fly([[4.9, 2, 3.5], [5.1, 1.9, 3.5], [5.5, 0, 3.5]])[0] > R_IN
    assert fly([[3, 0, 3.5], [4, 0, 3.5], [4.9, 0, 3.5]]) == []   # never reaches
    assert len(fly([[4.5, 0, 3.5], [5.5, 0, 3.5], [4.5, 0, 3.5]])) == 2

    node = MissionFSM.__new__(MissionFSM)
    node.gates, node.prev, node.done = {}, {}, {}
    node.home = None
    node.active, node.t_active, node.t_quiet = None, 0.0, time.time()
    node.t_detect, node.t_all_done = 0.0, 0.0
    logs = []
    node._publish = lambda: None
    node.get_logger = lambda: type('L', (), {
        'info': lambda _s, m, **k: logs.append(m),
        'warn': lambda _s, m, **k: logs.append(m)})()

    # identity survives a reorder: a late confirmation shifts every index
    kb = node._key(np.array([10.0, 2.0, 4.0]))
    node.gates[kb] = (np.array([10.0, 2.0, 4.0]), n)
    assert node._key(np.array([10.02, 1.98, 4.01])) == kb
    assert node._key(np.array([5.0, 0.0, 3.5])) != kb

    # a clean crossing finishes the gate; a graze inside CLEAR_MIN does NOT --
    # that is the R3 behaviour N4's n.(p - C) > 0 got wrong
    node.gates = {0: (C, n)}
    node._on_pose(np.array([4.9, 0.6, 3.5]))
    node._on_pose(np.array([5.1, 0.6, 3.5]))            # miss 0.6, clear 0.15
    assert node.done == {}, node.done
    node._on_pose(np.array([4.9, 0.0, 3.5]))
    node._on_pose(np.array([5.1, 0.0, 3.5]))            # dead centre
    assert 0 in node.done and 'LOT' in node.done[0], node.done

    # a gate already done is not re-judged (and cannot be un-done)
    node._on_pose(np.array([4.9, 2.0, 3.5]))
    node._on_pose(np.array([5.1, 2.0, 3.5]))
    assert 'LOT' in node.done[0]

    # the timeout clock belongs to the gate being CHASED, not to map age: a far
    # gate that has been on the map since takeoff must not expire while the
    # drone is still working a nearer one
    node.gates[1] = (np.array([9.0, 0.0, 3.5]), n)
    node.gates[2] = (np.array([30.0, 0.0, 3.5]), n)
    here, t0 = np.array([8.0, 0, 3.5]), time.time()
    node.check_timeouts(t0, here)
    assert node.active == 1                       # nearest un-done, not gate 2
    node.check_timeouts(t0 + GATE_TIMEOUT + 1, here)
    assert 'BO QUA' in node.done[1] and 2 not in node.done, node.done
    # ... and the clock restarts for the gate that inherits the target slot
    node.check_timeouts(t0 + GATE_TIMEOUT + 1, here)
    assert node.active == 2 and 2 not in node.done
    node.check_timeouts(t0 + GATE_TIMEOUT + 2, here)
    assert 2 not in node.done, 'gate 2 inherited gate 1 elapsed time'
    node._finish(2, 'test')

    assert not node.mission_over(time.time())

    # ... and once every gate is done, the mission ends only after the map has
    # been quiet -- a gate confirming late must not land the drone early
    now = time.time()
    assert not node.mission_over(now)
    assert node.mission_over(now + END_QUIET + 1)

    # --- REGRESSION: a gate still being SEEN must hold the ending open.
    # Two runs of four ended with a gate that had been detected once and needed
    # CONFIRM_HITS more sightings; a quiet timer that only watches the MAP calls
    # the course over while the drone is still looking straight at the thing.
    node.t_all_done = 0.0
    node.t_detect = now + END_QUIET          # still seeing something out there
    assert not node.mission_over(now + END_QUIET + 1), \
        'dang nhin thay vong ma van ket thuc'
    # the ceiling runs from when the last KNOWN gate was done, which is the first
    # tick above -- not from `now`, or this asserts nothing.
    t_done = node.t_all_done
    # ...and it is a ceiling: a gate already passed can sit in frame for ever,
    # so END_MAX ends the flight whatever the detector is still reporting.
    node.t_detect = t_done + END_MAX + 10
    assert node.mission_over(t_done + END_MAX + 1), \
        'detection lien tuc giu drone bay qua tran END_MAX'
    node.t_detect, node.t_all_done = 0.0, 0.0

    assert outside([0, 0, 20]) and outside([100, 0, 3]) and not outside([1, 2, 3])

    # finishing the course goes HOME (the last gate's stand is right below us);
    # an abort comes down where it stands
    modes = []
    node.set_mode = lambda m: modes.append(m) or True
    node.finish('done')
    node.land('mat pose')
    assert modes == ['RTL', 'LAND'], modes
    print('mission_fsm selfcheck OK')


if __name__ == '__main__':
    import sys
    if '--selfcheck' in sys.argv:
        _selfcheck()
    else:
        main()
