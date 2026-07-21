#!/usr/bin/env python3
"""N5 -- turns N4's geometric path into a timed trajectory and streams it.

Plan v2 par. 5.6-5.9. This is the node that replaces v1's carrot-chasing.

    N4 (/planner/path)  answers "WHICH WAY"   -- geometry, obstacle-free
    N5 (this)           answers "HOW FAST"    -- p, v, a, jerk vs time

Each consecutive pair of waypoints becomes one quintic Mueller segment (see
mueller.py). The segments are stitched C2-continuous: every segment starts from
the previous one's exact end state, so there is no jerk step at a join.

Three things here are load-bearing and easy to break:

1. STREAM NEVER STOPS. ArduCopter stops the vehicle after ~3 s without a
   setpoint (plan par. 5.1). With no trajectory we publish the held position at
   v=0, a=0 -- we never simply stop publishing.

2. REPLAN USES THE COMMANDED STATE, NOT THE MEASURED ONE (par. 5.8). Seeding a
   new trajectory from the noisy measured pose feeds position noise back into
   the trajectory, which the controller then chases: a self-exciting loop. The
   measured state is for monitoring only -- and for re-seeding after a hold,
   where the commanded state has gone stale.

3. THE MASK IS 2048 = pos+vel+accel+yaw. Measured on this vehicle
   (ArduCopter V4.6.3, see hold_and_watch --probe-accel): feeding accel moves
   the drone ~1.0 m where dropping it moves 0.02 m. If a future firmware ignores
   accel, MASK_FALLBACK below is the pos+vel-only escape.

Run:  ros2 run simtofly_gates trajectory_follower
      python3 trajectory_follower.py --selfcheck
"""
import collections
import math
import sys
import time

import numpy as np

try:
    from simtofly_gates import mueller
    from simtofly_gates.mueller import (
        A_HORIZ_MAX, A_VERT_MAX, J_HORIZ_MAX, Trajectory, solve)
except ImportError:                       # running the file directly
    import mueller
    from mueller import (
        A_HORIZ_MAX, A_VERT_MAX, J_HORIZ_MAX, Trajectory, solve)

V_CRUISE = 0.8             # m/s. Same number as v_traverse: the plan's par. 5.5
                           # candidate search is deferred (it is deferred BECAUSE
                           # ArduCopter's shaper re-shapes underneath, so fine
                           # candidate selection is sanded off anyway).
TF_MIN = 0.5               # s; floor on a segment's duration
CORRIDOR_R = 0.5           # m; how far a smooth segment may bow off the straight
                           # line N4 proved clear (plan par. 5.6)
L_SEG_MAX = 1.5            # m; longest straight run allowed in one segment. Equal
                           # to N4's D_PRE/D_POST on purpose -- see _split_long.
STREAM_HZ = 20.0           # setpoint rate. >= 1 Hz is required; 20 is comfortable
TAKEOFF_ALT = 3.5          # m
TRACK_HOLD = 0.5           # m; CROSS-TRACK error above this -> hold and re-seed.
                           # Cross-track only, deliberately. The total distance to
                           # the SHAPE_LAG-old command is dominated by how far
                           # along the path the vehicle is, which is bookkeeping,
                           # not danger: measured in flight, along +-0.45..0.52 m
                           # with the sign FLIPPING (0.8 m/s x 0.7 s = 0.56 m, so
                           # in steady cruise the compensation overshoots and the
                           # drone reads as being AHEAD of its own command) while
                           # cross-track stayed at 0.05..0.29 m. Guarding on the
                           # total aborted good trajectories at 0.50-0.54 m in a
                           # livelock -- abort, hold 2 s, rebuild, abort -- that
                           # cost a whole gate to GATE_TIMEOUT on two runs.
                           # 0.5 m is also N4's CORRIDOR_R: past that the drone is
                           # outside the tube the planner proved clear, which is
                           # exactly when holding is the right answer.
                           # Falling BEHIND needs no threshold here: the drone is
                           # flying the same cleared path, just late, and N6's
                           # GATE_TIMEOUT is already the ceiling on late.
EMPTY_PATH_STUCK = 10      # consecutive unusable paths before this stops being a
                           # transient and starts being a stuck mission. N4
                           # republishes at ~2 Hz, so this is ~5 s of getting
                           # nowhere -- long enough that a single bad path during
                           # a replan does not trip it.
SHAPE_LAG = 0.7            # s the vehicle runs BEHIND the setpoint, by design.
                           # GUIDED does not jump to a position target, it shapes
                           # its way there under WPNAV_ACCEL / PSC_JERK_XY, so the
                           # measured position always trails the commanded one.
                           # Measured live: a dead-steady 0.50-0.53 m at v_cruise
                           # 0.8 m/s, on 10 aborts across one run -> 0.53/0.8 =
                           # 0.66 s. Comparing against the command from SHAPE_LAG
                           # ago instead of the current one is what stops a
                           # trailing drone reading as a 0.5 m error.
                           #
                           # It is an APPROXIMATION, and knowing by how much is
                           # the point: the real lag varies with flight phase, so
                           # in steady cruise 0.7 s overshoots and the along-track
                           # term comes out around +0.5 m -- the drone reads as
                           # ahead of its own past command. That is why TRACK_HOLD
                           # guards the cross-track term only; no single constant
                           # here can make the along-track term trustworthy.
                           # Retune per airframe: this is a property of the
                           # vehicle's controller, not of N5.
RETRY_HOLD = 2.0           # s to sit still after that before rebuilding the path.
                           # Long enough for ArduCopter to actually stop, so the
                           # rebuild starts from a state it can hold; retrying
                           # immediately just re-trips TRACK_HOLD on the next tick.
SETTLE_S = 10.0            # s; longest wait for the takeoff climb to stop. 5 s was
                           # not enough: ArduCopter creeps the last 0.1 m.
SETTLE_SPEED = 0.15        # m/s; below this the drone counts as parked, so the
                           # cmd_v = 0 seed is honest
SOLVE_TRIES = 6            # how many times solve() may stretch Tf
SIMPLIFY_TOL = 0.10        # m; how far the flown line may sit from N4's path.
                           # Half of the planner's inflation margin: it inflates
                           # obstacles by INFLATE_R 0.60 m around a ROBOT_RADIUS
                           # of 0.40 m, so 0.20 m of shortcut still leaves the
                           # body clear of anything measured. Taking half of that
                           # buys the smoothing and keeps half the margin.
MIN_SEG_LEN = 0.15         # m; a target nearer than this is not worth a segment.
                           # Roughly the distance covered while accelerating to
                           # v_cruise inside the accel limit -- ask for less and
                           # the quintic has to reverse out to make room.

# Opening sweep: climb a step, turn all the way round, look again. This is what
# finds the gates that are not in front of the drone at takeoff -- one BEHIND it,
# and one too high to be in frame. The camera is bolted to the airframe and sees
# 42.5 deg vertically, so a ring more than 1.95 m above the drone at 5 m range is
# simply not there. Climbing is the only way to look up.
#
# Sweeping is worth doing HERE, at the start, and nowhere else: every gate on
# this course is within ~6 m of the launch point, while a sweep after the run
# would be 20 m past everything it was meant to find.
SCAN_LEVELS = (0.0, 1.5, 3.0)   # m above TAKEOFF_ALT, one full turn at each
SCAN_YAW_RATE = 0.5             # rad/s; a turn takes 2*pi/rate = 12.6 s
SCAN_CLIMB_S = 4.0              # s allowed to reach each level before turning
# A CLOSING sweep -- one turn where the run ended, to catch a gate the drone flew
# past -- was tried here and removed on 2026-07-21. It found nothing (it stared
# straight at the missing gate for 60 s across a whole batch and never got it
# onto the map, because the gate was being dropped in N2's TF lookup, not missed
# by the camera), and it could eat an entire flight: its idle timer counted the
# time spent in the OPENING sweep, so on one run it fired the instant the opening
# sweep ended and held the follower for the rest of the mission -- planner
# publishing paths the whole time, drone parked, all three gates timed out.
# The real fix was gate_tracker._tf_for. Do not re-add this without one.

MASK_PVA = 2048            # pos+vel+accel+yaw (ignore yaw_rate only)
MASK_FALLBACK = 2496       # pos+vel+yaw -- if a firmware ever drops accel


def _unit(v):
    n = float(np.linalg.norm(v))
    return np.asarray(v, float) / n if n > 1e-9 else np.zeros(3)


def _point_to_seg(q, a, b):
    """Distance from q to the segment a-b -- the self-check's safety measure."""
    ab = np.asarray(b, float) - np.asarray(a, float)
    L2 = float(ab @ ab)
    if L2 < 1e-18:
        return float(np.linalg.norm(q - a))
    t = float(np.clip((np.asarray(q, float) - a) @ ab / L2, 0.0, 1.0))
    return float(np.linalg.norm(q - (a + t * ab)))


def simplify(pts, yaws, tol=SIMPLIFY_TOL):
    """Douglas-Peucker: drop waypoints no further than `tol` off the chord.

    Recursive, on the polyline pts[first..last]: keep the point furthest from the
    straight line between the ends, recurse on both halves, and drop everything
    that never exceeds `tol`. Yaws follow their points.
    """
    pts = [np.asarray(q, float) for q in pts]
    keep = [False] * len(pts)
    keep[0] = keep[-1] = True
    stack = [(0, len(pts) - 1)]
    while stack:
        lo, hi = stack.pop()
        if hi <= lo + 1:
            continue
        axis = pts[hi] - pts[lo]
        L = float(np.linalg.norm(axis))
        worst, worst_i = -1.0, lo
        for i in range(lo + 1, hi):
            rel = pts[i] - pts[lo]
            if L < 1e-9:
                d = float(np.linalg.norm(rel))
            else:
                along = float(rel @ axis) / L
                d = float(np.linalg.norm(rel - along * axis / L))
            if d > worst:
                worst, worst_i = d, i
        if worst > tol:
            keep[worst_i] = True
            stack.append((lo, worst_i))
            stack.append((worst_i, hi))
    idx = [i for i, k in enumerate(keep) if k]
    return [pts[i] for i in idx], [yaws[i] for i in idx]


def merge_collinear(points, yaws, tol=SIMPLIFY_TOL):
    """Simplify N4's path, then cap the length of any one straight run.

    N4 resamples every 0.5 m, which is right for a carrot chaser and WRONG for
    this node: pinning a velocity vector at each of those points forces the drone
    to trace every wiggle of the Dijkstra path, and doing that at v_cruise costs
    jerk. Measured before this existed, the segment entering P_pre demanded
    6.85 m/s^3 on the y axis against a 3.5 limit, so solve() stretched it to
    3.4 s for 0.5 m -- the drone would have crawled at 0.02 m/s right at the
    gate mouth.

    This used to drop only points within 1 degree of collinear, which was too
    timid: it left the grid's own quantisation in the path. Flown, the segments
    that failed to solve were 0.15-0.47 m long with 7-36 degree kinks -- 0.2 m
    voxels rounding a smooth curve, not obstacle shape. Peak jerk on a segment
    goes as roughly 6a/T, so a short segment is expensive NO MATTER what else is
    done to it: at 0.8 m/s a 0.15 m kink of 36 degrees is not solvable at any
    duration, because the minimum turn radius alone (v^2/a_lat = 0.37 m) needs
    0.23 m of arc. The fix has to be to stop asking, which means longer segments.

    Douglas-Peucker with a DISTANCE tolerance is what an angle test cannot do:
    it bounds how far the flown line may sit from N4's path, so the safety
    argument is a number rather than a hope. SIMPLIFY_TOL is half of N4's
    inflation margin -- see its comment for why that keeps the body clear.

    As a bonus this is what makes the traverse come out right: P_pre..P_post is
    perfectly collinear, so it collapses to ONE constant-velocity segment through
    the gate centre with zero acceleration -- par. 5.4's requirement, for free.
    """
    pts = [np.asarray(q, float) for q in points]
    if len(pts) < 3:
        return list(pts), list(yaws)
    pts, yaws = simplify(pts, list(yaws), tol)
    return _split_long(pts, yaws)


def _split_long(pts, yaws, l_max=None):
    """Cut any straight run longer than `l_max` into equal pieces.

    Merging alone over-corrects. The traverse P_pre..P_post is 3.0 m of perfectly
    straight line, so it collapses to ONE segment -- and being the last segment
    it has to end at v = 0, which spreads the deceleration across the whole run,
    THROUGH the gate. Measured: speed swung 0.00..1.21 m/s across the hole, which
    is precisely the a=0-at-the-centre condition par. 5.4 exists to guarantee.

    Splitting at l_max = D_PRE puts a waypoint exactly at the gate centre (N4 uses
    d_pre == d_post == 1.5 m), so the run becomes par. 5.4's segment A and segment
    B: cross the centre at v_cruise with zero acceleration, decelerate only
    AFTER. Nothing here knows what a gate is -- it just refuses to let one
    straight segment get long enough to hide a speed change inside it.
    """
    l_max = L_SEG_MAX if l_max is None else l_max
    out_p, out_y = [pts[0]], [yaws[0]]
    for i in range(len(pts) - 1):
        a, b = pts[i], pts[i + 1]
        d = float(np.linalg.norm(b - a))
        k = max(1, int(math.ceil(d / l_max - 1e-9)))
        for j in range(1, k + 1):
            out_p.append(a + (b - a) * (float(j) / k))
            out_y.append(yaws[i + 1])
    return out_p, out_y


def lag_error_vec(cmd_log, now, here):
    """Displacement (here - where the vehicle was told to be SHAPE_LAG ago).

    `cmd_log` is (time, commanded position) in time order. Returns None until the
    log reaches that far back -- there is nothing to compare against yet, and
    guessing would abort a takeoff that is going perfectly well. A vector, not a
    distance, because only its cross-track part means anything: see TRACK_HOLD.
    """
    cutoff = now - SHAPE_LAG
    was = None
    for t, p in cmd_log:
        if t > cutoff:
            break
        was = p
    if was is None:
        return None
    return np.asarray(here, float) - was


def split_error(vec, heading):
    """(along, cross) -- the error resolved along the direction of travel and
    perpendicular to it.

    These two mean opposite things, and one scalar distance conflates them.
    Trailing ALONG the path is what a shaped controller does by construction: it
    is late, not lost, and the gate does not care. Drifting CROSS-track is the
    one par. 6.3 is about -- with a 0.75 m ring hole, that is the number that
    decides whether the drone hits the rim.
    """
    h = _unit(heading)
    if not h.any():
        return 0.0, float(np.linalg.norm(vec))
    along = float(vec @ h)
    return along, float(np.linalg.norm(vec - along * h))


def _fail_str(fail):
    """Readable form of mueller.LAST_FAIL, for the truncation warning."""
    if not fail:
        return 'khong ro'
    what, T, a_ratio, j_ratio = fail
    out = '{} @ T={:.1f}s, a={:.0%} j={:.0%} gioi han'.format(
        what, T, a_ratio, j_ratio)
    ask = mueller.LAST_ASK
    if ask:
        out += ' | doan {}/{}: d={:.2f}m, re {:.0f}deg, |v0|={:.2f}, |a0|={:.2f}'.format(
            ask[4], ask[5], ask[0], ask[1], ask[2], ask[3])
    return out


LAST_CORNER_V = []         # corner speed as a fraction of v_cruise, one per
                           # interior waypoint of the last build_segments().
                           # Reported so a course that has quietly become a crawl
                           # says WHICH corner is doing it.


def build_segments(points, yaws, p0, v0, a0,
                   v_cruise=V_CRUISE, corridor_r=CORRIDOR_R):
    """Chain of (Trajectory, yaw) from the current state through `points`.

    `points[0]` is where the path starts; the drone is at `p0`, which is close
    but not identical, so the first segment flies p0 -> points[1] rather than
    pretending the drone is already on the path.

    Target velocity at each interior waypoint looks ONE waypoint ahead
    (par. 5.6): aiming at the direction of W[k+2] - W[k] rather than stopping at
    every corner is what keeps the corner smooth. The final waypoint gets v = 0.

    Collinear runs come out constant-velocity for free, which is exactly the
    par. 5.4 requirement for the P_pre -> P_gate -> P_post traverse: with
    Tf = d / v_cruise and the target velocity along the same line, the quintic
    reduces to a straight constant-speed run with zero acceleration. Nothing
    special-cases the traverse -- it falls out, PROVIDED Tf is not clamped by
    TF_MIN, which is why the assert below guards that.
    """
    del LAST_CORNER_V[:]
    pts, yaws = merge_collinear(points, yaws)
    # Drop the part of the path the drone has already flown. A path is latched
    # (TRANSIENT_LOCAL) and N4 only republishes on a replan, so by the time this
    # runs the drone is usually somewhere in the middle of it -- and on a restart
    # it can be at the very end. Feeding it waypoints that are behind it asks for
    # a reversal, which is unbounded jerk, so every segment failed and the drone
    # sat at the gate mouth holding position (observed live).
    p0 = np.asarray(p0, float)
    if len(pts) > 2:
        start = int(np.argmin([float(np.linalg.norm(q - p0)) for q in pts]))
        # keep the point we are closest to only if it is still AHEAD of us, i.e.
        # not the one we just flew through
        if start < len(pts) - 1:
            ahead = pts[start + 1] - pts[start]
            if float((p0 - pts[start]) @ ahead) > 0.0:
                start += 1
        pts, yaws = pts[start:], yaws[start:]
    # ...and drop any waypoint the drone is effectively already standing on.
    #
    # This is not cosmetic. N4 plans FROM the current pose, so the path's first
    # point is the drone itself, and resampling can leave a second point a few
    # millimetres away. Targeting that point asks for a segment that travels 4 mm
    # while ending at v_cruise -- measured: 0.0042 m, needing a 6.40 m/s^2 and
    # 117 m/s^3 against limits of 1.75 and 3.50. Stretching Tf does not rescue it
    # either (2.68 s still wants 4.02 m/s^3), because the quintic has to reverse
    # out and run back up to speed inside 4 mm. solve() returned None, the chain
    # came out EMPTY, and the drone sat at the gate mouth holding position.
    while len(pts) > 2 and float(np.linalg.norm(pts[1] - p0)) < MIN_SEG_LEN:
        pts, yaws = pts[1:], yaws[1:]
    if len(pts) < 2 or float(np.linalg.norm(pts[-1] - p0)) < MIN_SEG_LEN:
        return [], True                   # nothing left to fly -- already there
    segs = []
    state_p = p0
    state_v = np.asarray(v0, float)
    state_a = np.asarray(a0, float)
    for k in range(len(pts) - 1):
        target = pts[k + 1]
        d = float(np.linalg.norm(target - state_p))
        if d < 1e-6:
            continue
        Tf = max(d / v_cruise, TF_MIN)
        if k + 2 < len(pts):
            # ARRIVE at a corner at a speed the corner can take.
            #
            # This used to be v_cruise flat, whatever the corner looked like, and
            # a path that doubles back -- which is every path to a gate BEHIND
            # the drone -- has corners near 180 deg. Flown: a 0.17 m segment with
            # a 52 deg kink entered at 0.80 m/s was refused at EVERY duration
            # solve() tried (a=88% j=170% at T=2.7 s), the chain came back empty,
            # ten paths in a row produced nothing, and the drone hovered 17 s
            # until N6 timed the course out. That run scored 4/4 only because
            # every gate was already behind it; one gate still open and it fails.
            #
            # Stretching Tf cannot rescue it -- 0.80 m/s for 2.7 s is 2.16 m of
            # travel with 0.17 m to spend it in, so the quintic must decelerate,
            # reverse, and run back up, which costs MORE jerk than the short
            # version. mueller.solve()'s "stretching ALWAYS helps" is false in
            # exactly this regime, and stretching is the only knob it has.
            #
            # Measured, sweeping this waypoint's speed alone does nothing (jerk
            # 1.13 -> 1.04 from full cruise down to 20%): the segment is
            # over-constrained at BOTH ends. What works is that this vector is
            # the exit of segment k and the ENTRY of segment k+1 at once, so
            # capping it slows both sides of the corner together.
            #
            # cos^2(theta/2): full cruise on a straight run, zero at a reversal.
            # Measured over 36 hairpin variants (corner tightness x approach
            # offset x entry speed 0.6-1.0 m/s): 0 failures, worst peak jerk 96%
            # of limit. cos(theta/2) alone still failed 7/7 at this tolerance.
            #
            # The geometry never moves, so N4's corridor clearance argument holds
            # exactly -- which merging the corner away would have spent.
            a_in = _unit(pts[k + 1] - pts[k])
            a_out = _unit(pts[k + 2] - pts[k + 1])
            turn = math.acos(float(np.clip(a_in @ a_out, -1.0, 1.0)))
            corner = math.cos(turn / 2.0) ** 2
            LAST_CORNER_V.append(corner)
            v_target = v_cruise * corner * _unit(pts[k + 2] - pts[k])
        else:
            v_target = np.zeros(3)        # the path ends stopped
        traj, _ = solve(state_p, state_v, state_a, target, v_target, np.zeros(3),
                        Tf, corridor=(state_p, target), radius=corridor_r,
                        tries=SOLVE_TRIES)
        if traj is None:
            # what the rejected segment was actually being ASKED for -- a length,
            # a turn, and an entry speed. The constraint that failed is only half
            # the story; this is the other half.
            turn = 0.0
            if float(np.linalg.norm(state_v)) > 1e-6:
                turn = math.degrees(math.acos(float(np.clip(
                    _unit(target - state_p) @ _unit(state_v), -1.0, 1.0))))
            mueller.LAST_ASK = (d, turn, float(np.linalg.norm(state_v)),
                                float(np.linalg.norm(state_a)), k, len(pts) - 1)
            return segs, False            # partial chain; caller decides
        segs.append((traj, float(yaws[k + 1]) if k + 1 < len(yaws) else 0.0))
        state_p = traj.position(traj.T)
        state_v = traj.velocity(traj.T)
        state_a = traj.acceleration(traj.T)
    # `complete` is against the MERGED point list, not the caller's raw path --
    # merging is expected to drop most of N4's 0.5 m filler, and comparing
    # against the raw count logs a false "gave up early" on every single plan.
    return segs, True


def _build_node():
    import rclpy
    from geometry_msgs.msg import TwistStamped
    from mavros_msgs.msg import PositionTarget
    from nav_msgs.msg import Path
    from rclpy.qos import (QoSDurabilityPolicy, QoSHistoryPolicy,
                           QoSProfile, QoSReliabilityPolicy)

    try:
        from simtofly_mavros_sitl.guided_base import GuidedNode, mavros_qos
    except ImportError:
        from guided_base import GuidedNode, mavros_qos

    class TrajectoryFollower(GuidedNode):
        def __init__(self):
            super().__init__('trajectory_follower')
            self.sp_pub = self.create_publisher(
                PositionTarget, '/mavros/setpoint_raw/local', 10)
            # N4 publishes the path TRANSIENT_LOCAL because a path is STATE, not
            # a stream -- match it or a late subscriber sees nothing forever.
            self.create_subscription(
                Path, '/planner/path', self.on_path,
                QoSProfile(durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
                           reliability=QoSReliabilityPolicy.RELIABLE,
                           history=QoSHistoryPolicy.KEEP_LAST, depth=1))
            self.create_subscription(
                TwistStamped, '/mavros/local_position/velocity_local',
                self._on_vel, mavros_qos())
            self.vel = np.zeros(3)
            self.hold = None              # position held when no trajectory
            self.hold_yaw = 0.0
            self.segs = []                # queue of (Trajectory, yaw)
            self.active = None
            self.active_yaw = 0.0
            self.t_start = 0.0
            # the COMMANDED state -- what we last told the vehicle to be doing.
            # This, never the measured state, seeds a replan (par. 5.8).
            self.cmd_p = None
            self.cmd_v = np.zeros(3)
            self.cmd_a = np.zeros(3)
            self.pending = None
            self.last_path = None      # kept so a truncated chain can be retried
            self.truncated = False
            self.n_replan = 0
            self.scan = '--no-scan' not in sys.argv
            self.n_empty = 0        # consecutive paths that built no segments
            self.retry_at = 0.0         # earliest rebuild after a tracking abort
            # (time, commanded position) for the last couple of seconds, so the
            # tracking check can ask "where did we tell it to be SHAPE_LAG ago?"
            self.cmd_log = collections.deque(maxlen=int(4.0 * STREAM_HZ))

        def _on_vel(self, msg):
            v = msg.twist.linear
            self.vel = np.array([v.x, v.y, v.z])

        def _xyz(self):
            p = self.pose.pose.position
            return np.array([p.x, p.y, p.z])

        def on_path(self, msg):
            if not msg.poses:
                return
            pts = np.array([[q.pose.position.x, q.pose.position.y,
                             q.pose.position.z] for q in msg.poses])
            yaws = np.array([2.0 * math.atan2(q.pose.orientation.z,
                                              q.pose.orientation.w)
                             for q in msg.poses])
            self.pending = (pts, yaws)

        def _adopt_pending(self):
            """Rebuild the segment queue from the COMMANDED state (par. 5.8)."""
            pts, yaws = self.pending
            self.pending = None
            self.last_path = (pts, yaws)
            segs, complete = build_segments(pts, yaws, self.cmd_p, self.cmd_v,
                                            self.cmd_a)
            if not segs:
                # Holding position is the right thing to DO here, but repeating
                # it quietly is not: this fired 95 times over 190 s while the
                # planner republished a path the drone had already flown past,
                # and the run looked like a drone calmly holding station rather
                # than a mission that had stopped dead. The state is only
                # recoverable from outside (N4 replans, or N6 times the gate
                # out), so say so once it is clearly not a transient.
                self.n_empty += 1
                if self.n_empty == EMPTY_PATH_STUCK:
                    self.get_logger().error(
                        'KET: {} path lien tiep khong sinh duoc doan nao -- '
                        'drone dang treo, cho N4 replan hoac N6 het gio'
                        .format(self.n_empty))
                else:
                    self.get_logger().warn(
                        'path moi khong sinh duoc doan nao -- giu vi tri',
                        throttle_duration_sec=2.0)
                return
            self.n_empty = 0
            n_seg = len(segs)
            total = sum(t.T for t, _ in segs)
            a_peak = max(t.peak_accel().max() for t, _ in segs)
            end = segs[-1][0].position(segs[-1][0].T)
            self.segs = segs
            self.active, self.active_yaw = self.segs.pop(0)
            self.t_start = time.time()
            self.n_replan += 1
            self.truncated = not complete
            if not complete:
                why = mueller.LAST_FAIL
                self.get_logger().warn(
                    'duong bi cat ngan: dung o {} vi doan ke tiep khong kha thi'
                    ' [{}]'.format(np.round(end, 2), _fail_str(why)))
            slowest = min(LAST_CORNER_V) if LAST_CORNER_V else 1.0
            self.get_logger().info(
                'quy dao moi #{}: {} doan, {:.1f} s, a_dinh {:.2f} m/s2,'
                ' goc cua cham nhat {:.0%} v_cruise, ket tai {}'
                .format(self.n_replan, n_seg, total, a_peak, slowest,
                        np.round(end, 2)))

        def stream(self):
            """One 20 Hz tick. MUST publish something every time."""
            if self.cmd_p is None:
                return                    # not airborne yet
            # Stop the moment somebody else owns the vehicle (par. 6.3: never
            # fight a failsafe). N6 lands by setting mode LAND; ArduCopter then
            # ignores setpoints, but this node does not know that and carried on
            # -- observed live, it built and logged a fresh 7-segment trajectory
            # while the drone was descending under LAND. Harmless there, but it
            # is the same code path that would fight a real failsafe, and the log
            # it writes makes a landing look like a flight.
            if self.state.mode and self.state.mode != 'GUIDED':
                self.get_logger().warn(
                    'mode = {} (khong phai GUIDED) -- ngung phat setpoint'
                    .format(self.state.mode), throttle_duration_sec=5.0)
                self.active, self.segs, self.pending = None, [], None
                return
            # A big tracking error means the vehicle is not where we think we
            # commanded it. Re-seeding from the stale commanded state would keep
            # widening the gap, so drop the trajectory, hold, and re-seed from
            # the MEASURED state -- the one case par. 5.9 allows that.
            vec = lag_error_vec(self.cmd_log, time.time(), self._xyz())
            # cmd_v = 0 (holding) makes the whole error cross-track, which is
            # what we want there: a drone that drifts while told to stand still
            # IS diverging.
            along, cross = split_error(vec, self.cmd_v) if vec is not None else (0.0, 0.0)
            if vec is not None and cross > TRACK_HOLD:
                if self.active is not None:
                    self.get_logger().warn(
                        'lech ngang {:.2f} m > {:.2f} (doc {:+.2f}) -- bo quy '
                        'dao, giu vi tri'.format(cross, TRACK_HOLD, along))
                self.active, self.segs = None, []
                self.cmd_p = self._xyz()
                self.cmd_v = np.zeros(3)
                self.cmd_a = np.zeros(3)
                self.hold = self.cmd_p.copy()
                # Forget the commands from the trajectory just abandoned, or the
                # very next tick compares the drone against one of them and trips
                # again -- the abort would never clear.
                self.cmd_log.clear()
                # ...and come back to the path once the vehicle has stopped.
                # Without this the abort is TERMINAL: pending was consumed when
                # the trajectory was adopted, and N4 does not republish because
                # from its side nothing changed -- the gate has not moved and the
                # path is not blocked, so needs_replan() says None forever. The
                # drone then holds at the abort point until something else ends
                # the mission. Measured live: N6 sat through the whole 90 s gate
                # timeout watching a drone that had stopped 2.5 m short of the
                # ring. Retrying is safe because it rebuilds from the MEASURED
                # state, and if the vehicle genuinely cannot track the path the
                # retry just repeats -- N6's GATE_TIMEOUT is the ceiling on that.
                self.retry_at = time.time() + RETRY_HOLD

            # Nothing to fly, but a path we never finished: rebuild it from here.
            if (self.active is None and self.pending is None
                    and self.last_path is not None
                    and time.time() >= self.retry_at):
                self.pending = self.last_path

            if self.pending is not None:
                self._adopt_pending()     # also mid-flight: C2 from commanded

            if self.active is None:
                self._publish(self.hold, np.zeros(3), np.zeros(3), self.hold_yaw)
                return

            tau = time.time() - self.t_start
            if tau > self.active.T:
                if self.segs:
                    self.active, self.active_yaw = self.segs.pop(0)
                    self.t_start = time.time()
                    tau = 0.0
                else:
                    end = self.active.position(self.active.T)
                    self.hold, self.hold_yaw = end, self.active_yaw
                    self.active = None
                    # If the chain was cut short, RETRY the rest from here rather
                    # than holding forever. A segment is usually rejected because
                    # of the state we would ARRIVE in -- carrying cruise speed into
                    # a turn the next leg cannot absorb -- and arriving stopped
                    # changes that: replayed from the truncation point, the very
                    # same remainder solves. Without this the drone flies to the
                    # cut and parks there, because N4 sees a valid path and never
                    # republishes, so nothing ever wakes N5 up again.
                    if self.truncated and self.last_path is not None:
                        self.get_logger().info(
                            'duong tung bi cat -- thu lai phan con lai tu {}'
                            .format(np.round(end, 2)))
                        self.pending = self.last_path
                        self.truncated = False
                    else:
                        self.get_logger().info(
                            'het quy dao -- giu diem cuoi {}'.format(
                                np.round(end, 2)))
                    self._publish(end, np.zeros(3), np.zeros(3), self.hold_yaw)
                    return
            p, v, a = self.active.state_at(tau)
            self._publish(p, v, a, self.active_yaw)

        def _publish(self, p, v, a, yaw):
            msg = PositionTarget()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = 'map'
            msg.coordinate_frame = PositionTarget.FRAME_LOCAL_NED
            msg.type_mask = MASK_PVA
            msg.position.x, msg.position.y, msg.position.z = (float(q) for q in p)
            msg.velocity.x, msg.velocity.y, msg.velocity.z = (float(q) for q in v)
            (msg.acceleration_or_force.x, msg.acceleration_or_force.y,
             msg.acceleration_or_force.z) = (float(q) for q in a)
            msg.yaw = float(yaw)
            self.sp_pub.publish(msg)
            self.cmd_p, self.cmd_v, self.cmd_a = (np.asarray(p, float).copy(),
                                                  np.asarray(v, float).copy(),
                                                  np.asarray(a, float).copy())
            self.cmd_log.append((time.time(), self.cmd_p))

        def _turn_circle(self, goal, yaw):
            """One full turn on the spot at `goal`, walking the yaw setpoint."""
            turned, t_prev, said = 0.0, time.time(), 0
            while rclpy.ok() and turned < 2.0 * math.pi:
                now = time.time()
                step = SCAN_YAW_RATE * (now - t_prev)
                t_prev = now
                turned += step
                yaw += step
                self._publish(goal, np.zeros(3), np.zeros(3),
                              math.atan2(math.sin(yaw), math.cos(yaw)))
                rclpy.spin_once(self, timeout_sec=1.0 / STREAM_HZ)
                # does the vehicle actually FOLLOW a yaw setpoint? Commanded
                # vs measured, every quarter turn.
                if turned > said * math.pi / 2:
                    said += 1
                    self.get_logger().info(
                        '  quay: ra lenh {:+.0f} deg, do duoc {:+.0f} deg'
                        .format(math.degrees(math.atan2(math.sin(yaw),
                                                        math.cos(yaw))),
                                math.degrees(self.heading_enu())))

        def opening_sweep(self):
            """Climb through SCAN_LEVELS, turning a full circle at each.

            Position setpoints only -- the yaw field of the same PositionTarget
            we already stream, walked round at SCAN_YAW_RATE. Nothing new talks
            to the FCU, and the 20 Hz stream never stops (par. 5.1), so this is
            just a different thing to say at the usual rate.

            N1/N2 are running the whole time; the sweep does not look at what
            they find, it just gives them the chance. Deciding whether enough
            gates are known is N6's job, not this node's.
            """
            base = self._xyz()
            for lvl, dz in enumerate(SCAN_LEVELS):
                goal = np.array([base[0], base[1], base[2] + dz])
                yaw = self.heading_enu()
                # climb first, holding the current heading
                end = time.time() + SCAN_CLIMB_S
                while rclpy.ok() and time.time() < end:
                    self._publish(goal, np.zeros(3), np.zeros(3), yaw)
                    rclpy.spin_once(self, timeout_sec=1.0 / STREAM_HZ)
                    if abs(self._xyz()[2] - goal[2]) < 0.2:
                        break
                self._turn_circle(goal, yaw)
                self.get_logger().info(
                    'quet tang {}/{} xong o z={:.1f} m'.format(
                        lvl + 1, len(SCAN_LEVELS), goal[2]))
            # Come back down to where the sweep started. Climbing was for
            # LOOKING; leaving the drone parked at the top level is a different
            # thing entirely, and it broke the ordinary course: 3.2 m above the
            # first gate's approach, build_segments could not make a single
            # feasible segment, N5 held, and the gate died on its 90 s timeout.
            yaw = self.heading_enu()
            end = time.time() + SCAN_CLIMB_S * len(SCAN_LEVELS)
            while rclpy.ok() and time.time() < end:
                self._publish(base, np.zeros(3), np.zeros(3), yaw)
                rclpy.spin_once(self, timeout_sec=1.0 / STREAM_HZ)
                if abs(self._xyz()[2] - base[2]) < 0.2:
                    break
            self.get_logger().info('quet xong, ve lai z={:.1f} m'.format(base[2]))
            self.hold = self._xyz()
            self.hold_yaw = self.heading_enu()
            self.cmd_p = self.hold.copy()
            self.cmd_v = np.zeros(3)
            self.cmd_a = np.zeros(3)
            self.cmd_log.clear()

        def run(self):
            if not self.wait_for_services():
                self.get_logger().error('MAVROS services khong len'); return
            if not self.wait_for_connection():
                self.get_logger().error('FCU khong connect'); return
            if not self.set_mode('GUIDED'):
                self.get_logger().error('set GUIDED that bai'); return
            # Take over an aircraft that is ALREADY flying instead of demanding a
            # takeoff. Restarting this node mid-flight is normal -- it is how the
            # stage-6 repeat trials work, and how a code fix gets reloaded without
            # landing -- and ArduCopter rejects NAV_TAKEOFF while airborne, so the
            # old unconditional takeoff() turned every restart into a dead node
            # sitting next to a hovering drone.
            if self.state.armed and self.altitude > 1.0:
                self.get_logger().info(
                    'drone dang bay o {:.2f} m -- tiep quan, khong takeoff lai'
                    .format(self.altitude))
            else:
                if not self.arm_with_retry():
                    self.get_logger().error('arm that bai'); return
                if not self.takeoff(TAKEOFF_ALT):
                    self.get_logger().error('takeoff bi tu choi'); return
                if not self.wait_for_altitude(TAKEOFF_ALT, tol=0.3):
                    self.get_logger().warn('khong dat do cao, van tiep tuc')
            # Let the takeoff SETTLE before taking command. wait_for_altitude
            # returns as soon as it is within tol -- the drone is still climbing
            # at that moment -- so seeding (cmd_p = here, cmd_v = 0) hands the
            # trajectory a state the vehicle is already leaving. Measured: it
            # kept climbing the last 0.3 m while the first trajectory pulled
            # away sideways, and the two errors added to trip TRACK_HOLD within
            # two seconds, every run.
            # Bound this by the CLOCK, not by an iteration count: spin_once
            # returns immediately whenever a message is already queued, so a
            # 200-iteration loop at 20 Hz burned through in milliseconds and
            # waited for nothing.
            deadline, settled = time.time() + SETTLE_S, False
            while time.time() < deadline:
                rclpy.spin_once(self, timeout_sec=1.0 / STREAM_HZ)
                if float(np.linalg.norm(self.vel)) < SETTLE_SPEED:
                    settled = True
                    break
            if not settled:
                self.get_logger().warn('takeoff chua on dinh sau {:.0f} s, van '
                                       'tiep quan'.format(SETTLE_S))
            # Seed from where the drone MEASURABLY is, not from TAKEOFF_ALT:
            # wait_for_altitude only promises 0.3 m, so forcing the commanded z
            # to the nominal altitude puts a tracking error on the books before
            # the first setpoint even goes out.
            self.hold = self._xyz()
            self.cmd_p = self.hold.copy()
            if self.scan:
                self.get_logger().info('quet mo dau: {} tang, moi tang mot vong '
                                       '360 do'.format(len(SCAN_LEVELS)))
                self.opening_sweep()
            self.get_logger().info(
                'N5 san sang tai {} -- doi /planner/path'.format(
                    np.round(self.hold, 2)))

            period = 1.0 / STREAM_HZ
            while rclpy.ok():
                self.stream()
                end = time.time() + period
                while rclpy.ok() and time.time() < end:
                    rclpy.spin_once(self, timeout_sec=period / 4.0)

    return rclpy, TrajectoryFollower


def _selfcheck():
    # a path shaped like a real plan: an approach leg, then the straight
    # P_pre -> P_gate -> P_post traverse resampled at 0.5 m
    n = np.array([1.0, 0.0, 0.0])
    C = np.array([5.0, 0.0, 3.5])

    def _rs(a, b, step=0.5):
        a, b = np.array(a, float), np.array(b, float)
        k = max(1, int(round(float(np.linalg.norm(b - a)) / step)))
        return [a + (b - a) * i / k for i in range(k)]

    # what N4 actually publishes: shortcut, then resampled every 0.5 m, ending
    # with the straight P_pre -> P_gate -> P_post traverse
    pts = np.array(_rs([0, 0, 3.5], [2.0, 1.0, 3.5]) + _rs([2.0, 1.0, 3.5], [3.5, 0, 3.5])
                   + [list(C + t * n) for t in np.arange(-1.5, 1.51, 0.5)])
    yaws = np.zeros(len(pts))

    segs, complete = build_segments(pts, yaws, pts[0], np.zeros(3), np.zeros(3))
    assert segs, 'khong sinh duoc doan nao'
    assert complete, 'duong bi cat ngan -- co doan khong kha thi'

    # --- every segment respects the ArduCopter-derived limits ---
    for i, (t, _) in enumerate(segs):
        assert t.input_feasible(), 'doan {} khong kha thi'.format(i)
        assert t.peak_accel()[0] <= A_HORIZ_MAX + 1e-9
        assert t.peak_accel()[2] <= A_VERT_MAX + 1e-9
        assert t.peak_jerk()[0] <= J_HORIZ_MAX + 1e-9

    # --- C2 continuity at every join: this is the whole point of stitching from
    # the previous segment's END state rather than from the waypoint
    for i in range(len(segs) - 1):
        a, b = segs[i][0], segs[i + 1][0]
        assert np.allclose(a.position(a.T), b.position(0.0), atol=1e-9), i
        assert np.allclose(a.velocity(a.T), b.velocity(0.0), atol=1e-9), i
        assert np.allclose(a.acceleration(a.T), b.acceleration(0.0), atol=1e-9), i

    # --- THE one that matters: the gate centre is crossed at cruise speed with
    # ZERO acceleration (par. 5.4). Everything else in this node is in service of
    # this: the margin through the hole is 50 mm, so a speed swing or a lateral
    # accel there is the difference between flying through and clipping the rim.
    crossings = []
    for t, _ in segs:
        xs = np.array([t.position(x)[0] for x in np.linspace(0, t.T, 400)])
        if xs.min() <= C[0] <= xs.max():
            k = int(np.argmin(np.abs(xs - C[0])))
            tt = np.linspace(0, t.T, 400)[k]
            crossings.append((t.velocity(tt), t.acceleration(tt)))
    assert crossings, 'khong doan nao di qua tam vong?'
    for v, a in crossings:
        assert np.allclose(v, V_CRUISE * n, atol=1e-3), \
            'qua tam vong voi v={} (can {})'.format(np.round(v, 3), V_CRUISE * n)
        assert np.linalg.norm(a) < 1e-6, \
            'qua tam vong voi a={} -- phai bang 0'.format(np.round(a, 4))

    # --- the deceleration happens AFTER the gate, not through it. Without
    # _split_long the whole 3 m traverse was one segment ending at v=0 and the
    # speed swung 0.00..1.21 m/s across the hole (measured).
    last = segs[-1][0]
    assert np.allclose(last.velocity(last.T), 0.0, atol=1e-9), 'duong phai ket thuc dung'
    assert last.position(0.0)[0] >= C[0] - 1e-6, \
        'doan giam toc bat dau TRUOC tam vong -- _split_long hong?'

    # --- merging keeps the endpoints and actually removes the 0.5 m filler ---
    merged, _ = merge_collinear(pts, yaws)
    assert len(merged) < len(pts), 'gop khong bo bot diem nao'
    assert np.allclose(merged[0], pts[0]) and np.allclose(merged[-1], pts[-1])
    assert all(float(np.linalg.norm(merged[i + 1] - merged[i])) <= L_SEG_MAX + 1e-6
               for i in range(len(merged) - 1)), 'con doan dai hon L_SEG_MAX'

    # --- replanning mid-flight starts from the COMMANDED state, so the new
    # chain is C2-continuous with the old one (par. 5.8 rule 1)
    mid = segs[2][0]
    tau = mid.T * 0.4
    cp, cv, ca = mid.state_at(tau)
    new, _ = build_segments(pts[4:], yaws[4:], cp, cv, ca)
    assert new, 'replan giua chung khong sinh duoc doan nao'
    assert np.allclose(new[0][0].position(0.0), cp, atol=1e-9)
    assert np.allclose(new[0][0].velocity(0.0), cv, atol=1e-9)
    assert np.allclose(new[0][0].acceleration(0.0), ca, atol=1e-9), \
        'replan phai noi lien C2 -- day la ly do dung trang thai LENH'

    # --- a path whose first half is already behind the drone must still fly the
    # REST of it, not fail. This is the normal case: the path is latched, so N5
    # re-reads it (or restarts against it) with the drone partway along.
    late, ok = build_segments(pts, yaws, pts[len(pts) // 2], np.zeros(3), np.zeros(3))
    assert late and ok, 'path da di mot nua ma khong bay tiep duoc'
    assert np.allclose(late[-1][0].position(late[-1][0].T), pts[-1], atol=1e-6), \
        'phai van ket thuc o cuoi duong'
    # ...and standing AT the end yields nothing to do rather than an error
    done, ok = build_segments(pts, yaws, pts[-1], np.zeros(3), np.zeros(3))
    assert done == [] and ok, 'dung o cuoi duong thi phai la khong con gi de bay'

    # --- REGRESSION: a path whose first waypoint is the drone's own position.
    # N4 plans FROM the current pose, so this is the NORMAL shape of every path,
    # and resampling can leave a second point millimetres away. Before MIN_SEG_LEN
    # that 4 mm hop had to end at v_cruise, which needs 117 m/s^3 (limit 3.5);
    # solve() gave up, the chain came out empty and the drone parked at the gate
    # mouth. This asserts the whole traverse survives -- and comes out as par.
    # 5.4's two segments, crossing the centre at cruise with zero accel.
    here = np.array([3.5041, -0.5009, 3.50])
    Cg = np.array([5.0, -0.5, 3.5])
    tight = np.array(_rs(here, Cg - 1.5 * n) + _rs(Cg - 1.5 * n, Cg)
                     + _rs(Cg, Cg + 1.5 * n) + [Cg + 1.5 * n])
    tseg, tok = build_segments(tight, np.zeros(len(tight)), here,
                               np.zeros(3), np.zeros(3))
    assert tseg and tok, 'waypoint dau trung vi tri drone lam hong ca chuoi'
    hit = None
    for t, _ in tseg:
        xs = np.array([t.position(x)[0] for x in np.linspace(0, t.T, 600)])
        if xs.min() <= Cg[0] <= xs.max():
            k = int(np.argmin(np.abs(xs - Cg[0])))
            hit = t.velocity(np.linspace(0, t.T, 600)[k]), \
                t.acceleration(np.linspace(0, t.T, 600)[k])
    assert hit is not None, 'khong doan nao qua tam vong'
    assert np.allclose(hit[0], V_CRUISE * n, atol=1e-3), hit[0]

    # --- REGRESSION: a path that DOUBLES BACK, which is every path to a gate
    # behind the drone. Dijkstra rounds the hairpin on 0.2 m voxels and simplify()
    # keeps both apex corners, leaving a 0.17 m segment with a 52 deg kink entered
    # at cruise. With a flat v_cruise corner target that segment is infeasible at
    # every duration solve() tries, build_segments returned a one-segment chain,
    # and the drone hovered until N6 timed the course out (flown 2026-07-20).
    hp = [np.array([0.0, 0.0, 3.5]), np.array([0.6, 0.0, 3.5]),
          np.array([1.0, 0.0, 3.5]), np.array([1.2, 0.06, 3.5]),
          np.array([1.3, 0.2, 3.5]), np.array([1.3, 0.4, 3.5]),
          np.array([1.2, 0.54, 3.5]), np.array([1.0, 0.6, 3.5]),
          np.array([0.4, 0.6, 3.5]), np.array([-0.6, 0.6, 3.5])]
    hseg, hok = build_segments(hp, [0.0] * len(hp), hp[0],
                               V_CRUISE * np.array([1.0, 0.0, 0.0]), np.zeros(3))
    assert hok and len(hseg) > 1, \
        'duong quay dau lai lam chuoi gay: {} doan, complete={}'.format(
            len(hseg), hok)
    # and the corner really is being slowed -- if this ever reads 100% the cap
    # has been bypassed and the assert above is passing for some other reason
    assert min(LAST_CORNER_V) < 0.85, LAST_CORNER_V
    # Par. 5.4 asks for no push OFF the axis while crossing; a whisper of
    # along-axis accel is not that. This used to be one |a| < 1e-4 test, which
    # conflated the two, and simplify() moved the along-axis term to 3.5e-3
    # (0.2% of A_HORIZ_MAX, 0.4 mm of speed change over the crossing) by folding
    # the traverse into fewer, longer segments. Measured lateral: 1e-6, and the
    # flown line still passes 0.45 mm from the centre.
    along = float(hit[1] @ n)
    assert abs(along) < 1e-2, along
    assert np.linalg.norm(hit[1] - along * n) < 1e-4, hit[1]

    # --- grid-quantisation kinks are simplified away, real corners are not ---
    # The flown failure: 0.15-0.47 m segments with 7-36 degree kinks, jerk
    # 112-163% of limit at EVERY duration solve() tried. Peak jerk goes as ~6a/T,
    # so a short segment is expensive whatever else is done to it -- the only fix
    # is to stop emitting them.
    # +-4 cm: 8 cm peak-to-peak, inside the 10 cm tolerance, so it all goes.
    # (+-6 cm would NOT: DP measures off the CHORD, and 12 cm peak-to-peak is
    # wider than the tolerance -- the fixture has to respect that or it is
    # testing the wrong thing.)
    zig = [np.array([x, 0.04 * ((i % 2) * 2 - 1), 3.5])
           for i, x in enumerate(np.arange(0.0, 4.01, 0.2))]
    kept, _ = simplify(zig, [0.0] * len(zig))
    assert len(kept) == 2, 'nhieu luoi 4 cm phai bi xoa sach, con {}'.format(len(kept))

    # a REAL 90 degree corner survives, because it is 1.5 m off the chord
    corner = [np.array([0.0, 0, 3.5]), np.array([1.5, 0, 3.5]),
              np.array([1.5, 1.5, 3.5])]
    kept, _ = simplify(corner, [0.0] * 3)
    assert len(kept) == 3, 'goc that bi cat mat'

    # nothing may stray further than the tolerance from N4's path -- this is the
    # whole safety argument for simplifying at all
    rng2 = np.random.default_rng(7)
    wiggly = [np.array([0.2 * i, 0.0, 3.5]) + rng2.normal(0, 0.03, 3)
              for i in range(25)]
    kept, _ = simplify(wiggly, [0.0] * 25)
    for q in wiggly:
        d = min(_point_to_seg(q, kept[i], kept[i + 1]) for i in range(len(kept) - 1))
        assert d <= SIMPLIFY_TOL + 1e-9, d

    # and the short kinks that actually failed in flight now solve
    for d_m, deg in ((0.15, 36), (0.32, 7), (0.47, 9)):
        th = math.radians(deg)
        path = [np.array([0.0, 0, 3.5]),
                np.array([d_m * math.cos(th), d_m * math.sin(th), 3.5]),
                np.array([d_m * math.cos(th) + 2.0, d_m * math.sin(th), 3.5])]
        segs_k, ok_k = build_segments(path, [0.0] * 3, path[0],
                                      np.array([V_CRUISE, 0, 0]), np.zeros(3))
        assert segs_k and ok_k, 'doan {} m / {} deg van cat ngan'.format(d_m, deg)

    # --- mask arithmetic matches what the probe proved on the vehicle ---
    assert MASK_PVA == 2048 and MASK_FALLBACK == 2496

    # --- the tracking guard measures DIVERGENCE, not the built-in GUIDED lag ---
    now = 1000.0
    # a drone flying the line perfectly, just SHAPE_LAG behind: the command ran
    # from x=0 to x=now*V_CRUISE, the drone is where the command was 0.7 s ago
    log = [(now - 2.0 + i * 0.05, np.array([i * 0.05 * V_CRUISE, 0.0, 3.0]))
           for i in range(40)]
    behind = log[-1][1] - np.array([SHAPE_LAG * V_CRUISE, 0.0, 0.0])
    head = np.array([V_CRUISE, 0.0, 0.0])
    _, cross = split_error(lag_error_vec(log, now, behind), head)
    # pure lag reads as ~zero cross-track. The residual is one log sample of
    # quantisation (0.05 s at 20 Hz x 0.8 m/s = 0.04 m), well under TRACK_HOLD.
    assert cross < 0.05, cross

    # A drone LATE along the path must not trip the guard, whatever the total
    # distance says. This is the livelock that cost a gate on two runs: measured
    # in flight the along term ran +-0.45..0.52 m -- sign flipping, because at
    # cruise the SHAPE_LAG compensation overshoots and the drone reads as being
    # AHEAD of its own past command -- while cross-track never left 0.05..0.29 m.
    for slip in (-0.9, +0.9):
        vec = lag_error_vec(log, now, behind + np.array([slip, 0.0, 0.0]))
        along, cross = split_error(vec, head)
        assert abs(along) > TRACK_HOLD and cross < 0.05, (along, cross)

    # ...but a drone off the path SIDEWAYS still does.
    vec = lag_error_vec(log, now, behind + np.array([0.0, 0.9, 0.0]))
    assert split_error(vec, head)[1] > TRACK_HOLD

    # holding (no commanded velocity): every direction counts as cross-track,
    # because drifting while told to stand still is divergence in any direction
    assert split_error(np.array([0.9, 0.0, 0.0]), np.zeros(3))[1] > TRACK_HOLD

    # and nothing is judged until the log reaches SHAPE_LAG back
    assert lag_error_vec(log[-3:], now, behind) is None
    assert lag_error_vec([], now, behind) is None

    total = sum(t.T for t, _ in segs)
    print('trajectory_follower self-check OK ({} doan, {:.1f} s, C2 lien tuc, '
          'doan chui v={} hang, a_dinh {:.2f} m/s2)'.format(
              len(segs), total, V_CRUISE,
              max(t.peak_accel().max() for t, _ in segs)))


def main():
    if '--selfcheck' in sys.argv:
        _selfcheck()
        return
    rclpy, TrajectoryFollower = _build_node()
    rclpy.init()
    node = TrajectoryFollower()
    try:
        node.run()
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
