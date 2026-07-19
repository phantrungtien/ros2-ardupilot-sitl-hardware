#!/usr/bin/env python3
"""N4 dijkstra_planner -- turns the gate map + voxel map into a flyable path.

The plan (par. 4.1) is explicit that flying a course is NOT one Dijkstra. Each gate
imposes three hard anchors on its own normal:

    P_pre  = C - d_pre  * n     (wait point in front)
    P_gate = C                  (the hole)
    P_post = C + d_post * n     (exit point behind)

Dijkstra plans only the FREE-SPACE leg, drone -> P_pre. The bit that actually
threads the ring, P_pre -> P_gate -> P_post, is a forced straight line along the
normal: a search would happily cut the hole diagonally and clip the rim. So the
search never gets a say in the one place a mistake is expensive.

The load-bearing trick is N3's carve_corridor (plan par. 3.2): inflate the rim by
the 0.60 m robot radius and the 1.5 m hole closes over, which would make even the
approach leg route AROUND the gate. Before searching we carve a 0.40 m tube along
the target's normal so P_pre is reachable and sits in free space. Rim stays
occupied+inflated; non-target gates keep their full inflation, so the planner
cannot shortcut through the wrong ring.

Cost (plan par. 4.2), edge u->v:

    w = dist(u,v) * (1 + LAMBDA_C * max(0, 1 - clearance(v)/D_SAFE))

i.e. shortest, but paying to hug obstacles -- this is what makes the result
"optimal AND safe" rather than optimal and scraping walls.
# ponytail: no UNKNOWN term (plan's lambda_u). N3 accumulates OCCUPIED only, it has
# no unknown state to penalise -- unseen space IS free space here. Add the term the
# day N3 grows log-odds ray-casting; the cost function is one line.
# ponytail: scipy.sparse.csgraph.dijkstra over a start/goal sub-box, not a hand-
# rolled heapq loop. Same algorithm, same answer, but the search runs in C: a pure
# Python 26-neighbour loop over even a trimmed box is seconds, and the plan asks
# for <200 ms. The sub-box (AABB(start,goal) + PAD) is what keeps it small.
# ponytail: shortcut + even resample, no Catmull-Rom/B-spline smoothing. Plan v2
# par. 4.4 (C4) drops the spline outright: N5 fits a quintic Mueller trajectory
# between consecutive waypoints, which supplies the geometry AND the time profile.
# Smoothing the polyline first would only give N5 a curved input to re-fit. This
# node owns "which way", N5 owns "how fast" -- keep the output pure geometry.

Output -- nav_msgs/Path on /planner/path, frame `map`, yaw baked into each pose:
free-flight waypoints face their direction of travel (camera looks where it goes),
the P_pre..P_post run is locked to the gate normal.

Run:  ros2 run simtofly_gates gate_planner
      python3 gate_planner.py --selfcheck
(Needs N2 on /gate/map, N3 on /mapping/voxels, MAVROS local_position/pose.)
"""
import math
import time

import numpy as np
from scipy import sparse
from scipy.sparse import csgraph

try:
    from simtofly_gates.voxel_map import (
        VoxelGrid, backproject_depth, quat_to_matrix,
        INFLATE_R, MAP_FRAME, DEPTH_TOPIC, BUILD_HZ)
except ImportError:                       # running the file directly
    from voxel_map import (
        VoxelGrid, backproject_depth, quat_to_matrix,
        INFLATE_R, MAP_FRAME, DEPTH_TOPIC, BUILD_HZ)

# ----- planner constants (plan par. 4.2 / 9) -----
LAMBDA_C = 2.0             # weight on the "stay clear of obstacles" term
D_SAFE = 1.0               # m; clearance beyond this costs nothing extra
Z_MIN, Z_MAX = 0.5, 6.0    # m; never plan into the ground or above the course
PAD = 3.0                  # m of slack around the start/goal AABB for the sub-box

D_PRE = D_POST = 1.5       # m; anchor stand-off along the gate normal
CORRIDOR_R = 0.40          # m; carved tube radius (< ring hole 0.75 m)
CORRIDOR_LEN = 1.5         # m; tube length through the ring centre

STEP = 0.5                 # m between resampled waypoints
REPLAN_GATE_SHIFT = 0.3    # m; target moved more than this -> replan (plan par. 4.5)
PASSED_R = 1.0             # m; a gate matches an N6 /mission/passed entry within
                           # this. Same value as N6's KEY_R -- both are matching
                           # the SAME estimate, and course spacing is >= 4 m.
PLAN_HZ = 2.0              # how often the replan CONDITIONS are checked
WHOLE_GRID_RETRY_S = 5.0   # min gap between whole-grid retries (see dijkstra_path)
MIN_GRID_FRAMES = 3        # depth frames folded in before the first plan is allowed
# DO NOT raise this. A tf2 lookup timeout is a BLOCKING wait, and this node runs on
# the default single-threaded executor -- so while it waits, the /tf subscription
# that would deliver the very transform it is waiting for cannot run. Waiting can
# only ever time out. Worse, it is self-reinforcing: the block starves the TF
# listener, the buffer falls further behind, the next lookup is further "into the
# future", and it blocks again. Setting this to 0.5 s took the map from 1265 voxels
# to ZERO -- the node spent its life asleep on a transform it had made unreachable.
#
# Zero timeout, skip the frame on a miss. That is nearly free here: MAVROS now
# streams the transform at 60 Hz (see --streamrate in gates_mission.launch.py) with
# median lag 11 ms, so the buffer almost always already covers the stamp; and the
# grid ACCUMULATES over a static course, so a dropped frame costs nothing.
# (If a blocking wait is ever genuinely needed, the fix is a MultiThreadedExecutor
# with the TF listener in its own callback group -- not a bigger number here.)
TF_TIMEOUT = 0.0           # s
# How stale the fallback transform may be when the exact-stamp lookup misses.
# 0.05 s x 1 m/s cruise = 5 cm of position error, a quarter of a 0.20 m voxel.
MAX_TF_SLIP = 0.05         # s

NEIGHBOURS = [(i, j, k)
              for i in (-1, 0, 1) for j in (-1, 0, 1) for k in (-1, 0, 1)
              if (i, j, k) != (0, 0, 0)]


def _unit(v):
    v = np.asarray(v, float)
    return v / (np.linalg.norm(v) + 1e-12)


def zaxis_from_quat(x, y, z, w):
    """A pose's local +Z in the parent frame.

    N2 encodes each gate's normal as the pose's +Z axis; this decodes it. Kept
    here rather than imported so this package needs no code dependency on
    simtofly_vision -- the two packages meet on ROS topics only. If the encoding
    ever changes, it changes in gate_tracker.quat_from_zaxis and here together.
    """
    return np.array([2 * (x * z + w * y), 2 * (y * z - w * x),
                     1 - 2 * (x * x + y * y)])


def anchors(centre, normal, d_pre=D_PRE, d_post=D_POST):
    """The three hard points a gate imposes: (P_pre, P_gate, P_post)."""
    c, n = np.asarray(centre, float), _unit(normal)
    return c - d_pre * n, c.copy(), c + d_post * n


def _shift_slices(off, shape):
    """Slice pair (source, destination) for comparing a grid against itself shifted."""
    src, dst = [], []
    for o, n in zip(off, shape):
        if o >= 0:
            src.append(slice(0, n - o))
            dst.append(slice(o, n))
        else:
            src.append(slice(-o, n))
            dst.append(slice(0, n + o))
    return tuple(src), tuple(dst)


def _build_graph(free, clear, res):
    """26-neighbour directed graph over the FREE voxels of a sub-box.

    Edge u->v is weighted by the CLEARANCE AT v, so entering a tight spot is what
    costs -- which is why the graph is directed even though the grid is not.
    """
    shape = free.shape
    ids = np.arange(free.size).reshape(shape)
    cost = np.maximum(0.0, 1.0 - clear / D_SAFE)      # 0 in the open, 1 at a wall
    rows, cols, data = [], [], []
    for off in NEIGHBOURS:
        s, d = _shift_slices(off, shape)
        ok = free[s] & free[d]
        if not ok.any():
            continue
        rows.append(ids[s][ok])
        cols.append(ids[d][ok])
        step = res * math.sqrt(off[0] ** 2 + off[1] ** 2 + off[2] ** 2)
        data.append(step * (1.0 + LAMBDA_C * cost[d][ok]))
    if not rows:
        return sparse.csr_matrix((free.size, free.size))
    return sparse.csr_matrix(
        (np.concatenate(data), (np.concatenate(rows), np.concatenate(cols))),
        shape=(free.size, free.size))


def dijkstra_path(grid, occ, start, goal, whole_grid_retry=True):
    """Shortest safe voxel path start->goal in world coords, or None.

    `occ` is an already-inflated (and corridor-carved) occupancy array over the
    whole grid. Searches the AABB(start, goal)+PAD sub-box first; a detour wider
    than PAD looks like FAILURE from inside that box, so a failure is retried once
    on the WHOLE grid before believing it.

    Measured on the course grid (130x60x40 = 312k voxels, idle box):

        leg to the nearest gate, 3.5 m     47 ms
        longest leg the D435i can feed     75 ms   <- realistic worst case
        whole-grid retry                  330 ms   <- only on failure

    pick_target always aims at the NEAREST un-passed gate and the depth camera only
    sees ~10 m, so the sub-box never spans more than about that: the normal path has
    a wide margin under the plan's 200 ms. Only the retry exceeds it.

    `whole_grid_retry=False` skips that retry. The caller wants this when it would
    otherwise repeat: the retry costs 330 ms and a failure REPEATS at tick rate, so
    an unconditional retry burns two thirds of a 2 Hz executor for as long as the
    map stays unsolvable -- and starves the grid builder sharing that executor.
    Retrying is worth it occasionally (it is what catches a detour wider than PAD),
    just never every tick. See GatePlanner.tick.
    """
    start, goal = np.asarray(start, float), np.asarray(goal, float)
    lo_w = np.minimum(start, goal) - PAD
    hi_w = np.maximum(start, goal) + PAD
    lo = np.clip(grid.world_to_idx(lo_w)[0][0], 0, np.array(grid.shape) - 1)
    hi = np.clip(grid.world_to_idx(hi_w)[0][0], 0, np.array(grid.shape) - 1)
    path = _search(grid, occ, start, goal, lo, hi)
    if path is not None or not whole_grid_retry:
        return path
    whole_lo, whole_hi = np.zeros(3, int), np.array(grid.shape) - 1
    if np.array_equal(lo, whole_lo) and np.array_equal(hi, whole_hi):
        return None                      # the box already WAS the whole grid
    return _search(grid, occ, start, goal, whole_lo, whole_hi)


def _search(grid, occ, start, goal, lo, hi):
    """Dijkstra confined to the voxel index box [lo, hi]."""
    box = tuple(slice(a, b + 1) for a, b in zip(lo, hi))
    sub = occ[box]
    free = ~sub
    # ground and ceiling are not obstacles in the map, they are flight rules
    zs = grid.idx_to_world(np.stack(np.indices(free.shape), -1) + lo)[..., 2]
    free &= (zs >= Z_MIN) & (zs <= Z_MAX)

    si, s_ok = grid.world_to_idx([start])
    gi, g_ok = grid.world_to_idx([goal])
    if not (s_ok[0] and g_ok[0]):
        return None
    si, gi = si[0] - lo, gi[0] - lo
    if np.any(si < 0) or np.any(si >= free.shape) or \
       np.any(gi < 0) or np.any(gi >= free.shape):
        return None
    # the drone is where it is, and the goal is an anchor we must reach: never let
    # inflation of a nearby wall declare either endpoint unreachable
    free[tuple(si)] = free[tuple(gi)] = True

    clear = grid.clearance(sub)
    graph = _build_graph(free, clear, grid.res)
    s_flat = int(np.ravel_multi_index(tuple(si), free.shape))
    g_flat = int(np.ravel_multi_index(tuple(gi), free.shape))
    dist, prev = csgraph.dijkstra(graph, directed=True, indices=s_flat,
                                  return_predecessors=True)
    if not np.isfinite(dist[g_flat]):
        return None

    chain, v = [], g_flat
    while v != s_flat:
        chain.append(v)
        v = prev[v]
        if v < 0:
            return None
    chain.append(s_flat)
    idx = np.array([np.unravel_index(v, free.shape) for v in reversed(chain)]) + lo
    return grid.idx_to_world(idx)


def stuck_report(grid, occ, p):
    """Why a search from p cannot start: (occupied?, free neighbours of 26).

    A FAILURE reads the same whether the map is genuinely walled off or the
    drone is simply sitting inside inflation -- _search force-frees the start
    voxel, but that buys nothing when all 26 of its neighbours are inflated, so
    the search has no outgoing edge and fails identically. These two numbers
    tell the cases apart.
    """
    idx, ok = grid.world_to_idx([p])
    if not ok[0]:
        return None
    i, j, k = idx[0]
    box = occ[max(i - 1, 0):i + 2, max(j - 1, 0):j + 2, max(k - 1, 0):k + 2]
    return bool(occ[i, j, k]), int((~box).sum()) - (0 if occ[i, j, k] else 1)


def line_is_clear(grid, occ, a, b):
    """True if the straight segment a->b touches no occupied voxel."""
    a, b = np.asarray(a, float), np.asarray(b, float)
    n = max(2, int(np.linalg.norm(b - a) / (grid.res * 0.5)) + 1)
    pts = a + np.linspace(0, 1, n)[:, None] * (b - a)
    idx, inb = grid.world_to_idx(pts)
    if not inb.all():
        return False
    return not occ[idx[:, 0], idx[:, 1], idx[:, 2]].any()


def shortcut(grid, occ, path):
    """Drop intermediate voxels the drone can simply fly past (line-of-sight prune)."""
    if len(path) < 3:
        return np.asarray(path, float)
    path = np.asarray(path, float)
    out, i = [path[0]], 0
    while i < len(path) - 1:
        j = len(path) - 1
        while j > i + 1 and not line_is_clear(grid, occ, path[i], path[j]):
            j -= 1
        out.append(path[j])
        i = j
    return np.array(out)


def resample(points, step=STEP):
    """Even spacing along a polyline, corners preserved."""
    pts = np.asarray(points, float)
    if len(pts) < 2:
        return pts
    out = [pts[0]]
    for a, b in zip(pts[:-1], pts[1:]):
        d = np.linalg.norm(b - a)
        for t in np.arange(step, d, step) / d:
            out.append(a + t * (b - a))
        out.append(b)
    return np.array(out)


def free_around_drone(grid, occ, drone):
    """Force the voxels the drone currently occupies FREE, in place.

    The drone is flying THERE. Whatever the map says about that volume is
    therefore wrong -- stale returns, or, far more often, the inflation skirt of
    a ring it is in the middle of threading. Without this the search has nowhere
    to step: _search force-frees the start VOXEL, but that is one cell in a
    solid block, so it has no outgoing edge and Dijkstra fails identically to a
    genuinely walled-off map.

    Measured live (run 3, chasing gate 4 from inside gate 3's ring):
    `drone (occupied=True, free neighbours 0/26)` -- and every plan from that
    pose failed for 90 s until the FSM gave up on the gate, twice a run.

    Radius INFLATE_R, not ROBOT_RADIUS: clearing only the body leaves the drone
    ringed by inflation one voxel further out, still with no way to step. This
    erases the safety margin exactly where the drone has already proved there is
    nothing to hit, and nowhere else.

    What it must NOT do is clear a real obstacle. Clearing 0.60 m of INFLATION
    around the drone is a statement about margin; clearing 0.60 m of MEASURED
    occupancy would let the planner route through a wall the drone happens to be
    hugging. So every measured voxel goes back afterwards -- which also makes
    this safe to call after the corridor carve, whose tube (CORRIDOR_R 0.40)
    lies inside the ring's 0.75 m hole and therefore contains no real voxels.

    Call this LAST, after any corridor carving, for that reason.
    """
    grid.carve_corridor(occ, drone, (0.0, 0.0, 1.0),
                        INFLATE_R, 2.0 * INFLATE_R)
    occ |= grid.occ
    return occ


def plan_to_gate(grid, drone, centre, normal, whole_grid_retry=True):
    """(path, lock_from) for drone -> P_pre -> P_gate -> P_post, or (None, 0).

    The approach leg is searched; the traverse is forced along the normal.
    `lock_from` is the index where the traverse starts -- from there yaw is locked
    to the gate normal instead of following the direction of travel.
    `whole_grid_retry` is passed straight through to dijkstra_path -- see there.
    """
    p_pre, p_gate, p_post = anchors(centre, normal)
    occ = grid.inflated(INFLATE_R)
    # reopen the target ring only -- other gates keep full inflation (plan par. 3.2)
    grid.carve_corridor(occ, centre, normal, CORRIDOR_R, CORRIDOR_LEN)
    free_around_drone(grid, occ, drone)
    approach = dijkstra_path(grid, occ, drone, p_pre, whole_grid_retry)
    if approach is None:
        return None, 0
    approach = resample(shortcut(grid, occ, approach))
    # Drop the approach's own last point. Dijkstra returns VOXEL CENTRES, so its
    # end is p_pre snapped to the 0.2 m grid -- up to ~0.17 m away from the exact
    # p_pre the traverse starts from. Keeping both put a tiny BACKWARD step at the
    # junction (measured live: x -0.030, y -0.087, z -0.050), i.e. a near-180 deg
    # direction reversal inside 0.1 m.
    #
    # v1's carrot chaser never noticed -- it just chased the next point. N5 fits a
    # quintic through consecutive waypoints, and reversing over 0.1 m demands
    # unbounded jerk, so N5 declared the whole traverse infeasible and stopped the
    # drone dead at the gate mouth. The path must be monotone here.
    approach = approach[:-1] if len(approach) > 1 else approach
    return np.vstack([approach, resample([p_pre, p_gate, p_post])]), len(approach)


def path_yaws(path, lock_from, normal):
    """Yaw per waypoint: heading of travel, locked to the gate normal from lock_from on."""
    path = np.asarray(path, float)
    lock = math.atan2(normal[1], normal[0])
    yaws = np.full(len(path), lock)
    for i in range(min(lock_from, len(path))):
        d = path[min(i + 1, len(path) - 1)] - path[i]
        yaws[i] = math.atan2(d[1], d[0]) if np.linalg.norm(d[:2]) > 1e-6 else lock
    return yaws


def path_blocked(grid, occ, path, ahead_from=0):
    """True if any remaining segment of the path now cuts an occupied voxel."""
    for a, b in zip(path[ahead_from:-1], path[ahead_from + 1:]):
        if not line_is_clear(grid, occ, a, b):
            return True
    return False


# =========================================================================
# ROS node
# =========================================================================
def _build_node():
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import (QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy,
                           QoSDurabilityPolicy)
    from geometry_msgs.msg import PoseArray, PoseStamped
    from nav_msgs.msg import Path
    from sensor_msgs.msg import Image
    from visualization_msgs.msg import Marker, MarkerArray
    from tf2_ros import (Buffer, TransformListener, LookupException,
                         ConnectivityException, ExtrapolationException)

    def mavros_qos():
        """BEST_EFFORT, to match what MAVROS actually publishes.

        MAVROS publishes local_position/pose BEST_EFFORT. Subscribing with the
        default RELIABLE profile is not a slow subscriber -- it is NO subscriber:
        the QoS never matches and not one message is delivered. Same profile as
        simtofly_mavros_sitl.guided_base.mavros_qos(); duplicated rather than
        imported so this package keeps no code dependency on that one.
        """
        return QoSProfile(reliability=QoSReliabilityPolicy.BEST_EFFORT,
                          history=QoSHistoryPolicy.KEEP_LAST, depth=10)

    class GatePlanner(Node):
        """N3 + N4 in one process: builds the occupancy grid AND plans on it.

        These were two nodes, and the grid crossed between them as an RViz
        MarkerArray -- the only message that already carried occupied voxel
        centres, so no new message type was needed. That was the wrong trade. A
        ~40 KB marker list at 5 Hz over a depth-1 topic arrived roughly once every
        25 s, so the planner planned against an EMPTY map and, because nothing then
        triggered a replan, never revisited it. It produced a plausible path with
        no obstacle knowledge at all.

        Nothing outside ever needed the grid, so it never needed to be a message.
        In one node the planner reads `self.grid` directly: no serialisation, no
        QoS to mismatch, no drops, and the map is provably current at plan time.
        /mapping/voxels is still PUBLISHED for RViz -- it just is not a data path.
        """

        def __init__(self):
            super().__init__('gate_planner')
            self.grid = VoxelGrid()
            self.grid_frames = 0       # depth frames folded in; 0 = map still blank
            self.gates = []            # [(centre, normal)] CONFIRMED, from N2
            self.passed = []           # centres N6 has judged done, from /mission/passed
            self.have_n6 = False       # heard from N6 at all? see pick_target
            self.drone = None
            self.path = None           # last published, world coords
            self.target = None         # centre of the gate the path was built for
            self.gate_normal = None
            self.t_whole_grid = 0.0    # last whole-grid retry; 0 = allow one now
            self.depth = None          # newest depth frame, built on the timer

            self.tf_buf = Buffer()
            TransformListener(self.tf_buf, self)
            self.create_subscription(Image, DEPTH_TOPIC, self.on_depth, 1)
            self.create_subscription(PoseArray, '/gate/map', self.on_gates, 10)
            # N6 latches this, so a planner restart still learns what is done.
            self.create_subscription(
                PoseArray, '/mission/passed', self.on_passed,
                QoSProfile(durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
                           history=QoSHistoryPolicy.KEEP_LAST, depth=1))
            self.create_subscription(PoseStamped, '/mavros/local_position/pose',
                                     self.on_pose, mavros_qos())
            # TRANSIENT_LOCAL: the path is STATE, not a stream. It is published only
            # on a replan, so with the default VOLATILE durability anything that
            # starts later -- RViz, N5, a debug echo -- sees an empty topic forever
            # even though a current plan exists. Latching hands the newest one to
            # every late joiner.
            self.pub = self.create_publisher(
                Path, '/planner/path',
                QoSProfile(durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
                           history=QoSHistoryPolicy.KEEP_LAST, depth=1))
            self.markers = self.create_publisher(MarkerArray, '/mapping/voxels', 1)
            self.create_timer(1.0 / BUILD_HZ, self.build_grid)
            self.create_timer(1.0 / PLAN_HZ, self.tick)

        # ---- N3 half: depth -> occupancy grid ----
        def on_depth(self, msg):
            self.depth = msg           # keep only the newest; fold in on the timer

        def lookup(self, header):
            """map <- the frame's own stamp, or the newest transform if it is close.

            The exact-stamp lookup is what keeps the map correct while the drone
            moves, so it is tried first. When it misses it misses by almost nothing
            -- measured gaps of 3 to 44 ms, because the image is stamped a hair
            ahead of the last transform. Throwing the frame away over 3 ms is
            silly, so fall back to the newest transform provided it is within
            MAX_TF_SLIP. At 1 m/s cruise that bounds the position error at 5 cm,
            a quarter of a voxel -- invisible in the grid. Beyond it, skip: a
            genuinely stale pose puts obstacles in the wrong place.
            """
            try:
                return self.tf_buf.lookup_transform(
                    MAP_FRAME, header.frame_id, header.stamp,
                    timeout=rclpy.duration.Duration(seconds=TF_TIMEOUT))
            except ExtrapolationException:
                pass
            except (LookupException, ConnectivityException) as e:
                self.get_logger().warn('no TF {}<-{}: {}'.format(
                    MAP_FRAME, header.frame_id, e), throttle_duration_sec=5.0)
                return None
            try:
                tf = self.tf_buf.lookup_transform(
                    MAP_FRAME, header.frame_id, rclpy.time.Time())
            except (LookupException, ConnectivityException,
                    ExtrapolationException):
                return None
            want = header.stamp.sec + header.stamp.nanosec * 1e-9
            got = (tf.header.stamp.sec + tf.header.stamp.nanosec * 1e-9)
            if abs(want - got) > MAX_TF_SLIP:
                self.get_logger().warn(
                    'TF tre {:.0f} ms (> {:.0f} ms), bo frame'.format(
                        (want - got) * 1e3, MAX_TF_SLIP * 1e3),
                    throttle_duration_sec=5.0)
                return None
            return tf

        def build_grid(self):
            msg = self.depth
            if msg is None:
                return
            if msg.encoding != '32FC1':
                self.get_logger().warn(
                    'depth encoding {} != 32FC1'.format(msg.encoding), once=True)
                return
            tf = self.lookup(msg.header)
            if tf is None:
                return
            depth = np.frombuffer(msg.data, np.float32).reshape(
                msg.height, msg.width)
            xyz = backproject_depth(depth)
            if not len(xyz):
                return
            t, q = tf.transform.translation, tf.transform.rotation
            R = quat_to_matrix(q.x, q.y, q.z, q.w)
            self.grid.integrate(xyz @ R.T + np.array([t.x, t.y, t.z]))
            self.grid_frames += 1
            self.publish_markers()

        def publish_markers(self):
            from geometry_msgs.msg import Point
            occ = np.argwhere(self.grid.occ)
            m = Marker()
            m.header.frame_id = MAP_FRAME
            m.header.stamp = self.get_clock().now().to_msg()
            m.ns, m.id = 'occ', 0
            m.type, m.action = Marker.CUBE_LIST, Marker.ADD
            m.scale.x = m.scale.y = m.scale.z = self.grid.res
            m.color.r, m.color.g, m.color.b, m.color.a = 1.0, 0.3, 0.0, 0.7
            for i in occ:
                w = self.grid.idx_to_world(i)
                m.points.append(Point(x=float(w[0]), y=float(w[1]), z=float(w[2])))
            self.markers.publish(MarkerArray(markers=[m]))

        # ---- N4 half ----
        def on_gates(self, msg):
            self.gates = [
                (np.array([p.position.x, p.position.y, p.position.z]),
                 zaxis_from_quat(p.orientation.x, p.orientation.y,
                                 p.orientation.z, p.orientation.w))
                for p in msg.poses]

        def on_pose(self, msg):
            p = msg.pose.position
            self.drone = np.array([p.x, p.y, p.z])

        def on_passed(self, msg):
            self.passed = [np.array([p.position.x, p.position.y, p.position.z])
                           for p in msg.poses]
            self.have_n6 = True

        def pick_target(self):
            """Nearest gate that is not done yet.

            "Done" belongs to N6, which runs the par. 7 R3 test -- interpolate
            the plane crossing, measure the off-axis miss there. The local
            n.(p - C) > 0 test below is only a FALLBACK for running N4 without
            the FSM: it calls a gate passed the moment the drone is on the far
            side of its plane, so drifting wide past the rim silently skips the
            gate instead of flying it again.
            """
            if self.have_n6:
                ahead = [(c, n) for c, n in self.gates
                         if not any(np.linalg.norm(c - d) < PASSED_R
                                    for d in self.passed)]
            else:
                ahead = [(c, n) for c, n in self.gates if n @ (self.drone - c) <= 0]
            if not ahead:
                return None
            return min(ahead, key=lambda g: np.linalg.norm(g[0] - self.drone))

        def needs_replan(self, centre):
            if self.path is None or self.target is None:
                return 'no path'
            if np.linalg.norm(centre - self.target) > REPLAN_GATE_SHIFT:
                return 'target moved'
            occ = self.grid.inflated(INFLATE_R)
            self.grid.carve_corridor(occ, centre, self.gate_normal,
                                     CORRIDOR_R, CORRIDOR_LEN)
            # Same clearing the planner uses, or this asks a different question
            # than the plan it is judging: while the drone sits inside a ring's
            # inflation, the path segment under it reads blocked on every tick
            # and 'path blocked' fires as a metronome again (par. 4.5).
            free_around_drone(self.grid, occ, self.drone)
            # Only the part of the path STILL AHEAD of the drone can block it.
            # path_blocked has taken `ahead_from` for exactly this since it was
            # written, and passing nothing checked the flown part too -- which is
            # precisely where the drone has just filled the map in at close range,
            # so the segments behind it read blocked from then on. Measured live:
            # 47 of 51 replans on one 4-gate run were 'path blocked', firing on
            # every 2 Hz tick and making N5 rebuild its trajectory 54 times. A
            # replan is supposed to be an event (par. 4.5); that was a metronome.
            i = int(np.argmin([float(np.linalg.norm(q - self.drone))
                               for q in self.path]))
            if path_blocked(self.grid, occ, self.path, ahead_from=i):
                return 'path blocked'
            return None            # plan par. 4.5: event-driven only, never periodic

        def waiting_for(self):
            """Which input is missing, or None when we have everything.

            Exists so the node can SAY it is alive and blocked rather than sitting
            silent -- a planner that logs only on replan is indistinguishable from
            a dead one, which cost real debugging time before this was here.
            """
            if self.drone is None:
                return 'pose (/mavros/local_position/pose)'
            if not self.gates:
                return 'gate CONFIRMED (/gate/map)'
            # Never plan against a blank map. An empty grid makes every Dijkstra
            # succeed -- straight to the goal, no obstacle anywhere -- so the
            # failure is silent and the path looks fine. Wait for real depth.
            if self.grid_frames < MIN_GRID_FRAMES:
                return 'ban do: {}/{} depth frame'.format(
                    self.grid_frames, MIN_GRID_FRAMES)
            return None

        def tick(self):
            missing = self.waiting_for()
            if missing is not None:
                self.get_logger().info(
                    'dang doi {} | grid={} voxel, {} frame'.format(
                        missing, int(self.grid.occ.sum()), self.grid_frames),
                    throttle_duration_sec=10.0)
                return
            tgt = self.pick_target()
            if tgt is None:
                self.get_logger().info(
                    'moi gate deu da vuot qua -- khong con muc tieu',
                    throttle_duration_sec=10.0)
                return
            centre, normal = tgt
            self.gate_normal = normal
            why = self.needs_replan(centre)
            if why is None:
                return
            # A failure repeats: self.path stays None, so needs_replan says 'no path'
            # again on the very next tick. Letting every one of those pay for the
            # 330 ms whole-grid retry costs 66% of this 2 Hz executor and starves
            # build_grid, which shares it -- so the retry is rate-limited.
            full = (time.time() - self.t_whole_grid) >= WHOLE_GRID_RETRY_S
            t0 = time.time()
            path, lock_from = plan_to_gate(self.grid, self.drone, centre, normal,
                                           whole_grid_retry=full)
            ms = (time.time() - t0) * 1e3
            if path is None:
                extra = ''
                if full:
                    self.t_whole_grid = time.time()
                    # Only on the whole-grid tick: it is already the slow path,
                    # and rebuilding the inflated grid every 0.5 s just to log
                    # would starve the builder this executor shares.
                    occ = self.grid.inflated(INFLATE_R)
                    self.grid.carve_corridor(occ, centre, normal,
                                             CORRIDOR_R, CORRIDOR_LEN)
                    free_around_drone(self.grid, occ, self.drone)
                    d = stuck_report(self.grid, occ, self.drone)
                    g = stuck_report(self.grid, occ, anchors(centre, normal)[0])
                    extra = ' | drone {} P_pre {}'.format(d, g)
                self.get_logger().warn(
                    'dijkstra FAILURE to gate at {} ({:.0f} ms, {}){}'.format(
                        np.round(centre, 2), ms,
                        'ca luoi' if full else 'chi sub-box', extra),
                    throttle_duration_sec=2.0)
                return
            self.path, self.target = path, centre.copy()
            self.publish(path, lock_from, normal)
            self.get_logger().info(
                'replan ({}) -> {} wpts, {:.1f} m, {:.0f} ms'.format(
                    why, len(path), np.linalg.norm(np.diff(path, axis=0),
                                                   axis=1).sum(), ms))

        def publish(self, path, lock_from, normal):
            yaws = path_yaws(path, lock_from, normal)
            msg = Path()
            msg.header.frame_id = MAP_FRAME
            msg.header.stamp = self.get_clock().now().to_msg()
            for p, yaw in zip(path, yaws):
                ps = PoseStamped()
                ps.header = msg.header
                ps.pose.position.x, ps.pose.position.y, ps.pose.position.z = \
                    (float(v) for v in p)
                ps.pose.orientation.z = math.sin(yaw / 2.0)
                ps.pose.orientation.w = math.cos(yaw / 2.0)
                msg.poses.append(ps)
            self.pub.publish(msg)

    return rclpy, GatePlanner


# =========================================================================
# self-check -- pure planning math, no ROS:  python3 gate_planner.py --selfcheck
# =========================================================================
def _selfcheck():
    # --- the gate-normal decoding matches N2's encoding (+Z of the pose) ---
    assert np.allclose(zaxis_from_quat(0, 0, 0, 1), [0, 0, 1])
    s = math.sin(math.pi / 4)
    assert np.allclose(zaxis_from_quat(0, s, 0, s), [1, 0, 0], atol=1e-9), \
        'a +90 deg pitch must send the pose +Z to world +X'

    # --- anchors sit on the normal, in the right order ---
    p_pre, p_gate, p_post = anchors([5, 0, 3], [1, 0, 0])
    assert np.allclose(p_pre, [3.5, 0, 3]) and np.allclose(p_post, [6.5, 0, 3])
    assert np.allclose(p_gate, [5, 0, 3])

    # --- Dijkstra routes AROUND a wall, never through it ---
    g = VoxelGrid(origin=(0, 0, 0), dims_m=(12, 8, 6), res=0.2)
    wall = np.array([[6.0, y, z]                              # full height, gap at y>5
                     for y in np.arange(0.0, 5.0, 0.1)
                     for z in np.arange(0.0, 6.0, 0.1)])
    g.integrate(wall)
    occ = g.inflated(INFLATE_R)
    t0 = time.time()
    path = dijkstra_path(g, occ, [2.0, 2.0, 2.0], [10.0, 2.0, 2.0])
    ms = (time.time() - t0) * 1e3
    assert path is not None, 'should route around the wall via the gap'
    idx, inb = g.world_to_idx(path)
    assert inb.all() and not occ[idx[:, 0], idx[:, 1], idx[:, 2]].any(), \
        'path cuts an inflated obstacle'
    assert path[:, 1].max() > 5.0, 'path did not detour through the gap'
    assert (path[:, 2] >= Z_MIN).all(), 'path dipped below the floor rule'
    # The plan's budget is 200 ms and an idle machine does this in ~115. But this
    # self-check also runs while the whole SITL stack is on the same box, where the
    # same call takes 300+ ms -- so asserting 200 here makes the test fail for being
    # BUSY, not for being wrong. The ceiling below is loose on purpose: it still
    # catches the failure that matters (someone drops the start/goal sub-box and the
    # search goes whole-grid, seconds not milliseconds) without crying under load.
    # Judge the real budget from the live `replan ... N ms` log on an idle machine.
    assert ms < 1500.0, 'plan took {:.0f} ms -- sub-box search is gone?'.format(ms)
    print('   (dijkstra {:.0f} ms -- budget 200 ms on an idle box)'.format(ms))

    # a fully sealed wall must FAIL, not silently tunnel
    g2 = VoxelGrid(origin=(0, 0, 0), dims_m=(12, 8, 6), res=0.2)
    g2.integrate(np.array([[6.0, y, z]
                           for y in np.arange(0.0, 8.0, 0.1)
                           for z in np.arange(0.0, 6.0, 0.1)]))
    occ2 = g2.inflated(INFLATE_R)
    t0 = time.time()
    assert dijkstra_path(g2, occ2, [2.0, 2.0, 2.0], [10.0, 2.0, 2.0]) is None
    ms_full = (time.time() - t0) * 1e3
    # ...and the same failure with the retry suppressed must still be None, but
    # WITHOUT paying for the whole-grid pass. This is the guard on the 2 Hz
    # executor: a failure repeats every tick, and an unconditional retry is what
    # turns one unsolvable map into a permanently starved node.
    t0 = time.time()
    assert dijkstra_path(g2, occ2, [2.0, 2.0, 2.0], [10.0, 2.0, 2.0],
                         whole_grid_retry=False) is None
    ms_sub = (time.time() - t0) * 1e3
    assert ms_sub < ms_full * 0.8, (
        'whole_grid_retry=False khong re hon ({:.0f} vs {:.0f} ms) -- co ai bo '
        'qua co nay khong?'.format(ms_sub, ms_full))

    # --- clearance cost actually pushes the path off the obstacle ---
    g3 = VoxelGrid(origin=(0, 0, 0), dims_m=(10, 8, 6), res=0.2)
    g3.integrate(np.array([[x, 2.0, z]                       # low wall along y=2
                           for x in np.arange(3.0, 7.0, 0.1)
                           for z in np.arange(0.0, 6.0, 0.1)]))
    p = dijkstra_path(g3, g3.inflated(INFLATE_R), [5.0, 1.0, 2.0], [5.0, 6.0, 2.0])
    assert p is not None
    # it has to get round the wall's ends; cost-shaping keeps it off the tips
    assert min(abs(p[:, 0].min() - 3.0), abs(p[:, 0].max() - 7.0)) > 0.3, p[:, 0]

    # --- the whole point: a ring's hole is only flyable because we carve it ---
    gr = VoxelGrid(origin=(0, 0, 0), dims_m=(12, 8, 6), res=0.2)
    centre, normal = np.array([6.0, 4.0, 2.0]), np.array([1.0, 0.0, 0.0])
    ang = np.linspace(0, 2 * math.pi, 400, endpoint=False)
    gr.integrate(np.stack([np.full_like(ang, 6.0),
                           4.0 + 0.83 * np.cos(ang),
                           2.0 + 0.83 * np.sin(ang)], axis=-1))
    drone = np.array([2.0, 4.0, 2.0])
    full, lock_from = plan_to_gate(gr, drone, centre, normal)
    # the junction must be MONOTONE -- no backward step where the approach hands
    # over to the traverse, or N5 sees a reversal it cannot fit a quintic through
    steps = np.diff(full, axis=0)
    dirs = steps / np.linalg.norm(steps, axis=1, keepdims=True)
    turns = np.degrees(np.arccos(np.clip((dirs[:-1] * dirs[1:]).sum(axis=1), -1, 1)))
    assert turns.max() < 120.0, \
        'path co goc {:.0f} deg -- co buoc lui o cho noi?'.format(turns.max())
    assert full is not None, 'no path to the gate'
    assert 0 < lock_from < len(full), lock_from
    # from lock_from on, the path is the forced straight run along the normal
    assert np.allclose(np.cross(full[lock_from:] - centre, normal), 0, atol=1e-9)
    # ends past the ring, on the far side, and passes within the hole radius
    assert normal @ (full[-1] - centre) > 1.4, full[-1]
    at_plane = full[np.argmin(np.abs((full - centre) @ normal))]
    assert np.linalg.norm(at_plane - centre) < 0.4, 'crossed the plane off-centre'
    assert np.linalg.norm(np.cross(full[-1] - centre, normal)) < 1e-6, \
        'exit anchor is off the normal axis'

    # --- a drone INSIDE a ring can still plan to the next gate ---
    # This is the stall that ended two live runs: after threading a ring the
    # drone sits in its inflation skirt, so its own voxel and all 26 neighbours
    # read occupied, and every plan to the NEXT gate failed for 90 s until the
    # FSM abandoned the gate. Ring 1 is the one the drone is stuck in; the
    # target is ring 2, five metres on, whose corridor carving does nothing for
    # a start buried in ring 1.
    gs = VoxelGrid(origin=(0, 0, 0), dims_m=(14, 8, 6), res=0.2)
    for cx in (4.0, 9.0):
        gs.integrate(np.stack([np.full_like(ang, cx),
                               4.0 + 0.83 * np.cos(ang),
                               2.0 + 0.83 * np.sin(ang)], axis=-1))
    inside = np.array([4.1, 4.0, 2.0])            # just past ring 1's plane
    occ_s = gs.inflated(INFLATE_R)
    gs.carve_corridor(occ_s, [9.0, 4.0, 2.0], [1.0, 0, 0], CORRIDOR_R, CORRIDOR_LEN)
    assert stuck_report(gs, occ_s, inside) == (True, 0), \
        'the fixture no longer reproduces the entombed start: {}'.format(
            stuck_report(gs, occ_s, inside))
    free_around_drone(gs, occ_s, inside)
    assert stuck_report(gs, occ_s, inside)[1] == 26, 'drone still has no way out'

    # ... but a MEASURED obstacle inside that bubble survives it. Clearing
    # inflation is a statement about margin; clearing a wall the drone is
    # hugging would route the next plan straight through it.
    gw = VoxelGrid(origin=(0, 0, 0), dims_m=(8, 8, 6), res=0.2)
    gw.integrate(np.array([[2.4, y, z] for y in np.arange(3.0, 5.0, 0.1)
                           for z in np.arange(1.0, 3.0, 0.1)]))
    near = np.array([2.0, 4.0, 2.0])                  # 0.4 m off the wall face
    occ_w = gw.inflated(INFLATE_R)
    free_around_drone(gw, occ_w, near)
    wall_idx, _ = gw.world_to_idx([[2.4, 4.0, 2.0]])
    assert occ_w[tuple(wall_idx[0])], 'buc tuong that bi xoa khoi ban do'
    p2, _ = plan_to_gate(gs, inside, np.array([9.0, 4.0, 2.0]),
                         np.array([1.0, 0.0, 0.0]))
    assert p2 is not None, 'still cannot plan out of the ring it just threaded'

    # --- shortcut prunes a staircase to a straight line in open space ---
    g4 = VoxelGrid(origin=(0, 0, 0), dims_m=(8, 8, 6), res=0.2)
    stair = np.array([[1 + 0.2 * i, 1 + 0.2 * (i % 2), 2.0] for i in range(20)])
    sc = shortcut(g4, g4.inflated(INFLATE_R), stair)
    assert len(sc) < len(stair), 'shortcut kept every zigzag point'
    assert np.allclose(sc[0], stair[0]) and np.allclose(sc[-1], stair[-1])

    # --- resample spacing ---
    rs = resample([[0, 0, 0], [2.0, 0, 0]], step=0.5)
    d = np.linalg.norm(np.diff(rs, axis=0), axis=1)
    assert d.max() <= 0.5 + 1e-9 and np.allclose(rs[-1], [2, 0, 0]), rs

    # --- yaw: travel heading, then locked to the normal ---
    line = np.array([[0, 0, 2], [1, 0, 2], [2, 0, 2], [3, 0, 2]], float)
    y = path_yaws(line, lock_from=2, normal=np.array([0.0, 1.0, 0.0]))
    assert abs(y[0]) < 1e-6, 'free leg should face +X travel'
    assert abs(y[-1] - math.pi / 2) < 1e-6, 'traverse should lock to the normal'

    # --- blocked-path detection drives the replan trigger ---
    g5 = VoxelGrid(origin=(0, 0, 0), dims_m=(8, 8, 6), res=0.2)
    straight = resample([[1.0, 4.0, 2.0], [7.0, 4.0, 2.0]])
    assert not path_blocked(g5, g5.inflated(INFLATE_R), straight)
    g5.integrate(np.array([[4.0, 4.0, 2.0]]))
    assert path_blocked(g5, g5.inflated(INFLATE_R), straight)

    print('gate_planner N4 self-check OK '
          '(wall detour {:.0f} ms, sealed wall fails, ring carved & threaded)'
          .format(ms))


def main():
    import sys
    if '--selfcheck' in sys.argv:
        _selfcheck()
        return
    rclpy, GatePlanner = _build_node()
    rclpy.init()
    node = GatePlanner()
    try:
        rclpy.spin(node)
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
