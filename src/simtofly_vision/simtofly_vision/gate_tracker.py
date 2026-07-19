#!/usr/bin/env python3
"""N2 gate_tracker -- turns noisy per-frame detections into a stable gate MAP.

N1 (gate_detect) fires a fresh 3D detection every frame, in the DEPTH optical
frame, with noise and the occasional miss. N2 does three jobs the planner needs
and N1 cannot:

  1. FRAME  -- transform each detection into `map` (the ENU frame MAVROS and the
               planner share), using tf2 at the detection's OWN stamp, so a moving
               drone does not smear the answer. This is the ONE place N2 touches
               frames; everything downstream is already in map.
  2. TRACK  -- associate detections to known gates by 3D distance, and low-pass the
               centre + normal per gate. One physical ring seen 40 times becomes one
               gate, not 40.
  3. CONFIRM-- a gate is TENTATIVE until seen CONFIRM_HITS consistent times, then
               CONFIRMED. Only CONFIRMED gates go out on /gate/map, so a one-frame
               false positive never reaches the planner.

Ordering (which gate is next) and PASSED marking live with the mission FSM, not
here -- they need the drone's live pose and the flight plan, which N2 deliberately
does not. N2's job ends at "here is the stable set of rings and where they are".
# ponytail: PoseArray output, not a custom Gate.msg. No consumer needs per-gate id
# or state on the wire yet (the planner reads centres+normals); add the message
# package when something actually needs to read the id, not before.

  PoseArray /gate/map:
    header.frame_id = 'map', stamp = now
    poses[i]        = a CONFIRMED gate, ordered by id (stable across frames):
        position    = filtered ring centre (map)
        orientation = quaternion whose +Z is the ring normal (fly-through dir, map)

Run:  ros2 run simtofly_vision gate_tracker
      python3 gate_tracker.py --selfcheck
(Needs N1 publishing /gate/detections, MAVROS publishing map->base_link, and the
static base_link->d435i_depth_optical_frame TF from gates_mission.launch.py.)
"""
import math

import numpy as np
import rclpy
from geometry_msgs.msg import Pose, PoseArray
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener, LookupException, \
    ConnectivityException, ExtrapolationException

MAP_FRAME = 'map'
ASSOC_DIST = 1.0            # m; a detection within this of a gate IS that gate
MERGE_DIST = 2.0           # m; two TRACKS this close are one ring seen twice --
                           # deliberately WIDER than ASSOC_DIST, see Tracker._merge.
                           # ponytail: the course's closest two rings are 5.1 m
                           # apart, so this cannot fuse real neighbours. Tighten it
                           # if a course ever packs rings closer than ~4 m.
CONFIRM_HITS = 5           # consistent detections before TENTATIVE -> CONFIRMED
GATE_ALPHA = 0.3           # low-pass weight for each new observation
PUBLISH_HZ = 5.0
# DO NOT raise this -- see the long note on gate_planner.TF_TIMEOUT. Short version:
# a tf2 timeout BLOCKS, this node is on a single-threaded executor, and the thing it
# blocks is the /tf callback that would deliver the transform. Waiting therefore
# cannot succeed, and it starves the listener so the next lookup is worse. Drop the
# detection instead; at ~10 detections/s, losing one is free.
TF_TIMEOUT = 0.0           # s

TENTATIVE, CONFIRMED = 'TENTATIVE', 'CONFIRMED'


# =========================================================================
# pure tracking logic (no ROS, no TF) -- this is what the self-check exercises
# =========================================================================
class Gate:
    __slots__ = ('id', 'centre', 'normal', 'hits', 'state')

    def __init__(self, gid, centre, normal):
        self.id = gid
        self.centre = np.asarray(centre, float)
        self.normal = _unit(normal)
        self.hits = 1
        self.state = TENTATIVE

    def update(self, centre, normal, alpha):
        self.centre = (1 - alpha) * self.centre + alpha * np.asarray(centre, float)
        # normals can arrive flipped; fold to the same hemisphere before averaging
        n = _unit(normal)
        if n @ self.normal < 0:
            n = -n
        self.normal = _unit((1 - alpha) * self.normal + alpha * n)
        self.hits += 1
        if self.hits >= CONFIRM_HITS:
            self.state = CONFIRMED


def _unit(v):
    v = np.asarray(v, float)
    return v / (np.linalg.norm(v) + 1e-12)


def associate(centre, gates, thresh=ASSOC_DIST):
    """Index of the nearest gate within thresh, or None to start a new one."""
    best, best_d = None, thresh
    for i, g in enumerate(gates):
        d = np.linalg.norm(g.centre - centre)
        if d < best_d:
            best, best_d = i, d
    return best


class Tracker:
    """The map of gates. Feed it map-frame (centre, normal) detections."""

    def __init__(self):
        self.gates = []
        self._next_id = 0

    def observe(self, centre, normal):
        i = associate(centre, self.gates)
        if i is None:
            self.gates.append(Gate(self._next_id, centre, normal))
            self._next_id += 1
            fresh = self.gates[-1]
        else:
            fresh = self.gates[i]
            fresh.update(centre, normal, GATE_ALPHA)
        self._merge(fresh)

    def _merge(self, fresh):
        """Fold twin tracks of one ring back into one.

        A ring first seen from far away is ranged badly -- depth bias runs along
        the view ray, and measured on hardware that is 0.22 m at 5 m and grows.
        If the first few detections land more than ASSOC_DIST off, they start a
        track THERE. As the drone closes, the good detections land at the true
        centre, more than ASSOC_DIST from that first track, so they start a
        SECOND one. Both pass CONFIRM_HITS and /gate/map carries the same ring
        twice, ~1 m apart (seen live).

        Association alone cannot undo this. The stale track is nearer to nothing
        -- every later detection prefers the good track -- so it never updates,
        never drifts, and sits there for the whole flight. Merging has to be a
        separate step with a WIDER radius than association, which is the entire
        reason MERGE_DIST > ASSOC_DIST; at equal radii this would never fire on
        the case it exists for.

        The track carrying the NEWEST detection wins its centre and normal; hits
        only break ties between two tracks the current detection did not touch.
        Not the better-observed track, and not an average of the two:

          - averaging folds the stale far-range reading back in, which is the
            bias that split the track in the first place;
          - preferring hits deadlocks outright, and the self-check below is the
            case that proved it. The stale track is the one with history, so it
            wins every merge, keeps its wrong centre, and swallows each new
            detection as a track that is created and immediately absorbed. The
            map then never moves off the bad estimate at all -- worse than the
            duplicate, because now there is nothing to notice.

        Recency is the right signal precisely because the failure is staleness:
        the freshest track holds the closest, best-ranged view of the ring. It
        keeps the lower id so the map does not renumber underneath the planner.
        """
        while True:
            pair = next((
                (i, j)
                for i in range(len(self.gates))
                for j in range(i + 1, len(self.gates))
                if np.linalg.norm(
                    self.gates[i].centre - self.gates[j].centre) < MERGE_DIST),
                None)
            if pair is None:
                return
            a, b = self.gates[pair[0]], self.gates[pair[1]]
            if fresh is a or fresh is b:
                keep, drop = (a, b) if fresh is a else (b, a)
            else:
                keep, drop = sorted((a, b), key=lambda g: -g.hits)
            keep.id = min(keep.id, drop.id)
            keep.hits += drop.hits
            if keep.hits >= CONFIRM_HITS:
                keep.state = CONFIRMED
            self.gates.remove(drop)

    def confirmed(self):
        return [g for g in self.gates if g.state == CONFIRMED]


# =========================================================================
# quaternion helpers (shared convention with N1: +Z of the pose IS the normal)
# =========================================================================
def quat_from_zaxis(n):
    z = _unit(n)
    a = np.array([1.0, 0.0, 0.0]) if abs(z[0]) < 0.9 else np.array([0.0, 1.0, 0.0])
    x = _unit(np.cross(a, z))
    y = np.cross(z, x)
    m = np.column_stack((x, y, z))
    t = np.trace(m)
    if t > 0:
        s = math.sqrt(t + 1.0) * 2
        return ((m[2, 1] - m[1, 2]) / s, (m[0, 2] - m[2, 0]) / s,
                (m[1, 0] - m[0, 1]) / s, 0.25 * s)
    i = int(np.argmax(np.diag(m)))
    j, k = (i + 1) % 3, (i + 2) % 3
    s = math.sqrt(m[i, i] - m[j, j] - m[k, k] + 1.0) * 2
    q = [0.0, 0.0, 0.0]
    q[i], q[j], q[k] = 0.25 * s, (m[j, i] + m[i, j]) / s, (m[k, i] + m[i, k]) / s
    return q[0], q[1], q[2], (m[k, j] - m[j, k]) / s


def zaxis_from_quat(x, y, z, w):
    """The pose's local +Z in the parent frame -- inverse of quat_from_zaxis."""
    return np.array([2 * (x * z + w * y), 2 * (y * z - w * x),
                     1 - 2 * (x * x + y * y)])


def quat_to_matrix(x, y, z, w):
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
        [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
        [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)]])


# =========================================================================
# ROS node
# =========================================================================
class GateTracker(Node):
    def __init__(self):
        super().__init__('gate_tracker')
        self.tracker = Tracker()
        self.tf_buf = Buffer()
        TransformListener(self.tf_buf, self)
        self.pub = self.create_publisher(PoseArray, '/gate/map', 10)
        self.create_subscription(PoseArray, '/gate/detections', self.on_dets, 10)
        self.create_timer(1.0 / PUBLISH_HZ, self.publish_map)

    def on_dets(self, msg):
        if not msg.poses:
            return
        try:
            # lookup at the frame's OWN stamp; a moving drone makes "latest" wrong
            tf = self.tf_buf.lookup_transform(
                MAP_FRAME, msg.header.frame_id, msg.header.stamp,
                timeout=rclpy.duration.Duration(seconds=TF_TIMEOUT))
        except (LookupException, ConnectivityException, ExtrapolationException) as e:
            self.get_logger().warn('no TF {}<-{} at stamp: {}'.format(
                MAP_FRAME, msg.header.frame_id, e), throttle_duration_sec=2.0)
            return
        t = tf.transform.translation
        q = tf.transform.rotation
        R = quat_to_matrix(q.x, q.y, q.z, q.w)
        trans = np.array([t.x, t.y, t.z])
        for pose in msg.poses:
            c = np.array([pose.position.x, pose.position.y, pose.position.z])
            n = zaxis_from_quat(pose.orientation.x, pose.orientation.y,
                                pose.orientation.z, pose.orientation.w)
            c_map = R @ c + trans
            n_map = R @ n                      # rotate the normal, do not translate
            self.tracker.observe(c_map, n_map)

    def publish_map(self):
        gates = self.tracker.confirmed()
        msg = PoseArray()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = MAP_FRAME
        for g in sorted(gates, key=lambda g: g.id):
            p = Pose()
            p.position.x, p.position.y, p.position.z = (float(v) for v in g.centre)
            qx, qy, qz, qw = quat_from_zaxis(g.normal)
            p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = \
                qx, qy, qz, qw
            msg.poses.append(p)
        self.pub.publish(msg)


# =========================================================================
# self-check -- pure logic, no ROS/TF:  python3 gate_tracker.py --selfcheck
# =========================================================================
def _selfcheck():
    # quaternion normal round-trip (must agree with N1's convention)
    for nn in ([0, 0, 1], [1, 0, 0], [0.3, -0.2, 0.93]):
        nn = _unit(nn)
        assert np.allclose(zaxis_from_quat(*quat_from_zaxis(nn)), nn, atol=1e-6), nn

    # rotate-a-vector via quat_to_matrix matches zaxis_from_quat for +Z
    qx, qy, qz, qw = quat_from_zaxis([0.3, -0.2, 0.93])
    R = quat_to_matrix(qx, qy, qz, qw)
    assert np.allclose(R @ [0, 0, 1], zaxis_from_quat(qx, qy, qz, qw), atol=1e-9)

    # --- tracking: two physical gates, jittered detections, some flipped normals ---
    tr = Tracker()
    rng = np.random.default_rng(1)
    g1, n1 = np.array([5.0, 0.0, 3.5]), _unit([1, 0, 0])
    g2, n2 = np.array([10.0, 2.0, 4.0]), _unit([0.87, 0.5, 0])
    for _ in range(8):
        tr.observe(g1 + rng.normal(0, 0.05, 3), n1 * rng.choice([1, -1]))
        tr.observe(g2 + rng.normal(0, 0.05, 3), n2 * rng.choice([1, -1]))
    assert len(tr.gates) == 2, 'expected 2 gates, got {}'.format(len(tr.gates))
    conf = tr.confirmed()
    assert len(conf) == 2, 'both should be CONFIRMED after 8 hits'
    by_x = sorted(conf, key=lambda g: g.centre[0])
    assert np.linalg.norm(by_x[0].centre - g1) < 0.1, by_x[0].centre
    assert np.linalg.norm(by_x[1].centre - g2) < 0.1, by_x[1].centre
    # normal recovered up to sign (folded to one hemisphere)
    assert abs(abs(by_x[0].normal @ n1) - 1.0) < 0.02, by_x[0].normal
    assert abs(abs(by_x[1].normal @ n2) - 1.0) < 0.05, by_x[1].normal

    # --- a lone one-frame detection stays TENTATIVE, never reaches /gate/map ---
    tr2 = Tracker()
    tr2.observe([0, 0, 2], [0, 0, 1])
    assert tr2.gates[0].state == TENTATIVE and tr2.confirmed() == []

    # --- association threshold: 1.1 m away starts a new track, 0.5 m does not.
    # Checked on associate() itself, not through Tracker: a Tracker would now
    # merge the 1.1 m pair straight back, which is a different rule.
    one = [Gate(0, [0, 0, 0], [0, 0, 1])]
    assert associate(np.array([1.1, 0, 0]), one) is None
    assert associate(np.array([0.5, 0, 0]), one) == 0
    tr4 = Tracker()
    tr4.observe([0, 0, 0], [0, 0, 1]); tr4.observe([0.5, 0, 0], [0, 0, 1])
    assert len(tr4.gates) == 1

    # --- the duplicate ring: a far-range track, then the true one 1.0 m off ---
    # This is the live failure -- one ring on /gate/map twice. The stale track is
    # never nearest again, so it never drifts and association alone leaves both.
    tr5 = Tracker()
    for _ in range(3):                      # ranged short from 8 m away
        tr5.observe([4.0, 0, 3.5], [1, 0, 0])
    for _ in range(9):                      # then closed in and got it right
        tr5.observe([5.0, 0, 3.5], [1, 0, 0])
    assert len(tr5.gates) == 1, 'mot vong van ra {} track'.format(len(tr5.gates))
    got = tr5.gates[0]
    assert got.id == 0, 'id phai giu so nho de planner khong bi danh so lai'
    assert np.linalg.norm(got.centre - [5.0, 0, 3.5]) < 0.1, \
        'track nhieu hit hon phai thang, khong lay trung binh: {}'.format(got.centre)
    assert len(tr5.confirmed()) == 1

    # ... and two REAL rings at the course's tightest spacing never fuse
    tr6 = Tracker()
    for _ in range(6):
        tr6.observe([5.0, 0.0, 3.5], [1, 0, 0])
        tr6.observe([10.0, 2.0, 4.0], [0.87, 0.5, 0])
    assert len(tr6.gates) == 2, 'gop nham hai vong that'

    print('gate_tracker N2 self-check OK')


def main():
    import sys
    if '--selfcheck' in sys.argv:
        _selfcheck()
        return
    rclpy.init()
    node = GateTracker()
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
