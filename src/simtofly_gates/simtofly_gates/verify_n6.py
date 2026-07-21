#!/usr/bin/env python3
"""Stage 6 grader -- did the drone actually fly THROUGH the ring?

Plan v2 par. 12 stage 6 asks for 10/10 PASSED with a minimum rim clearance above
0.25 m. This measures one run of that, using the par. 7 R3 test, which is the
only one that means "went through the hole":

    s(t) = n . (p(t) - C)          signed distance along the gate normal
    a crossing is a SIGN CHANGE of s between two samples
    interpolate the crossing point p*, then check the OFF-AXIS miss there:
        r = || (p* - C) - (n . (p* - C)) n ||        must be < r_in

Checking the off-axis distance at the CURRENT sample instead (what v1 did) is
wrong in both directions: the drone can pass clean through and drift wide a
moment later, or fly outside the ring and happen to line up when sampled.

The gate poses come from /gate/map -- the drone's own estimate, never the
simulator. EVERY gate in the map is graded, not just the first: the course has
four of them and a run that threads ring 1 then clips ring 2 is a failure.

Start this BEFORE N5 takes off and leave it running for the whole flight; it
grades on Ctrl-C. A fixed run length silently grades the wrong window -- the
map only fills in as the drone flies, so the interesting crossings are late.

Run (before starting N5):  ros2 run simtofly_gates verify_n6
"""
import numpy as np
import rclpy
from geometry_msgs.msg import PoseArray, PoseStamped
from rclpy.node import Node

try:
    from simtofly_mavros_sitl.guided_base import mavros_qos
    from simtofly_gates.mission_fsm import (CLEAR_MIN, KEY_R, R_IN,
                                                  plane_crossing,
                                                  zaxis_from_quat)
except ImportError:
    from guided_base import mavros_qos
    from mission_fsm import (CLEAR_MIN, KEY_R, R_IN, plane_crossing,
                             zaxis_from_quat)

# The R3 geometry and the pass/fail thresholds live in N6 (mission_fsm), which
# is what actually decides PASSED in flight. Grading against a second copy here
# would let the grader and the mission drift apart and still both look right.


class VerifyN6(Node):
    def __init__(self):
        super().__init__('verify_n6')
        self.create_subscription(PoseStamped, '/mavros/local_position/pose',
                                 self._pose, mavros_qos())
        self.create_subscription(PoseArray, '/gate/map', self._map, 10)
        self.gates = {}            # index -> (centre, normal) as the DRONE believes
        self.prev = {}             # index -> (position, signed distance) last tick
        self.crossings = {}        # index -> [miss, ...]
        self.track = []            # flown positions, for the "never got there" case

    def _key(self, C):
        """A gate identity that survives the map being reordered.

        The index in /gate/map is NOT stable: gate_tracker publishes the
        CONFIRMED subset of an append-ordered list, so a gate that confirms
        late inserts ahead of gates already being graded and shifts every index
        after it. Keying on index would then move a crossing onto the wrong
        gate. Position is stable instead -- gates never move and sit metres
        apart, so nearest-centre association cannot confuse two of them."""
        for k, (C0, _) in self.gates.items():
            if float(np.linalg.norm(C - C0)) < KEY_R:
                return k
        return len(self.gates)

    def _map(self, m):
        for pose in m.poses:
            p, o = pose.position, pose.orientation
            C = np.array([p.x, p.y, p.z])
            n = zaxis_from_quat(o.x, o.y, o.z, o.w)
            self.gates[self._key(C)] = (C, n / (np.linalg.norm(n) + 1e-12))

    def _pose(self, m):
        p = m.pose.position
        q = np.array([p.x, p.y, p.z])
        self.track.append(q)
        for i, (C, n) in self.gates.items():
            s = float(n @ (q - C))
            prev = self.prev.get(i)
            self.prev[i] = (q, s)
            if prev is None:
                continue                                   # need two samples
            hit = plane_crossing(prev[0], prev[1], q, s, C, n)
            if hit is None:
                continue
            star, miss = hit
            self.crossings.setdefault(i, []).append(miss)
            self.get_logger().info(
                'CAT MAT PHANG VONG {} tai {} -- lech truc {:.3f} m ({})'
                .format(i, np.round(star, 3), miss,
                        'LOT' if miss < R_IN else 'TRUOT NGOAI VANH'))


def report(node, partial=False, expect=0):
    """Grade every gate on the map. A gate crossed outside the rim is a failure,
    and so -- unless `partial` -- is a gate never crossed at all.

    Uncrossed gates used to be excused, on the grounds that the run may have
    been stopped early. That excuse graded a run which flew two of four rings
    and then stalled for 190 s as `GD6 PASS`: the two it never reached were
    silently dropped from the score. An interrupted run is the rarer case and
    the operator knows when they interrupted one, so the strict reading is the
    default and `--partial` asks for the lenient one.
    """
    log = node.get_logger()
    if not node.gates:
        log.error('khong nhan duoc /gate/map')
        return False
    # The denominator is the drone's OWN map, not the course. A gate never
    # detected is not graded, not counted, and not missed -- so PASS means
    # "flew every gate it found", which on a bad-light or long-range run can be
    # a fraction of the gates actually standing there. `--expect` supplies the
    # real number, which no topic carries because only the operator who placed
    # the gates knows it.
    log.info('=== GD6 === ({} vong tren ban do{}{})'.format(
        len(node.gates),
        ' / {} vong tren san'.format(expect) if expect
        else ' CUA DRONE -- khong biet san that co bao nhieu vong',
        ', cham kieu PARTIAL' if partial else ''))
    ok = bool(node.crossings)
    # Measured over four runs (2026-07-21) TWO flew three of four gates and were
    # reported as complete: the missing gate had been SEEN, localised to a few
    # centimetres, and dropped for want of a fifth confirming hit. Grading only
    # what reached the map scores that run as a clean sweep.
    if expect and len(node.gates) < expect:
        log.error('ban do chi co {}/{} vong -- {} vong CHUA BAO GIO LEN BAN DO,'
                  ' khong the cham'.format(len(node.gates), expect,
                                           expect - len(node.gates)))
        ok = False
    for i in sorted(node.gates):
        C = node.gates[i][0]
        misses = node.crossings.get(i)
        if not misses:
            far = min((float(np.linalg.norm(q - C)) for q in node.track),
                      default=float('nan'))
            log.info('  vong {} @ {}: KHONG cat mat phang, gan tam nhat {:.2f} m'
                     ' {}'.format(i, np.round(C, 2), far,
                                  '(bo qua)' if partial else '<- TRUOT'))
            ok = ok and partial
            continue
        best = min(misses)
        clear = R_IN - best
        good = best < R_IN and clear > CLEAR_MIN
        ok = ok and good
        log.info('  vong {} @ {}: cat {} lan, lech truc {:.3f} m, cach vanh '
                 '{:.3f} m [can > {:.2f}]  {}'
                 .format(i, np.round(C, 2), len(misses), best, clear,
                         CLEAR_MIN, 'DAT' if good else 'TRUOT'))
    return ok


def main():
    import sys
    partial = '--partial' in sys.argv
    expect = 0
    if '--expect' in sys.argv and sys.argv.index('--expect') + 1 < len(sys.argv):
        expect = int(sys.argv[sys.argv.index('--expect') + 1])
    rclpy.init()
    node = VerifyN6()
    node.get_logger().info('dang do... Ctrl-C de cham diem')
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.05)
    except KeyboardInterrupt:
        pass
    # Ctrl-C is the ONLY way this node ever produces its report, and under
    # `ros2 run` it does not arrive as KeyboardInterrupt. rclpy's signal handler
    # tears the context down first, and what surfaces from spin_once depends on
    # how far that teardown got: ExternalShutdownException, or RCLError ("the
    # given context is not valid") once the context is gone. Both were observed
    # on consecutive runs. Catching narrowly means the grader dies with a
    # traceback at the exact moment it is supposed to grade -- a clean 4/4 run
    # scored as a crash, twice. Nothing here is worth losing the report over, so
    # the spin loop ending ANY way just ends the measurement; the reason is
    # printed rather than swallowed.
    except Exception as exc:
        print('spin ket thuc: {}: {}'.format(type(exc).__name__, exc))
    crossed = len(node.crossings)
    print('GD6 {} ({}/{} vong da cat mat phang)'.format(
        'PASS' if report(node, partial, expect) else 'FAIL',
        crossed, expect or len(node.gates)))
    node.destroy_node()
    try:
        rclpy.shutdown()
    except Exception:
        pass


def _selfcheck():
    """Pure geometry, no ROS:  python3 verify_n6.py --selfcheck"""
    C, n = np.array([5.0, 0.0, 3.5]), np.array([1.0, 0.0, 0.0])

    def fly(pts):
        """Replay a track through one gate, return the misses it records."""
        out, prev = [], None
        for q in map(np.asarray, pts):
            s = float(n @ (q - C))
            if prev is not None:
                hit = plane_crossing(prev[0], prev[1], q, s, C, n)
                if hit is not None:
                    out.append(hit[1])
            prev = (q, s)
        return out

    # dead centre
    m = fly([[4.5, 0, 3.5], [5.5, 0, 3.5]])
    assert len(m) == 1 and m[0] < 1e-9, m

    # the C6 bug: passes clean through, then drifts 2 m wide AFTER the plane.
    # Grading at the sample instead of the interpolated crossing calls this a
    # miss; interpolation must still see it as centred.
    m = fly([[4.9, 0.0, 3.5], [5.1, 0.2, 3.5], [6.0, 2.0, 3.5]])
    assert len(m) == 1 and m[0] < 0.11, m

    # the mirror image: flies wide, lines up only after crossing -> must FAIL
    m = fly([[4.9, 2.0, 3.5], [5.1, 1.9, 3.5], [5.5, 0.0, 3.5]])
    assert len(m) == 1 and m[0] > R_IN, m

    # samples straddling the plane 1 m apart still interpolate, and approaching
    # without reaching the plane records nothing
    assert fly([[4.0, 0.3, 3.5], [6.0, 0.3, 3.5]])[0] - 0.3 < 1e-9
    assert fly([[3.0, 0, 3.5], [4.0, 0, 3.5], [4.9, 0, 3.5]]) == []

    # flying back out the way it came is a second crossing, not a cancelled one
    assert len(fly([[4.5, 0, 3.5], [5.5, 0, 3.5], [4.5, 0, 3.5]])) == 2

    # identity survives the map REORDERING: gate B is graded first, then a gate
    # that confirms late is inserted ahead of it and shifts every index down.
    # Keyed on index, B's second sighting would land on a different gate.
    node = VerifyN6.__new__(VerifyN6)
    node.gates = {}
    kb = node._key(np.array([10.0, 2.0, 4.0]))
    node.gates[kb] = (np.array([10.0, 2.0, 4.0]), n)
    assert node._key(np.array([10.02, 1.98, 4.01])) == kb      # same gate, drifted
    assert node._key(np.array([5.0, 0.0, 3.5])) != kb          # the late arrival

    # a run that flew 2 of 4 rings must NOT score PASS -- the exact false green
    # that hid a 190 s stall behind two good crossings
    g = VerifyN6.__new__(VerifyN6)
    g.gates = {i: (np.array([5.0 * i, 0.0, 3.5]), n) for i in range(4)}
    g.crossings = {0: [0.02], 1: [0.05]}
    g.track = [np.array([0.0, 0.0, 3.5])]
    g.get_logger = lambda: type('L', (), {
        'info': lambda _s, m, **k: None, 'error': lambda _s, m, **k: None})()
    assert not report(g), 'hai vong chua bay van cham PASS'
    assert report(g, partial=True), '--partial phai tha cho run bi cat ngan'
    g.crossings = {i: [0.02] for i in range(4)}
    assert report(g), 'bay du 4/4 ma cham FAIL'

    # the flown false green: a gate the drone never confirmed is not on the map,
    # so every gate that IS on the map was flown clean and the run scores PASS.
    # Twice in four runs on 2026-07-21. --expect is the only thing that knows.
    h = VerifyN6.__new__(VerifyN6)
    h.gates = {i: (np.array([5.0 * i, 0.0, 3.5]), np.array([1.0, 0.0, 0.0]))
               for i in range(3)}
    h.crossings = {i: [0.02] for i in range(3)}
    h.track = [np.array([0.0, 0.0, 3.5])]
    h.get_logger = g.get_logger
    assert report(h), 'ba vong tren ban do, ca ba lot -- khong khai expect thi PASS'
    assert not report(h, expect=4), 'san 4 vong ma ban do co 3 thi phai FAIL'
    assert report(h, expect=3), 'expect dung bang so vong tren ban do thi PASS'
    print('verify_n6 selfcheck OK')


if __name__ == '__main__':
    import sys
    if '--selfcheck' in sys.argv:
        _selfcheck()
    else:
        main()
