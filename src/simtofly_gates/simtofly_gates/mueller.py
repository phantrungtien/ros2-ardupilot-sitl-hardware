#!/usr/bin/env python3
"""N5 core -- Mueller quintic motion primitives (plan v2 par. 5.2, 5.3).

Mueller/Hehn/D'Andrea, "A Computationally Efficient Motion Primitive for
Quadrocopter Trajectory Generation", IEEE T-RO 2015.

Each axis is an independent triple integrator (position <- velocity <-
acceleration) whose control input is JERK. Minimising the integral of jerk
squared makes the optimal jerk a quadratic in t, so position is a quintic:

    j(t) = (a/2)t^2 + b t + c
    a(t) = a0 + c t + (b/2)t^2 + (a/6)t^3
    v(t) = v0 + a0 t + (c/2)t^2 + (b/6)t^3 + (a/24)t^4
    p(t) = p0 + v0 t + (a0/2)t^2 + (c/6)t^3 + (b/24)t^4 + (a/120)t^5

(alpha, beta, gamma are spelled a, b, c in code.)

WHAT THIS FILE DELIBERATELY DOES NOT DO -- read before "fixing" it:

The paper checks feasibility against THRUST and BODY RATE, because its platform
commands thrust + body rates directly. We do not: we hand setpoints to
ArduCopter's GUIDED position controller, which runs its own jerk-limited S-curve
shaper over whatever we send (mode_guided.cpp -> input_pos_vel_accel_xy ->
shape_pos_vel_accel_xy). So the binding limits are the shaper's, not the motors':

    WPNAV_ACCEL   2.5 m/s^2     (measured on this vehicle, ArduCopter V4.6.3)
    WPNAV_ACCEL_Z 1.0 m/s^2     <- 2.5x TIGHTER than horizontal
    PSC_JERK_XY   5.0 m/s^3
    PSC_JERK_Z    5.0 m/s^3

Ask for more than those and ArduCopter silently clamps: no error, just tracking
error creeping up. Hence the limits below sit ~30% under (plan par. 5.3.1), and
feasibility here checks ACCELERATION and JERK. Checking thrust/body-rate instead
would test a constraint that never binds and miss the one that does.

Self-check:  python3 mueller.py --selfcheck
"""
import math
import sys

import numpy as np

# Limits, ~70% of the measured ArduCopter values above (plan par. 5.3.1).
A_HORIZ_MAX = 1.75         # m/s^2
A_VERT_MAX = 0.70          # m/s^2  -- tighter, WPNAV_ACCEL_Z is only 1.0
J_HORIZ_MAX = 3.50         # m/s^3
J_VERT_MAX = 3.50          # m/s^3

V_HORIZ_MAX = 7.00         # m/s    -- 70% of WPNAV_SPEED 10.0; never binds at
                           #           the 0.8 m/s we actually fly
T_MIN = 1e-3               # s; below this the quintic is numerically meaningless

# Why the last solve() gave up: (which constraint, T tried, accel/limit,
# jerk/limit). Diagnostic only -- a failed solve is reported by the node that
# asked for it, and this is the only way library code can say WHY.
LAST_FAIL = None
# ...and what the caller was asking for when it did: set by the caller, kept
# here so one read gets the whole picture. (d, turn deg, |v0|, |a0|, k, n_seg)
LAST_ASK = None


class Trajectory:
    """One quintic segment on all three axes, valid over t in [0, T].

    Built from a full boundary condition set: (p0, v0, a0) -> (pf, vf, af) in T.
    Every argument is a 3-vector in ENU metres.
    """

    def __init__(self, p0, v0, a0, pf, vf, af, T):
        if T < T_MIN:
            raise ValueError('T = {} qua nho'.format(T))
        self.T = float(T)
        self.p0 = np.asarray(p0, float).copy()
        self.v0 = np.asarray(v0, float).copy()
        self.a0 = np.asarray(a0, float).copy()
        pf = np.asarray(pf, float)
        vf = np.asarray(vf, float)
        af = np.asarray(af, float)

        # strip the free-drift part (what the axis does with zero jerk), so the
        # coefficients only have to supply the DIFFERENCE (plan par. 5.2)
        T2, T3, T4, T5 = T ** 2, T ** 3, T ** 4, T ** 5
        da = af - self.a0
        dv = vf - self.v0 - self.a0 * T
        dp = pf - self.p0 - self.v0 * T - 0.5 * self.a0 * T2

        self.a = (60.0 * T2 * da - 360.0 * T * dv + 720.0 * dp) / T5
        self.b = (-24.0 * T3 * da + 168.0 * T2 * dv - 360.0 * T * dp) / T5
        self.c = (3.0 * T4 * da - 24.0 * T3 * dv + 60.0 * T2 * dp) / T5

    # --- sampling -----------------------------------------------------------
    def position(self, t):
        t = self._clamp(t)
        return (self.p0 + self.v0 * t + 0.5 * self.a0 * t ** 2
                + self.c * t ** 3 / 6.0 + self.b * t ** 4 / 24.0
                + self.a * t ** 5 / 120.0)

    def velocity(self, t):
        t = self._clamp(t)
        return (self.v0 + self.a0 * t + 0.5 * self.c * t ** 2
                + self.b * t ** 3 / 6.0 + self.a * t ** 4 / 24.0)

    def acceleration(self, t):
        t = self._clamp(t)
        return (self.a0 + self.c * t + 0.5 * self.b * t ** 2
                + self.a * t ** 3 / 6.0)

    def jerk(self, t):
        t = self._clamp(t)
        return 0.5 * self.a * t ** 2 + self.b * t + self.c

    def _clamp(self, t):
        # Holding the endpoint past T is what lets the 20 Hz stream degrade
        # gracefully when the next segment is late (plan par. 5.7).
        return min(max(float(t), 0.0), self.T)

    # --- feasibility (closed form, no sampling) -----------------------------
    def peak_jerk(self):
        """Max |jerk| per axis. j(t) is a parabola: check ends and its vertex."""
        cand = [0.0, self.T]
        with np.errstate(divide='ignore', invalid='ignore'):
            vertex = np.where(np.abs(self.a) > 1e-12, -self.b / np.where(
                np.abs(self.a) > 1e-12, self.a, 1.0), np.nan)
        return self._peak(self.jerk, cand, vertex)

    def peak_accel(self):
        """Max |accel| per axis. a(t) is a cubic; its extrema are where j(t)=0."""
        cand = [0.0, self.T]
        # roots of (a/2)t^2 + b t + c = 0, per axis
        roots = []
        for k in range(3):
            A, B, C = 0.5 * self.a[k], self.b[k], self.c[k]
            if abs(A) < 1e-12:
                roots.append([-C / B] if abs(B) > 1e-12 else [])
                continue
            disc = B * B - 4.0 * A * C
            if disc < 0.0:
                roots.append([])
            else:
                sq = math.sqrt(disc)
                roots.append([(-B + sq) / (2.0 * A), (-B - sq) / (2.0 * A)])
        peak = np.array([abs(self.acceleration(0.0)[k]) for k in range(3)])
        for k in range(3):
            peak[k] = max(peak[k], abs(self.acceleration(self.T)[k]))
            for r in roots[k]:
                if 0.0 < r < self.T:
                    peak[k] = max(peak[k], abs(self.acceleration(r)[k]))
        return peak

    def _peak(self, fn, ends, vertex):
        peak = np.zeros(3)
        for t in ends:
            peak = np.maximum(peak, np.abs(fn(t)))
        for k in range(3):
            tv = vertex[k]
            if np.isfinite(tv) and 0.0 < tv < self.T:
                peak[k] = max(peak[k], abs(fn(tv)[k]))
        return peak

    def input_feasible(self):
        """True if accel and jerk stay inside the ArduCopter-derived limits."""
        pa, pj = self.peak_accel(), self.peak_jerk()
        a_lim = np.array([A_HORIZ_MAX, A_HORIZ_MAX, A_VERT_MAX])
        j_lim = np.array([J_HORIZ_MAX, J_HORIZ_MAX, J_VERT_MAX])
        # 1e-9 slack: a boundary condition set exactly AT the limit is feasible,
        # and float error must not flip that into a rejection.
        return bool(np.all(pa <= a_lim + 1e-9) and np.all(pj <= j_lim + 1e-9))

    def cost(self):
        """Integral of jerk^2 over [0, T] -- the paper's cost, in closed form.

        NOTE the plan's par. 5.2 formula is this divided by T (an average rather
        than a total). Kept as the integral because that is what the paper ranks
        by; the self-check pins it against numerical integration either way.
        """
        T, a, b, c = self.T, self.a, self.b, self.c
        per_axis = (a * a * T ** 5 / 20.0 + a * b * T ** 4 / 4.0
                    + (b * b + a * c) * T ** 3 / 3.0 + b * c * T ** 2
                    + c * c * T)
        return float(np.sum(per_axis))

    def state_at(self, t):
        """(p, v, a) in one call -- what the 20 Hz publisher wants."""
        return self.position(t), self.velocity(t), self.acceleration(t)


def corridor_clear(traj, a, b, radius, samples=32):
    """True if the segment stays within `radius` of the line a->b.

    The paper solves a quartic per bounding half-space. This samples instead --
    deliberately. The quartic buys exactness on a check whose INPUT (the gate
    pose, good to ~1 cm in sim and far worse on real depth, see plan par. 13.2)
    is nowhere near that precise, and 32 samples of a quintic is microseconds.
    Swap in the quartic the day the corridor margin, not the pose, is the limit.
    """
    a, b = np.asarray(a, float), np.asarray(b, float)
    axis = b - a
    L = np.linalg.norm(axis)
    if L < 1e-9:
        return True
    axis = axis / L
    ts = np.linspace(0.0, traj.T, samples)
    pts = np.array([traj.position(t) for t in ts])
    rel = pts - a
    along = rel @ axis
    perp = rel - np.outer(along, axis)
    return bool(np.all(np.linalg.norm(perp, axis=1) <= radius))


def solve(p0, v0, a0, pf, vf, af, T, corridor=None, radius=0.5,
          tries=5, growth=1.4):
    """Build a feasible Trajectory, stretching T until it fits (plan par. 5.6).

    Returns (traj, T_used) or (None, None). Stretching ALWAYS helps: the
    coefficients scale as 1/T^3..1/T^5, so a longer segment demands strictly less
    acceleration and jerk for the same boundary conditions.

    `corridor` is (a, b) world points; when given, the path must also stay within
    `radius` of that segment -- the corridor N4's Dijkstra already proved clear.
    """
    global LAST_FAIL
    T = max(float(T), T_MIN)
    for _ in range(int(tries)):
        traj = Trajectory(p0, v0, a0, pf, vf, af, T)
        ok_input = traj.input_feasible()
        ok_corr = corridor is None or corridor_clear(
            traj, corridor[0], corridor[1], radius)
        if ok_input and ok_corr:
            return traj, T
        # WHICH constraint failed is worth knowing: stretching T strictly helps
        # the input limits and need not help the corridor at all, so the two
        # failures call for different fixes. Recorded rather than logged -- this
        # is library code with no logger, and the caller knows what it was flying.
        LAST_FAIL = ('accel/jerk' if not ok_input else 'hanh lang', T,
                     float(np.max(traj.peak_accel()
                                  / np.array([A_HORIZ_MAX, A_HORIZ_MAX,
                                              A_VERT_MAX]))),
                     float(np.max(traj.peak_jerk()
                                  / np.array([J_HORIZ_MAX, J_HORIZ_MAX,
                                              J_VERT_MAX]))))
        T *= growth
    return None, None


def _selfcheck():
    rng = np.random.default_rng(0)

    # --- boundary conditions are hit EXACTLY, both ends, all three axes ---
    for _ in range(200):
        p0, v0, a0 = rng.normal(size=3), rng.normal(size=3) * 0.5, rng.normal(size=3) * 0.2
        pf, vf, af = rng.normal(size=3), rng.normal(size=3) * 0.5, rng.normal(size=3) * 0.2
        T = float(rng.uniform(0.5, 4.0))
        tr = Trajectory(p0, v0, a0, pf, vf, af, T)
        assert np.allclose(tr.position(0.0), p0, atol=1e-9)
        assert np.allclose(tr.velocity(0.0), v0, atol=1e-9)
        assert np.allclose(tr.acceleration(0.0), a0, atol=1e-9)
        assert np.allclose(tr.position(T), pf, atol=1e-7), tr.position(T) - pf
        assert np.allclose(tr.velocity(T), vf, atol=1e-7)
        assert np.allclose(tr.acceleration(T), af, atol=1e-7)

    # --- the four polynomials really are each other's derivatives ---
    tr = Trajectory([0, 0, 2], [1, 0, 0], [0, 0, 0],
                    [5, 1, 3], [0.8, 0, 0], [0, 0, 0], 3.0)
    h = 1e-6
    for t in (0.3, 1.1, 2.7):
        assert np.allclose((tr.position(t + h) - tr.position(t - h)) / (2 * h),
                           tr.velocity(t), atol=1e-5)
        assert np.allclose((tr.velocity(t + h) - tr.velocity(t - h)) / (2 * h),
                           tr.acceleration(t), atol=1e-5)
        assert np.allclose((tr.acceleration(t + h) - tr.acceleration(t - h)) / (2 * h),
                           tr.jerk(t), atol=1e-5)

    # --- closed-form peaks match a dense sweep (this is what feasibility rests on) ---
    for _ in range(100):
        tr = Trajectory(rng.normal(size=3), rng.normal(size=3), rng.normal(size=3) * 0.3,
                        rng.normal(size=3), rng.normal(size=3), rng.normal(size=3) * 0.3,
                        float(rng.uniform(0.5, 3.0)))
        ts = np.linspace(0.0, tr.T, 4001)
        dense_a = np.abs(np.array([tr.acceleration(t) for t in ts])).max(axis=0)
        dense_j = np.abs(np.array([tr.jerk(t) for t in ts])).max(axis=0)
        assert np.all(tr.peak_accel() >= dense_a - 1e-6), 'peak_accel bo sot cuc tri'
        assert np.all(tr.peak_jerk() >= dense_j - 1e-6), 'peak_jerk bo sot cuc tri'
        assert np.allclose(tr.peak_accel(), dense_a, rtol=1e-3, atol=1e-3)
        assert np.allclose(tr.peak_jerk(), dense_j, rtol=1e-3, atol=1e-3)

    # --- cost() equals numerical integral of jerk^2 ---
    tr = Trajectory([0, 0, 0], [0, 0, 0], [0, 0, 0], [4, 1, 1], [0, 0, 0], [0, 0, 0], 2.5)
    ts = np.linspace(0.0, tr.T, 20001)
    num = np.trapz(np.sum(np.array([tr.jerk(t) for t in ts]) ** 2, axis=1), ts)
    assert abs(tr.cost() - num) / num < 1e-6, (tr.cost(), num)

    # --- stretching T always relaxes the demand (why solve() can just grow T) ---
    bc = ([0, 0, 3], [0, 0, 0], [0, 0, 0], [6, 0, 3], [0.8, 0, 0], [0, 0, 0])
    prev_a = prev_j = float('inf')
    for T in (1.0, 1.5, 2.5, 4.0, 7.0):
        tr = Trajectory(*bc, T)
        pa, pj = tr.peak_accel().max(), tr.peak_jerk().max()
        assert pa < prev_a and pj < prev_j, 'keo dai T ma khong giam ({} s)'.format(T)
        prev_a, prev_j = pa, pj

    # --- feasibility bites, and solve() escapes by stretching ---
    fast = Trajectory([0, 0, 3], [0, 0, 0], [0, 0, 0], [10, 0, 3], [0, 0, 0], [0, 0, 0], 1.0)
    assert not fast.input_feasible(), 'bay 10 m trong 1 s ma bao la kha thi?'
    tr, T = solve([0, 0, 3], [0, 0, 0], [0, 0, 0], [10, 0, 3], [0, 0, 0], [0, 0, 0], 1.0,
                  tries=12)
    assert tr is not None and T > 1.0, 'solve() phai noi dai T cho toi khi kha thi'
    assert tr.input_feasible()
    # and the limit it respects is ArduCopter's, not a thrust budget
    assert tr.peak_accel()[0] <= A_HORIZ_MAX + 1e-9
    assert tr.peak_jerk()[0] <= J_HORIZ_MAX + 1e-9

    # --- the VERTICAL limit is the tight one; a climb must be held to it ---
    tr, T = solve([0, 0, 1], [0, 0, 0], [0, 0, 0], [0, 0, 5], [0, 0, 0], [0, 0, 0], 1.5,
                  tries=15)
    assert tr is not None
    assert tr.peak_accel()[2] <= A_VERT_MAX + 1e-9, tr.peak_accel()
    assert A_VERT_MAX < A_HORIZ_MAX, 'WPNAV_ACCEL_Z chat hon WPNAV_ACCEL, dung quen'

    # --- the traverse case (plan par. 5.4): cross the gate centre with velocity
    # along the normal and ZERO lateral acceleration -- the thing v1 could not say
    n = np.array([1.0, 0.0, 0.0])
    C = np.array([5.0, 0.0, 3.5])
    v_tr = 0.8
    tr, T = solve(C - 1.5 * n, [0, 0, 0], [0, 0, 0], C, v_tr * n, [0, 0, 0], 2.5, tries=8)
    assert tr is not None
    assert np.allclose(tr.velocity(T), v_tr * n, atol=1e-6)
    assert np.allclose(tr.acceleration(T), [0, 0, 0], atol=1e-6), 'phai vao tam voi a=0'

    # --- segment B of the traverse is a straight CONSTANT-VELOCITY run, but only
    # when T is pinned to distance/speed (plan par. 5.4).
    #
    # Measured, correcting an earlier wrong claim: with the boundary conditions
    # collinear along n -- which they always are here, segment A hands over p=C,
    # v=v_tr*n, a=0 -- every off-axis axis is IDENTICALLY zero. There is no
    # sideways bulge and no overshoot past P_post. What a wrong T actually costs
    # is SPEED: stretch it 1.5x and the drone sags to 0.30 m/s inside the hole,
    # squeeze it to 0.6x and it spikes to 1.80 m/s. Both violate the a=0-at-centre
    # constraint that is the whole point of par. 5.4.
    P_post = C + 1.5 * n
    T_exact = float(np.linalg.norm(P_post - C) / v_tr)
    exact = Trajectory(C, v_tr * n, [0, 0, 0], P_post, v_tr * n, [0, 0, 0], T_exact)
    assert np.all(np.abs(exact.peak_accel()) < 1e-9), 'T dung phai cho gia toc = 0'
    ts = np.linspace(0.0, exact.T, 200)
    spd = np.array([exact.velocity(t)[0] for t in ts])
    assert np.allclose(spd, v_tr, atol=1e-9), 'T dung phai cho toc do HANG'
    for scale, expect in ((1.5, 'tut'), (0.6, 'vot')):
        w = Trajectory(C, v_tr * n, [0, 0, 0], P_post, v_tr * n, [0, 0, 0],
                       T_exact * scale)
        s = np.array([w.velocity(t)[0] for t in np.linspace(0.0, w.T, 200)])
        assert abs(s - v_tr).max() > 0.3 * v_tr, \
            'T sai {}x phai lam toc do {} ro ret'.format(scale, expect)
        # ...and still no lateral deviation, which is why the corridor check
        # would NOT have caught this. Speed is the symptom, not geometry.
        lat = np.array([w.position(t)[1:] for t in np.linspace(0.0, w.T, 200)])
        assert np.abs(lat - C[1:]).max() < 1e-12

    # --- corridor check does its job where the geometry IS off-axis ---
    assert corridor_clear(exact, C, P_post, 0.05)
    bulge, _ = solve(C, [0, 0.9, 0], [0, 0, 0], P_post, v_tr * n, [0, 0, 0], 2.0,
                     tries=1)
    if bulge is not None:                 # a sideways entry must leave the tube
        assert not corridor_clear(bulge, C, P_post, 0.05)

    print('mueller self-check OK (bien dung, dao ham khop, cuc tri kin, '
          'gioi han a={} j={} (ngang) a={} (doc))'
          .format(A_HORIZ_MAX, J_HORIZ_MAX, A_VERT_MAX))


if __name__ == '__main__':
    if '--selfcheck' in sys.argv:
        _selfcheck()
    else:
        print(__doc__)
