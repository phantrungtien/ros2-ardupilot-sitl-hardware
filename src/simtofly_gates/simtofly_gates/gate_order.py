#!/usr/bin/env python3
"""Which gate to fly next -- the one rule N4 and N6 must never disagree on.

N6 does not tell N4 which gate to chase; it only tells it which ones are
finished. Both then work out the target independently, so if the two rules ever
drift apart N6 times out a gate N4 was never flying at. That used to be two
copies of `min(left, key=distance)` in two files held together by a comment.
It is one function here instead.

The rule, in order:

  1. Gates IN FRONT first. "In front" is measured along the course axis --
     launch point to the centroid of every gate on the map -- so it needs no
     heading plumbing, only the positions both nodes already have.
  2. Within that group, nearest to the drone.
  3. Only when nothing is left in front, the gates behind.

Nearest-first alone flies the course as a zig-zag: on gates_course_hard it went
forward 5 m, back 11 m, forward 16 m, forward 6 m. Total distance is barely
different from the sensible order (38.8 m vs 38.1 m measured) -- distance is not
the reason. The reason is that each reversal is a ~180 deg turn, and a turn that
sharp is what makes trajectory_follower.build_segments infeasible at every
duration (see the hairpin note there). Nearest-first took two of them; front-
first takes one.

Run:  python3 gate_order.py --selfcheck
"""
import sys

import numpy as np

AXIS_MIN = 1.0             # m; shorter than this and the course has no axis --
                           # gates ringing the launch point average out to it --
                           # so "in front" is meaningless and we fall back to
                           # plain nearest rather than split on a rounding error.


def course_axis(centres, home):
    """Unit vector from the launch point towards the bulk of the course.

    HORIZONTAL. "In front" and "behind" are about which way the drone has to
    turn, and turning is a yaw: a gate 2.5 m higher than the rest is not in
    front of anything. Leaving z in tilted the axis of gates_course_hard to
    (0.82, 0.08, 0.56) purely because one gate sits at z = 6.

    None when the gates have no agreed direction (see AXIS_MIN).
    """
    if len(centres) == 0:
        return None
    v = np.mean(np.asarray(centres, float), axis=0) - np.asarray(home, float)
    v[2] = 0.0
    n = float(np.linalg.norm(v))
    return v / n if n >= AXIS_MIN else None


def pick_gate(centres, drone, home, all_centres=None):
    """Index into `centres` of the gate to fly next, or None if empty.

    `centres`    -- un-done gates, the ones we may pick from
    `all_centres`-- every gate on the map, done or not; the axis is a property
                    of the COURSE, so dropping flown gates from it would swing
                    the axis around as the run progresses and could re-classify
                    a gate from behind to in front halfway through.
    """
    centres = [np.asarray(c, float) for c in centres]
    if not centres:
        return None
    drone = np.asarray(drone, float)
    fwd = course_axis(all_centres if all_centres is not None else centres, home)
    pool = list(range(len(centres)))
    if fwd is not None:
        front = [i for i in pool
                 if (centres[i] - np.asarray(home, float)) @ fwd > 0.0]
        pool = front or pool
    return min(pool, key=lambda i: float(np.linalg.norm(centres[i] - drone)))


def _selfcheck():
    home = np.zeros(3)
    # gates_course_hard, as flown: three ahead (one of them high), one behind
    A = np.array([9.96, 2.06, 3.99])
    B = np.array([-6.0, 0.99, 3.50])
    C = np.array([4.99, 0.01, 5.99])
    D = np.array([15.04, -0.85, 2.97])
    every = [A, B, C, D]

    ax = course_axis(every, home)
    assert ax is not None and ax[0] > 0.9, ax        # course runs along +x

    # from the launch point the REAR gate B is nearer than A, and nearest-first
    # would take it. The rule must not.
    assert np.linalg.norm(B - home) < np.linalg.norm(A - home)
    assert pick_gate(every, home, home, every) == 2          # C, nearest ahead

    # walk the whole course and check B comes last
    order, left = [], list(every)
    drone = home.copy()
    while left:
        i = pick_gate(left, drone, home, every)
        drone = left[i]
        order.append(int(np.argmin([np.linalg.norm(g - drone) for g in every])))
        left.pop(i)
    assert order == [2, 0, 3, 1], order               # C, A, D, then B behind

    # exactly one reversal in that order, two in nearest-first
    def reversals(seq):
        pts = [home] + [every[i] for i in seq]
        legs = [pts[k + 1] - pts[k] for k in range(len(pts) - 1)]
        return sum(1 for k in range(len(legs) - 1)
                   if legs[k] @ legs[k + 1] < 0)
    assert reversals(order) == 1, reversals(order)
    assert reversals([2, 1, 0, 3]) == 2               # what nearest-first did

    # no axis -> plain nearest, and never a crash
    assert course_axis([np.array([0.4, 0, 0]), np.array([-0.4, 0, 0])],
                       home) is None
    assert pick_gate([], home, home) is None
    assert pick_gate([A], home, home, [A]) == 0
    print('gate_order self-check OK (thu tu C,A,D,B -- 1 lan quay dau)')


if __name__ == '__main__':
    if '--selfcheck' in sys.argv:
        _selfcheck()
    else:
        print(__doc__)
