#!/usr/bin/env python3
"""Pure geometry for velocity-based circular flight.

No ROS imports here: this module is importable and unit-testable anywhere
(plain python3 + math). All vectors are 2D in the local horizontal plane,
expressed in ENU (x = East, y = North) -- the same frame MAVROS publishes
on /mavros/local_position/pose. The companion ROS node feeds the latest
pose in and publishes the velocity/yaw this returns.

Control law (geometric, time-free):
    - feedforward: constant-speed tangent computed AT THE DRONE'S ACTUAL
      angular position (no time-parametrised target to chase -> no phase
      lag, no corner-cutting).
    - correction: a pure RADIAL P-term pulls the drone back onto the
      circle (e_r = rho - R). Tangential motion is left entirely to the
      feedforward.
    - laps are counted from the REAL swept angle (unwrapped), not a timer,
      so "done" means the vehicle actually flew N circles.
"""

import math
from dataclasses import dataclass


@dataclass
class CircleParams:
    """Tunables for one circular-flight run."""

    radius: float = 5.0       # R (m)
    speed: float = 2.0        # V, tangential speed (m/s)
    direction: float = 1.0    # +1 = CCW, -1 = CW
    altitude: float = 10.0    # altitude to hold (m)
    laps: float = 2.0         # number of full circles before done
    kp_r: float = 0.8         # radial P gain (1/s)
    kp_z: float = 0.5         # altitude P gain (1/s)
    vmax: float = 3.0         # max horizontal speed (m/s)
    vz_max: float = 1.0       # max vertical speed (m/s)


@dataclass
class Circle:
    """Circle geometry derived from the start pose (see make_circle)."""

    cx: float
    cy: float
    r_eff: float
    phi0: float


def wrap_to_pi(angle):
    """Wrap an angle (rad) to the half-open range [-pi, pi).

    The boundary representation (-pi vs +pi for the same point) is
    irrelevant to lap counting: each per-tick delta is far smaller than
    pi, so the wrap never lands on the boundary in practice.
    """
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


def clamp(value, lo, hi):
    """Clamp a scalar to [lo, hi]."""
    return max(lo, min(hi, value))


def clamp_magnitude(vx, vy, vmax):
    """Scale (vx, vy) so |v| <= vmax, KEEPING its direction.

    Clipping each axis on its own would skew the velocity vector when the
    P-term spikes at startup, so we scale the whole vector instead.
    """
    mag = math.hypot(vx, vy)
    if mag > vmax and mag > 1e-9:
        scale = vmax / mag
        return vx * scale, vy * scale
    return vx, vy


def tangent_unit(rx, ry, direction):
    """Unit tangent from a unit radial vector.

    Rotating the radial +90 deg gives the CCW tangent; `direction` flips it
    to CW. Result is perpendicular to (rx, ry).
    """
    return -ry * direction, rx * direction


def make_circle(p0, heading, params):
    """Derive circle geometry so the drone starts exactly ON the circle.

    p0      : (x, y) recorded while hovering (velocity ~ 0).
    heading : current yaw (rad, ENU: 0 = +x/East, CCW positive).
    params  : CircleParams (uses radius, direction).

    The centre is offset perpendicular to `heading` -- LEFT for CCW, RIGHT
    for CW -- so the first tangent points along `heading` and the drone
    curves into the circle with no position jump at t=0.

    r_eff and phi0 are DERIVED from the resulting centre (not assumed equal
    to radius / 0), so Pd(0) == p0 holds exactly for any offset choice.
    """
    x0, y0 = p0
    d = params.direction
    nx = -math.sin(heading) * d
    ny = math.cos(heading) * d
    cx = x0 + params.radius * nx
    cy = y0 + params.radius * ny
    dx, dy = x0 - cx, y0 - cy
    return Circle(cx=cx, cy=cy,
                  r_eff=math.hypot(dx, dy),
                  phi0=math.atan2(dy, dx))


def circle_setpoint(pcur, z, circle, params, acc_angle, phi_prev):
    """Compute one control tick (pure -- no side effects).

    Inputs:
        pcur      : (x, y) current position, ENU (m).
        z         : current altitude (m).
        circle    : Circle geometry from make_circle.
        params    : CircleParams.
        acc_angle : accumulated swept angle so far (rad).
        phi_prev  : angle (centre->drone) from the previous tick (rad).

    Returns (vx, vy, vz, yaw, acc_angle, phi, done):
        vx, vy    : horizontal velocity command, same frame as pcur (ENU).
        vz        : vertical velocity command (holds params.altitude).
        yaw       : absolute heading = tangent direction; None at centre.
        acc_angle : updated swept-angle accumulator -- feed back next tick.
        phi       : current centre->drone angle -- feed back as phi_prev.
        done      : True once |acc_angle| >= 2*pi*laps (real angle flown).
    """
    x, y = pcur
    dx, dy = x - circle.cx, y - circle.cy
    rho = math.hypot(dx, dy)
    if rho < 1e-6:
        # Sitting on the centre: radial direction is undefined. Hover and
        # keep the lap count untouched.
        return 0.0, 0.0, 0.0, None, acc_angle, phi_prev, False

    rx, ry = dx / rho, dy / rho
    tx, ty = tangent_unit(rx, ry, params.direction)

    vff_x = params.speed * tx
    vff_y = params.speed * ty

    e_r = rho - circle.r_eff               # >0 outside the circle
    vcorr_x = -params.kp_r * e_r * rx      # pull back toward the circle
    vcorr_y = -params.kp_r * e_r * ry

    vx, vy = clamp_magnitude(vff_x + vcorr_x, vff_y + vcorr_y, params.vmax)
    vz = clamp(params.kp_z * (params.altitude - z),
               -params.vz_max, params.vz_max)
    yaw = math.atan2(vff_y, vff_x)

    phi = math.atan2(dy, dx)
    acc_angle = acc_angle + wrap_to_pi(phi - phi_prev)
    done = abs(acc_angle) >= 2.0 * math.pi * params.laps
    return vx, vy, vz, yaw, acc_angle, phi, done


# =========================================================================
# self-check -- pure geometry, no ROS:  python3 circle_control.py --selfcheck
#
# These twelve checks were a pytest file under test/. They moved here because
# they were the only pytest in the workspace: running them needed
# PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 and a PYTHONPATH, while every other module
# answers --selfcheck with plain python3. Same assertions, one convention.
# =========================================================================
def _selfcheck():
    def close(a, b, tol=1e-9):
        return math.isclose(a, b, abs_tol=tol)

    # the start point is exactly ON the circle, so t=0 has no error spike
    p = CircleParams(radius=5.0, direction=1.0)
    c = make_circle((10.0, 20.0), heading=0.0, params=p)
    assert close(c.r_eff, 5.0)
    assert close(c.cx + c.r_eff * math.cos(c.phi0), 10.0)
    assert close(c.cy + c.r_eff * math.sin(c.phi0), 20.0)

    # the first feedforward points along `heading` for ANY heading, so the
    # drone curves into the circle instead of jumping sideways
    p = CircleParams(radius=5.0, speed=2.0, direction=1.0)
    for heading in (0.0, 0.7, -1.2, math.pi / 2):
        c = make_circle((0.0, 0.0), heading=heading, params=p)
        vx, vy, _vz, yaw, _a, _phi, _done = circle_setpoint(
            (0.0, 0.0), 0.0, c, p, acc_angle=0.0, phi_prev=c.phi0)
        assert close(math.atan2(vy, vx), heading, 1e-6), heading
        assert close(yaw, heading, 1e-6), heading

    # tangent: reversing direction flips it, and it is always perpendicular
    # to the radial
    assert tuple(round(v, 12) for v in tangent_unit(1.0, 0.0, 1.0)) == (0.0, 1.0)
    assert tuple(round(v, 12) for v in tangent_unit(1.0, 0.0, -1.0)) == (0.0, -1.0)
    tx, ty = tangent_unit(0.6, 0.8, 1.0)
    assert close(0.6 * tx + 0.8 * ty, 0.0)

    # speed=0 isolates the radial term: outside pulls in, inside pushes out
    p = CircleParams(speed=0.0, kp_r=1.0, direction=1.0, vmax=100.0)
    c = Circle(cx=0.0, cy=0.0, r_eff=5.0, phi0=0.0)
    vx, vy, *_ = circle_setpoint((6.0, 0.0), 0.0, c, p, 0.0, 0.0)
    assert vx < 0.0 and close(vy, 0.0)
    vx, *_ = circle_setpoint((4.0, 0.0), 0.0, c, p, 0.0, 0.0)
    assert vx > 0.0

    # clamping keeps the direction and leaves anything under the limit alone
    vx, vy = clamp_magnitude(30.0, 40.0, 5.0)          # |v| 50 -> 5
    assert close(math.hypot(vx, vy), 5.0) and close(vy / vx, 40.0 / 30.0)
    assert clamp_magnitude(1.0, 2.0, 5.0) == (1.0, 2.0)

    # wrap_to_pi: whole turns cancel, and the result never leaves [-pi, pi]
    assert close(wrap_to_pi(0.5), 0.5)
    assert close(wrap_to_pi(2 * math.pi + 0.5), 0.5)
    assert close(wrap_to_pi(-2 * math.pi - 0.5), -0.5)
    assert close(abs(wrap_to_pi(3 * math.pi)), math.pi)
    for a in (-10.0, -1.0, 0.0, 1.0, 7.3, 100.0):
        assert -math.pi - 1e-9 <= wrap_to_pi(a) <= math.pi + 1e-9, a

    # altitude hold is clamped at vz_max but proportional below it
    p = CircleParams(altitude=10.0, kp_z=0.5, vz_max=1.0, speed=0.0,
                     kp_r=0.0, vmax=100.0)
    c = Circle(cx=0.0, cy=0.0, r_eff=5.0, phi0=0.0)
    assert close(circle_setpoint((5.0, 0.0), 2.0, c, p, 0.0, 0.0)[2], 1.0)
    assert close(circle_setpoint((5.0, 0.0), 9.0, c, p, 0.0, 0.0)[2], 0.5)

    # walk the circle in 0.5 deg steps and integrate the REAL swept angle:
    # `done` must fire at two full laps, not one step early or late
    p = CircleParams(radius=5.0, speed=2.0, direction=1.0, laps=2.0, kp_r=0.0)
    c = Circle(cx=0.0, cy=0.0, r_eff=5.0, phi0=0.0)
    acc, phi_prev, done, steps = 0.0, 0.0, False, 0
    while not done and steps < 5000:
        steps += 1
        ang = math.radians(0.5) * steps
        *_, acc, phi_prev, done = circle_setpoint(
            (5.0 * math.cos(ang), 5.0 * math.sin(ang)), 0.0, c, p, acc, phi_prev)
    assert done, 'khong bao gio bao xong 2 vong'
    assert close(abs(acc), 4.0 * math.pi, math.radians(1.0)), acc

    # at the centre the radial is undefined, so hold still rather than guess
    p = CircleParams(radius=5.0, speed=2.0)
    c = make_circle((0.0, 0.0), 0.0, p)
    vx, vy, _vz, yaw, _a, _phi, done = circle_setpoint(
        (c.cx, c.cy), 0.0, c, p, 0.0, 0.0)
    assert (vx, vy) == (0.0, 0.0) and yaw is None and not done

    print('circle_control selfcheck OK (12 kiem tra hinh hoc)')


if __name__ == '__main__':
    import sys
    if '--selfcheck' in sys.argv:
        _selfcheck()
