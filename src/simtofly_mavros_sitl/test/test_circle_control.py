#!/usr/bin/env python3
"""Unit tests for the pure circle_control geometry (no ROS needed).

Run standalone:
    PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 \\
    PYTHONPATH=src/simtofly_mavros_sitl \\
    python3 -m pytest src/simtofly_mavros_sitl/test/test_circle_control.py
"""

import math

import pytest

from simtofly_mavros_sitl.circle_control import (
    Circle,
    CircleParams,
    circle_setpoint,
    clamp_magnitude,
    make_circle,
    tangent_unit,
    wrap_to_pi,
)


def test_make_circle_starts_on_circle():
    # r_eff equals the requested radius and the start point is exactly on
    # the circle (Pd(0) == p0), so there is no position-error spike at t=0.
    p = CircleParams(radius=5.0, direction=1.0)
    c = make_circle((10.0, 20.0), heading=0.0, params=p)
    assert math.isclose(c.r_eff, 5.0, abs_tol=1e-9)
    sx = c.cx + c.r_eff * math.cos(c.phi0)
    sy = c.cy + c.r_eff * math.sin(c.phi0)
    assert math.isclose(sx, 10.0, abs_tol=1e-9)
    assert math.isclose(sy, 20.0, abs_tol=1e-9)


def test_start_tangent_matches_heading_ccw():
    # The first feedforward must point along `heading` for any heading, so
    # the drone curves into the circle instead of jumping sideways.
    p = CircleParams(radius=5.0, speed=2.0, direction=1.0)
    for heading in (0.0, 0.7, -1.2, math.pi / 2):
        c = make_circle((0.0, 0.0), heading=heading, params=p)
        vx, vy, _vz, yaw, _a, _phi, _done = circle_setpoint(
            (0.0, 0.0), 0.0, c, p, acc_angle=0.0, phi_prev=c.phi0)
        assert math.isclose(math.atan2(vy, vx), heading, abs_tol=1e-6)
        assert math.isclose(yaw, heading, abs_tol=1e-6)


def test_cw_tangent_is_opposite_sign():
    ccw = tangent_unit(1.0, 0.0, 1.0)
    cw = tangent_unit(1.0, 0.0, -1.0)
    assert ccw == (pytest.approx(0.0), pytest.approx(1.0))
    assert cw == (pytest.approx(0.0), pytest.approx(-1.0))


def test_tangent_perpendicular_to_radial():
    rx, ry = 0.6, 0.8  # a unit radial vector
    tx, ty = tangent_unit(rx, ry, 1.0)
    assert math.isclose(rx * tx + ry * ty, 0.0, abs_tol=1e-9)


def test_radial_correction_pulls_inward_when_outside():
    # speed=0 isolates the radial correction term. Outside the circle the
    # command must point back toward the centre.
    p = CircleParams(speed=0.0, kp_r=1.0, direction=1.0, vmax=100.0)
    c = Circle(cx=0.0, cy=0.0, r_eff=5.0, phi0=0.0)
    vx, vy, _vz, _yaw, _a, _phi, _done = circle_setpoint(
        (6.0, 0.0), 0.0, c, p, acc_angle=0.0, phi_prev=0.0)
    assert vx < 0.0
    assert math.isclose(vy, 0.0, abs_tol=1e-9)


def test_radial_correction_pushes_outward_when_inside():
    p = CircleParams(speed=0.0, kp_r=1.0, direction=1.0, vmax=100.0)
    c = Circle(cx=0.0, cy=0.0, r_eff=5.0, phi0=0.0)
    vx, _vy, _vz, _yaw, _a, _phi, _done = circle_setpoint(
        (4.0, 0.0), 0.0, c, p, acc_angle=0.0, phi_prev=0.0)
    assert vx > 0.0


def test_clamp_magnitude_preserves_direction():
    vx, vy = clamp_magnitude(30.0, 40.0, 5.0)   # |v| 50 -> 5
    assert math.isclose(math.hypot(vx, vy), 5.0, abs_tol=1e-9)
    assert math.isclose(vy / vx, 40.0 / 30.0, abs_tol=1e-9)


def test_clamp_magnitude_below_limit_unchanged():
    assert clamp_magnitude(1.0, 2.0, 5.0) == (1.0, 2.0)


def test_wrap_to_pi_range():
    # Full turns cancel, sub-pi angles pass through untouched.
    assert math.isclose(wrap_to_pi(0.5), 0.5, abs_tol=1e-9)
    assert math.isclose(wrap_to_pi(2 * math.pi + 0.5), 0.5, abs_tol=1e-9)
    assert math.isclose(wrap_to_pi(-2 * math.pi - 0.5), -0.5, abs_tol=1e-9)
    # At the +-pi boundary either sign is the same point; |result| == pi.
    assert math.isclose(abs(wrap_to_pi(3 * math.pi)), math.pi, abs_tol=1e-9)
    # Result always stays within [-pi, pi].
    for a in (-10.0, -1.0, 0.0, 1.0, 7.3, 100.0):
        assert -math.pi - 1e-9 <= wrap_to_pi(a) <= math.pi + 1e-9


def test_altitude_hold_velocity_clamped():
    p = CircleParams(altitude=10.0, kp_z=0.5, vz_max=1.0, speed=0.0,
                     kp_r=0.0, vmax=100.0)
    c = Circle(cx=0.0, cy=0.0, r_eff=5.0, phi0=0.0)
    # 8 m below target -> 0.5*8 = 4 -> clamped to vz_max = 1.
    _vx, _vy, vz, _yaw, _a, _phi, _done = circle_setpoint(
        (5.0, 0.0), 2.0, c, p, 0.0, 0.0)
    assert math.isclose(vz, 1.0, abs_tol=1e-9)
    # 1 m below target -> 0.5, not clamped.
    _vx, _vy, vz, _yaw, _a, _phi, _done = circle_setpoint(
        (5.0, 0.0), 9.0, c, p, 0.0, 0.0)
    assert math.isclose(vz, 0.5, abs_tol=1e-9)


def test_lap_counting_done_after_n_laps():
    # Walk a point around the circle in 0.5 deg steps and integrate the
    # REAL swept angle; `done` must fire near exactly 2 full laps.
    p = CircleParams(radius=5.0, speed=2.0, direction=1.0, laps=2.0,
                     kp_r=0.0)
    c = Circle(cx=0.0, cy=0.0, r_eff=5.0, phi0=0.0)
    acc, phi_prev, done, steps = 0.0, 0.0, False, 0
    while not done and steps < 5000:
        steps += 1
        ang = math.radians(0.5) * steps
        px, py = 5.0 * math.cos(ang), 5.0 * math.sin(ang)
        _vx, _vy, _vz, _yaw, acc, phi_prev, done = circle_setpoint(
            (px, py), 0.0, c, p, acc, phi_prev)
    assert done
    assert math.isclose(abs(acc), 4.0 * math.pi, abs_tol=math.radians(1.0))


def test_at_centre_hovers():
    p = CircleParams(radius=5.0, speed=2.0)
    c = make_circle((0.0, 0.0), 0.0, p)
    vx, vy, _vz, yaw, _a, _phi, done = circle_setpoint(
        (c.cx, c.cy), 0.0, c, p, 0.0, 0.0)
    assert (vx, vy) == (0.0, 0.0)
    assert yaw is None
    assert not done
