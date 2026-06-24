#!/usr/bin/env python3
"""Velocity-controlled circular flight (Task 3).

Flies N circles by STREAMING velocity + tangent yaw on
/mavros/setpoint_raw/local, driven by the pure geometric controller in
circle_control.py (feedforward tangent at the actual position + radial P
correction; laps counted from the real swept angle).

Frame confirmed in SITL with verify_frame:
    velocity : command vx -> +East, vy -> +North  (clean ENU, == pose frame)
               so circle_setpoint's ENU output is published directly.
    yaw      : absolute tangent heading. CONFIRMED in SITL -- the nose
               tracks the travel direction with the clean default mapping
               (sp.yaw = enu_tangent). yaw always rides WITH a velocity
               setpoint, so ArduPilot honours it. YAW_SIGN / YAW_OFFSET stay
               as tunables in case a different airframe/firmware differs.

Flow: wait FCU -> GUIDED -> arm (retry) -> takeoff -> build circle from the
hover pose -> stream the circle -> RTL when done. Ctrl+C -> RTL.

SAFETY: test SITL -> HITL -> outdoor, safety pilot on RC ready to retake.
Pilot leaving GUIDED releases control (streaming stops). Altitude ceiling
MAX_ALT. Run:
    ros2 launch simtofly_mavros_sitl sitl_mission.launch.py mission:=mission_circle
"""

import math
import time

import rclpy
from mavros_msgs.msg import PositionTarget

from simtofly_mavros_sitl.circle_control import (
    CircleParams,
    circle_setpoint,
    make_circle,
    wrap_to_pi,
)
from simtofly_mavros_sitl.guided_base import GuidedNode

# ===== circle parameters =====
PARAMS = CircleParams(
    radius=5.0,        # R (m)
    speed=2.0,         # V (m/s) -> omega=V/R=0.4 rad/s, period~15.7 s
    direction=1.0,     # +1 CCW, -1 CW
    altitude=10.0,     # hold altitude (m)
    laps=2.0,          # number of full circles
    kp_r=0.8,          # radial P gain (1/s)
    kp_z=0.5,          # altitude P gain (1/s)
    vmax=3.0,          # max horizontal speed (m/s)
    vz_max=1.0,        # max vertical speed (m/s)
)
# Physical limit reminder: a = V^2/R, bank angle ~ atan(a/g).
#   default a=0.8 m/s^2 -> ~4.7deg (gentle). Raising V or lowering R grows
#   the bank fast -- keep it flyable.

CTRL_HZ = 30.0         # setpoint stream rate
MAX_ALT = 30.0         # altitude ceiling (m) -> hover if exceeded
RAMP_SEC = 2.0         # ease horizontal speed 0->full to avoid a start jerk

# yaw command convention for setpoint_raw/local (SITL-confirmed defaults):
#   sp.yaw = wrap_to_pi(YAW_SIGN * enu_tangent + YAW_OFFSET)
# Verified: nose stays tangent with SIGN=+1, OFFSET=0. Kept tunable in case
# another airframe/firmware needs it (constant lag -> YAW_OFFSET; nose
# counter-rotates -> flip YAW_SIGN).
YAW_SIGN = 1.0
YAW_OFFSET = 0.0

# velocity + absolute-yaw setpoint mask (ignore position, accel, yaw_rate).
_MASK = (
    PositionTarget.IGNORE_PX | PositionTarget.IGNORE_PY
    | PositionTarget.IGNORE_PZ
    | PositionTarget.IGNORE_AFX | PositionTarget.IGNORE_AFY
    | PositionTarget.IGNORE_AFZ
    | PositionTarget.IGNORE_YAW_RATE
)


class CircleMission(GuidedNode):
    def __init__(self):
        super().__init__('circle_mission')
        self.sp_pub = self.create_publisher(
            PositionTarget, '/mavros/setpoint_raw/local', 10)
        self.circle = None
        self.acc_angle = 0.0
        self.phi_prev = 0.0
        self.active = False
        self.done = False
        self.start_time = 0.0
        self._warned_mode = False
        self._tick = 0
        self.timer = self.create_timer(1.0 / CTRL_HZ, self._control_loop)

    def _cmd_yaw(self, enu_tangent):
        """Map the desired ENU tangent heading to the setpoint yaw field.
        enu_tangent None (at centre) -> hold the current heading."""
        if enu_tangent is None:
            enu_tangent = self.heading_enu()
        return wrap_to_pi(YAW_SIGN * enu_tangent + YAW_OFFSET)

    def _publish(self, vx, vy, vz, enu_tangent):
        sp = PositionTarget()
        sp.header.stamp = self.get_clock().now().to_msg()
        sp.coordinate_frame = PositionTarget.FRAME_LOCAL_NED
        sp.type_mask = _MASK
        sp.velocity.x = float(vx)   # +East  (confirmed in SITL)
        sp.velocity.y = float(vy)   # +North (confirmed in SITL)
        sp.velocity.z = float(vz)
        sp.yaw = float(self._cmd_yaw(enu_tangent))
        self.sp_pub.publish(sp)

    def _control_loop(self):
        if not self.active or self.done:
            return

        # Pilot took over (left GUIDED) -> release, stop streaming.
        if self.state.mode != 'GUIDED':
            if not self._warned_mode:
                self.get_logger().warn(
                    f'mode={self.state.mode} != GUIDED -> release control')
                self._warned_mode = True
            return
        self._warned_mode = False

        alt = self.altitude
        if alt > MAX_ALT:
            self.get_logger().warn(f'alt {alt:.1f} > MAX_ALT -> hover')
            self._publish(0.0, 0.0, 0.0, None)
            return

        pcur = (self.pose.pose.position.x, self.pose.pose.position.y)
        vx, vy, vz, yaw, self.acc_angle, self.phi_prev, done = circle_setpoint(
            pcur, alt, self.circle, PARAMS, self.acc_angle, self.phi_prev)

        # Ease horizontal speed in over RAMP_SEC so we don't step-command V.
        ramp = min(1.0, (time.time() - self.start_time) / RAMP_SEC)
        self._publish(vx * ramp, vy * ramp, vz, yaw)

        self._tick += 1
        if self._tick % int(CTRL_HZ) == 0:   # ~1 Hz progress line
            laps = abs(self.acc_angle) / (2.0 * math.pi)
            print(f'  laps {laps:.2f}/{PARAMS.laps:.0f} | '
                  f'alt {alt:.1f}m | pos ({pcur[0]:.1f}, {pcur[1]:.1f})')

        if done:
            self.done = True
            self.active = False
            print('\n✓ circle complete -> RTL')
            self.set_mode('RTL')

    def startup(self):
        print('=' * 56)
        print(f'CIRCLE: R={PARAMS.radius}m V={PARAMS.speed}m/s '
              f'N={PARAMS.laps:.0f} dir={"CCW" if PARAMS.direction > 0 else "CW"}')
        print('=' * 56)

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
        if not self.takeoff(PARAMS.altitude):
            print('x takeoff rejected')
            return False
        if not self.wait_for_altitude(PARAMS.altitude):
            print('x did not reach takeoff altitude')
            return False
        print(f'✓ at {PARAMS.altitude}m')
        time.sleep(2)

        # Build the circle from the hover pose: centre offset perpendicular
        # to the current heading so the first tangent points forward (no
        # position jump). phi0/r_eff are derived inside make_circle.
        p0 = (self.pose.pose.position.x, self.pose.pose.position.y)
        self.circle = make_circle(p0, self.heading_enu(), PARAMS)
        self.phi_prev = self.circle.phi0
        self.acc_angle = 0.0
        self.start_time = time.time()
        self.active = True
        print('✓ flying circle (Ctrl+C -> RTL)\n')
        return True

    def safe_shutdown(self):
        try:
            if self.state.armed and not self.done:
                print('\nRTL...')
                self.set_mode('RTL')
        except Exception:
            pass


def main():
    rclpy.init()
    node = CircleMission()
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
