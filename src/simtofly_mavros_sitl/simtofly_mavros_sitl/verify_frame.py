#!/usr/bin/env python3
"""Frame-convention check for /mavros/setpoint_raw/local (Task 1).

Before trusting the circular-flight maths we must KNOW, empirically in
SITL, how a velocity/yaw command on /mavros/setpoint_raw/local with
coordinate_frame=FRAME_LOCAL_NED maps onto the ENU pose that MAVROS
publishes on /mavros/local_position/pose. MAVROS converts ENU<->NED
internally, so this is easiest to confirm by commanding and measuring.

What it does (auto): GUIDED -> arm -> takeoff -> three probes -> RTL.
    Probe A: command velocity x = +SPEED  -> measure pose delta
    Probe B: command velocity y = +SPEED  -> measure pose delta
    Probe C: command absolute yaw = 0      -> measure resulting heading

Read the printed summary to fill in the convention used by mission_circle:
    "command vx>0 moves pose +X (East)?  command yaw=0 -> heading 0 (East)?"

SAFETY: keep an RC/QGC ready to take over. Ctrl+C triggers RTL. If the
pilot leaves GUIDED, the script stops streaming immediately.
"""

import math
import time

import rclpy
from mavros_msgs.msg import PositionTarget

from simtofly_mavros_sitl.guided_base import GuidedNode

SPEED = 1.0        # probe velocity (m/s)
PROBE_SEC = 4.0    # how long to hold each velocity probe (s)
SETTLE_SEC = 2.0   # zero-velocity settle between probes (s)
TAKEOFF_ALT = 10.0
STREAM_HZ = 20.0

# Masks: velocity (+ yaw_rate) probe, and absolute-yaw probe.
_IGNORE_POS = (PositionTarget.IGNORE_PX | PositionTarget.IGNORE_PY
               | PositionTarget.IGNORE_PZ)
_IGNORE_ACC = (PositionTarget.IGNORE_AFX | PositionTarget.IGNORE_AFY
               | PositionTarget.IGNORE_AFZ)
MASK_VEL = _IGNORE_POS | _IGNORE_ACC | PositionTarget.IGNORE_YAW
MASK_YAW = (_IGNORE_POS | _IGNORE_ACC
            | PositionTarget.IGNORE_VX | PositionTarget.IGNORE_VY
            | PositionTarget.IGNORE_VZ | PositionTarget.IGNORE_YAW_RATE)


def yaw_from_quat(q):
    """ENU yaw (rad) from a geometry_msgs quaternion."""
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class FrameCheck(GuidedNode):
    def __init__(self):
        super().__init__('frame_check')
        self.sp_pub = self.create_publisher(
            PositionTarget, '/mavros/setpoint_raw/local', 10)

    def _xy(self):
        return self.pose.pose.position.x, self.pose.pose.position.y

    def _publish_velocity(self, vx, vy, vz):
        sp = PositionTarget()
        sp.header.stamp = self.get_clock().now().to_msg()
        sp.coordinate_frame = PositionTarget.FRAME_LOCAL_NED
        sp.type_mask = MASK_VEL
        sp.velocity.x = float(vx)
        sp.velocity.y = float(vy)
        sp.velocity.z = float(vz)
        sp.yaw_rate = 0.0
        self.sp_pub.publish(sp)

    def _publish_yaw(self, yaw):
        sp = PositionTarget()
        sp.header.stamp = self.get_clock().now().to_msg()
        sp.coordinate_frame = PositionTarget.FRAME_LOCAL_NED
        sp.type_mask = MASK_YAW
        sp.yaw = float(yaw)
        self.sp_pub.publish(sp)

    def _stream(self, publish, duration):
        """Stream a setpoint at STREAM_HZ for `duration`; abort if the
        pilot leaves GUIDED. Returns True if it ran to completion."""
        end = time.time() + duration
        period = 1.0 / STREAM_HZ
        while rclpy.ok() and time.time() < end:
            if self.state.mode != 'GUIDED':
                self.get_logger().warn(
                    f'mode={self.state.mode} != GUIDED -> abort probe')
                return False
            publish()
            rclpy.spin_once(self, timeout_sec=period)
        return True

    def _probe_velocity(self, name, vx, vy):
        x0, y0 = self._xy()
        self._stream(lambda: self._publish_velocity(vx, vy, 0.0), PROBE_SEC)
        x1, y1 = self._xy()
        dx, dy = x1 - x0, y1 - y0
        print(f'[{name}] command vel=({vx:+.1f},{vy:+.1f}) -> '
              f'pose delta dX(East)={dx:+.2f} m, dY(North)={dy:+.2f} m')
        self._stream(lambda: self._publish_velocity(0.0, 0.0, 0.0), SETTLE_SEC)
        return dx, dy

    def run(self):
        print('=' * 60)
        print('FRAME CHECK: setpoint_raw/local (FRAME_LOCAL_NED) vs ENU pose')
        print('=' * 60)
        if not self.wait_for_services():
            print('x services not available')
            return
        if not self.wait_for_connection():
            print('x FCU not connected')
            return
        if not self.set_mode('GUIDED'):
            print('x could not set GUIDED')
            return
        time.sleep(1)
        if not self.arm_with_retry():
            print('x could not arm (prearm/EKF/GPS)')
            return
        time.sleep(2)
        if not self.takeoff(TAKEOFF_ALT):
            print('x takeoff command rejected')
            return
        if not self.wait_for_altitude(TAKEOFF_ALT):
            print('x did not reach takeoff altitude')
            return
        time.sleep(2)

        print('\n--- probing (watch the SITL map too) ---')
        ax_dx, ax_dy = self._probe_velocity('A vx+', SPEED, 0.0)
        bx_dx, bx_dy = self._probe_velocity('B vy+', 0.0, SPEED)

        # Probe C: hold yaw=0 and read the resulting heading.
        self._stream(lambda: self._publish_yaw(0.0), PROBE_SEC)
        heading = math.degrees(yaw_from_quat(self.pose.pose.orientation))
        print(f'[C yaw0] command yaw=0 rad -> heading={heading:+.1f} deg '
              f'(ENU: 0=East, 90=North)')

        print('\n' + '=' * 60)
        print('CONCLUSION (fill into mission_circle):')
        print(f'  vx+ -> dominant axis: '
              f'{"X/East" if abs(ax_dx) > abs(ax_dy) else "Y/North"} '
              f'(sign {"+" if (ax_dx or ax_dy) > 0 else "-"})')
        print(f'  vy+ -> dominant axis: '
              f'{"Y/North" if abs(bx_dy) > abs(bx_dx) else "X/East"} '
              f'(sign {"+" if (bx_dy or bx_dx) > 0 else "-"})')
        print(f'  yaw=0 command points the nose toward heading {heading:+.1f} '
              f'deg ENU')
        print('=' * 60)

    def rtl(self):
        try:
            if self.state.armed:
                print('\nRTL...')
                self.set_mode('RTL')
        except Exception:
            pass


def main():
    rclpy.init()
    node = FrameCheck()
    try:
        node.run()
    except KeyboardInterrupt:
        print('\ninterrupted by user')
    finally:
        node.rtl()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
