#!/usr/bin/env python3
"""
Simple Working Mission Script
------------------------------
Author: phantrungtien
GitHub: https://github.com/phantrungtien

Mission Steps:
1. Set GUIDED mode
2. Arm throttle
3. Takeoff to 10m
4. Move forward 10m
5. Wait 3 seconds
6. Return to Launch (RTL)
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, SetMode, CommandTOL
from geometry_msgs.msg import PoseStamped
import time


class SimpleMission(Node):
    def __init__(self):
        super().__init__('simple_mission')

        # QoS profile for MAVROS topics (BEST_EFFORT reliability)
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # Subscribers
        self.state_sub = self.create_subscription(
            State,
            '/mavros/state',
            self.state_callback,
            qos_profile
        )

        self.local_pos_sub = self.create_subscription(
            PoseStamped,
            '/mavros/local_position/pose',
            self.local_pos_callback,
            qos_profile
        )

        # Publisher for setpoints (will only be used AFTER takeoff)
        self.setpoint_pub = self.create_publisher(
            PoseStamped,
            '/mavros/setpoint_position/local',
            10
        )

        # Service clients
        self.arming_client = self.create_client(CommandBool, '/mavros/cmd/arming')
        self.set_mode_client = self.create_client(SetMode, '/mavros/set_mode')
        self.takeoff_client = self.create_client(CommandTOL, '/mavros/cmd/takeoff')

        # State variables
        self.current_state = State()
        self.current_pose = PoseStamped()

        # Wait for services
        print('Waiting for services...')
        self.arming_client.wait_for_service(timeout_sec=10.0)
        self.set_mode_client.wait_for_service(timeout_sec=10.0)
        self.takeoff_client.wait_for_service(timeout_sec=10.0)
        print('✓ Services ready!\n')

    def state_callback(self, msg):
        self.current_state = msg

    def local_pos_callback(self, msg):
        self.current_pose = msg

    def set_mode(self, mode):
        """Set flight mode"""
        print(f'Setting {mode} mode...')
        req = SetMode.Request()
        req.custom_mode = mode

        future = self.set_mode_client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)

        if future.result() and future.result().mode_sent:
            print(f'✓ {mode} mode set\n')
            return True
        else:
            print(f'✗ Failed to set {mode} mode\n')
            return False

    def arm(self, verbose=True):
        """Arm the vehicle (one attempt). ArduPilot rejects arming until prearm
        checks pass, so callers may need to retry (see arm_with_retry)."""
        if verbose:
            print('Arming throttle...')
        req = CommandBool.Request()
        req.value = True

        future = self.arming_client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)

        if future.result() and future.result().success:
            print('✓ Armed\n')
            return True
        if verbose:
            print('✗ Failed to arm\n')
        return False

    def arm_with_retry(self, timeout=60.0, interval=2.0):
        """Keep trying to arm until it succeeds or `timeout` seconds pass.

        Right after the MAVLink link connects, EKF/GPS/home are usually not
        ready, so the first arm attempts get rejected by prearm checks.
        Retrying waits exactly as long as needed (no magic sleep) and arms the
        moment the vehicle becomes armable."""
        start = time.time()
        attempt = 0
        while time.time() - start < timeout and rclpy.ok():
            attempt += 1
            if self.arm(verbose=(attempt == 1)):
                return True
            print(f'  ...prearm chua xong (EKF/GPS), thu lai ({int(time.time() - start)}s)')
            end = time.time() + interval
            while time.time() < end and rclpy.ok():
                rclpy.spin_once(self, timeout_sec=0.1)
        return False

    def takeoff(self, altitude):
        """Takeoff to specified altitude"""
        print(f'Taking off to {altitude}m...')

        req = CommandTOL.Request()
        req.altitude = altitude

        future = self.takeoff_client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)

        if future.result() and future.result().success:
            print(f'✓ Takeoff command sent\n')
            return True
        else:
            print('✗ Takeoff command failed\n')
            return False

    def wait_for_altitude(self, target_alt, tolerance=0.3):
        """Wait until altitude is reached"""
        print(f'Waiting to reach {target_alt}m altitude...')

        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)
            current_alt = self.current_pose.pose.position.z

            print(f'  Altitude: {current_alt:.2f}m / {target_alt}m', end='\r')

            if abs(current_alt - target_alt) < tolerance:
                print(f'\n✓ Reached altitude {current_alt:.2f}m\n')
                return True

            time.sleep(0.2)

    def move_to_position(self, x, y, z, duration=10):
        """Move to position by publishing setpoints"""
        print(f'Moving to position: ({x:.1f}, {y:.1f}, {z:.1f})m...')

        target = PoseStamped()
        target.header.frame_id = "map"
        target.pose.position.x = x
        target.pose.position.y = y
        target.pose.position.z = z

        # Publish setpoints for the specified duration
        rate = self.create_rate(20)  # 20Hz
        start_time = time.time()

        while time.time() - start_time < duration and rclpy.ok():
            target.header.stamp = self.get_clock().now().to_msg()
            self.setpoint_pub.publish(target)

            rclpy.spin_once(self, timeout_sec=0.01)

            # Show progress
            elapsed = time.time() - start_time
            curr_x = self.current_pose.pose.position.x
            curr_y = self.current_pose.pose.position.y
            print(f'  Position: ({curr_x:.2f}, {curr_y:.2f}) | Time: {elapsed:.1f}s', end='\r')

            time.sleep(0.05)

        print(f'\n✓ Position command completed\n')
        return True

    def run_mission(self):
        """Execute the mission"""
        print('='*50)
        print('MISSION: Takeoff -> Forward -> RTL')
        print('='*50)
        print()

        # Wait for connection
        print('Waiting for FCU connection...')
        while not self.current_state.connected and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)
            time.sleep(0.1)
        print('✓ Connected to FCU\n')

        # Step 1: Set GUIDED mode
        print('[Step 1] Set GUIDED mode')
        if not self.set_mode('GUIDED'):
            return
        time.sleep(1)

        # Step 2: Arm (retry: EKF/GPS need time after connect -> prearm checks)
        print('[Step 2] Arm throttle')
        if not self.arm_with_retry(timeout=60.0):
            print('✗ Khong arm duoc trong 60s (kiem tra prearm/EKF/GPS)\n')
            return
        time.sleep(2)

        # Step 3: Takeoff to 10m
        print('[Step 3] Takeoff to 10m')
        if not self.takeoff(10.0):
            return

        # Wait for takeoff to complete
        self.wait_for_altitude(10.0, tolerance=0.5)
        time.sleep(2)

        # Step 4: Move forward 10m
        print('[Step 4] Move forward 10m')
        self.move_to_position(10.0, 0.0, 10.0, duration=15)
        time.sleep(1)

        # Step 5: Wait 3 seconds
        print('[Step 5] Wait 3 seconds')
        for i in range(3, 0, -1):
            print(f'  {i}...', end='\r')
            time.sleep(1)
        print('  ✓ Wait complete\n')

        # Step 6: RTL
        print('[Step 6] Return to Launch (RTL)')
        self.set_mode('RTL')

        print('='*50)
        print('MISSION COMPLETE!')
        print('='*50)
        print('\nMonitoring RTL... (Ctrl+C to exit)')

        # Monitor while returning
        try:
            while rclpy.ok():
                rclpy.spin_once(self, timeout_sec=0.1)
                time.sleep(0.5)
        except KeyboardInterrupt:
            pass


def main():
    rclpy.init()
    node = SimpleMission()

    try:
        node.run_mission()
    except KeyboardInterrupt:
        print('\n\nMission interrupted by user')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
