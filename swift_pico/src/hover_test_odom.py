#!/usr/bin/env python3
"""
Fixed-throttle hover test using /rotors/odometry (real Gazebo physics, real meters)
instead of /whycon/poses (which uses a camera-relative distance, not real altitude).

Usage:
    python3 hover_test_odom.py 1550
"""

import sys
import time
import rclpy
from rclpy.node import Node
from swift_msgs.msg import SwiftMsgs
from nav_msgs.msg import Odometry


class HoverTest(Node):
    def __init__(self, throttle_val):
        super().__init__('hover_test_odom')
        self.throttle_val = throttle_val
        self.cmd = SwiftMsgs()
        self.command_pub = self.create_publisher(SwiftMsgs, '/drone_command', 10)
        self.create_subscription(Odometry, '/rotors/odometry', self.odom_callback, 1)

        self.start_time = time.time()
        self.z_log = []

        self.disarm()
        time.sleep(1.0)
        self.arm()
        self.get_logger().info(f'Testing fixed throttle = {self.throttle_val} (using real odometry)')

        self.timer = self.create_timer(0.033, self.publish_throttle)

    def disarm(self):
        self.cmd.rc_roll = 1500
        self.cmd.rc_pitch = 1500
        self.cmd.rc_yaw = 1500
        self.cmd.rc_throttle = 1000
        self.cmd.rc_aux4 = 1000
        self.command_pub.publish(self.cmd)

    def arm(self):
        self.cmd.rc_roll = 1500
        self.cmd.rc_pitch = 1500
        self.cmd.rc_yaw = 1500
        self.cmd.rc_throttle = 1500
        self.cmd.rc_aux4 = 2000
        self.command_pub.publish(self.cmd)

    def publish_throttle(self):
        self.cmd.rc_roll = 1500
        self.cmd.rc_pitch = 1500
        self.cmd.rc_yaw = 1500
        self.cmd.rc_throttle = self.throttle_val
        self.cmd.rc_aux4 = 2000
        self.command_pub.publish(self.cmd)

    def odom_callback(self, msg):
        t = time.time() - self.start_time
        z = msg.pose.pose.position.z
        self.z_log.append((t, z))
        print(f"t={t:.2f}s  z={z:.4f}")

        if t > 10.0:
            self.summarize()
            rclpy.shutdown()

    def summarize(self):
        if len(self.z_log) < 5:
            print("Not enough data collected.")
            return
        early = [z for t, z in self.z_log if 1.0 <= t <= 3.0]
        late = [z for t, z in self.z_log if 8.0 <= t <= 10.0]
        if not early or not late:
            print("Insufficient windowed data.")
            return
        avg_early = sum(early) / len(early)
        avg_late = sum(late) / len(late)
        drift = avg_late - avg_early
        drift_rate = drift / 6.0

        print("\n===== SUMMARY (real Gazebo altitude, meters) =====")
        print(f"Throttle tested : {self.throttle_val}")
        print(f"Avg Z (1-3s)    : {avg_early:.4f} m")
        print(f"Avg Z (8-10s)   : {avg_late:.4f} m")
        print(f"Drift           : {drift:+.4f} m over ~6s (rate: {drift_rate:+.5f} m/s)")
        if abs(drift_rate) < 0.01:
            print("=> Good hover throttle candidate!")
        elif drift_rate > 0:
            print("=> Drone is climbing. Try a LOWER throttle value.")
        else:
            print("=> Drone is sinking/not lifting off. Try a HIGHER throttle value.")
        print("====================\n")


def main():
    if len(sys.argv) < 2:
        print("Usage: python3 hover_test_odom.py <throttle_value>")
        sys.exit(1)

    throttle_val = int(sys.argv[1])
    rclpy.init()
    node = HoverTest(throttle_val)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.destroy_node()
        except Exception:
            pass


if __name__ == '__main__':
    main()
