#!/usr/bin/env python3
"""
Fixed-throttle hover test for swift_pico.
Publishes a constant throttle value and logs altitude (z) from /whycon/poses
so you can find the equilibrium (hover) throttle by observing whether the
drone climbs, sinks, or holds steady.

Usage:
    python3 hover_test.py 1550
    (pass the throttle value you want to test as a CLI arg)
"""

import sys
import time
import rclpy
from rclpy.node import Node
from swift_msgs.msg import SwiftMsgs
from geometry_msgs.msg import PoseArray


class HoverTest(Node):
    def __init__(self, throttle_val):
        super().__init__('hover_test')
        self.throttle_val = throttle_val
        self.cmd = SwiftMsgs()
        self.command_pub = self.create_publisher(SwiftMsgs, '/drone_command', 10)
        self.create_subscription(PoseArray, '/whycon/poses', self.whycon_callback, 1)

        self.start_time = time.time()
        self.z_log = []  # (t, z)

        # Arm first
        self.disarm()
        time.sleep(1.0)
        self.arm()
        self.get_logger().info(f'Testing fixed throttle = {self.throttle_val}')

        # Publish fixed throttle at 30Hz
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

    def whycon_callback(self, msg):
        if len(msg.poses) == 0:
            return
        t = time.time() - self.start_time
        z = msg.poses[0].position.z
        self.z_log.append((t, z))
        print(f"t={t:.2f}s  z={z:.3f}")

        # After 10 seconds, print a summary and exit
        if t > 10.0:
            self.summarize()
            rclpy.shutdown()

    def summarize(self):
        if len(self.z_log) < 5:
            print("Not enough data collected.")
            return
        # compare avg z in first 2s (after 1s settle) vs last 2s
        early = [z for t, z in self.z_log if 1.0 <= t <= 3.0]
        late = [z for t, z in self.z_log if 8.0 <= t <= 10.0]
        if not early or not late:
            print("Insufficient windowed data.")
            return
        avg_early = sum(early) / len(early)
        avg_late = sum(late) / len(late)
        drift = avg_late - avg_early
        drift_rate = drift / 6.0  # approx seconds between window centers

        print("\n===== SUMMARY =====")
        print(f"Throttle tested : {self.throttle_val}")
        print(f"Avg Z (1-3s)    : {avg_early:.3f}")
        print(f"Avg Z (8-10s)   : {avg_late:.3f}")
        print(f"Drift           : {drift:+.3f} m over ~6s (rate: {drift_rate:+.4f} m/s)")
        if abs(drift_rate) < 0.02:
            print("=> Looks like a good hover throttle candidate!")
        elif drift_rate > 0:
            print("=> Drone is climbing. Try a LOWER throttle value.")
        else:
            print("=> Drone is sinking. Try a HIGHER throttle value.")
        print("====================\n")


def main():
    if len(sys.argv) < 2:
        print("Usage: python3 hover_test.py <throttle_value>")
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
