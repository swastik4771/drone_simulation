#!/usr/bin/env python3

from swift_msgs.msg import SwiftMsgs
from geometry_msgs.msg import PoseArray
from controller_msg.msg import PIDTune
from error_msg.msg import Error
import rclpy
from rclpy.node import Node


class Swift_Pico(Node):
    def __init__(self):
        super().__init__('pico_controller')

        self.current_state = [0.0, 0.0, 0.0]

        # ---------------------------------------------------------------
        # YOUR TARGET IS Z=23, NOT Z=20
        # ---------------------------------------------------------------
        self.desired_state = [-7.0, 0.0, 20.0]

        self.cmd = SwiftMsgs()
        self.cmd.rc_roll     = 1500
        self.cmd.rc_pitch    = 1500
        self.cmd.rc_yaw      = 1500
        self.cmd.rc_throttle = 1500

        # ---------------------------------------------------------------
        # GAINS — tuned to stop the oscillation you're seeing
        #
        # What changed vs your previous run:
        #   Kp[2]: 12 → 5   (was building too much velocity, causing overshoot)
        #   Kd[2]: 30 → 22  (combined with better filter, stops the crossing spike)
        #   Ki[2]: 0.03→0.1 (helps hold exactly at z=23 without drift)
        #
        # Tuner slider equivalents if you want to override live:
        #   throttle: kp=167 ki=12 kd=37
        #   roll/pitch: kp=133 ki=2 kd=33
        # ---------------------------------------------------------------
        self.Kp = [4.0,  4.0,  4.0]
        self.Ki = [0.0, 0.0, 0.0]
        self.Kd = [20.0, 20.0, 15.0]

        self.error      = [0.0, 0.0, 0.0]
        self.prev_error = [0.0, 0.0, 0.0]
        self.error_sum  = [0.0, 0.0, 0.0]
        self.prev_deriv = [0.0, 0.0, 0.0]

        # 0.85 = heavy filtering — kills the crossing spike that was
        # spiking throttle to 1608 every time drone passed through target
        self.d_alpha = 0.92

        # Anti-windup: clamps accumulated integral before Ki multiplication
        self.i_limit = [15.0, 15.0, 25.0]

        # Max PID output BEFORE adding to 1500
        # Tighter = slower but no ceiling crashes
        self.out_max = [120.0, 120.0, 140.0]

        # ---------------------------------------------------------------
        # CRITICAL FIX: rc_min[2] raised from 1400 to 1460
        # At 1400 the drone freefalls when above target, builds velocity,
        # then slams down through target → causes the bounce you saw.
        # At 1460 the drone falls slowly, arrives at target gently.
        #
        # rc_max[2] lowered from 1700 to 1640
        # Limits climb speed so drone doesn't overshoot on the way up.
        # ---------------------------------------------------------------
        self.rc_min = [1400, 1400, 1460]
        self.rc_max = [1600, 1600, 1640]

        self.whycon_active = False
        self.pos_error     = Error()
        self.sample_time   = 0.033

        self.command_pub   = self.create_publisher(SwiftMsgs, '/drone_command', 10)
        self.pos_error_pub = self.create_publisher(Error, '/pos_error', 10)

        self.create_subscription(PoseArray, '/whycon/poses',  self.whycon_callback,  1)
        self.create_subscription(PIDTune,   '/throttle_pid',  self.altitude_set_pid, 1)
        self.create_subscription(PIDTune,   '/pitch_pid',     self.pitch_set_pid,    1)
        self.create_subscription(PIDTune,   '/roll_pid',      self.roll_set_pid,     1)

        self.arm()
        self.pid_timer = self.create_timer(self.sample_time, self.pid)
        self.get_logger().info('Controller started. Waiting for WhyCon...')

    def disarm(self):
        self.cmd.rc_roll     = 1000
        self.cmd.rc_yaw      = 1000
        self.cmd.rc_pitch    = 1000
        self.cmd.rc_throttle = 1000
        self.cmd.rc_aux4     = 1000
        self.command_pub.publish(self.cmd)

    def arm(self):
        self.disarm()
        self.cmd.rc_roll     = 1500
        self.cmd.rc_yaw      = 1500
        self.cmd.rc_pitch    = 1500
        self.cmd.rc_throttle = 1500
        self.cmd.rc_aux4     = 2000
        self.command_pub.publish(self.cmd)

    def whycon_callback(self, msg):
        if len(msg.poses) == 0:
            return
        self.current_state[0] = msg.poses[0].position.x
        self.current_state[1] = msg.poses[0].position.y
        self.current_state[2] = msg.poses[0].position.z

        if not self.whycon_active:
            # Seed prev_error from real position so derivative is 0 on tick 1
            # Without this: derivative = (32-0)/0.033 = 969 → instant spike
            for i in range(3):
                self.prev_error[i] = (self.current_state[i]
                                      - self.desired_state[i])
            self.whycon_active = True
            self.get_logger().info(
                f'WhyCon active. Spawn: '
                f'x={self.current_state[0]:.2f} '
                f'y={self.current_state[1]:.2f} '
                f'z={self.current_state[2]:.2f} | '
                f'Target z={self.desired_state[2]}'
            )

    def altitude_set_pid(self, alt):
        self.Kp[2] = alt.kp * 0.03
        self.Ki[2] = alt.ki * 0.008
        self.Kd[2] = alt.kd * 0.6
        self.get_logger().info(
            f'Throttle → Kp={self.Kp[2]:.2f} Ki={self.Ki[2]:.4f} Kd={self.Kd[2]:.1f}'
        )

    def pitch_set_pid(self, pitch):
        self.Kp[1] = pitch.kp * 0.03
        self.Ki[1] = pitch.ki * 0.008
        self.Kd[1] = pitch.kd * 0.6

    def roll_set_pid(self, roll):
        self.Kp[0] = roll.kp * 0.03
        self.Ki[0] = roll.ki * 0.008
        self.Kd[0] = roll.kd * 0.6

    def pid(self):
        if not self.whycon_active:
            return

        outputs = [0.0, 0.0, 0.0]

        for i in range(3):
            # Error: positive = drone below/behind target
            self.error[i] = self.current_state[i] - self.desired_state[i]

            # Integral with anti-windup
            self.error_sum[i] += self.error[i] * self.sample_time
            if self.error_sum[i] >  self.i_limit[i]:
                self.error_sum[i] =  self.i_limit[i]
            if self.error_sum[i] < -self.i_limit[i]:
                self.error_sum[i] = -self.i_limit[i]

            # Derivative with heavy low-pass filter (d_alpha=0.85)
            # This prevents the spike you saw (thr=1608) every time
            # the drone crossed through the setpoint at speed
            raw_d      = (self.error[i] - self.prev_error[i]) / self.sample_time
            filtered_d = (self.d_alpha         * self.prev_deriv[i]
                          + (1.0 - self.d_alpha) * raw_d)
            self.prev_deriv[i] = filtered_d

            outputs[i] = (self.Kp[i] * self.error[i]
                          + self.Ki[i] * self.error_sum[i]
                          + self.Kd[i] * filtered_d)

            # Clamp output swing
            if outputs[i] >  self.out_max[i]: outputs[i] =  self.out_max[i]
            if outputs[i] < -self.out_max[i]: outputs[i] = -self.out_max[i]

            self.prev_error[i] = self.error[i]

        # Map to RC
        # Throttle: positive error (drone too low) → add throttle ✓
        # Roll: IF DRONE DRIFTS AWAY FROM X TARGET → flip minus to plus
        # Pitch: IF DRONE DRIFTS AWAY FROM Y TARGET → flip plus to minus
        self.get_logger().info(
            f"err={self.error[2]:.2f}, out={outputs[2]:.2f}, throttle={1500 - outputs[2]:.2f}"
        )
        self.cmd.rc_throttle = int(1600 - outputs[2])
        self.cmd.rc_roll     = int(1500 - outputs[0])
        self.cmd.rc_pitch    = int(1500 + outputs[1])
        self.cmd.rc_yaw      = 1500

        # Hard safety clamp — this is the physical ceiling/ground guard
        self.cmd.rc_throttle = max(self.rc_min[2], min(self.rc_max[2], self.cmd.rc_throttle))
        self.cmd.rc_roll     = max(self.rc_min[0], min(self.rc_max[0], self.cmd.rc_roll))
        self.cmd.rc_pitch    = max(self.rc_min[1], min(self.rc_max[1], self.cmd.rc_pitch))

        self.command_pub.publish(self.cmd)

        self.pos_error.roll_error     = float(self.error[0])
        self.pos_error.pitch_error    = float(self.error[1])
        self.pos_error.throttle_error = float(self.error[2])
        self.pos_error.yaw_error      = 0.0
        self.pos_error_pub.publish(self.pos_error)

        self.get_logger().info(
            f'Z:{self.current_state[2]:.2f}→{self.desired_state[2]}'
            f' err={self.error[2]:+.2f}'
            f' thr={self.cmd.rc_throttle} | '
            f'X:{self.current_state[0]:.2f}→{self.desired_state[0]}'
            f' err={self.error[0]:+.2f}'
            f' roll={self.cmd.rc_roll}',
            throttle_duration_sec=0.5
        )


def main(args=None):
    rclpy.init(args=args)
    node = Swift_Pico()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down.')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()