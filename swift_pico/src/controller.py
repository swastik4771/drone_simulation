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
        self.desired_state = [-7.0, 0.0, 23.0]

        self.cmd = SwiftMsgs()
        self.cmd.rc_roll     = 1500
        self.cmd.rc_pitch    = 1500
        self.cmd.rc_yaw      = 1500
        self.cmd.rc_throttle = 1500

        self.Kp = [4.0, 4.0, 5.0]
        self.Ki = [0.0, 0.0, 0.0]
        self.Kd = [20.0, 20.0, 15.0]

        self.error      = [0.0, 0.0, 0.0]
        self.prev_error = [0.0, 0.0, 0.0]
        self.error_sum  = [0.0, 0.0, 0.0]
        self.prev_deriv = [0.0, 0.0, 0.0]

        self.d_alpha = 0.50
        self.i_limit = [15.0, 15.0, 25.0]
        self.out_max = [120.0, 120.0, 140.0]

        self.rc_min = [1000, 1000, 1000]
        self.rc_max = [2000, 2000, 2000]

        self.whycon_active = False
        self.pos_error     = Error()

        self.sample_time   = 0.033 
        self.last_time = None

        self.command_pub   = self.create_publisher(SwiftMsgs, '/drone_command', 10)
        self.pos_error_pub = self.create_publisher(Error, '/pos_error', 10)

        self.create_subscription(PoseArray, '/whycon/poses',  self.whycon_callback,  1)
        self.create_subscription(PIDTune,   '/throttle_pid',  self.altitude_set_pid, 1)
        self.create_subscription(PIDTune,   '/pitch_pid',     self.pitch_set_pid,    1)
        self.create_subscription(PIDTune,   '/roll_pid',      self.roll_set_pid,     1)

        self.arm()
        self.get_logger().info('Instrumentation Build Live. Ready for diagnostic flight...')

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
            self.last_time = self.get_clock().now()
            for i in range(3):
                self.prev_error[i] = (self.current_state[i] - self.desired_state[i])
            self.whycon_active = True
            self.get_logger().info('Time baseline locked. Tracking initialized.')
            return 

        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9
        self.last_time = current_time

        # Safeguard burst frequencies
        if dt <= 0.01:
            return

        self.pid(dt)

    def altitude_set_pid(self, alt):
        self.Kp[2] = alt.kp * 0.03
        self.Ki[2] = alt.ki * 0.008
        self.Kd[2] = alt.kd * 0.6

    def pitch_set_pid(self, pitch):
        self.Kp[1] = pitch.kp * 0.03
        self.Ki[1] = pitch.ki * 0.008
        self.Kd[1] = pitch.kd * 0.6

    def roll_set_pid(self, roll):
        self.Kp[0] = roll.kp * 0.03
        self.Ki[0] = roll.ki * 0.008
        self.Kd[0] = roll.kd * 0.6

    def pid(self, dt):
        outputs = [0.0, 0.0, 0.0]
        z_raw_d = 0.0
        z_filt_d = 0.0
        z_output = 0.0
        
        for i in range(3):
            self.error[i] = self.current_state[i] - self.desired_state[i]

            self.error_sum[i] += self.error[i] * dt
            self.error_sum[i] = max(min(self.error_sum[i], self.i_limit[i]), -self.i_limit[i])

            raw_d = (self.error[i] - self.prev_error[i]) / dt
            filtered_d = (self.d_alpha * self.prev_deriv[i]) + ((1.0 - self.d_alpha) * raw_d)
            self.prev_deriv[i] = filtered_d
            
            if i == 2:
                z_raw_d = raw_d
                z_filt_d = filtered_d

            outputs[i] = (self.Kp[i] * self.error[i]) + (self.Ki[i] * self.error_sum[i]) + (self.Kd[i] * filtered_d)

            if outputs[i] >  self.out_max[i]: outputs[i] =  self.out_max[i]
            if outputs[i] < -self.out_max[i]: outputs[i] = -self.out_max[i]

            if i == 2:
                z_output = outputs[i]

            self.prev_error[i] = self.error[i]

        self.cmd.rc_throttle = int(1535 + outputs[2])
        self.cmd.rc_roll     = int(1500 - outputs[0])
        self.cmd.rc_pitch    = int(1500 + outputs[1])
        self.cmd.rc_yaw      = 1500

        self.cmd.rc_throttle = max(self.rc_min[2], min(self.rc_max[2], self.cmd.rc_throttle))
        self.cmd.rc_roll     = max(self.rc_min[0], min(self.rc_max[0], self.cmd.rc_roll))
        self.cmd.rc_pitch    = max(self.rc_min[1], min(self.rc_max[1], self.cmd.rc_pitch))

        self.command_pub.publish(self.cmd)

        self.pos_error.roll_error     = float(self.error[0])
        self.pos_error.pitch_error    = float(self.error[1])
        self.pos_error.throttle_error = float(self.error[2])
        self.pos_error.yaw_error      = 0.0
        self.pos_error_pub.publish(self.pos_error)

        # 🔒 Safe division fallback conversion
        calculated_fps = 1.0 / dt if dt > 0.0 else 0.0

        print(
            f"dt={dt:.3f}, "
            f"fps={calculated_fps:.2f}, "
            f"Z={self.current_state[2]:.2f}, "
            f"err={self.error[2]:.2f}, "
            f"rawD={z_raw_d:.2f}, "
            f"filtD={z_filt_d:.2f}, "
            f"out={z_output:.2f}, "
            f"thr={self.cmd.rc_throttle}"
        )

def main(args=None):
    rclpy.init(args=args)
    node = Swift_Pico()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()