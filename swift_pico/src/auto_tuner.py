#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from controller_msg.msg import PIDTune
from error_msg.msg import Error
import subprocess
import time

class UltimateAutoTuner(Node):
    def __init__(self):
        super().__init__('ultimate_auto_tuner')
        
        self.throttle_pub = self.create_publisher(PIDTune, '/throttle_pid', 10)
        self.error_sub = self.create_subscription(Error, '/pos_error', self.error_callback, 10)
        
        self.current_z_error = 0.0
        self.max_overshoot = 0.0
        
        # 💥 WIDER, COMPREHENSIVE SEARCH GRID (36 Combinations)
        # Testing safe lows, mid-ranges, and higher stabilized bounds
        self.kp_choices = [200.0, 280.0, 350.0, 420.0, 500.0, 580.0]
        self.kd_choices = [30.0, 50.0, 70.0, 90.0, 110.0, 130.0]
        
        self.kp_index = 0
        self.kd_index = 0
        self.best_score = float('inf')
        self.best_kp = 200.0
        self.best_kd = 30.0
        
        # State Machine: PRE_TAMP -> RESET_STAGE -> WAIT_FOR_PHYSICS -> FLIGHT_TEST
        self.state = "PRE_TAMP"
        self.timer_counter = 0
        
        self.control_timer = self.create_timer(0.1, self.loop_tick)

    def error_callback(self, msg: Error):
        self.current_z_error = abs(msg.throttle_error)
        if self.current_z_error > self.max_overshoot:
            self.max_overshoot = self.current_z_error

    def loop_tick(self):
        if self.kp_index >= len(self.kp_choices):
            self.get_logger().info(f"🏁 ALL 36 COMBINATIONS COMPLETED!")
            self.get_logger().info(f"🏆 ULTIMATE WINNING GAINS -> Kp: {self.best_kp}, Kd: {self.best_kd}")
            self.destroy_timer(self.control_timer)
            return

        current_kp = self.kp_choices[self.kp_index]
        current_kd = self.kd_choices[self.kd_index]

        # STEP 1: PRE-TAMP (Solves your doubt. Grounds the drone, kills momentum entirely)
        if self.state == "PRE_TAMP":
            self.timer_counter += 1
            # Force zero gains temporarily to guarantee the drone drops flat
            kill_msg = PIDTune()
            kill_msg.kp = 0
            kill_msg.ki = 0
            kill_msg.kd = 0
            self.throttle_pub.publish(kill_msg)
            
            if self.timer_counter >= 20:  # Hold down for 2.0 seconds solid
                self.state = "RESET_STAGE"
                self.timer_counter = 0

        # STEP 2: PHYSICAL RESET
        elif self.state == "RESET_STAGE":
            self.get_logger().info(f"🤖 TESTING COMBINATION [{self.kp_index*len(self.kd_choices) + self.kd_index + 1}/36]: Kp={current_kp}, Kd={current_kd}")
            
            subprocess.Popen(["ros2", "service", "call", "/reset_simulation", "std_srvs/srv/Empty", "{}"],
                             stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
            
            self.max_overshoot = 0.0
            self.timer_counter = 0
            self.state = "WAIT_FOR_PHYSICS"

        # STEP 3: INITIAL DAMPING DELAY
        elif self.state == "WAIT_FOR_PHYSICS":
            self.timer_counter += 1
            if self.timer_counter >= 25:  # Let environment register the ground coordinates cleanly
                tune_msg = PIDTune()
                tune_msg.kp = int(current_kp)
                tune_msg.ki = 0 
                tune_msg.kd = int(current_kd)
                self.throttle_pub.publish(tune_msg)
                
                self.timer_counter = 0
                self.state = "FLIGHT_TEST"

        # STEP 4: TRACK EVALUATION WINDOW
        elif self.state == "FLIGHT_TEST":
            self.timer_counter += 1
            
            if self.timer_counter % 25 == 0:
                self.get_logger().info(f"🛰️ Testing... Time: {self.timer_counter//10}s/12s | Current Error: {self.current_z_error:.2f}m")

            if self.timer_counter >= 120:  # 12 seconds evaluation is plenty to see stability
                # Penalize massive final errors (indicates crashing or flipping out)
                if self.current_z_error > 10.0:
                    score = 999.0
                else:
                    score = (self.current_z_error * 6.0) + self.max_overshoot
                
                self.get_logger().info(f"📋 Score for Kp={current_kp}, Kd={current_kd} -> {score:.2f}")
                
                if score < self.best_score:
                    self.best_score = score
                    self.best_kp = current_kp
                    self.best_kd = current_kd
                    self.get_logger().info(f"🌟 NEW LEADERBOARD TOPPER: Kp={self.best_kp}, Kd={self.best_kd}")

                # Step through the 2D Array Matrix grid coordinates
                self.kd_index += 1
                if self.kd_index >= len(self.kd_choices):
                    self.kd_index = 0
                    self.kp_index += 1
                
                self.state = "PRE_TAMP"
                self.timer_counter = 0

def main(args=None):
    rclpy.init(args=args)
    tuner = UltimateAutoTuner()
    try:
        rclpy.spin(tuner)
    except KeyboardInterrupt:
        pass
    finally:
        tuner.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
