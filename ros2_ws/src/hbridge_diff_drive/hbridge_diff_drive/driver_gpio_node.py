#!/usr/bin/env python3
import time
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from geometry_msgs.msg import Twist
from std_msgs.msg import String
import RPi.GPIO as GPIO

def clamp(x, lo, hi):
    return max(lo, min(hi, x))

class HBridgeGPIODriver(Node):
    """
    Differential-drive H-bridge driver using RPi.GPIO with fixed IN pins:
    Left:  IN1=17, IN2=22   (forward = IN1 LOW,  IN2 HIGH)   <-- matches your code
    Right: IN1=23, IN2=24   (forward = IN1 HIGH, IN2 LOW)    <-- matches your code
    Subscribes to /cmd_vel (Twist) and motor_command (String).
    """
    def __init__(self):
        super().__init__('hbridge_gpio_driver')

        # -------- Parameters --------
        self.declare_parameter('left_in1_pin', 17)
        self.declare_parameter('left_in2_pin', 22)
        self.declare_parameter('right_in1_pin', 23)
        self.declare_parameter('right_in2_pin', 24)
        self.declare_parameter('wheel_base_m', 0.16)          # distance between wheels
        self.declare_parameter('deadband', 0.05)               # velocity threshold to move
        self.declare_parameter('watchdog_timeout_s', 0.5)      # stop if /cmd_vel stale
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('motor_command_topic', 'motor_command')

        # Fetch
        self.L_IN1 = int(self.get_parameter('left_in1_pin').value)
        self.L_IN2 = int(self.get_parameter('left_in2_pin').value)
        self.R_IN1 = int(self.get_parameter('right_in1_pin').value)
        self.R_IN2 = int(self.get_parameter('right_in2_pin').value)
        self.wheel_base = float(self.get_parameter('wheel_base_m').value)
        self.deadband = float(self.get_parameter('deadband').value)
        self.watchdog = float(self.get_parameter('watchdog_timeout_s').value)
        self.cmd_topic = self.get_parameter('cmd_vel_topic').get_parameter_value().string_value
        self.cmd_str_topic = self.get_parameter('motor_command_topic').get_parameter_value().string_value

        # -------- GPIO setup --------
        GPIO.setwarnings(False)
        GPIO.setmode(GPIO.BCM)
        for p in (self.L_IN1, self.L_IN2, self.R_IN1, self.R_IN2):
            GPIO.setup(p, GPIO.OUT, initial=GPIO.LOW)

        # -------- Subscriptions --------
        qos = QoSProfile(reliability=QoSReliabilityPolicy.RELIABLE,
                         history=QoSHistoryPolicy.KEEP_LAST, depth=5)
        self.sub_twist = self.create_subscription(Twist, self.cmd_topic, self.cmd_cb, qos)
        self.sub_string = self.create_subscription(String, self.cmd_str_topic, self.string_cb, 10)

        # -------- Timer (watchdog) --------
        self._last_cmd_time = None
        self.timer = self.create_timer(0.05, self._watchdog_cb)  # 20 Hz

        self.get_logger().info(
            f"H-bridge (GPIO) ready on pins L(17,22) R(23,24). "
            f"Listening to '{self.cmd_topic}' (Twist) and '{self.cmd_str_topic}' (String)."
        )

    # === Low-level output helpers ===
    def _left(self, mode: int):
        """mode: +1 fwd, -1 rev, 0 stop (matches your truth table)."""
        if mode > 0:      # forward -> 17 LOW, 22 HIGH
            GPIO.output(self.L_IN1, GPIO.LOW)
            GPIO.output(self.L_IN2, GPIO.HIGH)
        elif mode < 0:    # reverse -> 17 HIGH, 22 LOW
            GPIO.output(self.L_IN1, GPIO.HIGH)
            GPIO.output(self.L_IN2, GPIO.LOW)
        else:             # stop (coast)
            GPIO.output(self.L_IN1, GPIO.LOW)
            GPIO.output(self.L_IN2, GPIO.LOW)

    def _right(self, mode: int):
        """mode: +1 fwd, -1 rev, 0 stop (matches your truth table)."""
        if mode > 0:      # forward -> 23 HIGH, 24 LOW
            GPIO.output(self.R_IN1, GPIO.HIGH)
            GPIO.output(self.R_IN2, GPIO.LOW)
        elif mode < 0:    # reverse -> 23 LOW, 24 HIGH
            GPIO.output(self.R_IN1, GPIO.LOW)
            GPIO.output(self.R_IN2, GPIO.HIGH)
        else:             # stop (coast)
            GPIO.output(self.R_IN1, GPIO.LOW)
            GPIO.output(self.R_IN2, GPIO.LOW)

    def stop(self):
        self._left(0)
        self._right(0)

    # === ROS callbacks ===
    def cmd_cb(self, msg: Twist):
        v = msg.linear.x
        w = msg.angular.z
        # Simple diff-drive split (no speed control; just direction/coast)
        vl = v - 0.5 * w * self.wheel_base
        vr = v + 0.5 * w * self.wheel_base

        # Convert to discrete motor states based on deadband
        ml = 1 if vl >  self.deadband else (-1 if vl < -self.deadband else 0)
        mr = 1 if vr >  self.deadband else (-1 if vr < -self.deadband else 0)

        self._left(ml)
        self._right(mr)
        self._last_cmd_time = self.get_clock().now()

    def string_cb(self, msg: String):
        cmd = msg.data.strip().lower()
        if   cmd == 'forward':
            self._left(+1);  self._right(+1)
        elif cmd == 'backward':
            self._left(-1);  self._right(-1)
        elif cmd == 'left':
            self._left(-1);  self._right(+1)
        elif cmd == 'right':
            self._left(+1);  self._right(-1)
        elif cmd == 'stop':
            self.stop()
        else:
            self.get_logger().warn(f"Unknown motor_command: '{cmd}'")
            return
        self._last_cmd_time = self.get_clock().now()

    def _watchdog_cb(self):
        # If no recent commands (from either topic), stop the robot
        if self._last_cmd_time is None:
            return
        age = (self.get_clock().now() - self._last_cmd_time).nanoseconds * 1e-9
        if age > self.watchdog:
            self.stop()

    # === Cleanup ===
    def destroy_node(self):
        try:
            self.stop()
            GPIO.cleanup([self.L_IN1, self.L_IN2, self.R_IN1, self.R_IN2])
        finally:
            super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = HBridgeGPIODriver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
