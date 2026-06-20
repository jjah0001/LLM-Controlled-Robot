#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import RPi.GPIO as GPIO


class MotorDriver(Node):
    def __init__(self):
        super().__init__('motor_driver')
        self.subscription = self.create_subscription(
            Twist, '/cmd_vel', self.cmd_vel_callback, 10
        )

        # Motor A (Left):  IN1=17, IN2=22, ENA=18 (PWM)
        # Motor B (Right): IN3=23, IN4=24, ENB=19 (PWM)
        self.IN1, self.IN2, self.ENA = 17, 22, 18
        self.IN3, self.IN4, self.ENB = 23, 24, 19

        GPIO.setmode(GPIO.BCM)
        for pin in [self.IN1, self.IN2, self.IN3, self.IN4, self.ENA, self.ENB]:
            GPIO.setup(pin, GPIO.OUT)

        self.pwmA = GPIO.PWM(self.ENA, 2000)
        self.pwmB = GPIO.PWM(self.ENB, 2000)
        self.pwmA.start(0)
        self.pwmB.start(0)

        self._last_cmd_time = self.get_clock().now()
        self._timeout_timer = self.create_timer(0.1, self._check_timeout)

        self.get_logger().info('Motor driver node started, listening on /cmd_vel.')

    def cmd_vel_callback(self, msg: Twist):
        self._last_cmd_time = self.get_clock().now()

        linear = msg.linear.x
        angular = msg.angular.z

        left = max(-1.0, min(1.0, linear - angular))
        right = max(-1.0, min(1.0, linear + angular))

        self._set_motor_a(left)
        self._set_motor_b(right)

    def _set_motor_a(self, speed: float):
        duty = abs(speed) * 100.0
        if speed > 0:
            GPIO.output(self.IN1, GPIO.LOW)
            GPIO.output(self.IN2, GPIO.HIGH)
        elif speed < 0:
            GPIO.output(self.IN1, GPIO.HIGH)
            GPIO.output(self.IN2, GPIO.LOW)
        else:
            GPIO.output(self.IN1, GPIO.LOW)
            GPIO.output(self.IN2, GPIO.LOW)
        self.pwmA.ChangeDutyCycle(duty)

    def _set_motor_b(self, speed: float):
        duty = abs(speed) * 100.0
        if speed > 0:
            GPIO.output(self.IN3, GPIO.HIGH)
            GPIO.output(self.IN4, GPIO.LOW)
        elif speed < 0:
            GPIO.output(self.IN3, GPIO.LOW)
            GPIO.output(self.IN4, GPIO.HIGH)
        else:
            GPIO.output(self.IN3, GPIO.LOW)
            GPIO.output(self.IN4, GPIO.LOW)
        self.pwmB.ChangeDutyCycle(duty)

    def stop(self):
        self._set_motor_a(0.0)
        self._set_motor_b(0.0)

    def _check_timeout(self):
        elapsed = (self.get_clock().now() - self._last_cmd_time).nanoseconds / 1e9
        if elapsed > 0.5:
            self.stop()

    def destroy_node(self):
        try:
            self.stop()
            self.pwmA.stop()
            self.pwmB.stop()
            GPIO.cleanup()
        except Exception as e:
            self.get_logger().error(f'Cleanup error: {e}')
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = MotorDriver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
