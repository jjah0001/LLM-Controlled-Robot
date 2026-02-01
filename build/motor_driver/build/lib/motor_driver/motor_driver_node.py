#!/usr/bin/env python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import RPi.GPIO as GPIO
import time

class MotorDriver(Node):
    def __init__(self):
        super().__init__('motor_driver')
        self.subscription = self.create_subscription(
            String, 'motor_command', self.listener_callback, 10
        )


        # Pin (BCM)
        self.dir_pins = [17, 22, 23, 24]   # IN1..IN4
        self.en_pins  = [18, 19]           # ENA, ENB (PWM)
        self.motor_pins = self.dir_pins + self.en_pins

        GPIO.setmode(GPIO.BCM)
        for pin in self.motor_pins:
            GPIO.setup(pin, GPIO.OUT)

        self.f_pwm = 2000          # MULAI di 2 kHz dulu (RPi.GPIO di 20 kHz sering jitter)
        self.default_duty = 60.0   # %
        self.pwmA = GPIO.PWM(18, self.f_pwm)
        self.pwmB = GPIO.PWM(19, self.f_pwm)
        self.pwmA.start(0)
        self.pwmB.start(0)

        self.get_logger().info('Motor driver node started.')

    def listener_callback(self, msg):
        command = msg.data.strip().lower()
        self.get_logger().info(f'Received command: {command}')

        # HANYA cocokkan string persis 'forward', 'backward', dll.
        # Kalau kamu publish 'forward 0.5' ini TIDAK akan cocok.
        if command == 'forward':
            self.forward(0.5)
        elif command == 'backward':
            self.backward(0.5)
        elif command == 'left':
            self.left(0.5)
        elif command == 'right':
            self.right(0.5)
        elif command == 'increase':
            self.increase_speed()
        elif command == 'decrease':
            self.decrease_speed()
        elif command == 'stop':
            self.stop()
        else:
            self.get_logger().warn('Unknown command! Use: forward/backward/left/right/stop')

    def _set_speed(self, duty_percent):
        duty = max(0.0, min(100.0, float(duty_percent)))
        self.pwmA.ChangeDutyCycle(duty)
        self.pwmB.ChangeDutyCycle(duty)

    def forward(self, sec):
        # Arah: sesuaikan dengan wiring kamu
        GPIO.output(17, GPIO.LOW)
        GPIO.output(22, GPIO.HIGH)
        GPIO.output(23, GPIO.HIGH)
        GPIO.output(24, GPIO.LOW)
        self._set_speed(self.default_duty)   # <<< penting!
        time.sleep(sec)
        self.stop()

    def backward(self, sec):
        GPIO.output(17, GPIO.HIGH)
        GPIO.output(22, GPIO.LOW)
        GPIO.output(23, GPIO.LOW)
        GPIO.output(24, GPIO.HIGH)
        self._set_speed(self.default_duty)
        time.sleep(sec)
        self.stop()

    def left(self, sec):
        GPIO.output(17, GPIO.HIGH)
        GPIO.output(22, GPIO.LOW)
        GPIO.output(23, GPIO.HIGH)
        GPIO.output(24, GPIO.LOW)
        self._set_speed(self.default_duty)
        time.sleep(sec)
        self.stop()

    def right(self, sec):
        GPIO.output(17, GPIO.LOW)
        GPIO.output(22, GPIO.HIGH)
        GPIO.output(23, GPIO.LOW)
        GPIO.output(24, GPIO.HIGH)
        self._set_speed(self.default_duty)
        time.sleep(sec)
        self.stop()

    def increase_speed(self):
        self.default_duty += 5.0
        self.get_logger().info(f'Speed is now: {self.default_duty}%')

    def decrease_speed(self):
        self.default_duty -= 5.0
        self.get_logger().info(f'Speed is now: {self.default_duty}%')

    def stop(self):
        # Matikan daya motor dengan duty 0 dan lepas arah
        self._set_speed(0.0)
        for pin in self.dir_pins:
            GPIO.output(pin, GPIO.LOW)
        # (Pilihan) kalau mau benar-benar disable, juga LOW-kan ENA/ENB:
        # for pin in self.en_pins:
        #     GPIO.output(pin, GPIO.LOW)

    def destroy_node(self):
        try:
            self._set_speed(0.0)
            for pin in self.motor_pins:
                GPIO.output(pin, GPIO.LOW)
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
