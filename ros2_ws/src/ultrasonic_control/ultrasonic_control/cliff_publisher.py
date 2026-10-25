# ultrasonic_control/cliff_publisher.py
"""Downward-facing IR edge ("cliff") sensors -> /cliff (std_msgs/String, JSON).

Each sensor is a digital IR module (e.g. TCRT5000 / FC-51) pointing at the floor a few
centimetres ahead of the wheels. Over the floor it sees its own reflection; past an edge
the reflection disappears. Published every tick:

    {"front": true, "rear": false, "sensors": {"front_left": true, "front_right": false}}

The motion controller stops forward (or backward) travel as soon as "front" (or "rear")
is true, and stops all travel if this topic goes silent while cliff sensors are enabled.
"""
import json

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import String
import RPi.GPIO as GPIO

from ultrasonic_control.cliff import CliffDebounce, summarise


class CliffPublisher(Node):
    def __init__(self):
        super().__init__('cliff_publisher')
        self.declare_parameter('sensor_names', ['front_left', 'front_right'])
        self.declare_parameter('sensor_pins', [5, 6])     # BCM
        self.declare_parameter('drop_level', 1)           # GPIO level that means "no floor"
        self.declare_parameter('rate_hz', 50.0)
        self.declare_parameter('clear_count', 3)
        names = list(self.get_parameter('sensor_names').value)
        pins = [int(p) for p in self.get_parameter('sensor_pins').value]
        if len(names) != len(pins) or not names:
            raise ValueError('sensor_names and sensor_pins must be non-empty and the same length')
        bad = [n for n in names if not n.startswith(('front', 'rear'))]
        if bad:
            raise ValueError(f'cliff sensor names must start with "front" or "rear": {bad}')
        self.drop_level = GPIO.HIGH if int(self.get_parameter('drop_level').value) else GPIO.LOW
        clear_count = int(self.get_parameter('clear_count').value)

        GPIO.setwarnings(False)
        GPIO.setmode(GPIO.BCM)
        # Pull towards the "drop" level, so a loose wire reads as an edge and stops the robot.
        pull = GPIO.PUD_UP if self.drop_level == GPIO.HIGH else GPIO.PUD_DOWN
        for pin in pins:
            GPIO.setup(pin, GPIO.IN, pull_up_down=pull)
        self.sensors = {n: (pin, CliffDebounce(clear_count)) for n, pin in zip(names, pins)}
        self._last = None

        self.pub = self.create_publisher(String, '/cliff', 10)
        self.create_timer(1.0 / float(self.get_parameter('rate_hz').value), self._tick)
        self.get_logger().info('Cliff sensors started: ' + ', '.join(
            f'{n}=GPIO{p}' for n, p in zip(names, pins)))

    def _tick(self):
        states = {n: deb.update(GPIO.input(pin) == self.drop_level)
                  for n, (pin, deb) in self.sensors.items()}
        if states != self._last:
            edges = [n for n, d in states.items() if d]
            self.get_logger().info('Edge seen by: ' + ', '.join(edges) if edges else
                                   'All cliff sensors see the floor.')
            self._last = states
        self.pub.publish(String(data=json.dumps(summarise(states))))

    def destroy_node(self):
        try:
            GPIO.cleanup([pin for pin, _ in self.sensors.values()])
        except Exception:
            pass
        super().destroy_node()


def main():
    rclpy.init()
    node = CliffPublisher()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
