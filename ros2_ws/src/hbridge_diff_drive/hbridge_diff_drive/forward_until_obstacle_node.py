#!/usr/bin/env python3
import math
from typing import Optional
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from sensor_msgs.msg import Range
from geometry_msgs.msg import Twist

class ForwardUntilObstacle(Node):
    def __init__(self):
        super().__init__('forward_until_obstacle')
        # Parameters
        self.declare_parameter('range_topic', '/ultrasonic/front')
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('stop_distance_m', 0.25)
        self.declare_parameter('slowdown_distance_m', 0.50)
        self.declare_parameter('forward_speed_mps', 0.15)
        self.declare_parameter('control_hz', 20)
        self.declare_parameter('no_data_behavior', 'stop')    # 'stop' or 'go'
        self.declare_parameter('stale_timeout_s', 0.5)

        self.range_topic = self.get_parameter('range_topic').value
        self.cmd_vel_topic = self.get_parameter('cmd_vel_topic').value
        self.stop_distance = float(self.get_parameter('stop_distance_m').value)
        self.slowdown_distance = float(self.get_parameter('slowdown_distance_m').value)
        self.forward_speed = float(self.get_parameter('forward_speed_mps').value)
        self.control_hz = int(self.get_parameter('control_hz').value)
        self.no_data_behavior = str(self.get_parameter('no_data_behavior').value).lower()
        self.stale_timeout = float(self.get_parameter('stale_timeout_s').value)

        if self.slowdown_distance < self.stop_distance:
            self.get_logger().warn("slowdown_distance_m < stop_distance_m; clamping.")
            self.slowdown_distance = self.stop_distance

        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST, depth=5
        )
        self.sub = self.create_subscription(Range, self.range_topic, self.range_cb, qos)
        self.pub = self.create_publisher(Twist, self.cmd_vel_topic, 10)

        self.last_range: Optional[float] = None
        self.last_msg_time = None
        self.warned_stale = False

        self.timer = self.create_timer(max(1.0/self.control_hz, 0.01), self.control_cb)
        self.get_logger().info(f"ForwardUntilObstacle → {self.range_topic} → {self.cmd_vel_topic}")

    def range_cb(self, msg: Range):
        r = msg.range
        if math.isnan(r) or r <= 0:
            return
        r = max(min(r, msg.max_range if msg.max_range > 0 else r),
                msg.min_range if msg.min_range > 0 else r)
        self.last_range = r
        self.last_msg_time = self.get_clock().now()
        self.warned_stale = False

    def control_cb(self):
        now = self.get_clock().now()
        fresh = False
        if self.last_msg_time:
            age = (now - self.last_msg_time).nanoseconds * 1e-9
            fresh = age <= self.stale_timeout
            if not fresh and not self.warned_stale:
                self.get_logger().warn(f"No fresh range data for {age:.2f}s.")
                self.warned_stale = True

        speed = 0.0
        if fresh and self.last_range is not None:
            r = self.last_range
            if r <= self.stop_distance:
                speed = 0.0
            elif r <= self.slowdown_distance:
                alpha = (r - self.stop_distance) / (self.slowdown_distance - self.stop_distance or 1.0)
                speed = max(0.0, min(1.0, alpha)) * self.forward_speed
            else:
                speed = self.forward_speed
        else:
            speed = 0.0 if self.no_data_behavior == 'stop' else self.forward_speed

        t = Twist()
        t.linear.x = float(speed)
        t.angular.z = 0.0
        self.pub.publish(t)

def main(args=None):
    rclpy.init(args=args)
    n = ForwardUntilObstacle()
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass
    finally:
        n.pub.publish(Twist())  # stop on shutdown
        n.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
