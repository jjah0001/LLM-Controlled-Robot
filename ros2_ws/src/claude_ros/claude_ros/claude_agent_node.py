#!/usr/bin/env python3
import base64
import json
import os
import threading

import anthropic
import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import Twist
from rclpy.node import Node
from sensor_msgs.msg import Image, LaserScan, Range


SYSTEM_PROMPT = """You are the autonomous navigation brain of a differential-drive robot. \
You receive real-time sensor data and a camera image, then decide the robot's next motion.

Sensors available:
- Camera (RGB, forward-facing)
- Ultrasonic sensor (forward distance in metres)
- LiDAR (360° scan summarised into 6 sectors: front, front_left, left, rear, right, front_right)

Respond ONLY with a single JSON object — no markdown, no extra text:
{
  "linear_x": <float, -1.0 to 1.0>,
  "angular_z": <float, -1.0 to 1.0>,
  "reason": "<one sentence>"
}

Rules:
- Positive linear_x = forward, negative = backward.
- Positive angular_z = turn left (counter-clockwise), negative = turn right.
- If any forward sensor reads < 0.3 m, set linear_x <= 0.
- Prefer smooth, small increments. If uncertain, output all zeros (stop).
- Use the camera to identify context: obstacles, paths, open space, humans, etc.
"""


class ClaudeAgentNode(Node):
    def __init__(self):
        super().__init__('claude_agent_node')

        self.declare_parameter('model', 'claude-haiku-4-5-20251001')
        self.declare_parameter('call_rate_hz', 0.5)
        self.declare_parameter('max_linear', 0.5)
        self.declare_parameter('max_angular', 0.5)
        self.declare_parameter('image_topic', '/camera/image_raw')
        self.declare_parameter('image_width', 320)
        self.declare_parameter('image_height', 240)
        self.declare_parameter('jpeg_quality', 70)

        self.model = self.get_parameter('model').value
        call_rate = float(self.get_parameter('call_rate_hz').value)
        self.max_linear = float(self.get_parameter('max_linear').value)
        self.max_angular = float(self.get_parameter('max_angular').value)
        image_topic = self.get_parameter('image_topic').value
        self._img_w = int(self.get_parameter('image_width').value)
        self._img_h = int(self.get_parameter('image_height').value)
        self._jpeg_q = int(self.get_parameter('jpeg_quality').value)

        api_key = os.environ.get('ANTHROPIC_API_KEY', '')
        if not api_key:
            self.get_logger().error(
                'ANTHROPIC_API_KEY is not set. Export the variable and restart the node.'
            )
        self._client = anthropic.Anthropic(api_key=api_key)

        self._bridge = CvBridge()
        self._scan: LaserScan | None = None
        self._range: Range | None = None
        self._image: Image | None = None
        self._lock = threading.Lock()

        self.create_subscription(LaserScan, '/scan', self._scan_cb, 10)
        self.create_subscription(Range, 'ultrasonic/range', self._range_cb, 10)
        self.create_subscription(Image, image_topic, self._image_cb, 1)

        self._cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        self.create_timer(1.0 / call_rate, self._call_claude)

        self.get_logger().info(
            f'Claude agent ready — model={self.model}, rate={call_rate} Hz, '
            f'image_topic={image_topic}'
        )

    # ── Sensor callbacks ──────────────────────────────────────────────────────

    def _scan_cb(self, msg: LaserScan):
        with self._lock:
            self._scan = msg

    def _range_cb(self, msg: Range):
        with self._lock:
            self._range = msg

    def _image_cb(self, msg: Image):
        with self._lock:
            self._image = msg

    # ── Sensor summarisation ──────────────────────────────────────────────────

    def _summarise_scan(self, scan: LaserScan) -> dict:
        ranges = np.array(scan.ranges, dtype=np.float32)
        ranges = np.where(np.isfinite(ranges), ranges, scan.range_max)
        n = len(ranges)

        def sector_min(a, b):
            return float(ranges[int(n * a):int(n * b)].min())

        rear_idx = np.concatenate([ranges[:int(n * 0.10)], ranges[int(n * 0.90):]])
        sectors = {
            'front':       round(sector_min(0.45, 0.55), 3),
            'front_left':  round(sector_min(0.55, 0.75), 3),
            'left':        round(sector_min(0.75, 0.90), 3),
            'rear':        round(float(rear_idx.min()), 3),
            'right':       round(sector_min(0.10, 0.25), 3),
            'front_right': round(sector_min(0.25, 0.45), 3),
        }
        return sectors

    def _encode_image(self, image_msg: Image) -> str | None:
        try:
            cv_img = self._bridge.imgmsg_to_cv2(image_msg, desired_encoding='bgr8')
            cv_img = cv2.resize(cv_img, (self._img_w, self._img_h))
            ok, buf = cv2.imencode(
                '.jpg', cv_img, [cv2.IMWRITE_JPEG_QUALITY, self._jpeg_q]
            )
            if not ok:
                return None
            return base64.b64encode(buf.tobytes()).decode('utf-8')
        except Exception as e:
            self.get_logger().warn(f'Image encode failed: {e}')
            return None

    # ── Claude call ───────────────────────────────────────────────────────────

    def _call_claude(self):
        with self._lock:
            scan = self._scan
            rng = self._range
            img = self._image

        lines = []
        if rng is not None:
            lines.append(f'Ultrasonic (forward): {rng.range:.3f} m')
        if scan is not None:
            lines.append(f'LiDAR sectors (min dist m): {json.dumps(self._summarise_scan(scan))}')

        if not lines and img is None:
            self.get_logger().warn('No sensor data available yet — skipping.')
            return

        text = '\n'.join(lines) if lines else 'No distance sensor data.'
        content: list = [{'type': 'text', 'text': text}]

        if img is not None:
            b64 = self._encode_image(img)
            if b64:
                content.append({
                    'type': 'image',
                    'source': {'type': 'base64', 'media_type': 'image/jpeg', 'data': b64},
                })

        try:
            response = self._client.messages.create(
                model=self.model,
                max_tokens=256,
                system=SYSTEM_PROMPT,
                messages=[{'role': 'user', 'content': content}],
            )
            raw = response.content[0].text.strip()
            self.get_logger().debug(f'Claude: {raw}')
            self._apply_response(raw)
        except Exception as e:
            self.get_logger().error(f'Claude API error: {e}')
            self._publish_stop()

    def _apply_response(self, raw: str):
        try:
            if '```' in raw:
                raw = raw.split('```')[1].lstrip('json').strip()
            data = json.loads(raw)
            linear = float(data.get('linear_x', 0.0))
            angular = float(data.get('angular_z', 0.0))
            reason = str(data.get('reason', ''))
        except (json.JSONDecodeError, KeyError, ValueError) as e:
            self.get_logger().warn(f'Could not parse Claude response ({e}): {raw!r}')
            self._publish_stop()
            return

        linear = max(-self.max_linear, min(self.max_linear, linear))
        angular = max(-self.max_angular, min(self.max_angular, angular))

        msg = Twist()
        msg.linear.x = linear
        msg.angular.z = angular
        self._cmd_pub.publish(msg)
        self.get_logger().info(
            f'CMD linear={linear:+.2f}  angular={angular:+.2f} | {reason}'
        )

    def _publish_stop(self):
        self._cmd_pub.publish(Twist())


def main(args=None):
    rclpy.init(args=args)
    node = ClaudeAgentNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node._publish_stop()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
