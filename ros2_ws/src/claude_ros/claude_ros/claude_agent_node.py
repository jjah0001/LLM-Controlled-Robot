#!/usr/bin/env python3
"""Claude decision layer: camera + fused range data -> high-level motion intent.

Runs Claude in a background thread at a low rate (one decision every ~1-3 s) and
publishes each decision on /claude/intent. It never drives the motors directly;
motion_controller_node executes intents smoothly and enforces the safety limits,
so API latency or an outage can never cause jerky or unsafe motion.

Subscribes:
  /camera/image_raw     sensor_msgs/Image   (RPi Camera via camera_ros)
  /perception/summary   std_msgs/String     (fused LiDAR + ultrasonic, from motion_controller)
Publishes:
  /claude/intent        std_msgs/String     (JSON, see claude_ros/intent.py)
"""
import base64
import collections
import json
import threading
import time

import anthropic
import cv2
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import String

from claude_ros import intent as intent_mod

SYSTEM_PROMPT = """You are the navigation brain of a small differential-drive robot.
Every couple of seconds you get a fresh camera frame and a fused distance summary, and you \
choose the next motion. A local controller executes your choice smoothly (it ramps speed, \
slows near obstacles, steers away from walls and stops on its own if something gets too close), \
so you set the direction and pace, not the low-level safety.

Sensors:
- Camera: forward-facing RPi Camera, roughly 60 degrees horizontal field of view.
- LiDAR (360 degree dTOF): nearest obstacle per 45 degree sector, metres from the robot centre. \
Sectors: front, front_left, left, back_left, back, back_right, right, front_right.
- Ultrasonic: forward only, catches glass and low objects the LiDAR misses. \
"clear" = nothing in range.
- front_clearance_m / rear_clearance_m: free space between the bumper and the nearest obstacle in \
the robot's straight-line path (null = nothing seen).
- controller_flags tell you what the safety layer did, e.g. blocked_front, slowing_front, \
emergency_stop_front, steering_away_from_wall.

Your decision:
- linear: -1..1 fraction of max forward speed. angular: -1..1 fraction of max turn rate, \
positive turns left (counter-clockwise).
- duration_s: how long to hold this motion if no newer decision arrives. Your next decision \
usually arrives within {period:.0f}-{horizon:.0f} s, so pick about {horizon:.0f} s for steady \
motion, shorter for tentative moves.
- Prefer smooth, continuous motion: gentle curves (linear 0.3-0.8 with a modest angular) over \
stop-and-turn. Keep consistent with your recent decisions unless the scene changed.
- If front_clearance_m is under about 0.4 m or the front is blocked, turn toward the most open \
sector (rotate in place with linear 0) instead of pushing forward. Reverse only when boxed in.
- If the sensors or the image look unreliable, choose a slow or zero motion.

Mission: {mission}
"""


class ClaudeAgentNode(Node):
    def __init__(self):
        super().__init__('claude_agent')

        p = self.declare_parameter
        p('model', 'claude-opus-5-5')
        p('effort', 'low')
        p('max_tokens', 4096)
        p('refusal_fallback', True)
        p('decision_period_s', 2.0)
        p('request_timeout_s', 15.0)
        p('min_intent_duration_s', 1.0)
        p('max_intent_duration_s', 6.0)
        p('summary_timeout_s', 1.0)
        p('image_timeout_s', 1.0)
        p('image_topic', '/camera/image_raw')
        p('image_width', 512)
        p('image_height', 384)
        p('jpeg_quality', 75)
        p('history_length', 6)
        p('mission', 'Explore the area safely, heading for open space and avoiding obstacles.')

        g = self.get_parameter
        self.model = g('model').value
        self.effort = g('effort').value
        self.max_tokens = int(g('max_tokens').value)
        self.period = float(g('decision_period_s').value)
        self.min_dur = float(g('min_intent_duration_s').value)
        self.max_dur = float(g('max_intent_duration_s').value)
        self.summary_timeout = float(g('summary_timeout_s').value)
        self.image_timeout = float(g('image_timeout_s').value)
        self._img_size = (int(g('image_width').value), int(g('image_height').value))
        self._jpeg_q = int(g('jpeg_quality').value)
        # Server-side refusal fallback is only accepted by these model families.
        self.use_fallback = bool(g('refusal_fallback').value) and self.model.startswith(
            ('claude-opus-5', 'claude-fable-5', 'claude-sonnet-5-5'))
        self.system_prompt = SYSTEM_PROMPT.format(
            period=self.period, horizon=self.period * 2.0, mission=g('mission').value)

        # Credentials come from ANTHROPIC_API_KEY (or an `ant auth login` profile).
        self._client = anthropic.Anthropic(
            timeout=float(g('request_timeout_s').value), max_retries=1)

        self._bridge = CvBridge()
        self._lock = threading.Lock()
        self._image = None
        self._image_t = -1e9
        self._summary = None
        self._summary_t = -1e9
        self._history = collections.deque(maxlen=int(g('history_length').value))

        self.create_subscription(Image, g('image_topic').value, self._image_cb,
                                 qos_profile_sensor_data)
        self.create_subscription(String, '/perception/summary', self._summary_cb, 10)
        self._intent_pub = self.create_publisher(String, '/claude/intent', 10)

        self._stop_evt = threading.Event()
        self._thread = threading.Thread(target=self._loop, daemon=True)
        self._thread.start()
        self.get_logger().info(
            f'Claude agent ready: model={self.model} effort={self.effort} '
            f'period={self.period:.1f}s fallback={self.use_fallback}')

    # ── Inputs ────────────────────────────────────────────────────────────────

    def _image_cb(self, msg: Image):
        with self._lock:
            self._image, self._image_t = msg, time.monotonic()

    def _summary_cb(self, msg: String):
        try:
            data = json.loads(msg.data)
        except json.JSONDecodeError:
            return
        with self._lock:
            self._summary, self._summary_t = data, time.monotonic()

    # ── Decision loop (background thread) ─────────────────────────────────────

    def _loop(self):
        while not self._stop_evt.is_set() and rclpy.ok():
            t0 = time.monotonic()
            try:
                self._decide_once()
            except Exception as e:  # never let the thread die
                self.get_logger().error(f'Decision loop error: {e}')
                self._publish(intent_mod.stop_intent('agent error'))
            self._stop_evt.wait(max(0.0, self.period - (time.monotonic() - t0)))

    def _decide_once(self):
        now = time.monotonic()
        with self._lock:
            summary = self._summary if now - self._summary_t <= self.summary_timeout else None
            image = self._image if now - self._image_t <= self.image_timeout else None

        if summary is None:
            self.get_logger().warn('No fresh /perception/summary (is motion_controller running?)',
                                   throttle_duration_sec=5.0)
            return

        content = []
        b64 = self._encode_image(image) if image is not None else None
        if b64:
            content.append({'type': 'image', 'source': {
                'type': 'base64', 'media_type': 'image/jpeg', 'data': b64}})
        state = {
            'sensors': summary,
            'camera': 'attached' if b64 else 'unavailable',
            'recent_decisions_oldest_first': list(self._history),
        }
        content.append({'type': 'text', 'text': json.dumps(state)})

        t_call = time.monotonic()
        response = self._call_claude(content)
        latency = time.monotonic() - t_call
        if response is None:
            self._publish(intent_mod.stop_intent('Claude unavailable'))
            return

        if response.stop_reason == 'refusal':
            self.get_logger().warn('Claude declined this request; stopping.')
            self._publish(intent_mod.stop_intent('refusal'))
            return
        text = next((b.text for b in response.content if b.type == 'text'), None)
        try:
            decision = intent_mod.normalise(json.loads(text), self.min_dur, self.max_dur)
        except (TypeError, ValueError) as e:
            self.get_logger().warn(f'Unparseable decision ({e}), stop_reason='
                                   f'{response.stop_reason}: {text!r}')
            self._publish(intent_mod.stop_intent('unparseable decision'))
            return

        self._publish(decision)
        self._history.append({k: decision[k] for k in ('linear', 'angular', 'reason')})
        self.get_logger().info(
            f'[{latency:.1f}s] lin={decision["linear"]:+.2f} ang={decision["angular"]:+.2f} '
            f'for {decision["duration_s"]:.1f}s | {decision["reason"]}')

    def _call_claude(self, content):
        kwargs = dict(
            model=self.model,
            max_tokens=self.max_tokens,
            system=self.system_prompt,
            messages=[{'role': 'user', 'content': content}],
            output_config={
                'effort': self.effort,
                'format': {'type': 'json_schema', 'schema': intent_mod.DECISION_SCHEMA},
            },
        )
        try:
            if self.use_fallback:
                return self._client.beta.messages.create(
                    betas=['server-side-fallback-2026-07-01'], fallbacks='default', **kwargs)
            return self._client.messages.create(**kwargs)
        except anthropic.AuthenticationError:
            self.get_logger().error('Anthropic authentication failed: set ANTHROPIC_API_KEY.',
                                    throttle_duration_sec=30.0)
        except anthropic.RateLimitError:
            self.get_logger().warn('Rate limited by the Claude API; consider a longer '
                                   'decision_period_s.', throttle_duration_sec=10.0)
        except anthropic.BadRequestError as e:
            self.get_logger().error(f'Claude API rejected the request: {e}',
                                    throttle_duration_sec=30.0)
        except anthropic.APIStatusError as e:
            self.get_logger().warn(f'Claude API error {e.status_code}: {e}')
        except anthropic.APIConnectionError as e:
            self.get_logger().warn(f'Cannot reach the Claude API: {e}', throttle_duration_sec=10.0)
        return None

    def _encode_image(self, msg: Image):
        try:
            img = self._bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            img = cv2.resize(img, self._img_size, interpolation=cv2.INTER_AREA)
            ok, buf = cv2.imencode('.jpg', img, [cv2.IMWRITE_JPEG_QUALITY, self._jpeg_q])
            return base64.b64encode(buf.tobytes()).decode('ascii') if ok else None
        except Exception as e:
            self.get_logger().warn(f'Image encode failed ({msg.encoding}): {e}',
                                   throttle_duration_sec=10.0)
            return None

    def _publish(self, decision: dict):
        self._intent_pub.publish(String(data=intent_mod.encode(decision)))

    def shutdown(self):
        self._stop_evt.set()
        self._thread.join(timeout=1.0)


def main(args=None):
    rclpy.init(args=args)
    node = ClaudeAgentNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
