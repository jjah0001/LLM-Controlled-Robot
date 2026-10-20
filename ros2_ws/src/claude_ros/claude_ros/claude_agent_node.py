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
  /claude/say           std_msgs/String     (words for people nearby; feed a TTS node)
  /claude/note          std_msgs/String     (each new memory note)

The robot's identity comes from the repository's CLAUDE.md; its memory notes are
kept in a JSON Lines file (memory_file) and replayed into every request.
"""
import base64
import collections
import json
import os
import threading
import time

import anthropic
import cv2
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import String

from claude_ros import intent as intent_mod
from claude_ros.memory import NoteStore

# The robot's identity (who it is, goals, hardware, senses, actions, personality,
# communication, memory) lives in the repository's CLAUDE.md and is loaded at start-up.
# These rules are the technical contract with the motion controller and stay in code.
OPERATING_RULES = """# Operating rules (from the control software)

Every {period:.0f} s or so you receive the latest camera frame (if the camera works) and a JSON \
message with:
- sensors: the fused view from the motion controller. lidar_sectors_m = nearest obstacle per \
45 degree sector (front, front_left, left, back_left, back, back_right, right, front_right), \
metres from the robot centre. front_clearance_m / rear_clearance_m = free space between the \
bumper and the nearest obstacle in the straight-line path (null = nothing seen). ultrasonic_m \
("clear" = nothing in range). controller_flags = what the safety layer is doing \
(blocked_front, slowing_front, emergency_stop_front, cliff_front, steering_away_from_wall, \
...). cliff = the downward edge sensors (edge_ahead / edge_behind true = the floor ends \
there), or a note that none are installed (then only your camera can spot drops). \
active_intent = the motion you chose last time and how long it has left.
- motor_power: the motors' current floor mode and PWM duty range.
- recent_decisions_oldest_first: your last few decisions, each with the front clearance at the \
time. If you kept asking to move but the distances stayed the same, you are probably stalled.
- memory_notes: notes you saved earlier, oldest first.

Answer with one decision:
- linear: -1..1 fraction of max forward speed. angular: -1..1 fraction of max turn rate, \
positive turns left (counter-clockwise).
- duration_s: how long to hold this motion if no newer decision arrives. The next decision \
usually arrives within {period:.0f}-{horizon:.0f} s, so about {horizon:.0f} s keeps motion \
continuous; use less for tentative moves.
- reason: one short sentence, for the logs.
- say: words to speak to people nearby, or "" (most of the time).
- note: a new fact worth remembering for later, or "". Don't repeat notes you already have.
- floor: "carpet" or "hard_floor" to change the motor power mode, or "" to keep it. \
motor_power shows the current mode.

The motion controller ramps speed, slows near obstacles and stops on its own, so you choose \
direction and pace, not low-level safety. If front_clearance_m is under about 0.4 m or the \
front is blocked, rotate in place (linear 0) toward the most open sector rather than pushing \
forward. Reverse only when boxed in. If the sensors or image look unreliable, go slowly or stop.

Edges: falling is the worst thing that can happen to you, and the LiDAR and front ultrasonic \
look straight ahead, so they cannot see a drop. If cliff.edge_ahead is true or the flags show \
cliff_front, back away slowly (linear about -0.3 for about 1 s), then turn away from the edge; \
never try forward again there. If you see stairs, a step down, a ledge or the \
edge of a table or platform in the image, stay well away (at least half a metre) and note it. \
If no cliff sensors are installed and there is no camera image, move only slowly (linear 0.3 or \
less) and only over floor you have already crossed.

Current mission: {mission}
"""

FALLBACK_IDENTITY = """You are a small, careful, curious differential-drive robot exploring \
indoors. Move smoothly, avoid obstacles, and remember what you see."""


def find_persona(explicit_path: str = '') -> str | None:
    """Locate the robot's CLAUDE.md.

    Order: the persona_file parameter, $ROBOT_PERSONA_FILE, then CLAUDE.md in a parent
    directory of this source file (works with colcon --symlink-install) or of the cwd.
    """
    candidates = [explicit_path, os.environ.get('ROBOT_PERSONA_FILE', '')]
    for start in (os.path.dirname(os.path.realpath(__file__)), os.getcwd()):
        d = start
        while True:
            candidates.append(os.path.join(d, 'CLAUDE.md'))
            parent = os.path.dirname(d)
            if parent == d:
                break
            d = parent
    for c in candidates:
        if c and os.path.isfile(os.path.expanduser(c)):
            return os.path.expanduser(c)
    return None


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
        p('persona_file', '')
        p('memory_file', '~/.ros/robot_memory/notes.jsonl')
        p('memory_recall', 30)
        p('memory_max_notes', 500)

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
        persona_path = find_persona(g('persona_file').value)
        if persona_path:
            with open(persona_path, encoding='utf-8') as f:
                identity = f.read()
            self.get_logger().info(f'Loaded robot identity from {persona_path}')
        else:
            identity = FALLBACK_IDENTITY
            self.get_logger().warn('CLAUDE.md not found: using a minimal built-in identity. '
                                   'Set persona_file or build with --symlink-install.')
        self.system_prompt = identity.strip() + '\n\n' + OPERATING_RULES.format(
            period=self.period, horizon=self.period * 2.0, mission=g('mission').value)

        self.memory = NoteStore(g('memory_file').value, int(g('memory_max_notes').value))
        self.memory_recall = int(g('memory_recall').value)

        # Credentials come from ANTHROPIC_API_KEY (or an `ant auth login` profile).
        self._client = anthropic.Anthropic(
            timeout=float(g('request_timeout_s').value), max_retries=1)

        self._bridge = CvBridge()
        self._lock = threading.Lock()
        self._image = None
        self._image_t = -1e9
        self._summary = None
        self._summary_t = -1e9
        self._motor_power = None
        self._history = collections.deque(maxlen=int(g('history_length').value))

        self.create_subscription(Image, g('image_topic').value, self._image_cb,
                                 qos_profile_sensor_data)
        self.create_subscription(String, '/perception/summary', self._summary_cb, 10)
        self._intent_pub = self.create_publisher(String, '/claude/intent', 10)
        self._say_pub = self.create_publisher(String, '/claude/say', 10)
        self._surface_pub = self.create_publisher(String, '/motor_driver/surface', 10)
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(String, '/motor_driver/power', self._power_cb, latched)
        self._note_pub = self.create_publisher(String, '/claude/note', 10)

        self._stop_evt = threading.Event()
        self._thread = threading.Thread(target=self._loop, daemon=True)
        self._thread.start()
        self.get_logger().info(
            f'Claude agent ready: model={self.model} effort={self.effort} '
            f'period={self.period:.1f}s fallback={self.use_fallback} '
            f'memory={len(self.memory)} notes in {self.memory.path}')

    # ── Inputs ────────────────────────────────────────────────────────────────

    def _image_cb(self, msg: Image):
        with self._lock:
            self._image, self._image_t = msg, time.monotonic()

    def _power_cb(self, msg: String):
        try:
            self._motor_power = json.loads(msg.data)
        except json.JSONDecodeError:
            pass

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
            'motor_power': self._motor_power or 'unknown',
            'camera': 'attached' if b64 else 'unavailable',
            'recent_decisions_oldest_first': list(self._history),
            'memory_notes': self.memory.recent(self.memory_recall),
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
            raw = json.loads(text)
            decision = intent_mod.normalise(raw, self.min_dur, self.max_dur)
            say, note = intent_mod.extras(raw)
        except (TypeError, ValueError, AttributeError) as e:
            self.get_logger().warn(f'Unparseable decision ({e}), stop_reason='
                                   f'{response.stop_reason}: {text!r}')
            self._publish(intent_mod.stop_intent('unparseable decision'))
            return

        self._publish(decision)
        floor = raw.get('floor') if isinstance(raw, dict) else None
        current = (self._motor_power or {}).get('surface')
        if floor in ('hard_floor', 'carpet') and floor != current:
            self._surface_pub.publish(String(data=floor))
            self.get_logger().info(f'Switching motor power to {floor} mode')
        if say:
            self._say_pub.publish(String(data=say))
            self.get_logger().info(f'Says: "{say}"')
        if note:
            try:
                if self.memory.add(note):
                    self._note_pub.publish(String(data=note))
                    self.get_logger().info(f'Remembered: {note}')
            except OSError as e:
                self.get_logger().warn(f'Could not save note to {self.memory.path}: {e}')
        entry = {k: decision[k] for k in ('linear', 'angular', 'reason')}
        entry['front_clearance_m_at_decision'] = summary.get('front_clearance_m')
        self._history.append(entry)
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
