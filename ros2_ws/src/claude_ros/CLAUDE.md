# CLAUDE.md — claude_ros

## Package purpose
`claude_ros` makes Claude the autonomous navigation brain of this differential-drive robot.
On each decision tick it collects the latest sensor snapshot, sends it to the Claude API,
and translates the structured JSON response into a `/cmd_vel` command.

## Architecture

```
/scan  (LaserScan)  ──┐
ultrasonic/range ──────┤
                       ├──► ClaudeAgentNode ──► /cmd_vel ──► motor_driver_node ──► GPIO
/camera/image_raw ─────┘         │
                                  └── Anthropic API (HTTPS)
```

## Key file
`claude_ros/claude_agent_node.py` — the only executable node.

## How Claude is prompted
- **System prompt**: defines the robot, the sensor schema, and strict output format rules.
- **User message** (each tick):
  - Text block: ultrasonic distance + LiDAR sector summary (6 sectors, min dist per sector).
  - Image block: camera frame resized to 320×240, JPEG-encoded, base64.
- **Expected response**: a single JSON object `{"linear_x", "angular_z", "reason"}`.

## Parameters
| Parameter | Default | Description |
|---|---|---|
| `model` | `claude-haiku-4-5-20251001` | Anthropic model ID |
| `call_rate_hz` | `0.5` | How often to call Claude (calls/sec) |
| `max_linear` | `0.5` | Max absolute linear velocity (m/s) |
| `max_angular` | `0.5` | Max absolute angular velocity (rad/s) |
| `image_topic` | `/camera/image_raw` | Camera topic to subscribe to |
| `image_width` | `320` | Resize width before sending to Claude |
| `image_height` | `240` | Resize height before sending to Claude |
| `jpeg_quality` | `70` | JPEG compression quality (0–100) |

## Environment variables
| Variable | Required | Description |
|---|---|---|
| `ANTHROPIC_API_KEY` | Yes | Your Anthropic API key |

## Safety
- Any obstacle < 0.3 m in the forward direction is communicated to Claude via the system prompt rule; the node also hard-clamps velocities to `max_linear` / `max_angular`.
- If the API call fails or the response cannot be parsed, the node publishes a zero `Twist` (stop).
- If no `/cmd_vel` arrives for 0.5 s, `motor_driver_node` stops the motors independently.

## LiDAR sector mapping
Assuming a 0°-centred forward scan (index 0 = right, increasing counter-clockwise):

| Sector | Index range |
|---|---|
| front | 45–55 % |
| front_left | 55–75 % |
| left | 75–90 % |
| rear | 0–10 % + 90–100 % |
| right | 10–25 % |
| front_right | 25–45 % |

Adjust these percentages in `_summarise_scan()` if your LiDAR has a different zero convention.

## Modifying the behaviour
Edit `SYSTEM_PROMPT` in `claude_agent_node.py` to change navigation goals (e.g. "follow a person",
"stay in the corridor", "avoid all movement"). The JSON schema must stay the same.

## Dependencies
- Python: `anthropic`, `opencv-python`, `numpy`
- ROS 2: `rclpy`, `geometry_msgs`, `sensor_msgs`, `cv_bridge`
- Install Python deps: `pip install anthropic opencv-python numpy`
