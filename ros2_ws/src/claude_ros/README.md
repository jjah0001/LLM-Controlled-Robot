# claude_ros

Autonomous robot navigation package that uses [Claude](https://www.anthropic.com/claude) as the
decision-making brain. On each tick the node collects camera, LiDAR, and ultrasonic data, sends
it to the Claude API, and publishes the resulting velocity command to `/cmd_vel`.

## Prerequisites

| Requirement | Notes |
|---|---|
| ROS 2 Jazzy | Tested on Raspberry Pi (Ubuntu 24.04) |
| `motor_driver` package | Listens on `/cmd_vel` → GPIO |
| `ultrasonic_control` package | Publishes `ultrasonic/range` |
| `camera_ros` package | Publishes `/camera/image_raw` |
| Anthropic API key | `export ANTHROPIC_API_KEY=sk-ant-...` |
| Python deps | `pip install anthropic opencv-python numpy` |

## Quick start

```bash
# 1. Set your API key
export ANTHROPIC_API_KEY=sk-ant-...

# 2. Build
cd ~/Documents/LLM-Controlled-Robot/ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build --packages-select claude_ros
source install/setup.bash

# 3. Launch everything
ros2 launch claude_ros claude_ros.launch.py
```

## What gets launched

| Node | Package | Role |
|---|---|---|
| `ultrasonic_publisher` | `ultrasonic_control` | Reads HC-SR04, publishes `ultrasonic/range` |
| `motor_driver` | `motor_driver` | Drives L298N from `/cmd_vel` |
| `claude_agent` | `claude_ros` | Collects sensors → Claude API → `/cmd_vel` |

> **Note:** The camera node (`camera_ros`) must be started separately as it requires
> libcamera and a device argument. See `camera_ros/README.md`.

## Tuning

```bash
# Slower, more deliberate decisions
ros2 launch claude_ros claude_ros.launch.py call_rate_hz:=0.25

# Faster decisions, lower speed limits
ros2 launch claude_ros claude_ros.launch.py call_rate_hz:=1.0 max_linear:=0.3

# Use a smarter (but slower/more expensive) model
ros2 launch claude_ros claude_ros.launch.py model:=claude-sonnet-4-6
```

## Topic graph

```
/scan  ──────────────────────────────────┐
ultrasonic/range  ──────────────────────►│ /claude_agent
/camera/image_raw  ─────────────────────►│      │
                                          │      └──► /cmd_vel ──► /motor_driver ──► GPIO
                                          │
                               Anthropic API (HTTPS, ~0.5 Hz)
```

## Safety notes

- Claude is instructed to stop if any forward sensor reads < 0.3 m.
- If the API call fails the agent publishes a zero velocity (stop).
- `motor_driver_node` has its own 0.5 s watchdog: if `/cmd_vel` goes silent the motors stop.
- Keep `call_rate_hz` ≤ 1.0 to avoid hitting API rate limits.

## Customising Claude's behaviour

Open `claude_ros/claude_agent_node.py` and edit `SYSTEM_PROMPT`. Examples:

- Add a goal: *"Your mission is to patrol the room and return to the start."*
- Add a constraint: *"Never turn right."*
- Change the task: *"Follow the person in the camera image."*

The JSON output schema (`linear_x`, `angular_z`, `reason`) must remain unchanged.
