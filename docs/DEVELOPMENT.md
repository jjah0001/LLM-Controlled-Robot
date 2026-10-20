# Development guide

How the code is built, wired and tuned. The robot's own identity (who it is, its senses,
personality and memory) is in [`../CLAUDE.md`](../CLAUDE.md), which `claude_agent_node`
loads as Claude's system prompt at start-up.

## What this is

A Raspberry Pi differential-drive robot whose navigation decisions come from Claude.
It fuses an **RPi Camera**, a **Witmotion/M1CT D05 dTOF LiDAR** (360°, `/scan`) and a
**front HC-SR04 ultrasonic** sensor, asks Claude where to go, and drives an L298N motor
board with smooth, continuous motion.

- ROS 2 **Jazzy** on Ubuntu 24.04 (Raspberry Pi), Python `ament_python` packages.
- Workspace: `ros2_ws/` (build there, never at the repo root).
- LiDAR driver: [`jjah0001/D05_lidar`](https://github.com/jjah0001/D05_lidar) (`m1ct_d2`
  package, C++), pulled in with `ros2_ws/robot.repos`. Camera driver: `camera_ros`.

## Architecture (the one rule that matters)

**Claude never drives the motors directly.** It runs slowly (one decision every ~2 s plus API
latency); a local controller runs at 20 Hz and owns smoothness and safety.

```
 /scan (LaserScan, m1ct_d2) ─────┐
 ultrasonic/range (Range) ───────┤
                                 ▼
                  motion_controller_node (20 Hz)  ──► /cmd_vel ──► motor_driver_node ──► L298N
                   fuse sensors, ramp velocity,         (Twist)     twist→wheel PWM,
                   slow/stop near obstacles,                        0.5 s watchdog
                   steer off walls, e-stop
                     ▲                │
      /claude/intent │                │ /perception/summary (JSON, 5 Hz)
      (JSON String)  │                ▼
                  claude_agent_node (background thread, ~0.5 Hz)
                     ▲   camera frame + fused distances → Claude API → intent
 /camera/image_raw ──┘   (camera_ros)
```

- **Decision** (`claude_ros/intent.py`): `{"linear": -1..1, "angular": -1..1, "duration_s",
  "reason", "say", "note"}`. Motion values are fractions of the controller's `max_linear` /
  `max_angular` (`angular > 0` = turn left) and go to `/claude/intent`; `say` goes to
  `/claude/say`; `note` is saved to memory. Claude's reply is constrained to this schema with
  structured outputs (`output_config.format`).
- **Identity and memory**: the system prompt is the repo-root `CLAUDE.md` (found via the
  `persona_file` parameter, `$ROBOT_PERSONA_FILE`, or by walking up from the installed source,
  which works with `colcon build --symlink-install`) plus the operating rules in
  `claude_agent_node.py`. Notes are appended to `memory_file`
  (`~/.ros/robot_memory/notes.jsonl`) and the latest `memory_recall` notes go into every request.
  Delete that file to make the robot forget.
- An intent holds for `duration_s`, so motion continues between decisions. When it expires
  (API down, slow, refused) the controller ramps smoothly to a stop.
- Safety lives only in `claude_ros/control.py`; prompts are not a safety mechanism.

## Packages

| Package | Executables | Notes |
|---|---|---|
| `claude_ros` | `claude_agent_node`, `motion_controller_node` | The brain + local controller. Launch files and `config/robot.yaml` live here. |
| `motor_driver` | `motor_driver_node`, `keyboard_controller` | L298N driver on `/cmd_vel`; keyboard teleop for testing. |
| `ultrasonic_control` | `ultrasonic_publisher`, `motor_driver` (legacy) | Publishes `ultrasonic/range`. The legacy `motor_driver` here is an old drive-until-obstacle demo; **never run it alongside `motor_driver_node`** (same GPIO pins). |
| `py_hello_world` | `hello_pub`, `hello_sub` | ROS tutorial, unused by the robot. |

Pure logic is kept free of ROS imports so it unit-tests anywhere:
`claude_ros/perception.py` (scan → sectors/clearances, sensor fusion),
`claude_ros/control.py` (ramping, speed caps, avoidance, e-stop),
`claude_ros/intent.py` (schema + validation), `claude_ros/memory.py` (note store),
`motor_driver/kinematics.py` (twist → wheel duty).
Put new logic there and keep the `*_node.py` files as thin ROS wiring.

## Build, run, test

```bash
cd ros2_ws
vcs import src < robot.repos              # D05 LiDAR driver + camera_ros (first time)
# RPi Camera: also build Raspberry Pi's libcamera fork, see the README ("RPi Camera")
sudo apt install ros-jazzy-cv-bridge python3-opencv python3-numpy
pip install --break-system-packages -U anthropic   # or use a venv visible to ROS
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash

export ANTHROPIC_API_KEY=sk-ant-...
ros2 launch claude_ros robot.launch.py                       # everything
ros2 launch claude_ros robot.launch.py use_agent:=false      # no Claude (bench test)
ros2 launch claude_ros robot.launch.py mission:="Find the kitchen" model:=claude-haiku-5-5
```

Tests (no ROS or Pi needed):

```bash
cd ros2_ws/src/claude_ros && python3 -m pytest -q test/
cd ros2_ws/src/motor_driver && python3 -m pytest -q test/test_wheels.py
```

Useful checks on the robot:
- `ros2 topic echo /perception/summary`: fused view the agent sees (sectors, clearances, flags).
- `ros2 topic pub --once /claude/intent std_msgs/String "{data: '{\"linear\":0.5,\"angular\":0.2,\"duration_s\":3,\"reason\":\"test\"}'}"`:
  drive by hand through the same safety layer, with the agent disabled.
- `ros2 run motor_driver keyboard_controller`: raw teleop (bypasses the safety layer; only with
  the motion controller stopped, since both publish `/cmd_vel`).

## Configuration

All tunables live in `ros2_ws/src/claude_ros/config/robot.yaml`, one section per node.
Most important on a new robot:
1. **LiDAR orientation**: `lidar_yaw_offset_deg` / `lidar_reverse`. Put a box in front and check
   `lidar_sectors_m.front` drops; put it on the left and check `left` drops.
2. **Footprint**: `lidar_to_front_m`, `lidar_to_rear_m`, `robot_half_width_m`.
3. **Motors**: `max_wheel_speed_mps` (measure), `track_width_m`, `min_duty` (lowest duty that
   still turns the wheels), `kick_duty` / `kick_s` (start burst for carpet),
   `invert_left` / `invert_right`. The motor parameters can be changed live with
   `ros2 param set /motor_driver ...`; see the README ("Motor power").
4. **Agent**: `model` (default `claude-opus-5-5`; `claude-haiku-5-5` for lower latency/cost),
   `effort`, `decision_period_s`, `mission`, `persona_file`, `memory_file`, `memory_recall`.
5. **Personality**: edit the repo-root `CLAUDE.md`; restart the agent to apply.

## Hardware facts

- BCM pins: L298N IN1=17, IN2=22, ENA=18 (left), IN3=23, IN4=24, ENB=19 (right);
  HC-SR04 TRIG=4, ECHO=27 (ECHO is 5 V: use a divider to 3.3 V).
- LiDAR on `/dev/ttyUSB0` @ 230400; install `sc_mini.rules` from the driver repo for permissions
  (README, "LiDAR").
- RPi Camera via `camera_ros`, which needs Raspberry Pi's libcamera fork on Ubuntu
  (`ros2_ws/camera.repos`, README "RPi Camera").
- On a Pi 5, `RPi.GPIO` does not work: install `python3-rpi-lgpio` (drop-in replacement).

## Gotchas

- The `m1ct_d2` driver reports "no return" as `0.0` (not `inf`), publishes angles 0..2π, and
  names its node `c1ct_d2` (so its `params/m1ct_d2.yaml` never applies). The launch file renames
  it to `lidar` and passes parameters from `robot.yaml`; `perception.scan_to_polar` drops the zeros.
- The ultrasonic follows REP 117: `+inf` = nothing in range. A silent topic means a faulty sensor.
- `/cmd_vel` must have exactly one publisher while driving (the motion controller).
- Claude API: Python `anthropic` SDK, structured outputs, `fallbacks: "default"` with the
  `server-side-fallback-2026-07-01` beta on models that support it. Handle `stop_reason ==
  "refusal"` and API errors by publishing a stop intent; never block the ROS executor on the API.
- Don't commit `build/`, `install/` or `log/`.
