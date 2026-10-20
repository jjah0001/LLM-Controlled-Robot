# claude_ros

Claude as the navigation brain of a differential-drive robot, with a local controller that
keeps the motion smooth and safe between Claude's decisions.

| Node | Rate | Role |
|---|---|---|
| `motion_controller_node` | 20 Hz | Fuses `/scan` (D05 dTOF LiDAR) and `ultrasonic/range`, executes the latest Claude intent with acceleration limits, slows/stops near obstacles, steers away from walls, publishes `/cmd_vel` and `/perception/summary`. |
| `claude_agent_node` | ~0.5 Hz | Sends the camera frame, `/perception/summary` and memory notes to Claude with the repo's `CLAUDE.md` as the system prompt; publishes motion on `/claude/intent`, speech on `/claude/say`, and saves notes. |

See the repository's `docs/DEVELOPMENT.md` for the full architecture, setup and tuning guide.

## Quick start

```bash
cd ros2_ws
vcs import src < robot.repos
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install && source install/setup.bash
export ANTHROPIC_API_KEY=sk-ant-...
ros2 launch claude_ros robot.launch.py
```

Launch arguments: `use_lidar`, `use_camera`, `use_agent`, `use_cliff` (true/false),
`lidar_port`, `surface` (`hard_floor`/`carpet`), `min_duty`, `max_duty`, `kick_duty`, `kick_s`,
`model`, `mission`, `params_file`.

## Topics

| Topic | Type | Direction |
|---|---|---|
| `/scan` | `sensor_msgs/LaserScan` | in (m1ct_d2) |
| `ultrasonic/range` | `sensor_msgs/Range` | in (ultrasonic_control) |
| `/cliff` | `std_msgs/String` (JSON) | in (cliff_publisher, IR edge sensors, `use_cliff:=true`) |
| `/camera/image_raw` | `sensor_msgs/Image` | in (camera_ros) |
| `/claude/intent` | `std_msgs/String` (JSON) | agent → controller |
| `/claude/say` | `std_msgs/String` | agent → (a text-to-speech node) |
| `/claude/note` | `std_msgs/String` | agent → (each new memory note) |
| `/perception/summary` | `std_msgs/String` (JSON) | controller → agent |
| `/cmd_vel` | `geometry_msgs/Twist` | controller → motor_driver |
| `/motor_driver/surface` | `std_msgs/String` | agent → motor_driver (`hard_floor`/`carpet`) |
| `/motor_driver/power` | `std_msgs/String` (JSON, latched) | motor_driver → agent |

## Changing the behaviour

See the repository README for LiDAR, camera, edge sensor and motor power setup.

Edit the repository's `CLAUDE.md` to change who the robot is (personality, goals, how it
speaks and what it remembers). Set the current task with `mission` in `config/robot.yaml` or
on the command line (`mission:="Follow the person in front of you"`). The decision schema in
`intent.py` is the contract with the controller; change both sides together.
