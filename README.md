# LLM-Controlled-Robot

A Raspberry Pi robot (ROS 2 Jazzy) that fuses an RPi Camera, a D05 dTOF LiDAR and a front
ultrasonic sensor, asks Claude where to go, and drives with smooth, continuous motion.
A 20 Hz local controller executes Claude's decisions with acceleration limits and an
obstacle safety layer, so motion stays smooth and safe between API calls.

- [CLAUDE.md](CLAUDE.md) is the robot's identity (goals, hardware, senses, actions,
  personality, communication, memory). It is loaded as Claude's system prompt.
- [docs/DEVELOPMENT.md](docs/DEVELOPMENT.md) covers architecture, setup, tuning and wiring.
- [`ros2_ws/src/claude_ros`](ros2_ws/src/claude_ros) lists the nodes and topics.

```bash
cd ros2_ws && vcs import src < robot.repos
source /opt/ros/jazzy/setup.bash && colcon build --symlink-install && source install/setup.bash
export ANTHROPIC_API_KEY=sk-ant-...
ros2 launch claude_ros robot.launch.py
```
