# LLM-Controlled-Robot

A Raspberry Pi robot (ROS 2 Jazzy) that fuses an RPi Camera, a D05 dTOF LiDAR and a front
ultrasonic sensor, asks Claude where to go, and drives with smooth, continuous motion.
A 20 Hz local controller executes Claude's decisions with acceleration limits and an
obstacle safety layer, so motion stays smooth and safe between API calls.

- [CLAUDE.md](CLAUDE.md) is the robot's identity (goals, hardware, senses, actions,
  personality, communication, memory). It is loaded as Claude's system prompt.
- [docs/DEVELOPMENT.md](docs/DEVELOPMENT.md) covers architecture, tuning and wiring.
- [`ros2_ws/src/claude_ros`](ros2_ws/src/claude_ros) lists the nodes and topics.

```bash
cd ros2_ws && vcs import src < robot.repos
source /opt/ros/jazzy/setup.bash && colcon build --symlink-install && source install/setup.bash
export ANTHROPIC_API_KEY=sk-ant-...
ros2 launch claude_ros robot.launch.py
```

Each sensor is optional: `use_lidar:=false`, `use_camera:=false` and `use_agent:=false` turn
parts off, so you can bring the robot up one piece at a time. Without the camera, Claude still
navigates using the LiDAR and ultrasonic distances.

All commands below run on the Pi (Ubuntu 24.04, ROS 2 Jazzy), from `ros2_ws/`, with
`source /opt/ros/jazzy/setup.bash` done first. Settings live in
[`ros2_ws/src/claude_ros/config/robot.yaml`](ros2_ws/src/claude_ros/config/robot.yaml).

## LiDAR (D05 dTOF)

The driver is [`jjah0001/D05_lidar`](https://github.com/jjah0001/D05_lidar) (`m1ct_d2`
package), pulled in by `robot.repos`.

1. **Mount it** in the middle of the robot, level, and higher than everything else on the
   chassis, so its 360° view is not blocked by wires or the camera.
2. **Plug it in** over USB and check the port:
   ```bash
   ls /dev/ttyUSB*          # usually /dev/ttyUSB0
   sudo dmesg | tail        # should show a ch341 or cp210x converter attached to ttyUSB0
   ```
   If the port shows up and then vanishes, Ubuntu's braille driver has taken it:
   `sudo apt remove brltty`, then replug.
3. **Allow access without sudo** (once):
   ```bash
   sudo cp src/D05_lidar/m1ct_d2/sc_mini.rules /etc/udev/rules.d/
   sudo udevadm control --reload-rules && sudo udevadm trigger
   sudo usermod -aG dialout $USER     # log out and back in afterwards
   ```
   The rules also create a fixed name (`/dev/sc_mini` for CH340 adapters, `/dev/sc_m_c` for
   CP2102), which stays the same even when other USB serial devices are plugged in. Use it with
   `lidar_port:=/dev/sc_mini`.
4. **Build and test it alone**:
   ```bash
   colcon build --symlink-install && source install/setup.bash
   ros2 launch claude_ros robot.launch.py use_camera:=false use_agent:=false
   ros2 topic hz /scan                     # in a second terminal: about 10 Hz
   ```
5. **Line up the directions.** Hold a box in front of the robot and watch
   `ros2 topic echo /perception/summary`: `lidar_sectors_m.front` must drop. Move it to the
   left and `left` must drop. If not, adjust `lidar_yaw_offset_deg` (rotate) and
   `lidar_reverse` (left/right swapped) in `robot.yaml`. Then measure `lidar_to_front_m`,
   `lidar_to_rear_m` and `robot_half_width_m`, so the robot stops at the right distance.

| Problem | Fix |
|---|---|
| `Permission denied` on the port | Step 3 (or `sudo chmod 666 /dev/ttyUSB0` until reboot). |
| `node_lidar init error` / node exits after ~3 s | Wrong port or the LiDAR is not responding: check `lidar_port`, unplug and replug. |
| `[robot.launch] m1ct_d2 not found` | Run `vcs import src < robot.repos`, then rebuild. |

## RPi Camera

**Why it usually fails on Ubuntu:** the `ros-jazzy-camera-ros` apt package comes with the
*upstream* libcamera, which may not support your camera module (the Camera Module 3, for
example, only works with Raspberry Pi's fork). Full Pi camera support on this robot's
**Raspberry Pi 4** comes from **Raspberry Pi's libcamera fork**, built from source next to
`camera_ros`. Ubuntu also blocks the memory buffers libcamera needs, unless you allow them
(step 3).

1. **Connect the cable** with the Pi powered off. On the Pi 4, the camera port is the
   connector labelled CAMERA, between the micro-HDMI ports and the audio jack (not the DISPLAY
   connector by the SD card). Lift the latch, insert the ribbon with its metal contacts facing
   the HDMI ports (blue tab towards the USB/Ethernet ports), and press the latch down.
2. **Enable the camera** in `/boot/firmware/config.txt`, then reboot:
   ```ini
   camera_auto_detect=1
   dtoverlay=vc4-kms-v3d
   ```
   Remove or comment out any `start_x=1` line (the old legacy camera stack, which libcamera
   cannot use). If it is still not found, turn auto detection off and name the sensor:
   ```ini
   camera_auto_detect=0
   dtoverlay=imx708        # Camera Module 3   (imx219 = Module 2, ov5647 = Module 1, imx477 = HQ)
   ```
   Check that the kernel sees it: `sudo dmesg | grep -iE 'imx|ov5647'` should list the sensor,
   and `ls /dev/media*` should show some devices.
3. **Allow camera access** (once):
   ```bash
   sudo usermod -aG video $USER
   echo 'SUBSYSTEM=="dma_heap", GROUP="video", MODE="0660"' | sudo tee /etc/udev/rules.d/99-dma-heap.rules
   sudo udevadm control --reload-rules && sudo udevadm trigger   # then log out and back in
   ```
4. **Remove the apt camera packages**, if you installed them, so they don't get picked up
   instead of the fork:
   ```bash
   sudo apt remove ros-jazzy-camera-ros ros-jazzy-libcamera
   ```
5. **Build libcamera (Raspberry Pi fork) and camera_ros** in the workspace. The first build
   takes about 20-40 minutes on a Pi 4:
   ```bash
   sudo apt install -y python3-colcon-meson meson ninja-build pkg-config python3-yaml python3-ply \
     python3-jinja2 libyaml-dev libgnutls28-dev openssl libdw-dev libunwind-dev libudev-dev \
     libevent-dev libdrm-dev libjpeg-dev libtiff-dev nlohmann-json3-dev
   vcs import src < camera.repos            # clones raspberrypi/libcamera into src/libcamera
   rosdep install -y --from-paths src --ignore-src --rosdistro jazzy --skip-keys=libcamera
   colcon build --packages-select libcamera --meson-args \
     -Dpipelines=rpi/vc4 -Dipas=rpi/vc4 -Dcam=enabled \
     -Dgstreamer=disabled -Dqcam=disabled -Dpycamera=disabled -Ddocumentation=disabled \
     -Dtest=false -Dlc-compliance=disabled
   colcon build --symlink-install           # camera_ros and the rest; later builds just use this
   source install/setup.bash
   ```
   (`rpi/vc4` is the Pi 4's camera pipeline.) If the Pi runs out of memory while building
   (likely on a 1-2 GB Pi 4), add `--executor sequential`, and if it still fails, add swap:
   `sudo fallocate -l 2G /swapfile && sudo chmod 600 /swapfile && sudo mkswap /swapfile && sudo swapon /swapfile`.
6. **Test it**:
   ```bash
   cam -l                                   # must list the camera, e.g. 'imx708_wide'
                                            # (if `cam` is not found: install/libcamera/bin/cam -l)
   ros2 launch claude_ros robot.launch.py use_lidar:=false use_agent:=false
   ros2 topic hz /camera/image_raw          # in a second terminal: a steady rate
   ```
   To see the picture, run `ros2 run rqt_image_view rqt_image_view` on a computer on the
   same network (`sudo apt install ros-jazzy-rqt-image-view`).

| Problem | Fix |
|---|---|
| `no cameras available` | `cam -l` is empty too: check the cable and step 2. `cam -l` works: camera_ros is using the wrong libcamera (step 4, then rebuild from clean: `rm -rf build/camera_ros install/camera_ros`). |
| `Could not open any dma-buf provider` / `Failed to allocate buffers` | Step 3 (log out and back in). Still failing: lower `width`/`height` in `robot.yaml`. |
| Image is zoomed in (Camera Module 2) | Set `sensor_mode: "1640:1232"` under `camera:` in `robot.yaml`. |
| Image is upside down | Set `orientation: 180` under `camera:` in `robot.yaml`. |
| `[robot.launch] camera_ros not found` | `camera_ros` didn't build: rerun step 5 and read the first error. |
| Need more detail | `LIBCAMERA_LOG_LEVELS=*:DEBUG ros2 run camera_ros camera_node` |

The agent reads `/camera/image_raw`. If no frame newer than `image_timeout_s` is available,
it tells Claude the camera is unavailable and decides from the distances alone.

## Motor power (carpet, rugs)

Carpet needs more power than a hard floor, both to get the wheels moving and to keep them
turning. The motor driver maps each wheel's speed onto a PWM duty between `min_duty` (slowest
speed) and `max_duty` (full speed). It can also give a short **start kick**: a burst of
`kick_duty` for `kick_s` seconds whenever a wheel starts or reverses.

| Parameter | Default | Carpet starting point | What it does |
|---|---|---|---|
| `min_duty` | 60 | 75-90 | Duty (%) at the slowest speed. Raise it if the robot hums but doesn't move. |
| `max_duty` | 100 | 100 | Duty (%) at full speed. |
| `kick_duty` | 100 | 100 | Duty (%) during the start kick. |
| `kick_s` | 0 (off) | 0.15-0.3 | Length of the start kick. Lets `min_duty` stay lower for smoother slow driving. |

Change them **while the robot is running** (takes effect immediately, nothing to rebuild).
Type the values with a decimal point:

```bash
ros2 param set /motor_driver min_duty 80.0
ros2 param set /motor_driver kick_s 0.25
ros2 param get /motor_driver min_duty
```

or **at launch**:

```bash
ros2 launch claude_ros robot.launch.py min_duty:=80 kick_s:=0.25
```

To keep a setting, put it in the `motor_driver:` section of `robot.yaml`. Find the lowest
`min_duty` where both wheels start turning on your floor, using
`ros2 launch claude_ros robot.launch.py use_agent:=false` and short test commands (see
"Useful checks" in [docs/DEVELOPMENT.md](docs/DEVELOPMENT.md)), then add about 5. A value that
is too high makes slow driving jerky. Out-of-range values (e.g. `min_duty` above `max_duty`)
are rejected with an error, and the old value stays.
