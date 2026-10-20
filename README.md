# LLM-Controlled-Robot

A Raspberry Pi 4 robot (ROS 2 Jazzy, Ubuntu 24.04) that fuses an RPi Camera, a Witmotion D05
dTOF LiDAR and a front ultrasonic sensor, asks Claude where to go, and drives with smooth,
continuous motion. A 20 Hz local controller executes Claude's decisions with acceleration
limits and an obstacle safety layer, so motion stays smooth and safe between API calls.

- [CLAUDE.md](CLAUDE.md) is the robot's identity (goals, hardware, senses, actions,
  personality, communication, memory). It is loaded as Claude's system prompt.
- [docs/DEVELOPMENT.md](docs/DEVELOPMENT.md) covers architecture, tuning and wiring.
- [`ros2_ws/src/claude_ros`](ros2_ws/src/claude_ros) lists the nodes and topics.

Contents: [Quick start](#quick-start) · [LiDAR setup](#lidar-setup-witmotion-d05) ·
[Camera setup](#rpi-camera-setup) · [Edge sensors](#edge-sensors-stops-falls) ·
[Motor power](#motor-power-pwm-and-carpet)

## Quick start

On the Pi, with ROS 2 Jazzy installed:

```bash
sudo apt install -y python3-vcstool python3-rosdep python3-colcon-common-extensions \
    ros-jazzy-cv-bridge python3-opencv python3-numpy
pip install --break-system-packages -U anthropic

cd ~/Documents/Miscellaneous_code/LLM-Controlled-Robot/ros2_ws   # wherever you cloned it
vcs import src < robot.repos          # LiDAR driver + camera_ros
source /opt/ros/jazzy/setup.bash
rosdep install -y --from-paths src --ignore-src --rosdistro jazzy --skip-keys=libcamera
colcon build --symlink-install
source install/setup.bash

export ANTHROPIC_API_KEY=sk-ant-...
ros2 launch claude_ros robot.launch.py
```

For the camera, also follow [RPi Camera setup](#rpi-camera-setup) (it needs Raspberry Pi's
libcamera, built once). Useful launch options: `use_agent:=false` (sensors and motors only, no Claude),
`use_camera:=false`, `use_lidar:=false`, `lidar_port:=/dev/ttyUSB1`, `surface:=carpet`, `min_duty:=85`, `kick_s:=0.25`,
`use_cliff:=true`,
`mission:="..."`, `model:=claude-haiku-5-5`. If the LiDAR driver or `camera_ros` isn't built,
the launch file skips it with a warning and starts the rest.

## LiDAR setup (Witmotion D05)

The LiDAR uses the `m1ct_d2` driver from [jjah0001/D05_lidar](https://github.com/jjah0001/D05_lidar).
`vcs import src < robot.repos` clones it into `ros2_ws/src/D05_lidar`.

1. **Plug it in and find the port.** The USB adapter shows up as `/dev/ttyUSB0`:
   ```bash
   ls /dev/ttyUSB*
   sudo dmesg | tail        # should show a ch341 or cp210x converter attached to ttyUSB0
   ```
   If the port shows up and then vanishes, Ubuntu's braille driver has taken it:
   `sudo apt remove brltty`, then plug the LiDAR in again.
2. **Give your user access to the port** (one-time; log out and back in afterwards):
   ```bash
   sudo usermod -aG dialout $USER
   sudo cp src/D05_lidar/m1ct_d2/sc_mini.rules /etc/udev/rules.d/
   sudo udevadm control --reload-rules && sudo udevadm trigger
   ```
   The rules also create a stable name: `/dev/sc_mini` (CH340 adapter) or `/dev/sc_m_c`
   (CP210x). Use it as `lidar_port:=/dev/sc_mini` if `ttyUSB0` keeps changing.
3. **Build it** (the Quick start build already does this):
   ```bash
   colcon build --symlink-install --packages-select m1ct_d2 && source install/setup.bash
   ```
4. **Test it on its own:**
   ```bash
   ros2 run m1ct_d2 m1ct_d2 --ros-args -p port:=/dev/ttyUSB0
   ros2 topic hz /scan          # in a second terminal: a steady rate means it works
   ```
   To see the scan, run `rviz2` (on the Pi or a laptop on the same network), add a
   **LaserScan** display on `/scan`, and set **Fixed Frame** to `laser_link`.
5. **Check the orientation.** This matters for safety. Start the stack without Claude and
   watch the fused view:
   ```bash
   ros2 launch claude_ros robot.launch.py use_agent:=false use_camera:=false
   ros2 topic echo /perception/summary
   ```
   Hold a box about 30 cm in front of the robot, and `lidar_sectors_m.front` should drop.
   Then move it to the robot's left, and `left` should drop. If the box shows up in the
   wrong sector, edit `config/robot.yaml`:
   - The front reading appears in another sector: set `lidar_yaw_offset_deg` to the angle
     between the LiDAR's zero mark and the robot's front (for example `180` or `-90`).
   - Left and right are swapped: set `lidar_reverse: true`.
   - Also set `lidar_to_front_m`, `lidar_to_rear_m` and `robot_half_width_m` to your robot's
     measurements, so the robot stops at the right distance.

| Problem | Fix |
|---|---|
| `Permission denied` on `/dev/ttyUSB0` | Do step 2, then log out and in. |
| `node_lidar init error` | Wrong port or the LiDAR isn't spinning. Check `ls /dev/ttyUSB*` and the USB cable. |
| `[robot.launch] m1ct_d2 not found` | Run `vcs import src < robot.repos` in `ros2_ws`, then rebuild. |
| Robot thinks something is always in front | The robot's own body is in the scan. Raise `lidar_self_filter_m`. |

## RPi Camera setup

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

If the camera still won't cooperate, the robot runs without it: launch with
`use_camera:=false`. If no frame newer than `image_timeout_s` is available, Claude is told the
camera is unavailable and decides from the LiDAR and ultrasonic distances alone.

## Edge sensors (stops falls)

The LiDAR and ultrasonic look straight ahead, so **they cannot see a drop**. A floor that ends
looks like open space to them. Software alone can't fix that. Without extra sensors, the only
protection is Claude watching the camera (it is told to keep at least 0.5 m from any edge it
sees), and that isn't reliable enough to count on. **Fit downward-facing IR edge sensors**
before letting the robot drive near stairs or on a table.

**Parts:** two digital IR reflectance modules (TCRT5000 boards, a few dollars each, with a
digital `DO`/`OUT` pin and a sensitivity potentiometer).

**Mounting:** one under each front corner, pointing straight down, about 5-10 mm above the
floor and **at least 5 cm in front of the wheels**. The robot needs that much room to stop.
Put them at the corners, not in the middle: if the robot meets an edge at a shallow angle, a
corner reaches it first.
Add a third one at the back (named e.g. `rear`) if the robot often reverses.

**Wiring** (power the modules from 3.3 V so their output is safe for the Pi's GPIO; any 3.3 V
and GND header pins will do):

| Module pin | front_left | front_right |
|---|---|---|
| VCC | 3.3 V (pin 1) | 3.3 V (pin 17) |
| GND | GND (pin 9) | GND (pin 25) |
| DO / OUT | GPIO5 (pin 29) | GPIO6 (pin 31) |

**Set up and test:**
1. With the robot on the floor, turn each module's potentiometer until its LED is on. Then lift
   the robot about 3 cm: the LED must go off. Check this on your darkest floor or carpet too,
   since dark surfaces reflect less IR.
2. Start the robot with the edge sensors on:
   ```bash
   ros2 launch claude_ros robot.launch.py use_cliff:=true
   ros2 topic echo /cliff          # second terminal: lift the front, "front" must become true
   ```
3. Bench test: push the robot slowly toward a table edge by hand (agent off:
   `use_agent:=false`) and check it stops while its front wheels are still well on the table.

Both modules can share the 3.3 V supply. Never power them from 5 V: their output would then
be 5 V, which can damage the Pi's GPIO.

**What the robot does:**
- **Edge ahead:** forward travel stops immediately, without the usual gentle slow-down. The
  robot may still turn in place or back away, and Claude is told to back off and turn away.
- **Sensors silent:** with `use_cliff:=true`, the robot will not drive forward or backward
  until `/cliff` is publishing again.
- **Loose wire:** reads as an edge, so the robot stops.
- **Very dark floor or rug:** may also read as an edge, because it reflects little IR, so the
  robot refuses to drive onto it. Turn the potentiometer up a little (step 1), but re-check
  that lifting the robot still turns the LED off.

Claude is also told to keep half a metre from any edge it sees in the camera and to note it in
memory, but the sensors are what actually stop a fall.

Pins, the "no floor" level and the debounce are under `cliff_publisher:` in `robot.yaml`. To
always use the sensors, change the `use_cliff` default to `'true'` in
`ros2_ws/src/claude_ros/launch/robot.launch.py`.

## Motor power (PWM and carpet)

Wheel commands are mapped onto a PWM duty range, `min_duty`..`max_duty`. `min_duty` is the
power used at the slowest speed, and it needs to be just high enough for the wheels to turn.
Carpet needs more power than a hard floor, so there are two presets:

| Setting (`motor_driver` in `config/robot.yaml`) | Default | Meaning |
|---|---|---|
| `surface` | `hard_floor` | Active preset: `hard_floor` or `carpet` |
| `hard_floor_min_duty` | `60` | Minimum duty % on wood, tiles, lino |
| `carpet_min_duty` | `80` | Minimum duty % on carpet and rugs |
| `min_duty` | `-1` | Set to 0-100 to override both presets (`-1` = use the preset) |
| `max_duty` | `100` | Duty % at full speed (lower it to cap power) |
| `kick_duty` | `100` | Duty % for a short burst when a wheel starts or reverses |
| `kick_s` | `0` (off) | Length of that burst in seconds. On carpet try `0.15`-`0.3` |

The **start kick** helps on carpet because a wheel needs more power to start turning than to
keep turning. With a kick, `min_duty` can stay lower, so slow driving stays smooth.

You can change these at start-up, or while the robot is running, without restarting:

```bash
ros2 launch claude_ros robot.launch.py surface:=carpet      # start in carpet mode
ros2 launch claude_ros robot.launch.py min_duty:=85 kick_s:=0.25

ros2 param set /motor_driver surface carpet                 # switch while running
ros2 param set /motor_driver carpet_min_duty 90.0           # tune the carpet preset
ros2 param set /motor_driver min_duty 70.0                  # fixed override
ros2 param set /motor_driver min_duty -1.0                  # back to the presets
ros2 param set /motor_driver kick_s 0.25                    # turn on the start kick
ros2 topic echo /motor_driver/power                         # what's active now
```

Type values with a decimal point (`90.0`, not `90`). Invalid values (outside 0-100, or a minimum
above the maximum) are rejected with a message, and the old value stays.
Scout can also switch the preset itself. If it keeps asking to move but its distance readings
don't change, it assumes the wheels are stalling and switches to `carpet`. It switches back
when the camera shows a hard floor again.

**Finding your numbers:** run `ros2 launch claude_ros robot.launch.py use_agent:=false`, then
in another terminal send a slow forward command through the safety layer:

```bash
ros2 topic pub --rate 2 /claude/intent std_msgs/String \
  "{data: '{\"linear\":0.15,\"angular\":0.0,\"duration_s\":1.0,\"reason\":\"duty test\"}'}"
```

Lower `min_duty` with `ros2 param set` until the wheels stop turning, then set the preset about
5 above that value. Do this once on a hard floor and once on carpet. Copy the results into
`config/robot.yaml` so they stick.
