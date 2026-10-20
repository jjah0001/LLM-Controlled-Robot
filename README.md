# LLM-Controlled-Robot

**Scout** is a small Raspberry Pi 4 robot that asks Claude where to go. It combines a camera,
a 360° LiDAR and a front ultrasonic sensor, and it drives smoothly while a local safety layer
keeps it off walls, people and edges. It runs ROS 2 Jazzy on Ubuntu 24.04.

**New to the robot? Start at [Step 1](#step-1-put-ubuntu-on-the-sd-card) and work down.**
The steps assume you have never used a Raspberry Pi or ROS before, and each one ends with a
check, so you know it worked before moving on.

| | |
|---|---|
| **Getting started** | [What you need](#what-you-need) · [1. Ubuntu](#step-1-put-ubuntu-on-the-sd-card) · [2. Connect](#step-2-connect-to-the-pi) · [3. ROS 2 Jazzy](#step-3-install-ros-2-jazzy) · [4. Robot code](#step-4-get-and-build-the-robot-code) · [5. First drive](#step-5-check-the-wiring-and-drive-by-keyboard) · [6. Run Scout](#step-6-let-claude-drive) |
| **Hardware setup** | [LiDAR](#lidar-setup-witmotion-d05) · [Camera](#rpi-camera-setup) · [Edge sensors](#edge-sensors-stops-falls) · [Motor power](#motor-power-pwm-and-carpet) |
| **More** | [CLAUDE.md](CLAUDE.md) (Scout's personality and rules) · [docs/DEVELOPMENT.md](docs/DEVELOPMENT.md) (how the code works) |

## What you need

- The robot, which has a **Raspberry Pi 4** (2 GB or more; 4 GB is easier), an L298N motor
  driver, two motors, the LiDAR, the camera and the ultrasonic sensor.
- A **microSD card**, 32 GB or larger, and a way to plug it into your computer.
- **Power:** a charged battery for the motors, and a 5 V 3 A USB-C supply or power bank for the
  Pi. Use the mains supply while installing, because it takes a while.
- A computer (Mac, Windows or Linux) on the **same Wi-Fi** as the robot, and your Wi-Fi name
  and password.
- An **Anthropic API key**, so Claude can drive. You can make one at
  [console.anthropic.com](https://console.anthropic.com/). Everything up to Step 6 works
  without it.

You don't need a screen or keyboard for the Pi: you'll control it from your computer over
Wi-Fi.

## Step 1: Put Ubuntu on the SD card

ROS 2 Jazzy runs on **Ubuntu 24.04**. Raspberry Pi OS won't work, because the ready-made ROS 2
Jazzy packages are built for Ubuntu 24.04 only.

1. On your computer, install **Raspberry Pi Imager** from
   [raspberrypi.com/software](https://www.raspberrypi.com/software/) and open it.
2. **Choose Device:** Raspberry Pi 4.
3. **Choose OS:** Other general-purpose OS → Ubuntu → **Ubuntu Server 24.04.x LTS (64-bit)**.
   Pick "Server", not "Desktop": the robot has no screen, and Server leaves more memory free.
4. **Choose Storage:** your microSD card. Everything on it will be erased.
5. Click **Next**, then **Edit Settings** when asked about OS customisation:
   - **General:** set the hostname to `scout`, choose a username (for example `robot`) and a
     password, enter your Wi-Fi name and password, and set your Wi-Fi country and time zone.
   - **Services:** tick **Enable SSH** and "Use password authentication".
   - Click **Save**, then **Yes** to apply the settings, then **Yes** to write the card.
6. When it says the write is finished, take out the card and put it in the Pi (the slot is on
   the underside).

## Step 2: Connect to the Pi

1. Power on the Pi and **wait about 5 minutes**. The first boot sets everything up and may
   restart once. If it never shows up on your network, unplug the power, plug it back in and
   wait another 5 minutes. Wi-Fi sometimes only connects on the second boot.
2. On your computer, open a terminal: **Terminal** on a Mac, **PowerShell** on Windows. Then
   connect with the username you chose:
   ```bash
   ssh robot@scout.local
   ```
   Type `yes` the first time it asks about the fingerprint, then your password. Your prompt
   changes to `robot@scout:~$`, which means you are now typing **on the Pi**.

   If `scout.local` isn't found, look up the Pi's IP address in your Wi-Fi router's app or
   admin page (it's listed as `scout`) and use that, for example `ssh robot@192.168.1.42`.
3. Make `scout.local` always work, and bring the Pi up to date (this takes a few minutes):
   ```bash
   sudo apt update && sudo apt full-upgrade -y
   sudo apt install -y avahi-daemon git curl
   sudo reboot
   ```
   If apt says `Could not get lock`, Ubuntu is still installing updates in the background.
   Wait a few minutes and try again. After `sudo reboot`, wait a minute and `ssh` in again.

**Every command from here on runs on the Pi, over SSH**, unless a step says otherwise.

## Step 3: Install ROS 2 Jazzy

ROS 2 (Robot Operating System) is the framework the robot's programs use to talk to each
other. These are the [official install steps](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html).
Copy each block in turn and paste it into the SSH window.

1. Set a UTF-8 language setting:
   ```bash
   sudo apt update && sudo apt install -y locales
   sudo locale-gen en_US en_US.UTF-8
   sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
   export LANG=en_US.UTF-8
   ```
2. Add the ROS 2 package source:
   ```bash
   sudo apt install -y software-properties-common
   sudo add-apt-repository -y universe
   export ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F "tag_name" | awk -F'"' '{print $4}')
   curl -L -o /tmp/ros2-apt-source.deb "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo ${UBUNTU_CODENAME:-${VERSION_CODENAME}})_all.deb"
   sudo dpkg -i /tmp/ros2-apt-source.deb
   ```
3. Install ROS 2 Jazzy. Use **ros-base**, which has no graphical tools, since the robot has no
   screen. This downloads about 1 GB and takes 10-20 minutes:
   ```bash
   sudo apt update && sudo apt upgrade -y
   sudo apt install -y ros-jazzy-ros-base ros-dev-tools ros-jazzy-demo-nodes-py
   ```
   If this fails with "unmet dependencies", run `grep Suites /etc/apt/sources.list.d/ubuntu.sources`.
   The line must read `Suites: noble noble-updates noble-backports`. Fix it with
   `sudo nano /etc/apt/sources.list.d/ubuntu.sources`, then run the two commands again.
4. Load ROS automatically in every new terminal, and set up `rosdep`, the tool that installs
   the robot code's dependencies:
   ```bash
   echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
   source ~/.bashrc
   sudo rosdep init
   rosdep update
   ```
5. **Check it works.** Start a test publisher:
   ```bash
   ros2 run demo_nodes_py talker
   ```
   Open a **second** terminal on your computer, `ssh robot@scout.local` again, and run
   `ros2 run demo_nodes_py listener`. If the second window prints `I heard: [Hello World: 1]`
   and counts up, ROS 2 works. Press **Ctrl+C** in both windows to stop.

## Step 4: Get and build the robot code

1. Give your user access to the GPIO pins (motors and sensors) and the USB serial port (LiDAR),
   and install the Python packages the robot needs:
   ```bash
   sudo usermod -aG dialout,video $USER
   echo 'SUBSYSTEM=="gpio", KERNEL=="gpiochip*", GROUP="dialout", MODE="0660"' | \
       sudo tee /etc/udev/rules.d/99-gpio.rules
   sudo apt install -y python3-pip python3-rpi-lgpio
   pip install --break-system-packages -U anthropic
   sudo reboot
   ```
   `python3-rpi-lgpio` provides the `RPi.GPIO` library the robot's code uses, in a version that
   works on Ubuntu 24.04. After the reboot, `ssh` in again.
2. Download the code and the LiDAR and camera drivers:
   ```bash
   git clone https://github.com/jjah0001/LLM-Controlled-Robot.git ~/LLM-Controlled-Robot
   cd ~/LLM-Controlled-Robot/ros2_ws
   vcs import src < robot.repos
   ```
3. Install the dependencies and build. The build takes a few minutes:
   ```bash
   rosdep install -y --from-paths src --ignore-src --rosdistro jazzy --skip-keys=libcamera
   colcon build --symlink-install
   echo "source ~/LLM-Controlled-Robot/ros2_ws/install/setup.bash" >> ~/.bashrc
   source ~/.bashrc
   ```
   It's fine if the build reports that `camera_ros` failed. The camera needs extra steps,
   covered in [RPi Camera setup](#rpi-camera-setup). The rest of the robot works without it.
   If anything else fails, read the first error it prints.
4. **Check the GPIO pins work** without `sudo`:
   ```bash
   python3 -c "import RPi.GPIO as G; G.setmode(G.BCM); G.setup(17, G.OUT); G.cleanup(); print('GPIO OK')"
   ```
   It should print `GPIO OK`. If it says `Permission denied`, check that you rebooted after
   item 1, and that `ls -l /dev/gpiochip0` shows `dialout`.

To get updates later: `cd ~/LLM-Controlled-Robot && git pull`, then repeat the `colcon build`
line in `ros2_ws`.

## Step 5: Check the wiring and drive by keyboard

**Lift the robot so its wheels are off the ground** (put it on a box) for this whole step.

Check the wiring against this table. Pin numbers are the Pi's physical header pins, with the
GPIO (BCM) number in brackets. [pinout.xyz](https://pinout.xyz/) shows where each pin is.

| Part | Connection |
|---|---|
| L298N left motor | IN1 → pin 11 (GPIO17), IN2 → pin 15 (GPIO22), ENA → pin 12 (GPIO18) |
| L298N right motor | IN3 → pin 16 (GPIO23), IN4 → pin 18 (GPIO24), ENB → pin 35 (GPIO19) |
| L298N power | Motor battery + to `12V`, battery − to `GND`, **and** L298N `GND` to a Pi GND pin (for example pin 6). Remove the ENA/ENB jumpers so the Pi controls the speed. |
| Front ultrasonic (HC-SR04) | VCC → 5 V (pin 2), GND → GND, TRIG → pin 7 (GPIO4), ECHO → pin 13 (GPIO27) **through a voltage divider** (for example 1 kΩ and 2 kΩ), because ECHO is 5 V and would damage the Pi |
| LiDAR | USB port. See [LiDAR setup](#lidar-setup-witmotion-d05) |
| Camera | Ribbon cable in the CAMERA connector. See [RPi Camera setup](#rpi-camera-setup) |
| Edge sensors (optional) | See [Edge sensors](#edge-sensors-stops-falls) |

Then drive the motors with the keyboard. In one terminal, start the motor driver:
```bash
ros2 run motor_driver motor_driver_node --ros-args \
  --params-file ~/LLM-Controlled-Robot/ros2_ws/src/claude_ros/config/robot.yaml
```
In a second terminal (`ssh` in again), start the keyboard control:
```bash
ros2 run motor_driver keyboard_controller
```
Press **W** (forward), **S** (back), **A** (left), **D** (right), **X** (stop) and **Q**
(quit). Hold a key down to keep moving: the wheels stop by themselves about half a second
after the last key press.

- If a wheel turns backwards on **W**, set `invert_left: true` or `invert_right: true` under
  `motor_driver:` in `ros2_ws/src/claude_ros/config/robot.yaml`. Then restart the motor driver.
- If **A** turns right, both wheels are swapped: swap the left and right motor wires.
- If the wheels hum but don't turn, the motor power is too low. See
  [Motor power](#motor-power-pwm-and-carpet).

Press **Ctrl+C** in both terminals when you're done. Next, set up the
[LiDAR](#lidar-setup-witmotion-d05) and check its directions. The robot uses the LiDAR to
avoid walls, so do this before letting it drive on its own. Then set up the
[camera](#rpi-camera-setup).

## Step 6: Let Claude drive

1. Store your API key on the Pi. It is a secret, so don't share it or commit it to git:
   ```bash
   echo 'export ANTHROPIC_API_KEY=sk-ant-...your key...' >> ~/.bashrc
   source ~/.bashrc
   ```
2. Put the robot on the floor in an open space, **away from stairs and table edges** unless
   you've fitted the [edge sensors](#edge-sensors-stops-falls). Stay close by the first time.
3. Start everything:
   ```bash
   ros2 launch claude_ros robot.launch.py
   ```
   Each decision appears in the log as a line from `claude_agent`, such as
   `lin=+0.40 ang=+0.10 for 4.0s | Heading for the open doorway`, along with anything Scout
   says or remembers. Press **Ctrl+C** to stop it: the motors stop straight away.

Useful options, added to the end of the launch line:

| Option | What it does |
|---|---|
| `mission:="Find the kitchen"` | Gives Scout a task. The default is to explore safely. |
| `use_agent:=false` | Starts the sensors and safety layer without Claude (no API key needed) |
| `use_camera:=false` / `use_lidar:=false` | Runs without that sensor |
| `use_cliff:=true` | Turns on the [edge sensors](#edge-sensors-stops-falls) |
| `surface:=carpet` | More motor power for carpet. See [Motor power](#motor-power-pwm-and-carpet) |
| `lidar_port:=/dev/ttyUSB1` | Uses a different LiDAR port |
| `model:=claude-haiku-5-5` | Uses a faster, cheaper Claude model |

All the settings are in `ros2_ws/src/claude_ros/config/robot.yaml`. To change Scout's
personality and rules, edit [CLAUDE.md](CLAUDE.md). Both take effect the next time you launch.

## LiDAR setup (Witmotion D05)

The LiDAR uses the `m1ct_d2` driver from [jjah0001/D05_lidar](https://github.com/jjah0001/D05_lidar).
`vcs import src < robot.repos` clones it into `ros2_ws/src/D05_lidar`. Run the commands below
on the Pi, in `~/LLM-Controlled-Robot/ros2_ws` (`cd ~/LLM-Controlled-Robot/ros2_ws` first).

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

Run the commands below on the Pi, in `~/LLM-Controlled-Robot/ros2_ws`.

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
