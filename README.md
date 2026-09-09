# turtle_gorod

ROS 2 Humble stack for an autonomous differential-drive robot for the RTK Cup High League hackathon **«Движение по городу»**.

## Hardware currently assumed

- Raspberry Pi 4 8 GB, Ubuntu 22.04, ROS 2 Humble
- Arduino Uno as the lower-level wheel controller
- ZK-5AD motor driver
- 2 × JGA25-370B geared DC motors with quadrature encoders
- effective odometry wheel diameter: **60.13 mm**
- physical center-to-center wheel spacing: **245 mm**
- calibrated effective kinematic wheel base: **228.84 mm**
- measured encoder scale on 2026-09-09:
  - left: **897.4 ticks/wheel revolution**
  - right: **898.8 ticks/wheel revolution**
- YDLIDAR X3 / X3 Pro through USB adapter (YB-1)
- differential drive

## Competition constraints implemented in the architecture

The 2025 task uses a 4 × 4 m field made from 5 × 5 cells (0.8 m each). The robot must obey road signs/markings, stop for 2 seconds at each passenger pickup sign, and finish stopped before the parking sign. A prebuilt map is explicitly allowed.

This repository currently provides the **motion + odometry + LiDAR + SLAM + Nav2 foundation**. The next layer is semantic road-sign perception and the city mission state machine.

### Hardware gap for a complete competition solution

YDLIDAR can provide geometry/obstacles, but it cannot distinguish the printed traffic-sign classes (straight / left / right / forbidden turn / stop / parking). For a fully autonomous attempt with unknown sign placement the robot needs a camera. A USB UVC camera or Raspberry Pi camera is enough; the sign detector will be added as the next package once the camera/model/FOV are known.

## Repository layout

```text
firmware/Low_level.ino
ros2_ws/src/turtle_gorod/
  turtle_gorod/serial_bridge.py
  turtle_gorod/collision_guard.py
  launch/
  config/
docs/CALIBRATION.md
scripts/
```

## 1. Flash the Arduino from Ubuntu terminal

The repository can compile and upload `.ino` files directly from the Raspberry Pi/Ubuntu terminal using Ubuntu AVR packages, Arduino-Makefile and `avrdude`.

One-time terminal toolchain setup:

```bash
cd ~/turtle_gorod
bash scripts/install_arduino_cli.sh
```

The script name is kept for compatibility, but on this robot it installs the Ubuntu-packaged AVR/Arduino toolchain instead of downloading Arduino CLI. This avoids the MIREA network's third-party SSL certificate problem.

To compile and upload the current lower-level firmware to the connected Arduino Uno:

```bash
cd ~/turtle_gorod
bash scripts/flash_arduino.sh
```

The script automatically prefers the persistent Arduino `/dev/serial/by-id/...` device, compiles for Arduino Uno and uploads with `avrdude`.

You can also upload any other `.ino` file:

```bash
bash scripts/flash_arduino.sh /path/to/MySketch.ino
```

Or explicitly specify both sketch and port:

```bash
bash scripts/flash_arduino.sh /path/to/MySketch.ino /dev/ttyACM0
```

Before flashing, stop `ros2 launch turtle_gorod base.launch.py` and close serial monitors so the Arduino port is free.

Current robot firmware:

```text
firmware/Low_level.ino
```

It provides:

- non-blocking UART parsing;
- 50 Hz wheel PID;
- 20 Hz telemetry;
- atomic encoder snapshots;
- command watchdog (350 ms);
- differential odometry;
- safe stop on loss of Raspberry Pi;
- `SET_WHEELS_SPEED`, `SET_POSE`, `SET_COEFF`, `SET_PWM`, `STOP`, `PING`.

Telemetry format:

```text
TEL,ms,x_mm,y_mm,theta_rad,encL,encR,vL_mm_s,vR_mm_s,targetL,targetR,watchdog
```

## 2. Install Raspberry Pi dependencies

```bash
cd ~/turtle_gorod
bash scripts/install_dependencies.sh
```

Log out/in once if the script adds you to the `dialout` group.

Find persistent USB device names:

```bash
ls -l /dev/serial/by-id/
```

Prefer `/dev/serial/by-id/...` over `/dev/ttyUSB0`/`ttyACM0` whenever possible.

## 3. Build ROS 2 workspace

```bash
cd ~/turtle_gorod
bash scripts/build.sh
source ros2_ws/install/setup.bash
```

## 4. Base-only bench test

Put the robot on a stand first.

```bash
ros2 launch turtle_gorod base.launch.py
```

In another shell:

```bash
source ~/turtle_gorod/ros2_ws/install/setup.bash
ros2 topic echo /odom
```

Short forward command:

```bash
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist \
"{linear: {x: 0.10}, angular: {z: 0.0}}"
```

Because both ROS and Arduino have command timeouts, a one-shot command only causes a short movement. For continuous testing publish at a fixed rate or use teleop.

Raw encoder totals are available as:

```bash
ros2 topic echo /wheel_ticks --once
```

## 5. LiDAR test

```bash
ros2 launch turtle_gorod lidar.launch.py
```

Check:

```bash
ros2 topic hz /scan
ros2 topic echo /scan --once
```

X3 defaults in `config/ydlidar_x3.yaml`:

- 115200 baud
- triangle lidar
- 3 kHz sampling
- single-channel
- 0.10–8.0 m
- motor DTR enabled

If the scan is mirrored or rotated, do **not** compensate randomly in Nav2: first correct `inverted`/`reversion` and the `laser_frame` mounting yaw.

## 6. Full hardware bringup

```bash
ros2 launch turtle_gorod bringup.launch.py \
  lidar_x:=0.0 lidar_y:=0.0 lidar_z:=0.10 lidar_yaw:=0.0
```

`collision_guard` sits between `/cmd_vel` and `/cmd_vel_safe`. If `/scan` is stale, motion is blocked.

## 7. Build a competition map

Start hardware bringup, then:

```bash
ros2 launch turtle_gorod slam.launch.py
```

Drive around the entire field slowly and save:

```bash
mkdir -p ~/turtle_gorod/maps
ros2 run nav2_map_server map_saver_cli \
  -f ~/turtle_gorod/maps/gorod
```

The competition rules allow a map built during preparation, so localization against a stable map should be used for official attempts rather than remapping every run.

## 8. Run Nav2 on the saved map

```bash
ros2 launch turtle_gorod navigation.launch.py \
  map:=/home/turtle/turtle_gorod/maps/gorod.yaml
```

Set the initial pose in RViz (`2D Pose Estimate`) for the first tests. Automatic selection among competition start corners/directions will be implemented in the mission layer after the real map is available.

## Calibration

See:

```text
docs/CALIBRATION.md
```

Encoder ticks/revolution, effective rolling diameter and effective wheel base have been calibrated on the real robot. The next motion task is straight-line drift/PID verification under floor load before tuning SLAM/Nav2.
