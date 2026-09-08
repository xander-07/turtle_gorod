# turtle_gorod

ROS 2 Humble stack for an autonomous differential-drive robot for the RTK Cup High League hackathon **«Движение по городу»**.

## Hardware currently assumed

- Raspberry Pi 4 8 GB, Ubuntu 22.04, ROS 2 Humble
- Arduino Nano as the lower-level wheel controller
- ZK-5AD motor driver
- 2 × JGA25-370B geared DC motors with quadrature encoders
- wheel diameter: **69 mm** (from the supplied firmware)
- wheel base: **185 mm** (from the supplied firmware)
- YDLIDAR X3 / X3 Pro through USB adapter (YB-1)
- differential drive

> **Important:** the original firmware uses `GEAR_RATIO=78`, `11` motor encoder pulses/rev and x4 quadrature decoding = **3432 ticks/wheel revolution**. Do not assume this is correct only from the motor name. Calibrate it on the real robot before tuning Nav2.

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

## 1. Flash the Arduino

Flash:

```text
firmware/Low_level.ino
```

The firmware adds:

- non-blocking UART parsing;
- 50 Hz wheel PID;
- 20 Hz telemetry;
- atomic encoder snapshots;
- command watchdog (350 ms);
- differential odometry;
- safe stop on loss of Raspberry Pi;
- backward-compatible `SET_WHEELS_SPEED`, `SET_POSE`, `SET_COEFF`, `SET_PWM`.

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

Prefer `/dev/serial/by-id/...` over `/dev/ttyUSB0`/`1` when both Arduino and YDLIDAR are connected.

## 3. Build

```bash
cd ~/turtle_gorod
bash scripts/build.sh
source ros2_ws/install/setup.bash
```

## 4. Base-only bench test

Put the robot on a stand first.

```bash
ros2 launch turtle_gorod base.launch.py \
  serial_port:=/dev/serial/by-id/YOUR_ARDUINO
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

Because both ROS and Arduino have command timeouts, a one-shot command only causes a short movement. For continuous testing use `teleop_twist_keyboard`.

## 5. LiDAR test

```bash
ros2 launch turtle_gorod lidar.launch.py \
  lidar_port:=/dev/serial/by-id/YOUR_YDLIDAR \
  lidar_x:=0.0 lidar_y:=0.0 lidar_z:=0.10 lidar_yaw:=0.0
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
  serial_port:=/dev/serial/by-id/YOUR_ARDUINO \
  lidar_port:=/dev/serial/by-id/YOUR_YDLIDAR \
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
  map:=/home/ubuntu/turtle_gorod/maps/gorod.yaml
```

Set the initial pose in RViz (`2D Pose Estimate`) for the first tests. Automatic selection among competition start corners/directions will be implemented in the mission layer after the real map is available.

## What I need from the first test

Please send the terminal output of:

```bash
ros2 topic hz /scan
ros2 topic echo /odom --once
ros2 run tf2_ros tf2_echo odom base_link
```

and tell me:

1. exact Arduino serial device;
2. exact YDLIDAR serial device;
3. LiDAR position relative to robot center: X, Y, Z and whether its cable/zero mark points forward;
4. chassis outer length × width;
5. whether a camera is already installed (model + resolution/FOV if known).

Then the next commit can lock the geometry, calibrate odometry, and add semantic sign recognition + the topological mission planner for the actual RTK city field.
