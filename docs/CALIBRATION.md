# Calibration

Do this before tuning SLAM or Nav2.

## Current observed result (2026-09-09)

First floor test with a straight command produced approximately:

- actual travel: 1.15 m
- odometry X: 0.33079 m
- lateral drift: ~0.07 m to the right
- odometry Y: -0.00283 m
- odometry yaw: about -1.27 deg

The original linear odometry scale was therefore wrong by roughly 3.48x.

Direct encoder measurement was then performed with 10 full wheel revolutions:

- left wheel: 8974 ticks / 10 = **897.4 ticks/rev**
- right wheel: 8988 ticks / 10 = **898.8 ticks/rev**
- difference between sides: about **0.16%**

The previous calculated assumption of 3432 ticks/rev was therefore incorrect.
The firmware now uses separate measured values for the left and right encoders.

The straight-line scale estimate made before the direct measurement (~987 ticks/rev)
did not include wheel-diameter error. With the measured encoder values applied, the
next road test is used to calibrate the effective wheel diameter.

The ROS bridge publishes raw encoder totals on `/wheel_ticks` as:

```text
[left_encoder_ticks, right_encoder_ticks]
```

## 1. Encoder direction

Lift the drive wheels.

Command +0.10 m/s straight ahead. Both `vL` and `vR` in telemetry must be positive.
If a wheel physically runs backward, change its `MOTOR_DIR_*`. If it physically
runs forward but its encoder speed is negative, change only `ENC_DIR_*`.

Current direction test passed:

- positive linear X: both wheels forward
- positive angular Z: left wheel backward, right wheel forward

At angular Z = 0.5 rad/s the right motor did not initially overcome its low-speed
dead zone, but at angular Z = 1.0 rad/s both wheels rotated in the correct directions.
Do not tune the dead-zone compensation until the robot is tested under floor load.

## 2. Ticks per wheel revolution — completed

Measured directly on 2026-09-09:

```text
LEFT_ENCODER_TICKS_PER_WHEEL_REV  = 897.4
RIGHT_ENCODER_TICKS_PER_WHEEL_REV = 898.8
```

These values are now written into `firmware/Low_level.ino`.

For future verification:

1. Stop motion and keep the encoder electronics powered.
2. Mark a wheel at a clear reference position.
3. Read `/wheel_ticks` once.
4. Rotate the selected wheel exactly 10 full revolutions by hand.
5. Read `/wheel_ticks` again.
6. `ticks_per_rev = abs(delta_ticks) / 10`.

ROS command:

```bash
ros2 topic echo /wheel_ticks --once
```

## 3. Effective wheel diameter — next step

Only do this after flashing the firmware with the corrected ticks/rev values.

Command a slow straight run over a carefully measured distance, ideally 1.0–2.0 m.

If odometry reports `D_odom` for actual distance `D_real`:

```text
diameter_new = diameter_old × D_real / D_odom
```

Current starting value is 69.0 mm. Do not change it from the old 1.15 m test,
because that test was made while the encoder scale was still wrong.

Repeat the road test after flashing the calibrated firmware. If possible, repeat
forward and backward and average the result.

## 4. Effective wheel base

Only do this after ticks/rev and wheel diameter are correct.

Rotate the robot several full turns on a high-friction surface and independently
measure the actual angle.

If odometry reports `theta_odom` while the actual rotation is `theta_real`:

```text
wheel_base_new = wheel_base_old × theta_odom / theta_real
```

Use at least 5–10 turns to reduce measurement error.

## 5. Straight-line drift / PID

The first floor run drifted about 7 cm to the right over 1.15 m. Do not compensate
for this by changing wheel base or encoder scale. Re-evaluate it after encoder and
wheel-diameter calibration.

If the drift remains, compare left/right target and measured wheel speeds under
floor load. Then tune PID/feed-forward or apply small per-wheel calibration rather
than adding an artificial yaw correction at the ROS layer.

Suggested PID tuning order:

1. set `Ki=0`, `Kd=0`;
2. increase `Kp` until both sides track without persistent oscillation;
3. set feedforward so steady-state PWM is mostly provided by `Kff`;
4. add a small `Ki` to remove remaining steady error;
5. use `Kd` only if needed.

Start with speeds 80, 150, 250 mm/s and compare target vs measured values.

## 6. LiDAR transform

Measure from `base_link` (robot center, X forward, Y left, Z up) to the LiDAR
optical/rotation center. Enter the transform in `bringup.launch.py` arguments.

A wrong LiDAR transform can look like bad wheel odometry during SLAM, so calibrate
it before judging SLAM quality.
