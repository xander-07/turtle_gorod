# Calibration

Do this before tuning SLAM or Nav2.

## Current observed result (2026-09-09)

First floor test with a straight command produced approximately:

- actual travel: 1.15 m
- odometry X: 0.33079 m
- lateral drift: ~0.07 m to the right
- odometry Y: -0.00283 m
- odometry yaw: about -1.27 deg

The linear odometry scale is therefore wrong by roughly 3.48x. Do **not** tune
Nav2 or wheel base yet. First measure the real encoder ticks per wheel revolution.
The current 3432 ticks/rev assumption is very likely incorrect.

A preliminary estimate from the straight-line scale alone would be about
987 ticks/rev, but this is **not** written into the firmware yet because direct
encoder measurement is more reliable.

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

## 2. Ticks per wheel revolution

The current firmware preserves the original assumption:

```text
78 gear ratio × 11 pulses/motor-rev × 4 quadrature = 3432 ticks/wheel-rev
```

Verify it directly before changing the firmware.

1. Stop motion and keep the robot safely powered so the encoder electronics remain active.
2. Mark both wheels at a clearly visible reference position.
3. Read `/wheel_ticks` once.
4. Rotate only the left wheel exactly 10 full revolutions by hand in the forward direction.
5. Read `/wheel_ticks` again.
6. `left_ticks_per_rev = abs(delta_left_ticks) / 10`.
7. Return to a stable position and repeat for the right wheel.
8. `right_ticks_per_rev = abs(delta_right_ticks) / 10`.

ROS commands:

```bash
ros2 topic echo /wheel_ticks --once
```

If the two sides differ by more than ~1%, inspect the encoders/wiring before
averaging them.

After direct measurement, update `ENCODER_TICKS_PER_WHEEL_REV` in
`firmware/Low_level.ino` and reflash the Uno.

## 3. Effective wheel diameter

Only do this after ticks/rev has been corrected.

Command a slow straight run over a measured distance (for example 2.0 m).

If odometry reports `D_odom` for actual distance `D_real`:

```text
diameter_new = diameter_old × D_real / D_odom
```

Repeat in both directions and use the average.

## 4. Effective wheel base

Only do this after ticks/rev and wheel diameter are correct.

Rotate the robot several full turns on a high-friction surface and independently
measure the actual angle.

If odometry reports `theta_odom` while the actual rotation is `theta_real`:

```text
wheel_base_new = wheel_base_old × theta_odom / theta_real
```

Use at least 5–10 turns to reduce measurement error.

## 5. PID

Tune on blocks first.

Suggested order:

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
