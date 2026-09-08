# Calibration

Do this before tuning SLAM or Nav2.

## 1. Encoder direction

Lift the drive wheels.

Command +0.10 m/s straight ahead. Both `vL` and `vR` in telemetry must be positive.
If a wheel physically runs backward, change its `MOTOR_DIR_*`. If it physically
runs forward but its encoder speed is negative, change only `ENC_DIR_*`.

## 2. Ticks per wheel revolution

The current firmware preserves the original assumption:

```text
78 gear ratio × 11 pulses/motor-rev × 4 quadrature = 3432 ticks/wheel-rev
```

Verify it.

1. Mark one wheel.
2. Read `encL`/`encR`.
3. Rotate the wheel exactly 10 full revolutions by hand with power disabled.
4. Read the encoder again.
5. `ticks_per_rev = abs(delta_ticks) / 10`.
6. Repeat for both wheels.

If the two sides differ by more than ~1%, inspect the encoders/wiring before
averaging them.

Update `ENCODER_TICKS_PER_WHEEL_REV` in `firmware/Low_level.ino` and reflash.

## 3. Effective wheel diameter

Command a slow straight run over a measured distance (for example 2.0 m).

If odometry reports `D_odom` for actual distance `D_real`:

```text
diameter_new = diameter_old × D_real / D_odom
```

Repeat in both directions and use the average.

## 4. Effective wheel base

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
