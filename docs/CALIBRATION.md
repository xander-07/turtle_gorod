# Calibration

Do this before tuning SLAM or Nav2.

## Current observed result (2026-09-09)

First floor test with the old encoder scale:

- actual travel: 1.15 m
- odometry X: 0.33079 m
- lateral drift: ~0.07 m to the right

Direct encoder measurement with 10 wheel revolutions:

- left wheel: 8974 ticks / 10 = **897.4 ticks/rev**
- right wheel: 8988 ticks / 10 = **898.8 ticks/rev**
- difference between sides: about **0.16%**

After flashing those measured encoder values, a 1.00 m floor test produced:

- odometry X: 1.14515 m
- odometry Y: -0.06933 m
- odometry displacement: ~1.14725 m
- raw ticks after the run: left 4775, right 4733
- odometry yaw: about -3.69 deg

Using wheel travel derived directly from the encoder totals with the old 69.0 mm
diameter gives an average odometric travel of about 1147.45 mm. Therefore:

```text
WHEEL_DIAMETER_MM = 69.0 × 1000 / 1147.45 ≈ 60.13 mm
```

The firmware now uses **60.13 mm** as the effective wheel diameter. This is an
effective rolling diameter for odometry calibration; it does not need to equal the
physical ruler measurement of the tire.

The ROS bridge publishes raw encoder totals on `/wheel_ticks` as:

```text
[left_encoder_ticks, right_encoder_ticks]
```

## 1. Encoder direction — completed

Current direction test passed:

- positive linear X: both wheels forward
- positive angular Z: left wheel backward, right wheel forward

At angular Z = 0.5 rad/s the right motor did not initially overcome its low-speed
dead zone, but at angular Z = 1.0 rad/s both wheels rotated correctly.

## 2. Ticks per wheel revolution — completed

```text
LEFT_ENCODER_TICKS_PER_WHEEL_REV  = 897.4
RIGHT_ENCODER_TICKS_PER_WHEEL_REV = 898.8
```

For future verification, rotate one selected wheel exactly 10 full revolutions and
compare `/wheel_ticks` before and after.

## 3. Effective wheel diameter — calibrated, verify once more

Current firmware value:

```text
WHEEL_DIAMETER_MM = 60.13
```

Repeat a carefully measured 1.00 m straight run after flashing this value. The
odometry displacement should now be close to 1.00 m. If the remaining scale error
is more than about 1–2%, calculate one final correction:

```text
diameter_new = diameter_old × D_real / D_odom
```

## 4. Effective wheel base — next geometric calibration

Only do this after the repeated 1 m test confirms the linear scale.

Rotate the robot several full turns on a high-friction surface and independently
measure the actual angle.

If odometry reports `theta_odom` while the actual rotation is `theta_real`:

```text
wheel_base_new = wheel_base_old × theta_odom / theta_real
```

Use at least 5–10 turns to reduce measurement error.

## 5. Straight-line drift / PID

Do not compensate straight-line drift by changing wheel base or adding an artificial
ROS yaw correction. Re-evaluate drift after the 60.13 mm diameter is flashed.

If drift remains, compare left/right target and measured wheel speeds under floor
load. Then tune PID/feed-forward or apply a small per-wheel calibration.

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
