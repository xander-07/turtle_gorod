# Calibration

Do this before tuning SLAM or Nav2.

## Current calibrated values (2026-09-09)

Measured encoder scale from 10 full wheel revolutions:

```text
LEFT_ENCODER_TICKS_PER_WHEEL_REV  = 897.4
RIGHT_ENCODER_TICKS_PER_WHEEL_REV = 898.8
```

Straight-line calibration produced the current effective rolling diameter:

```text
WHEEL_DIAMETER_MM = 60.13
```

The physical center-to-center distance between the left and right driven wheels is **245 mm**.
For differential-drive odometry, the effective kinematic wheel base was calibrated with two independent five-turn tests on the floor.

Counter-clockwise five-turn test:

```text
delta left ticks  = -17337
delta right ticks = +17213
effective base    = 231.32 mm
```

Clockwise five-turn test:

```text
start ticks        = [-17335, 17213]
end ticks          = [-505, 235]
delta left ticks   = +16830
delta right ticks  = -16978
effective base     = 226.35 mm
```

Average effective kinematic wheel base:

```text
WHEEL_BASE_MM = 228.84
wheel_base_m  = 0.22884
```

This effective value is intentionally different from the physical 245 mm measurement because tire contact, deformation and in-place slip affect the differential-drive turning geometry.

The ROS bridge publishes raw encoder totals on `/wheel_ticks` as:

```text
[left_encoder_ticks, right_encoder_ticks]
```

## 1. Encoder direction — completed

Current direction test passed:

- positive linear X: both wheels forward
- positive angular Z: left wheel backward, right wheel forward

At angular Z = 0.5 rad/s the right motor did not initially overcome its low-speed dead zone, but at angular Z = 1.0 rad/s both wheels rotated correctly.

## 2. Ticks per wheel revolution — completed

For future verification, rotate one selected wheel exactly 10 full revolutions and compare `/wheel_ticks` before and after.

## 3. Effective wheel diameter — calibrated

Forward 1 m verification after the first diameter correction produced approximately:

```text
x = 1.02243 m
y = -0.04068 m
```

Backward 1 m verification produced approximately:

```text
x = -0.98211 m
y = -0.04958 m
```

The average linear scale is close enough that `60.13 mm` is retained rather than overfitting to manual stopping error.

## 4. Effective wheel base — calibrated, final 360-degree verification next

Current values:

```text
physical wheel spacing = 245 mm
effective wheel base    = 228.84 mm
```

After flashing/building these values, perform one controlled 360-degree turn in each direction to verify the result. Small residual differences can be averaged; do not tune straight-line drift using wheel base.

## 5. Straight-line drift / PID — next

The robot still tends to drift to the right by roughly 4–5 cm over a 1 m straight run.
Do not compensate this with wheel base or an artificial ROS yaw correction.

Next compare left/right target and measured wheel speeds under floor load, then tune PID/feed-forward or apply a small per-wheel calibration if the speed tracking confirms a persistent asymmetry.

Suggested PID tuning order:

1. set `Ki=0`, `Kd=0`;
2. increase `Kp` until both sides track without persistent oscillation;
3. set feedforward so steady-state PWM is mostly provided by `Kff`;
4. add a small `Ki` to remove remaining steady error;
5. use `Kd` only if needed.

Start with speeds 80, 150, 250 mm/s and compare target vs measured values.

## 6. LiDAR transform

Measure from `base_link` (robot center, X forward, Y left, Z up) to the LiDAR optical/rotation center. Enter the transform in `bringup.launch.py` arguments.

A wrong LiDAR transform can look like bad wheel odometry during SLAM, so calibrate it before judging SLAM quality.
