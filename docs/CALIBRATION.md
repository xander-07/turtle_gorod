# Calibration

Do this before tuning SLAM or Nav2.

## Current calibrated values (2026-09-09)

Measured encoder scale from 10 full wheel revolutions:

```text
LEFT_ENCODER_TICKS_PER_WHEEL_REV  = 897.4
RIGHT_ENCODER_TICKS_PER_WHEEL_REV = 898.8
```

The original common effective rolling diameter was calibrated to 60.13 mm. After the straight-line drift tests, the same average linear scale is retained but split into separate left/right effective diameters:

```text
LEFT_WHEEL_DIAMETER_MM  = 60.39
RIGHT_WHEEL_DIAMETER_MM = 59.87
```

These are odometry/control calibration values, not ruler measurements of the tire.

The physical center-to-center distance between the left and right driven wheels is **245 mm**. For differential-drive odometry, the effective kinematic wheel base was calibrated with two independent five-turn tests on the floor.

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

## 3. Effective linear scale — calibrated

Forward 1 m verification after the first common-diameter correction produced approximately:

```text
x = 1.02243 m
y = -0.04068 m
```

Backward 1 m verification produced approximately:

```text
x = -0.98211 m
y = -0.04958 m
```

The average linear scale is close enough that the mean effective diameter remains approximately 60.13 mm rather than being retuned from manual stopping error.

## 4. Effective wheel base — calibrated

Current values:

```text
physical wheel spacing = 245 mm
effective wheel base    = 228.84 mm
```

A one-turn counter-clockwise verification after applying 228.84 mm ended with odometry about +12.4 degrees past zero. The robot was physically observed to have been manually over-rotated by roughly 5–10 degrees, so the five-turn calibration is retained.

Do not tune straight-line drift using wheel base.

## 5. Straight-line tracking — per-wheel rolling scale calibrated

A 2.00 m run with `linear.x=0.10`, `angular.z=0.0` produced:

```text
delta left ticks  = 9358
delta right ticks = 9322
odom x             = 1.96331 m
odom y             = -0.06915 m
physical drift     = about 13 cm right
```

A second 2.00 m run used a temporary diagnostic correction only:

```text
linear.x  = 0.10 m/s
angular.z = +0.0065 rad/s
```

The second run produced approximately:

```text
delta left ticks  = 9276
delta right ticks = 9379
odom x             = 1.96041 m
odom y             = +0.06105 m
physical drift     = about 0.5–1.0 cm left
```

The temporary angular command demonstrated the wheel asymmetry but is **not** kept as a hidden steering bias. Interpolating the two physical drift results to zero lateral error gives a required right/left encoder-rate ratio of about 1.0103 during physically straight travel.

To encode that relationship correctly in both odometry and wheel-speed feedback while preserving the existing mean linear scale, the firmware now uses:

```text
LEFT_WHEEL_DIAMETER_MM  = 60.39
RIGHT_WHEEL_DIAMETER_MM = 59.87
```

Because the PID measures wheel speed using these per-side scales, an ordinary equal wheel-speed command should naturally produce the small right-side encoder-rate increase needed for physically straight motion. No artificial `angular.z` offset is added in ROS.

Next verification: flash the firmware and repeat a 2.00 m run with exactly `linear.x=0.10`, `angular.z=0.0`. Record physical lateral drift, `/odom`, and `/wheel_ticks` before/after.

## 6. LiDAR transform

Measure from `base_link` (robot center, X forward, Y left, Z up) to the LiDAR optical/rotation center. Enter the transform in `bringup.launch.py` arguments.

A wrong LiDAR transform can look like bad wheel odometry during SLAM, so calibrate it before judging SLAM quality.
