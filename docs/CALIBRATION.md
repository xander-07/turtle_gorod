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

## 5. Straight-line tracking — calibrated at 0.10 m/s

After the per-wheel rolling-scale calibration, a 2.00 m run with an ordinary straight command:

```text
linear.x  = 0.10 m/s
angular.z = 0.0
```

produced:

```text
start ticks         = [0, -1]
end ticks           = [9269, 9312]
odom x              = 1.95232 m
odom y              = -0.08472 m
physical drift      = about 8.5 cm right
```

This was useful because odometry now saw essentially the same lateral error as the real robot. Therefore the per-wheel odometry scale is retained and the remaining error is treated as a drive-command asymmetry, not an odometry error.

A diagnostic 2.00 m run with:

```text
linear.x  = 0.10 m/s
angular.z = +0.0044 rad/s
```

produced:

```text
start ticks         = [9269, 9313]
end ticks           = [18730, 18926]
odom x              = 2.00545 m
odom y              = +0.01311 m
physical drift      = about 0.5–1.0 cm left
```

So `+0.0044 rad/s` is slightly too much correction. Linear interpolation between the 8.5 cm right drift at zero correction and roughly 0.5–1.0 cm left drift at `+0.0044` gives a required equivalent correction of about `+0.0040 rad/s` at `0.10 m/s`.

Instead of keeping a hidden ROS angular offset, the lower-level firmware applies symmetric per-wheel command calibration:

```text
LEFT_DRIVE_COMMAND_SCALE  = 0.9954
RIGHT_DRIVE_COMMAND_SCALE = 1.0046
```

The mean command scale remains 1.0. For a nominal 100 mm/s straight command this corresponds to approximately:

```text
left target  = 99.54 mm/s
right target = 100.46 mm/s
```

The trim is applied to `SET_WHEELS_SPEED`, so the external API remains normal: `linear.x > 0` with `angular.z = 0` is still the correct straight-drive command. No artificial `angular.z` bias is added in ROS.

Next verification after flashing v2.6: repeat a 2.00 m run with exactly `linear.x=0.10`, `angular.z=0.0`. If physical drift is within about 1–2 cm over 2 m, keep these values. Then verify one 360-degree turn because the small per-wheel command trim also acts during turning. Later repeat a shorter straight test at 0.20 m/s to check whether the trim is sufficiently speed-independent.

## 6. LiDAR transform

Measure from `base_link` (robot center, X forward, Y left, Z up) to the LiDAR optical/rotation center. Enter the transform in `bringup.launch.py` arguments.

A wrong LiDAR transform can look like bad wheel odometry during SLAM, so calibrate it before judging SLAM quality.