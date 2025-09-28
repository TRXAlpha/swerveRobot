# SwerveRobot Library — Documentation

> A compact, configurable swerve-drive library for FTC teams using GoBilda Pinpoint Odometry.

---

## Table of Contents

1. [Introduction](#introduction)
2. [Features](#features)
3. [Units & Conventions](#units--conventions)
4. [Quick Start](#quick-start)

   * [Install / Add to project](#install--add-to-project)
   * [Create the robot](#create-the-robot)
   * [Minimal TeleOp example](#minimal-teleop-example)
   * [Minimal Autonomous example](#minimal-autonomous-example)
5. [Public API Reference](#public-api-reference)

   * [ModuleConfig](#moduleconfig)
   * [SwerveRobot public methods](#swerverobot-public-methods)
6. [Configuration & Tuning](#configuration--tuning)

   * [Default parameters](#default-parameters)
   * [Tuning guidance](#tuning-guidance)
7. [Calibration Guide](#calibration-guide)

   * [Servo neutral & range](#servo-neutral--range)
   * [Verify odometry units & heading](#verify-odometry-units--heading)
8. [Testing Checklist](#testing-checklist)
9. [Troubleshooting & Common Problems](#troubleshooting--common-problems)
10. [Telemetry & Debugging Tips](#telemetry--debugging-tips)
11. [Advanced Topics](#advanced-topics)
12. [Safety & Best Practices](#safety--best-practices)
13. [FAQ](#faq)
14. [Changelog](#changelog)
15. [License](#license)

---

## Introduction

`SwerveRobot` is a small, documented Java library intended to make it easy for FTC teams to add a swerve-drive control layer to their robotics codebase. It provides:

* Inverse kinematics for 2–4 swerve modules.
* Simple waypoint-based autonomous driving (`driveTo`, `runPath`).
* TeleOp-friendly joystick driving with optional field-oriented control.
* Ramping (slew-rate limiting) and wheel-speed normalization.
* Integration with GoBilda `PinpointOdo` odometry.

This README explains how to integrate, configure, calibrate, and tune the library for your robot.

---

## Features

* Clean `ModuleConfig` API to describe hardware mapping and geometry.
* `driveTo(x_mm, y_mm, heading_deg)` for point-and-face autonomous tasks.
* `driveWithJoystick(forward, strafe, turn, fieldOriented)` for TeleOp.
* Built-in ramping (accel limits) and normalization to avoid motor saturation.
* Telemetry helper `reportPose(Telemetry)` for fast debugging.
* Simple, documented tuning API: set PID gains / speeds / accelerations.

---

## Units & Conventions

The library uses the following units and conventions across the API:

* **Robot geometry (module offsets):** centimeters (cm)
* **Target positions:** millimeters (mm)
* **Angles presented to public API:** degrees (°)
* **Internal math:** meters (m) for distances, radians for rotational math
* **Default angular sign convention:** CW-positive by default (configurable via `setCWPositive(boolean)`).

> **Important:** Make sure your odometry (`PinpointOdo`) reports X/Y/heading in the expected units. The library assumes `odo.getX()` and `odo.getY()` return **millimeters**.

---

## Quick Start

### Install / Add to project

1. Place `SwerveRobot.java` (and any supporting files) under your team code package, e.g. `org.firstinspires.ftc.teamcode.swerve`.
2. Ensure `com.gobilda.pinpoint.PinpointOdo` is available on your robot controller (installed correctly by your team or vendor).
3. Add a short OpMode to import and test the robot.

### Create the robot

Define module configurations and create the `SwerveRobot` instance in your OpMode initialization code:

```java
// Example (4-wheel swerve):
SwerveRobot.ModuleConfig fl = new SwerveRobot.ModuleConfig("flDrive", "flSteer",  10.0,  10.0, 0.50, 0.50);
SwerveRobot.ModuleConfig fr = new SwerveRobot.ModuleConfig("frDrive", "frSteer",  10.0, -10.0, 0.50, 0.50);
SwerveRobot.ModuleConfig bl = new SwerveRobot.ModuleConfig("blDrive", "blSteer", -10.0,  10.0, 0.50, 0.50);
SwerveRobot.ModuleConfig br = new SwerveRobot.ModuleConfig("brDrive", "brSteer", -10.0, -10.0, 0.50, 0.50);

SwerveRobot robot = SwerveRobot.createRobot(hardwareMap, 35.0, 35.0, fl, fr, bl, br);
```

**Note:** adjust `rx_cm`/`ry_cm` to actual module offsets from robot center.

### Minimal TeleOp example

```java
@TeleOp
public class SwerveTeleOp extends LinearOpMode {
    @Override
    public void runOpMode() {
        SwerveRobot.ModuleConfig fl = new SwerveRobot.ModuleConfig("flDrive", "flSteer",  10.0,  10.0, 0.50, 0.50);
        // ... other module configs
        SwerveRobot robot = SwerveRobot.createRobot(hardwareMap, 35.0, 35.0, fl, fr, bl, br);

        waitForStart();
        while (opModeIsActive()) {
            double forward = -gamepad1.left_stick_y; // adjust sign to your preference
            double strafe =  gamepad1.left_stick_x;
            double turn   =  gamepad1.right_stick_x;
            robot.driveWithJoystick(forward, strafe, turn, true);
            robot.reportPose(telemetry);
            telemetry.update();
        }
    }
}
```

### Minimal Autonomous example

```java
@Autonomous
public class SwerveAuto extends LinearOpMode {
    @Override
    public void runOpMode() {
        // Set up modules and robot
        SwerveRobot robot = SwerveRobot.createRobot(hardwareMap, 35.0, 35.0, fl, fr, bl, br);

        waitForStart();

        // Drive to (500 mm, 0 mm) and face 90 degrees (CW-positive)
        robot.addWaypoint(500.0, 0.0, 90.0);
        while (opModeIsActive() && !robot.runPath()) {
            // OpMode loop — runPath calls driveTo internally
            sleep(10);
        }
    }
}
```

---

## Public API Reference

> This section documents the main public classes and methods teams will use.

### `ModuleConfig`

```java
public static class ModuleConfig {
    public ModuleConfig(String driveName, String steerName,
                        double rx_cm, double ry_cm,
                        double neutral, double range) { ... }
}
```

* `driveName` / `steerName`: names from your hardware map.
* `rx_cm`, `ry_cm`: module offsets from robot center in centimeters (positive +x is forward, +y is right depending on your coordinate convention).
* `neutral`: servo position when the wheel is pointing forward (0..1 servo units).
* `range`: fraction of servo travel corresponding to ±90° wheel sweep (usually between 0.3–0.6 depending on your servo travel).

### `SwerveRobot` public methods

```java
public static SwerveRobot createRobot(HardwareMap hw, double robotWidth_cm, double robotLength_cm, ModuleConfig... configs)
```

* Build the swerve robot instance. It looks up a `PinpointOdo` under hardware name `"odo"` and resets it.

```java
public void driveTo(double x_mm, double y_mm, double heading_deg)
```

* Autonomous point-and-face: drives toward `(x_mm,y_mm)` and then rotates to `heading_deg` (degrees). Uses proportional controllers and ramping.

```java
public void driveWithJoystick(double forward, double strafe, double turn, boolean fieldOriented)
```

* TeleOp: `forward`/`strafe`/`turn` in [-1..1]. `fieldOriented` uses the odometry heading so joystick directions are with respect to the field.

```java
public void addWaypoint(double x_mm, double y_mm, double heading_deg)
public void clearWaypoints()
public boolean runPath()
```

* Waypoint queue API. `runPath()` should be called in your OpMode loop; it returns true when the queue is empty.

```java
public boolean isAtTarget()
```

* Returns `true` when the last `driveTo` target is reached (using current distance and heading tolerances).

Tuning/reset methods:

```java
public void setLinearPID(double kP)
public void setAngularPID(double kP)
public void setMaxSpeeds(double lin_mps, double turn_degps)
public void setMaxAcceleration(double linAccel, double turnAccel)
public void setCWPositive(boolean cwPositive)
```

* Use these to adjust behavior per your robot.

Additional helpers:

```java
public void reportPose(Telemetry telemetry)
```

* Adds the current odometry X/Y/heading and `atTarget` to the provided telemetry.

---

## Configuration & Tuning

### Default parameters

* `kP_lin = 2.0` (m/s per meter of error)
* `kP_ang = 5.0` (deg/s per degree of heading error)
* `maxLin_mps = 1.5` m/s
* `maxTurn_degps = 180.0` deg/s
* `maxLinAccel = 2.0` m/s²
* `maxTurnAccel = 360.0` deg/s²
* `position tolerance = 0.05` m (5 cm)
* `heading tolerance = 2.0` degrees

These are intentionally conservative starting points for typical FTC robots.

### Tuning guidance

1. **Verify odometry**: before tuning controllers, ensure `PinpointOdo` reports realistic distances. Move the robot a known distance and compare.
2. **Linear gain (kP_lin)**: start at default. If robot is too slow to correct, increase `kP_lin` in small steps (0.5). If it oscillates/overshoots, lower it.
3. **Angular gain (kP_ang)**: tune until heading corrects quickly but without oscillation.
4. **Max speeds**: set `maxLin_mps` to a value your motors can sustain; adjust `maxTurn_degps` to sensible rotational speed.
5. **Acceleration (ramping)**: reduce `maxLinAccel` or `maxTurnAccel` if wheels slip or servos stutter on direction changes.

---

## Calibration Guide

### Servo neutral & range (step-by-step)

1. **Power the robot safely** and ensure wheels can rotate freely.
2. **Set wheel to forward-facing** (the orientation you consider "0°" — robot front).
3. **Read servo position at forward**:

   * Add a tiny test OpMode that sets a servo to a test position or reads `steer.getPosition()` and prints to telemetry.
4. **Rotate wheel +90° and -90° physically** (or command servo to positions known to move ±90°) and record servo positions for left/right limits.
5. **Compute `neutral`** = servo position at forward.
6. **Compute `range`** = `max(abs(left - neutral), abs(right - neutral))`.

Example: if forward `=0.50`, left `=0.10`, right `=0.90` → `neutral = 0.50`, `range = 0.40`.

> **Note:** many hobby servos do not have exact 180° travel. `range` expresses how much servo travel corresponds to ±90° wheel rotation. If your servo only travels ±60° physically, choose `range` accordingly.

### Verify odometry units & heading

* **Distance units**: Mark a tape on the floor; drive a known mm distance (e.g., 1000 mm). Compare `odo.getX()`/`odo.getY()` values. If they differ by a factor of 1000 (mm vs m), adjust usage accordingly.
* **Heading sign**: Rotate robot a known angle and confirm `odo.getHeading()` sign/direction. Use `setCWPositive()` to align conventions.

---

## Testing Checklist

Before competition, go through this list step-by-step:

1. **Module servo mapping**

   * Command each wheel to `angle = 0` and visually verify it points forward.
   * Command ±45° / ±90° and check travel and direction.
2. **DriveWithJoystick sanity**

   * Slowly move forward/back and observe wheel vectors.
   * Strafe left/right at low speed.
3. **Rotation-only**

   * Command `turn = 0.5` and `forward=strafe=0` — robot should rotate in place smoothly.
4. **Normalization**

   * Test combined rotation+translation at high command to ensure no wheel command > 1.0.
5. **Autonomous waypoint**

   * Add a short waypoint 200–500 mm in front and run `runPath()`.
6. **Heading correction**

   * When close to waypoint, confirm robot rotates to the specified heading within tolerance.
7. **Edge cases**

   * Test very small targets, oscillation, and sudden joystick reversals.

---

## Troubleshooting & Common Problems

**Problem:** Wheel steering is reversed (wheel turns the wrong way)

**Fix:** Swap servo wiring (if possible) or invert servo mapping per module. If you prefer code fix, modify `angleToServo` or add a per-module `steerInvert` flag and multiply angle by -1.

**Problem:** Servos saturate at ends (go beyond 0..1)

**Fix:** Reduce `range` or re-evaluate `neutral`. Confirm `angleToServo` clamps output to `[0,1]`.

**Problem:** Robot rotates the long way around when commanded to face e.g. 179° → -179°

**Fix:** Ensure `wrapNeg180to180` is used when computing heading error (library does this). If odometry provides unwrapped heading (growing beyond ±180), normalise it first.

**Problem:** Odometry units wrong (robot moves 1m but odo reports 1.0)

**Fix:** Confirm `PinpointOdo` returns mm. If it returns meters, multiply/divide appropriately in your code or adjust usage.

**Problem:** Jerky behavior when OpMode pausing / long dt

**Fix:** The library clamps `dt` to a maximum (e.g., 0.1s) — make sure that clamp is enabled. Reset ramping when switching between Auto/TeleOp or when you see stuck previous values.

**Problem:** Wheels try to exceed motor power even after normalization

**Fix:** Verify `maxLin_mps` matches the mapping used to set motor power (i.e., `drive.setPower(wheelSpeed / maxLin_mps)`). If your motors are limited in RPM and gearbox, reduce `maxLin_mps`.

---

## Telemetry & Debugging Tips

During tests, the following telemetry is helpful:

* `odo.getX()`, `odo.getY()` (mm)
* `odo.getHeading()` (deg)
* Each wheel: `angle_deg` and `speed_mps` (or normalized `power`).
* `atTarget` flag
* Current `kP_lin` & `kP_ang` (for runtime tuning)

Consider adding a debug mode to your OpMode that prints these values less frequently (e.g., every 100 ms).

---

## Advanced Topics

* **Per-module inversion:** Add a `steerInvert` and/or `driveInvert` flag per module for wheels mounted backwards.
* **Field-oriented vs. robot-oriented:** Field-oriented requires reliable heading — prefer field-oriented for human drivers when odometry heading is stable.
* **PID/Feedforward:** The library uses P-only controllers for simplicity. For tight control, add D or I terms, and consider a small feedforward proportional to desired speed.
* **Unit tests / simulation:** Implement a small kinematic simulator to verify `computeWheel` math. Use known vectors and compute expected wheel angles.
* **Alternative odometry sources:** Make the odometry dependency an interface so teams can plug in encoders, IMU+encoders, or other odometry libraries.

---

## Safety & Best Practices

* Always test at low speeds before increasing `maxLin_mps`.
* Clamp `dt` used by ramping to avoid jumps after pauses.
* Do not run Auto and TeleOp commands at the same time — reset ramping state when switching modes.
* Physically secure servos and check for binding before high-speed tests.

---

## FAQ

**Q:** What if my servo travel is only ±60°?
**A:** Measure the actual servo travel and set `range` accordingly (range < 0.5). `range` is the servo position delta corresponding to +90° wheel rotation.

**Q:** My PinpointOdo heading is radians — do I need to convert?
**A:** The library converts `odo.getHeading()` from radians to degrees internally where required. Confirm `odo.getHeading()` units and adjust if necessary.

**Q:** Can I use this without PinpointOdo?
**A:** Yes — you can adapt the library to accept a different odometry provider by creating a small interface and modifying `createRobot` to accept it.

---

## Changelog

* **v1.0.0** — Initial polished library: inverse kinematics, ramping, waypointing, field-oriented TeleOp, telemetry helper.

Contributions welcome — please follow the contribution guidelines and include a reproducible test.

---

## License

**MIT License** Example:

```
MIT License

Copyright (c) 2025 HexRobotics#19143

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction... (full MIT text)
```

---

## Contributing & Contact

If you or other teams want to contribute improvements (per-module inversion, extra odometry adapters, telemetry helpers), please open a pull request or contact the maintainer. Include a brief description and test results.

---

*End of documentation.*
