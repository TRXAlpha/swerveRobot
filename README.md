# SwerveRobot Developer Guide

A complete FTC-ready swerve library powered by GoBilda Pinpoint Odometry.

---

## ⚙️ Setup

### 1. Robot Configuration

* Add drive motors + steer servos to your config.
* Add Pinpoint Odo as device named **"odo"**.

### 2. Create Robot Instance

```java
bot = SwerveRobot.createRobot(hardwareMap, 40, 40,
    new SwerveRobot.ModuleConfig("driveS","steerS", 20, 15, 0.5, 0.3),
    new SwerveRobot.ModuleConfig("driveD","steerD", -20, 15, 0.5, 0.3)
);
```

* Robot size: cm.
* Module position: cm from robot center (fwd, right).
* `neutral`: servo position when wheel faces forward.
* `range`: fraction of servo travel that equals ±90°.

---

## 🚀 Public API

### Autonomous

* `driveTo(x_mm, y_mm, heading_deg)` → drive to coordinates in mm + heading in deg.
* `isAtTarget()` → returns true if within 5 cm + 2°.
* `addWaypoint(x_mm, y_mm, heading_deg)` → queue movement.
* `runPath()` → call in loop; returns true when path finished.
* `clearWaypoints()` → reset path.

### TeleOp

* `driveWithJoystick(forward, strafe, turn, fieldOriented)`

  * Inputs: -1..+1 from gamepad.
  * `fieldOriented` → true = controls relative to field.

### Telemetry

* `reportPose(telemetry)` → prints X, Y, Heading, Target status.

### Tuning

* `setLinearPID(kP)` → drive aggressiveness.
* `setAngularPID(kP)` → turn aggressiveness.
* `setMaxSpeeds(lin_mps, turn_degps)` → clamp robot speed.
* `setMaxAcceleration(linAccel, turnAccel)` → ramping rate.
* `setCWPositive(flag)` → choose CW-positive or CCW-positive rotation.

---

## 🛠️ Internal Details

* **Ramping**: Limits acceleration. `dt` is clamped to 0.1s to prevent jumps.
* **Normalization**: If any wheel > max speed, all scale proportionally.
* **Servo mapping**: `neutral ± range` = ±90°. Teams must calibrate.
* **Odo units**: Pinpoint returns X/Y in mm, Heading in radians. Converted internally to m + deg.
* **Rotation**: Default `cwPositive = true` (CW = positive). Teams can flip if needed.

---

## 📘 Example Autonomous

```java
@Override
public void init() {
    bot = SwerveRobot.createRobot(hardwareMap, 40, 40,
        new SwerveRobot.ModuleConfig("driveS","steerS", 20, 15, 0.5, 0.3),
        new SwerveRobot.ModuleConfig("driveD","steerD", -20, 15, 0.5, 0.3)
    );
    bot.addWaypoint(1000, 0, 0);
    bot.addWaypoint(1000, 1000, 90);
}

@Override
public void loop() {
    if (bot.runPath()) telemetry.addLine("Path finished!");
    bot.reportPose(telemetry);
    telemetry.update();
}
```

---

## 📘 Example TeleOp

```java
@Override
public void loop() {
    double fwd = -gamepad1.left_stick_y;
    double str = gamepad1.left_stick_x;
    double turn = gamepad1.right_stick_x;
    bot.driveWithJoystick(fwd, str, turn, true);

    bot.reportPose(telemetry);
    telemetry.update();
}
```

---

## 📗 JavaDoc Basics

* Add `/** ... */` comments above **classes** and **methods**.
* Use `@param` and `@return` for parameters/outputs.
* Example:

```java
/**
 * Drive to target coordinates.
 * @param x_mm target X in millimeters
 * @param y_mm target Y in millimeters
 * @param heading_deg target heading in degrees
 */
public void driveTo(double x_mm, double y_mm, double heading_deg) { ... }
```

### Generate Docs

In Android Studio or command line:

```bash
javadoc -d docs src/main/java/org/firstinspires/ftc/teamcode/swerve/SwerveRobot.java
```

* Output: HTML docs inside `docs/`.
* Open `index.html` in browser → full documentation.

---

## ✅ Best Practices

* Calibrate servos carefully.
* Tune `kP_lin` and `kP_ang` on field.
* Always call `reportPose()` during testing.
* Flip `cwPositive` if rotations are reversed.
* Clamp speeds conservatively to avoid wheel slip.
