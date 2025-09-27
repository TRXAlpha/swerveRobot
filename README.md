# SwerveRobot Library for FTC

A simple all-in-one swerve drive library for FTC teams.

* Supports **2–4 swerve modules**.
* Works with **Pinpoint / GoBilda odometry** (plug your odo object in).
* Inputs in **millimeters + degrees** (targets), robot size + module geometry in **centimeters**.
* Outputs directly to **drive motors + steer servos**.

---

## 🚀 Creating a Robot

```java
SwerveRobot bot = SwerveRobot.createRobot(
    hardwareMap,
    40, 40,   // robot size in cm
    new String[]{"driveS","steerS"}, 20, 15, 0.5, 0.3,   // Module S (20cm fwd, 15cm right)
    new String[]{"driveD","steerD"}, -20, 15, 0.5, 0.3   // Module D
    // up to 4 modules total
);
```

Each module requires:

* `{driveMotorName, steerServoName}` (strings from your config)
* `rx_cm, ry_cm` position from robot center (cm)
* `neutral` servo pos (e.g. 0.5 for forward)
* `range` servo offset (e.g. 0.3 for ±90° steering)

---

## 🛠️ API Methods

### Autonomous

* `driveTo(x_mm, y_mm, heading_deg)`

  * Drives robot to `(x,y)` in **mm**, then rotates to `heading_deg`.
  * Uses odo internally to know current pose.

* `isAtTarget()`

  * Returns `true` if the robot is within 5 cm and 2° of target.

* **Waypoint system**:

  * `addWaypoint(x_mm, y_mm, heading_deg)` – add step to queue
  * `clearWaypoints()` – reset path
  * `runPath()` – call each loop; drives through waypoints.

    * Returns `true` when finished.

Example:

```java
bot.addWaypoint(1000, 0, 0);
bot.addWaypoint(1000, 2000, 90);
bot.addWaypoint(0, 2000, 180);

while (opModeIsActive()) {
    if (bot.runPath()) break; // done
}
```

### TeleOp

* `driveWithJoystick(forward, strafe, turn)`

  * Inputs are `-1..+1` (from gamepad joysticks).
  * Forward = left stick y, Strafe = left stick x, Turn = right stick x.
  * Internally maps to max linear speed (`1.5 m/s`) and max turn (`180°/s`).

Example:

```java
double fwd = -gamepad1.left_stick_y;
double str = gamepad1.left_stick_x;
double turn = gamepad1.right_stick_x;
bot.driveWithJoystick(fwd, str, turn);
```

---

## ⚙️ Tuning Parameters (inside SwerveRobot)

* `kP_lin` – how aggressively robot drives toward target (m/s per m error).
* `kP_ang` – how aggressively robot turns to heading (deg/s per deg error).
* `maxLin_mps` – clamp on max linear speed (default 1.5 m/s).
* `maxTurn_degps` – clamp on max turn speed (default 180°/s).

---

## ✅ Features

* Fully autonomous + teleop ready.
* Easy to configure: robot size in cm, targets in mm.
* Works with **2, 3, or 4 modules**.
* Servo steering calibrated with `neutral` + `range`.
* Simple waypoint system for Auto.

---

## 📌 Example Autonomous

```java
@Override
public void init() {
    bot = SwerveRobot.createRobot(
        hardwareMap, 40, 40,
        new String[]{"driveS","steerS"}, 20, 15, 0.5, 0.3,
        new String[]{"driveD","steerD"}, -20, 15, 0.5, 0.3
    );
    bot.addWaypoint(1000, 0, 0);
    bot.addWaypoint(1000, 1000, 90);
}

@Override
public void loop() {
    if (bot.runPath()) {
        telemetry.addLine("Path finished!");
    }
}
```

---

## 📌 Example TeleOp

```java
@Override
public void loop() {
    double fwd = -gamepad1.left_stick_y;
    double str = gamepad1.left_stick_x;
    double turn = gamepad1.right_stick_x;
    bot.driveWithJoystick(fwd, str, turn);
}
```
