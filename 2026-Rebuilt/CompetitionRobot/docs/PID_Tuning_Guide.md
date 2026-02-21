# PID Tuning Guide

Step-by-step instructions for tuning PID (and feedforward) on each mechanism, in recommended order. Tuning improves accuracy, response, and consistency so auto and teleop behave predictably.

---

## Prerequisites

- **Safety**: Ensure the robot is secured (blocks, stand) and no one is in the path of moving parts. E-stop ready.
- **Data logging**: For SysId-based tuning, enable WPILib data logging (Driver Station or `SignalLogger`) and use the [FRC SysId tool](https://docs.wpilib.org/en/stable/docs/software/pathplanning/system-identification/introduction.html) to analyze logs.
- **One mechanism at a time**: Tune one subsystem fully before moving to the next.

---

## Tuning Mode and Mechanism Mode

Controller profiles are defined in `SwerveDrivetrainBindings`. **Drive controller** has three profiles; **mechanism controller** has three modes (MECHANISM, SYSID, INDIVIDUAL), set automatically from the drive profile.

- **Drive profiles** (cycle with **Back + Start** on drive controller):
  - **NORMAL** — Normal driving; mechanism controller uses **MECHANISM** (molecule bindings: intake combo, shoot combo, deploy/stow).
  - **SYSID** — Drivetrain SysId only (drive brake; A/B/Right Bumper select translation/steer/rotation; Back+Y/X, Start+Y/X run dynamic/quasistatic). Mechanism controller uses **SYSID** (SysId bindings for PID tuning).
  - **TUNING** — Path tuning (drive brake; **A** = run tuning path). Mechanism controller uses **INDIVIDUAL** (one button per mechanism for manual testing).

- **Mechanism mode** (derived from drive profile):
  - **MECHANISM** — Competition molecule bindings: D-pad down = deploy intake, D-pad up = stow intake; left trigger = intake+floor+feeder; left bumper = reverse those three; right trigger = shoot (shooter then delayed feeder then floor); right bumper = reverse shooter/feeder/floor; Back = stop all.
  - **SYSID** — Mechanism SysId bindings (PID tuning): A/B/X/Y = shooter quasi/dynamic; Right bumper + A/B/X/Y = feeder; Left bumper + A/B/X/Y = floor; POV Up/Down + A/B/X/Y = deploy/intake motors.
  - **INDIVIDUAL** — One button per mechanism: A/X/B = feeder fwd/rev/off; Y/right trigger = floor fwd/rev; left trigger = intake; D-pad down = deploy, D-pad up = stow; right/left bumper = shooter fwd/rev; Back = shooter off.

**SysId mechanism bindings** (mechanism controller when drive profile is **SYSID**):

| Mechanism   | Buttons        | A = quasi fwd, B = quasi rev, X = dynamic fwd, Y = dynamic rev |
|------------|----------------|----------------------------------------------------------------|
| Shooter    | A / B / X / Y  | (no modifier)                                                   |
| Feeder     | Right bumper + A/B/X/Y |                                                         |
| Floor      | Left bumper + A/B/X/Y |                                                          |
| Deploy     | POV Up + A/B/X/Y | Deploy motor SysId                                            |
| Intake     | POV Down + A/B/X/Y | Intake motor SysId                                           |

**Tuning path (swerve path-following):** In **TUNING** profile, press **A on the drive controller** to run the built-in tuning path. Path name: `zzTuning-1` (file `src/main/deploy/pathplanner/paths/zzTuning-1.path`). Constant: `AutoConstants.TUNING_PATH_NAME`. Linked paths `zzTuning-2`, `zzTuning-3`, `zzTuning-4` exist for longer runs; you can add a binding or auto sequence for them if needed.

On **disable**, the robot resets to NORMAL profile (and MECHANISM mode) so the next enable starts in normal operation.

**Locking to a single profile:** In `Constants/ControllerBindingConstants.java` set `ENABLE_PROFILE_SWITCHING = false` and `DEFAULT_DRIVE_PROFILE` to the desired profile (e.g. `InputProfile.TUNING` for tuning only). Then Back+Start does nothing and the robot stays on that profile for the session. Set `ENABLE_PROFILE_SWITCHING = true` to allow cycling again.

**Encapsulation:** Drive bindings (default command, SysId, profile cycle) live in `SwerveDrivetrainBindings`. Mechanism bindings and the drive tuning-path binding are built by `Controllers/ControllerBindingFactory`, which applies mappings by mode (MECHANISM, SYSID, INDIVIDUAL).

**Dashboard:** On the Shuffleboard **Driver** tab, **Bindings Profile** is a dropdown (default: **NORMAL (Competition)**). Select **SYSID** or **TUNING** to switch without using Back+Start. **Current Profile** shows the active profile (updates when you use the dropdown or Back+Start).

---

## 1. Shooter (velocity control)

**Why first:** Shooter speed directly affects scoring in auto and teleop. Good velocity control means consistent shots.

**Where the gains live:** `src/main/java/frc/robot/Constants/ShooterConstants.java`  
- `VELOCITY_KV` — feedforward, volts per (rps)  
- `VELOCITY_KS` — static friction feedforward (volts)  
- `VELOCITY_KP`, `VELOCITY_KI`, `VELOCITY_KD` — velocity error PID  

The shooter uses **velocity closed-loop** (Slot0) on the leader TalonFX; followers mirror the leader.

### Step 1.1 — Run SysId (recommended)

1. **Trigger SysId:** Put the drive controller in **SYSID** profile (Back + Start until you’re in SYSID, or select SYSID on the Driver tab). On the **mechanism controller**, use **A** (quasistatic forward), **B** (quasistatic reverse), **X** (dynamic forward), **Y** (dynamic reverse) to run:
   - Quasistatic forward
   - Quasistatic reverse
   - Dynamic forward
   - Dynamic reverse  

2. **Enable logging:** Start WPILib log (or use `SignalLogger` if your project writes SysId-compatible logs). Run each test to completion with the shooter unloaded (no balls).

3. **Analyze in SysId:** Export the log and open it in the FRC SysId tool. Fit the data and note the suggested **kV** and **kS** (and kA if you use acceleration feedforward).

4. **Apply feedforward:** In `ShooterConstants.java`, set:
   - `VELOCITY_KV` = SysId-reported kV  
   - `VELOCITY_KS` = SysId-reported kS  
   Leave `VELOCITY_KI` and `VELOCITY_KD` at 0 for now.

### Step 1.2 — Tune kP (velocity stiffness)

1. Set `ShooterConstants.VELOCITY_KP` to a small value (e.g. `0.05`).  
2. Spin the shooter to your normal shot speed (e.g. `ShooterSpeed.FORWARD`).  
3. In telemetry or Shuffleboard, watch velocity vs setpoint.  
4. **Increase kP** in small steps until:
   - Velocity tracks the setpoint well with minimal steady-state error.  
   - You do **not** get sustained oscillation or loud “hunting.”  
5. If you see oscillation, reduce kP slightly. Optional: add a small `VELOCITY_KD` to dampen.

### Step 1.3 — Validate

- Command the shooter to `FORWARD` (and optionally `REVERSE`) and confirm it reaches and holds speed quickly.  
- If you have a “shoot” command that waits for “at speed,” confirm it becomes reliable.

---

## 2. Drivetrain — path-following PID

**Why second:** After the shooter is consistent, path-following tuning makes auto paths accurate (translation and heading).

**Where the gains live:** `src/main/java/frc/robot/SwerveDrivetrain/DrivetrainConstants.java`  
- `kPathTranslationPID` — P, I, D for X/Y position error (PathPlanner / `PPHolonomicDriveController`)  
- `kPathRotationPID` — P, I, D for rotation/heading error  

These are **WPILib PIDConstants** used by PathPlanner’s holonomic controller, not the CTRE module gains.

### Step 2.1 — Translation (X/Y)

1. Start with current values (e.g. `kPathTranslationPID = new PIDConstants(7, 0, 0)`).  
2. Put the drive controller in **Tuning Mode** (Back + Start until TUNING), then press **A** to run the built-in tuning path (`zzTuning-1`), or run any path that has straight segments and curves.  
3. **If the robot lags behind the path:** Increase P (e.g. try 8, 10).  
4. **If the robot overshoots or oscillates:** Decrease P or add a small D term.  
5. **If there is steady-state error** (e.g. stops short): Add a small I (e.g. 0.1); keep I small to avoid integral windup.

### Step 2.2 — Rotation (heading)

1. Use `kPathRotationPID` (e.g. currently `(7, 0, 0)`).  
2. Run paths that require heading changes (e.g. turn in place or curved paths).  
3. **If heading lags:** Increase P. **If it overshoots or oscillates:** Decrease P or add a small D.  
4. Add small I only if you see persistent heading error.

### Step 2.3 — Drivetrain module gains (optional, advanced)

**Where:** `SwerveDrivetrainA.java` — `steerGains` (steer motor) and `driveGains` (drive motor).  

Your project has **SysId routines** for the drivetrain (translation, steer, rotation) in `CommandSwerveDrivetrain` and bindings in `SwerveDrivetrainBindings` (e.g. SYSID profile: A = translation, B = steer, Right Bumper = rotation; Back+Start + X/Y for quasistatic/dynamic).  

1. Run the appropriate SysId tests (translation, steer, or rotation) with logging.  
2. Use the SysId tool to get suggested gains.  
3. Apply suggested gains to `steerGains` / `driveGains` in `SwerveDrivetrainA` as appropriate (the tool may output values for velocity/position loops that map to Slot0 kP, kI, kD, kV, kS, kA).  
4. Re-test path following; path PID (Step 2.1–2.2) and module gains interact, so iterate if needed.

---

## 3. Climber (position / MotionMagic)

**Why third:** Climber is used after shooter and path following in typical auto; tuning it makes climb reliable and smooth.

**Where the gains live:** `src/main/java/frc/robot/Subsystems/Climber.java` — constructor applies **Slot0** and **MotionMagic** configs directly (not from `ClimberConstants`).  

- **Slot0:** `kG`, `kS`, `kV`, `kA`, `kP`, `kI`, `kD`  
- **MotionMagic:** `MotionMagicCruiseVelocity`, `MotionMagicAcceleration`, `MotionMagicJerk`  

`ClimberConstants` contains alternate tuning values (e.g. `kP`, `kS`, `kV`) but the climber code currently uses the constructor literals; you can refactor to use constants later.

### Step 3.1 — Safety and direction

1. Ensure the climber cannot pinch or damage anything at full travel.  
2. Confirm positive/negative direction matches your convention (e.g. “up” = negative position per `ClimberPosition`).

### Step 3.2 — Tune Slot0 (position + feedforward)

1. **kP:** Start with the current value (e.g. 4.8). Command a position and watch error.  
   - Too low: slow to reach setpoint.  
   - Too high: oscillation or overshoot.  
   Increase kP until response is quick but stable, then back off slightly if needed.  
2. **kD:** Add or increase kD (e.g. 0.1) to reduce overshoot and settling time.  
3. **kS, kV, kG, kA:** These improve motion quality.  
   - **kS** — voltage to overcome static friction; increase if the mechanism “sticks” at start.  
   - **kV** — approximate volts per unit velocity; helps track motion during move.  
   - **kG** — gravity feedforward if the mechanism is vertical or angled (your code uses -0.2).  
   - **kA** — optional acceleration feedforward.  
   Tune after kP/kD are stable; SysId (if you add a climber SysId routine) can suggest kS/kV/kA.

### Step 3.3 — Tune MotionMagic

1. **MotionMagicCruiseVelocity** — max velocity (in sensor units/sec). Increase for faster climbs; decrease if motion is jerky or motors are overloaded.  
2. **MotionMagicAcceleration** — acceleration limit. Higher = faster ramp-up; too high can cause jerk or slip.  
3. **MotionMagicJerk** — limits rate of change of acceleration. Increase for smoother starts/stops; decrease if the mechanism is too slow to get moving.

### Step 3.4 — Validate

- Run to DOWN, ACQUIRE, and CLIMB (or equivalent positions). Confirm smooth motion, no oscillation, and consistent end positions.

---

## 4. Center-to (vision / pose alignment)

**Why last:** These P gains affect driver-assist or auto alignment; tune after shooter, path, and climber so the rest of the robot is predictable.

**Where the gains live:** Commands that extend `BaseCenterToCommand` pass a `Config` object. The config holds:

- **Pose method:** `poseXP`, `poseYP`, `poseOmegaP` (defaults 1.0, 1.0, 0.03) and tolerances `poseXError`, `poseYError`, `poseAngleError`.  
- **Camera method:** `cameraXP`, `cameraYP`, `cameraOmegaP` (defaults 0.03) and tolerances `cameraAreaError`, `cameraXOffsetError`, `cameraAngleError`.  

Subclasses (e.g. `CenterToStationCommand`, `CenterToReefCommand`) build the `Config` and pass it to the base; tune the values in those configs or in a shared config builder.

### Step 4.1 — Pose-based centering

1. Use a command that uses `CenterMethod.POSE` (e.g. center to reef/tag).  
2. **poseXP / poseYP:** Control how aggressively the robot corrects X/Y error.  
   - Increase if the robot is slow to reach the target pose.  
   - Decrease if it overshoots or oscillates.  
3. **poseOmegaP:** Same idea for heading.  
   - Increase for faster rotation to target angle; decrease if rotation overshoots.  
4. **Tolerances:** `poseXError`, `poseYError`, `poseAngleError` define “at goal.”  
   - Tighten for precision; loosen slightly to avoid jitter near the target.

### Step 4.2 — Camera-based centering

1. Use a command that uses `CenterMethod.CAMERA`.  
2. **cameraXP, cameraYP, cameraOmegaP:** Tune like pose: higher P = faster correction, lower P = less overshoot.  
3. **Tolerances:** `cameraAreaError`, `cameraXOffsetError`, `cameraAngleError` — set so the command finishes when alignment is “good enough” without hunting.

### Step 4.3 — General

- Test with driver override (driver input typically cancels or pauses the command).  
- If the robot “hunts” around the setpoint, reduce P or widen tolerances so the command ends instead of oscillating.

---

## Summary table

| Order | Mechanism           | Main gains / constants file(s)                         | Main tuning method              |
|-------|---------------------|--------------------------------------------------------|---------------------------------|
| 1     | Shooter             | `ShooterConstants.java` (kV, kS, kP, kI, kD)           | SysId + kP for velocity         |
| 2     | Path-following      | `DrivetrainConstants.java` (kPathTranslationPID, kPathRotationPID) | Test paths, adjust P/I/D        |
| 2b    | Swerve modules      | `SwerveDrivetrainA.java` (steerGains, driveGains)     | SysId (translation/steer/rotation) |
| 3     | Climber             | `Climber.java` (Slot0 + MotionMagic in constructor)   | kP/kD then feedforward, then MotionMagic |
| 4     | Center-to (vision)  | `BaseCenterToCommand.Config` (poseXP/YP/omegaP, camera*, tolerances) | Test align, adjust P and tolerances |

---

## Tips

- **Log and compare:** After each change, note the values and behavior (e.g. “kP=0.1, slight overshoot”) so you can revert or iterate.  
- **Small steps:** Change one gain at a time when possible.  
- **I and D:** Use I sparingly (small values) and only when you have steady-state error; use D to dampen overshoot.  
- **Feedforward first:** For velocity/position systems, get kV/kS (and kA if used) from SysId or manual tuning before pushing kP high.

If a mechanism uses a different control mode (e.g. pure voltage or percent output) in some code paths, tune the closed-loop gains in the mode you use in competition (e.g. velocity for shooter, position/MotionMagic for climber).
