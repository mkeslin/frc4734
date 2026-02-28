# Autonomous Routines

This document describes the autonomous routines available on the robot: how they are selected and run, what each routine does step-by-step, and the constants that affect behavior. For testing individual commands and sequences, see [Autonomous Testing Framework](autonomous_testing_framework.md).

---

## How Auto Is Run

- **Selection**: The drive team chooses a routine from the **"Auto Mode (manager)"** chooser on SmartDashboard. `AutoManager` holds a `SendableChooser<AutoRoutine>` and publishes it there.
- **Start**: When the match enters autonomous, `Robot.autonomousInit()` calls `AutoManager.runSelectedRoutine()`. That:
  - Resets odometry to the routine’s **initial pose**
  - Schedules the routine’s command (with a one-time timer stop when the command finishes)
- **End**: In `Robot.autonomousExit()`, `AutoManager.endRoutine()` cancels the running command.

Landmarks and paths are **alliance-aware**: `Landmarks` uses `AllianceUtils` so blue/red get the correct poses (e.g. red is a 180° flip). Path names use **L** (left / POS_1), **C** (center / POS_2), **R** (right / POS_3)—e.g. `L_StartToShot`, `C_StartToCenter`, `R_CenterToShot`. See `MoleculeTests.getPathNameForPose()` and PathPlanner path files under `src/main/deploy/pathplanner/`.

---

## Available Routines

| Routine               | Start position | Description                                      |
|-----------------------|----------------|--------------------------------------------------|
| ClimberAuto (Left)    | POS_1 (left)   | Shoot preload, drive to tower, climb L1, hold    |
| ClimberAuto (Middle)  | POS_2 (center) | Same, tower align at bar center                  |
| ClimberAuto (Right)   | POS_3 (right)  | Same, tower align on right side                  |
| ShooterAuto           | POS_1 (left)   | Shoot preload → center → intake 1 → return → shoot |

---

## ClimberAuto (Left / Middle / Right)

**Goal**: Shoot the preload, then drive to the tower and climb (L1), then hold until auto ends.

**Steps (in order):**

1. **Seed odometry** – Set pose to the chosen start (Left/Middle/Right from `Landmarks.OurStart1()`, `OurStart2()`, or `OurStart3()`).
2. **Tag snap (if good)** – If vision sees enough AprilTags with low ambiguity and within 5 m, snap pose once. Uses `AutoConstants` (e.g. max ambiguity 0.2, min 2 targets).
3. **Drive to shot pose (with shooter spin-up)** – In parallel:
   - **CmdDriveToPose** to `Landmarks.OurShotPosition()` (in front of hub, facing hub). XY tolerance 0.1 m, rotation 5°, pose timeout 5 s.
   - **CmdShooterSpinUp** to 3000 RPM.
4. **Acquire hub aim** – Vision-based heading to hub; fallback heading 180° if no target.
5. **Wait shooter at speed** – Block until shooter is at 3000 RPM ± 100.
6. **Shoot preload** – Feed for 1.0 s (`CmdShootForTime`).
7. **Drive to tower align pose** – `CmdDriveToPose` to `Landmarks.OurTowerAlign(startId)`:
   - **Left (POS_1):** TowerAlignLeft (high Y, climb side nearest left start).
   - **Center (POS_2):** TowerAlignCenter (bar center).
   - **Right (POS_3):** TowerAlignRight (low Y).
   Pose is “back into circle end of tower bar” (e.g. 270° in blue).
8. **Tag snap again** – Same vision snap for better alignment.
9. **Drive to tower align again** – Same pose as step 7 for fine alignment.
10. **Climb L1** – `ClimbWhileHeldCommand.ascentToCompletion(climber)` (one full extend L1 → retract cycle), with **15 s** timeout.
11. **Hold climb until end** – `CmdHoldClimbUntilEnd` keeps climb state until autonomous ends.

ClimberAuto uses **no PathPlanner paths**; all motion is **drive-to-pose** via `CmdDriveToPose`. Build logic: `AutoRoutines.buildClimberAuto()`; registration: `AutoConfigurator.registerFullAutos()`.

---

## ShooterAuto

**Goal**: Shoot preload, go to center and intake one note, return to shot position and shoot again.

**Steps (in order):**

1. **Seed odometry** – Start at POS_1 (left); pose from `Landmarks.OurStart1()`.
2. **Tag snap (if good)** – Same as ClimberAuto.
3. **Path to shot (with shooter spin-up)** – In parallel:
   - **CmdFollowPath** `L_StartToShot` (PathPlanner), 10 s timeout.
   - **CmdShooterSpinUp** to 3000 RPM.
4. **Acquire hub aim** – Same as ClimberAuto.
5. **Wait shooter at speed** – 3000 RPM ± 100.
6. **Shoot** – Feed for 1.0 s.
7. **Path to center (with intake on)** – In parallel:
   - **CmdFollowPath** `L_StartToCenter`, 10 s timeout.
   - **CmdIntakeOn** (intake deployed and running).
8. **Intake until count** – `CmdIntakeUntilCount` until `PositionTracker` reports **1** extra note (or timeout).
9. **Path back to shot (with shooter spin-up)** – In parallel:
   - **CmdFollowPath** `L_CenterToShot`, 10 s timeout.
   - **CmdShooterSpinUp** to 3000 RPM.
10. **Acquire hub aim** – Again.
11. **Wait shooter at speed** – Again.
12. **Shoot** – Feed for 1.0 s again.

Summary: **preload shot → center → intake 1 note → return → second shot.** All paths are for the **left** start (L_*). Build logic: `AutoRoutines.buildShooterAuto()`; registration: `AutoConfigurator.registerFullAutos()`.

---

## Key Constants

Defined in `AutoConstants.java`; used by the commands above.

| Constant                         | Value   | Use                          |
|----------------------------------|---------|------------------------------|
| `DEFAULT_PATH_TIMEOUT`           | 10.0 s  | Path following               |
| `DEFAULT_POSE_TIMEOUT`            | 5.0 s   | Drive-to-pose                |
| `DEFAULT_HEADING_TIMEOUT`         | 3.0 s   | Snap to heading              |
| `DEFAULT_CLIMB_TIMEOUT`           | 15.0 s  | L1 climb cycle               |
| `DEFAULT_XY_TOLERANCE`           | 0.1 m   | Position tolerance           |
| `DEFAULT_ROTATION_TOLERANCE`      | 5°      | Heading tolerance            |
| `DEFAULT_MAX_AMBIGUITY`           | 0.2     | Tag snap (vision)            |
| `DEFAULT_MAX_TAG_DISTANCE`        | 5.0 m   | Tag snap (vision)            |
| `DEFAULT_MIN_TARGETS`            | 2       | Tag snap (vision)            |
| `DEFAULT_FALLBACK_HEADING_DEG`    | 180.0°  | Hub aim when no target       |

Routine-specific values (in `AutoConfigurator` when building autos): shooter 3000 RPM, ±100 RPM tolerance, 1.0 s feed per shot.

---

## Related Code

| Purpose              | Location |
|----------------------|----------|
| Routine builders     | `frc.robot.Auto.commands.AutoRoutines` |
| Registration         | `frc.robot.AutoConfigurator` (`registerFullAutos`) |
| Selection & execution| `frc.robot.Auto.AutoManager` |
| Start poses & landmarks | `frc.robot.PathPlanner.Landmarks`, `BlueLandmarks` |
| Path names (L/C/R)   | `frc.robot.autotest.MoleculeTests.getPathNameForPose()` |
| Testing atoms/molecules/full autos | [autonomous_testing_framework.md](autonomous_testing_framework.md) |
