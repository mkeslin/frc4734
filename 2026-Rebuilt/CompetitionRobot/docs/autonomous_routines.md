# Autonomous Routines

This document describes the autonomous routines available on the robot: how they are selected and run, what each routine does step-by-step, and the constants that affect behavior. For testing individual commands and sequences, see [Autonomous Testing Framework](autonomous_testing_framework.md).

---

## How Auto Is Run

- **Selection**: The drive team chooses a routine from the **"Auto Mode (manager)"** chooser on SmartDashboard. `AutoManager` holds a `SendableChooser<AutoRoutine>` and publishes it there. **ClimberAuto (Middle)** always uses the **left** tower face (`TowerAlignLeftOffset`); there is no climb-side chooser.
- **Start**: When the match enters autonomous, `Robot.autonomousInit()` calls `AutoManager.runSelectedRoutine()`. That:
  - Resets odometry to the routine’s **initial pose**
  - Schedules the routine’s command (with a one-time timer stop when the command finishes)
- **End**: In `Robot.autonomousExit()`, `AutoManager.endRoutine()` cancels the running command.

Landmarks and paths are **alliance-aware**: `Landmarks` uses `AllianceUtils` so blue/red get the correct poses (e.g. red is a 180° flip). Path names use **L** (left / POS_1), **C** (center / POS_2), **R** (right / POS_3)—e.g. `L_StartToShot`, `C_StartToCenter`, `R_CenterToShot`. See `MoleculeTests.getPathNameForPose()` and PathPlanner path files under `src/main/deploy/pathplanner/`.

---

## Available Routines

| Routine               | Start position | Description                                      |
|-----------------------|----------------|--------------------------------------------------|
| Test - PathPlanner    | POS_1 (left)   | PathPlanner sanity check: seed left start, pathfind to shot pose (`AutoRoutines.buildTestPathPlanner`). |
| Test - Drive and Shoot | POS_2 (center) | Path to shot, deploy intake, shoot once (`AutoRoutines.buildTestDriveAndShoot`). |
| Test - Climb          | Test pose      | Short climb checkout (`AutoRoutines.buildTestClimb`). |
| ClimberAuto (Middle)  | POS_2 (center) | Same prelude as **ShooterAuto (Center)** (`C_StartToShot` + `SHOOTER_AUTO_CENTER_SHOOT_DURATION`), then tower + climb on the **left** tower face only. |
| ShooterAuto (Left)    | POS_1 (left)   | Shoot preload → through center (intake) → return → shoot |
| ShooterAuto (Center)  | POS_2 (center) | C_StartToShot → shoot preload → stop.               |
| ShooterAuto (Right)   | POS_3 (right)  | Same flow from right start.                         |

---

## ClimberAuto (Middle)

**Goal**: Shoot the preload from **center start (POS_2)**, then drive to the tower and climb (L1), then hold until auto ends.

**Shot prelude (steps 1–5 of `pathToShotThenShoot`)** matches **ShooterAuto (Center)**: seed at POS_2, tag snap, **`CmdFollowPath` `C_StartToShot`**, deploy intake, shoot for `AutoConstants.SHOOTER_AUTO_CENTER_SHOOT_DURATION`.

**Tower + climb (steps 8–15)** — drive to **`TowerAlignLeftOffset`** (blue), tag snap, fine align, extend L1, drive toward bar, nudge, retract, hold.

Build: `AutoRoutines.buildClimberAuto()`; registration: `AutoConfigurator.registerFullAutos()`. Walkthrough: [ClimberAuto walkthrough](climber_auto_walkthrough.md).

---

## ShooterAuto (Left / Right)

**Goal**: Shoot preload, drive through center on a serpentine path (intaking notes), then shoot again at the other shot position.

**Steps (in order):**

1. **Seed odometry** – Start at POS_1 (left) or POS_3 (right); pose from `Landmarks.OurStart1()` or `OurStart3()`.
2. **Tag snap (if good)** – Same as ClimberAuto.
3. **Path to shot (with shooter spin-up)** – In parallel:
   - **CmdFollowPath** `L_StartToShot` (Left) or `R_StartToShot` (Right), 10 s timeout.
   - **CmdShooterSpinUp** to 3000 RPM.
4. **Lower intake** – Deploy intake so webcam is unblocked for hub aim.
5. **Acquire hub aim** – Same as ClimberAuto.
6. **Wait shooter at speed** – 3000 RPM ± 100.
7. **Shoot** – Feed for 1.0 s.
8. **Path through center (with intake on until endpoint)** – In parallel:
   - **CmdFollowPath** `L_ThroughCenter` (Left) or `R_ThroughCenter` (Right), 10 s timeout.
   - **CmdIntakeOn** (intake deployed and running). Intake stops when path finishes.
9. **Path back to shot (with shooter spin-up)** – In parallel:
   - **CmdFollowPath** `L_ThroughCenterReturn` (Left) or `R_ThroughCenterReturn` (Right), 10 s timeout.
   - **CmdShooterSpinUp** to 3000 RPM.
10. **Acquire hub aim** – Again.
11. **Wait shooter at speed** – Again.
12. **Shoot** – Feed for 1.0 s again.

Summary: **preload shot → serpentine through center (intake) → second shot.** Left uses `L_*` paths; Right uses `R_*` paths. Build logic: `AutoRoutines.buildShooterAutoThroughCenter()` (chooser: **ShooterAuto (Left | Right)**); registration: `AutoConfigurator.registerFullAutos()`.

---

## ShooterAuto (Center)

**Goal**: Follow C_StartToShot, shoot preload, then stop.

**Steps (in order):**

1. **Seed odometry** – Start at POS_2 (center).
2. **Tag snap (if good)** – Same as ClimberAuto.
3. **Path to shot (with shooter spin-up)** – **CmdFollowPath** `C_StartToShot`, 10 s timeout.
4. **Lower intake** – Deploy intake so webcam is unblocked (if present).
5. **Acquire hub aim** – Vision-based heading to hub.
6. **Wait shooter at speed** – 3000 RPM ± 100.
7. **Shoot** – Feed for 1.0 s.
8. **Stop** – Routine ends.

Build logic: `AutoRoutines.buildShooterAutoCenter()` (chooser: **ShooterAuto (Center)**); registration: `AutoConfigurator.registerFullAutos()`.

---

**PathPlanner paths (ShooterAuto):**

| Path file | Left (L_) | Center (C_) | Right (R_) | Use |
|-----------|-----------|-------------|------------|-----|
| `*_StartToShot.path` | ✓ | ✓ | ✓ | Start → shot (preload) |
| `*_ThroughCenter.path` | ✓ | — | ✓ | Shot → through center |
| `*_ThroughCenterReturn.path` | ✓ | — | ✓ | Center → back to shot |

---

## Key Constants

Defined in `AutoConstants.java`; used by the commands above.

| Constant                         | Value   | Use                          |
|----------------------------------|---------|------------------------------|
| `DEFAULT_PATH_TIMEOUT`           | 10.0 s  | Path following               |
| `DEFAULT_POSE_TIMEOUT`            | 5.0 s   | Drive-to-pose                |
| `DEFAULT_HEADING_TIMEOUT`         | 3.0 s   | Snap to heading              |
| `DEFAULT_INTAKE_DEPLOY_TIMEOUT`   | 2.0 s   | Lower intake before hub aim  |
| `DEFAULT_CLIMB_ACQUIRE_DISTANCE_METERS` | 0.2 m | Drive forward to acquire bar |
| `DEFAULT_CLIMB_ACQUIRE_TIMEOUT`   | 3.0 s   | Drive-forward-to-acquire timeout |
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
| Routine builders     | `frc.robot.Auto.commands.AutoRoutines` (class javadoc maps chooser labels → `build*` methods) |
| Registration         | `frc.robot.AutoConfigurator` (`registerFullAutos`) |
| Climber tower face   | Hardcoded **left** in `AutoRoutines.buildClimberAuto` (`BlueLandmarks.TowerAlignLeftOffset`) |
| Selection & execution| `frc.robot.Auto.AutoManager` |
| Start poses & landmarks | `frc.robot.PathPlanner.Landmarks`, `BlueLandmarks` |
| Path names (L/C/R)   | `frc.robot.autotest.MoleculeTests.getPathNameForPose()` |
| Testing atoms/molecules/full autos | [autonomous_testing_framework.md](autonomous_testing_framework.md) |
