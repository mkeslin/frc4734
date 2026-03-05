# PathPlanner “Drive to Pose” in Auto Routines — How Other Teams Do It

This doc summarizes how PathPlanner pathfinding is used for **driving to a specific pose** in autonomous, based on official docs and common FRC patterns. It’s meant to align our implementation with best practice and fix “robot didn’t move” / pathfinding issues.

---

## 1. Official PathPlanner Patterns

### 1.1 Pathfind to pose (what we use)

- **Docs:** [Pathfinding | PathPlanner](https://pathplanner.dev/pplib-pathfinding.html), [AutoBuilder API](https://pathplanner.dev/api/java/com/pathplanner/lib/auto/AutoBuilder.html).
- **Idea:** Use `AutoBuilder.pathfindToPose(pose, constraints, goalEndVelocity)` or **`pathfindToPoseFlipped`** so the path is mirrored for red alliance (origin stays blue).
- **Requirements:**
  - `navgrid.json` in `deploy/pathplanner/` (we have it).
  - AutoBuilder configured once (pose supplier, reset pose, robot-relative speeds, drive output, controller, config, **shouldFlipPath**, drive subsystem).
  - For **holonomic (swerve)** the pose’s rotation is the goal holonomic rotation; for differential it’s ignored.

**Java (from docs):**

```java
Pose2d targetPose = new Pose2d(10, 5, Rotation2d.fromDegrees(180));
PathConstraints constraints = new PathConstraints(
    3.0, 4.0, Units.degreesToRadians(540), Units.degreesToRadians(720));
Command pathfindingCommand = AutoBuilder.pathfindToPose(
    targetPose, constraints, 0.0, 0.0);
```

**Alliance flipping:** Use `pathfindToPoseFlipped(poseBlue, constraints, goalEndVelocity)` and pass **blue** poses; the flip supplier in AutoBuilder handles red. We do this in `pathfindToPoseBlue()`.

### 1.2 Pathfind then follow path (for precision)

- **Docs:** same pathfinding page, “Pathfind Then Follow Path”.
- **Idea:** Pathfinding is not great at final alignment (heading/position). For precise spots (e.g. human player, scoring), teams often:
  1. **Pathfind** to the start of a **pre-planned path**.
  2. **Follow** that path for the last bit.

```java
PathPlannerPath path = PathPlannerPath.fromPathFile("Example Human Player Pickup");
PathConstraints constraints = new PathConstraints(3.0, 4.0, ...);
Command cmd = AutoBuilder.pathfindThenFollowPath(path, constraints);
```

So: **“Drive to pose”** in auto is either:
- **Direct:** `pathfindToPose` / `pathfindToPoseFlipped` only, or  
- **Precision:** `pathfindThenFollowPath` with a short path that ends at the desired pose.

---

## 2. How teams structure auto routines

- **Load autos from GUI:** `new PathPlannerAuto("Example Auto")` or `AutoBuilder.buildAutoChooser()` so autos are defined in PathPlanner and loaded by name.
- **Mix pathfinding with other commands:** Pathfinding commands are normal WPILib `Command`s. Teams use `Commands.sequence()`, `Commands.deadline()`, etc. Example pattern:
  - Seed odometry → optional tag snap → **pathfind to pose** (or pathfindThenFollowPath) → shoot / intake / climb, etc.
- **Target pose in blue:** PathPlanner’s field/navgrid are blue-origin. Pass **blue** targets and use `pathfindToPoseFlipped` so red is handled by the library.

So “drive to a specific pose” in auto is:
1. Define the target pose (usually blue).
2. Build a command with `AutoBuilder.pathfindToPoseFlipped(poseBlue, constraints, 0)` (or pathfindThenFollowPath).
3. Put that command in a sequence with seed, vision, and mechanism commands.

---

## 3. Our implementation vs standard pattern

| Aspect | Standard (docs / typical teams) | Our code |
|--------|---------------------------------|----------|
| **API** | `AutoBuilder.pathfindToPoseFlipped(pose, constraints, goalEndVel)` | Same in `CommandSwerveDrivetrain.pathfindToPoseBlue(poseBlue)` ✓ |
| **Alliance** | Blue poses + flip supplier | We pass blue; flip supplier in AutoBuilder ✓ |
| **In a sequence** | Pathfind command used in `Commands.sequence(...)` | We use `Commands.defer(...)` so target is resolved when the command is **scheduled** ✓ |
| **Completion** | Command ends when pathfinder/controller says done | We add `.until(() -> atPose(...)).withTimeout(timeout)` for tolerance + timeout ✓ |
| **navgrid** | Required in `deploy/pathplanner/` | Present ✓ |

Our `CmdDriveToPose` wraps the pathfind command with:
- **Defer** so the target pose is from a supplier at schedule time (alliance/chooser correct).
- **beforeStarting / finallyDo** for logging (and we made logging safe so it can’t break the command).
- **.until(atPose)** and **.withTimeout** so we finish at a defined tolerance or time.

This matches the usual “pathfind to pose in a sequence” idea; the main difference is we resolve the target lazily and add explicit completion/tolerance.

---

## 4. AdvantageKit and pathfinding

PathPlanner’s pathfinding page states for **AdvantageKit**:

- Call **`Pathfinding.setPathfinder(new LocalADStarAK());`** at the **start** of `robotInit()` (before other init that might create pathfinding commands).
- `LocalADStarAK` is an AdvantageKit-compatible pathfinder so pathfinding behaves correctly during **log replay** (see [PathPlanner pathfinding docs](https://pathplanner.dev/pplib-pathfinding.html), “AdvantageKit Compatibility”).

**Our project:** We use AdvantageKit but do **not** set a custom pathfinder; we only schedule warmup commands. Without `LocalADStarAK`:
- Pathfinding may still work on the robot if the default pathfinder is used.
- Replay of pathfinding in AdvantageKit may be wrong or inconsistent.

**Recommendation:** Add `Pathfinding.setPathfinder(new LocalADStarAK());` at the top of `robotInit()` and add the `LocalADStarAK` class (e.g. from [Michael Jansen’s gist](https://gist.github.com/mjansen4857/a8024b55eb427184dbd10ae8923bd57d)) if we want correct replay. If the robot doesn’t move at all in auto, the first thing to verify is AutoBuilder config + navgrid + no exceptions; then try the AdvantageKit pathfinder if needed.

---

## 5. Things that prevent “drive to pose” from working

- **No navgrid:** Pathfinding won’t run correctly. We have `navgrid.json`.
- **AutoBuilder not configured:** e.g. exception in `RobotConfig.fromGUISettings()` or in `configure()`. Check logs for “[PathPlanner] Failed to load…”.
- **Target pose invalid:** e.g. null from supplier, or outside navigable area. We guard null and log; ensure pose is in blue and inside the grid.
- **AdvantageKit (optional):** Default pathfinder might not play nice with AdvantageKit; set `LocalADStarAK` if we use log replay and see odd pathfinding.
- **Command never runs:** e.g. previous command in the sequence doesn’t finish, or requirement conflict. Our use of `Commands.defer` and `Set.of(drivetrain)` is correct so the deferred command gets the drivetrain when it runs.

---

## 6. Minimal “drive to pose” test (like PathPlanner Test A→B)

Pattern used in docs and by teams:

1. **Seed odometry** to start pose.
2. **One pathfind command** to a fixed blue pose (e.g. shot position).
3. Optional: timeout and/or `.until(atPose)` for early exit.

Our PathPlanner Test (A→B) does exactly that: seed then `CmdDriveToPose.create(drivetrain, () -> BlueLandmarks.ShotPosition, ...)`. No tag snap, no mechanisms. If that fails, the problem is in pathfinding/AutoBuilder/navgrid or logging; if it passes, the issue is likely in a later step (e.g. climber auto’s first CmdDriveToPose or a supplier).

---

## 7. References

- [Pathfinding | PathPlanner](https://pathplanner.dev/pplib-pathfinding.html)
- [Build an Auto | PathPlanner](https://pathplanner.dev/pplib-build-an-auto.html)
- [AutoBuilder Java API](https://pathplanner.dev/api/java/com/pathplanner/lib/auto/AutoBuilder.html) — `pathfindToPose`, `pathfindToPoseFlipped`, `pathfindThenFollowPath`
- [LocalADStarAK gist (AdvantageKit)](https://gist.github.com/mjansen4857/a8024b55eb427184dbd10ae8923bd57d)
- PathPlanner source: [mjansen4857/pathplanner](https://github.com/mjansen4857/pathplanner)

---

## 8. Summary

- **“Drive to pose” in auto** = use `AutoBuilder.pathfindToPoseFlipped(poseBlue, constraints, goalEndVel)` (or `pathfindThenFollowPath` for precision), with AutoBuilder configured and `navgrid.json` in place.
- **Our approach** (CmdDriveToPose with defer, blue targets, .until(atPose), timeout) matches this; we added safe logging so it can’t break the command.
- **Next steps if it still doesn’t move:** (1) Confirm AutoBuilder config and navgrid, (2) add AdvantageKit pathfinder `LocalADStarAK` in `robotInit()` if we care about replay and/or see odd behavior, (3) use the minimal PathPlanner Test (A→B) to isolate pathfinding from the rest of the routine.
