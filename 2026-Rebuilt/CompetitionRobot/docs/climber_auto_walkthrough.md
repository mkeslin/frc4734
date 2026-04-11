# ClimberAuto Walkthrough

This document walks through what the **ClimberAuto (Left)** routine does step by step. **ClimberAuto (Right)** matches left except start pose (POS_3) and midpoint shot geometry. **ClimberAuto (Middle)** uses the same **shot prelude as ShooterAuto (Center)** (`C_StartToShot` path + `SHOOTER_AUTO_CENTER_SHOOT_DURATION`), then the **same tower + climb steps** as left/right (steps 8–15 below). Only the start position and (for tower alignment) the **Climb Side** chooser differ among routines.

---

## Goal

Shoot the preload, drive to the tower, climb once (L1), then hold until autonomous ends.

---

## Steps (in order)

1. **Seed odometry**  
   The drivetrain’s pose is set to the chosen start: **Left** = `Landmarks.OurStart1()`, **Middle** = `OurStart2()`, **Right** = `OurStart3()` (alliance-aware). The robot “thinks” it is at that position.

2. **Tag snap (if good)**  
   If vision sees enough AprilTags (min 2, low ambiguity, within 5 m), the pose is snapped once to correct odometry. If not, this step does nothing.

3. **Drive to shot pose (with shooter spin-up)**  
   - **Deadline:** The “drive to shot” command is the main one; “shooter spin-up” runs until that drive finishes.  
   - **Drive:** `CmdDriveToPose` to the **midpoint** between the start pose and the tower align pose for the selected climb side (`Landmarks.midpointShotPose(start, towerAlign)`), so the shot position is equidistant from start and tower; rotation faces the hub. Tolerances: 0.1 m XY, 5° rotation; timeout 5 s.  
   - **Shooter:** Spins to 3000 RPM while driving.  
   When the robot reaches the shot pose (or times out), this step ends.

4. **Lower intake (unblock webcam)**  
   If the intake is stowed, it can block the webcam. This step deploys (lowers) the intake so vision can see the hub before aiming.

5. **Acquire hub aim**  
   Vision turns the robot to aim at the hub. If no target is seen, it uses the fallback heading (180°).

6. **Wait shooter at speed**  
   Waits until the shooter is at 3000 RPM ± 100.

7. **Shoot preload**  
   `CmdShootForTime`: feeder runs for **1.0 s** to shoot the preload.

8. **Drive to tower align pose**  
   `CmdDriveToPose` to the tower align pose from the **"Climb Side"** Shuffleboard chooser:  
   - **Climb side: Left** → `Landmarks.OurTowerAlignLeft()` (high Y in blue).  
   - **Climb side: Right** → `Landmarks.OurTowerAlignRight()` (low Y in blue).  
   Pose is “back into the circle end of the tower bar” (e.g. 270° in blue). Same timeouts/tolerances as before.

9. **Tag snap again**  
   Same vision snap as step 2, for better alignment at the tower.

10. **Drive to tower align again**  
    Same tower align pose as step 8, for a second, finer alignment.

11. **Drive forward to acquire bar**  
    The align pose stops the robot at the bar; the climber must then move forward a short distance to actually acquire (grab) the bar. `CmdDriveForward` drives forward **0.2 m** (configurable) along the current heading. Timeout **3 s**.

12. **Climb L1**  
    `ClimbWhileHeldCommand.ascentToCompletion(climber)`: one full L1 cycle (extend then retract). Timeout **15 s**.

13. **Hold climb until end**  
    `CmdHoldClimbUntilEnd`: keeps the climber in its current state until autonomous ends.

---

## Summary

**ClimberAuto (Left)** = start at left → (optional tag snap) → drive to **midpoint shot** + spin shooter → **lower intake** → aim at hub → wait for speed → shoot 1 s → drive to tower (side from chooser) → tag snap → drive to tower again → **drive forward to acquire bar** → climb L1 (15 s max) → hold until auto end.

---

## Constants

From `AutoConstants` / config:

- **Pose:** 0.1 m XY tolerance, 5° rotation, 5 s drive timeout.  
- **Shooter:** 3000 RPM, ±100 tolerance, 1.0 s shoot.  
- **Climb acquire:** 0.2 m forward, 3 s timeout.  
- **Climb:** 15 s timeout.  
- **Tag snap:** max ambiguity 0.2, max distance 5 m, min 2 targets.

---

## Related code

- **Builder:** `AutoRoutines.buildClimberAuto()`  
- **Registration:** `AutoConfigurator.registerFullAutos()`  
- **Overview:** [autonomous_routines.md](autonomous_routines.md)
