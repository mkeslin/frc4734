# ClimberAuto (Middle) Walkthrough

This document walks through **ClimberAuto (Middle)** — **center start (POS_2)** only. The climb **tower face** is hardcoded to the **left** bar side (`BlueLandmarks.TowerAlignLeftOffset`).

---

## Goal

Shoot the preload (same prelude as **ShooterAuto (Center)**), drive to the tower, climb once (L1), then hold until autonomous ends.

---

## Steps (in order)

### A. Shot prelude (same as ShooterAuto (Center))

1. **Seed odometry** — `OurStart2()` (alliance-aware).
2. **Tag snap (if good)** — Optional vision snap if quality passes thresholds.
3. **Path + spin up** — Deadline: **`C_StartToShot`** + shooter spin-up (center tuning).
4. **Deploy intake** — If present, lower for shoot / camera clearance.
5. **Shoot** — Duration `SHOOTER_AUTO_CENTER_SHOOT_DURATION` (not the general Shuffleboard shoot duration).

### B. Tower + climb

6. **Drive to tower align** — `CmdDriveToPose` to **`TowerAlignLeftOffset`** (blue coordinates).
7. **Tag snap again** — Refine pose at tower.
8. **Fine align** — Second drive to same tower pose.
9. **Extend L1** — `extendL1Only` (timeout from `AutoConstants`).
10. **Drive toward bar** — Short pathfind step toward the bar (−X in blue from captured pose).
11. **Nudge** — `CmdDriveFieldRelative.forDistance` (small field-relative jog).
12. **Retract from L1** — `retractFromL1`.
13. **Hold climb** — `CmdHoldClimbUntilEnd` until auto ends.

---

## Related code

- **Builder:** `AutoRoutines.buildClimberAuto()`
- **Registration:** `AutoConfigurator.registerFullAutos()`
- **Overview:** [autonomous_routines.md](autonomous_routines.md)
