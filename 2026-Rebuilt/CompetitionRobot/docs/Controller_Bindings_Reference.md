# Controller Bindings Reference

Print this for driver/pit reference. **Mode** is selected on the Driver tab (or Back+Start on drive controller). Modes: Auto (no bindings), Teleop, SysId, Mechanism.

---

## Mode summary

| Mode | Drive controller | Mechanism controller |
|------|------------------|----------------------|
| **Teleop** | Normal driving | Teleop commands (intake combo, shoot combo, deploy/stow) |
| **SysId** | Drivetrain SysId | SysId (PID tuning) |
| **Mechanism** | Brake; A = tuning path | Individual mechanism commands (one button per mechanism) |

**Switch mode:** Driver tab dropdown **Bindings Profile** (default: Teleop), or hold **Back + Start** on drive controller to cycle Teleop → SysId → Mechanism → Teleop.

---

## Drive controller (all modes)

| Action | Binding |
|--------|--------|
| Drive | Left stick (field-centric) |
| Rotate | Right stick X |
| Turtle mode | Left bumper (held) |
| Reset heading | Right bumper |
| **Profile cycle** | **Back + Start** (when enabled) |

**SysId mode only:** A = translation SysId, B = steer SysId, Right bumper = rotation SysId; Back+Y / Back+X = dynamic; Start+Y / Start+X = quasistatic.

**Mechanism mode only:** A = run tuning path (zzTuning-1).

---

## Mechanism controller — Teleop (competition)

*When mode is Teleop.*

| Button | Action |
|--------|--------|
| **D-pad down** | Deploy intake (single press) |
| **D-pad up** | Stow intake (single press) |
| **Left trigger** (held) | Intake + floor (forward) |
| **Left bumper** (held) | Reverse intake, floor, feeder |
| **Right trigger** (held) | Shoot: shooter, then delayed feeder, then delayed floor |
| **Right bumper** (held) | Reverse shooter, feeder, floor |
| **Back** | Stop all (shooter, feeder, floor, intake) |

---

## Mechanism controller — SysId (PID tuning)

*When mode is SysId. A/B/X/Y = quasi fwd, quasi rev, dynamic fwd, dynamic rev.*

| Mechanism | Buttons |
|-----------|---------|
| Shooter | A, B, X, Y (no modifier) |
| Feeder | Right bumper + A, B, X, Y |
| Floor | Left bumper + A, B, X, Y |
| Deploy motor | POV Up + A, B, X, Y |
| Intake motor | POV Down + A, B, X, Y |

---

## Mechanism controller — Mechanism (one button per mechanism)

*When mode is Mechanism (tuning).*

| Button | Action |
|--------|--------|
| **D-pad down** | Deploy intake (single press) |
| **D-pad up** | Stow intake (single press) |
| **Left trigger** (held) | Intake in |
| **Left bumper** (held) | Shooter reverse |
| **Right bumper** (held) | Shooter forward |
| **Right trigger** (held) | Floor reverse |
| **A** (held) | Feeder forward |
| **B** | Feeder off |
| **X** (held) | Feeder reverse |
| **Y** (held) | Floor forward |
| **Back** | Shooter off |

---

*Generated from ControllerBindingFactory and SwerveDrivetrainBindings. Default mode: Teleop (Competition).*
