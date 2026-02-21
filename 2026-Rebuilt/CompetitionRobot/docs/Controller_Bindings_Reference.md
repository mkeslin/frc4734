# Controller Bindings Reference

Print this for driver/pit reference. **Drive profile** is selected on the Driver tab (or Back+Start on drive controller). Mechanism mode follows automatically.

---

## Drive profile → mechanism mode

| Drive profile | Mechanism mode | Drive controller | Mechanism controller |
|---------------|----------------|------------------|----------------------|
| **NORMAL**    | MECHANISM      | Normal driving   | Molecule bindings    |
| **SYSID**     | SYSID          | Drivetrain SysId | SysId (PID tuning)   |
| **TUNING**    | INDIVIDUAL     | Brake; A = tuning path | One button per mechanism |

**Switch profile:** Driver tab dropdown **Bindings Profile** (default: NORMAL), or hold **Back + Start** on drive controller to cycle NORMAL → SYSID → TUNING → NORMAL.

---

## Drive controller (all profiles)

| Action | Binding |
|--------|--------|
| Drive | Left stick (field-centric) |
| Rotate | Right stick X |
| Turtle mode | Left bumper (held) |
| Reset heading | Right bumper |
| **Profile cycle** | **Back + Start** (when enabled) |

**SYSID profile only:** A = translation SysId, B = steer SysId, Right bumper = rotation SysId; Back+Y / Back+X = dynamic; Start+Y / Start+X = quasistatic.

**TUNING profile only:** A = run tuning path (zzTuning-1).

---

## Mechanism controller — MECHANISM (competition)

*When drive profile is NORMAL.*

| Button | Action |
|--------|--------|
| **D-pad down** | Deploy intake (single press) |
| **D-pad up** | Stow intake (single press) |
| **Left trigger** (held) | Intake + floor + feeder (forward) |
| **Left bumper** (held) | Reverse intake, floor, feeder |
| **Right trigger** (held) | Shoot: shooter, then delayed feeder, then delayed floor |
| **Right bumper** (held) | Reverse shooter, feeder, floor |
| **Back** | Stop all (shooter, feeder, floor, intake) |

---

## Mechanism controller — SYSID (PID tuning)

*When drive profile is SYSID. A/B/X/Y = quasi fwd, quasi rev, dynamic fwd, dynamic rev.*

| Mechanism | Buttons |
|-----------|---------|
| Shooter | A, B, X, Y (no modifier) |
| Feeder | Right bumper + A, B, X, Y |
| Floor | Left bumper + A, B, X, Y |
| Deploy motor | POV Up + A, B, X, Y |
| Intake motor | POV Down + A, B, X, Y |

---

## Mechanism controller — INDIVIDUAL (one button per mechanism)

*When drive profile is TUNING.*

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

*Generated from ControllerBindingFactory and SwerveDrivetrainBindings. Default profile: NORMAL (Competition).*
