# Motor Settings - Prusa-Matched Reference

## X/Y Axis Pin Assignments (CRITICAL)
```ini
[stepper_x]
step_pin: PD7
dir_pin: PD6
# Homes to X=-8 (LEFT)

[stepper_y]
step_pin: PD5
dir_pin: PD4
# Homes to Y=-9 (FRONT) - CRITICAL: must be -9, not -8!
```

## Coordinate System (Prusa-matched)
- **X:** Home at X=-8 (left), max ~361, X+ moves RIGHT
- **Y:** Home at Y=-9 (front), max 461 (includes toolchanger), Y+ moves BACK (toward docks)
- **CRITICAL:** Y homes to -9 (Prusa Y_MIN_POS), NOT -8
- **Homing:** Goes to FRONT-LEFT corner (Prusa X_HOME_DIR=-1, Y_HOME_DIR=-1)

## Extruder (Nextruder with 1:10 planetary gear)
- rotation_distance: **8.42mm** (= 380 steps/mm with 16 microsteps)
- microsteps: 16

## TMC2130 Chopper Timing (CHOPPER_PRUSAMK3_24V)

Prusa defines: `{ 3, -2, 6 }` → register values:
| Parameter | Marlin Value | Register Value | Klipper Setting |
|-----------|--------------|----------------|-----------------|
| TOFF | 3 | 3 | driver_TOFF: 3 |
| HEND | -2 | 1 (adds +3) | driver_HEND: 1 |
| HSTRT | 6 | 5 (subtracts 1) | driver_HSTRT: 5 |

## Motor Currents (mA RMS)
| Axis | Run | Hold | Homing |
|------|-----|------|--------|
| X | 650 | 650 | 750 |
| Y | 650 | 650 | 750 |
| Z | 700 | 700 | 700 |
| E | 450 | 450 | - |

## Homing Settings (Prusa-matched)
- Homing retract: 20mm (X_HOME_BUMP_MM, Y_HOME_BUMP_MM)
- Homing current: 750mA during homing (via homing_override macro)
- Microsteps: 16 (with interpolate to 256)

## StallGuard Sensitivity (driver_SGT)
- X: 1, Y: 1 (for sensorless homing)
- Z: 4 (for crash detection - but Z tilt doesn't use StallGuard)

**Z Tilt Note:** Z tilt does NOT use StallGuard. It drives both Z motors until they physically crash into the top frame - pure mechanical alignment.

## Applied Fixes (2026-02-03)
- driver_HEND: Changed from 0 to 1 on X/Y/Z
- Homing current bump: 750mA during homing
- Homing retract: 20mm bump for X/Y
- Microsteps: Changed from 32 to 16 (less CPU load)
- X/Y motor pins swapped to fix Y direction
- Y coordinate system: Changed to Y_MIN at front (Prusa-matched)
- Y position_endstop: Changed from -8 to -9 (Prusa Y_MIN_POS)
- Tool pickup wiggle: Changed to dock_y + DOCK_WIGGLE
- All 5 tools (T0-T4) pick and park correctly with Prusa default DOCK_Y=455
