# Future Features & Research Notes

## Nozzle Cleaner/Wiper

**Status:** NOT YET INSTALLED - add hardware first, then create macro

### Prusa Firmware Reference (iX only)
The nozzle cleaner code in `Prusa-Firmware-Buddy/src/feature/nozzle_cleaner/nozzle_cleaner.cpp` is for **Prusa iX (industrial)** only, not standard XL.

### XL Community Options

**1. Front-Mount Brush** (most popular)
- Position: X=160-200, Y=-5 to -9, Z=1-5mm
- Mounts on front frame

**2. Back Frame Mount** (near tool parking)
- Mounts on 3030 frame with M3 T-nuts
- Wipes during park/unpark

**Note:** Prusa XL does NOT have built-in nozzle brushes at tool docks.

### Resources
- [Printables: Nozzle Scrubber](https://www.printables.com/model/462473)
- [Printables: Pre-Leveling Nozzle Wiper](https://www.printables.com/model/678695)
- [Amazon: FYSETC Nozzle Wiper Kit](https://www.amazon.com/Original-Prussa-Upgrade-Nozzle-Wiper/dp/B0DMP7K5HY)

### Integration Plan (when hardware added)
1. Create `CLEAN_NOZZLE` / `G12` macro
2. Add to deviation check - clean before retry if tare deviation detected
3. Option to clean before bed mesh probing

---

## MSCNT Phase-Based Precise Homing (NOT IMPLEMENTED)

### What is MSCNT?
MSCNT (Microstep Counter) is TMC driver register (0x6A) reporting position within motor's electrical cycle:
- 10-bit value (0-1023)
- Each full electrical cycle = 4 full steps = 1024 microsteps

### Klipper Already Has MSCNT Support
```python
tmc = self.printer.lookup_object('tmc2130 stepper_x')
reg = tmc.mcu_tmc.get_register('MSCNT')
mscnt = reg & 0x3ff
```
Or via console: `DUMP_TMC STEPPER=stepper_x`

We also have `MSCNT_TEST` command in puppy_bootloader.py.

### How Prusa Uses It
**Key file:** `Prusa-Firmware-Buddy/lib/Marlin/Marlin/src/module/prusa/homing_corexy.cpp`

1. Phase stepping DISABLED during homing
2. After sensorless home, reads MSCNT for both A and B motors
3. Calculates `phase_backoff_steps()` to align to phase zero
4. Measures from 9 grid points to calibrate "cycle origin"
5. Stores calibration in EEPROM
6. Gives sub-microstep homing repeatability

### Implementation Challenge
In Klipper, can read MSCNT but not while motors are moving. Would need integration into homing_override or custom module.

---

## Babystep Z Offset - KNOWN ISSUE

**WARNING:** Babysteps cause Z coordinate drift. DO NOT USE until debugged.

**Workaround:** Use slicer Z offset instead of Klipper babysteps.

**TODO:** Debug why babystep/gcode offset causes Z position desync with loadcell probing.

To reset: Set `babystep_z_offset = 0.0` in variables.cfg and restart Klipper.

---

## First Layer & Extrusion Settings (Prusa-Matched)

**From G26.cpp:**
- filamentD = 1.75f, layerHeight = 0.2f, threadWidth = 0.5f

**From Configuration_XL.h:**
- DEFAULT_NOMINAL_FILAMENT_DIA 1.75
- EXTRUDE_MAXLENGTH 1000
- DEFAULT_MAX_FEEDRATE { 550, 550, 12, 45 }
- DEFAULT_ACCELERATION 1800
- DEFAULT_RETRACT_ACCELERATION 1200
- DEFAULT_TRAVEL_ACCELERATION 5000

| Setting | Prusa Value | Our Klipper | Status |
|---------|-------------|-------------|--------|
| First layer height | 0.2mm | slicer | Use 0.2mm |
| Extrusion width | 0.5mm | slicer | Use 0.5mm |
| Filament diameter | 1.75mm | 1.75mm | Match |
| Extruder rotation_dist | 8.42mm | 8.42mm | Match |
| Print acceleration | 1800mm/s^2 | 1800mm/s^2 | Match |
