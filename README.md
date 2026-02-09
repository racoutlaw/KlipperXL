<p align="center">
  <img src="KlipperXL.png" alt="KlipperXL Logo" width="300">
</p>

<h1 align="center">KlipperXL</h1>
<p align="center"><b>Run Klipper firmware on your Prusa XL.</b></p>

KlipperXL replaces the stock Prusa firmware with [Klipper](https://www.klipper3d.org/), giving you full control over your Prusa XL through Mainsail or Fluidd. All 5 Dwarf toolheads work via MODBUS communication with the XLBuddy mainboard.

---

## Features

### Tool Management
- **Full 5-Tool Support** - Automatic tool changes with Prusa-matched dock sequences (pick, park, wiggle retry)
- **Tool Offset Calibration** - Prusa G425-equivalent calibration using the calibration pin (70C, 3mm/s probe speed, bounds checking)
- **Multi-Color / Multi-Material** - Slicer-managed tool temperatures with wipe tower support
- **Spool Join** - Continue printing with a different tool after filament runout (remaps tool commands on the fly)
- **Tool Buttons** - Physical Dwarf buttons for manual filament load/unload (up = retract, down = extrude)

### Filament Handling
- **Filament Autoload** - Insert filament and it automatically loads, heats, purges, and retracts (Prusa-matched sequence)
- **Filament Sensor Monitoring** - Side and nozzle sensor state tracking per tool via MODBUS polling

### Bed & Probing
- **Loadcell Z Probing** - Uses the Dwarf's built-in loadcell for Z homing and bed mesh leveling
- **Adaptive Bed Mesh** - Probes only the print area, not the entire bed
- **Modular Bed Control** - 16-zone (4x4 grid) heated bed with individual bedlet temperature control
- **Adaptive Bed Heating** - Heats only bedlets under the print area with gradient heating on adjacent zones (anti-warp)
- **Adjustable Probe Thresholds** - Change loadcell sensitivity for Z, mesh, and XY calibration from Mainsail

### Motion & Calibration
- **Input Shaper** - Resonance compensation via Dwarf accelerometers (per-tool measurement)
- **Z Tilt Alignment** - Prusa-matched mechanical bed leveling
- **Babystep Z** - Live Z offset adjustment during printing

### Heater Management
- **Multi-Tool Heater Preservation** - Parked tools maintain temperature via periodic MODBUS refresh
- **Independent Tool Heating** - Heat any tool by number (M104 T2 S200) without selecting it
- **Automatic Heater Sync** - Active tool temperature synced to Klipper's extruder for accurate display

### Interface & Monitoring
- **Side LED Strips** - Status effects for idle, printing, heating, paused, error, and complete states
- **Modular Bed Monitoring** - `BED_STATUS` command shows 4x4 bedlet temperature grid with physical wiring map
- **Webcam Integration** - USB camera support via Crowsnest
- **OrcaSlicer Ready** - Pre-configured profile and macros included

## Hardware Requirements

| Component | Details |
|-----------|---------|
| **Printer** | Prusa XL (any toolhead count, 1-5) |
| **Mainboard** | XLBuddy with STM32F407 |
| **Toolheads** | Dwarf boards (stock Prusa firmware, unmodified) |
| **Host** | Raspberry Pi 4 or 5 |
| **Calibration** | Prusa calibration pin (for tool offsets) |

## How It Works

KlipperXL uses a two-layer architecture:

1. **MCU Firmware** - Custom Klipper firmware on the XLBuddy acts as a MODBUS RTU master, communicating with the Dwarf toolheads over RS485 at 230400 baud
2. **Python Host Module** - A Klipper extra (`puppy_bootloader.py`) running on the Raspberry Pi handles Dwarf boot sequencing, tool changes, heater control, loadcell probing, filament management, and calibration

The Dwarf toolheads run **stock Prusa firmware** - no modifications needed. KlipperXL speaks the same MODBUS protocol that Prusa's firmware uses.

## Quick Start

1. Download the install package from this repo
2. Follow the [Installation Guide](docs/INSTALLATION_GUIDE.md)
3. Set up OrcaSlicer using the [Slicer Guide](orcaslicer/README.md)

## Package Contents

```
KlipperXL/
  docs/                      Technical documentation & installation guide
  config/                    Klipper configs (printer.cfg, macros, LEDs)
  klippy/                    Klipper Python extras (tool control, probing, MODBUS)
  src/                       XLBuddy MCU firmware source (MODBUS + babystep)
  firmware/                  Build config & recovery files
  orcaslicer/                Slicer profile and configuration guide
  scripts/                   Installation & fix scripts
```

## Key Commands

| Command | What It Does |
|---------|-------------|
| `T0` - `T4` | Select tool (automatic park/pick) |
| `TOOL_PARK` | Park current tool |
| `G28` | Home all axes |
| `PRUSA_Z_LEVEL` | Z tilt alignment (loud) |
| `CALIBRATE_TOOL_OFFSETS` | Calibrate tool XYZ offsets |
| `SHOW_TOOL_OFFSETS` | Display current offsets |
| `DWARF_STATUS` | Show all toolhead temperatures |
| `BED_STATUS` | Show 4x4 bedlet temperature grid |
| `SET_BED_AREA X0= Y0= X1= Y1=` | Set adaptive bed heating zone |
| `CLEAR_BED_AREA` | Reset to full bed heating |
| `SET_DWARF_TEMP DWARF= TEMP=` | Set individual tool temperature |
| `SPOOL_JOIN TO=` | Remap current tool to a different physical tool |
| `SPOOL_JOIN_STATUS` | Show active spool join remaps |
| `SHOW_THRESHOLDS` | Show probe threshold settings |
| `SET_Z_FAST VALUE=` | Adjust Z fast probe threshold |
| `SET_Z_SLOW VALUE=` | Adjust Z slow probe threshold |
| `SET_MESH VALUE=` | Adjust bed mesh probe threshold |
| `SET_XY VALUE=` | Adjust XY calibration probe threshold |

## OrcaSlicer Setup

Use the built-in **Generic Tool Changer** profile as your base. You only need to change 4 settings:

1. **G-code flavor** - Set to Klipper
2. **Start G-code** - Replace with single `START_PRINT` macro call
3. **End G-code** - Replace with `END_PRINT`
4. **Acceleration values** - Match to Prusa XL limits

Full details in the [Slicer Guide](orcaslicer/README.md).

## Calibrated Settings

These values have been tuned on a 5-tool Prusa XL:

| Setting | Value |
|---------|-------|
| Pressure Advance | 0.025 |
| Input Shaper X | EI @ 60.2 Hz |
| Input Shaper Y | MZV @ 23.0 Hz |
| Max Volumetric Flow (PLA 215C) | 10 mm³/s |
| Probe Threshold (Z/Mesh) | 125g |
| Probe Threshold (XY Cal) | 40g |

*Recalibrate for your own machine - these are starting points.*

## Documentation

| Document | Description |
|----------|-------------|
| [Installation Guide](docs/INSTALLATION_GUIDE.md) | Step-by-step setup from scratch |
| [OrcaSlicer Guide](orcaslicer/README.md) | Slicer configuration |
| [Technical Specification](docs/SPECIFICATION.md) | MODBUS protocol, register maps, architecture |
| [Motor Settings](docs/reference/motor_settings.md) | Pin assignments, TMC config, coordinates |
| [Probe Thresholds](docs/reference/probe_thresholds.md) | Loadcell calibration constants |
| [Bed Mesh](docs/reference/bed_mesh.md) | Mesh algorithm, adaptive probing |
| [Recovery](firmware/recovery/RECOVERY_INSTRUCTIONS.md) | DFU recovery for bricked XLBuddy |

## Safety Notes

- **Z+ = bed DOWN (safe), Z- = bed UP (crash risk)**
- Tool offsets MUST be recalibrated for your specific machine
- The XLBuddy requires DFU mode (physical jumper) for firmware flashing
- Dwarfs run stock Prusa firmware - do not modify them
- Prusa XL hardware acceleration limit is 7000 mm/s² - never exceed this
- Modular bed has a 120C maximum target temperature enforced in software

## Disclaimer

THIS SOFTWARE IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND. BY USING THIS SOFTWARE YOU ACKNOWLEDGE THAT YOU ARE MODIFYING YOUR PRINTER'S FIRMWARE AND ACCEPT ALL RISKS ASSOCIATED WITH DOING SO. THE AUTHOR IS NOT RESPONSIBLE FOR ANY DAMAGE TO YOUR PRINTER, PROPERTY, OR PERSON, INCLUDING BUT NOT LIMITED TO HARDWARE DAMAGE, FIRE, INJURY, OR LOSS OF ANY KIND. THIS PROJECT REPLACES STOCK FIRMWARE AND MAY VOID YOUR WARRANTY. USE AT YOUR OWN RISK. YOU ASSUME FULL RESPONSIBILITY FOR ANY AND ALL CONSEQUENCES OF INSTALLING AND USING THIS SOFTWARE.

## License

Copyright (C) 2026 Richard Crook. All rights reserved.

---

*Built for the Prusa XL community. If this project helps you, give it a star.*
