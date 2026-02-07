# KlipperXL

**Run Klipper firmware on your Prusa XL.**

KlipperXL replaces the stock Prusa firmware with [Klipper](https://www.klipper3d.org/), giving you full control over your Prusa XL through Mainsail or Fluidd. All 5 Dwarf toolheads work via MODBUS communication with the XLBuddy mainboard.

---

## Features

- **Full 5-Tool Support** - Automatic tool changes with Prusa-matched dock sequences
- **Loadcell Z Probing** - Uses the Dwarf's built-in loadcell for Z homing and bed mesh
- **Adaptive Bed Mesh** - Probes only the print area, not the entire bed
- **Tool Offset Calibration** - Prusa G425-equivalent calibration using the calibration pin
- **Multi-Color / Multi-Material** - Slicer-managed tool temperatures, wipe tower support
- **Modular Bed Monitoring** - All 16 bedlets visible with temperature tracking
- **Input Shaper** - Resonance compensation via Dwarf accelerometers
- **Side LED Strips** - Status effects for printing, heating, paused, complete
- **Webcam Integration** - USB camera support via Crowsnest
- **OrcaSlicer Ready** - Pre-configured profile and macros included

## Hardware Requirements

| Component | Details |
|-----------|---------|
| **Printer** | Prusa XL (any toolhead count) |
| **Mainboard** | XLBuddy with STM32F407 |
| **Toolheads** | Dwarf boards (stock firmware, unmodified) |
| **Host** | Raspberry Pi 4 or 5 |
| **Calibration** | Prusa calibration pin (for tool offsets) |

## How It Works

KlipperXL uses a two-layer architecture:

1. **MCU Firmware** - Custom Klipper firmware on the XLBuddy acts as a MODBUS RTU master, communicating with the Dwarf toolheads over RS485
2. **Python Host Module** - A Klipper extra (`puppy_bootloader.py`) running on the Raspberry Pi handles Dwarf boot sequencing, tool changes, heater control, loadcell probing, and calibration

The Dwarf toolheads run **stock Prusa firmware** - no modifications needed. KlipperXL speaks the same MODBUS protocol that Prusa's firmware uses.

## Quick Start

1. Download `XlKlipper_Install_Package.zip` from this repo
2. Follow the [Installation Guide](XlKlipper_Install_Package/INSTALLATION_GUIDE.md)
3. Set up OrcaSlicer using the [Slicer Guide](XlKlipper_Install_Package/orcaslicer/README.md)

## Package Contents

```
XlKlipper_Install_Package/
  INSTALLATION_GUIDE.md        Step-by-step setup instructions
  config_files/                Klipper configs (printer.cfg, macros, LEDs)
  python_modules/              Klipper extras (tool control, probing, MODBUS)
  mcu_firmware/                XLBuddy firmware source (MODBUS + babystep)
  orcaslicer/                  Slicer profile and configuration guide
  post_install/                Fix scripts for Moonraker Update Manager
```

## OrcaSlicer Setup

Use the built-in **Generic Tool Changer** profile as your base. You only need to change 4 settings:

1. **G-code flavor** - Set to Klipper
2. **Start G-code** - Replace with single `START_PRINT` macro call
3. **End G-code** - Replace with `END_PRINT`
4. **Acceleration values** - Match to Prusa XL limits

Full details in the [Slicer Guide](XlKlipper_Install_Package/orcaslicer/README.md).

## Project Structure

```
KlipperXL/
  XlKlipper_Install_Package/   Ready-to-install package (configs + firmware + docs)
  XlKlipper_Install_Package.zip Downloadable zip of the above
  klipper-modbus-module/        Source code (MCU firmware + Python modules)
  reference/                    Technical reference docs
```

## Key Commands

| Command | What It Does |
|---------|-------------|
| `T0` - `T4` | Select tool |
| `TOOL_PARK` | Park current tool |
| `G28` | Home all axes |
| `PRUSA_Z_LEVEL` | Z tilt alignment (loud) |
| `CALIBRATE_TOOL_OFFSETS` | Calibrate tool XYZ offsets |
| `SHOW_TOOL_OFFSETS` | Display current offsets |
| `DWARF_STATUS` | Show all toolhead temperatures |
| `BED_STATUS` | Show 4x4 bedlet temperature grid |

## Calibrated Settings

These values have been tuned on a 5-tool Prusa XL:

| Setting | Value |
|---------|-------|
| Pressure Advance | 0.025 |
| Input Shaper X | EI @ 60.2 Hz |
| Input Shaper Y | MZV @ 23.0 Hz |
| Max Volumetric Flow (PLA 215C) | 10 mm^3/s |
| Probe Threshold | 125g (Z, mesh) / 40g (XY cal) |

*Recalibrate for your own machine - these are starting points.*

## Safety Notes

- **Z+ = bed DOWN (safe), Z- = bed UP (crash risk)**
- Tool offsets MUST be recalibrated for your specific machine
- The XLBuddy requires DFU mode (physical jumper) for firmware flashing
- Dwarfs run stock Prusa firmware - do not modify them
- Prusa XL acceleration hardware limit is 7000 mm/s^2 - never exceed this

## License

Copyright (C) 2026 Richard Crook. All rights reserved.

---

*Built for the Prusa XL community. If this project helps you, give it a star.*
