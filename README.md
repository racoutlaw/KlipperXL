<p align="center">
  <img src="KlipperXL.png" alt="KlipperXL Logo" width="300">
</p>

<h1 align="center">KlipperXL</h1>
<p align="center"><b>Run Klipper firmware on your Prusa XL.</b></p>

KlipperXL replaces the stock Prusa firmware with [Klipper](https://www.klipper3d.org/), giving you full control over your Prusa XL through Mainsail or Fluidd. All 5 Dwarf toolheads work via MODBUS communication with the XLBuddy mainboard.

[![KlipperXL Printing](https://img.youtube.com/vi/xOmtuuXfN5g/0.jpg)](https://www.youtube.com/watch?v=xOmtuuXfN5g)

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
- **Load / Unload Buttons** - `FILAMENT_LOAD` and `FILAMENT_UNLOAD` give Mainsail a button and a parameter form for the Prusa M701 / M702 sequences, which are registered in Python and so are otherwise invisible in the Macros panel
- **Melt Zone Recovery** - `END_PRINT` empties the hot end with a deep retract, which used to leave the *next* print's first perimeters running dry. The retract is now recorded per tool and given back at the next prime line

### Bed & Probing
- **Loadcell Z Probing** - Uses the Dwarf's built-in loadcell for Z homing and bed mesh leveling
- **Adaptive Bed Mesh** - Probes only the print area, not the entire bed
- **Modular Bed Control** - 16-zone (4x4 grid) heated bed with individual bedlet temperature control
- **Adaptive Bed Heating** - Heats only bedlets under the print area with gradient heating on adjacent zones (anti-warp)
- **Adjustable Probe Thresholds** - Change loadcell sensitivity for Z, mesh, and XY calibration from Mainsail
- **Single-Tool Z Datum** - A single-tool job homes Z with the nozzle that is actually printing, instead of heating T0, probing with it, and relying on the stored offset to translate. Driven by the `TOOLS` parameter from the slicer; files sliced before this change fall back to the original multi-tool path

### Motion & Calibration
- **Automatic Per-Tool Pressure Advance** *(new — requires a firmware re-flash)* - Measures PA for **each tool separately** using that Dwarf's own loadcell as a back-pressure sensor, then applies the right value on every tool change. One button, ~6 minutes, no test prints and no eyeballing corners. Tools are not interchangeable in practice — on a 5-tool XL, the same spool measured 0.0367 / 0.0552 / 0.0647 across T0/T1/T2, a 77% spread — so a single shared PA value is wrong for most of your tools. Analysis by [CNC Kitchen](https://github.com/CNCKitchen/PrusaPATuner). See the [Pressure Advance Guide](docs/PRESSURE_ADVANCE.md)
- **Input Shaper** - Resonance compensation via Dwarf accelerometers (per-tool measurement)
- **Z Tilt Alignment** - Prusa-matched mechanical bed leveling
- **Per-Tool Z Offset** - Fine-tune first layer height per tool using Mainsail's built-in Z offset controls, saved to printer.cfg via `[tool_offsets]`
- **LED Stroboscope** - Belt tension / VFA visualization via the side LED strip white channel. TIM13-driven autonomous strobe at 60-130 Hz with configurable duty cycle. Button-friendly macros: `STROBE_BELT` (start) / `STROBE_OFF` (stop)

### Heater Management
- **Multi-Tool Heater Preservation** - Parked tools maintain temperature via periodic MODBUS refresh
- **Independent Tool Heating** - Heat any tool by number (M104 T2 S200) without selecting it
- **Automatic Heater Sync** - Active tool temperature synced to Klipper's extruder for accurate display
- **Tool-Count Independent Shutdown** - `DISABLE_ALL_HOTENDS` turns off the tools that actually booted rather than a hardcoded five, so end-of-print works the same on a 1-tool XL as on a 5-tool
- **Print Fan Follows the Tool** - One logical print fan routed to whichever tool is picked, matching stock. Parking a tool stops its fan instead of leaving it running in the dock

### Interface & Monitoring
- **Side LED Strips** - Status effects for idle, printing, heating, paused, error, and complete states
- **Modular Bed Monitoring** - `BED_STATUS` command shows 4x4 bedlet temperature grid with physical wiring map
- **Bedlet Grid Panel** - Live 4x4 temperature heatmap embedded in Mainsail dashboard via HTML Iframe (color-coded, auto-detect host)
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
| **Flashing** | FAT32 USB drive |
| **Appendix** | **Broken appendix / safety seal on the XLBuddy — this is permanent** |

> ⚠️ **Read this before you start.** The Prusa bootloader verifies firmware
> signatures and will reject unsigned Klipper firmware unless the **appendix**
> (safety seal) on the XLBuddy is broken. Breaking it is a **physical,
> irreversible modification to your mainboard** — it cannot be undone, and it is
> the same requirement as for any custom firmware on the XL.
>
> Everything else here is reversible: you can flash stock Prusa firmware back at
> any time and the printer returns to normal. The appendix is the one step that
> does not come back. Decide about it before you begin, not halfway through.
>
> Details in the [Installation Guide](docs/INSTALLATION_GUIDE.md#1-prerequisites).

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
  src/                       XLBuddy MCU firmware source (MODBUS)
  firmware/                  Build config & recovery files
  orcaslicer/                Slicer profile and configuration guide
  scripts/                   Installation & fix scripts
  pa_analysis/               Pressure Advance analyser (separate venv, see its README)
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
| `DISABLE_ALL_HOTENDS` | Turn off every booted tool heater (used by END_PRINT / CANCEL_PRINT) |
| `FILAMENT_LOAD TOOL= TEMP=` | Load filament into a tool (Prusa M701 sequence) |
| `FILAMENT_UNLOAD TOOL= TEMP=` | Unload filament from a tool (Prusa M702 sequence) |
| `FILAMENT_SENSORS` | Show filament sensor states and calibration for all tools |
| `SPOOL_JOIN TO=` | Remap current tool to a different physical tool |
| `SPOOL_JOIN_STATUS` | Show active spool join remaps |
| `SHOW_THRESHOLDS` | Show probe threshold settings |
| `SET_Z_FAST VALUE=` | Adjust Z fast probe threshold |
| `SET_Z_SLOW VALUE=` | Adjust Z slow probe threshold |
| `SET_MESH VALUE=` | Adjust bed mesh probe threshold |
| `SET_XY VALUE=` | Adjust XY calibration probe threshold |
| `GET_TOOL_Z_OFFSETS` | Display per-tool Z offsets |
| `SET_TOOL_Z_OFFSET TOOL= Z=` | Set Z offset for a tool (in memory) |
| `SAVE_TOOL_Z_OFFSET TOOL= Z=` | Save Z offset for a tool to config |
| `PA_TUNE_T0` - `PA_TUNE_T4` | Auto-tune Pressure Advance for one tool, end to end |
| `GET_TOOL_PA` | Display per-tool Pressure Advance values |
| `SET_TOOL_PA TOOL= K=` | Set a tool's Pressure Advance (in memory) |
| `SAVE_TOOL_PA TOOL= K=` | Save a tool's Pressure Advance to config |
| `CALIBRATE_PA TOOL= TEMP=` | Run a PA sweep only (no analysis, no save) |
| `ANALYZE_PA TOOL= SAVE=1` | Analyse the last PA recording |

## Per-Tool Z Offset

Each tool can have its own Z offset fine-tuning, independent of calibration offsets. This solves the problem where one tool's first layer is perfect but others are too high or too low.

### How It Works

The `[tool_offsets]` section in `printer.cfg` stores a Z offset for each tool:

```ini
[tool_offsets]
t0_z_offset: 0.0350
t1_z_offset: 0.0000
t2_z_offset: -0.0200
t3_z_offset: 0.0000
t4_z_offset: 0.0000
```

These offsets are automatically combined with the calibrated tool offsets during tool changes. Two independent layers:

1. **Calibration offsets** - Set by `CALIBRATE_TOOL_OFFSETS` (stored in `[puppy_bootloader]`)
2. **Per-tool Z offsets** - Fine-tuned per tool for first layer (stored in `[tool_offsets]`)

### Tuning Workflow

1. Run `CALIBRATE_TOOL_OFFSETS` to calibrate all tools
2. Start a first-layer test print with **T0**
3. Use Mainsail's **Z Offset** section (+/- buttons) to adjust until the first layer looks right
4. Click the **Save** button in Mainsail's Z Offset section — this saves the value to T0
5. Click **SAVE_CONFIG** in the top-right to write it to `printer.cfg`
6. Repeat steps 2-5 for each tool (T1, T2, T3, T4)

### Important Notes

- **Do NOT set z_offset in OrcaSlicer** — leave it at 0. All Z offset tuning is done per-tool in Klipper
- Mainsail's Save button automatically saves to whichever tool is currently active
- Safety bounds: -0.3mm to +0.5mm per tool
- These offsets persist across restarts (stored in `printer.cfg`)

## Per-Tool Pressure Advance

Every tool stores and uses **its own** Pressure Advance value. This is not one PA
setting shared by the toolhead group — all five are measured independently and
the right one is applied on each tool change.

> ⚠️ **Already running KlipperXL? This needs a firmware rebuild and re-flash.**
> The measurement depends on MCU-side loadcell streaming, which lives in
> `src/modbus_stm32f4.c` — copying the Python modules alone will capture nothing.
> Step-by-step upgrade instructions are in the
> [CHANGELOG](CHANGELOG.md#upgrading-to-2026-08-16-from-an-earlier-version).

The tools look identical on paper. They do not behave identically. Measured on
one machine, **same spool, same nozzle size, same temperature**:

| Tool | Measured K (PLA @ 215 °C) |
|------|---------------------------|
| T0 | 0.0367 |
| T1 | 0.0552 |
| T2 | 0.0647 |

A 77% spread. Extruder gear wear, PTFE liner friction and heatbreak tolerance all
land on this one knob; on a single-tool printer they disappear into one
calibration, but on a five-tool machine they are five different printers sharing
a frame.

### Tuning Workflow

1. Load filament into the tool
2. Click **`PA_TUNE_T1`** (or set `TEMP` / `FILAMENT` in its dropdown first)
3. Wait ~6 minutes. It homes, heats, sweeps, analyses and applies the result
4. Print a test if you want — the value is already live
5. Run **`SAVE_CONFIG`** to keep it
6. Repeat for each tool

The values land in `[tool_pa]` and are re-applied automatically from then on:

```ini
[tool_pa]
t0_pa: 0.0367
t1_pa: 0.0552
t2_pa: 0.0647
```

### Important Notes

- **Do NOT set pressure advance in your slicer** — leave it out. Klipper applies
  the per-tool value on every tool change and a slicer `SET_PRESSURE_ADVANCE`
  will override it
- PA depends on the **filament**, not only the tool. Re-run it when you change
  material; the `FILAMENT` label records what a value was measured on
- Nothing is saved until you run `SAVE_CONFIG` — deliberate, so you can print
  first and discard a bad result by simply not saving
- A never-tuned tool is left alone, not forced to zero

Full details, including how to get a more precise number: the
[Pressure Advance Guide](docs/PRESSURE_ADVANCE.md).

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
| Pressure Advance | **Per tool** — measured 0.0367 / 0.0552 / 0.0647 on T0 / T1 / T2 (PLA, one spool). Do not copy these; run `PA_TUNE_T<n>` and let the machine measure your own |
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
| [Pressure Advance Guide](docs/PRESSURE_ADVANCE.md) | Automatic per-tool PA tuning |
| [OrcaSlicer Guide](orcaslicer/README.md) | Slicer configuration |
| [Technical Specification](docs/SPECIFICATION.md) | MODBUS protocol, register maps, architecture |
| [Motor Settings](docs/reference/motor_settings.md) | Pin assignments, TMC config, coordinates |
| [Probe Thresholds](docs/reference/probe_thresholds.md) | Loadcell calibration constants |
| [Bed Mesh](docs/reference/bed_mesh.md) | Mesh algorithm, adaptive probing |
| [Recovery](firmware/recovery/RECOVERY_INSTRUCTIONS.md) | DFU recovery for bricked XLBuddy |

## Mid-Print Safety Guards

Calibration and test routines refuse to run while a print is in progress. Pressing one
from Mainsail or the display during a job now returns an error instead of starting.

This matters because several of them home or probe. `PA_TUNE` homes Z **before** picking
the tool - `homing_override` couples T0 for the loadcell probe - so mid-print it drives
to bed centre and probes down onto the part, re-datuming Z off the top of the print. The
job then carries on at a completely wrong Z. `LOADCELL_TEST` is a bare passthrough to
`LOADCELL_PROBE`, which with no arguments probes straight down from wherever the nozzle
is standing.

Guarded commands:

| Command | Why it is dangerous mid-print |
|---|---|
| `CALIBRATE_PA`, `CALIBRATE_PA_FINE` | homes Z, picks a tool, heats and extrudes |
| `PA_TUNE_T0`..`PA_TUNE_T4` | wrappers around the above |
| `CALIBRATE_INPUT_SHAPER` | homes, then shakes the gantry |
| `TEST_RESONANCES_X` / `_Y` | homes, then shakes the gantry |
| `CALIBRATE_TOOLS` | tool offset calibration - homes and probes |
| `DOCK_CAL` | dock calibration - moves into the dock row |
| `LOADCELL_TEST` | probes straight down from the current position |
| `PRUSA_Z_LEVEL`, `STALLGUARD_Z_ALIGN` | rams Z into the endstops |
| `PARK_TOOL` | ejects the toolhead you are printing with |

`paused` is refused as well as `printing` - pausing does not take the part off the bed.

### If you add your own macros

Call `_ASSERT_NOT_PRINTING WHAT=<YOUR_MACRO>` as the **first line** of anything that
homes, probes, calibrates or swaps tools:

```
[gcode_macro MY_CALIBRATION]
gcode:
    _ASSERT_NOT_PRINTING WHAT=MY_CALIBRATION
    ...
```

Two rules worth copying:

- **Guard the command, not the button.** Guarding only the menu wrapper leaves the
  underlying command reachable, and one direct call walks straight past it
- **Do not guard things a print legitimately needs.** `START_PRINT` homes *and* probes
  while `print_stats` already reads `printing`, and `BED_MESH_CALIBRATE` calls the
  loadcell probe during it. `START_PRINT`, `END_PRINT`, `CANCEL_PRINT`, `PAUSE`,
  `RESUME` and the Mainsail/timelapse client macros are deliberately unguarded

### Fan faults no longer cancel a print

A failed MODBUS write to a Dwarf print fan used to raise, which aborts the running job.
Stock Prusa cannot do this - `hwio_XLBuddy.cpp` discards the return value of
`Fans::print(active_extruder).set_pwm()`, as does every caller of
`CFanCtl3Wire::set_pwm()`.

A bare `M106` - the only form a slicer emits - now warns and keeps printing. An explicit
`M106 T<n>` still raises, because a wrong tool number there is an authoring error worth
seeing. Warnings are edge-triggered: one line when the bus fails, one when it recovers.

## Safety Notes

- **Z+ = bed DOWN (safe), Z- = bed UP (crash risk)**
- Tool offsets MUST be recalibrated for your specific machine
- **Breaking the XLBuddy appendix is permanent and cannot be undone.** It is required to flash unsigned firmware. Everything else about KlipperXL is reversible — stock Prusa firmware flashes back and the printer returns to normal — but this does not
- **Normal flashing is from a USB drive** using Prusa's own bootloader — no jumper, no opening the printer. Build at offset `0x08020200` so the bootloader is preserved
- **DFU is an advanced recovery method only.** It requires opening the printer and installing the **BOOT0 jumper** on the XLBuddy, and it flashes at `0x08000000` — which overwrites the Prusa bootloader, so USB `.bbf` flashing stops working until you put the bootloader back. That is recoverable: see [Section 12.5](docs/INSTALLATION_GUIDE.md#125-restoring-the-prusa-bootloader), which reflashes `firmware/recovery/bootloader-xl-2.5.0.bin`. Use DFU only if you have no working bootloader or no broken appendix seal
- **Calibration and test macros refuse to run mid-print.** See [Mid-Print Safety Guards](#mid-print-safety-guards). If you add your own, call `_ASSERT_NOT_PRINTING WHAT=<NAME>` first
- Dwarfs run stock Prusa firmware - do not modify them
- Prusa XL hardware acceleration limit is 7000 mm/s² - never exceed this
- Modular bed has a 120C maximum target temperature enforced in software

## Disclaimer

THIS SOFTWARE IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND. BY USING THIS SOFTWARE YOU ACKNOWLEDGE THAT YOU ARE MODIFYING YOUR PRINTER'S FIRMWARE AND ACCEPT ALL RISKS ASSOCIATED WITH DOING SO. THE AUTHOR IS NOT RESPONSIBLE FOR ANY DAMAGE TO YOUR PRINTER, PROPERTY, OR PERSON, INCLUDING BUT NOT LIMITED TO HARDWARE DAMAGE, FIRE, INJURY, OR LOSS OF ANY KIND. THIS PROJECT REPLACES STOCK FIRMWARE AND MAY VOID YOUR WARRANTY. USE AT YOUR OWN RISK. YOU ASSUME FULL RESPONSIBILITY FOR ANY AND ALL CONSEQUENCES OF INSTALLING AND USING THIS SOFTWARE.

## License

Copyright (C) 2026 Richard Crook

This program is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.

This program is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the [GNU General Public License](LICENSE) for more details.

### Third-party components

`pa_analysis/prusa_pa_tuner/` is **not** KlipperXL code. It is the Pressure
Advance analysis library from
[PrusaPATuner](https://github.com/CNCKitchen/PrusaPATuner) by **Stefan Hermann
(CNC Kitchen)**, included unmodified under the **GNU Affero General Public
License v3.0 or later**. It remains AGPL-licensed wherever it is copied.

It is bundled rather than downloaded during installation so that KlipperXL stays
buildable if the upstream repository ever becomes unavailable. It runs as a
separate subprocess and is never imported into the Klipper process.

See [`pa_analysis/prusa_pa_tuner/NOTICE`](pa_analysis/prusa_pa_tuner/NOTICE) for
full attribution and licence details.

## Acknowledgements

- **Stefan Hermann / [CNC Kitchen](https://www.youtube.com/@CNCKitchen)** — the
  loadcell Pressure Advance method and the analysis mathematics behind
  KlipperXL's automatic PA tuning

---

*Built for the Prusa XL community. If this project helps you, give it a star.*
