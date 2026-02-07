===============================================================================
                    XlKlipper Installation Package
                    Prusa XL + Klipper Integration
                         Version: 2026-02-06
===============================================================================

This package contains everything needed to run Klipper on a Prusa XL with
XLBuddy mainboard and 5 Dwarf toolheads.

CONTENTS
--------
  INSTALLATION_GUIDE.md    - Complete step-by-step installation instructions
  README.txt               - This file

  mcu_firmware/            - Files to build Klipper MCU firmware
    modbus_stm32f4.c       - MODBUS master for STM32F407 (copy to klipper/src/stm32/)
    babystep.c             - Babystep support (copy to klipper/src/)
    klipper_config         - Klipper .config for XLBuddy (copy to klipper/.config)

  python_modules/          - Klipper Python extras (copy to klippy/extras/)
    puppy_bootloader.py    - Main module: Dwarf control, tools, heaters, calibration
    loadcell_probe.py      - Loadcell-based Z probing
    pca9557.py             - I2C GPIO expander for Dwarf resets
    dwarf_accelerometer.py - Accelerometer for Input Shaper
    modbus_master.py       - MODBUS communication layer
    babystep.py            - Babystep Z offset support

  config_files/            - Klipper configuration (copy to ~/printer_data/config/)
    printer.cfg            - Main printer configuration
    tool_offsets.cfg       - Tool X/Y/Z offsets (recalibrate for your machine!)
    variables.cfg          - Saved variables
    led_effects.cfg        - Side LED strip effects (optional)
    crowsnest.conf         - Webcam streaming config (optional)
    macros/
      print_macros.cfg     - START_PRINT, END_PRINT, PAUSE, RESUME, CANCEL macros

  post_install/            - Post-installation fixes
    fix_klipper_dirty.sh   - Fix Klipper "dirty" in Moonraker Update Manager
    git_exclude_entries.txt - Git exclude entries reference

  orcaslicer/              - Slicer profiles
    XlKlipper_Profile.ini  - OrcaSlicer profile bundle
    README.md              - Slicer configuration notes

QUICK START
-----------
1. Read INSTALLATION_GUIDE.md completely
2. Set up Raspberry Pi with Klipper (via KIAUH)
3. Build and flash MCU firmware to XLBuddy
4. Copy Python modules to Pi
5. Copy config files to Pi
6. Run fix_klipper_dirty.sh to fix Update Manager
7. Update serial port in printer.cfg
8. Restart Klipper and calibrate

IMPORTANT NOTES
---------------
- Tool offsets MUST be recalibrated for your specific machine
- You need the calibration pin installed for tool calibration
- The XLBuddy requires DFU mode (jumper) for flashing
- Dwarfs run stock Prusa firmware - do not modify them

FEATURES
--------
- Full 5-tool support with automatic tool changes
- Loadcell-based Z homing and bed mesh probing
- Prusa-style tool offset calibration (G425 equivalent)
- Prusa-matched offset application (Marlin-to-Klipper sign correction)
- Multi-color/multi-material printing with slicer-managed temperatures
- Modular bed temperature monitoring (16 bedlets in 4x4 grid)
- Input Shaper via Dwarf accelerometers
- Side LED strip effects (klipper-led_effect plugin)
- USB webcam support via Crowsnest

CHANGES FROM 2026-02-06
------------------------
- Fixed END_PRINT: retract/fan off BEFORE parking (was causing "Tool -1" error)
- Fixed START_PRINT: Turn off T0 heater after probing when printing with other tool
- Added code attribution and copyright (Richard Crook)
- Updated OrcaSlicer docs for Generic Tool Changer profile approach
- All acceleration and speed values Prusa-matched

CHANGES FROM 2026-02-04
------------------------
- CRITICAL: Fixed tool offset sign (negate X,Y,Z for Klipper SET_GCODE_OFFSET)
- Added side LED strip support (led_effects.cfg)
- Added webcam support (crowsnest.conf)
- Added post_install/fix_klipper_dirty.sh script
- Added modbus_master.py and babystep.py to package
- Removed temp transfer fallback (slicer manages temps, matches Prusa)
- Updated all files to current working versions

SUPPORT
-------
See INSTALLATION_GUIDE.md for troubleshooting tips.

===============================================================================
