# KlipperXL Installation Guide

Complete step-by-step guide to install KlipperXL on a Prusa XL with XLBuddy board.

**Last Updated:** 2026-02-09
**Tested Configuration:** Prusa XL 5-tool with XLBuddy mainboard

---

## Table of Contents

1. [Prerequisites](#1-prerequisites)
2. [Raspberry Pi Setup](#2-raspberry-pi-setup)
3. [Extract KlipperXL Package](#3-extract-xlklipper-package)
4. [Build Klipper MCU Firmware](#4-build-klipper-mcu-firmware)
5. [Flash XLBuddy MCU](#5-flash-xlbuddy-mcu)
6. [Deploy Python Modules](#6-deploy-python-modules)
7. [Deploy Configuration Files](#7-deploy-configuration-files)
8. [Fix Klipper Update Manager (Dirty State)](#8-fix-klipper-update-manager-dirty-state)
9. [Optional: LED Strips and Webcam](#9-optional-led-strips-and-webcam)
10. [Configure OrcaSlicer](#10-configure-orcaslicer)
11. [First Boot and Calibration](#11-first-boot-and-calibration)
12. [Troubleshooting](#12-troubleshooting)

---

## 1. Prerequisites

### Hardware Required
- Prusa XL with XLBuddy mainboard (stock, unmodified)
- Raspberry Pi 4 (2GB+ RAM recommended)
- MicroSD card (16GB+)
- USB cable (USB-A to USB-C for XLBuddy connection)
- Ethernet or WiFi connection for Pi
- Computer with WSL/Linux for building firmware

### Software Required (on your build computer)
- Git
- ARM GCC toolchain (`arm-none-eabi-gcc`)
- Python 3.8+
- SSH client

### Knowledge Required
- Basic Linux command line
- SSH access to Raspberry Pi
- Understanding of 3D printer coordinates

---

## 2. Raspberry Pi Setup

### 2.1 Install Raspberry Pi OS

1. Download Raspberry Pi Imager
2. Flash **Raspberry Pi OS Lite (64-bit)** to SD card
3. Configure:
   - Hostname: `klipper-xl`
   - Username: `pi`
   - Password: (your choice)
   - WiFi (optional)
   - Enable SSH

### 2.2 Install KIAUH (Klipper Installation And Update Helper)

SSH into your Pi and run:

```bash
sudo apt update && sudo apt upgrade -y
sudo apt install git -y

cd ~
git clone https://github.com/dw-0/kiauh.git
./kiauh/kiauh.sh
```

### 2.3 Install Klipper Components via KIAUH

From the KIAUH menu, install:
1. **Klipper** (option 1)
2. **Moonraker** (option 2)
3. **Mainsail** (option 3)

After installation, verify Klipper service:
```bash
sudo systemctl status klipper
```

### 2.4 Note Your Pi's IP Address

```bash
hostname -I
```

Example: `<YOUR_PI_IP>`

---

## 3. Extract KlipperXL Package

On your **build computer** (WSL/Linux), not the Pi:

### 3.1 Extract the Package

```bash
mkdir -p ~/projects
cd ~/projects
tar -xzf KlipperXL_Install_Package_20260206.tar.gz
cd KlipperXL_Install_Package
```

You should see these folders:
- `mcu_firmware/` - MCU source files and build config
- `python_modules/` - Klipper Python extras
- `config_files/` - Printer configuration
- `post_install/` - Post-installation fixes
- `orcaslicer/` - Slicer profiles

### 3.2 Clone Klipper Source (needed for building firmware)

```bash
cd ~
git clone https://github.com/Klipper3d/klipper.git
cd klipper
```

---

## 4. Build Klipper MCU Firmware

### 4.1 Copy Custom Modules to Klipper Source

```bash
# From the KlipperXL_Install_Package directory
cp mcu_firmware/modbus_stm32f4.c ~/klipper/src/stm32/
```

### 4.2 Update Makefiles

You need to add the custom source files to the build system:

**Edit `~/klipper/src/stm32/Makefile`** - Add `modbus_stm32f4.c` to the source list:
```
src-y += stm32/modbus_stm32f4.c
```

### 4.3 Copy Klipper Build Configuration

```bash
cp mcu_firmware/klipper_config ~/klipper/.config
```

The config targets STM32F407 (XLBuddy), 12MHz crystal, no bootloader, USB serial.

### 4.4 Build the Firmware

```bash
cd ~/klipper
make clean
make -j4
```

This creates `~/klipper/out/klipper.bin`

---

## 5. Flash XLBuddy MCU

### 5.1 Enter DFU Mode

**IMPORTANT: This requires physical access to the XLBuddy board!**

1. **Power OFF** the printer completely
2. Locate the **BOOT0 jumper** on the XLBuddy board
3. **Install the jumper** to enable DFU mode
4. **Power ON** the printer
5. The XLBuddy is now in DFU mode (no display activity is normal)

### 5.2 Flash via DFU

SSH to your Pi and run:

```bash
# Install dfu-util if not present
sudo apt install dfu-util -y

# Verify DFU device is detected
sudo dfu-util -l
```

You should see a device with `Internal Flash` listed.

Copy the firmware to the Pi:
```bash
# From your build computer
scp ~/klipper/out/klipper.bin pi@<PI_IP>:/tmp/klipper.bin
```

Flash the firmware:
```bash
# On the Pi
sudo dfu-util -a 0 -s 0x08000000:leave -D /tmp/klipper.bin
```

### 5.3 Exit DFU Mode

1. **Power OFF** the printer
2. **Remove the BOOT0 jumper**
3. **Power ON** the printer
4. The XLBuddy should now appear as a USB serial device

Verify:
```bash
ls /dev/serial/by-id/
```

You should see something like: `usb-Klipper_stm32f407xx_...`

---

## 6. Deploy Python Modules

Copy all Python extras to the Pi's Klipper installation:

```bash
# From the KlipperXL_Install_Package directory
PI_IP="<PI_IP>"  # Change to your Pi's IP

scp python_modules/puppy_bootloader.py pi@${PI_IP}:/home/pi/klipper/klippy/extras/
scp python_modules/loadcell_probe.py pi@${PI_IP}:/home/pi/klipper/klippy/extras/
scp python_modules/pca9557.py pi@${PI_IP}:/home/pi/klipper/klippy/extras/
scp python_modules/dwarf_accelerometer.py pi@${PI_IP}:/home/pi/klipper/klippy/extras/
scp python_modules/modbus_master.py pi@${PI_IP}:/home/pi/klipper/klippy/extras/
```

---

## 7. Deploy Configuration Files

### 7.1 Copy Configuration Files

```bash
# From the KlipperXL_Install_Package directory
PI_IP="<PI_IP>"  # Change to your Pi's IP

# Main printer.cfg
scp config_files/printer.cfg pi@${PI_IP}:~/printer_data/config/printer.cfg

# Tool offsets (will need recalibration for your machine)
scp config_files/tool_offsets.cfg pi@${PI_IP}:~/printer_data/config/tool_offsets.cfg

# Variables file
scp config_files/variables.cfg pi@${PI_IP}:~/printer_data/config/variables.cfg

# Create macros directory and copy macros
ssh pi@${PI_IP} "mkdir -p ~/printer_data/config/macros"
scp config_files/macros/print_macros.cfg pi@${PI_IP}:~/printer_data/config/macros/print_macros.cfg
```

### 7.2 Update Serial Port in printer.cfg

SSH to your Pi and edit printer.cfg:

```bash
ssh pi@${PI_IP}
nano ~/printer_data/config/printer.cfg
```

Find the `[mcu]` section and update the serial port:

```ini
[mcu]
serial: /dev/serial/by-id/usb-Klipper_stm32f407xx_XXXXXXXXX-if00
```

Replace `XXXXXXXXX` with your actual device ID from `ls /dev/serial/by-id/`

### 7.3 Restart Klipper

```bash
sudo systemctl restart klipper
```

---

## 8. Fix Klipper Update Manager (Dirty State)

### The Problem

After deploying KlipperXL files, Moonraker's Update Manager will show Klipper as **"dirty"** because we added custom files to the Klipper git repo. This prevents applying Klipper updates through Mainsail.

### The Fix

We provide an automated script that fixes this. SSH to your Pi and run:

```bash
# Copy the script to your Pi first (from your build computer)
scp post_install/fix_klipper_dirty.sh pi@${PI_IP}:/tmp/

# SSH to Pi and run it
ssh pi@${PI_IP}
bash /tmp/fix_klipper_dirty.sh
```

The script does three things:

#### 1. Hide custom (untracked) files from git

Adds our custom files to `/home/pi/klipper/.git/info/exclude` (a local-only gitignore that is never committed):

```
klippy/extras/puppy_bootloader.py
klippy/extras/dwarf_accelerometer.py
klippy/extras/loadcell_probe.py
klippy/extras/modbus_master.py
klippy/extras/pca9557.py
src/stm32/modbus_stm32f4.c
.config_xlbuddy
```

#### 2. Hide modifications to tracked files

Our Makefile edits (adding modbus_stm32f4.c to the build) modify tracked files. We mark them as "assume-unchanged" so git ignores the modifications:

```bash
cd /home/pi/klipper
git update-index --assume-unchanged src/stm32/Makefile
```

#### 3. Clear Moonraker's cached dirty state

Moonraker caches the repo state in its SQLite database. Even after fixing git, Moonraker may still show "dirty" from its cache. The script clears this:

```bash
sqlite3 /home/pi/printer_data/database/moonraker-sql.db \
  "DELETE FROM namespace_store WHERE namespace='update_manager' AND key='klipper';"
```

### After Running the Script

Restart Moonraker for changes to take effect:

```bash
sudo systemctl restart moonraker
```

Then refresh the Update Manager in Mainsail - Klipper should now show as clean and updates can be applied normally.

### Important: After Klipper Updates

When you apply a Klipper update through Moonraker, git may reset the `assume-unchanged` flags on the Makefiles. If Klipper shows as "dirty" again after an update, just re-run the script:

```bash
bash /tmp/fix_klipper_dirty.sh
sudo systemctl restart moonraker
```

### Manual Fix (if you prefer not to use the script)

If you want to do it by hand, here are the steps:

```bash
# 1. Edit the exclude file
nano /home/pi/klipper/.git/info/exclude
# Add the entries from post_install/git_exclude_entries.txt

# 2. Mark Makefiles as unchanged
cd /home/pi/klipper
git update-index --assume-unchanged src/stm32/Makefile

# 3. Clear Moonraker cache
sqlite3 ~/printer_data/database/moonraker-sql.db \
  "DELETE FROM namespace_store WHERE namespace='update_manager' AND key='klipper';"

# 4. Restart Moonraker
sudo systemctl restart moonraker
```

---

## 9. Optional: LED Strips and Webcam

### 9.1 Side LED Strips (klipper-led_effect)

If your Prusa XL has side LED strips connected to PG14 (SPI6 MOSI):

**Install the klipper-led_effect plugin:**
```bash
ssh pi@${PI_IP}
cd ~
git clone https://github.com/julianschill/klipper-led_effect.git
ln -s ~/klipper-led_effect/src/led_effect.py ~/klipper/klippy/extras/led_effect.py
```

**Deploy the LED effects config:**
```bash
# From your build computer
scp config_files/led_effects.cfg pi@${PI_IP}:~/printer_data/config/led_effects.cfg
```

The LED configuration is already included in the printer.cfg (`[neopixel side_leds]` section).
Status macros are wired into START_PRINT, END_PRINT, PAUSE, RESUME, and CANCEL_PRINT.

**LED Notes:**
- Color order is RGB (not GRB despite WS2812 LEDs)
- Chain count is 2 (driver 1 = RGB, driver 2 = White channel)
- Mux select pin PE9 is held HIGH via `[output_pin led_mux_select]`

### 9.2 USB Webcam (Crowsnest)

**Install Crowsnest:**
```bash
ssh pi@${PI_IP}
cd ~
git clone https://github.com/mainsail-crew/crowsnest.git
cd crowsnest
make install
```

**Deploy the webcam config:**
```bash
# From your build computer
scp config_files/crowsnest.conf pi@${PI_IP}:~/printer_data/config/crowsnest.conf
```

**Start the service:**
```bash
sudo systemctl restart crowsnest
```

The default config is set for 1920x1080 at 30fps using the ustreamer backend. Edit `crowsnest.conf` if your camera needs different settings.

---

## 10. Configure OrcaSlicer

Use the built-in **Generic Tool Changer** profile as your base. It already has
multi-tool settings (wipe tower, purge volumes, tool change handling). You only
need to make the changes below.

### 10.1 Select Printer Profile

1. Open OrcaSlicer
2. Go to **Printer** tab > **Add Printer** or select from dropdown
3. Choose **Generic Tool Changer** (5-tool variant)

### 10.2 Change These 4 Settings

#### 1. G-code Flavor

Set to: **Klipper**

#### 2. Start G-code (REPLACE ENTIRE BLOCK)

Delete everything in the start G-code box and replace with this single line:

```gcode
START_PRINT EXTRUDER_TEMP=[nozzle_temperature_initial_layer] BED_TEMP=[bed_temperature_initial_layer_single] TOTAL_LAYER_COUNT={total_layer_count} X0={first_layer_print_min[0]} Y0={first_layer_print_min[1]} X1={first_layer_print_max[0]} Y1={first_layer_print_max[1]}
```

**IMPORTANT:** Replace the ENTIRE start gcode. The generic profile has per-tool
purge sequences that will cause "Extrude below minimum temp" errors if left in.
Our START_PRINT macro handles homing, heating, probing, and adaptive bed mesh.

**DO NOT** add `BED_MESH_PROFILE LOAD=default` - START_PRINT does adaptive mesh.

#### 3. End G-code (REPLACE ENTIRE BLOCK)

```gcode
END_PRINT
```

#### 4. Acceleration Values (Prusa XL Matched)

The generic profile has high acceleration values (up to 10000) that are not
safe for Prusa XL hardware. Change to these Prusa-matched values:

| Setting | Value |
|---------|-------|
| Default acceleration | **1250** |
| Perimeter acceleration | **1000** |
| Infill acceleration | **2000** |
| Solid infill acceleration | **1500** |
| Top solid infill acceleration | **800** |
| Bridge acceleration | **1000** |
| Travel acceleration | **5000** |
| First layer acceleration | **800** |
| Machine max accel X/Y | **5000** |

Prusa XL hardware limit is 7000. Never exceed this.

### 10.3 Additional Settings

#### Pause G-code
```gcode
PAUSE
```

#### Before Layer Change G-code
```gcode
;BEFORE_LAYER_CHANGE
;[layer_z]
G92 E0
_ON_LAYER_CHANGE LAYER={layer_num + 1}
```

#### After Layer Change G-code
```gcode
;AFTER_LAYER_CHANGE
;[layer_z]
```

#### Between Objects G-code
```gcode
;BETWEEN_OBJECTS
G92 E0
```

#### Print Settings (Process Tab)

| Setting | Location | Value |
|---------|----------|-------|
| Label objects | Others | **Firmware** (enables cancel single object) |
| Arc fitting | Others | **Emit center** (enables G2/G3 arcs) |

#### Wipe Tower (verify these match)

| Setting | Value |
|---------|-------|
| Enable prime tower | **ON** |
| Prime tower width | **60** |
| Prime tower brim width | **3** |
| Wall type | **Cone** |
| Stabilization cone apex angle | **25** |
| Enable ooze prevention | **ON** |
| Standby temperature delta | **-110** |

### 10.4 Save as New Profile

Save as a new printer profile (e.g., "KlipperXL 5-Tool") so your changes
don't get overwritten by OrcaSlicer updates.

### 10.5 Alternative: Import Pre-configured Profile

If you prefer importing a complete profile bundle instead of modifying
the generic tool changer profile:

1. Go to **File > Import > Import Config Bundle**
2. Select `KlipperXL_Install_Package/orcaslicer/KlipperXL_Profile.ini`
3. Profiles appear in your printer/print/filament dropdowns

---

## 11. First Boot and Calibration

### 11.1 Initial Startup

1. Open Mainsail in browser: `http://<PI_IP>`
2. You should see the printer interface
3. Check for any errors in the console

### 11.2 Verify Dwarf Communication

In Mainsail console, run:
```gcode
DWARF_STATUS
```

You should see temperatures for all 5 Dwarfs (D1-D5).

### 11.3 Home the Printer

```gcode
G28 X Y    ; Home X and Y (sensorless)
T0         ; Pick tool 0
G28 Z      ; Home Z with loadcell
```

### 11.4 Z Tilt Correction (Level the Bed)

```gcode
PRUSA_Z_LEVEL
```

**Note:** This is LOUD - the bed crashes into the top frame. This is normal!

### 11.5 Calibrate Tool Offsets

**IMPORTANT: You need the calibration pin installed at bed center (X=180, Y=180)**

```gcode
CALIBRATE_TOOL_OFFSETS
```

This will:
1. Pick each tool (T0-T4)
2. Probe the calibration pin from multiple angles
3. Calculate and save X, Y, Z offsets for each tool

**Note on tool offsets:** Offsets are stored in Prusa/Marlin convention but automatically negated when applied via Klipper's SET_GCODE_OFFSET. This is handled internally - you don't need to worry about sign conventions.

### 11.6 Test Each Tool

```gcode
T0    ; Pick tool 0
T1    ; Pick tool 1 (parks T0, picks T1)
T2    ; Pick tool 2
T3    ; Pick tool 3
T4    ; Pick tool 4
T0    ; Back to T0
```

Verify each tool picks and parks cleanly.

### 11.7 Heat Test

```gcode
T0
M104 T0 S200    ; Heat T0 to 200C
M104 T1 S200    ; Heat T1 to 200C
M109 T0 S200    ; Wait for T0
```

Verify temperatures in Mainsail match expectations.

---

## 12. Troubleshooting

### "MCU 'mcu' shutdown: Timer too close"
- The MCU firmware wasn't flashed correctly
- Re-flash via DFU mode

### "Unable to connect to MCU"
- Check USB cable connection
- Verify serial port in printer.cfg matches `ls /dev/serial/by-id/`
- Verify DFU jumper is REMOVED

### Dwarfs not responding / "Dwarf X not booted"
- Power cycle the printer completely (full off, wait 10 seconds, on)
- The Dwarf boot sequence takes ~30 seconds after power on

### "Extrude below minimum temp"
- This can happen if switching to a cold tool without the slicer setting its temperature first
- The slicer (OrcaSlicer) manages all tool temperatures via M104 commands
- For manual tool changes, set the temperature first: `M104 T2 S200` then `T2`

### Tool won't pick / "Not picked after wiggle"
- Verify Y position_endstop is -9 (not -8) in printer.cfg
- Check dock positions match Prusa defaults (DOCK_Y=455)
- Try parking and re-homing, then picking again

### Homing fails / crashes
- Verify motor pins and directions in printer.cfg
- X homes to LEFT (X=-8)
- Y homes to FRONT (Y=-9)

### "verify_heater" errors
- Normal during tool changes - the code resets verify_heater state automatically
- If persistent, check Dwarf heater connections

### Klipper shows "dirty" in Update Manager
- Run the fix script: `bash /tmp/fix_klipper_dirty.sh`
- Restart Moonraker: `sudo systemctl restart moonraker`
- See [Section 8](#8-fix-klipper-update-manager-dirty-state) for details

### Tool offsets seem wrong / first layer too high or low
- Re-run `CALIBRATE_TOOL_OFFSETS` with calibration pin installed
- The offset sign is automatically handled (negated for Klipper)
- Make sure nozzles are clean before calibration

---

## File Summary

### Files on Pi (`/home/pi/klipper/klippy/extras/`)
| File | Purpose |
|------|---------|
| `puppy_bootloader.py` | Main Dwarf control, tool changes, heaters, calibration |
| `loadcell_probe.py` | Loadcell-based Z probing |
| `pca9557.py` | I2C GPIO expander for Dwarf reset lines |
| `dwarf_accelerometer.py` | Accelerometer for Input Shaper |
| `modbus_master.py` | MODBUS communication layer |

### Files on Pi (`/home/pi/printer_data/config/`)
| File | Purpose |
|------|---------|
| `printer.cfg` | Main Klipper configuration |
| `tool_offsets.cfg` | Calibrated tool X/Y/Z offsets |
| `variables.cfg` | Saved variables (probe thresholds, etc.) |
| `macros/print_macros.cfg` | START_PRINT, END_PRINT, PAUSE, RESUME, CANCEL macros |
| `led_effects.cfg` | Side LED strip effects (optional) |
| `crowsnest.conf` | Webcam streaming config (optional) |

### Files Flashed to XLBuddy
| File | Purpose |
|------|---------|
| `klipper.bin` | Klipper MCU firmware with MODBUS support |

---

## Quick Reference Commands

### Tool & Motion
| Command | Description |
|---------|-------------|
| `G28 X Y` | Home X/Y (sensorless) |
| `G28 Z` | Home Z (loadcell, needs tool picked) |
| `T0`-`T4` | Select tool (automatic park/pick) |
| `TOOL_PICK T=n` | Pick specific tool |
| `TOOL_PARK` | Park current tool |
| `PRUSA_Z_LEVEL` | Crash-based Z alignment |
| `CALIBRATE_TOOL_OFFSETS` | Full tool offset calibration |
| `SHOW_TOOL_OFFSETS` | Show calibrated tool offsets |

### Heaters & Bed
| Command | Description |
|---------|-------------|
| `M104 Tn Sxxx` | Set tool n temperature |
| `SET_DWARF_TEMP DWARF=n TEMP=x` | Set individual tool temperature |
| `DWARF_STATUS` | Show all Dwarf temperatures |
| `BED_STATUS` | Show 4x4 bedlet temperature grid |
| `SET_BED_AREA X0= Y0= X1= Y1=` | Set adaptive bed heating zone |
| `CLEAR_BED_AREA` | Reset to full bed heating |

### Filament & Spool Join
| Command | Description |
|---------|-------------|
| `SPOOL_JOIN TO=n` | Remap current tool to physical tool n |
| `SPOOL_JOIN_STATUS` | Show active spool join remaps |
| `SPOOL_JOIN_RESET` | Clear all spool join remaps |

### Probe Thresholds
| Command | Description |
|---------|-------------|
| `SHOW_THRESHOLDS` | Show current probe threshold settings |
| `SET_Z_FAST VALUE=x` | Set Z fast probe threshold (default: 125g) |
| `SET_Z_SLOW VALUE=x` | Set Z slow probe threshold (default: 125g) |
| `SET_MESH VALUE=x` | Set bed mesh probe threshold (default: 125g) |
| `SET_XY VALUE=x` | Set XY calibration threshold (default: 40g) |

---

*Guide updated 2026-02-09 for KlipperXL project*
