#!/bin/bash
# KlipperXL - Installation Script for Prusa XL
# Copyright (C) 2026 Richard Crook
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# Run this on your Raspberry Pi after installing Klipper via KIAUH
# and cloning the KlipperXL repo to ~/KlipperXL
#
# Usage: bash ~/KlipperXL/scripts/INSTALL_ON_PI.sh
#
# Prerequisites are checked first, then:
# 1. Copy MODBUS module to Klipper source
# 2. Copy Python extras to Klipper
# 3. Deploy config files
# 4. Set up the Pressure Advance analyser in its own venv
# 5. Launch make menuconfig for build setup
# 6. Build firmware (Makefiles patched only for the build, then restored)

set -e

echo "=========================================="
echo " KlipperXL Installation Script"
echo " for Prusa XL with XLBuddy"
echo "=========================================="
echo ""

# Prerequisites check - Klipper must be installed via KIAUH first
MISSING=""

if [ ! -d "$HOME/klipper" ]; then
    MISSING="$MISSING\n  - ~/klipper (Klipper repo not found)"
fi

if [ ! -d "$HOME/klippy-env" ]; then
    MISSING="$MISSING\n  - ~/klippy-env (Klipper Python environment not found)"
fi

if ! command -v cpp &>/dev/null; then
    MISSING="$MISSING\n  - cpp (C preprocessor not found — install build-essential)"
fi

if ! command -v arm-none-eabi-gcc &>/dev/null; then
    MISSING="$MISSING\n  - arm-none-eabi-gcc (ARM build toolchain not found)"
fi

if [ ! -f "$HOME/printer_data/config/printer.cfg" ]; then
    MISSING="$MISSING\n  - ~/printer_data/config/printer.cfg (Klipper not configured)"
fi

if [ -n "$MISSING" ]; then
    echo "ERROR: Klipper is not fully installed. Missing:"
    echo -e "$MISSING"
    echo ""
    echo "Install Klipper first using KIAUH:"
    echo "  git clone https://github.com/dw-0/kiauh.git"
    echo "  ./kiauh/kiauh.sh"
    echo ""
    echo "Install Klipper, Moonraker, and Mainsail/Fluidd, then re-run this script."
    echo ""
    echo "For build tools, run:"
    echo "  sudo apt install build-essential gcc-arm-none-eabi"
    exit 1
fi

echo "Klipper installation found, proceeding..."
echo ""

# Determine where our module files are
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
MODULE_DIR="$(dirname "$SCRIPT_DIR")"

if [ ! -f "$MODULE_DIR/src/modbus_stm32f4.c" ]; then
    echo "ERROR: Cannot find src/modbus_stm32f4.c in $MODULE_DIR"
    echo "Make sure KlipperXL is cloned to ~/KlipperXL"
    exit 1
fi

# Step 1: Copy MODBUS MCU module
echo "[1/6] Copying MODBUS module to Klipper source..."
cp "$MODULE_DIR/src/modbus_stm32f4.c" ~/klipper/src/
cp "$MODULE_DIR/src/neopixel_spi.c" ~/klipper/src/
echo "  Done."

# Step 2: Copy Python extras
echo ""
echo "[2/6] Copying Python extras to Klipper..."
cp "$MODULE_DIR/klippy/modbus_master.py" ~/klipper/klippy/extras/
cp "$MODULE_DIR/klippy/puppy_bootloader.py" ~/klipper/klippy/extras/
cp "$MODULE_DIR/klippy/loadcell_probe.py" ~/klipper/klippy/extras/
cp "$MODULE_DIR/klippy/pca9557.py" ~/klipper/klippy/extras/
cp "$MODULE_DIR/klippy/dwarf_accelerometer.py" ~/klipper/klippy/extras/
cp "$MODULE_DIR/klippy/tool_offsets.py" ~/klipper/klippy/extras/
cp "$MODULE_DIR/klippy/strobe_test.py" ~/klipper/klippy/extras/
cp "$MODULE_DIR/klippy/dwarf_pa_tuner.py" ~/klipper/klippy/extras/
cp "$MODULE_DIR/klippy/tool_pa.py" ~/klipper/klippy/extras/
~/klippy-env/bin/pip install numpy -q 2>/dev/null
echo "  Done (numpy installed for input shaper)."

# Step 3: Deploy config files
echo ""
echo "[3/6] Deploying configuration files..."
cp "$MODULE_DIR/config/printer.cfg" ~/printer_data/config/printer.cfg
cp "$MODULE_DIR/config/tool_offsets.cfg" ~/printer_data/config/tool_offsets.cfg
cp "$MODULE_DIR/config/variables.cfg" ~/printer_data/config/variables.cfg
cp "$MODULE_DIR/config/pa_tuner.cfg" ~/printer_data/config/pa_tuner.cfg
cp "$MODULE_DIR/config/pa_menu.cfg" ~/printer_data/config/pa_menu.cfg
mkdir -p ~/printer_data/config/macros
cp "$MODULE_DIR/config/macros/print_macros.cfg" ~/printer_data/config/macros/print_macros.cfg
echo "  Done."
echo ""
echo "  NOTE: Edit ~/printer_data/config/printer.cfg if you use Fluidd -"
echo "    change [include mainsail.cfg] to [include fluidd.cfg]"
echo ""
echo "  The LED-effects and timelapse includes ship COMMENTED OUT. Both depend on"
echo "  software this script does not install, and Klipper will not start on an"
echo "  include it cannot find. Turn each on only after its setup step:"
echo "    - led_effects.cfg  needs the klipper-led_effect plugin  (guide 8.1)"
echo "    - timelapse.cfg    needs moonraker-timelapse            (guide 8.3)"

# Step 4: Pressure Advance analyser, in its OWN venv
echo ""
echo "[4/6] Setting up the Pressure Advance analyser..."
echo ""
echo "  This goes in its own virtualenv, NOT klippy-env. scipy is a heavy"
echo "  scientific stack and must never be able to disturb the environment"
echo "  that drives the printer."

# PA is an optional extra. Nothing in this step may abort the install -- the
# firmware build below is the part people actually came for, and `set -e` would
# otherwise kill the whole script on a missing python3-venv (not present on
# Raspberry Pi OS Lite by default).
PA_OK=1

if [ -d "$HOME/pa_analysis" ]; then
    rm -rf "$HOME/pa_analysis.old"
    mv "$HOME/pa_analysis" "$HOME/pa_analysis.old"
    echo "  Existing ~/pa_analysis moved aside to ~/pa_analysis.old"
fi
cp -r "$MODULE_DIR/pa_analysis" ~/pa_analysis

if [ ! -x "$HOME/pa_analysis_env/bin/python" ]; then
    if ! python3 -m venv ~/pa_analysis_env 2>/dev/null; then
        PA_OK=0
        echo ""
        echo "  WARNING: could not create the virtualenv. Usually missing python3-venv:"
        echo "    sudo apt install -y python3-venv"
    fi
fi

if [ "$PA_OK" = "1" ]; then
    echo "  Installing numpy + scipy (this can take several minutes on a Pi)..."
    if ~/pa_analysis_env/bin/pip install -q -r ~/pa_analysis/requirements.txt; then
        ~/pa_analysis_env/bin/python -c \
            "import numpy,scipy;print('  OK - numpy',numpy.__version__,'/ scipy',scipy.__version__)"
    else
        PA_OK=0
        echo ""
        echo "  WARNING: analyser dependencies failed to install."
    fi
fi

if [ "$PA_OK" = "0" ]; then
    echo ""
    echo "  Automatic PA tuning will NOT work until you finish this. Everything"
    echo "  else is unaffected. To retry later:"
    echo "    sudo apt install -y python3-venv"
    echo "    python3 -m venv ~/pa_analysis_env"
    echo "    ~/pa_analysis_env/bin/pip install -r ~/pa_analysis/requirements.txt"
    echo ""
    echo "  Until then, comment these out in ~/printer_data/config/printer.cfg or"
    echo "  Klipper will refuse to start:"
    echo "    #[include pa_tuner.cfg]"
    echo "    #[include pa_menu.cfg]"
    echo "    #[dwarf_pa_tuner]"
    echo "    #[tool_pa]"
fi

echo ""
echo "  The printer.cfg deployed in step 3 already enables Pressure Advance"
echo "  ([include pa_tuner.cfg], [include pa_menu.cfg], [dwarf_pa_tuner],"
echo "  [tool_pa]) - do NOT add them again, Klipper rejects duplicate sections."
echo ""
echo "  Tune each tool with PA_TUNE_T0 .. PA_TUNE_T4 (~6 min per tool), then"
echo "  SAVE_CONFIG. Every tool stores its OWN value - docs/PRESSURE_ADVANCE.md"

# Step 5: Configure build via menuconfig
echo ""
echo "[5/6] Build configuration..."
echo ""
echo "  make menuconfig will now launch. Set these options:"
echo "    - Micro-controller Architecture: STMicroelectronics STM32"
echo "    - Processor model: STM32F407"
echo "    - Bootloader offset: 128KiB with STM32 bootloader (0x08020200)"
echo "    - Clock Reference: 12 MHz crystal"
echo "    - Communication interface: USB (on PA11/PA12)"
echo "    - USB ids: 0x1d50 / 0x614e"
echo ""
echo "  THE OFFSET MATTERS. 0x08020200 places Klipper AFTER Prusa's bootloader,"
echo "  which stays intact - so this and every future update is a drag-and-drop"
echo "  .bbf onto a USB stick, no jumper and no opening the printer."
echo "  Only use 'No bootloader' for the DFU recovery path in docs section 12,"
echo "  which OVERWRITES the Prusa bootloader."
echo ""
echo "  Save and exit when done."
echo ""
read -p "  Press Enter to launch menuconfig..."

cd ~/klipper
make menuconfig

# Step 6: Build and package as .bbf for the Prusa bootloader
echo ""
echo "[6/6] Building firmware..."

# Build through the helper, NOT a bare `make`.
#
# KlipperXL adds two C files to Klipper's MCU build, and Klipper has no hook
# for out-of-tree sources - the only way in is to edit two files that belong
# to Klipper: src/Makefile and src/stm32/Makefile.
#
# Earlier versions of this installer edited them permanently. That quietly
# broke every future Klipper update, because `git pull` compares real file
# content and refuses to overwrite local changes:
#
#     error: Your local changes to the following files would be overwritten
#     by merge: src/Makefile
#
# build_klipperxl.sh adds those two lines, builds, and puts the files back -
# so the repo is clean at rest and Klipper updates apply normally. It also
# restores them if the build fails or is interrupted.
bash "$MODULE_DIR/scripts/build_klipperxl.sh" clean
bash "$MODULE_DIR/scripts/build_klipperxl.sh" -j4

echo ""
echo "  Packaging klipper.bin as a .bbf for the Prusa bootloader..."
BBF_OK=0
if ~/klippy-env/bin/pip install ecdsa -q 2>/dev/null; then
    if ~/klippy-env/bin/python "$MODULE_DIR/scripts/pack_fw.py" ~/klipper/out/klipper.bin \
        --no-sign --version 1.0.0+1 --printer-type 3 \
        --printer-version 1 --printer-subversion 0 --bbf-version 2; then
        BBF_OK=1
        echo "  Done -> ~/klipper/out/klipper.bbf"
    fi
fi
if [ "$BBF_OK" = "0" ]; then
    echo "  WARNING: could not package the .bbf. Do it by hand:"
    echo "    ~/klippy-env/bin/pip install ecdsa"
    echo "    ~/klippy-env/bin/python ~/KlipperXL/scripts/pack_fw.py \\"
    echo "      ~/klipper/out/klipper.bin --no-sign --version 1.0.0+1 \\"
    echo "      --printer-type 3 --printer-version 1 --printer-subversion 0 --bbf-version 2"
fi

echo ""
echo "=========================================="
echo " BUILD COMPLETE!"
echo "=========================================="
echo ""
echo "Firmware: ~/klipper/out/klipper.bin"
echo "Flashable: ~/klipper/out/klipper.bbf"
echo ""
echo "Next steps:"
echo ""
echo "  1. Fix config includes (if not done already):"
echo "     nano ~/printer_data/config/printer.cfg"
echo ""
echo "  2. Flash the XLBuddy from a USB drive (keeps the Prusa bootloader):"
echo "     - Copy ~/klipper/out/klipper.bbf to a FAT32 USB drive"
echo "     - Plug it into the PRINTER's front USB port"
echo "     - Power cycle the printer"
echo "     - When the Prusa logo appears, press the selector knob 2-3 times"
echo "     - Confirm the flash; choose Ignore if it warns about the signature"
echo "     - The progress bar STOPS AT ~50% - that is normal, Klipper has taken"
echo "       over and does not drive the printer display"
echo "     - Power OFF, remove the USB drive, power ON"
echo "     (Full detail: docs/INSTALLATION_GUIDE.md section 6)"
echo ""
echo "     Requires the broken appendix / safety seal on the XLBuddy. If you do"
echo "     not have that, use the DFU method in docs section 12 instead - but"
echo "     note it OVERWRITES the Prusa bootloader and needs a different build"
echo "     offset ('No bootloader')."
echo ""
echo "  3. Update serial port in printer.cfg:"
echo "     ls /dev/serial/by-id/"
echo "     nano ~/printer_data/config/printer.cfg"
echo ""
echo "  4. Hide KlipperXL's added files from Moonraker's update manager:"
echo "     bash ~/KlipperXL/scripts/fix_klipper_dirty.sh"
echo "     sudo systemctl restart moonraker"
echo ""
echo "  5. Restart Klipper:"
echo "     sudo systemctl restart klipper"
echo ""
echo "  6. Calibrate Pressure Advance per tool (~6 min each):"
echo "     PA_TUNE_T0 ... PA_TUNE_T4, then SAVE_CONFIG"
echo "     Every tool stores its OWN value - docs/PRESSURE_ADVANCE.md"
echo ""
