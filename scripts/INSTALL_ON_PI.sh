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
# This will:
# 1. Check prerequisites
# 2. Copy MODBUS module to Klipper source
# 3. Copy Python extras to Klipper
# 4. Patch the STM32 Makefile
# 5. Launch make menuconfig for build setup
# 6. Build firmware

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
echo "  Done."

# Step 3: Deploy config files
echo ""
echo "[3/6] Deploying configuration files..."
cp "$MODULE_DIR/config/printer.cfg" ~/printer_data/config/printer.cfg
cp "$MODULE_DIR/config/tool_offsets.cfg" ~/printer_data/config/tool_offsets.cfg
cp "$MODULE_DIR/config/variables.cfg" ~/printer_data/config/variables.cfg
mkdir -p ~/printer_data/config/macros
cp "$MODULE_DIR/config/macros/print_macros.cfg" ~/printer_data/config/macros/print_macros.cfg
echo "  Done."
echo ""
echo "  NOTE: Edit ~/printer_data/config/printer.cfg to:"
echo "    - Change [include mainsail.cfg] to [include fluidd.cfg] if using Fluidd"
echo "    - Comment out [include led_effects.cfg] if not using side LED strips"

# Step 4: Patch STM32 Makefile
echo ""
echo "[4/6] Patching STM32 Makefile..."

STM32_MK=~/klipper/src/stm32/Makefile

if grep -q "modbus_stm32f4" "$STM32_MK"; then
    echo "  Already patched, skipping."
else
    sed -i '/^src-\$(CONFIG_MACH_STM32F4) += stm32\/stm32f4.c/a src-$(CONFIG_MACH_STM32F4) += modbus_stm32f4.c' "$STM32_MK"
    if grep -q "modbus_stm32f4" "$STM32_MK"; then
        echo "  Done."
    else
        echo "  WARNING: Automatic patching failed."
        echo "  Please manually edit ~/klipper/src/stm32/Makefile"
        echo "  Find the line: src-\$(CONFIG_MACH_STM32F4) += stm32/stm32f4.c ..."
        echo "  Add below it:  src-\$(CONFIG_MACH_STM32F4) += modbus_stm32f4.c"
    fi
fi

# Step 5: Configure build via menuconfig
echo ""
echo "[5/6] Build configuration..."
echo ""
echo "  make menuconfig will now launch. Set these options:"
echo "    - Micro-controller Architecture: STMicroelectronics STM32"
echo "    - Processor model: STM32F407"
echo "    - Bootloader offset: No bootloader"
echo "    - Clock Reference: 12 MHz crystal"
echo "    - Communication interface: USB (on PA11/PA12)"
echo "    - USB ids: 0x1d50 / 0x614e"
echo ""
echo "  Save and exit when done."
echo ""
read -p "  Press Enter to launch menuconfig..."

cd ~/klipper
make menuconfig

# Step 6: Build
echo ""
echo "[6/6] Building firmware..."

make clean
make -j4

echo ""
echo "=========================================="
echo " BUILD COMPLETE!"
echo "=========================================="
echo ""
echo "Firmware location: ~/klipper/out/klipper.bin"
echo ""
echo "Next steps:"
echo ""
echo "  1. Fix config includes (if not done already):"
echo "     nano ~/printer_data/config/printer.cfg"
echo ""
echo "  2. Flash the XLBuddy via DFU:"
echo "     - Power OFF printer"
echo "     - Install BOOT0 jumper on XLBuddy"
echo "     - Connect USB to Pi, power ON"
echo "     - sudo apt install dfu-util -y"
echo "     - sudo dfu-util -l  (verify device detected)"
echo "     - sudo dfu-util -a 0 -s 0x08000000:leave -D ~/klipper/out/klipper.bin"
echo "     - Power OFF, remove BOOT0 jumper, power ON"
echo ""
echo "  3. Update serial port in printer.cfg:"
echo "     ls /dev/serial/by-id/"
echo "     nano ~/printer_data/config/printer.cfg"
echo ""
echo "  4. Fix Klipper dirty state:"
echo "     bash ~/KlipperXL/scripts/fix_klipper_dirty.sh"
echo "     sudo systemctl restart moonraker"
echo ""
echo "  5. Restart Klipper:"
echo "     sudo systemctl restart klipper"
echo ""
