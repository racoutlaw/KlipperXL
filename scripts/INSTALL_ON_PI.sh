#!/bin/bash
# KlipperXL - MODBUS Master Installation Script for Prusa XL
# Copyright (C) 2026 Richard Crook
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# Run this on your Raspberry Pi
#
# Usage: ./INSTALL_ON_PI.sh
#
# This will:
# 1. Clone/update Klipper
# 2. Copy MODBUS module files
# 3. Build firmware for XLBuddy
# 4. Generate flashable .bin file

set -e

echo "=========================================="
echo " Klipper MODBUS Master for Prusa XL"
echo " Installation Script"
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
    echo "Install Klipper, Moonraker, and Mainsail, then re-run this script."
    exit 1
fi

echo "Klipper installation found, proceeding..."
echo ""

cd ~

# Step 2: Copy MODBUS module files
echo ""
echo "[2/5] Installing MODBUS module..."

# Determine where our module files are
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

if [ -f "$SCRIPT_DIR/src/modbus_stm32f4.c" ]; then
    MODULE_DIR="$SCRIPT_DIR"
else
    echo "ERROR: Cannot find modbus_stm32f4.c"
    echo "Make sure you're running this from the klipper-modbus-module directory"
    exit 1
fi

# Copy MCU code
echo "Copying modbus_stm32f4.c to klipper/src/"
cp "$MODULE_DIR/src/modbus_stm32f4.c" ~/klipper/src/

# Copy Python module
echo "Copying modbus_master.py to klipper/klippy/extras/"
cp "$MODULE_DIR/klippy/modbus_master.py" ~/klipper/klippy/extras/

echo "Module files installed!"

# Step 3: Create menuconfig preset
echo ""
echo "[3/5] Creating build configuration..."

cat > ~/klipper/.config << 'EOF'
#
# Klipper config for Prusa XL XLBuddy (STM32F407)
#
CONFIG_LOW_LEVEL_OPTIONS=y
CONFIG_MACH_STM32=y
CONFIG_MACH_STM32F407=y
CONFIG_MACH_STM32F4=y
CONFIG_CLOCK_FREQ=168000000
CONFIG_FLASH_SIZE=0x100000
CONFIG_RAM_START=0x20000000
CONFIG_RAM_SIZE=0x20000
CONFIG_STACK_SIZE=512
CONFIG_STM32_CLOCK_REF_8M=y
CONFIG_USBSERIAL=y
CONFIG_USB_VENDOR_ID=0x1d50
CONFIG_USB_DEVICE_ID=0x614e
CONFIG_USB_SERIAL_NUMBER="XLBuddy"
CONFIG_CANBUS_FREQUENCY=1000000
CONFIG_INITIAL_PINS=""
CONFIG_HAVE_GPIO=y
CONFIG_HAVE_GPIO_ADC=y
CONFIG_HAVE_GPIO_SPI=y
CONFIG_HAVE_GPIO_I2C=y
CONFIG_HAVE_GPIO_BITBANGING=y
CONFIG_HAVE_STRICT_TIMING=y
CONFIG_INLINE_STEPPER_HACK=y
EOF

echo "Build config created!"

# Step 4: Add MODBUS to Makefile
echo ""
echo "[4/5] Patching Makefile..."

MAKEFILE=~/klipper/src/Makefile

# Check if already patched
if grep -q "modbus_stm32f4" "$MAKEFILE"; then
    echo "Makefile already patched"
else
    # Add modbus_stm32f4.c to the build
    sed -i '/src-\$(CONFIG_MACH_STM32F4) += stm32f4.c/a src-$(CONFIG_MACH_STM32F4) += modbus_stm32f4.c' "$MAKEFILE"
    echo "Makefile patched!"
fi

# Step 5: Build
echo ""
echo "[5/5] Building firmware..."

cd ~/klipper
make clean
make

echo ""
echo "=========================================="
echo " BUILD COMPLETE!"
echo "=========================================="
echo ""
echo "Firmware location: ~/klipper/out/klipper.bin"
echo ""
echo "To flash to XLBuddy:"
echo ""
echo "Option A - DFU Mode (requires USB connection):"
echo "  1. Hold BOOT button on XLBuddy"
echo "  2. Power on printer"
echo "  3. Run: cd ~/klipper && make flash FLASH_DEVICE=0483:df11"
echo ""
echo "Option B - USB Drive:"
echo "  1. Copy ~/klipper/out/klipper.bin to USB drive"
echo "  2. Rename to firmware.bbf (or use pack_fw.py)"
echo "  3. Insert USB into XLBuddy"
echo "  4. Printer should detect and flash"
echo ""
echo "After flashing, update your printer.cfg with the"
echo "[modbus_master] and [prusa_dwarf] sections."
echo ""
echo "See printer.cfg.example for a complete configuration."
echo ""
