#!/bin/bash
# =============================================================================
# XlKlipper - Fix Klipper "dirty" in Moonraker Update Manager
# =============================================================================
#
# Problem: After installing XlKlipper, the Klipper repo shows as "dirty" in
# Moonraker's Update Manager because we added custom files and modified
# Makefiles. This prevents Klipper updates from being applied.
#
# This script fixes it by:
# 1. Adding custom files to .git/info/exclude (local gitignore)
# 2. Marking modified tracked files as assume-unchanged
# 3. Clearing Moonraker's cached dirty state
#
# Run this ON THE PI after deploying all XlKlipper files.
# Usage: bash fix_klipper_dirty.sh
# =============================================================================

set -e

KLIPPER_DIR="/home/pi/klipper"
MOONRAKER_DB="/home/pi/printer_data/database/moonraker-sql.db"

echo "=== XlKlipper: Fixing Klipper dirty state ==="

# Step 1: Add custom files to .git/info/exclude
echo ""
echo "Step 1: Adding custom files to .git/info/exclude..."
EXCLUDE_FILE="$KLIPPER_DIR/.git/info/exclude"

# Check if our entries already exist
if grep -q "XlKlipper custom files" "$EXCLUDE_FILE" 2>/dev/null; then
    echo "  Already configured, skipping."
else
    cat >> "$EXCLUDE_FILE" << 'EOF'

# XlKlipper custom files - exclude from dirty tracking
klippy/extras/puppy_bootloader.py
klippy/extras/puppy_bootloader.py.*
klippy/extras/puppy_bootloader_working_probe.py.bak
klippy/extras/dwarf_accelerometer.py
klippy/extras/dwarf_accelerometer.py.*
klippy/extras/dwarf_extruder.py.unused
klippy/extras/led_effect.py
klippy/extras/loadcell_probe.py
klippy/extras/modbus_master.py
klippy/extras/pca9557.py
klippy/extras/babystep.py
src/babystep.c
src/modbus_stm32f4.c
src/stm32/modbus_stm32f4.c
.config_xlbuddy
.config_xlbuddy.old
EOF
    echo "  Done."
fi

# Step 2: Mark modified tracked files as assume-unchanged
echo ""
echo "Step 2: Marking modified Makefiles as assume-unchanged..."
cd "$KLIPPER_DIR"

# These are tracked files we modified to add babystep.c and modbus_stm32f4.c to the build
git update-index --assume-unchanged src/Makefile 2>/dev/null && echo "  src/Makefile marked" || echo "  src/Makefile not found (OK)"
git update-index --assume-unchanged src/stm32/Makefile 2>/dev/null && echo "  src/stm32/Makefile marked" || echo "  src/stm32/Makefile not found (OK)"

# Step 3: Clear Moonraker's cached dirty state
echo ""
echo "Step 3: Clearing Moonraker's cached state..."
if [ -f "$MOONRAKER_DB" ]; then
    # Delete the cached klipper entry so Moonraker re-checks fresh
    sqlite3 "$MOONRAKER_DB" "DELETE FROM namespace_store WHERE namespace='update_manager' AND key='klipper';" 2>/dev/null && echo "  Cleared cached state." || echo "  No cached state found (OK)."
else
    echo "  Moonraker database not found (OK if first install)."
fi

# Step 4: Verify
echo ""
echo "Step 4: Verifying..."
cd "$KLIPPER_DIR"
DIRTY=$(git status --porcelain 2>/dev/null | head -5)
if [ -z "$DIRTY" ]; then
    echo "  SUCCESS: Klipper repo is clean!"
else
    echo "  WARNING: Still showing modified files:"
    echo "$DIRTY"
    echo ""
    echo "  You may need to run: git update-index --assume-unchanged <file>"
    echo "  for each file listed above."
fi

echo ""
echo "=== Done! Restart Moonraker for changes to take effect ==="
echo "Run: sudo systemctl restart moonraker"
