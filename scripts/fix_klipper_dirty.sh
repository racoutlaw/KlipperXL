#!/bin/bash
# KlipperXL - Fix Klipper "dirty" in Moonraker Update Manager
# Copyright (C) 2026 Richard Crook
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# Problem: After installing KlipperXL, the Klipper repo shows as "dirty" in
# Moonraker's Update Manager because we added custom files and modified
# Makefiles. This prevents Klipper updates from being applied.
#
# This script fixes it by:
# 1. Adding custom files to .git/info/exclude (local gitignore)
# 2. Clearing the OLD assume-unchanged flags (they hid a real problem)
# 3. Clearing Moonraker's cached dirty state
#
# Run this ON THE PI after deploying all KlipperXL files.
# Usage: bash fix_klipper_dirty.sh
# =============================================================================

set -e

KLIPPER_DIR="/home/pi/klipper"
MOONRAKER_DB="/home/pi/printer_data/database/moonraker-sql.db"

echo "=== KlipperXL: Fixing Klipper dirty state ==="

# Step 1: Add custom files to .git/info/exclude
echo ""
echo "Step 1: Adding custom files to .git/info/exclude..."
EXCLUDE_FILE="$KLIPPER_DIR/.git/info/exclude"

# Check if our entries already exist
if grep -q "KlipperXL custom files" "$EXCLUDE_FILE" 2>/dev/null; then
    echo "  Already configured, skipping."
else
    cat >> "$EXCLUDE_FILE" << 'EOF'

# KlipperXL custom files - exclude from dirty tracking
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
klippy/extras/tool_offsets.py
src/modbus_stm32f4.c
src/stm32/modbus_stm32f4.c
.config_xlbuddy
.config_xlbuddy.old

# Strobe project (LED belt-tension visualizer)
klippy/extras/strobe_test.py
src/neopixel_spi.c
EOF
    echo "  Done."
fi

# Step 2: UNDO the old assume-unchanged flags (migration from older KlipperXL)
#
# Earlier versions of this script marked src/Makefile and src/stm32/Makefile
# as assume-unchanged, because the installer edited them permanently and the
# repo therefore always looked dirty.
#
# That was the wrong fix. assume-unchanged only hides a file from `git status`
# - it does NOT stop `git merge` reading the bytes. So Moonraker reported the
# repo clean and valid while every Klipper update still failed with:
#
#     error: Your local changes to the following files would be overwritten
#     by merge: src/Makefile
#
# Worse, Moonraker treats that as a diverged repo and falls back to
# `git reset --hard`, which would silently delete the MODBUS build line and
# leave a firmware that cannot talk to the Dwarfs at all.
#
# KlipperXL no longer edits those files at rest - scripts/build_klipperxl.sh
# adds the lines, builds, and puts them back. So the flags must come OFF.
echo ""
echo "Step 2: Clearing old assume-unchanged flags..."
cd "$KLIPPER_DIR"

for f in src/Makefile src/stm32/Makefile; do
    if git ls-files -v "$f" 2>/dev/null | grep -q '^[a-z]'; then
        git update-index --no-assume-unchanged "$f"
        echo "  $f - flag cleared"
        if ! git diff --quiet -- "$f"; then
            git checkout -- "$f"
            echo "  $f - restored to pristine (build-time patching is used now)"
        fi
    else
        echo "  $f - no flag set (OK)"
    fi
done
echo ""
echo "  NOTE: build firmware with scripts/build_klipperxl.sh from now on."
echo "  A plain 'make' will now produce a binary with NO MODBUS support."

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
    echo "  If a Makefile is listed, an interrupted build left it patched. Fix with:"
    echo "    cd ~/klipper && git checkout -- src/Makefile src/stm32/Makefile"
fi

echo ""
echo "=== Done! Restart Moonraker for changes to take effect ==="
echo "Run: sudo systemctl restart moonraker"
