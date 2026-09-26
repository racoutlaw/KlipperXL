#!/bin/bash
# =============================================================================
# KlipperXL - build the XLBuddy firmware WITHOUT leaving the Klipper repo dirty
# Copyright (C) 2026 Richard Crook
#
# WHY THIS EXISTS
#
# KlipperXL adds two C files to Klipper's MCU build, and Klipper has no hook for
# out-of-tree sources - the only way in is to edit two TRACKED Makefiles:
#
#     src/stm32/Makefile   src-$(CONFIG_MACH_STM32F4) += modbus_stm32f4.c
#     src/Makefile         src-$(CONFIG_WANT_NEOPIXEL) += neopixel_spi.c
#
# Leaving those edits in place is what breaks Klipper updates. `git pull`
# compares real file content, so it aborts with:
#
#     error: Your local changes to the following files would be overwritten
#     by merge: src/Makefile
#
# Marking them assume-unchanged does NOT help: it only hides them from
# `git status`, and merge still reads the bytes. Moonraker then treats the
# repo as "diverged" and falls back to `git reset --hard`, which would delete
# both lines silently - leaving a build with NO MODBUS, so the board can no
# longer talk to the Dwarfs at all.
#
# So: patch, build, and put the Makefiles back. The repo is clean at rest,
# Klipper updates apply normally, and the .py modules and .c files are all
# untracked, which git never touches.
#
# Usage:  bash scripts/build_klipperxl.sh            # build
#         bash scripts/build_klipperxl.sh menuconfig # configure, then build
#         KLIPPER_DIR=~/klipper bash scripts/build_klipperxl.sh
# =============================================================================
set -e

KLIPPER_DIR="${KLIPPER_DIR:-$HOME/klipper}"
STM32_LINE='src-$(CONFIG_MACH_STM32F4) += modbus_stm32f4.c'
MAIN_LINE='src-$(CONFIG_WANT_NEOPIXEL) += neopixel_spi.c'

cd "$KLIPPER_DIR" || { echo "ERROR: no such directory: $KLIPPER_DIR"; exit 1; }
[ -f src/stm32/Makefile ] || { echo "ERROR: $KLIPPER_DIR is not a Klipper tree"; exit 1; }

# ---- the source files must already be in place (INSTALL_ON_PI.sh copies them)
for f in src/modbus_stm32f4.c src/neopixel_spi.c; do
    [ -f "$f" ] || { echo "ERROR: missing $f - run INSTALL_ON_PI.sh first"; exit 1; }
done

# ---- clear any legacy assume-unchanged flags, or the revert below cannot work
for f in src/Makefile src/stm32/Makefile; do
    if git ls-files -v "$f" 2>/dev/null | grep -q '^[a-z]'; then
        echo "  clearing stale assume-unchanged on $f"
        git update-index --no-assume-unchanged "$f"
    fi
done

# ---- refuse to run on a tree that already has Makefile edits we did not make
for f in src/Makefile src/stm32/Makefile; do
    if ! git diff --quiet -- "$f"; then
        echo "ERROR: $f is already modified."
        echo "       Inspect it, then restore with: git checkout -- $f"
        exit 1
    fi
done

# ---- ALWAYS put them back, even on a failed or interrupted build
revert_makefiles() {
    cd "$KLIPPER_DIR" 2>/dev/null || return
    git checkout -- src/Makefile src/stm32/Makefile 2>/dev/null || true
    echo "  Makefiles restored - repo is clean, Klipper updates will work"
}
trap revert_makefiles EXIT INT TERM

echo "=== patching the build (temporarily) ==="
grep -qF 'modbus_stm32f4.c' src/stm32/Makefile || { echo "$STM32_LINE" >> src/stm32/Makefile; echo "  + modbus_stm32f4.c"; }
grep -qF 'neopixel_spi.c'   src/Makefile       || { echo "$MAIN_LINE"  >> src/Makefile;       echo "  + neopixel_spi.c"; }

echo
echo "=== building ==="
if [ "$1" = "menuconfig" ]; then
    make menuconfig
    shift
fi
make "$@"

echo
echo "=== verifying MODBUS actually got compiled in ==="
if [ -f out/src/modbus_stm32f4.o ]; then
    echo "  OK: out/src/modbus_stm32f4.o present"
else
    echo "  !! modbus_stm32f4.o NOT built - do NOT flash this binary."
    echo "     The board would come up unable to talk to any Dwarf."
    exit 1
fi
ls -l out/klipper.bin 2>/dev/null | sed 's/^/  /'
echo
echo "Done. Flash with your usual method (see docs/USB_FLASH_METHOD.md)."
