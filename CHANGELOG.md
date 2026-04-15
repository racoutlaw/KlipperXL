# Changelog

All notable changes to KlipperXL are documented here.

---

## 2026-04-14

### USB BBF is now the default flash method
- Restructured `INSTALLATION_GUIDE.md` to make USB drive flashing the primary path (Section 6)
- DFU flashing moved to Section 12 as an advanced / recovery-only method
- Rationale: USB BBF requires no opening the printer, no BOOT0 jumper, preserves the Prusa bootloader, and future updates are drag-and-drop to a USB stick
- Section 3.4 now instructs users to set bootloader offset to **128KiB with STM32 bootloader (0x08020200)** by default
- New Section 3.6: packaging the firmware as `.bbf` with `pack_fw.py`
- Section 1 (Prerequisites) now lists the FAT32 USB drive and broken-appendix requirement up front
- Troubleshooting updated with USB BBF-first error guidance
- Thanks to @Ro3Deee for raising the issue

---

## 2026-04-05

### Build & Install Fixes
- Fixed missing `build-essential` dependency that caused "cpp: No such file or directory" error during firmware build
- Install script now checks for `cpp` before building and tells you exactly what to install
- Fixed install script path bug that couldn't find source files when run from the scripts directory
- Guide now uses `git clone` instead of referencing a tarball package that was never published
- Guide uses `make menuconfig` instead of a pre-built .config file that was incompatible with current Klipper versions (caused "Multiple definitions for command 'identify'" error)
- All commands now run directly on the Pi — no separate build computer needed
- Added `tool_offsets.py` to the Python module deployment (was missing)
- Fixed section order: Python modules and config files deploy before MCU flash so you can prep the Pi without the printer connected
- Added note that KIAUH already clones Klipper (no duplicate clone step)
- Added section for fixing config includes (Mainsail vs Fluidd, LED effects)

### Side LED Strip Fix
- Corrected LED configuration from `chain_count: 1` / `color_order: RGBW` to `chain_count: 2` / `color_order: GRB`
- Prusa XL uses two daisy-chained WS2812 drivers per strip: Driver 1 handles RGB color, Driver 2 handles white on its green channel
- Confirmed from Prusa firmware source code

### USB Drive Firmware Flashing (New)
- Added advanced guide for flashing KlipperXL via USB drive instead of DFU
- Preserves the Prusa bootloader — no BOOT0 jumper needed for future updates
- Klipper compiled at 0x08020200 offset, packaged as .bbf using Prusa's pack_fw.py tool
- Bootloader progress bar stops at ~50% when Klipper takes over (normal — Klipper doesn't drive the display, just restart the Klipper service)
- Requires broken appendix (safety seal) to bypass signature verification
- Included pack_fw.py in the repo for packaging

### Recovery & Stock Restore
- Complete recovery instructions for both flash methods
- With bootloader preserved: just put stock Prusa .bbf on USB drive and power cycle
- Without bootloader: DFU flash bootloader first, then stock firmware via USB
- Included bootloader-xl-2.5.0.bin and verified stock XL_firmware_6.4.0.bbf in the recovery directory
- Emergency DFU noboot recovery option as last resort

### Dirty State Fix
- Added `tool_offsets.py` to the git exclude list so Klipper update manager stays clean

All changes tested with a complete fresh install from SD card flash through DWARF_STATUS confirmation.
