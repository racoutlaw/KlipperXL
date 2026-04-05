# Advanced: USB Drive Firmware Flashing

Flash KlipperXL firmware via USB drive instead of DFU — no BOOT0 jumper needed for updates.

**Requirements:**
- Broken appendix (safety seal) on the XLBuddy board
- Prusa bootloader intact at 0x08000000
- FAT32 USB drive

---

## How It Works

The Prusa bootloader checks for `.bbf` firmware files on USB drives at boot. With a broken appendix, signature verification can be bypassed, allowing unsigned Klipper firmware to be flashed.

Klipper is compiled with a **0x08020200 offset** so it sits after the bootloader in flash memory. The bootloader stays at 0x08000000 and handles the firmware update process.

After flashing, the printer screen shows the Prusa logo with a progress bar that stops at approximately 50% — **this is normal**. The bootloader has handed off to Klipper, which does not drive the display. Just restart the Klipper service on the Pi.

---

## First-Time Setup

If your printer currently has Klipper flashed at 0x08000000 (no bootloader), you need to restore the Prusa bootloader first.

### 1. Flash the Bootloader via DFU

1. **Install the BOOT0 jumper** on the XLBuddy board
2. **Power cycle** the printer (off, wait, on) with USB connected to Pi
3. Flash the bootloader:
   ```bash
   sudo apt install dfu-util -y
   sudo dfu-util -a 0 -s 0x08000000 -D ~/KlipperXL/firmware/recovery/bootloader-xl-2.5.0.bin
   ```
4. **Remove the BOOT0 jumper** and **power off**

### 2. Flash Stock Prusa Firmware

This updates the Dwarfs (toolheads) to match the bootloader version:

1. Download the latest XL firmware `.bbf` from [Prusa downloads](https://www.prusa3d.com/page/firmware-and-software_541542/)
2. Copy it to a FAT32 USB drive
3. Plug into printer and power on
4. Let it flash completely — all Dwarfs will be updated

### 3. Build Klipper with Bootloader Offset

If you haven't built KlipperXL yet, follow the main [Installation Guide](INSTALLATION_GUIDE.md) through section 3.3. Then for `make menuconfig`, set:

- **Bootloader offset:** 128KiB with STM32 bootloader (0x08020200)

All other settings remain the same (STM32F407, 12MHz crystal, USB on PA11/PA12, USB ids 0x1d50/0x614e).

```bash
cd ~/klipper
make clean
make -j4
```

### 4. Package as .bbf

```bash
~/klippy-env/bin/pip install ecdsa
~/klippy-env/bin/python ~/KlipperXL/scripts/pack_fw.py ~/klipper/out/klipper.bin \
  --no-sign --version 1.0.0+1 --printer-type 3 \
  --printer-version 1 --printer-subversion 0 --bbf-version 2
```

This creates `~/klipper/out/klipper.bbf`

### 5. Flash Klipper via USB

1. Copy `klipper.bbf` to a FAT32 USB drive
2. Plug the USB drive into the printer
3. **Power cycle** the printer
4. Click **Ignore** if prompted about signature verification
5. Progress bar stops at ~50% — this is normal
6. Remove the USB drive
7. Update serial port in `printer.cfg` if this is first time:
   ```bash
   ls /dev/serial/by-id/
   nano ~/printer_data/config/printer.cfg
   ```
8. Restart Klipper:
   ```bash
   sudo systemctl restart klipper
   ```

---

## Future Firmware Updates

Once set up, updating Klipper is simple — no jumper, no opening the printer:

```bash
cd ~/klipper
make clean
make -j4
~/klippy-env/bin/python ~/KlipperXL/scripts/pack_fw.py ~/klipper/out/klipper.bin \
  --no-sign --version 1.0.0+1 --printer-type 3 \
  --printer-version 1 --printer-subversion 0 --bbf-version 2
```

Copy `klipper.bbf` to USB drive, plug into printer, power cycle, restart Klipper.

---

## Restoring Stock Prusa Firmware

To go back to stock Prusa firmware at any time:

1. Download the official XL firmware `.bbf` from Prusa
2. Copy to FAT32 USB drive
3. Plug into printer and power cycle
4. The bootloader will flash the stock firmware and update all Dwarfs

---

## Troubleshooting

### Bootloader shows "Firmware corrupted"
- Klipper was flashed at the wrong offset (0x08020000 instead of 0x08020200)
- Rebuild with `make menuconfig` → Bootloader offset: **128KiB with STM32 bootloader**
- The correct offset is 0x08020200 (NOT 0x08020000)

### Bootloader stuck at progress bar
- If it stops at ~50%, this is normal — Klipper has taken over but doesn't drive the display
- Restart Klipper on the Pi: `sudo systemctl restart klipper`

### BBF file fails verification
- With a broken appendix, click **Ignore** to bypass signature verification
- Make sure the `.bbf` was built with `--no-sign` flag

### Bootloader is missing
- Follow the [First-Time Setup](#first-time-setup) section above to restore it via DFU

---

*See the main [Installation Guide](INSTALLATION_GUIDE.md) for the complete KlipperXL setup.*
