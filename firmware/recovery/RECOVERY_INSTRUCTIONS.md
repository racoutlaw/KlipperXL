# Prusa XL Recovery — Restoring Stock Firmware

How to restore your Prusa XL to stock Prusa firmware from KlipperXL.

Choose the section that matches how you installed KlipperXL.

---

## Recovery WITH Prusa Bootloader (USB Drive Method)

If you installed KlipperXL using the USB drive method (bootloader offset 0x08020200), the Prusa bootloader is still intact. Recovery is simple:

### Steps

1. Download the latest XL firmware from [Prusa Downloads](https://www.prusa3d.com/page/firmware-and-software_541542/)
2. Copy the `.bbf` file to a FAT32 USB drive
3. Plug the USB drive into the printer
4. Power cycle the printer
5. The bootloader will flash the stock firmware and update all Dwarfs
6. Done — your printer is back to stock Prusa

---

## Recovery WITHOUT Prusa Bootloader (DFU Method)

If you installed KlipperXL using the DFU method (no bootloader, flashed at 0x08000000), the Prusa bootloader was overwritten. Recovery requires two steps: restoring the bootloader, then flashing stock firmware.

### Requirements

- Raspberry Pi (or any Linux machine) with `dfu-util` installed
- USB cable from Pi to XLBuddy
- Physical access to the BOOT0 jumper on the XLBuddy board

### Files Included

| File | Purpose |
|------|---------|
| `bootloader-xl-2.5.0.bin` | Prusa XL bootloader (131KB, flashed to 0x08000000) |
| `XL_firmware_6.4.0.bbf` | Stock Prusa XL firmware (for USB drive recovery) |
| `firmware.bin` | Prusa XL firmware, noboot version (for DFU emergency recovery) |

**Note:** You can also download the latest firmware from [Prusa Downloads](https://www.prusa3d.com/page/firmware-and-software_541542/) if a newer version is available.

### Step 1: Flash the Bootloader via DFU

1. **Power OFF** the printer completely
2. **Install the BOOT0 jumper** on the XLBuddy board
3. **Connect USB cable** from Pi to XLBuddy
4. **Power ON** the printer (screen stays off — this is normal)

On the Pi:
```bash
sudo apt install dfu-util -y

# Verify DFU device is detected
sudo dfu-util -l
```

You should see output containing `[0483:df11]` and `@Internal Flash /0x08000000/...`

Flash the bootloader:
```bash
sudo dfu-util -a 0 -s 0x08000000 -D bootloader-xl-2.5.0.bin
```

Wait for it to complete. You should see:
```
Download done.
File downloaded successfully
```

### Step 2: Remove Jumper and Boot

1. **Power OFF** the printer
2. **Remove the BOOT0 jumper**
3. **Power ON** the printer

The Prusa bootloader logo should appear. It may show "Firmware corrupted" — this is expected since there's no application firmware yet.

### Step 3: Flash Stock Prusa Firmware via USB

1. Download the latest XL firmware `.bbf` from [Prusa Downloads](https://www.prusa3d.com/page/firmware-and-software_541542/)
2. Copy the `.bbf` file to a FAT32 USB drive
3. Plug the USB drive into the printer
4. The bootloader will detect and flash the firmware
5. All Dwarfs (toolheads) will be updated automatically
6. Once complete, the printer boots to the stock Prusa home screen

---

## Emergency DFU Recovery (No Bootloader, No USB)

If the bootloader is missing AND you can't use USB, you can flash the noboot firmware directly:

```bash
sudo dfu-util -a 0 -s 0x08000000:leave -D firmware.bin
```

This flashes stock Prusa firmware without a bootloader. The printer will run but won't have USB drive update capability. To restore full functionality, follow the full recovery procedure above.

---

## Troubleshooting

### "No DFU capable USB device available"
- BOOT0 jumper not installed or in wrong position
- USB cable not connected or faulty
- Try a different USB port on the Pi

### Screen shows "Firmware corrupted"
- Expected after flashing only the bootloader — proceed to flash firmware via USB drive

### "Looking for BBF" stuck
- Copy the latest Prusa firmware `.bbf` to a USB drive and plug it in

### Flashing completes but XL still won't boot
- Make sure the BOOT0 jumper is REMOVED before powering on
- Try the recovery process again

---

## Technical Details

- MCU: STM32F407 (168MHz ARM Cortex-M4)
- Bootloader address: 0x08000000 (128KB)
- Application address: 0x08020200 (after bootloader + descriptor)
- DFU Device ID: 0483:df11
- Bootloader source: [Prusa S3 downloads](https://prusa-buddy-firmware-dependencies.s3.eu-central-1.amazonaws.com/)

These binaries are from [Prusa-Firmware-Buddy](https://github.com/prusa3d/Prusa-Firmware-Buddy), licensed under GPLv3.
