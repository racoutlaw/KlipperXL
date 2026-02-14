# Prusa XL XLBuddy DFU Recovery

Use this if your XLBuddy is bricked (blank screen, won't boot) after a bad flash.

## Files Included

- `firmware.bin` - Prusa XL stock firmware (noboot version, ~1.8MB)
- `firmware.bbf` - Resource file (copy to USB drive if XL shows "Looking for BBF")

These binaries are from [Prusa-Firmware-Buddy](https://github.com/prusa3d/Prusa-Firmware-Buddy),
licensed under GPLv3. Source code is available at: https://github.com/prusa3d/Prusa-Firmware-Buddy

## Requirements

- Raspberry Pi (or any Linux machine) with `dfu-util` installed
- USB cable from Pi to XLBuddy
- DFU jumper location on XLBuddy board

## Recovery Steps

### 1. Copy firmware.bin to your Pi

From Windows PowerShell:
```powershell
scp firmware.bin pi@YOUR_PI_IP:~/
```

### 2. SSH into your Pi

```powershell
ssh pi@YOUR_PI_IP
```

### 3. Install dfu-util (if not already installed)

```bash
sudo apt update && sudo apt install dfu-util -y
```

### 4. Put XLBuddy in DFU Mode

1. **Power OFF** the printer completely
2. **Add the DFU jumper** on the XLBuddy board
3. **Power ON** the printer (screen stays off - this is normal)
4. **Connect USB cable** from Pi to XLBuddy

### 5. Verify DFU Device is Detected

```bash
sudo dfu-util -l
```

You should see output containing `[0483:df11]` and `@Internal Flash /0x08000000/...`

If not detected:
- Check USB cable connection
- Verify DFU jumper is installed correctly
- Try a different USB port

### 6. Flash the Firmware

```bash
sudo dfu-util -a 0 -s 0x08000000:leave -D ~/firmware.bin
```

Wait for it to complete (30-60 seconds). You should see:
```
Erase           [=========================] 100%
Download        [=========================] 100%
File downloaded successfully
```

### 7. Exit DFU Mode and Boot

1. **Power OFF** the printer
2. **Remove the DFU jumper**
3. **Power ON** the printer

### 8. If Screen Shows "Looking for BBF"

1. Copy `firmware.bbf` to a USB drive
2. Insert USB drive into the XL
3. It will find and install the resources automatically

## Troubleshooting

### "No DFU capable USB device available"
- DFU jumper not installed or in wrong position
- USB cable not connected or faulty
- Try different USB port on Pi

### Flashing completes but XL still won't boot
- Make sure you removed the DFU jumper before powering on
- Try the recovery process again

### "Looking for BBF" stuck forever
- Copy the firmware.bbf file to USB drive
- Or download latest XL firmware from Prusa website

## Technical Details

- MCU: STM32F407 (168MHz ARM Cortex-M4)
- Flash Address: 0x08000000 (no bootloader offset)
- Firmware Type: xl_release_noboot
- DFU Device ID: 0483:df11

## Notes

- This firmware has NO bootloader - it's flashed directly at 0x08000000
- The XLBuddy uses a **jumper** for DFU mode (not a button)
- Keep this recovery package handy when experimenting with custom firmware
