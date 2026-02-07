# Klipper MODBUS Master Module for Prusa XL

**STATUS: CODE COMPLETE - READY FOR TESTING**

This project implements a native MODBUS RTU master module for Klipper that enables full control of Prusa XL 5-tool printers without requiring any additional hardware.

## What This Does

- Adds MODBUS RTU master capability directly to Klipper's MCU firmware
- Uses the XLBuddy's built-in RS485 transceiver (USART3 + PG1)
- Communicates with Dwarf toolheads running stock Prusa firmware
- Enables full Klipper ecosystem: Mainsail/Fluidd, webcam, timelapse

## Files

```
klipper-modbus-module/
├── SPECIFICATION.md          # Complete technical specification
├── README.md                 # This file
├── printer.cfg.example       # Example Klipper configuration
├── src/
│   ├── modbus.h              # MCU MODBUS header (old version)
│   ├── modbus.c              # MCU MODBUS implementation (old version)
│   └── modbus_stm32f4.c      # ⭐ COMPLETE STM32F4 implementation
└── klippy/
    └── modbus_master.py      # ⭐ Python host module
```

## Hardware Configuration

**XLBuddy RS485 Interface:**
- UART: USART3
- Baud: 230,400
- TX Pin: PD8
- RX Pin: PD9
- Direction Pin: PG1 (HIGH=TX, LOW=RX)

**Dwarf Addresses:**
| Tool | Address |
|------|---------|
| T0   | 1       |
| T1   | 2       |
| T2   | 3       |
| T3   | 4       |
| T4   | 5       |

## Installation

### Step 1: Get Klipper Source

```bash
cd ~
git clone https://github.com/Klipper3d/klipper.git
cd klipper
```

### Step 2: Add MODBUS Code

Copy the MODBUS source into Klipper:

```bash
# Copy MCU code
cp /path/to/klipper-modbus-module/src/modbus_stm32f4.c src/

# Copy Python module
cp /path/to/klipper-modbus-module/klippy/modbus_master.py klippy/extras/
```

### Step 3: Configure Build

```bash
make menuconfig
```

Select:
- **Micro-controller Architecture:** STMicroelectronics STM32
- **Processor model:** STM32F407
- **Bootloader offset:** No bootloader (with broken appendix)
- **Clock Reference:** 8 MHz crystal
- **Communication interface:** USB (on PA11/PA12)

### Step 4: Build Firmware

```bash
make clean
make
```

Output: `out/klipper.bin`

### Step 5: Create BBF Package (Optional)

If flashing via USB drive instead of DFU:

```bash
python3 /path/to/Prusa-Firmware-Buddy/utils/pack_fw.py \
    --no-sign --no-checksum \
    --printer-type 3 \
    --version "0.0.1-klipper" \
    out/klipper.bin \
    out/klipper.bbf
```

### Step 6: Flash to XLBuddy

**Option A: DFU Mode**
1. Hold BOOT button while powering on
2. `make flash FLASH_DEVICE=/dev/ttyACM0`

**Option B: USB Drive**
1. Copy `klipper.bbf` to USB drive
2. Insert into XLBuddy USB port
3. Navigate to Settings > Firmware Update

### Step 7: Configure printer.cfg

See `printer.cfg.example` for a complete configuration.

Minimum required:
```ini
[mcu]
serial: /dev/serial/by-id/usb-Klipper_stm32f407xx_...

[modbus_master]
# Uses fixed USART3 configuration

[prusa_dwarf T0]
address: 1
```

## GCode Commands

Once configured, you can control Dwarfs with:

```gcode
; Check status of Dwarf T0
DWARF_STATUS TOOL=T0

; Set hotend temperature
DWARF_SET_TEMP TOOL=T0 TEMP=200

; Set fan speed (0-255, or 65535 for auto)
DWARF_SET_FAN TOOL=T0 FAN=0 SPEED=128

; Select tool (enables loadcell, etc.)
DWARF_SELECT TOOL=T0 SELECT=1
```

## How It Works

```
┌─────────────────────────────────────────────────────┐
│ Raspberry Pi                                        │
│ └── Klipper Host                                   │
│     └── modbus_master.py                           │
│         ├── read_input_registers()                 │
│         ├── write_register()                       │
│         └── write_coil()                           │
└────────────────────────┬────────────────────────────┘
                         │ USB Serial
                         ▼
┌─────────────────────────────────────────────────────┐
│ XLBuddy (STM32F407 running Klipper)                │
│ └── modbus_stm32f4.c                               │
│     ├── USART3 @ 230400 baud                       │
│     ├── PG1 direction control                      │
│     └── MODBUS RTU framing + CRC16                 │
└────────────────────────┬────────────────────────────┘
                         │ RS485 Bus
          ┌──────────────┼──────────────┐
          ▼              ▼              ▼
    ┌──────────┐   ┌──────────┐   ┌──────────┐
    │ Dwarf 1  │   │ Dwarf 2  │   │ Dwarf 3  │ ...
    │ (Stock)  │   │ (Stock)  │   │ (Stock)  │
    └──────────┘   └──────────┘   └──────────┘
```

## Testing

After flashing, test with:

```bash
# In Mainsail/Fluidd console:
DWARF_STATUS TOOL=T0

# Should report temperature, fan RPM, etc.
```

If no response:
1. Check MCU serial connection
2. Verify USART3 is initialized (check Klipper logs)
3. Try reading from different addresses (1-5)

## Known Limitations

- Modular Bed heating not yet implemented
- Loadcell probing not yet implemented  
- Tool change macros need refinement
- No accelerometer support yet

## Troubleshooting

**"Unknown command: modbus_read_regs"**
- MCU firmware doesn't have MODBUS code compiled in
- Rebuild with modbus_stm32f4.c included

**Timeout errors**
- Check RS485 wiring
- Verify Dwarfs are powered
- Try different baud rate (unlikely to help)

**CRC errors**
- Electrical noise on RS485 bus
- Bad termination
- Ground issues

## License

GPL v3 - Same as Klipper

## Credits

- Richard - Hardware testing, project concept
- Claude - Code development
- Prusa Research - Firmware reference (dwarf_registers.hpp, etc.)
- Klipper team - Excellent 3D printer firmware

---

⚠️ **DISCLAIMER**: This modifies your printer's firmware. Incorrect configuration could damage your printer. This project is NOT affiliated with or endorsed by Prusa Research.
