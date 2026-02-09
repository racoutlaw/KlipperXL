# Klipper MODBUS Master Module for Prusa XL
## Complete Technical Specification

**Project Goal:** Create a native Klipper MODBUS master module that runs on the XLBuddy MCU (STM32F407) to communicate with Dwarf toolheads via the built-in RS485 hardware.

**Result:** Full Klipper control of Prusa XL 5-tool printer with no extra hardware required.

---

## 1. Hardware Configuration

### XLBuddy RS485 Interface

| Parameter | Value | Notes |
|-----------|-------|-------|
| MCU | STM32F407VGT6 | Same as many Klipper boards |
| UART | USART3 | Dedicated to puppies |
| Baud Rate | 230,400 | Fixed by Dwarf firmware |
| Flow Control Pin | PG1 | RS485 direction (TX/RX) |
| Word Length | 8 bits | Standard |
| Stop Bits | 1 | Standard |
| Parity | None | Standard |
| TX Pin | PD8 (USART3_TX) | Via RS485 transceiver |
| RX Pin | PD9 (USART3_RX) | Via RS485 transceiver |

### RS485 Direction Control
- **PG1 HIGH** = Transmit mode (driving the bus)
- **PG1 LOW** = Receive mode (listening)
- Must switch BEFORE transmit and AFTER transmit complete

### MODBUS Devices on Bus

| Device | Address | Description |
|--------|---------|-------------|
| Dwarf 0 | 1 | Tool 1 (T0) |
| Dwarf 1 | 2 | Tool 2 (T1) |
| Dwarf 2 | 3 | Tool 3 (T2) |
| Dwarf 3 | 4 | Tool 4 (T3) |
| Dwarf 4 | 5 | Tool 5 (T4) |
| Modular Bed | 0 | 16-zone heated bed |

---

## 2. MODBUS Protocol Details

### Frame Format (RTU)
```
[Address:1][Function:1][Data:N][CRC16:2]
```

### CRC16 Calculation
```c
uint16_t crc_modbus(uint8_t *data, size_t len) {
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (int j = 0; j < 8; j++) {
            if (crc & 1) {
                crc >>= 1;
                crc ^= 0xA001;
            } else {
                crc >>= 1;
            }
        }
    }
    return crc;
}
```

### Supported Functions

| Code | Function | Description |
|------|----------|-------------|
| 0x02 | Read Discrete Inputs | Read bool status (picked, parked, buttons) |
| 0x03 | Read Holding Registers | Read R/W registers |
| 0x04 | Read Input Registers | Read sensor data (temps, fans, etc.) |
| 0x05 | Write Single Coil | Set bool (TMC enable, loadcell enable) |
| 0x06 | Write Single Register | Set single value |
| 0x10 | Write Multiple Registers | Set multiple values (temps, fans, PID) |
| 0x18 | Read FIFO | Read loadcell/accelerometer stream |

### Timing Requirements
- **Inter-frame gap:** Minimum 1750µs (3.5 char times at 230400 baud)
- **Response timeout:** 20ms typical, 100ms max
- **Polling interval:** 200ms for non-critical, 50ms for selected tool

---

## 3. Dwarf Register Map

### Discrete Inputs (Function 0x02) - Read Only
| Address | Name | Description |
|---------|------|-------------|
| 0x0000 | is_picked | Tool is picked up (Hall sensor) |
| 0x0001 | is_parked | Tool is parked (Hall sensor) |
| 0x0002 | is_button_up_pressed | Upper button state |
| 0x0003 | is_button_down_pressed | Lower button state |

### Coils (Function 0x05) - Read/Write
| Address | Name | Description |
|---------|------|-------------|
| 0x4000 | tmc_enable | Enable/disable TMC2130 stepper |
| 0x4001 | is_selected | Mark tool as selected/active |
| 0x4002 | loadcell_enable | Enable loadcell readings |
| 0x4003 | accelerometer_enable | Enable accelerometer |

### Input Registers (Function 0x04) - Read Only
| Address | Name | Type | Description |
|---------|------|------|-------------|
| 0x8001 | hw_bom_id | uint16 | Hardware BOM ID |
| 0x8060 | fault_status | uint16 | Fault status mask |
| 0x8061 | hotend_measured_temperature | uint16 | Hotend temp (raw ADC or °C*10) |
| 0x8062 | hotend_pwm_state | uint16 | Heater PWM 0-255 |
| 0x8063 | tool_filament_sensor | uint16 | Filament sensor state |
| 0x8064 | board_temperature | int16 | Board temp °C |
| 0x8065 | mcu_temperature | int16 | MCU temp °C |
| 0x8066 | heatbreak_temp | uint16 | Heatbreak temp |
| 0x8067 | is_picked_raw | uint16 | Raw Hall sensor picked |
| 0x8068 | is_parked_raw | uint16 | Raw Hall sensor parked |
| 0x8069 | fan0_rpm | uint16 | Print fan RPM |
| 0x806A | fan0_pwm | uint16 | Print fan PWM |
| 0x806B | fan0_state | uint16 | Print fan state |
| 0x806C | fan0_is_rpm_ok | uint16 | Print fan RPM OK |
| 0x806D | fan1_rpm | uint16 | Heatbreak fan RPM |
| 0x806E | fan1_pwm | uint16 | Heatbreak fan PWM |
| 0x806F | fan1_state | uint16 | Heatbreak fan state |
| 0x8070 | fan1_is_rpm_ok | uint16 | Heatbreak fan RPM OK |
| 0x8071 | system_24V_mV | uint16 | 24V rail voltage in mV |
| 0x8072 | heater_current_mA | uint16 | Heater current in mA |
| 0x8073 | time_sync_lo | uint16 | Time sync low word |
| 0x8074 | time_sync_hi | uint16 | Time sync high word |

### Holding Registers (Function 0x03/0x10) - Read/Write
| Address | Name | Type | Description |
|---------|------|------|-------------|
| 0xE000 | nozzle_target_temperature | uint16 | Target hotend temp |
| 0xE001 | heatbreak_requested_temperature | uint16 | Target heatbreak temp |
| 0xE002 | fan0_pwm | uint16 | Print fan PWM (0-255, 0xFFFF=auto) |
| 0xE003 | fan1_pwm | uint16 | Heatbreak fan PWM (0-255, 0xFFFF=auto) |
| 0xE004 | led_pwm | uint16 | Cheese LED (MSB=selected, LSB=not selected) |
| 0xE005 | status_color_0 | uint16 | Status LED (MSB=Green, LSB=Red) |
| 0xE006 | status_color_1 | uint16 | Status LED (MSB=Mode, LSB=Blue) |
| 0xE007-0xE00C | pid | float[3] | PID values (P, I, D) |
| 0xE020 | tmc_read_request | uint16 | TMC register to read |
| 0xE021 | tmc_write_request_address | uint16 | TMC register to write |
| 0xE022-0xE023 | tmc_write_request_value | uint32 | TMC value to write |

### FIFO (Function 0x18)
| Address | Name | Description |
|---------|------|-------------|
| 0x0000 | encoded_stream | Loadcell + accelerometer data stream |

---

## 4. Klipper Module Architecture

### Files to Create

```
klipper/
├── klippy/
│   └── extras/
│       ├── modbus_master.py      # Main MODBUS host module
│       ├── prusa_dwarf.py        # Dwarf toolhead wrapper
│       └── prusa_modular_bed.py  # Modular bed wrapper
└── src/
    └── modbus/                   # MCU-side code
        ├── modbus.c              # MODBUS RTU master implementation
        ├── modbus.h              # Header file
        └── command.c             # Klipper command interface
```

### MCU-Side Implementation (C)

#### modbus.h
```c
#ifndef __MODBUS_H
#define __MODBUS_H

#include <stdint.h>

// MODBUS function codes
#define MODBUS_READ_DISCRETE_INPUTS   0x02
#define MODBUS_READ_HOLDING_REGISTERS 0x03
#define MODBUS_READ_INPUT_REGISTERS   0x04
#define MODBUS_WRITE_SINGLE_COIL      0x05
#define MODBUS_WRITE_SINGLE_REGISTER  0x06
#define MODBUS_WRITE_MULTIPLE_REGS    0x10
#define MODBUS_READ_FIFO              0x18

// Error codes
#define MODBUS_OK                     0
#define MODBUS_ERR_TIMEOUT            1
#define MODBUS_ERR_CRC                2
#define MODBUS_ERR_EXCEPTION          3
#define MODBUS_ERR_FRAME              4

struct modbus_master;

struct modbus_master *modbus_master_alloc(void);
void modbus_master_init(struct modbus_master *mb, uint32_t uart_bus,
                        uint32_t dir_pin, uint32_t baud);

int modbus_read_input_registers(struct modbus_master *mb, uint8_t address,
                                uint16_t start_reg, uint16_t count,
                                uint16_t *data);

int modbus_write_single_register(struct modbus_master *mb, uint8_t address,
                                 uint16_t reg, uint16_t value);

int modbus_write_coil(struct modbus_master *mb, uint8_t address,
                      uint16_t coil, uint8_t value);

int modbus_read_discrete_inputs(struct modbus_master *mb, uint8_t address,
                                uint16_t start, uint16_t count,
                                uint8_t *data);

#endif
```

### Host-Side Implementation (Python)

#### modbus_master.py (excerpt)
```python
class ModbusMaster:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.reactor = self.printer.get_reactor()
        
        # Get MCU and pins
        ppins = self.printer.lookup_object('pins')
        self.mcu = ppins.get_mcu()
        
        # RS485 configuration
        self.uart_bus = config.get('uart_bus')  # e.g., "usart3"
        self.dir_pin = config.get('dir_pin')    # e.g., "PG1"
        self.baud = config.getint('baud', 230400)
        
        # Register commands
        self.mcu.register_config_callback(self._build_config)
        
        # Dwarf tracking
        self.dwarfs = {}
        
    def _build_config(self):
        # Send configuration to MCU
        self.mcu.add_config_cmd(
            "config_modbus_master uart_bus=%s dir_pin=%s baud=%d"
            % (self.uart_bus, self.dir_pin, self.baud)
        )
```

---

## 5. Implementation Phases

### Phase 1: MCU MODBUS Driver - COMPLETE
- [x] Implement UART setup for USART3
- [x] Implement RS485 direction control
- [x] Implement MODBUS RTU framing (CRC16)
- [x] Implement basic read/write functions
- [x] Test with logic analyzer

### Phase 2: Klipper Integration - COMPLETE
- [x] Create Klipper command interface
- [x] Create Python host module
- [x] Implement register read/write from host
- [x] Test basic communication

### Phase 3: Dwarf Control - COMPLETE
- [x] Dwarf boot sequencing and registration
- [x] Heater control (per-tool, multi-tool preservation)
- [x] Fan control (print fan + heatbreak fan)
- [x] Temperature reading and display
- [x] TMC driver access

### Phase 4: Full Integration - COMPLETE
- [x] Multi-tool support (5 tools, automatic park/pick)
- [x] Toolchanger macros (Prusa-matched dock sequences)
- [x] Loadcell probing (Z home, bed mesh, tool calibration)
- [x] Filament sensor monitoring and autoload
- [x] Tool offset calibration (G425 equivalent)
- [x] Spool join (tool remapping on runout)
- [x] Tool buttons (physical load/unload)
- [x] Modular bed control (16-zone adaptive heating)
- [x] Side LED strips (status effects)
- [x] Input shaper (Dwarf accelerometer)
- [x] Stable multi-day operation verified

---

## 6. printer.cfg Example

```ini
[mcu]
serial: /dev/serial/by-id/usb-Klipper_stm32f407xx_...

[modbus_master puppybus]
uart_bus: usart3
dir_pin: PG1
baud: 230400

[prusa_dwarf T0]
modbus: puppybus
address: 1
extruder: extruder

[prusa_dwarf T1]
modbus: puppybus
address: 2
extruder: extruder1

# ... T2, T3, T4 ...

[extruder]
# Controlled via MODBUS to Dwarf
step_pin: modbus:T0:step
dir_pin: modbus:T0:dir
enable_pin: modbus:T0:enable
heater_pin: modbus:T0:heater
sensor_pin: modbus:T0:temp
# ... etc

[heater_fan heatbreak_fan_T0]
pin: modbus:T0:fan1
heater: extruder

[fan]
pin: modbus:T0:fan0
```

---

## 7. Risk Assessment

| Risk | Likelihood | Impact | Mitigation |
|------|------------|--------|------------|
| Timing issues | Medium | High | Careful interrupt handling, DMA |
| CRC errors | Low | Medium | Retry mechanism, error counting |
| Bus contention | Low | High | Proper flow control |
| Klipper architecture conflicts | Medium | High | Study existing code carefully |
| Dwarf firmware incompatibility | Low | High | Match exact protocol |

---

## 8. Success Criteria

1. ✅ Can read temperature from all 5 Dwarfs
2. ✅ Can set target temperature on selected Dwarf
3. ✅ Can control fans on all Dwarfs
4. ✅ Can enable/disable TMC steppers
5. ✅ Can read Hall sensors (picked/parked)
6. ✅ Can perform tool changes
7. ✅ Can probe with loadcell
8. ✅ Stable operation over 24+ hours

---

## 9. Resources

- Prusa Firmware Source: `Prusa-Firmware-Buddy/` (reference archive)
- Klipper Source: https://github.com/Klipper3d/klipper
- STM32F407 Reference Manual
- MODBUS RTU Specification

---

*Created: January 2026*
*Last Updated: February 2026*
*Author: Richard Crook*
*Status: WORKING - All phases complete, deployed on 5-tool Prusa XL*
