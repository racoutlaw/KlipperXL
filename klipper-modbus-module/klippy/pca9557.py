# PCA9557 I2C GPIO Expander for Prusa XL
# Releases Dwarf toolheads from reset
#
# Copyright (C) 2025
# This file may be distributed under the terms of the GNU GPLv3 license.

import logging
from . import bus

# PCA9557 Registers
REG_INPUT = 0x00
REG_OUTPUT = 0x01
REG_POLARITY = 0x02
REG_CONFIG = 0x03

# XLBuddy io_expander1 pin mapping:
# p0 = dwarf6Reset
# p1 = dwarf1Reset
# p2 = dwarf2Reset
# p3 = dwarf3Reset
# p4 = dwarf4Reset
# p5 = dwarf5Reset
# p6 = fanPowerSwitch
# p7 = modularBedReset

# All resets are active LOW (low = reset, high = run)
# We need to set all pins HIGH to release from reset

RELEASE_ALL_MASK = 0xFF  # All pins HIGH

class PCA9557:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.name = config.get_name()
        
        # Setup I2C using Klipper's bus module
        self.i2c = bus.MCU_I2C_from_config(
            config, 
            default_addr=0x19,  # XLBuddy PCA9557 address
            default_speed=100000
        )
        
        self.output_state = 0x00
        self.initialized = False
        
        # Register startup callback
        self.printer.register_event_handler("klippy:connect", 
                                            self._handle_connect)
        
        # Register gcode commands
        gcode = self.printer.lookup_object('gcode')
        gcode.register_command('PCA9557_RELEASE_DWARFS',
                               self.cmd_RELEASE_DWARFS,
                               desc="Release all Dwarf toolheads from reset")
        gcode.register_command('PCA9557_RESET_DWARFS',
                               self.cmd_RESET_DWARFS,
                               desc="Put all Dwarf toolheads into reset")
        gcode.register_command('PCA9557_STATUS',
                               self.cmd_STATUS,
                               desc="Show PCA9557 GPIO expander status")
        gcode.register_command('PCA9557_SET_PIN',
                               self.cmd_SET_PIN,
                               desc="Set a specific PCA9557 pin")
        
        logging.info("PCA9557: GPIO expander configured")
    
    def _handle_connect(self):
        """Called when Klipper connects to MCU - release Dwarfs from reset"""
        logging.info("PCA9557: Releasing Dwarf toolheads from reset...")
        try:
            self._release_all()
            self.initialized = True
            logging.info("PCA9557: Dwarfs released successfully!")
        except Exception as e:
            logging.error("PCA9557: Failed to release Dwarfs - %s", str(e))
    
    def _write_register(self, reg, value):
        """Write a byte to a PCA9557 register"""
        self.i2c.i2c_write([reg, value])
    
    def _read_register(self, reg):
        """Read a byte from a PCA9557 register"""
        params = self.i2c.i2c_read([reg], 1)
        return bytearray(params['response'])[0]
    
    def _release_all(self):
        """Release all Dwarfs and modular bed from reset"""
        # First set output register to all HIGH
        self._write_register(REG_OUTPUT, RELEASE_ALL_MASK)
        self.output_state = RELEASE_ALL_MASK
        
        # Then configure all pins as outputs (0 = output, 1 = input)
        self._write_register(REG_CONFIG, 0x00)
        
        logging.info("PCA9557: All resets released (output=0x%02x)", 
                     self.output_state)
    
    def _reset_all(self):
        """Put all Dwarfs and modular bed into reset"""
        # Set output register to all LOW
        self._write_register(REG_OUTPUT, 0x00)
        self.output_state = 0x00
        
        logging.info("PCA9557: All resets active (output=0x%02x)", 
                     self.output_state)
    
    def _set_pin(self, pin, state):
        """Set a specific pin high or low"""
        if pin < 0 or pin > 7:
            raise ValueError("Pin must be 0-7")
        
        if state:
            self.output_state |= (1 << pin)
        else:
            self.output_state &= ~(1 << pin)
        
        self._write_register(REG_OUTPUT, self.output_state)
    
    def cmd_RELEASE_DWARFS(self, gcmd):
        """G-code command to release Dwarfs from reset"""
        try:
            self._release_all()
            gcmd.respond_info("PCA9557: Dwarf toolheads released from reset")
        except Exception as e:
            gcmd.respond_info("PCA9557: Error - %s" % str(e))
    
    def cmd_RESET_DWARFS(self, gcmd):
        """G-code command to put Dwarfs into reset"""
        try:
            self._reset_all()
            gcmd.respond_info("PCA9557: Dwarf toolheads put into reset")
        except Exception as e:
            gcmd.respond_info("PCA9557: Error - %s" % str(e))
    
    def cmd_SET_PIN(self, gcmd):
        """G-code command to set a specific pin"""
        pin = gcmd.get_int('PIN', minval=0, maxval=7)
        state = gcmd.get_int('STATE', minval=0, maxval=1)
        try:
            self._set_pin(pin, state)
            gcmd.respond_info("PCA9557: Pin %d set to %d" % (pin, state))
        except Exception as e:
            gcmd.respond_info("PCA9557: Error - %s" % str(e))
    
    def cmd_STATUS(self, gcmd):
        """G-code command to show PCA9557 status"""
        try:
            input_val = self._read_register(REG_INPUT)
            output_val = self._read_register(REG_OUTPUT)
            config_val = self._read_register(REG_CONFIG)
            
            # Decode which dwarfs are released
            dwarf_status = []
            pin_map = {
                0: "Dwarf6",
                1: "Dwarf1", 
                2: "Dwarf2",
                3: "Dwarf3",
                4: "Dwarf4",
                5: "Dwarf5",
                6: "FanPwr",
                7: "ModBed"
            }
            for pin in range(8):
                state = "RUN" if (output_val & (1 << pin)) else "RESET"
                dwarf_status.append("%s:%s" % (pin_map[pin], state))
            
            gcmd.respond_info(
                "PCA9557 Status:\n"
                "  Input:  0x%02x\n"
                "  Output: 0x%02x\n"
                "  Config: 0x%02x (0=output, 1=input)\n"
                "  Pins: %s" % (
                    input_val, output_val, config_val,
                    ", ".join(dwarf_status)
                )
            )
        except Exception as e:
            gcmd.respond_info("PCA9557: Error reading status - %s" % str(e))

def load_config(config):
    return PCA9557(config)
