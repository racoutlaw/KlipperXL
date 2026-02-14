# KlipperXL - MODBUS Master Module for Prusa XL
# Copyright (C) 2026 Richard Crook
#
# Based on Klipper 3D Printer Firmware
#   Copyright (C) 2016-2024 Kevin O'Connor <kevin@koconnor.net>
# Portions derived from Prusa-Firmware-Buddy
#   Copyright (C) 2019-2024 Prusa Research a.s. - www.prusa3d.com
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program. If not, see <https://www.gnu.org/licenses/>.

import logging
import struct

# Dwarf register addresses (from dwarf_registers.hpp)
class DwarfRegs:
    # Discrete Inputs (FC 0x02)
    IS_PICKED = 0x0000
    IS_PARKED = 0x0001
    IS_BUTTON_UP = 0x0002
    IS_BUTTON_DOWN = 0x0003
    
    # Coils (FC 0x05)
    TMC_ENABLE = 0x4000
    IS_SELECTED = 0x4001
    LOADCELL_ENABLE = 0x4002
    ACCEL_ENABLE = 0x4003
    
    # Input Registers (FC 0x04)
    FAULT_STATUS = 0x8060
    HOTEND_TEMP = 0x8061
    HOTEND_PWM = 0x8062
    FILAMENT_SENSOR = 0x8063
    BOARD_TEMP = 0x8064
    MCU_TEMP = 0x8065
    HEATBREAK_TEMP = 0x8066
    FAN0_RPM = 0x8069
    FAN0_PWM = 0x806A
    FAN1_RPM = 0x806D
    FAN1_PWM = 0x806E
    VOLTAGE_24V = 0x8071
    HEATER_CURRENT = 0x8072
    
    # Holding Registers (FC 0x06)
    TARGET_TEMP = 0xE000
    HEATBREAK_TARGET = 0xE001
    FAN0_PWM_SET = 0xE002
    FAN1_PWM_SET = 0xE003
    LED_PWM = 0xE004
    STATUS_LED_0 = 0xE005
    STATUS_LED_1 = 0xE006


class ModbusMaster:
    """
    MODBUS Master for Prusa XL Dwarf communication.
    
    Config:
        [modbus_master]
        # No options needed - uses fixed USART3 config
    """
    
    def __init__(self, config):
        self.printer = config.get_printer()
        self.reactor = self.printer.get_reactor()
        self.name = config.get_name().split()[-1]
        self.mcu = self.printer.lookup_object('mcu')
        
        self.read_cmd = self.write_cmd = self.coil_cmd = None
        self.pending = False
        self.response = None
        
        self.mcu.register_config_callback(self._build_config)
        self.printer.register_event_handler("klippy:connect", self._connect)
        
        logging.info("ModbusMaster: initialized")
        # Register test command
        gcode = self.printer.lookup_object("gcode")
        gcode.register_command("MODBUS_TEST", self.cmd_MODBUS_TEST,
                              desc="Test MODBUS read")
    
    def _build_config(self):
        self.mcu.add_config_cmd("config_modbus")
    
    def _connect(self):
        self.read_cmd = self.mcu.lookup_command(
            "modbus_read_regs address=%c start=%hu count=%hu")
        self.write_cmd = self.mcu.lookup_command(
            "modbus_write_reg address=%c reg=%hu value=%hu")
        self.coil_cmd = self.mcu.lookup_command(
            "modbus_write_coil address=%c coil=%hu value=%c")
        
        self.mcu.register_response(self._handle_result, "modbus_response")
        self.mcu.register_response(self._handle_write, "modbus_write_response")
        self.mcu.register_response(self._handle_coil, "modbus_coil_response")
    
    def _handle_result(self, params):
        if params['status'] == 0:
            data = params['data']
            if len(data) >= 2:
                # Parse as big-endian uint16 array
                count = len(data) // 2
                self.response = struct.unpack('>%dH' % count, data)
            else:
                self.response = ()
        else:
            self.response = None
        self.pending = False
    
    def _handle_write(self, params):
        self.response = params['status'] == 0
        self.pending = False
    
    def _handle_coil(self, params):
        self.response = params['status'] == 0
        self.pending = False
    def cmd_MODBUS_TEST(self, gcmd):
        addr = gcmd.get_int("ADDR", 1)
        reg = gcmd.get_int("REG", 0x8000)
        count = gcmd.get_int("COUNT", 1)
        gcmd.respond_info("Testing MODBUS: addr=%d reg=0x%04X count=%d" % (addr, reg, count))
        result = self.read_registers(addr, reg, count)
        if result is not None:
            gcmd.respond_info("Result: %s" % str(result))
        else:
            gcmd.respond_info("MODBUS read failed or timeout")
    
    def _wait_response(self, timeout=0.5):
        eventtime = self.reactor.monotonic()
        deadline = eventtime + timeout
        while self.pending and self.reactor.monotonic() < deadline:
            eventtime = self.reactor.pause(eventtime + 0.01)
        if self.pending:
            self.pending = False
            return None
        return self.response
    
    def read_registers(self, addr, start, count):
        """Read input registers from MODBUS device."""
        if not self.read_cmd:
            return None
        self.pending = True
        self.response = None
        self.read_cmd.send([addr, start, min(count, 32)])
        return self._wait_response()
    
    def write_register(self, addr, reg, value):
        """Write single register."""
        if not self.write_cmd:
            return False
        self.pending = True
        self.write_cmd.send([addr, reg, value])
        return self._wait_response() or False
    
    def write_coil(self, addr, coil, value):
        """Write single coil."""
        if not self.coil_cmd:
            return False
        self.pending = True
        self.coil_cmd.send([addr, coil, 1 if value else 0])
        return self._wait_response() or False


class PrusaDwarf:
    """
    Prusa XL Dwarf toolhead control via MODBUS.
    
    Config:
        [prusa_dwarf T0]
        address: 1
    """
    
    def __init__(self, config):
        self.printer = config.get_printer()
        self.reactor = self.printer.get_reactor()
        self.name = config.get_name().split()[-1]
        self.modbus = self.printer.load_object(config, 'modbus_master')
        self.address = config.getint('address')
        
        # State
        self.temp = 0.0
        self.target = 0.0
        self.heatbreak = 0.0
        self.fan0_rpm = 0
        self.fan1_rpm = 0
        
        self.printer.register_event_handler("klippy:ready", self._ready)
        
        # GCode commands
        gcode = self.printer.lookup_object('gcode')
        gcode.register_mux_command("DWARF_STATUS", "TOOL", self.name,
                                   self.cmd_STATUS)
        gcode.register_mux_command("DWARF_TEMP", "TOOL", self.name,
                                   self.cmd_TEMP)
        gcode.register_mux_command("DWARF_FAN", "TOOL", self.name,
                                   self.cmd_FAN)
        gcode.register_mux_command("DWARF_SELECT", "TOOL", self.name,
                                   self.cmd_SELECT)
        gcode.register_mux_command("DWARF_LED", "TOOL", self.name,
                                   self.cmd_LED)
        
        logging.info("PrusaDwarf %s: address %d", self.name, self.address)
    
    def _ready(self):
        self.reactor.register_timer(self._poll, self.reactor.NOW)
    
    def _poll(self, eventtime):
        try:
            # Read temps (0x8061-0x8066)
            regs = self.modbus.read_registers(self.address, DwarfRegs.HOTEND_TEMP, 6)
            if regs and len(regs) >= 6:
                self.temp = regs[0] if regs[0] < 1000 else regs[0] / 10.0
                self.heatbreak = regs[5] if regs[5] < 1000 else regs[5] / 10.0
            
            # Read fans (0x8069-0x806E)
            regs = self.modbus.read_registers(self.address, DwarfRegs.FAN0_RPM, 6)
            if regs and len(regs) >= 5:
                self.fan0_rpm = regs[0]
                self.fan1_rpm = regs[4]
        except Exception as e:
            logging.debug("Dwarf %s poll: %s", self.name, e)
        
        return eventtime + 0.25  # Poll every 250ms
    
    def set_temp(self, temp):
        self.target = temp
        self.modbus.write_register(self.address, DwarfRegs.TARGET_TEMP, int(temp))
    
    def set_fan(self, fan, speed):
        reg = DwarfRegs.FAN0_PWM_SET if fan == 0 else DwarfRegs.FAN1_PWM_SET
        self.modbus.write_register(self.address, reg, speed)
    
    def set_selected(self, sel):
        self.modbus.write_coil(self.address, DwarfRegs.IS_SELECTED, sel)
    
    def set_led(self, r, g, b, mode=1):
        # mode: 0=off, 1=solid, 2=blink, 3=pulse
        c0 = (g << 8) | r
        c1 = (mode << 8) | b
        self.modbus.write_register(self.address, DwarfRegs.STATUS_LED_0, c0)
        self.modbus.write_register(self.address, DwarfRegs.STATUS_LED_1, c1)
    
    # GCode commands
    def cmd_STATUS(self, gcmd):
        gcmd.respond_info(
            "Dwarf %s [%d]: Temp=%.1f/%.1f Heatbreak=%.1f Fan0=%d Fan1=%d"
            % (self.name, self.address, self.temp, self.target,
               self.heatbreak, self.fan0_rpm, self.fan1_rpm))
    
    def cmd_TEMP(self, gcmd):
        temp = gcmd.get_float('S', 0.0)
        self.set_temp(temp)
        gcmd.respond_info("Dwarf %s: temp -> %.1f" % (self.name, temp))
    
    def cmd_FAN(self, gcmd):
        fan = gcmd.get_int('F', 0)
        speed = gcmd.get_int('S', 0)
        self.set_fan(fan, speed)
        gcmd.respond_info("Dwarf %s: fan%d -> %d" % (self.name, fan, speed))
    
    def cmd_SELECT(self, gcmd):
        sel = gcmd.get_int('S', 1)
        self.set_selected(bool(sel))
        gcmd.respond_info("Dwarf %s: %s" % (self.name, "selected" if sel else "deselected"))
    
    def cmd_LED(self, gcmd):
        r = gcmd.get_int('R', 0)
        g = gcmd.get_int('G', 0)
        b = gcmd.get_int('B', 0)
        m = gcmd.get_int('M', 1)
        self.set_led(r, g, b, m)
        gcmd.respond_info("Dwarf %s: LED R=%d G=%d B=%d M=%d" % (self.name, r, g, b, m))


def load_config(config):
    return ModbusMaster(config)

def load_config_prefix(config):
    return PrusaDwarf(config)
