# Prusa Puppy Bootloader Protocol for Klipper
# Sends START_APPLICATION command to boot Dwarfs into their application firmware
#
# The Dwarfs boot into a bootloader that waits for commands.
# We need to send START_APPLICATION (0x05) to make them run their app.
# Only then will they respond to MODBUS commands.

import logging

class PuppyBoot:
    """Handles Prusa Dwarf bootloader protocol to start application firmware"""
    
    # Bootloader commands
    CMD_GET_PROTOCOL_VERSION = 0x00
    CMD_SET_ADDRESS = 0x01
    CMD_GET_HARDWARE_INFO = 0x03
    CMD_START_APPLICATION = 0x05
    
    # Addresses
    ADDR_DEFAULT = 0x00  # All puppies start here after reset
    ADDR_FIRST_BOOT = 0x0A  # First assigned address in bootloader
    ADDR_MODBUS_OFFSET = 0x1A  # Offset for MODBUS addresses after app start
    
    # CRC16-IBM table (polynomial 0xA001, same as MODBUS but init 0x0000)
    CRC_TABLE = [
        0x0000, 0xC0C1, 0xC181, 0x0140, 0xC301, 0x03C0, 0x0280, 0xC241,
        0xC601, 0x06C0, 0x0780, 0xC741, 0x0500, 0xC5C1, 0xC481, 0x0440,
        0xCC01, 0x0CC0, 0x0D80, 0xCD41, 0x0F00, 0xCFC1, 0xCE81, 0x0E40,
        0x0A00, 0xCAC1, 0xCB81, 0x0B40, 0xC901, 0x09C0, 0x0880, 0xC841,
        0xD801, 0x18C0, 0x1980, 0xD941, 0x1B00, 0xDBC1, 0xDA81, 0x1A40,
        0x1E00, 0xDEC1, 0xDF81, 0x1F40, 0xDD01, 0x1DC0, 0x1C80, 0xDC41,
        0x1400, 0xD4C1, 0xD581, 0x1540, 0xD701, 0x17C0, 0x1680, 0xD641,
        0xD201, 0x12C0, 0x1380, 0xD341, 0x1100, 0xD1C1, 0xD081, 0x1040,
        0xF001, 0x30C0, 0x3180, 0xF141, 0x3300, 0xF3C1, 0xF281, 0x3240,
        0x3600, 0xF6C1, 0xF781, 0x3740, 0xF501, 0x35C0, 0x3480, 0xF441,
        0x3C00, 0xFCC1, 0xFD81, 0x3D40, 0xFF01, 0x3FC0, 0x3E80, 0xFE41,
        0xFA01, 0x3AC0, 0x3B80, 0xFB41, 0x3900, 0xF9C1, 0xF881, 0x3840,
        0x2800, 0xE8C1, 0xE981, 0x2940, 0xEB01, 0x2BC0, 0x2A80, 0xEA41,
        0xEE01, 0x2EC0, 0x2F80, 0xEF41, 0x2D00, 0xEDC1, 0xEC81, 0x2C40,
        0xE401, 0x24C0, 0x2580, 0xE541, 0x2700, 0xE7C1, 0xE681, 0x2640,
        0x2200, 0xE2C1, 0xE381, 0x2340, 0xE101, 0x21C0, 0x2080, 0xE041,
        0xA001, 0x60C0, 0x6180, 0xA141, 0x6300, 0xA3C1, 0xA281, 0x6240,
        0x6600, 0xA6C1, 0xA781, 0x6740, 0xA501, 0x65C0, 0x6480, 0xA441,
        0x6C00, 0xACC1, 0xAD81, 0x6D40, 0xAF01, 0x6FC0, 0x6E80, 0xAE41,
        0xAA01, 0x6AC0, 0x6B80, 0xAB41, 0x6900, 0xA9C1, 0xA881, 0x6840,
        0x7800, 0xB8C1, 0xB981, 0x7940, 0xBB01, 0x7BC0, 0x7A80, 0xBA41,
        0xBE01, 0x7EC0, 0x7F80, 0xBF41, 0x7D00, 0xBDC1, 0xBC81, 0x7C40,
        0xB401, 0x74C0, 0x7580, 0xB541, 0x7700, 0xB7C1, 0xB681, 0x7640,
        0x7200, 0xB2C1, 0xB381, 0x7340, 0xB101, 0x71C0, 0x7080, 0xB041,
        0x5000, 0x90C1, 0x9181, 0x5140, 0x9301, 0x53C0, 0x5280, 0x9241,
        0x9601, 0x56C0, 0x5780, 0x9741, 0x5500, 0x95C1, 0x9481, 0x5440,
        0x9C01, 0x5CC0, 0x5D80, 0x9D41, 0x5F00, 0x9FC1, 0x9E81, 0x5E40,
        0x5A00, 0x9AC1, 0x9B81, 0x5B40, 0x9901, 0x59C0, 0x5880, 0x9841,
        0x8801, 0x48C0, 0x4980, 0x8941, 0x4B00, 0x8BC1, 0x8A81, 0x4A40,
        0x4E00, 0x8EC1, 0x8F81, 0x4F40, 0x8D01, 0x4DC0, 0x4C80, 0x8C41,
        0x4400, 0x84C1, 0x8581, 0x4540, 0x8701, 0x47C0, 0x4680, 0x8641,
        0x8201, 0x42C0, 0x4380, 0x8341, 0x4100, 0x81C1, 0x8081, 0x4040,
    ]
    
    def __init__(self, config):
        self.printer = config.get_printer()
        self.gcode = self.printer.lookup_object('gcode')
        self.mcu = None
        self.modbus_send_cmd = None
        
        # Register event handler
        self.printer.register_event_handler("klippy:connect", self._handle_connect)
        
        # Register gcode commands
        self.gcode.register_command("PUPPY_START_APP", self.cmd_PUPPY_START_APP,
                                    desc="Send START_APPLICATION to Dwarf bootloader")
        self.gcode.register_command("PUPPY_DISCOVER", self.cmd_PUPPY_DISCOVER,
                                    desc="Try to discover puppies in bootloader")
        self.gcode.register_command("PUPPY_BOOT_ALL", self.cmd_PUPPY_BOOT_ALL,
                                    desc="Boot all Dwarfs into application mode")
        
        logging.info("PuppyBoot: Module initialized")
    
    def _handle_connect(self):
        """Called when Klipper connects to MCU"""
        self.mcu = self.printer.lookup_object('mcu')
        
        # Try to get the raw serial send command
        # We'll use the modbus_send_raw command if available
        try:
            self.modbus_send_cmd = self.mcu.lookup_query_command(
                "modbus_send_raw data=%*s",
                "modbus_raw_response data=%*s")
            logging.info("PuppyBoot: Found modbus_send_raw command")
        except:
            logging.info("PuppyBoot: modbus_send_raw not available, will use alternative")
            self.modbus_send_cmd = None
    
    def _crc16_ibm(self, data):
        """Calculate CRC16-IBM (init=0x0000, poly=0xA001)"""
        crc = 0x0000  # Bootloader uses init 0x0000, not 0xFFFF like MODBUS
        for byte in data:
            crc = (crc >> 8) ^ self.CRC_TABLE[(crc ^ byte) & 0xFF]
        return crc
    
    def _build_bootloader_frame(self, address, command, data=None):
        """Build a bootloader protocol frame"""
        if data is None:
            data = []
        
        # Frame: [address][command][data...][CRC16-LE]
        frame = [address, command] + list(data)
        crc = self._crc16_ibm(frame)
        frame.append(crc & 0xFF)
        frame.append((crc >> 8) & 0xFF)
        
        return bytes(frame)
    
    def _build_start_app_frame(self, address, salt=0, fingerprint=None):
        """Build START_APPLICATION command frame"""
        if fingerprint is None:
            fingerprint = [0] * 32  # 32-byte fingerprint, all zeros
        
        # Data: 4-byte salt (big-endian) + 32-byte fingerprint
        data = [
            (salt >> 24) & 0xFF,
            (salt >> 16) & 0xFF,
            (salt >> 8) & 0xFF,
            salt & 0xFF
        ] + list(fingerprint)
        
        return self._build_bootloader_frame(address, self.CMD_START_APPLICATION, data)
    
    def _build_get_protocol_version_frame(self, address):
        """Build GET_PROTOCOL_VERSION command frame"""
        return self._build_bootloader_frame(address, self.CMD_GET_PROTOCOL_VERSION)
    
    def _build_set_address_frame(self, current_address, new_address):
        """Build SET_ADDRESS command frame"""
        return self._build_bootloader_frame(current_address, self.CMD_SET_ADDRESS, [new_address])
    
    def cmd_PUPPY_START_APP(self, gcmd):
        """Send START_APPLICATION command to a specific address"""
        address = gcmd.get_int('ADDR', 0)  # Default to broadcast/default address
        salt = gcmd.get_int('SALT', 0)
        
        frame = self._build_start_app_frame(address, salt)
        
        self.gcode.respond_info(
            f"PuppyBoot: Sending START_APPLICATION to addr={address}\n"
            f"Frame: {frame.hex()}"
        )
        
        # Send via modbus raw if available
        if self.modbus_send_cmd:
            try:
                response = self.modbus_send_cmd.send([frame])
                self.gcode.respond_info(f"Response: {response}")
            except Exception as e:
                self.gcode.respond_info(f"Send error: {e}")
        else:
            # Use the existing MODBUS test command to send raw bytes
            # This is a workaround - we're hijacking the modbus interface
            self.gcode.respond_info("PuppyBoot: No raw send available, trying alternative...")
            # Try sending via gcode command
            self._send_raw_via_modbus(frame)
    
    def _send_raw_via_modbus(self, frame):
        """Try to send raw frame using available methods"""
        # For now, just log what we would send
        # The actual sending needs MCU firmware support
        self.gcode.respond_info(
            f"Would send {len(frame)} bytes: {frame.hex()}\n"
            "Need MCU firmware support for raw serial send"
        )
    
    def cmd_PUPPY_DISCOVER(self, gcmd):
        """Try to discover puppies by sending GET_PROTOCOL_VERSION"""
        address = gcmd.get_int('ADDR', 0)
        
        frame = self._build_get_protocol_version_frame(address)
        
        self.gcode.respond_info(
            f"PuppyBoot: Sending GET_PROTOCOL_VERSION to addr={address}\n"
            f"Frame: {frame.hex()}"
        )
        
        self._send_raw_via_modbus(frame)
    
    def cmd_PUPPY_BOOT_ALL(self, gcmd):
        """Boot all Dwarfs: assign addresses and start applications"""
        self.gcode.respond_info("PuppyBoot: Booting all Dwarfs...")
        
        # Step 1: Send START_APPLICATION to default address (0x00)
        # This should start any puppet sitting at the default address
        frame = self._build_start_app_frame(self.ADDR_DEFAULT, salt=0)
        self.gcode.respond_info(
            f"Step 1: START_APPLICATION to addr=0x00\n"
            f"Frame: {frame.hex()}"
        )
        
        # If we had proper MCU support, we would:
        # 1. Reset all puppies via PCA9557
        # 2. Wait for bootloader to start
        # 3. Assign unique addresses to each puppet
        # 4. Send START_APPLICATION to each
        # 5. Wait for application to start
        # 6. Now MODBUS communication works
        
        self.gcode.respond_info(
            "NOTE: Full boot sequence requires MCU firmware support for raw serial.\n"
            "Current MODBUS firmware sends MODBUS frames, not bootloader frames.\n"
            "We need to either:\n"
            "1. Add bootloader protocol support to MCU firmware, OR\n"
            "2. Modify existing MODBUS code to support raw frame sending"
        )

def load_config(config):
    return PuppyBoot(config)
