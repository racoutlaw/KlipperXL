# KlipperXL - Dwarf Firmware Flasher (Clean Rebuild)
# Matches Prusa-Firmware-Buddy's exact flash protocol.
#
# Copyright (C) 2026 Richard Crook
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.

import logging
import os
import random
import struct
import time

# Flash protocol constants (matching Prusa-Firmware-Buddy)
CHUNK_SIZE = 247      # MAX_FLASH_BLOCK_LENGTH per Prusa
LOAD_SIZE = 48        # Max bytes per dwarf_flash_load (MESSAGE_MAX - overhead)
MAX_RETRIES = 3       # Full flash retry attempts (like Prusa)
MAX_CHUNK_RETRIES = 5 # Per-chunk retries before aborting attempt

# Bootloader addresses
BOOT_ADDR_DEFAULT = 0x00
BOOT_ADDR_BASE = 0x0A  # Dwarf N = 0x0A + N

# Bootloader commands (matching Prusa bootloader protocol)
BL_GET_PROTOCOL_VERSION = 0x00
BL_SET_ADDRESS = 0x01
BL_GET_HARDWARE_INFO = 0x03
BL_START_APPLICATION = 0x05
BL_WRITE_FLASH = 0x06
BL_FINALIZE_FLASH = 0x07
BL_READ_FLASH = 0x08
BL_GET_FINGERPRINT = 0x0E
BL_COMPUTE_FINGERPRINT = 0x0F

# MCU-level error codes (from send_frame_wait in modbus_stm32f4.c)
MCU_OK = 0
MCU_ERR_TIMEOUT = 1
MCU_ERR_CRC = 2
MCU_ERR_BUSY = 0xFE  # Matches MODBUS_ERR_BUSY (disambiguated from BL status=5)

MCU_ERROR_NAMES = {
    0: 'OK', 1: 'TIMEOUT', 2: 'CRC', 3: 'EXCEPTION',
    4: 'FRAME', 0xFE: 'MCU_BUSY',
}


class DwarfFlasher:
    """Flashes firmware to Dwarf toolheads via Prusa bootloader protocol.

    Uses MCU-side buffering (dwarf_flash_load + dwarf_flash_send) to send
    247-byte chunks matching Prusa's MAX_FLASH_BLOCK_LENGTH.  The MCU
    accumulates data via multiple 48-byte load commands, then builds and
    sends the full WRITE_FLASH RS485 frame in one burst.

    Protocol sequence (matching Prusa exactly):
      1. PCA9557 reset all other Dwarfs (bus isolation)
      2. Reset target Dwarf (1ms pulse → bootloader)
      3. SET_ADDRESS → assigned address
      4. GET_PROTOCOL_VERSION (verify alive)
      5. GET_HARDWARE_INFO (verify hw_type=42)
      6. COMPUTE_FINGERPRINT (pre-flash reference)
      7. WRITE_FLASH loop (247B chunks)
      8. FINALIZE_FLASH
      9. COMPUTE_FINGERPRINT + GET_FINGERPRINT (post-flash verify)
     10. START_APPLICATION

    G-code: DWARF_FLASH DWARF=1 FILE=/path/to/klipper.bin
    """

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
        self.puppy = None
        self._raw_send = None
        self._raw_send_boot = None
        self.flash_load_cmd = None
        self.flash_send_cmd = None
        self.flash_buf_reset_cmd = None
        self.flash_finalize_cmd = None
        self.read_flash_cmd = None
        self.pca9557_cmd = None

        self.printer.register_event_handler("klippy:connect",
                                            self._handle_connect)
        self.gcode.register_command('DWARF_FLASH', self.cmd_DWARF_FLASH,
                                    desc="Flash firmware to a Dwarf toolhead")
        self.gcode.register_command('DWARF_READ_FLASH',
                                    self.cmd_DWARF_READ_FLASH,
                                    desc="Read Dwarf firmware to file")

    def _handle_connect(self):
        self.mcu = self.printer.lookup_object('mcu')
        try:
            self.puppy = self.printer.lookup_object('puppy_bootloader')
        except Exception:
            self.puppy = None

        # Fire-and-forget raw RS485 (MODBUS response parser)
        self._raw_send = self.mcu.lookup_command("modbus_send_raw data=%*s")
        # Bootloader-aware raw RS485 (bootloader response parser)
        self._raw_send_boot = self.mcu.lookup_command(
            "modbus_send_raw_boot data=%*s")

        # Flash commands (query_command for synchronous send/response)
        self.flash_load_cmd = self.mcu.lookup_query_command(
            "dwarf_flash_load data=%*s",
            "dwarf_flash_load_response pos=%u", is_async=True)
        self.flash_send_cmd = self.mcu.lookup_query_command(
            "dwarf_flash_send address=%c offset=%u",
            "dwarf_flash_send_response status=%c offset=%u len=%u",
            is_async=True)
        self.flash_buf_reset_cmd = self.mcu.lookup_query_command(
            "dwarf_flash_buf_reset",
            "dwarf_flash_buf_reset_response pos=%u", is_async=True)
        self.flash_finalize_cmd = self.mcu.lookup_query_command(
            "dwarf_flash_finalize address=%c",
            "dwarf_flash_finalize_response status=%c", is_async=True)
        self.read_flash_cmd = self.mcu.lookup_query_command(
            "dwarf_read_flash address=%c offset=%u len=%c",
            "dwarf_read_flash_response status=%c offset=%u data=%*s",
            is_async=True)

        logging.info("DwarfFlasher: Connected, commands registered")

    def _ensure_pca9557(self):
        """Borrow pca9557_cmd from puppy_bootloader (lazy — puppy connects after us)."""
        if self.pca9557_cmd:
            return True
        if self.puppy:
            self.pca9557_cmd = getattr(self.puppy, 'pca9557_cmd', None)
        return self.pca9557_cmd is not None

    # ----------------------------------------------------------------
    # Low-level helpers
    # ----------------------------------------------------------------

    def _calc_crc16(self, data):
        crc = 0xFFFF
        for byte in data:
            crc = (crc >> 8) ^ self.CRC_TABLE[(crc ^ byte) & 0xFF]
        return crc

    def _build_frame(self, addr, cmd, data=b''):
        """Build bootloader frame: [addr][cmd][data...][crc16_LE]"""
        frame = bytes([addr, cmd]) + data
        crc = self._calc_crc16(frame)
        return frame + struct.pack('<H', crc)

    def _bl_cmd(self, addr, cmd, data=b'', timeout=2.0):
        """Send bootloader command via RS485 and wait for response.

        Returns dict:
          mcu_ret:   MCU send_frame_wait return (0=OK, 0xFE=BUSY, etc.)
          bl_status: bootloader status byte (-1 if MCU failed)
          bl_data:   parsed bootloader data bytes (empty if none)
          raw:       full raw RS485 response bytes
        """
        frame = self._build_frame(addr, cmd, data)
        reactor = self.printer.get_reactor()
        completion = reactor.completion()

        def response_cb(params):
            reactor.async_complete(completion, params)

        self.mcu._serial.register_response(
            response_cb, "modbus_raw_boot_response")
        try:
            self._raw_send_boot.send([list(frame)])
            params = completion.wait(reactor.monotonic() + timeout)
            if params is None:
                return {'mcu_ret': -1, 'bl_status': -1,
                        'bl_data': b'', 'raw': b''}

            mcu_ret = params.get('status', -1)
            raw = bytes(params.get('data', []))
            bl_status = -1
            bl_data = b''

            # Parse bootloader response: [addr][status][len][data...][crc16]
            if mcu_ret == MCU_OK and len(raw) >= 3:
                bl_status = raw[1]
                bl_len = raw[2]
                if len(raw) >= 3 + bl_len:
                    bl_data = raw[3:3 + bl_len]

            return {'mcu_ret': mcu_ret, 'bl_status': bl_status,
                    'bl_data': bl_data, 'raw': raw}
        finally:
            try:
                self.mcu._serial.register_response(
                    None, "modbus_raw_boot_response")
            except Exception:
                pass

    def _bl_noreply(self, addr, cmd, data=b''):
        """Send bootloader command without waiting for response."""
        frame = self._build_frame(addr, cmd, data)
        self._raw_send.send([list(frame)])

    def _pca9557_set(self, value):
        """Set PCA9557 output (controls Dwarf reset pins)."""
        self.pca9557_cmd.send([value])

    def _mcu_err(self, code):
        """Human-readable MCU error name."""
        return MCU_ERROR_NAMES.get(code, str(code))

    # ----------------------------------------------------------------
    # Flash chunk transfer
    # ----------------------------------------------------------------

    def _send_chunk(self, boot_addr, offset, chunk):
        """Load chunk into MCU buffer and send via RS485.

        Returns status from dwarf_flash_send_response:
          0 = bootloader OK
          5 = bootloader INVALID_ARGUMENTS (now distinct from MCU_BUSY=0xFE)
          0xFF = MCU-level failure (timeout, CRC, or buffer error)
        """
        # Reset MCU flash buffer
        self.flash_buf_reset_cmd.send([])

        # Load in LOAD_SIZE (48B) pieces — 6 loads for a full 247B chunk
        for i in range(0, len(chunk), LOAD_SIZE):
            piece = chunk[i:i + LOAD_SIZE]
            resp = self.flash_load_cmd.send([piece])
            if resp.get('pos', 0) != i + len(piece):
                return 0xFF  # Buffer mismatch

        # MCU builds WRITE_FLASH frame from buffer and sends on RS485
        resp = self.flash_send_cmd.send([boot_addr, offset])
        return resp.get('status', 0xFF)

    # ----------------------------------------------------------------
    # Flash protocol — single attempt
    # ----------------------------------------------------------------

    def _do_flash(self, gcmd, dwarf, boot_addr, firmware):
        """Execute one full flash attempt. Returns True on success."""
        fw_size = len(firmware)
        dwarf_bit = 1 << dwarf

        # Step 1: Hold ALL other Dwarfs in PCA9557 reset (bus isolation)
        quiet_mask = 0
        for d in range(1, 6):
            if d != dwarf:
                quiet_mask |= (1 << d)
        gcmd.respond_info(
            "Holding other dwarfs in reset (PCA9557=0x%02X)" % quiet_mask)
        self._pca9557_set(quiet_mask)
        time.sleep(0.010)

        # Step 2: Reset target Dwarf into bootloader (1ms pulse per Prusa)
        gcmd.respond_info("Resetting Dwarf %d into bootloader..." % dwarf)
        self._pca9557_set(quiet_mask | dwarf_bit)
        time.sleep(0.001)  # 1ms reset pulse
        self._pca9557_set(quiet_mask)  # Release target → bootloader
        time.sleep(0.005)  # 5ms bootloader init

        # Pause MODBUS polling for exclusive RS485 access
        if self.puppy:
            self.puppy._pause_polling_timer()
        time.sleep(0.200)  # Wait for any in-flight MODBUS to drain

        try:
            return self._flash_inner(gcmd, dwarf, boot_addr, firmware,
                                     fw_size)
        finally:
            # Always restore bus and PCA9557
            if self.puppy:
                self.puppy._resume_polling_timer(delay=2.0)
            self._pca9557_set(0x00)
            time.sleep(0.100)

    def _flash_inner(self, gcmd, dwarf, boot_addr, firmware, fw_size):
        """Inner flash sequence (called inside try/finally for cleanup)."""

        # Step 3: SET_ADDRESS (fire-and-forget, no response per Prusa)
        gcmd.respond_info("SET_ADDRESS -> 0x%02X" % boot_addr)
        self._bl_noreply(BOOT_ADDR_DEFAULT, BL_SET_ADDRESS,
                         bytes([boot_addr, 0x00]))
        time.sleep(0.050)

        # Step 4: GET_PROTOCOL_VERSION (verify bootloader alive)
        resp = self._bl_cmd(boot_addr, BL_GET_PROTOCOL_VERSION)
        if resp['mcu_ret'] != MCU_OK:
            gcmd.respond_info(
                "ERROR: No response at 0x%02X (%s)" %
                (boot_addr, self._mcu_err(resp['mcu_ret'])))
            return False
        if resp['bl_status'] != 0:
            gcmd.respond_info(
                "ERROR: Bootloader error (bl_status=%d)" % resp['bl_status'])
            return False
        gcmd.respond_info(
            "Bootloader alive (version=%s)" % resp['bl_data'].hex())

        # Step 5: GET_HARDWARE_INFO (verify hw_type=42 for STM32G070)
        resp = self._bl_cmd(boot_addr, BL_GET_HARDWARE_INFO)
        if resp['mcu_ret'] == MCU_OK and resp['bl_status'] == 0:
            hw = resp['bl_data']
            if len(hw) >= 1:
                hw_type = hw[0]
                gcmd.respond_info(
                    "Hardware type=%d %s" %
                    (hw_type,
                     '(STM32G070)' if hw_type == 42 else '(unexpected!)'))
        else:
            gcmd.respond_info("GET_HARDWARE_INFO: no response (non-fatal)")

        # Step 6: COMPUTE_FINGERPRINT with random salt (pre-flash reference)
        salt = random.getrandbits(32)
        salt_bytes = struct.pack('>I', salt)
        gcmd.respond_info("Computing pre-flash fingerprint...")
        self._bl_cmd(boot_addr, BL_COMPUTE_FINGERPRINT, salt_bytes)

        # Step 7: Poll until bootloader responsive (SHA256 takes ~330-600ms)
        fp_ready = False
        for i in range(40):  # Up to 2s
            time.sleep(0.050)
            resp = self._bl_cmd(boot_addr, BL_GET_PROTOCOL_VERSION,
                                timeout=0.5)
            if resp['mcu_ret'] == MCU_OK and resp['bl_status'] == 0:
                fp_ready = True
                break
        if not fp_ready:
            gcmd.respond_info(
                "WARNING: Bootloader unresponsive after COMPUTE_FINGERPRINT")

        # Step 8: GET_FINGERPRINT (pre-flash reference)
        pre_fp = None
        if fp_ready:
            resp = self._bl_cmd(boot_addr, BL_GET_FINGERPRINT,
                                bytes([0, 32]))
            if resp['mcu_ret'] == MCU_OK and len(resp['raw']) >= 35:
                pre_fp = bytes(resp['raw'][3:35])
                gcmd.respond_info(
                    "Pre-flash fingerprint: %s..." % pre_fp[:8].hex())

        # Step 9: WRITE_FLASH loop (247-byte chunks via MCU buffer)
        total_chunks = (fw_size + CHUNK_SIZE - 1) // CHUNK_SIZE
        gcmd.respond_info(
            "Writing %d bytes in %d x %dB chunks..." %
            (fw_size, total_chunks, CHUNK_SIZE))

        offset = 0
        while offset < fw_size:
            chunk = firmware[offset:offset + CHUNK_SIZE]

            ok = False
            last_status = 0xFF
            for retry in range(MAX_CHUNK_RETRIES):
                last_status = self._send_chunk(boot_addr, offset, chunk)
                if last_status == 0:
                    ok = True
                    break
                err = ("MCU_FAIL" if last_status == 0xFF else
                       "BL_ERR=%d" % last_status)
                if retry < MAX_CHUNK_RETRIES - 1:
                    gcmd.respond_info(
                        "  Retry %d at offset %d: %s" %
                        (retry + 1, offset, err))
                    time.sleep(0.050)

            if not ok:
                gcmd.respond_info(
                    "ERROR: Failed at offset %d after %d retries "
                    "(status=%d)" % (offset, MAX_CHUNK_RETRIES, last_status))
                return False

            offset += len(chunk)
            if offset % (CHUNK_SIZE * 20) == 0 or offset >= fw_size:
                pct = min(100, offset * 100 // fw_size)
                gcmd.respond_info(
                    "  Progress: %d/%d (%d%%)" % (offset, fw_size, pct))

        gcmd.respond_info("Flash data sent")

        # Step 10: FINALIZE_FLASH
        gcmd.respond_info("Finalizing flash...")
        fin_resp = self.flash_finalize_cmd.send([boot_addr])
        fin_status = fin_resp.get('status', 0xFF)
        if fin_status != 0:
            gcmd.respond_info("WARNING: Finalize status=%d" % fin_status)
        time.sleep(0.500)

        # Step 11: COMPUTE_FINGERPRINT with new salt (post-flash verify)
        new_salt = random.getrandbits(32)
        new_salt_bytes = struct.pack('>I', new_salt)
        gcmd.respond_info("Computing post-flash fingerprint...")
        self._bl_cmd(boot_addr, BL_COMPUTE_FINGERPRINT, new_salt_bytes)

        fp_ready = False
        for i in range(40):
            time.sleep(0.050)
            resp = self._bl_cmd(boot_addr, BL_GET_PROTOCOL_VERSION,
                                timeout=0.5)
            if resp['mcu_ret'] == MCU_OK and resp['bl_status'] == 0:
                fp_ready = True
                break

        # Step 12: GET_FINGERPRINT (post-flash)
        post_fp = None
        if fp_ready:
            resp = self._bl_cmd(boot_addr, BL_GET_FINGERPRINT,
                                bytes([0, 32]))
            if resp['mcu_ret'] == MCU_OK and len(resp['raw']) >= 35:
                post_fp = bytes(resp['raw'][3:35])
                gcmd.respond_info(
                    "Post-flash fingerprint: %s..." % post_fp[:8].hex())

        if pre_fp and post_fp and pre_fp == post_fp:
            gcmd.respond_info(
                "WARNING: Fingerprint unchanged -- flash may not have "
                "taken effect")

        # Step 13: START_APPLICATION with salt + fingerprint
        if post_fp:
            start_data = new_salt_bytes + bytes(post_fp)
            resp = self._bl_cmd(boot_addr, BL_START_APPLICATION, start_data)
            gcmd.respond_info(
                "START_APPLICATION: bl_status=%d" % resp['bl_status'])
            if resp['bl_status'] != 0:
                gcmd.respond_info(
                    "WARNING: Application may not have started")
        else:
            gcmd.respond_info(
                "WARNING: No fingerprint -- cannot start application")
            return False

        return True

    # ----------------------------------------------------------------
    # G-code commands
    # ----------------------------------------------------------------

    def cmd_DWARF_FLASH(self, gcmd):
        """DWARF_FLASH DWARF=1 FILE=/path/to/klipper.bin"""
        dwarf = gcmd.get_int('DWARF', 1)
        filepath = gcmd.get('FILE', '')

        if dwarf < 1 or dwarf > 5:
            gcmd.respond_info("ERROR: DWARF must be 1-5")
            return
        if not filepath or not os.path.exists(filepath):
            gcmd.respond_info("ERROR: File not found: %s" % filepath)
            return
        if not self._ensure_pca9557():
            gcmd.respond_info(
                "ERROR: pca9557 not available "
                "(puppy_bootloader not connected?)")
            return

        with open(filepath, 'rb') as f:
            firmware = f.read()
        fw_size = len(firmware)

        if fw_size > 120 * 1024:
            gcmd.respond_info("ERROR: Firmware too large (max 120KB)")
            return

        boot_addr = BOOT_ADDR_BASE + dwarf
        gcmd.respond_info(
            "Flashing Dwarf %d at 0x%02X (%d bytes, %dB chunks)" %
            (dwarf, boot_addr, fw_size, CHUNK_SIZE))

        for attempt in range(MAX_RETRIES):
            if attempt > 0:
                gcmd.respond_info(
                    "--- Retry attempt %d/%d ---" %
                    (attempt + 1, MAX_RETRIES))
                time.sleep(1.0)

            if self._do_flash(gcmd, dwarf, boot_addr, firmware):
                gcmd.respond_info(
                    "Flash complete! Dwarf %d running new firmware." % dwarf)
                return

        gcmd.respond_info(
            "ERROR: All %d attempts failed for Dwarf %d" %
            (MAX_RETRIES, dwarf))

    def cmd_DWARF_READ_FLASH(self, gcmd):
        """DWARF_READ_FLASH DWARF=1 FILE=/path/to/backup.bin [SIZE=122880]

        Resets the Dwarf into bootloader mode, reads the application flash
        region (0x08002000+), and saves to a binary file.
        """
        dwarf = gcmd.get_int('DWARF', 1)
        filepath = gcmd.get('FILE', '')
        flash_size = gcmd.get_int('SIZE', 120 * 1024)

        if dwarf < 1 or dwarf > 5:
            gcmd.respond_info("ERROR: DWARF must be 1-5")
            return
        if not filepath:
            gcmd.respond_info("ERROR: FILE parameter required")
            return
        if not self._ensure_pca9557():
            gcmd.respond_info("ERROR: pca9557 not available")
            return

        boot_addr = BOOT_ADDR_BASE + dwarf
        dwarf_bit = 1 << dwarf

        # Hold all OTHER dwarfs in reset for clean bus
        quiet_mask = 0
        for d in range(1, 6):
            if d != dwarf:
                quiet_mask |= (1 << d)
        self._pca9557_set(quiet_mask)
        time.sleep(0.010)

        # Reset target into bootloader
        gcmd.respond_info("Resetting Dwarf %d into bootloader..." % dwarf)
        self._pca9557_set(quiet_mask | dwarf_bit)
        time.sleep(0.001)
        self._pca9557_set(quiet_mask)
        time.sleep(0.005)

        if self.puppy:
            self.puppy._pause_polling_timer()
        time.sleep(0.200)

        try:
            # SET_ADDRESS
            self._bl_noreply(BOOT_ADDR_DEFAULT, BL_SET_ADDRESS,
                             bytes([boot_addr, 0x00]))
            time.sleep(0.050)

            # Verify alive
            resp = self._bl_cmd(boot_addr, BL_GET_PROTOCOL_VERSION)
            if resp['mcu_ret'] != MCU_OK or resp['bl_status'] != 0:
                gcmd.respond_info(
                    "ERROR: Dwarf %d not responding at 0x%02X" %
                    (dwarf, boot_addr))
                return
            gcmd.respond_info(
                "Bootloader alive at 0x%02X" % boot_addr)

            # Resume polling — read_flash uses its own MCU response type
            if self.puppy:
                self.puppy._resume_polling_timer(delay=0.5)

            # Read flash in 48-byte chunks
            read_size = 48
            total = (flash_size + read_size - 1) // read_size
            gcmd.respond_info(
                "Reading %d bytes in %d chunks..." % (flash_size, total))

            data = bytearray()
            offset = 0
            errors = 0
            while offset < flash_size:
                rlen = min(read_size, flash_size - offset)
                resp = self.read_flash_cmd.send(
                    [boot_addr, offset, rlen])
                if resp.get('status', 0xFF) != 0 or not resp.get('data'):
                    errors += 1
                    if errors > 20:
                        gcmd.respond_info(
                            "ERROR: Too many read errors at offset %d" %
                            offset)
                        return
                    time.sleep(0.050)
                    continue
                chunk = bytes(resp['data'])
                data.extend(chunk)
                offset += len(chunk)
                if offset % (read_size * 64) == 0 or offset >= flash_size:
                    pct = min(100, offset * 100 // flash_size)
                    gcmd.respond_info(
                        "  Progress: %d/%d (%d%%)" %
                        (offset, flash_size, pct))
                time.sleep(0.005)

            # Save to file
            gcmd.respond_info(
                "Saving %d bytes to %s" % (len(data), filepath))
            with open(filepath, 'wb') as f:
                f.write(data)

            # Restart application
            gcmd.respond_info("Restarting Dwarf application...")
            if self.puppy:
                self.puppy._pause_polling_timer()
            time.sleep(0.100)

            new_salt = random.getrandbits(32)
            new_salt_bytes = struct.pack('>I', new_salt)
            self._bl_cmd(boot_addr, BL_COMPUTE_FINGERPRINT, new_salt_bytes)
            for i in range(40):
                time.sleep(0.050)
                r = self._bl_cmd(boot_addr, BL_GET_PROTOCOL_VERSION,
                                 timeout=0.5)
                if r['mcu_ret'] == MCU_OK and r['bl_status'] == 0:
                    break

            r = self._bl_cmd(boot_addr, BL_GET_FINGERPRINT,
                             bytes([0, 32]))
            if r['mcu_ret'] == MCU_OK and len(r['raw']) >= 35:
                fp = bytes(r['raw'][3:35])
                self._bl_cmd(boot_addr, BL_START_APPLICATION,
                             new_salt_bytes + fp)

        finally:
            if self.puppy:
                self.puppy._resume_polling_timer(delay=2.0)
            self._pca9557_set(0x00)
            time.sleep(0.100)

        gcmd.respond_info(
            "Backup complete! %d bytes saved to %s" % (len(data), filepath))
        gcmd.respond_info(
            "To restore: DWARF_FLASH DWARF=%d FILE=%s" % (dwarf, filepath))


def load_config(config):
    return DwarfFlasher(config)
