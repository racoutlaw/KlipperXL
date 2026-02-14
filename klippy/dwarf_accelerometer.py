# KlipperXL - Dwarf Accelerometer Support for Prusa XL
# LIS2DH12 accelerometer on each Dwarf toolhead, accessed via MODBUS FIFO
# Used for input shaper calibration with Klipper resonance_tester
#
# Copyright (C) 2026 Richard Crook
#
# Based on Klipper 3D Printer Firmware
#   Copyright (C) 2016-2024 Kevin O'Connor <kevin@koconnor.net>
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

import collections
import logging
import struct

Accel_Measurement = collections.namedtuple(
    'Accel_Measurement', ('time', 'accel_x', 'accel_y', 'accel_z'))


class DwarfAccelQueryHelper:
    """Helper class matching Klipper AccelQueryHelper interface."""

    def __init__(self, printer):
        self.printer = printer
        self.is_finished = False
        # Use estimated_print_time for accurate current time
        mcu = printer.lookup_object('mcu')
        reactor = printer.get_reactor()
        eventtime = reactor.monotonic()
        self.request_start_time = mcu.estimated_print_time(eventtime)
        self.request_end_time = self.request_start_time
        self.msgs = []
        self.samples = []

    def finish_measurements(self):
        toolhead = self.printer.lookup_object('toolhead')
        self.request_end_time = toolhead.get_last_move_time()
        toolhead.wait_moves()
        self.is_finished = True

    def handle_batch(self, msg):
        if self.is_finished:
            return False
        if len(self.msgs) >= 10000:
            return False
        self.msgs.append(msg)
        return True

    def has_valid_samples(self):
        for msg in self.msgs:
            data = msg.get('data', [])
            if not data:
                continue
            first_sample_time = data[0][0]
            last_sample_time = data[-1][0]
            if (first_sample_time > self.request_end_time
                    or last_sample_time < self.request_start_time):
                continue
            return True
        return False

    def get_samples(self):
        if not self.msgs:
            return self.samples
        total = sum([len(m.get('data', [])) for m in self.msgs])
        count = 0
        self.samples = samples = [None] * total
        for msg in self.msgs:
            for samp_time, x, y, z in msg.get('data', []):
                if samp_time < self.request_start_time:
                    continue
                if samp_time > self.request_end_time:
                    break
                samples[count] = Accel_Measurement(samp_time, x, y, z)
                count += 1
        del samples[count:]
        return self.samples

    def write_to_file(self, filename):
        """Write samples to CSV file in background process."""
        import multiprocessing
        import os

        samples_to_write = list(self.samples) if self.samples else list(self.get_samples())

        def write_impl(samples, fname):
            try:
                os.nice(20)
            except:
                pass
            with open(fname, "w") as f:
                f.write("#time,accel_x,accel_y,accel_z\n")
                for t, accel_x, accel_y, accel_z in samples:
                    f.write("%.6f,%.6f,%.6f,%.6f\n" % (t, accel_x, accel_y, accel_z))

        write_proc = multiprocessing.Process(target=write_impl, args=(samples_to_write, filename))
        write_proc.daemon = True
        write_proc.start()


class DwarfAccelerometer:
    """Accelerometer driver for Prusa XL Dwarf via MODBUS FIFO."""

    ACCEL_COIL = 0x4003
    FIFO_ADDR = 0x0000

    MSG_NO_DATA = 0
    MSG_LOG = 1
    MSG_LOADCELL = 2
    MSG_ACCEL_FAST = 4
    MSG_ACCEL_FREQ = 5

    SAMPLE_RATE = 1344.0
    SCALE = 2.0 * 9806.65 / 32768.0  # ~0.598 mm/s^2 per LSB

    def __init__(self, puppy_bootloader):
        self.puppy = puppy_bootloader
        self.printer = puppy_bootloader.printer
        self.gcode = puppy_bootloader.gcode
        self.reactor = puppy_bootloader.reactor
        self.mcu = puppy_bootloader.mcu
        self.name = "dwarf_accel"

        self._enabled = False
        self._dwarf = None
        self._collecting = False
        self._clients = []
        self._batch_timer = None
        self._sample_time = 0.0
        self._errors = 0
        self._overflows = 0
        self._measured_rate = 0.0
        self._bg_client = None
        self._polling_was_active = False

        self._axes_map = [(2, self.SCALE), (1, self.SCALE), (0, self.SCALE)]

        self._register_commands()
        # Register during config phase for resonance_tester
        self.printer.add_object('adxl345 ' + self.name, self)
        logging.info("DwarfAccelerometer: Registered as 'adxl345 %s'" % self.name)
        self.printer.register_event_handler("klippy:ready", self._handle_ready)
        logging.info("DwarfAccelerometer: Initialized (scale=%.6f mm/s^2 per LSB)" % self.SCALE)

    def _register_commands(self):
        self.gcode.register_command("ACCEL_ENABLE", self.cmd_ACCEL_ENABLE,
            desc="Enable Dwarf accelerometer")
        self.gcode.register_command("ACCEL_DISABLE", self.cmd_ACCEL_DISABLE,
            desc="Disable Dwarf accelerometer")
        self.gcode.register_command("ACCEL_QUERY", self.cmd_ACCEL_QUERY,
            desc="Query Dwarf accelerometer")
        self.gcode.register_command("ACCEL_MEASURE", self.cmd_ACCEL_MEASURE,
            desc="Start/stop accelerometer measurement")
        self.gcode.register_command("ACCEL_STATUS", self.cmd_ACCEL_STATUS,
            desc="Show accelerometer status")

    def _handle_ready(self):
        pass

    def get_status(self, eventtime):
        return {
            'enabled': self._enabled,
            'dwarf': self._dwarf,
            'collecting': self._collecting,
            'sample_rate': self.SAMPLE_RATE,
            'measured_rate': self._measured_rate,
            'errors': self._errors,
            'overflows': self._overflows
        }

    def enable(self, dwarf=None):
        """Enable accelerometer on specified Dwarf."""
        if dwarf is None:
            if self.puppy.active_tool < 0:
                self.gcode.respond_info("ACCEL_ENABLE: No tool selected - pick a tool first (T0-T4)")
                return False
            dwarf = self.puppy.active_tool + 1

        if dwarf not in self.puppy.booted_dwarfs:
            self.gcode.respond_info("ACCEL_ENABLE: Dwarf %d not booted" % dwarf)
            return False

        expected_tool = dwarf - 1
        if self.puppy.active_tool != expected_tool:
            self.gcode.respond_info("ACCEL_ENABLE: Tool T%d not picked" % expected_tool)
            return False

        # Disable loadcell first (they share the FIFO) - Prusa sequence
        if self.puppy.loadcell_enabled.get(dwarf, False):
            self.puppy._write_coil(dwarf, 0x4002, False)
            self.puppy.loadcell_enabled[dwarf] = False
            self.reactor.pause(self.reactor.monotonic() + 0.05)
            logging.info("DwarfAccelerometer: Disabled loadcell on Dwarf %d" % dwarf)

        # Enable accelerometer
        success = self.puppy._write_coil(dwarf, self.ACCEL_COIL, True)
        if success:
            self._dwarf = dwarf
            self._enabled = True
            self._errors = 0
            self._overflows = 0
            # Allow LIS2DH12 to initialize
            self.reactor.pause(self.reactor.monotonic() + 0.05)
            logging.info("DwarfAccelerometer: Enabled on Dwarf %d (T%d)" % (dwarf, dwarf-1))
        return success

    def disable(self):
        """Disable accelerometer."""
        if self._dwarf is None:
            return True
        success = self.puppy._write_coil(self._dwarf, self.ACCEL_COIL, False)
        if success:
            logging.info("DwarfAccelerometer: Disabled on Dwarf %d" % self._dwarf)
        self._enabled = False
        return success

    def _read_fifo(self):
        """Read FIFO from Dwarf via MODBUS FC 0x18 with retry logic."""
        if self._dwarf is None:
            return None

        addr = self.puppy.ADDR_MODBUS_OFFSET + self._dwarf
        data = [(self.FIFO_ADDR >> 8) & 0xFF, self.FIFO_ADDR & 0xFF]
        frame = self.puppy._build_modbus_frame(addr, 0x18, data)

        # Retry logic (3 retries like Prusa)
        for attempt in range(3):
            try:
                response = self.puppy._send_frame(frame)

                # Validate response is a dict
                if not isinstance(response, dict):
                    logging.debug("DwarfAccelerometer: Invalid response type: %s" % type(response).__name__)
                    continue

                status = response.get('status', -1)
                resp_data = response.get('data', b'')

                if status != 0:
                    logging.debug("DwarfAccelerometer: FIFO read attempt %d status=%d" % (attempt+1, status))
                    continue

                if not resp_data or len(resp_data) < 7:
                    logging.debug("DwarfAccelerometer: FIFO response too short")
                    continue

                byte_count = (resp_data[2] << 8) | resp_data[3]
                fifo_count = (resp_data[4] << 8) | resp_data[5]

                if fifo_count == 0:
                    return []

                fifo_data = resp_data[6:6 + byte_count - 2]
                registers = []
                for i in range(0, len(fifo_data) - 1, 2):
                    val = (fifo_data[i] << 8) | fifo_data[i + 1]
                    registers.append(val)
                return registers

            except Exception as e:
                logging.debug("DwarfAccelerometer: FIFO read attempt %d error: %s" % (attempt+1, e))
                continue

        self._errors += 1
        return None

    def _decode_fifo(self, registers):
        """Decode FIFO stream into accelerometer samples."""
        if not registers:
            return [], 0.0

        data = bytearray()
        for reg in registers:
            data.append(reg & 0xFF)
            data.append((reg >> 8) & 0xFF)

        samples = []
        measured_rate = self._measured_rate
        pos = 0

        while pos < len(data):
            msg_type = data[pos]
            pos += 1

            if msg_type == self.MSG_NO_DATA:
                continue
            elif msg_type == self.MSG_LOG:
                if pos + 8 > len(data):
                    break
                pos += 8
            elif msg_type == self.MSG_LOADCELL:
                if pos + 8 > len(data):
                    break
                pos += 8
            elif msg_type == self.MSG_ACCEL_FAST:
                if pos + 8 > len(data):
                    break
                for _ in range(2):
                    if pos + 4 > len(data):
                        break
                    packed = (data[pos] | (data[pos+1] << 8) |
                             (data[pos+2] << 16) | (data[pos+3] << 24))
                    pos += 4
                    sample = self._unpack_sample(packed)
                    if sample:
                        samples.append(sample)
            elif msg_type == self.MSG_ACCEL_FREQ:
                if pos + 4 > len(data):
                    break
                freq_bytes = bytes(data[pos:pos+4])
                measured_rate = struct.unpack('<f', freq_bytes)[0]
                self._measured_rate = measured_rate
                pos += 4
            else:
                break

        return samples, measured_rate

    def _unpack_sample(self, packed):
        """Unpack 32-bit packed accelerometer sample."""
        buffer_overflow = (packed >> 30) & 1
        sample_overrun = (packed >> 31) & 1

        if buffer_overflow:
            self._overflows += 1
        if sample_overrun:
            self._errors += 1

        x = (packed << 6) & 0xFFC0
        if x >= 0x8000:
            x -= 0x10000

        y = (packed >> 4) & 0xFFC0
        if y >= 0x8000:
            y -= 0x10000

        z = (packed >> 14) & 0xFFC0
        if z >= 0x8000:
            z -= 0x10000

        return (x, y, z)

    def _convert_samples(self, raw_samples, eventtime):
        """Convert raw samples to physical units with proper timestamps."""
        (x_pos, x_scale), (y_pos, y_scale), (z_pos, z_scale) = self._axes_map
        converted = []

        # Use estimated_print_time for accurate current time
        current_print_time = self.mcu.estimated_print_time(eventtime)

        # Calculate timestamps backwards from current time
        num_samples = len(raw_samples)
        for i, raw_xyz in enumerate(raw_samples):
            x = round(raw_xyz[x_pos] * x_scale, 6)
            y = round(raw_xyz[y_pos] * y_scale, 6)
            z = round(raw_xyz[z_pos] * z_scale, 6)

            # Timestamp: current time minus time for remaining samples
            sample_time = current_print_time - (num_samples - i - 1) / self.SAMPLE_RATE
            converted.append((round(sample_time, 6), x, y, z))

        return converted

    def start_internal_client(self):
        """Start measurement client for resonance_tester."""
        aqh = DwarfAccelQueryHelper(self.printer)
        self._clients.append(aqh.handle_batch)
        self._start_measurements()
        return aqh

    def _start_measurements(self):
        """Start collecting accelerometer data."""
        if self._collecting:
            return

        if not self._enabled:
            if not self.enable():
                raise self.printer.command_error("Failed to enable Dwarf accelerometer")

        self._collecting = True
        self._errors = 0
        self._overflows = 0

        # CRITICAL FIX: Pause temperature polling to avoid MODBUS bus contention
        if hasattr(self.puppy, '_pause_polling_timer'):
            self.puppy._pause_polling_timer()
            self._polling_was_active = True
            logging.info("DwarfAccelerometer: Paused temperature polling")

        # Start polling timer
        self._batch_timer = self.reactor.register_timer(
            self._batch_callback, self.reactor.monotonic() + 0.01)
        logging.info("DwarfAccelerometer: Started measurements on Dwarf %d" % self._dwarf)

    def _finish_measurements(self):
        """Stop collecting accelerometer data."""
        if not self._collecting:
            return

        self._collecting = False
        self._clients.clear()

        if self._batch_timer is not None:
            self.reactor.unregister_timer(self._batch_timer)
            self._batch_timer = None

        self.disable()

        # CRITICAL FIX: Resume temperature polling
        if self._polling_was_active and hasattr(self.puppy, '_resume_polling_timer'):
            self.puppy._resume_polling_timer(delay=0.5)
            self._polling_was_active = False
            logging.info("DwarfAccelerometer: Resumed temperature polling")

        logging.info("DwarfAccelerometer: Finished (errors=%d, overflows=%d)"
                    % (self._errors, self._overflows))

    def _batch_callback(self, eventtime):
        """Timer callback to poll FIFO and distribute samples."""
        if not self._collecting:
            return self.reactor.NEVER

        # Drain FIFO until empty (up to 5 reads like Prusa)
        total_samples = []
        for _ in range(5):
            registers = self._read_fifo()
            if registers is None:
                break  # Error
            if len(registers) == 0:
                break  # Empty

            raw_samples, _ = self._decode_fifo(registers)
            if raw_samples:
                total_samples.extend(raw_samples)

            # If less than full FIFO, done
            if len(registers) < 31:
                break

        if total_samples:
            converted = self._convert_samples(total_samples, eventtime)
            if converted:
                msg = {
                    'data': converted,
                    'errors': self._errors,
                    'overflows': self._overflows
                }
                self._clients = [c for c in self._clients if c(msg)]

        return eventtime + 0.025

    # G-code commands
    def cmd_ACCEL_ENABLE(self, gcmd):
        dwarf = gcmd.get_int('DWARF', None)
        if self.enable(dwarf):
            gcmd.respond_info("Accelerometer enabled on Dwarf %d (T%d)"
                            % (self._dwarf, self._dwarf - 1))
        else:
            raise gcmd.error("Failed to enable accelerometer - check tool is picked")

    def cmd_ACCEL_DISABLE(self, gcmd):
        if self.disable():
            gcmd.respond_info("Accelerometer disabled")
        else:
            gcmd.respond_info("Accelerometer disable failed")

    def cmd_ACCEL_QUERY(self, gcmd):
        aclient = self.start_internal_client()
        self.printer.lookup_object('toolhead').dwell(1.0)
        aclient.finish_measurements()
        self._finish_measurements()

        values = aclient.get_samples()
        if not values:
            raise gcmd.error("No accelerometer measurements found")

        _, accel_x, accel_y, accel_z = values[-1]
        gcmd.respond_info("Accelerometer query (%d samples):" % len(values))
        gcmd.respond_info("  Last: x=%.3f y=%.3f z=%.3f mm/s^2" % (accel_x, accel_y, accel_z))
        gcmd.respond_info("  Rate: %.0f Hz (measured: %.1f Hz)"
                         % (self.SAMPLE_RATE, self._measured_rate))

    def cmd_ACCEL_MEASURE(self, gcmd):
        import time
        if self._bg_client is None:
            self._bg_client = self.start_internal_client()
            gcmd.respond_info("Accelerometer measurements started")
            gcmd.respond_info("Run ACCEL_MEASURE again to stop and save data")
            return

        name = gcmd.get("NAME", time.strftime("%Y%m%d_%H%M%S"))
        bg_client = self._bg_client
        self._bg_client = None
        bg_client.finish_measurements()
        self._finish_measurements()

        samples = bg_client.get_samples()
        filename = "/tmp/accel-%s.csv" % name
        bg_client.write_to_file(filename)
        gcmd.respond_info("Saved %d samples to %s" % (len(samples), filename))

    def cmd_ACCEL_STATUS(self, gcmd):
        status = "ENABLED" if self._enabled else "DISABLED"
        dwarf_str = "Dwarf %d (T%d)" % (self._dwarf, self._dwarf - 1) if self._dwarf else "none"
        collecting = "YES" if self._collecting else "NO"

        gcmd.respond_info("Accelerometer Status:")
        gcmd.respond_info("  State: %s" % status)
        gcmd.respond_info("  Dwarf: %s" % dwarf_str)
        gcmd.respond_info("  Collecting: %s" % collecting)
        gcmd.respond_info("  Sample rate: %.0f Hz (measured: %.1f Hz)"
                         % (self.SAMPLE_RATE, self._measured_rate))
        gcmd.respond_info("  Errors: %d, Overflows: %d" % (self._errors, self._overflows))
