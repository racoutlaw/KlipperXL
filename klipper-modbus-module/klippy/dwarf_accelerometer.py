
# =============================================================================
# DWARF ACCELEROMETER SUPPORT
# =============================================================================
# LIS2DH12 accelerometer on each Dwarf toolhead, accessed via MODBUS FIFO
# Used for input shaper calibration with Klipper resonance_tester
# =============================================================================

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
        print_time = printer.lookup_object('toolhead').get_last_move_time()
        self.request_start_time = self.request_end_time = print_time
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
        import multiprocessing
        import os
        def write_impl():
            try:
                os.nice(20)
            except:
                pass
            f = open(filename, "w")
            f.write("#time,accel_x,accel_y,accel_z\n")
            samples = self.samples or self.get_samples()
            for t, accel_x, accel_y, accel_z in samples:
                f.write("%.6f,%.6f,%.6f,%.6f\n" % (t, accel_x, accel_y, accel_z))
            f.close()
        write_proc = multiprocessing.Process(target=write_impl)
        write_proc.daemon = True
        write_proc.start()


class DwarfAccelerometer:
    """Accelerometer driver for Prusa XL Dwarf via MODBUS FIFO.

    Each Dwarf has an LIS2DH12 accelerometer accessible through:
    - Coil 0x4003: Enable/disable accelerometer
    - FIFO 0x0000: Packed sample stream (FC18 Read FIFO Queue)

    ARCHITECTURE FIX (2026-02):
    ============================
    The previous implementation created a separate lookup_query_command for
    FIFO reads with is_async=True. This caused response routing conflicts with
    puppy_bootloader's send_raw_cmd (is_async=False) because both register
    handlers for 'modbus_raw_response'. When responses arrived, they could be
    routed to the wrong handler, causing the error:
        ('modbus_raw_response', None)

    The fix is to use puppy_bootloader._send_frame() exclusively for all
    MODBUS communication, which uses a single command object with proper
    response handling. During accelerometer collection, we:
    1. Pause the regular Dwarf polling timer to avoid MODBUS contention
    2. Use _send_frame() for FIFO reads with explicit pauses for reactor
    3. Resume polling when collection finishes

    This ensures single-threaded MODBUS access and proper response routing.
    """

    ACCEL_COIL = 0x4003
    FIFO_ADDR = 0x0000

    # FIFO message types from Prusa firmware
    MSG_NO_DATA = 0
    MSG_LOADCELL = 2
    MSG_ACCEL_FAST = 4
    MSG_ACCEL_FREQ = 5

    # LIS2DH12 at 1344 Hz, scale for mm/s^2
    SAMPLE_RATE = 1344.0
    SCALE = 0.003774 * 9806.65  # mg/LSB * mm/s^2 per g

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

        # Axes map: compensate for Dwarf orientation (X<->Z swap)
        self._axes_map = [(2, self.SCALE), (1, self.SCALE), (0, self.SCALE)]

        self._register_commands()
        self.printer.register_event_handler("klippy:ready", self._handle_ready)
        logging.info("DwarfAccelerometer: Initialized")

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
        # Register with resonance_tester if available
        try:
            res_tester = self.printer.lookup_object('resonance_tester', None)
            if res_tester is not None:
                res_tester.register_chip(self.name, self)
                logging.info("DwarfAccelerometer: Registered with resonance_tester")
        except Exception as e:
            logging.info("DwarfAccelerometer: resonance_tester not available: %s" % e)

    def enable(self, dwarf=None):
        """Enable accelerometer on specified Dwarf."""
        if dwarf is None:
            dwarf = self.puppy.active_tool + 1
            if dwarf < 1:
                logging.warning("DwarfAccelerometer: No tool selected")
                return False

        if dwarf not in self.puppy.booted_dwarfs:
            logging.warning("DwarfAccelerometer: Dwarf %d not booted" % dwarf)
            return False

        # Disable loadcell first (shared FIFO - mutually exclusive)
        if self.puppy.loadcell_enabled.get(dwarf, False):
            self.puppy._write_coil(dwarf, 0x4002, False)  # Loadcell coil
            self.puppy.loadcell_enabled[dwarf] = False
            self.reactor.pause(self.reactor.monotonic() + 0.05)

        # Enable accelerometer
        success = self.puppy._write_coil(dwarf, self.ACCEL_COIL, True)
        if success:
            self._dwarf = dwarf
            self._enabled = True
            self._errors = 0
            self._overflows = 0
            logging.info("DwarfAccelerometer: Enabled on Dwarf %d" % dwarf)
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
        """Read FIFO from Dwarf via MODBUS FC 0x18.

        CRITICAL: Uses puppy._send_frame() which shares the MODBUS command
        object with all other MODBUS operations. This ensures proper response
        routing without conflicts.

        The polling timer must be paused before calling this to avoid
        concurrent MODBUS operations.
        """
        if self._dwarf is None:
            return None

        if self.puppy.send_raw_cmd is None:
            logging.warning("DwarfAccelerometer: MODBUS command not available")
            return None

        # Dwarf 1 = address 0x1A, Dwarf 2 = 0x1B, etc.
        addr = self.puppy.ADDR_MODBUS_OFFSET + (self._dwarf - 1)

        # Build FC18 Read FIFO request
        data = [(self.FIFO_ADDR >> 8) & 0xFF, self.FIFO_ADDR & 0xFF]
        frame = self.puppy._build_modbus_frame(addr, 0x18, data)

        try:
            # Use puppy's _send_frame which properly handles MODBUS responses
            # Small pause to let reactor process - critical for timer callbacks
            self.reactor.pause(self.reactor.monotonic() + 0.001)

            response = self.puppy._send_frame(frame)

            # Validate response is a proper dict
            if not isinstance(response, dict):
                logging.debug("DwarfAccelerometer: Invalid response type: %s" %
                             type(response).__name__)
                return None

            status = response.get('status', -1)
            resp_data = response.get('data', b'')

            if status != 0:
                logging.debug("DwarfAccelerometer: FIFO read status=%d" % status)
                return None

            if not resp_data or len(resp_data) < 7:
                logging.debug("DwarfAccelerometer: FIFO response too short: %d bytes" %
                             (len(resp_data) if resp_data else 0))
                return None

            # FC18 response format:
            # [addr][fc][byte_count_hi][byte_count_lo][fifo_count_hi][fifo_count_lo][data...][crc]
            byte_count = (resp_data[2] << 8) | resp_data[3]
            fifo_count = (resp_data[4] << 8) | resp_data[5]

            if fifo_count == 0:
                return []

            # Extract FIFO data (skip header, exclude CRC)
            fifo_data = resp_data[6:6 + byte_count - 2]
            registers = []
            for i in range(0, len(fifo_data) - 1, 2):
                val = (fifo_data[i] << 8) | fifo_data[i + 1]
                registers.append(val)
            return registers

        except Exception as e:
            logging.debug("DwarfAccelerometer: FIFO read error: %s" % e)
            return None

    def _decode_fifo(self, registers):
        """Decode FIFO stream into accelerometer samples."""
        if not registers:
            return [], 0.0

        # Convert registers to byte array
        data = bytearray()
        for reg in registers:
            data.append((reg >> 8) & 0xFF)
            data.append(reg & 0xFF)

        samples = []
        measured_rate = self._measured_rate
        pos = 0

        while pos < len(data) - 1:
            msg_type = data[pos]
            pos += 1

            if msg_type == self.MSG_NO_DATA:
                continue
            elif msg_type == self.MSG_ACCEL_FAST:
                # Fast accelerometer data: 2 packed samples (4 bytes each)
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
                # Frequency report: 4 bytes float
                if pos + 4 > len(data):
                    break
                freq_bytes = bytes(data[pos:pos+4])
                measured_rate = struct.unpack('<f', freq_bytes)[0]
                self._measured_rate = measured_rate
                pos += 4
            elif msg_type == self.MSG_LOADCELL:
                # Loadcell data: 8 bytes (skip, accelerometer doesn't need this)
                if pos + 8 > len(data):
                    break
                pos += 8
            else:
                # Unknown message type - stop parsing
                break

        return samples, measured_rate

    def _unpack_sample(self, packed):
        """Unpack 32-bit accelerometer sample.

        Format from Prusa firmware (accelerometer.cpp):
        - bits 0-9: X axis (10-bit signed)
        - bits 10-19: Y axis (10-bit signed)
        - bits 20-29: Z axis (10-bit signed)
        - bit 30: buffer_overflow
        - bit 31: sample_overrun
        """
        buffer_overflow = (packed >> 30) & 1
        sample_overrun = (packed >> 31) & 1

        if buffer_overflow:
            self._overflows += 1
        if sample_overrun:
            self._errors += 1

        # Extract 10-bit values and sign extend to 16-bit
        x_raw = packed & 0x3FF
        if x_raw & 0x200:
            x_raw -= 0x400
        x = x_raw << 6  # Scale to 16-bit range

        y_raw = (packed >> 10) & 0x3FF
        if y_raw & 0x200:
            y_raw -= 0x400
        y = y_raw << 6

        z_raw = (packed >> 20) & 0x3FF
        if z_raw & 0x200:
            z_raw -= 0x400
        z = z_raw << 6

        # Swap X and Z for Dwarf orientation
        return (z, y, x)

    def _convert_samples(self, raw_samples):
        """Convert raw samples to physical units (mm/s^2)."""
        (x_pos, x_scale), (y_pos, y_scale), (z_pos, z_scale) = self._axes_map
        converted = []
        for raw_xyz in raw_samples:
            x = round(raw_xyz[x_pos] * x_scale, 6)
            y = round(raw_xyz[y_pos] * y_scale, 6)
            z = round(raw_xyz[z_pos] * z_scale, 6)
            sample_time = self._sample_time
            self._sample_time += 1.0 / self.SAMPLE_RATE
            converted.append((round(sample_time, 6), x, y, z))
        return converted

    def _pause_polling(self):
        """Pause Dwarf polling timer to avoid MODBUS contention."""
        if self.puppy.poll_timer is not None:
            self.reactor.update_timer(self.puppy.poll_timer, self.reactor.NEVER)
            logging.debug("DwarfAccelerometer: Paused Dwarf polling")

    def _resume_polling(self):
        """Resume Dwarf polling timer."""
        if self.puppy.poll_timer is not None and self.puppy.polling_active:
            waketime = self.reactor.monotonic() + 1.0
            self.reactor.update_timer(self.puppy.poll_timer, waketime)
            logging.debug("DwarfAccelerometer: Resumed Dwarf polling")

    # Klipper resonance_tester interface
    def start_internal_client(self):
        """Start measurement client for resonance_tester.

        CRITICAL: Accelerometer must be enabled and collecting BEFORE we capture
        request_start_time, otherwise sample timestamps won't overlap with the
        motion period and has_valid_samples() will return False.
        """
        # Step 1: Enable accelerometer and start collecting FIRST
        self._start_measurements()

        # Step 2: Small delay to ensure accelerometer is actually producing data
        # The LIS2DH12 needs a few sample periods to fill its FIFO
        self.reactor.pause(self.reactor.monotonic() + 0.05)

        # Step 3: Flush any stale data from before motion starts
        self._read_fifo()  # Discard - this clears old samples

        # Step 4: NOW create the query helper - its request_start_time will be
        # AFTER we've started collecting, ensuring timestamp overlap
        aqh = DwarfAccelQueryHelper(self.printer)
        self._clients.append(aqh.handle_batch)

        # Step 5: Sync sample_time with the query helper's start time
        self._sample_time = aqh.request_start_time

        logging.info("DwarfAccelerometer: Client started, sample_time=%.3f" % self._sample_time)
        return aqh

    def _start_measurements(self):
        """Start accelerometer data collection."""
        if self._collecting:
            return
        if not self._enabled:
            if not self.enable():
                raise self.printer.command_error("Failed to enable Dwarf accelerometer")
            # Allow accelerometer to initialize and start filling FIFO
            self.reactor.pause(self.reactor.monotonic() + 0.1)

        # CRITICAL: Pause regular Dwarf polling to avoid MODBUS contention
        self._pause_polling()

        self._collecting = True
        self._sample_time = self.printer.lookup_object('toolhead').get_last_move_time()
        self._errors = 0
        self._overflows = 0

        # Start batch collection timer
        self._batch_timer = self.reactor.register_timer(
            self._batch_callback, self.reactor.NOW)
        logging.info("DwarfAccelerometer: Started measurements on Dwarf %d" % self._dwarf)

    def _finish_measurements(self):
        """Stop accelerometer data collection."""
        if not self._collecting:
            return
        self._collecting = False
        self._clients.clear()

        if self._batch_timer is not None:
            self.reactor.unregister_timer(self._batch_timer)
            self._batch_timer = None

        self.disable()

        # CRITICAL: Resume regular Dwarf polling
        self._resume_polling()

        logging.info("DwarfAccelerometer: Finished (errors=%d, overflows=%d)" %
                     (self._errors, self._overflows))

    def _batch_callback(self, eventtime):
        """Timer callback to read FIFO and distribute samples to clients."""
        if not self._collecting:
            return self.reactor.NEVER

        try:
            registers = self._read_fifo()
            if registers is not None and len(registers) > 0:
                raw_samples, _ = self._decode_fifo(registers)
                if raw_samples:
                    converted = self._convert_samples(raw_samples)
                    if converted:
                        msg = {'data': converted, 'errors': self._errors,
                               'overflows': self._overflows}
                        # Distribute to clients, removing any that return False
                        self._clients = [c for c in self._clients if c(msg)]
        except Exception as e:
            logging.warning("DwarfAccelerometer: Batch callback error: %s" % e)

        # 50ms polling interval - fast enough for 1344Hz data with some margin
        return eventtime + 0.050

    # G-code commands
    def cmd_ACCEL_ENABLE(self, gcmd):
        """Enable accelerometer on current or specified Dwarf."""
        dwarf = gcmd.get_int('DWARF', None)
        if self.enable(dwarf):
            gcmd.respond_info("Accelerometer enabled on Dwarf %d" % self._dwarf)
        else:
            raise gcmd.error("Failed to enable accelerometer")

    def cmd_ACCEL_DISABLE(self, gcmd):
        """Disable accelerometer."""
        if self.disable():
            gcmd.respond_info("Accelerometer disabled")
        else:
            gcmd.respond_info("Accelerometer disable failed")

    def cmd_ACCEL_QUERY(self, gcmd):
        """Query accelerometer and show current values."""
        aclient = self.start_internal_client()
        self.printer.lookup_object('toolhead').dwell(1.0)
        aclient.finish_measurements()
        self._finish_measurements()

        values = aclient.get_samples()
        if not values:
            raise gcmd.error("No accelerometer measurements found")

        _, accel_x, accel_y, accel_z = values[-1]
        gcmd.respond_info("accel x=%.6f y=%.6f z=%.6f" % (accel_x, accel_y, accel_z))

    def cmd_ACCEL_MEASURE(self, gcmd):
        """Start/stop background accelerometer measurement."""
        import time
        if self._bg_client is None:
            self._bg_client = self.start_internal_client()
            gcmd.respond_info("Accelerometer measurements started")
            return

        name = gcmd.get("NAME", time.strftime("%Y%m%d_%H%M%S"))
        bg_client = self._bg_client
        self._bg_client = None
        bg_client.finish_measurements()
        self._finish_measurements()

        filename = "/tmp/accel-%s.csv" % name
        bg_client.write_to_file(filename)
        gcmd.respond_info("Writing accelerometer data to %s" % filename)

    def cmd_ACCEL_STATUS(self, gcmd):
        """Show accelerometer status."""
        status = "ENABLED" if self._enabled else "DISABLED"
        collecting = "COLLECTING" if self._collecting else "IDLE"
        dwarf = self._dwarf if self._dwarf else "none"
        gcmd.respond_info("Accelerometer: %s (%s), Dwarf: %s" % (status, collecting, dwarf))
        gcmd.respond_info("Sample rate: %.0f Hz (measured: %.1f Hz)" %
                          (self.SAMPLE_RATE, self._measured_rate))
        gcmd.respond_info("Errors: %d, Overflows: %d" % (self._errors, self._overflows))
