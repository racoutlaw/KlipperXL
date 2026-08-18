# KlipperXL - Dwarf loadcell sample recorder (Road B, step 1)
#
# Automatic Pressure Advance for the Prusa XL: host-side loadcell FIFO capture.
#
# WHY THIS IS SMALL: the Dwarf's loadcell samples already travel to the host in the
# SAME MODBUS FIFO the accelerometer uses. dwarf_accelerometer.py::_decode_fifo
# already recognises MSG_LOADCELL == 2 -- it just throws the record away:
#
#     elif msg_type == self.MSG_LOADCELL:
#         if pos + 8 > len(data):
#             break
#         pos += 8          # <-- timestamp(4) + raw(4), discarded
#
# So no MCU firmware change and no reflash are needed: we read the same FIFO the
# same way and decode the 8 bytes instead of skipping them. The MCU's
# modbus_read_loadcell command is deliberately NOT used -- it collapses the stream
# to last-value + count, which is useless for PA.
#
# This module is ADDITIVE and REVERSIBLE. It edits nothing; it looks puppy_bootloader
# up by name. Delete the file and remove [dwarf_pa_tuner] and it is gone.
#
# STEP 1 SCOPE: capture only. NO MOTION IS COMMANDED. This exists to answer the one
# question the whole design rests on -- can we sustain the loadcell sample rate
# host-side? Accel already polls 1344 Hz host-side; loadcell is ~320 Hz, so it should
# be comfortable. Verify before building the sweep on top of it.
#
# Copyright (C) 2026 Richard Crook
#
# Based on Klipper 3D Printer Firmware
#   Copyright (C) 2016-2024 Kevin O'Connor <kevin@koconnor.net>
# FIFO access patterned on KlipperXL's own dwarf_accelerometer.py.
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
import json
import logging
import os
import shutil
import struct
import subprocess
import time


class DwarfPATuner:
    # Coils are shared with the accelerometer -- the Dwarf feeds ONE FIFO, so the two
    # are mutually exclusive. puppy_bootloader does the same disable-accel-then-enable
    # -loadcell dance (puppy_bootloader.py:6396-6399).
    LOADCELL_COIL = 0x4002
    ACCEL_COIL = 0x4003
    FIFO_ADDR = 0x0000

    # FIFO record types (dwarf_accelerometer.py:118-122)
    MSG_NO_DATA = 0
    MSG_LOG = 1
    MSG_LOADCELL = 2
    MSG_ACCEL_FAST = 4
    MSG_ACCEL_FREQ = 5

    # grams per raw ADC unit. puppy_bootloader.py:1532 LOADCELL_SCALE, and
    # loadcell_probe.py:56 -- both 0.0192, "From Prusa: scale = 0.0192f".
    SCALE = 0.0192

    # The Dwarf loadcell is a 24-bit ADC, so a genuine reading always fits in
    # +/-2^23. Anything outside that is not a measurement.
    # MEASURED: the FIRST record after enabling the loadcell comes back as exactly
    # INT32_MIN (0x80000000) -- a stale/sentinel FIFO entry. In the polling path we
    # drained the FIFO before recording and never saw it; the MCU stream has no such
    # drain, so it arrives as sample 0 and, being ~2.1e9 counts out, it destroys the
    # tare and with it every value in the run.
    RAW_LIMIT = 1 << 23

    def __init__(self, config):
        self.printer = config.get_printer()
        self.name = config.get_name()
        self.gcode = self.printer.lookup_object('gcode')
        self.reactor = self.printer.get_reactor()
        # How often to poll the FIFO. The MCU's own loadcell_probe_task polls the
        # Dwarf FIFO every 3ms; matching that means we never leave samples to age in
        # the FIFO and risk an overflow.
        # 20ms. MEASURED 2026-08-15, 5s captures, and this one really matters:
        #   3ms  -> 52 Hz  : fifo read max 5023ms, 3 gaps. HAMMERS the Dwarf.
        #   10ms -> 51 Hz  : fifo read max 5012ms, still stalling
        #   15ms -> 40 Hz  : fifo read max 5019ms, still stalling
        #   20ms -> 194 Hz : fifo read max 11-12ms, ZERO gaps, ZERO errors (x2 runs)
        #   30ms -> 131 Hz : no stalls, but 161 of 655 deltas >12ms = dropping samples
        # The failure at short intervals is NOT reactor contention (measured reactor
        # gap: median 0.1ms, never >50ms). It is the Dwarf occasionally not answering
        # on RS485, which then costs a full 5s because _send_frame is a Klipper MCU
        # query and its wrapper retries to a 5.0s timeout. USB was clean throughout
        # (bytes_retransmit flat, bytes_invalid=0), so the MCU got the command fine.
        # Polling at 3ms wastes a whole RS485 transaction to collect ~1.8 samples;
        # at 20ms we collect 4.0 and the Dwarf keeps up.
        # CEILING: the FIFO returns at most 4 records per read, so the maximum
        # sustainable rate is 4/poll_interval = 200 Hz here. Going faster needs more
        # records per read, which is a firmware-side change. 194 Hz is already above
        # the ~180 Hz Stefan's analyzer takes on stock, and it resamples to 1kHz
        # internally, so this is enough.
        self.poll_interval = config.getfloat('poll_interval', 0.020, above=0.)
        self.read_retries = config.getint('read_retries', 1, minval=1)
        self.puppy = None
        self._samples = []          # (dwarf_ts_raw, raw_counts)
        self._reads = 0
        self._empty_reads = 0
        self._errors = 0
        self.printer.register_event_handler("klippy:connect", self._handle_connect)
        # Hard ceiling on how long a recording may run. If STOP is never reached --
        # a gcode error, an M112, a cancelled sweep -- the timer would otherwise keep
        # hammering the shared RS485 bus forever and starve the other Dwarfs' 30s
        # comms watchdog (a starved Dwarf resets, blue LED, then fails its next pick).
        # A FULL sweep is 26 K values x 10 cycles x 2.714s = ~12 MINUTES, so 180s
        # would have auto-killed it mid-run. Safe to allow now: in MCU stream mode we
        # no longer pause the unified Dwarf poll, so the other four toolheads keep
        # getting fed and their 30s comms watchdog is never at risk.
        self.max_record = config.getfloat('max_record_seconds', 1800., above=0.)
        self._recording = False
        self._rec_timer = None
        self._rec_dwarf = None
        self._rec_start = 0.
        self._rec_was_loadcell = False
        self.mcu = None
        self._stream_ok = False
        self._stream_start_cmd = None
        self._stream_stop_cmd = None
        self._mode = 'poll'
        self._paused_poll = False
        self._poll = self.poll_interval
        self._mcu_sent = 0
        self._mcu_batches = 0
        self._bad = 0
        self.gcode.register_command(
            'PA_RECORD_TEST', self.cmd_PA_RECORD_TEST,
            desc="Capture Dwarf loadcell samples host-side (no motion) and report rate")
        self.gcode.register_command(
            'START_PA_RECORDING', self.cmd_START_PA_RECORDING,
            desc="Begin buffering loadcell samples in the background (non-blocking)")
        self.gcode.register_command(
            'STOP_PA_RECORDING', self.cmd_STOP_PA_RECORDING,
            desc="End recording, restore bus state and write the samples out")
        self.gcode.register_command(
            'ANALYZE_PA', self.cmd_ANALYZE_PA,
            desc="Analyse the last PA recording and report K (SAVE=1 to store it)")
        # Analyser subprocess plumbing. It runs in its OWN venv (numpy+scipy), which
        # klippy-env deliberately does not have. Defaults are resolved against the
        # invoking user's home rather than a hardcoded one -- the Klipper user is
        # `pi` on some installs, `klipper` on others.
        home = os.path.expanduser('~')
        self.analysis_python = config.get(
            'analysis_python', os.path.join(home, 'pa_analysis_env/bin/python'))
        self.analysis_script = config.get(
            'analysis_script', os.path.join(home, 'pa_analysis/pa_analyze.py'))
        self._an_proc = None
        self._an_timer = None
        self._an_tool = -1
        self._an_save = False
        self._an_saveconfig = False
        self._an_label = ''
        logging.info("dwarf_pa_tuner: loaded (scale=%.4f g/count)" % (self.SCALE,))

    def _handle_connect(self):
        # Looked up by name so this module never has to be wired into
        # puppy_bootloader. tool_offsets.py resolves it the same way.
        self.puppy = self.printer.lookup_object('puppy_bootloader', None)
        if self.puppy is None:
            logging.warning("dwarf_pa_tuner: puppy_bootloader not found - "
                            "PA_RECORD_TEST will be unavailable")
            return
        # MCU-SIDE STREAMING (firmware loadcell_stream_* commands).
        # Host polling cannot survive extrusion: the Dwarf intermittently fails to
        # answer and each miss costs a flat 5s Klipper query timeout (~1 per 10-15s
        # while extruding, measured). The MCU polls this FIFO in C every 3ms and never
        # waits on a host query, so it is immune. If the firmware predates the stream
        # patch we fall back to polling rather than crashing.
        self.mcu = self.puppy.mcu
        try:
            self._stream_start_cmd = self.mcu.lookup_command(
                "loadcell_stream_start addr=%c")
            self._stream_stop_cmd = self.mcu.lookup_command(
                "loadcell_stream_stop")
            # NOTE: this Klipper exposes register_serial_response (NOT
            # register_response), and it takes the FULL message format string, the
            # same way adxl345/analog_in do:
            #   "analog_in_state oid=%c next_clock=%u values=%*s"
            # Keep the returned wrappers referenced so they are not garbage collected.
            self._resp_data = self.mcu.register_serial_response(
                self._handle_stream_data, "loadcell_stream_data data=%*s")
            self._resp_result = self.mcu.register_serial_response(
                self._handle_stream_result,
                "loadcell_stream_result sent=%u batches=%u")
            self._stream_ok = True
            logging.info("dwarf_pa_tuner: MCU stream mode AVAILABLE")
        except Exception as e:
            self._stream_ok = False
            logging.warning("dwarf_pa_tuner: MCU stream mode NOT available (%s) - "
                            "falling back to host polling" % (e,))

    # ---- MCU stream consumers ------------------------------------------------
    def _handle_stream_data(self, params):
        """Batch of (dwarf_ts u32 LE, raw i32 LE) pairs pushed by the MCU."""
        if not self._recording:
            return
        data = params.get('data', b'')
        n = len(data) // 8
        for i in range(n):
            off = i * 8
            ts = struct.unpack('<I', bytes(data[off:off + 4]))[0]
            raw = struct.unpack('<i', bytes(data[off + 4:off + 8]))[0]
            if raw > self.RAW_LIMIT or raw < -self.RAW_LIMIT:
                self._bad += 1          # stale/sentinel record -- see RAW_LIMIT
                continue
            self._samples.append((ts, raw))
        self._reads += 1

    def _handle_stream_result(self, params):
        self._mcu_sent = params.get('sent', 0)
        self._mcu_batches = params.get('batches', 0)

    # ---- FIFO plumbing (mirrors dwarf_accelerometer.py so the byte stream we see
    # ---- is byte-for-byte the one its decoder sees) --------------------------
    def _read_fifo(self, dwarf):
        """MODBUS FC 0x18 FIFO read. Returns a register list, [] if empty, None on error."""
        addr = self.puppy.ADDR_MODBUS_OFFSET + dwarf
        data = [(self.FIFO_ADDR >> 8) & 0xFF, self.FIFO_ADDR & 0xFF]
        frame = self.puppy._build_modbus_frame(addr, 0x18, data)
        # RETRIES ARE EXPENSIVE HERE. _send_frame is a Klipper MCU *query*; its
        # wrapper retries every 500ms and gives up at 5.0s. Measured: one read in
        # ~174 blocks the full 5023ms while USB stays clean (bytes_retransmit flat,
        # bytes_invalid=0), i.e. the MCU got the command but the Dwarf did not answer
        # on RS485. At 3 attempts that is a 15s worst case, which would destroy a
        # sweep. For streaming, one attempt and move on -- a missed FIFO read costs a
        # few samples; a 15s stall costs the whole run.
        for _attempt in range(self.read_retries):
            try:
                response = self.puppy._send_frame(frame)
                if not isinstance(response, dict):
                    continue
                if response.get('status', -1) != 0:
                    continue
                resp_data = response.get('data', b'')
                if not resp_data or len(resp_data) < 7:
                    continue
                byte_count = (resp_data[2] << 8) | resp_data[3]
                fifo_count = (resp_data[4] << 8) | resp_data[5]
                if fifo_count == 0:
                    return []
                fifo_data = resp_data[6:6 + byte_count - 2]
                registers = []
                for i in range(0, len(fifo_data) - 1, 2):
                    registers.append((fifo_data[i] << 8) | fifo_data[i + 1])
                return registers
            except Exception as e:
                logging.debug("dwarf_pa_tuner: FIFO read error: %s" % (e,))
                continue
        self._errors += 1
        return None

    def _decode_loadcell(self, registers):
        """Walk the FIFO exactly as _decode_fifo does, but KEEP the loadcell records.

        Record layout (modbus_stm32f4.c:1208): type(1) + timestamp(4 LE) + raw(4 LE).
        raw is SIGNED -- compression reads negative (loadcell_probe uses a -2083 raw
        threshold for ~40g, and -2083 * 0.0192 = -40.0g).
        """
        if not registers:
            return 0
        data = bytearray()
        for reg in registers:
            data.append(reg & 0xFF)
            data.append((reg >> 8) & 0xFF)
        got = 0
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
                ts = struct.unpack('<I', bytes(data[pos:pos + 4]))[0]
                raw = struct.unpack('<i', bytes(data[pos + 4:pos + 8]))[0]
                if raw > self.RAW_LIMIT or raw < -self.RAW_LIMIT:
                    self._bad += 1      # 24-bit ADC: out of range = not a reading
                else:
                    self._samples.append((ts, raw))
                    got += 1
                pos += 8
            elif msg_type == self.MSG_ACCEL_FAST:
                if pos + 8 > len(data):
                    break
                pos += 8               # 2 packed accel samples; not our business
            elif msg_type == self.MSG_ACCEL_FREQ:
                if pos + 4 > len(data):
                    break
                pos += 4
            else:
                break                  # unknown type -> stream desync, stop
        return got

    # ---- background recording (this is the one the sweep actually uses) -------
    #
    # PA_RECORD_TEST blocks the whole time it captures, which is fine for a bench
    # check but useless for a sweep: the extrusion moves have to RUN while we record.
    # So recording is a reactor timer that fires every poll_interval, drains the FIFO
    # and buffers, while the gcode queue proceeds independently.
    def _begin(self, gcmd, dwarf, mode):
        self._samples = []
        self._reads = 0
        self._empty_reads = 0
        self._errors = 0
        self._mcu_sent = 0
        self._mcu_batches = 0
        self._bad = 0
        self._rec_dwarf = dwarf
        self._mode = mode
        self._rec_was_loadcell = self.puppy.loadcell_enabled.get(dwarf, False)

        if mode == 'poll':
            # Legacy host-polling path. Kept for A/B comparison only -- it cannot
            # survive extrusion (see poll_interval notes). Must quiet the unified
            # Dwarf poll because we are competing for the shared bus ourselves.
            self.puppy._pause_polling_timer()
            self._paused_poll = True
        else:
            # MCU stream: the firmware owns the FIFO reads, so we are NOT competing
            # for the bus and must NOT pause the unified poll -- leaving it running
            # keeps the other four Dwarfs' 30s comms watchdog fed. That matters: a
            # full sweep is ~12 minutes, far past the watchdog, and starved Dwarfs
            # reset (blue LED) and then fail their next pick.
            self._paused_poll = False

        # accel and loadcell share the FIFO, so accel goes down first
        self.puppy._write_coil(dwarf, self.ACCEL_COIL, False)
        self.reactor.pause(self.reactor.monotonic() + 0.05)
        if not self._rec_was_loadcell:
            self.puppy._write_coil(dwarf, self.LOADCELL_COIL, True)
            self.puppy.loadcell_enabled[dwarf] = True
            self.reactor.pause(self.reactor.monotonic() + 0.05)

        self._recording = True
        self._rec_start = self.reactor.monotonic()
        if mode == 'poll':
            self._read_fifo(dwarf)          # drop stale queue
            self._samples = []
            self._rec_timer = self.reactor.register_timer(
                self._rec_callback, self.reactor.NOW)
        else:
            # The MCU uses this value DIRECTLY as the MODBUS frame address
            # (modbus_stm32f4.c: frame[0] = loadcell_probe.dwarf_addr), so it must be
            # ADDR_MODBUS_OFFSET + dwarf -- the same thing every other loadcell
            # command sends. Passing the bare dwarf number yields zero samples.
            self._stream_start_cmd.send([self.puppy.ADDR_MODBUS_OFFSET + dwarf])

    def _rec_callback(self, eventtime):
        if not self._recording:
            return self.reactor.NEVER
        if eventtime - self._rec_start > self.max_record:
            # safety stop -- see max_record_seconds
            logging.warning("dwarf_pa_tuner: recording exceeded %.0fs, auto-stopping"
                            % (self.max_record,))
            self._end()
            return self.reactor.NEVER
        regs = self._read_fifo(self._rec_dwarf)
        self._reads += 1
        if regs is None:
            pass
        elif not regs:
            self._empty_reads += 1
        else:
            self._decode_loadcell(regs)
        return eventtime + getattr(self, '_poll', self.poll_interval)

    def _end(self):
        """Idempotent teardown -- safe to call twice, safe to call from the timer."""
        was_recording = self._recording
        self._recording = False
        if getattr(self, '_mode', 'poll') != 'poll' and was_recording:
            try:
                self._stream_stop_cmd.send()
            except Exception as e:
                logging.warning("dwarf_pa_tuner: stream stop failed: %s" % (e,))
        if self._rec_timer is not None:
            try:
                self.reactor.unregister_timer(self._rec_timer)
            except Exception:
                pass
            self._rec_timer = None
        if self._rec_dwarf is not None:
            # leave the loadcell as we found it; puppy normally keeps it on for probing
            if not self._rec_was_loadcell:
                try:
                    self.puppy._write_coil(self._rec_dwarf, self.LOADCELL_COIL, False)
                    self.puppy.loadcell_enabled[self._rec_dwarf] = False
                except Exception:
                    pass
            self._rec_dwarf = None
        if getattr(self, '_paused_poll', False):
            try:
                self.puppy._resume_polling_timer(0.5)
            except Exception:
                pass
            self._paused_poll = False

    def cmd_START_PA_RECORDING(self, gcmd):
        if self.puppy is None:
            raise gcmd.error("START_PA_RECORDING: puppy_bootloader not available")
        if self._recording:
            raise gcmd.error("START_PA_RECORDING: already recording")
        tool = self.puppy.active_tool
        if tool is None or tool < 0:
            raise gcmd.error("START_PA_RECORDING: no tool picked - pick T0-T4 first "
                             "(the loadcell is per-Dwarf)")
        dwarf = gcmd.get_int('DWARF', tool + 1)
        if dwarf not in self.puppy.booted_dwarfs:
            raise gcmd.error("START_PA_RECORDING: Dwarf %d not booted" % (dwarf,))
        # Runtime override. The stall-free interval measured at IDLE (20ms) is not
        # necessarily right during MOTION -- the bus is busier and the Dwarf misses
        # more reads. Sweepable without a restart so we can find the motion-safe value.
        self._poll = gcmd.get_float('POLL', self.poll_interval, above=0., maxval=0.2)
        # MODE=stream (default when the firmware supports it) or MODE=poll to force
        # the old host-polling path for comparison.
        mode = gcmd.get('MODE', 'stream' if self._stream_ok else 'poll').lower()
        if mode == 'stream' and not self._stream_ok:
            raise gcmd.error("START_PA_RECORDING: MCU stream mode unavailable - "
                             "firmware does not have loadcell_stream_start "
                             "(flash the stream build, or use MODE=poll)")
        if mode not in ('stream', 'poll'):
            raise gcmd.error("START_PA_RECORDING: MODE must be stream or poll")
        self._begin(gcmd, dwarf, mode)
        if mode == 'stream':
            gcmd.respond_info("PA recording started (T%d / Dwarf %d, MCU STREAM, "
                              "unified poll left running)" % (dwarf - 1, dwarf))
        else:
            gcmd.respond_info("PA recording started (T%d / Dwarf %d, host poll %.0fms)"
                              % (dwarf - 1, dwarf, self._poll * 1000.))

    def cmd_STOP_PA_RECORDING(self, gcmd):
        if not self._recording:
            raise gcmd.error("STOP_PA_RECORDING: not recording")
        elapsed = self.reactor.monotonic() - self._rec_start
        dwarf = self._rec_dwarf
        self._end()
        n = len(self._samples)
        if not n:
            gcmd.respond_info("STOP_PA_RECORDING: no samples captured in %.2fs"
                              % (elapsed,))
            return
        raws = [s[1] for s in self._samples]
        ts = [s[0] for s in self._samples]
        # Tare off the head of the recording. The sweep's pre-roll dwell after the
        # Z-marker is deliberately quiet, so the opening samples are a good baseline.
        n_tare = max(1, min(n // 20, 100))
        tare = float(sum(raws[:n_tare])) / n_tare
        t0 = ts[0]
        # Dwarf timestamps are microseconds; emit seconds relative to the first sample
        # so the analyzer gets a clean monotonic time base.
        # Keep EVERY recording. Writing them all to one path destroyed T0's raw trace
        # the moment T1 was swept -- the K survived but the data behind it did not, so
        # the two runs could no longer be compared. Per-tool timestamped file is the
        # real artifact; /tmp/pa_recording.csv stays as a "latest" convenience copy so
        # ANALYZE_PA with no CSV= argument still does the obvious thing.
        stamp = time.strftime("%Y%m%d-%H%M%S")
        path = "/tmp/pa_recording_T%d_%s.csv" % (dwarf - 1 if dwarf else -1, stamp)
        latest = "/tmp/pa_recording.csv"
        try:
            with open(path, "w") as f:
                f.write("# tool=T%d dwarf=%d samples=%d elapsed=%.3f tare=%.1f "
                        "scale=%.4f\n" % (dwarf - 1 if dwarf else -1, dwarf or -1,
                                          n, elapsed, tare, self.SCALE))
                for k in gcmd.get_command_parameters().items():
                    f.write("# param %s=%s\n" % k)
                f.write("time_s,grams\n")
                for tsv, raw in self._samples:
                    f.write("%.6f,%.4f\n" % ((tsv - t0) * 1e-6,
                                             (raw - tare) * self.SCALE))
            shutil.copyfile(path, latest)
        except Exception as e:
            gcmd.respond_info("STOP_PA_RECORDING: WRITE FAILED: %s" % (e,))
            return
        span = ts[-1] - ts[0]
        rate = (n - 1) / (span * 1e-6) if span > 0 else 0.
        tg = [(r - tare) * self.SCALE for r in raws]
        # gap check -- the whole point of the MCU stream is that these vanish
        gaps = [ts[i + 1] - ts[i] for i in range(len(ts) - 1)]
        big = sum(1 for d in gaps if d > 100000)      # >100ms = a real hole
        worst = max(gaps) if gaps else 0
        gcmd.respond_info(
            "PA recording stopped [%s]: %d samples in %.2fs (%.1f Hz)\n"
            "  tared grams: min=%.2f max=%.2f\n"
            "  ts gaps: worst=%.0fms  (>100ms: %d of %d)\n"
            "  batches=%d mcu_sent=%d errors=%d rejected=%d\n"
            "  written to %s"
            % (self._mode, n, elapsed, rate, min(tg), max(tg),
               worst / 1000., big, len(gaps),
               self._reads, self._mcu_sent, self._errors, self._bad, path))

    # ---- analysis ------------------------------------------------------------
    #
    # MUST be asynchronous. Fitting 200k+ samples takes tens of seconds, and blocking
    # Klipper's single-threaded reactor that long trips its timing watchdogs and can
    # take the MCU down. So: spawn the analyser as a detached process and poll it from
    # a reactor timer, exactly the pattern KAPAT uses.
    def cmd_ANALYZE_PA(self, gcmd):
        if self._an_proc is not None:
            raise gcmd.error("ANALYZE_PA: an analysis is already running")
        csv = gcmd.get('CSV', '/tmp/pa_recording.csv')
        if not os.path.exists(csv):
            raise gcmd.error("ANALYZE_PA: no recording at %s" % (csv,))
        self._an_save = bool(gcmd.get_int('SAVE', 0))
        # SAVECONFIG=1 issues SAVE_CONFIG once the K has been stored, which persists
        # it and restarts Klipper. It CANNOT be a separate line in a macro: the
        # analyser runs as a background subprocess, so a macro would reach SAVE_CONFIG
        # while the analysis was still running and save nothing.
        self._an_saveconfig = bool(gcmd.get_int('SAVECONFIG', 0))
        tool = gcmd.get_int('TOOL', -1)
        if tool < 0:
            tool = self.puppy.active_tool if self.puppy is not None else -1
        self._an_tool = tool
        self._an_label = gcmd.get('FILAMENT', '')
        try:
            self._an_proc = subprocess.Popen(
                [self.analysis_python, self.analysis_script, csv],
                cwd=os.path.dirname(self.analysis_script) or '/tmp',
                stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
        except Exception as e:
            self._an_proc = None
            raise gcmd.error("ANALYZE_PA: could not start analyser: %s" % (e,))
        gcmd.respond_info("Analysing %s in the background (T%d)%s ..."
                          % (csv, tool, " and will SAVE" if self._an_save else ""))
        self._an_timer = self.reactor.register_timer(
            self._an_callback, self.reactor.monotonic() + 1.0)

    def _an_callback(self, eventtime):
        proc = self._an_proc
        if proc is None:
            return self.reactor.NEVER
        if proc.poll() is None:
            return eventtime + 1.0          # still working
        out = b''
        try:
            out = proc.stdout.read() or b''
        except Exception:
            pass
        rc = proc.returncode
        self._an_proc = None
        if self._an_timer is not None:
            try:
                self.reactor.unregister_timer(self._an_timer)
            except Exception:
                pass
            self._an_timer = None
        if rc != 0:
            tail = out.decode('utf-8', 'replace').strip().splitlines()[-6:]
            self.gcode.respond_info("ANALYZE_PA FAILED (rc=%d):\n  %s"
                                    % (rc, "\n  ".join(tail)))
            return self.reactor.NEVER
        try:
            with open('/tmp/pa_result.json') as fh:
                res = json.load(fh)
        except Exception as e:
            self.gcode.respond_info("ANALYZE_PA: could not read result: %s" % (e,))
            return self.reactor.NEVER
        self._report_result(res)
        return self.reactor.NEVER

    def _report_result(self, res):
        # bd_k_opt is the analyser's headline answer. Everything else is reported so
        # a bad fit is visible rather than silently trusted -- on the first real sweep
        # phase_fit returned a confident-looking 0.0652 with r2=0.008, i.e. noise.
        k = res.get('bd_k_opt')
        lines = ["PA analysis:"]
        lines.append("  bd_k_opt            %s   <-- headline" % (k,))
        # Show every method with its r^2 so a bad fit is obvious. A high k_opt with a
        # near-zero r^2 is noise dressed up as an answer.
        for fname in ('integral_fit', 'bd_signed_rise_fit', 'bd_signed_fall_fit',
                      'integral_legacy_fit', 'phase_fit'):
            fit = res.get(fname)
            if not isinstance(fit, dict):
                continue
            r2 = fit.get('r_squared')
            flag = ""
            if isinstance(r2, float):
                flag = "  <-- strong" if r2 >= 0.9 else (
                    "  <-- IGNORE, no fit" if r2 < 0.2 else "")
            lines.append("  %-19s k=%s  r2=%s%s"
                         % (fname, fit.get('k_opt'), r2, flag))
        seg_n = res.get('bd_k_from_segments_n')
        if seg_n:
            lines.append("  segments            %s used, per-segment k=%s (MAD %s)"
                         % (seg_n, res.get('bd_k_from_segments'),
                            res.get('bd_k_from_segments_mad')))
        if 'sample_rate_hz' in res:
            lines.append("  sample rate         %s Hz" % (res['sample_rate_hz'],))
        self.gcode.respond_info("\n".join(lines))
        if k is None:
            self.gcode.respond_info(
                "  no K_opt -- usually too few cycles per K (needs >=4 INCLUDED "
                "segments; use CYCLES=6 or more)")
            return
        if not self._an_save:
            self.gcode.respond_info(
                "  not saved. Re-run with SAVE=1, or: SAVE_TOOL_PA TOOL=%d K=%.4f"
                % (self._an_tool, k))
            return
        if self._an_tool is None or self._an_tool < 0:
            self.gcode.respond_info("  SAVE requested but no tool known - not saved")
            return
        cmd = "SAVE_TOOL_PA TOOL=%d K=%.4f" % (self._an_tool, k)
        if self._an_label:
            cmd += " FILAMENT=%s" % (self._an_label,)
        self.gcode.run_script_from_command(cmd)
        if self._an_saveconfig:
            self.gcode.respond_info("  persisting and restarting (SAVE_CONFIG)...")
            self.gcode.run_script_from_command("SAVE_CONFIG")
        else:
            # The K is already LIVE in memory, so the tool is using it right now and
            # can be print-tested before committing. Make the reminder impossible to
            # miss -- an unsaved value silently reverts on the next restart.
            self.gcode.respond_info(
                "\n"
                "  ============================================================\n"
                "    T%d PA = %.4f is ACTIVE NOW but NOT YET SAVED.\n"
                "    It will be LOST on the next restart.\n"
                "    Run  SAVE_CONFIG  to keep it (Klipper will restart).\n"
                "  ============================================================"
                % (self._an_tool, k))

    # ---- command -------------------------------------------------------------
    def cmd_PA_RECORD_TEST(self, gcmd):
        if self.puppy is None:
            raise gcmd.error("PA_RECORD_TEST: puppy_bootloader not available")
        seconds = gcmd.get_float('SECONDS', 2., above=0., maxval=30.)
        # Runtime override so the poll rate can be swept without a restart. Polling
        # every 3ms yields only ~1.8 samples/read because the Dwarf produces one per
        # ~3.1ms -- so most reads are nearly empty while still costing a full RS485
        # transaction. Slower polling collects more records per read and puts far less
        # load on the shared bus; the FIFO buffers, so the sample rate should hold.
        poll = gcmd.get_float('POLL', self.poll_interval, above=0., maxval=0.2)

        # THE #1 OPERATIONAL RULE ON THE XL (learned the hard way on stock, 2026-07-20):
        # the loadcell is PER-DWARF and is gated on a tool actually being coupled. With
        # no tool picked the metric silently produces nothing and it looks like the
        # firmware lacks the feature. So refuse rather than report a confusing zero.
        tool = self.puppy.active_tool
        if tool is None or tool < 0:
            raise gcmd.error("PA_RECORD_TEST: no tool picked - pick T0-T4 first "
                             "(the loadcell is per-Dwarf and only streams for the "
                             "coupled tool)")
        dwarf = gcmd.get_int('DWARF', tool + 1)
        if dwarf not in self.puppy.booted_dwarfs:
            raise gcmd.error("PA_RECORD_TEST: Dwarf %d not booted" % (dwarf,))

        self._samples = []
        self._reads = 0
        self._empty_reads = 0
        self._errors = 0

        was_loadcell = self.puppy.loadcell_enabled.get(dwarf, False)
        paused = False
        try:
            # Single shared RS485 bus: stop the unified poll while we hammer the FIFO,
            # exactly as the accelerometer does. NOTE the 30s Dwarf comms watchdog --
            # a starved Dwarf resets (blue LED) and then fails its next pick, which is
            # why SECONDS is capped well under it.
            self.puppy._pause_polling_timer()
            paused = True

            # accel and loadcell share the FIFO, so the accel coil must go down first
            # (puppy_bootloader.py:6396-6399 does the same order).
            self.puppy._write_coil(dwarf, self.ACCEL_COIL, False)
            self.reactor.pause(self.reactor.monotonic() + 0.05)
            if not was_loadcell:
                self.puppy._write_coil(dwarf, self.LOADCELL_COIL, True)
                self.puppy.loadcell_enabled[dwarf] = True
                self.reactor.pause(self.reactor.monotonic() + 0.05)

            # drain whatever is already sitting in the FIFO so it does not pollute the
            # rate figure with a burst of stale samples
            self._read_fifo(dwarf)
            self._samples = []

            start = self.reactor.monotonic()
            end = start + seconds
            now = start
            # INSTRUMENTATION. A multi-second hole in the sample stream has exactly
            # two possible causes and they need opposite fixes, so measure both
            # separately instead of guessing:
            #   read_times -> how long the MODBUS/USB transaction itself blocks
            #   idle_times -> how long between our read finishing and us being
            #                 scheduled again (i.e. reactor preemption)
            read_times = []
            idle_times = []
            while now < end:
                t0 = self.reactor.monotonic()
                regs = self._read_fifo(dwarf)
                t1 = self.reactor.monotonic()
                read_times.append(t1 - t0)
                self._reads += 1
                if regs is None:
                    pass
                elif not regs:
                    self._empty_reads += 1
                else:
                    self._decode_loadcell(regs)
                now = self.reactor.pause(now + poll)
                idle_times.append(self.reactor.monotonic() - t1)
            elapsed = self.reactor.monotonic() - start
        finally:
            # leave the loadcell exactly as we found it -- puppy normally keeps it on
            # for probing, so do not switch it off underneath the probe
            if not was_loadcell:
                try:
                    self.puppy._write_coil(dwarf, self.LOADCELL_COIL, False)
                    self.puppy.loadcell_enabled[dwarf] = False
                except Exception:
                    pass
            if paused:
                self.puppy._resume_polling_timer(0.5)

        n = len(self._samples)
        if not n:
            gcmd.respond_info(
                "PA_RECORD_TEST: NO loadcell samples in %.2fs (T%d/Dwarf %d)\n"
                "  reads=%d empty=%d errors=%d\n"
                "  The FIFO is being read but no MSG_LOADCELL records arrived."
                % (elapsed, dwarf - 1, dwarf, self._reads, self._empty_reads,
                   self._errors))
            return

        raws = [s[1] for s in self._samples]
        grams = [r * self.SCALE for r in raws]
        ts = [s[0] for s in self._samples]
        # Consecutive Dwarf-timestamp deltas. This distinguishes the two ways a low
        # capture rate can happen:
        #   uniform ~16000us  -> the Dwarf really only produces ~62Hz here
        #   bursty ~3000us with big gaps -> it produces ~320Hz and we are LOSING
        #                                   samples between FIFO reads
        # The firmware comment at modbus_stm32f4.c:1385 says 320Hz is expected
        # ("allows ~10 samples per poll at 320Hz"), so this matters.
        deltas = sorted(ts[i + 1] - ts[i] for i in range(len(ts) - 1)) \
            if len(ts) > 1 else []
        if deltas:
            dmin = deltas[0]
            dmed = deltas[len(deltas) // 2]
            dmax = deltas[-1]
            small = sum(1 for d in deltas if d < 6000)     # ~320Hz-ish spacing
            big = sum(1 for d in deltas if d > 12000)      # a gap = lost samples
            delta_line = ("  ts delta us: min=%d median=%d max=%d  "
                          "(<6ms: %d, >12ms: %d of %d)\n"
                          % (dmin, dmed, dmax, small, big, len(deltas)))
        else:
            delta_line = ""
        # TARE. The Dwarf loadcell is a 24-bit ADC with a large standing offset (raw
        # sits around 815,000 at rest), so raw*SCALE alone reads ~15.6 kg. Everything
        # downstream cares only about CHANGE in force, so subtract a baseline -- the
        # same thing loadcell_probe.py does with its tare_offset.
        # Taring off the head of the capture also hands us the NOISE FLOOR, which is
        # what decides whether a PA signal is detectable at all.
        def _stat(vals, label):
            if not vals:
                return "  %s: (none)\n" % (label,)
            sv = sorted(vals)
            over = sum(1 for v in sv if v > 0.05)      # >50ms is a real stall
            return ("  %s: median=%.1fms max=%.0fms  (>50ms: %d of %d)\n"
                    % (label, sv[len(sv) // 2] * 1000., sv[-1] * 1000.,
                       over, len(sv)))
        timing_lines = (_stat(read_times, "fifo read ") +
                        _stat(idle_times, "reactor gap"))
        n_tare = max(1, min(len(raws) // 5, 100))
        tare = float(sum(raws[:n_tare])) / n_tare
        tg = [(r - tare) * self.SCALE for r in raws]
        mean_tg = sum(tg) / len(tg)
        sd = (sum((g - mean_tg) ** 2 for g in tg) / len(tg)) ** 0.5
        # Dwarf timestamps are the real thing (unlike accel, which reconstructs them
        # from a fixed rate) -- so we can measure the true on-Dwarf sample interval.
        span = ts[-1] - ts[0]
        ts_rate = (n - 1) / (span * 1e-6) if span > 0 else 0.
        gcmd.respond_info(
            "PA_RECORD_TEST: %d samples in %.2fs  (T%d / Dwarf %d)\n"
            "  host rate   : %.1f Hz\n"
            "  dwarf ts rate: %.1f Hz  (span=%d ts units)\n"
            "%s"
            "  raw   : min=%d max=%d  (UNTARED, 24-bit ADC has a large offset)\n"
            "  tare  : %.1f counts  (mean of the first %d samples)\n"
            "  tared grams: min=%.3f max=%.3f  p2p=%.3f  noise sd=%.4f g\n"
            "%s"
            "  reads=%d empty=%d errors=%d  (%.1f samples/read)"
            % (n, elapsed, dwarf - 1, dwarf, n / elapsed if elapsed else 0.,
               ts_rate, span, delta_line, min(raws), max(raws),
               tare, n_tare, min(tg), max(tg), max(tg) - min(tg), sd,
               timing_lines, self._reads, self._empty_reads,
               self._errors, float(n) / self._reads if self._reads else 0.))


def load_config(config):
    return DwarfPATuner(config)
