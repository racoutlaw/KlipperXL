# XlKlipper - Loadcell Probe Module for Prusa XL
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
#
# Implements real-time loadcell probing using the Dwarf's loadcell sensor.
# Uses MCU-level monitoring for fast trigger detection.
#
# Two modes:
#   Z mode: Detects compression (negative load) for bed probing
#   XY mode: Detects absolute load (tension/compression) for tool calibration

import logging
import math


class LoadcellProbe:
    """
    Loadcell-based probe for Prusa XL toolchanger.

    Uses the Dwarf's loadcell to detect contact during probing.
    MCU monitors loadcell FIFO and triggers when threshold exceeded.

    Config:
        [loadcell_probe]
        dwarf_address: 0x1A          # MODBUS address (T0=0x1A, T1=0x1B, etc.)
        z_threshold: -80.0           # Z probe threshold (negative = compression)
        xy_threshold: 40.0           # XY probe threshold (absolute value)
        speed: 3.0                   # Probe speed mm/s
        lift_speed: 10.0             # Retract speed mm/s
        samples: 3                   # Number of samples per point
        sample_retract_dist: 2.0     # Retract distance between samples
        samples_tolerance: 0.100     # Allowed variation between samples
        tare_samples: 48             # Samples for tare averaging
        z_offset: 0.0                # Z offset adjustment
    """

    # Loadcell scale factor (raw ADC to grams)
    # From Prusa: scale = 0.0192f
    SCALE = 0.0192

    # Default thresholds matching Prusa (in grams)
    DEFAULT_Z_THRESHOLD = -80.0     # Z probing (negative = compression)
    DEFAULT_XY_THRESHOLD = 40.0     # XY probing (absolute value)

    # Hysteresis for trigger reset
    Z_HYSTERESIS = 80.0   # grams
    XY_HYSTERESIS = 20.0  # grams

    def __init__(self, config):
        self.printer = config.get_printer()
        self.reactor = self.printer.get_reactor()
        self.gcode = self.printer.lookup_object('gcode')
        self.name = config.get_name()

        # Configuration
        self.dwarf_address = config.getint('dwarf_address', 0x1A)
        self.z_threshold = config.getfloat('z_threshold', self.DEFAULT_Z_THRESHOLD)
        self.xy_threshold = config.getfloat('xy_threshold', self.DEFAULT_XY_THRESHOLD)
        self.speed = config.getfloat('speed', 3.0, minval=0.1)
        self.lift_speed = config.getfloat('lift_speed', 10.0, minval=0.1)
        self.samples = config.getint('samples', 3, minval=1)
        self.sample_retract_dist = config.getfloat('sample_retract_dist', 2.0,
                                                    minval=0.0)
        self.samples_tolerance = config.getfloat('samples_tolerance', 0.100,
                                                  minval=0.0)
        self.tare_samples = config.getint('tare_samples', 48, minval=8)
        self.z_offset = config.getfloat('z_offset', 0.0)

        # Convert thresholds to raw ADC units
        self.z_threshold_raw = int(self.z_threshold / self.SCALE)
        self.xy_threshold_raw = int(self.xy_threshold / self.SCALE)

        # State
        self.last_z_result = None
        self.last_state = {}
        self.mcu = None
        self.probe_start_cmd = None
        self.probe_stop_cmd = None
        self.probe_query_cmd = None
        self.tare_cmd = None

        # Event handlers
        self.printer.register_event_handler("klippy:connect",
                                            self._handle_connect)
        self.printer.register_event_handler("klippy:ready",
                                            self._handle_ready)

        # Register GCode commands
        self.gcode.register_command('LOADCELL_TARE', self.cmd_LOADCELL_TARE,
                                    desc="Tare the loadcell (zero offset)")
        self.gcode.register_command('LOADCELL_STATUS', self.cmd_LOADCELL_STATUS,
                                    desc="Show loadcell probe status")
        self.gcode.register_command('LOADCELL_PROBE', self.cmd_LOADCELL_PROBE,
                                    desc="Perform Z probe with loadcell")
        self.gcode.register_command('LOADCELL_PROBE_XY', self.cmd_LOADCELL_PROBE_XY,
                                    desc="Perform XY probe with loadcell")

        logging.info("LoadcellProbe: initialized for Dwarf 0x%02X, "
                     "Z threshold=%.1fg, XY threshold=%.1fg",
                     self.dwarf_address, self.z_threshold, self.xy_threshold)

    def _handle_connect(self):
        """Called when Klipper connects to MCU."""
        self.mcu = self.printer.lookup_object('mcu')

        # Register MCU commands - note xy=%c parameter added
        self.probe_start_cmd = self.mcu.lookup_command(
            "loadcell_probe_start addr=%c threshold=%i halt=%c xy=%c")

        self.probe_stop_cmd = self.mcu.lookup_query_command(
            "loadcell_probe_stop",
            "loadcell_probe_result triggered=%c load=%i time=%u count=%u "
            "min=%i max=%i errors=%u xy=%c")

        self.probe_query_cmd = self.mcu.lookup_query_command(
            "loadcell_probe_query",
            "loadcell_probe_state triggered=%c load=%i count=%u monitoring=%c")

        self.tare_cmd = self.mcu.lookup_query_command(
            "loadcell_tare addr=%c samples=%c",
            "loadcell_tare_result offset=%i samples=%c")

        logging.info("LoadcellProbe: MCU commands registered")

    def _handle_ready(self):
        """Called when Klipper is ready."""
        # Initial tare
        try:
            self.tare()
            logging.info("LoadcellProbe: initial tare complete")
        except Exception as e:
            logging.warning("LoadcellProbe: initial tare failed: %s", e)

    def _load_to_grams(self, raw_load):
        """Convert raw ADC value to grams."""
        return raw_load * self.SCALE

    def _grams_to_raw(self, grams):
        """Convert grams to raw ADC value."""
        return int(grams / self.SCALE)

    def set_dwarf_address(self, address):
        """Set the MODBUS address for the active Dwarf."""
        self.dwarf_address = address

    def tare(self, samples=None):
        """
        Zero the loadcell offset on current load.

        Should be called before probing when the nozzle is free (not touching
        anything). This establishes the baseline for threshold detection.

        Args:
            samples: Number of samples to average (default: self.tare_samples)

        Returns:
            Tare offset in raw ADC units
        """
        if self.tare_cmd is None:
            raise self.printer.command_error("Loadcell not initialized")

        if samples is None:
            samples = self.tare_samples

        result = self.tare_cmd.send([self.dwarf_address, min(samples, 64)])
        offset = result.get('offset', 0)
        sample_count = result.get('samples', 0)

        logging.info("LoadcellProbe: tare offset=%d (%.2fg) from %d samples",
                     offset, self._load_to_grams(offset), sample_count)

        return offset

    def query_state(self):
        """Query current probe state from MCU."""
        if self.probe_query_cmd is None:
            return {}

        result = self.probe_query_cmd.send([])
        self.last_state = {
            'triggered': bool(result.get('triggered', 0)),
            'load_raw': result.get('load', 0),
            'load_grams': self._load_to_grams(result.get('load', 0)),
            'sample_count': result.get('count', 0),
            'monitoring': bool(result.get('monitoring', 0)),
        }
        return self.last_state

    def run_probe(self, speed=None, max_distance=20.0, threshold=None, settle_time=0.5):
        """
        Execute a Z probe move using MCU-level loadcell monitoring.

        Moves the nozzle down until the loadcell detects bed contact or
        max_distance is reached.

        Args:
            speed: Probe speed in mm/s (default: self.speed)
            max_distance: Maximum probe distance in mm
            threshold: Threshold in grams (default: self.z_threshold)
            settle_time: Time to wait after tare for loadcell to settle

        Returns:
            Z position where probe triggered

        Raises:
            printer.command_error: If probe does not trigger
        """
        if speed is None:
            speed = self.speed
        if threshold is None:
            threshold = self.z_threshold

        threshold_raw = self._grams_to_raw(threshold)
        toolhead = self.printer.lookup_object('toolhead')

        # Get current position
        toolhead.wait_moves()
        curpos = toolhead.get_position()
        start_z = curpos[2]

        # Calculate probe endpoint
        probe_z = max(0.0, start_z - max_distance)

        # Tare the loadcell
        self.tare()

        # Wait for loadcell to settle after tare
        if settle_time > 0:
            self.reactor.pause(self.reactor.monotonic() + settle_time)

        # Start MCU-level monitoring (Z mode = xy=0)
        self.probe_start_cmd.send([self.dwarf_address, threshold_raw, 0, 0])

        try:
            # Issue continuous motion command
            target_pos = list(curpos)
            target_pos[2] = probe_z

            # Calculate expected move time
            move_distance = start_z - probe_z
            move_time = move_distance / speed

            # Start the move - this queues the motion
            toolhead.manual_move(target_pos, speed)

            # Poll MCU for trigger during move
            start_time = self.reactor.monotonic()
            poll_interval = 0.010  # 10ms
            triggered = False
            trigger_z = None

            while True:
                eventtime = self.reactor.monotonic()
                elapsed = eventtime - start_time

                # Check if move should be complete
                if elapsed > move_time + 1.0:
                    break

                # Query MCU probe state
                state = self.query_state()

                if state.get('triggered', False):
                    triggered = True
                    # Stop any pending motion
                    toolhead.flush_step_generation()
                    toolhead.wait_moves()
                    trigger_z = toolhead.get_position()[2]
                    break

                # Wait before next poll
                self.reactor.pause(eventtime + poll_interval)

            # Make sure motion is complete
            toolhead.wait_moves()

        finally:
            # Stop monitoring and get results
            result = self.probe_stop_cmd.send([])

        if not result.get('triggered', 0):
            raise self.printer.command_error(
                "Z probe did not trigger! "
                "Load=%.1fg (raw=%d), min=%.1fg, samples=%d, errors=%d" % (
                    self._load_to_grams(result.get('load', 0)),
                    result.get('load', 0),
                    self._load_to_grams(result.get('min', 0)),
                    result.get('count', 0),
                    result.get('errors', 0)))

        # Use final position if we didn't catch trigger during polling
        if trigger_z is None:
            trigger_z = toolhead.get_position()[2]

        # Apply Z offset
        trigger_z += self.z_offset
        self.last_z_result = trigger_z

        logging.info("LoadcellProbe: Z triggered at %.4f "
                     "(load=%.1fg, min=%.1fg, samples=%d)",
                     trigger_z,
                     self._load_to_grams(result.get('load', 0)),
                     self._load_to_grams(result.get('min', 0)),
                     result.get('count', 0))

        return trigger_z

    def run_xy_probe(self, direction, speed=None, max_distance=10.0,
                     threshold=None, settle_time=0.5):
        """
        Execute an XY probe move for tool calibration.

        Moves the nozzle in the specified direction until the loadcell
        detects contact (tension from hitting calibration pin).

        Args:
            direction: 'x+', 'x-', 'y+', or 'y-'
            speed: Probe speed in mm/s (default: self.speed)
            max_distance: Maximum probe distance in mm
            threshold: Threshold in grams (default: self.xy_threshold)
            settle_time: Time to wait after tare

        Returns:
            (x, y) position where probe triggered

        Raises:
            printer.command_error: If probe does not trigger
        """
        if speed is None:
            speed = self.speed
        if threshold is None:
            threshold = self.xy_threshold

        threshold_raw = self._grams_to_raw(threshold)
        toolhead = self.printer.lookup_object('toolhead')

        # Get current position
        toolhead.wait_moves()
        curpos = toolhead.get_position()
        start_x, start_y = curpos[0], curpos[1]

        # Calculate probe endpoint based on direction
        target_pos = list(curpos)
        if direction == 'x+':
            target_pos[0] = start_x + max_distance
        elif direction == 'x-':
            target_pos[0] = start_x - max_distance
        elif direction == 'y+':
            target_pos[1] = start_y + max_distance
        elif direction == 'y-':
            target_pos[1] = start_y - max_distance
        else:
            raise self.printer.command_error(
                "Invalid direction '%s', use x+, x-, y+, or y-" % direction)

        # Tare the loadcell
        self.tare()

        # Wait for loadcell to settle
        if settle_time > 0:
            self.reactor.pause(self.reactor.monotonic() + settle_time)

        # Start MCU-level monitoring (XY mode = xy=1)
        self.probe_start_cmd.send([self.dwarf_address, threshold_raw, 0, 1])

        try:
            # Calculate expected move time
            if direction.startswith('x'):
                move_distance = abs(target_pos[0] - start_x)
            else:
                move_distance = abs(target_pos[1] - start_y)
            move_time = move_distance / speed

            # Start the move
            toolhead.manual_move(target_pos, speed)

            # Poll MCU for trigger during move
            start_time = self.reactor.monotonic()
            poll_interval = 0.010  # 10ms
            triggered = False
            trigger_pos = None

            while True:
                eventtime = self.reactor.monotonic()
                elapsed = eventtime - start_time

                if elapsed > move_time + 1.0:
                    break

                state = self.query_state()

                if state.get('triggered', False):
                    triggered = True
                    toolhead.flush_step_generation()
                    toolhead.wait_moves()
                    pos = toolhead.get_position()
                    trigger_pos = (pos[0], pos[1])
                    break

                self.reactor.pause(eventtime + poll_interval)

            toolhead.wait_moves()

        finally:
            result = self.probe_stop_cmd.send([])

        if not result.get('triggered', 0):
            raise self.printer.command_error(
                "XY probe did not trigger (direction=%s)! "
                "Load=%.1fg, max=%.1fg, samples=%d, errors=%d" % (
                    direction,
                    self._load_to_grams(result.get('load', 0)),
                    self._load_to_grams(result.get('max', 0)),
                    result.get('count', 0),
                    result.get('errors', 0)))

        if trigger_pos is None:
            pos = toolhead.get_position()
            trigger_pos = (pos[0], pos[1])

        logging.info("LoadcellProbe: XY triggered at (%.4f, %.4f) direction=%s "
                     "(load=%.1fg, max=%.1fg)",
                     trigger_pos[0], trigger_pos[1], direction,
                     self._load_to_grams(result.get('load', 0)),
                     self._load_to_grams(result.get('max', 0)))

        return trigger_pos

    def multi_probe(self, max_distance=20.0):
        """
        Perform multiple probe samples and return averaged result.

        Returns:
            Average Z position from all samples

        Raises:
            printer.command_error: If samples vary too much
        """
        toolhead = self.printer.lookup_object('toolhead')
        results = []

        for i in range(self.samples):
            z = self.run_probe(max_distance=max_distance)
            results.append(z)

            # Retract for next sample (except on last)
            if i < self.samples - 1 and self.sample_retract_dist > 0:
                curpos = toolhead.get_position()
                curpos[2] += self.sample_retract_dist
                toolhead.manual_move(curpos, self.lift_speed)
                toolhead.wait_moves()

        # Check tolerance
        z_min = min(results)
        z_max = max(results)
        if z_max - z_min > self.samples_tolerance:
            raise self.printer.command_error(
                "Probe samples exceed tolerance: min=%.4f max=%.4f "
                "(tolerance=%.4f)" % (z_min, z_max, self.samples_tolerance))

        z_avg = sum(results) / len(results)

        logging.info("LoadcellProbe: multi-probe result=%.4f "
                     "(samples=%s, range=%.4f)",
                     z_avg, [round(z, 4) for z in results], z_max - z_min)

        return z_avg

    # Klipper probe interface methods

    def get_probe_params(self):
        """Return probe parameters for Klipper's probe infrastructure."""
        return {
            'pin': None,  # Virtual endstop
            'x_offset': 0.0,
            'y_offset': 0.0,
            'z_offset': self.z_offset,
            'speed': self.speed,
            'lift_speed': self.lift_speed,
            'samples': self.samples,
            'sample_retract_dist': self.sample_retract_dist,
            'samples_tolerance': self.samples_tolerance,
        }

    def multi_probe_begin(self):
        """Called before a series of probe points."""
        pass

    def multi_probe_end(self):
        """Called after a series of probe points."""
        pass

    def get_status(self, eventtime):
        """Return current probe status for Klipper's status system."""
        return {
            'name': self.name,
            'last_z_result': self.last_z_result,
            'z_threshold': self.z_threshold,
            'xy_threshold': self.xy_threshold,
            'dwarf_address': self.dwarf_address,
            'z_offset': self.z_offset,
        }

    # GCode commands

    def cmd_LOADCELL_TARE(self, gcmd):
        """GCode command to tare the loadcell."""
        samples = gcmd.get_int('SAMPLES', self.tare_samples)
        offset = self.tare(samples)
        gcmd.respond_info("Loadcell tared: offset=%d (%.2fg)" %
                          (offset, self._load_to_grams(offset)))

    def cmd_LOADCELL_STATUS(self, gcmd):
        """GCode command to show loadcell status."""
        state = self.query_state()
        gcmd.respond_info(
            "Loadcell Status:\n"
            "  Address: 0x%02X\n"
            "  Z Threshold: %.1fg (raw=%d)\n"
            "  XY Threshold: %.1fg (raw=%d)\n"
            "  Current load: %.2fg (raw=%d)\n"
            "  Triggered: %s\n"
            "  Monitoring: %s\n"
            "  Samples: %d\n"
            "  Last Z result: %s" % (
                self.dwarf_address,
                self.z_threshold, self.z_threshold_raw,
                self.xy_threshold, self.xy_threshold_raw,
                state.get('load_grams', 0), state.get('load_raw', 0),
                "Yes" if state.get('triggered') else "No",
                "Yes" if state.get('monitoring') else "No",
                state.get('sample_count', 0),
                "%.4f" % self.last_z_result if self.last_z_result else "None"
            ))

    def cmd_LOADCELL_PROBE(self, gcmd):
        """GCode command to perform Z probe."""
        speed = gcmd.get_float('SPEED', self.speed)
        max_dist = gcmd.get_float('MAX_DIST', 20.0)
        threshold = gcmd.get_float('THRESHOLD', self.z_threshold)
        samples = gcmd.get_int('SAMPLES', 1)

        gcmd.respond_info("Z Probe: speed=%.1fmm/s, max=%.1fmm, threshold=%.1fg"
                          % (speed, max_dist, threshold))

        if samples == 1:
            z = self.run_probe(speed=speed, max_distance=max_dist,
                              threshold=threshold)
        else:
            old_samples = self.samples
            self.samples = samples
            try:
                z = self.multi_probe(max_distance=max_dist)
            finally:
                self.samples = old_samples

        gcmd.respond_info("Z Probe result: Z=%.4f" % z)

    def cmd_LOADCELL_PROBE_XY(self, gcmd):
        """GCode command to perform XY probe."""
        direction = gcmd.get('DIRECTION')
        speed = gcmd.get_float('SPEED', self.speed)
        max_dist = gcmd.get_float('MAX_DIST', 10.0)
        threshold = gcmd.get_float('THRESHOLD', self.xy_threshold)

        gcmd.respond_info("XY Probe: direction=%s, speed=%.1fmm/s, "
                          "max=%.1fmm, threshold=%.1fg"
                          % (direction, speed, max_dist, threshold))

        x, y = self.run_xy_probe(direction, speed=speed, max_distance=max_dist,
                                 threshold=threshold)

        gcmd.respond_info("XY Probe result: X=%.4f Y=%.4f" % (x, y))


def load_config(config):
    return LoadcellProbe(config)
