# XlKlipper - Prusa XL Klipper Integration
# Prusa Puppy Bootloader + MODBUS for Klipper
# Boots Dwarfs and communicates via MODBUS
#
# Copyright (C) 2026 Richard Crook
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
import random
import time
import threading
import collections
import math
import os
import struct
import mcu as mcu_module  # For TriggerDispatch and MCU_trsync

# Import accelerometer support (same directory when deployed to extras/)
try:
    from . import dwarf_accelerometer
    DwarfAccelerometer = dwarf_accelerometer.DwarfAccelerometer
except ImportError:
    # Fallback for direct import
    try:
        import dwarf_accelerometer
        DwarfAccelerometer = dwarf_accelerometer.DwarfAccelerometer
    except ImportError:
        DwarfAccelerometer = None
        logging.warning("PuppyBootloader: dwarf_accelerometer module not found - accelerometer disabled")

# ProbeResult namedtuple - compatible with Klipper's manual_probe.ProbeResult
ProbeResult = collections.namedtuple('probe_result', [
    'bed_x', 'bed_y', 'bed_z', 'test_x', 'test_y', 'test_z'])


class NullPWM:
    """Dummy PWM pin for Dwarf heaters - actual control via MODBUS.

    This satisfies Klipper's heater interface but does nothing.
    The Dwarf runs its own PID loop; we control it via M104/M109
    which write to MODBUS register 0xE000.
    """

    def __init__(self, mcu, pin_params):
        self._mcu = mcu
        self._pin = pin_params.get('pin', 'null')
        logging.info(f"NullPWM: Created for pin '{self._pin}' (heater via MODBUS)")

    def get_mcu(self):
        return self._mcu

    def setup_max_duration(self, max_duration):
        pass  # Dwarf handles its own safety

    def setup_start_value(self, start_value, shutdown_value):
        pass  # Dwarf handles shutdown

    def setup_cycle_time(self, cycle_time, hardware_pwm=False):
        pass  # Dwarf uses its own PWM frequency

    def set_pwm(self, print_time, value, cycle_time=None):
        pass  # Actual heating controlled via MODBUS, not PWM


class NullDigitalOut:
    """Dummy digital output pin for virtual enable signals.

    Used for extruder enable_pin when actual TMC enable is via MODBUS.
    """

    def __init__(self, mcu, pin_params):
        self._mcu = mcu
        self._pin = pin_params.get('pin', 'null')
        logging.info(f"NullDigitalOut: Created for pin '{self._pin}' (enable via MODBUS)")

    def get_mcu(self):
        return self._mcu

    def setup_max_duration(self, max_duration):
        pass

    def setup_start_value(self, start_value, shutdown_value):
        pass

    def set_digital(self, print_time, value):
        pass  # Actual TMC enable controlled via MODBUS coil 0x4000


class NullPinChip:
    """Virtual pin chip that provides NullPWM and NullDigitalOut for 'null:' prefix.

    Usage in printer.cfg:
        heater_pin: null:0     # For MODBUS-controlled heater
        enable_pin: null:0     # For MODBUS-controlled TMC enable

    This allows [extruder] sections to have valid pins
    while actual control is done via MODBUS to the Dwarf.
    """

    def __init__(self, printer):
        self._printer = printer
        self._mcu = None

    def setup_pin(self, pin_type, pin_params):
        if self._mcu is None:
            self._mcu = self._printer.lookup_object('mcu')
        if pin_type == 'pwm':
            return NullPWM(self._mcu, pin_params)
        elif pin_type == 'digital_out':
            return NullDigitalOut(self._mcu, pin_params)
        else:
            raise self._printer.config_error(
                f"NullPinChip supports 'pwm' and 'digital_out', got '{pin_type}'")


class LoadcellEndstop:
    """Loadcell-based endstop using trsync for instant motion stop.

    This class implements the same interface as MCU_endstop, allowing
    Klipper's homing system to use the loadcell as a probe trigger.
    The MCU monitors the loadcell and calls trsync_do_trigger() when
    the force threshold is exceeded, instantly stopping motion.
    """

    REASON_ENDSTOP_HIT = mcu_module.MCU_trsync.REASON_ENDSTOP_HIT

    def __init__(self, printer, main_mcu, puppy_bootloader):
        self._printer = printer
        self._mcu = main_mcu
        self._puppy = puppy_bootloader
        self._dispatch = mcu_module.TriggerDispatch(main_mcu)
        self._home_cmd = None
        self._query_cmd = None
        # Probe parameters (set before home_start)
        self._dwarf_addr = 0x1B  # Default to Dwarf 1 (Prusa standard)
        self._threshold_raw = -2083  # ~40g - gentle threshold
        self._xy_mode = 0

    def get_mcu(self):
        return self._mcu

    def add_stepper(self, stepper):
        self._dispatch.add_stepper(stepper)

    def get_steppers(self):
        return self._dispatch.get_steppers()


    def setup_commands(self, home_cmd):
        """Store reference to the home command."""
        self._home_cmd = home_cmd

    def set_probe_params(self, dwarf_addr, threshold_raw, xy_mode=0):
        """Set parameters for the next probe."""
        self._dwarf_addr = dwarf_addr
        self._threshold_raw = threshold_raw
        self._xy_mode = xy_mode

    def home_start(self, print_time, sample_time, sample_count, rest_time,
                   triggered=True):
        """Start homing/probing - MCU will monitor loadcell and trigger trsync.

        This is called by Klipper's homing system. We start the trsync dispatch
        and send the loadcell_endstop_home command to the MCU.
        """
        if self._home_cmd is None:
            raise self._printer.command_error(
                "Loadcell endstop command not available - check firmware")

        # Start trsync dispatch - this returns a completion that fires on trigger
        trigger_completion = self._dispatch.start(print_time)

        # Get trsync OID and send command to MCU
        trsync_oid = self._dispatch.get_oid()
        trigger_reason = self.REASON_ENDSTOP_HIT

        # Send command to start loadcell monitoring
        # MCU will poll MODBUS and call trsync_do_trigger() when threshold exceeded
        clock = self._mcu.print_time_to_clock(print_time)
        self._home_cmd.send([
            self._dwarf_addr,
            self._threshold_raw,
            self._xy_mode,
            trsync_oid,
            trigger_reason
        ], reqclock=clock)

        return trigger_completion

    def home_wait(self, home_end_time):
        """Wait for homing to complete and return trigger time."""
        # Wait for trsync to complete
        self._dispatch.wait_end(home_end_time)

        # Stop monitoring on MCU (trsync_oid=0xff disables)
        if self._home_cmd is not None:
            self._home_cmd.send([self._dwarf_addr, 0, 0, 0xff, 0])

        # Get result from dispatch
        res = self._dispatch.stop()

        if res >= mcu_module.MCU_trsync.REASON_COMMS_TIMEOUT:
            raise self._printer.command_error(
                "Communication timeout during loadcell probe")

        if res != self.REASON_ENDSTOP_HIT:
            # No trigger - return 0 to indicate failure
            return 0.

        # Trigger happened - return the trigger time
        # For now we use home_end_time as approximation
        # (proper implementation would query MCU for exact trigger clock)
        return home_end_time


class LoadcellProbe:
    """Klipper probe interface for loadcell-based probing.

    This wraps LoadcellEndstop to provide the standard Klipper probe interface
    so that BED_MESH_CALIBRATE, PROBE, PROBE_ACCURACY etc. all work.
    """

    def __init__(self, printer, loadcell_endstop, puppy_bootloader):
        self._printer = printer
        self._endstop = loadcell_endstop
        self._puppy = puppy_bootloader
        self._mcu = loadcell_endstop.get_mcu()

        # Probe configuration
        self._x_offset = 0.0
        self._y_offset = 0.0
        self._z_offset = 0.0
        self._speed = 1.0  # Reduced from 5.0 for gentle bed mesh probing
        self._lift_speed = 5.0
        self._samples = 1
        self._sample_retract_dist = 0.2  # Prusa: Z_CLEARANCE_MULTI_PROBE
        self._samples_tolerance = 0.100
        self._samples_tolerance_retries = 0
        self._samples_result = 'average'

        # Probe threshold settings (grams) - user configurable
        self._z_fast_threshold = 125.0   # Fast Z probe threshold
        self._z_slow_threshold = 125.0   # Slow Z probe threshold
        self._mesh_threshold = 125.0     # Bed mesh probe threshold
        self._xy_threshold = 40.0       # XY calibration probe threshold

        # Multi-probe state
        self._multi_probe_pending = False
        self._last_z_result = None
        self._results = []  # Accumulated probe results for pull_probed_results()
        self._reference_tare = None  # Baseline tare for deviation check (Prusa-style)
        self._max_tare_deviation = 125.0  # grams - Prusa uses threshold (125g)

        # Register as pin chip so probe:z_virtual_endstop works
        self._printer.lookup_object('pins').register_chip('probe', self)

        # Register as probe with printer
        self._printer.register_event_handler("klippy:ready", self._handle_ready)
        self._printer.register_event_handler("homing:homing_move_begin",
                                             self._handle_homing_move_begin)
        self._printer.register_event_handler("homing:homing_move_end",
                                             self._handle_homing_move_end)

    def _handle_ready(self):
        """Register probe commands when printer is ready."""
        self.gcode = self._printer.lookup_object('gcode')

        # Register PROBE command
        self.gcode.register_command('PROBE', self.cmd_PROBE,
            desc="Probe Z using loadcell")
        self.gcode.register_command('QUERY_PROBE', self.cmd_QUERY_PROBE,
            desc="Query loadcell probe status")
        # NOTE: PROBE_CALIBRATE removed - not needed with Prusa-style loadcell probing
        # The loadcell probes WITH the nozzle, so no probe-to-nozzle offset needed
        self.gcode.register_command('PROBE_ACCURACY', self.cmd_PROBE_ACCURACY,
            desc="Test loadcell probe accuracy")
        self.gcode.register_command('PROBE_XY', self.cmd_PROBE_XY,
            desc="Probe XY using loadcell lateral detection")
        self.gcode.register_command('CALIBRATE_TOOL_OFFSETS', self.cmd_CALIBRATE_TOOL_OFFSETS,
            desc="Automatic tool offset calibration (Prusa G425 method)")

        # Threshold configuration commands
        self.gcode.register_command("PROBE_THRESHOLDS", self.cmd_PROBE_THRESHOLDS,
            desc="Show current probe threshold settings")
        self.gcode.register_command("SET_Z_FAST_THRESHOLD", self.cmd_SET_Z_FAST_THRESHOLD,
            desc="Set fast Z probe threshold (grams)")
        self.gcode.register_command("SET_Z_SLOW_THRESHOLD", self.cmd_SET_Z_SLOW_THRESHOLD,
            desc="Set slow Z probe threshold (grams)")
        self.gcode.register_command("SET_MESH_THRESHOLD", self.cmd_SET_MESH_THRESHOLD,
            desc="Set bed mesh probe threshold (grams)")
        self.gcode.register_command("SET_XY_THRESHOLD", self.cmd_SET_XY_THRESHOLD,
            desc="Set XY calibration probe threshold (grams)")
        
        # Load saved thresholds
        self._load_thresholds()

        logging.info("LoadcellProbe: Registered probe commands")

    def setup_pin(self, pin_type, pin_params):
        """Setup pin for z_virtual_endstop - returns loadcell endstop."""
        if pin_type != 'endstop' or pin_params['pin'] != 'z_virtual_endstop':
            raise self._printer.lookup_object('pins').error(
                "Probe virtual endstop only useful as endstop pin")
        if pin_params['invert'] or pin_params['pullup']:
            raise self._printer.lookup_object('pins').error(
                "Can not pullup/invert probe virtual endstop")
        return self._endstop

    def _handle_homing_move_begin(self, hmove):
        """Called when homing begins - setup loadcell for probing."""
        if self._endstop not in hmove.get_mcu_endstops():
            return
        # Setup loadcell for homing
        active_tool = self._puppy.active_tool
        if active_tool >= 0:
            modbus_addr = 0x1B + active_tool  # Dwarf address (T0=0x1B, T1=0x1C, etc.)
            dwarf = active_tool + 1
            # Enable loadcell if needed
            if not self._puppy.loadcell_enabled.get(dwarf, False):
                self._puppy._write_coil(dwarf, 0x4003, False)  # Disable accel
                self._puppy.reactor.pause(self._puppy.reactor.monotonic() + 0.1)
                self._puppy._write_coil(dwarf, self._puppy.LOADCELL_COIL, True)
                self._puppy.loadcell_enabled[dwarf] = True
                self._puppy.reactor.pause(self._puppy.reactor.monotonic() + 0.2)
            # Tare
            if self._puppy.tare_mcu_cmd is not None:
                self._puppy.tare_mcu_cmd.send([modbus_addr, 48])
                self._puppy.reactor.pause(self._puppy.reactor.monotonic() + 0.1)
            # Set endstop params
            threshold = self._z_slow_threshold  # User configurable threshold
            threshold_raw = -int(threshold / self._puppy.LOADCELL_SCALE)
            self._endstop.set_probe_params(modbus_addr, threshold_raw, xy_mode=0)

    def _handle_homing_move_end(self, hmove):
        """Called when homing ends."""
        if self._endstop not in hmove.get_mcu_endstops():
            return
        # Could disable loadcell here if desired

    def get_probe_params(self, gcmd=None):
        """Return probe parameters for Klipper's probe system."""
        return {
            'probe_speed': self._speed,
            'lift_speed': self._lift_speed,
            'samples': self._samples,
            'sample_retract_dist': self._sample_retract_dist,
            'samples_tolerance': self._samples_tolerance,
            'samples_tolerance_retries': self._samples_tolerance_retries,
            'samples_result': self._samples_result,
        }

    def get_offsets(self, gcmd=None):
        """Return probe offsets (x, y, z)."""
        return self._x_offset, self._y_offset, self._z_offset

    def get_status(self, eventtime):
        """Return probe status for Klipper's status system."""
        return {
            'name': 'loadcell_probe',
            'last_z_result': self._last_z_result,
        }

    def get_lift_speed(self, gcmd=None):
        """Return lift speed for retracting after probe."""
        return self._lift_speed

    def _get_z_min_position(self):
        """Get the minimum Z position from stepper_z config.

        This is needed because the probe target must be able to reach
        position_min, which may be negative if position_endstop is negative.
        """
        try:
            # Try to get from stepper_z config
            configfile = self._printer.lookup_object('configfile')
            stepper_z_config = configfile.get_status(None)['settings'].get('stepper_z', {})
            z_min = stepper_z_config.get('position_min', 0.)
            return float(z_min)
        except Exception:
            # Fallback to 0 if can't get config
            return 0.

    def multi_probe_begin(self):
        """Called before a series of probes (e.g., bed mesh)."""
        self._multi_probe_pending = True
        # Enable loadcell on active tool
        active_tool = self._puppy.active_tool
        if active_tool >= 0:
            dwarf = active_tool + 1
            modbus_addr = 0x1B + active_tool  # Dwarf address (T0=0x1B, T1=0x1C, etc.)
            if not self._puppy.loadcell_enabled.get(dwarf, False):
                self._puppy._write_coil(dwarf, 0x4003, False)  # Disable accel
                self._puppy.reactor.pause(self._puppy.reactor.monotonic() + 0.1)
                self._puppy._write_coil(dwarf, self._puppy.LOADCELL_COIL, True)
                self._puppy.loadcell_enabled[dwarf] = True
                self._puppy.reactor.pause(self._puppy.reactor.monotonic() + 0.2)
            # Tare and store as reference baseline for deviation check (Prusa-style)
            if self._puppy.tare_mcu_cmd is not None:
                result = self._puppy.tare_mcu_cmd.send([modbus_addr, 48])
                offset = result.get('offset', 0)
                self._reference_tare = offset  # Store baseline for deviation check
                offset_g = offset * self._puppy.LOADCELL_SCALE
                logging.info(f"LoadcellProbe: Reference tare={offset} ({offset_g:.1f}g)")
                self._puppy.reactor.pause(self._puppy.reactor.monotonic() + 0.15)
            # Set endstop params for probing
            threshold = self._z_slow_threshold  # User configurable threshold
            threshold_raw = -int(threshold / self._puppy.LOADCELL_SCALE)
            self._endstop.set_probe_params(modbus_addr, threshold_raw, xy_mode=0)
        logging.info("LoadcellProbe: multi_probe_begin")

    def multi_probe_end(self):
        """Called after a series of probes."""
        self._multi_probe_pending = False
        self._reference_tare = None  # Clear reference
        logging.info("LoadcellProbe: multi_probe_end")

    def start_probe_session(self, gcmd):
        """Start a probing session - called by bed_mesh etc."""
        self._results = []  # Clear results at session start
        self.multi_probe_begin()
        return self

    def end_probe_session(self):
        """End a probing session."""
        self.multi_probe_end()
        self._results = []

    def pull_probed_results(self):
        """Return accumulated probe results and clear the list."""
        res = self._results
        self._results = []
        return res

    def run_probe(self, gcmd, travel=None):
        """Execute a single probe and return [x, y, z] position.

        Args:
            gcmd: G-code command context
            travel: Optional override for probe travel distance (default 15mm)
        """
        toolhead = self._printer.lookup_object('toolhead')
        
        # Prusa XY_ACCELERATION_MMSS limit (500 mm/s^2) for precision probing
        orig_max_accel = toolhead.max_accel
        toolhead.max_accel = min(500.0, orig_max_accel)
        phoming = self._printer.lookup_object('homing')

        # Get current position
        curpos = toolhead.get_position()

        # Get probe parameters
        speed = gcmd.get_float('PROBE_SPEED', self._speed, above=0.)
        lift_speed = gcmd.get_float('LIFT_SPEED', self._lift_speed, above=0.)
        sample_retract_dist = gcmd.get_float('SAMPLE_RETRACT_DIST',
                                              self._sample_retract_dist, minval=0.)

        # Setup loadcell parameters
        active_tool = self._puppy.active_tool
        if active_tool < 0:
            raise gcmd.error("No tool selected - cannot probe")

        modbus_addr = 0x1B + active_tool  # Dwarf address (T0=0x1B, T1=0x1C, etc.)

        # Enable loadcell if not already enabled
        dwarf = active_tool + 1
        if not self._puppy.loadcell_enabled.get(dwarf, False):
            self._puppy._write_coil(dwarf, 0x4003, False)
            self._puppy.reactor.pause(self._puppy.reactor.monotonic() + 0.1)
            self._puppy._write_coil(dwarf, self._puppy.LOADCELL_COIL, True)
            self._puppy.loadcell_enabled[dwarf] = True
            self._puppy.reactor.pause(self._puppy.reactor.monotonic() + 0.2)

        # Settle delay: wait for vibration to dampen after travel move
        # Prusa uses 200ms VIBRATION_SETTLE_TIME for similar purposes
        self._puppy.reactor.pause(self._puppy.reactor.monotonic() + 0.2)

        # Tare strategy depends on probe context:
        # - Multi-probe (mesh): single quick tare (session already tared at start)
        # - Standalone probe (Z home): double tare for accuracy (flush FIFO, then fresh)
        if self._puppy.tare_mcu_cmd is not None:
            if self._multi_probe_pending:
                # Multi-probe: single tare with deviation check (Prusa-style)
                # Prusa Z_FIRST_PROBE_DELAY = 300ms - let vibrations settle before tare
                self._puppy.reactor.pause(self._puppy.reactor.monotonic() + 0.3)
                result = self._puppy.tare_mcu_cmd.send([modbus_addr, 48])
                offset = result.get('offset', 0)
                offset_g = offset * self._puppy.LOADCELL_SCALE

                # Check deviation from reference (Prusa probe.cpp style)
                # Prusa: re-tare each point, compare to reference.
                # If offset > threshold: lift, re-tare, update reference, continue.
                # Our MODBUS loadcell drifts thermally more than Prusa's direct
                # connection, so we update the reference on every point to track
                # the drift, and only lift for extreme deviations (>500g).
                if self._reference_tare is not None:
                    deviation = abs(offset - self._reference_tare) * self._puppy.LOADCELL_SCALE
                    if deviation > 500.0:
                        # Extreme deviation - lift and re-tare (Prusa Z_AFTER_PROBING)
                        logging.warning(f"LoadcellProbe: Tare deviation {deviation:.1f}g - lifting to re-establish baseline")
                        toolhead = self._printer.lookup_object('toolhead')
                        curpos = toolhead.get_position()
                        toolhead.manual_move([None, None, curpos[2] + 20.0], 10.0)
                        toolhead.wait_moves()
                        self._puppy.reactor.pause(self._puppy.reactor.monotonic() + 0.3)
                        result = self._puppy.tare_mcu_cmd.send([modbus_addr, 48])
                        offset = result.get('offset', 0)
                        offset_g = offset * self._puppy.LOADCELL_SCALE
                        logging.info(f"LoadcellProbe: New reference tare={offset} ({offset_g:.1f}g)")
                    else:
                        logging.info(f"LoadcellProbe: Mesh tare={offset} ({offset_g:.1f}g), drift={deviation:.1f}g")
                    # Always update reference to track thermal drift (Prusa line 633)
                    self._reference_tare = offset
                else:
                    logging.info(f"LoadcellProbe: Mesh tare={offset} ({offset_g:.1f}g)")

                self._puppy.reactor.pause(self._puppy.reactor.monotonic() + 0.1)
            else:
                # Standalone probe: double tare for best accuracy
                # First tare - starts FIFO flush
                result = self._puppy.tare_mcu_cmd.send([modbus_addr, 48])
                offset1 = result.get('offset', 0)
                logging.info(f"LoadcellProbe: Tare1 offset={offset1} ({offset1 * self._puppy.LOADCELL_SCALE:.1f}g)")
                # Wait for FIFO to refill with fresh samples (3x 0.1s = 0.3s)
                for _ in range(3):
                    self._puppy.reactor.pause(self._puppy.reactor.monotonic() + 0.1)
                # Second tare - gets accurate offset with fresh data
                result = self._puppy.tare_mcu_cmd.send([modbus_addr, 48])
                offset2 = result.get('offset', 0)
                logging.info(f"LoadcellProbe: Tare2 offset={offset2} ({offset2 * self._puppy.LOADCELL_SCALE:.1f}g)")
                # Brief pause before probing
                self._puppy.reactor.pause(self._puppy.reactor.monotonic() + 0.1)

        # Set endstop parameters (Z endstop has only Z steppers)
        # Prusa uses 125g static threshold (thresholdStatic in loadcell.hpp)
        threshold = self._mesh_threshold if self._multi_probe_pending else self._z_slow_threshold
        threshold_raw = -int(threshold / self._puppy.LOADCELL_SCALE)
        self._endstop.set_probe_params(modbus_addr, threshold_raw, xy_mode=0)

        # Probe down - use position_min as limit instead of 0
        # This is critical because position_endstop may be negative
        z_min = self._get_z_min_position()
        probe_travel = travel if travel is not None else 15.0
        target_z = max(curpos[2] - probe_travel, z_min)
        pos = list(curpos)
        pos[2] = target_z
        logging.info(f"LoadcellProbe: Probing from Z={curpos[2]:.2f} to Z={target_z:.2f} (z_min={z_min:.2f})")

        try:
            epos = phoming.probing_move(self._endstop, pos, speed)
        except self._printer.command_error as e:
            if "prior to movement" in str(e):
                raise gcmd.error("Loadcell triggered before probe - check tare")
            raise

        # Store result
        self._last_z_result = epos[2]

        # Create ProbeResult and append to results
        # ProbeResult fields: bed_x, bed_y, bed_z, test_x, test_y, test_z
        # Since loadcell has zero x/y offset, bed and test positions are the same
        probe_result = ProbeResult(
            epos[0] + self._x_offset, epos[1] + self._y_offset,
            epos[2] - self._z_offset,
            epos[0], epos[1], epos[2])
        self._results.append(probe_result)

        # Retract
        if sample_retract_dist > 0:
            liftpos = list(epos)
            liftpos[2] = min(epos[2] + sample_retract_dist, curpos[2])
            toolhead.move(liftpos, lift_speed)
            toolhead.wait_moves()

        return epos[:3]

    def cmd_PROBE(self, gcmd):
        """PROBE command - single probe."""
        pos = self.run_probe(gcmd)
        gcmd.respond_info("Loadcell probe at %.4f,%.4f Z=%.4f" %
                          (pos[0], pos[1], pos[2]))

    def cmd_QUERY_PROBE(self, gcmd):
        """Query probe status."""
        gcmd.respond_info("Loadcell probe: last_z=%.4f" %
                          (self._last_z_result or 0.0))

    # NOTE: cmd_PROBE_CALIBRATE removed - not needed with Prusa-style loadcell probing
    # The loadcell probes WITH the nozzle itself, so:
    # - When loadcell triggers = nozzle at bed surface = Z=0
    # - No probe-to-nozzle offset calculation needed
    # - Tool offsets from CALIBRATE_TOOL_OFFSETS handle all tool-to-tool differences
    # This matches how Prusa XL works - no Live-Z or probe offset needed

    def _load_thresholds(self):
        """Load threshold settings from save_variables."""
        try:
            save_vars = self._printer.lookup_object("save_variables", None)
            if save_vars is not None:
                variables = save_vars.allVariables
                self._z_fast_threshold = variables.get("probe_z_fast_threshold", 125.0)
                self._z_slow_threshold = variables.get("probe_z_slow_threshold", 125.0)
                self._mesh_threshold = variables.get("probe_mesh_threshold", 125.0)
                self._xy_threshold = variables.get("probe_xy_threshold", 40.0)
                logging.info(f"LoadcellProbe: Loaded thresholds - Z_fast:{self._z_fast_threshold}g Z_slow:{self._z_slow_threshold}g Mesh:{self._mesh_threshold}g XY:{self._xy_threshold}g")
        except Exception as e:
            logging.info(f"LoadcellProbe: Using default thresholds: {e}")

    def _save_threshold(self, name, value):
        """Save a threshold value to save_variables."""
        save_vars = self._printer.lookup_object("save_variables", None)
        if save_vars is None:
            raise self.gcode.error("save_variables not configured")
        save_vars.cmd_SAVE_VARIABLE(self.gcode.create_gcode_command(
            "SAVE_VARIABLE", "SAVE_VARIABLE", {"VARIABLE": name, "VALUE": str(value)}))

    def cmd_PROBE_THRESHOLDS(self, gcmd):
        """Show current probe threshold settings."""
        gcmd.respond_info("=== Probe Threshold Settings (grams) ===")
        gcmd.respond_info(f"  Z Fast Probe:  {self._z_fast_threshold}g")
        gcmd.respond_info(f"  Z Slow Probe:  {self._z_slow_threshold}g")
        gcmd.respond_info(f"  Bed Mesh:      {self._mesh_threshold}g")
        gcmd.respond_info(f"  XY Calibration: {self._xy_threshold}g")

    def cmd_SET_Z_FAST_THRESHOLD(self, gcmd):
        """Set fast Z probe threshold."""
        value = gcmd.get_float("VALUE", minval=10.0, maxval=200.0)
        self._z_fast_threshold = value
        self._save_threshold("probe_z_fast_threshold", value)
        gcmd.respond_info(f"Z Fast threshold set to {value}g and saved")

    def cmd_SET_Z_SLOW_THRESHOLD(self, gcmd):
        """Set slow Z probe threshold."""
        value = gcmd.get_float("VALUE", minval=10.0, maxval=200.0)
        self._z_slow_threshold = value
        self._save_threshold("probe_z_slow_threshold", value)
        gcmd.respond_info(f"Z Slow threshold set to {value}g and saved")

    def cmd_SET_MESH_THRESHOLD(self, gcmd):
        """Set bed mesh probe threshold."""
        value = gcmd.get_float("VALUE", minval=10.0, maxval=200.0)
        self._mesh_threshold = value
        self._save_threshold("probe_mesh_threshold", value)
        gcmd.respond_info(f"Mesh threshold set to {value}g and saved")

    def cmd_SET_XY_THRESHOLD(self, gcmd):
        """Set XY calibration probe threshold."""
        value = gcmd.get_float("VALUE", minval=10.0, maxval=200.0)
        self._xy_threshold = value
        self._save_threshold("probe_xy_threshold", value)
        gcmd.respond_info(f"XY threshold set to {value}g and saved")


    def cmd_PROBE_XY(self, gcmd):
        """Probe XY using loadcell lateral detection (for tool calibration)."""
        # Parameters
        direction = gcmd.get('DIRECTION', 'X+').upper()
        distance = gcmd.get_float('DISTANCE', 10.0, above=0.)
        speed = gcmd.get_float('SPEED', 3.0, above=0.)  # Prusa uses 3mm/s

        # Validate direction
        if direction not in ['X+', 'X-', 'Y+', 'Y-']:
            raise gcmd.error("DIRECTION must be X+, X-, Y+, or Y-")

        # Check tool is picked
        active_tool = self._puppy.active_tool
        if active_tool < 0:
            raise gcmd.error("No tool picked - pick a tool first")

        toolhead = self._printer.lookup_object('toolhead')
        
        # Prusa XY_ACCELERATION_MMSS limit (500 mm/s^2) for precision probing
        orig_max_accel = toolhead.max_accel
        toolhead.max_accel = min(500.0, orig_max_accel)
        curpos = toolhead.get_position()

        # Get Dwarf MODBUS address (Prusa standard: Dwarf 1-5 = 0x1B-0x1F)
        modbus_addr = 0x1B + active_tool  # Dwarf address (T0=0x1B)

        # XY probe threshold (user configurable)
        # Positive threshold for absolute value comparison in XY mode
        threshold = self._xy_threshold  # User configurable
        threshold_raw = int(threshold / self._puppy.LOADCELL_SCALE)

        # Enable loadcell if not already enabled
        dwarf = active_tool + 1
        if not self._puppy.loadcell_enabled.get(dwarf, False):
            self._puppy._write_coil(dwarf, 0x4003, False)
            self._puppy.reactor.pause(self._puppy.reactor.monotonic() + 0.1)
            self._puppy._write_coil(dwarf, self._puppy.LOADCELL_COIL, True)
            self._puppy.loadcell_enabled[dwarf] = True
            self._puppy.reactor.pause(self._puppy.reactor.monotonic() + 0.2)

        # Double tare for XY (same as Z probe)
        if self._puppy.tare_mcu_cmd is not None:
            result = self._puppy.tare_mcu_cmd.send([modbus_addr, 48])
            for _ in range(3):
                self._puppy.reactor.pause(self._puppy.reactor.monotonic() + 0.1)
            result = self._puppy.tare_mcu_cmd.send([modbus_addr, 48])
            offset = result.get('offset', 0)
            logging.info(f"PROBE_XY: Tare offset={offset} ({offset * self._puppy.LOADCELL_SCALE:.1f}g)")
            self._puppy.reactor.pause(self._puppy.reactor.monotonic() + 0.1)

        # Set endstop parameters with XY MODE ENABLED
        self._endstop.set_probe_params(modbus_addr, threshold_raw, xy_mode=1)

        # Calculate target position based on direction
        pos = list(curpos)
        if direction == 'X+':
            pos[0] += distance
        elif direction == 'X-':
            pos[0] -= distance
        elif direction == 'Y+':
            pos[1] += distance
        elif direction == 'Y-':
            pos[1] -= distance

        # Do the probing move
        phoming = self._printer.lookup_object('homing')
        try:
            epos = phoming.probing_move(self._endstop, pos, speed)
        except self._printer.command_error as e:
            if "prior to movement" in str(e):
                raise gcmd.error("Loadcell triggered before XY probe - check tare")
            raise

        # Report result
        if direction.startswith('X'):
            gcmd.respond_info("PROBE_XY %s: X=%.4f (Y=%.4f, Z=%.4f)" %
                              (direction, epos[0], epos[1], epos[2]))
        else:
            gcmd.respond_info("PROBE_XY %s: Y=%.4f (X=%.4f, Z=%.4f)" %
                              (direction, epos[1], epos[0], epos[2]))

        return epos[:3]

    def _probe_xy_verified(self, gcmd, center_x, center_y, probe_z, angle_rad,
                           start_radius, probe_distance, speed, toolhead, pin_top_z):
        """Take 2 XY samples and verify within 0.03mm (Prusa PROBE_ALLOWED_ERROR)."""
        PROBE_ALLOWED_ERROR = 0.03  # mm
        NUM_PROBE_SAMPLES = 2
        NUM_PROBE_TRIES = 5
        
        for attempt in range(NUM_PROBE_TRIES):
            samples = []
            for _ in range(NUM_PROBE_SAMPLES):
                hit = self._probe_xy_at_angle(gcmd, center_x, center_y, probe_z, angle_rad,
                                              start_radius, probe_distance, speed)
                samples.append(hit)
                # Retract between samples
                toolhead.manual_move([None, None, pin_top_z + 5], 10.0)
                toolhead.wait_moves()
                # Move back to start for next sample
                start_x = center_x + start_radius * math.cos(angle_rad)
                start_y = center_y + start_radius * math.sin(angle_rad)
                toolhead.manual_move([start_x, start_y, None], 25.0)
                toolhead.manual_move([None, None, probe_z], 10.0)
                toolhead.wait_moves()
            
            # Calculate average
            avg_x = sum(s[0] for s in samples) / len(samples)
            avg_y = sum(s[1] for s in samples) / len(samples)
            
            # Check max deviation from average
            max_dist = max(math.sqrt((s[0]-avg_x)**2 + (s[1]-avg_y)**2) for s in samples)
            
            if max_dist <= PROBE_ALLOWED_ERROR:
                logging.info(f"PROBE_XY_VERIFIED: Samples agree within {max_dist:.4f}mm")
                return (avg_x, avg_y)
            
            logging.warning(f"PROBE_XY_VERIFIED: Sample deviation {max_dist:.4f}mm > {PROBE_ALLOWED_ERROR}mm, retrying...")
        
        # Failed after all retries - return last average anyway with warning
        gcmd.respond_info("  WARNING: XY samples inconsistent after %d tries" % NUM_PROBE_TRIES)
        return (avg_x, avg_y)

    def _probe_xy_at_angle(self, gcmd, center_x, center_y, probe_z, angle_rad,
                           start_radius, probe_distance, speed):
        """Probe toward center from a given angle. Returns hit position.

        Uses Prusa's two-stage approach:
        1. Start XY monitoring (without trsync)
        2. Wait 500ms for mechanical settling
        3. Tare
        4. Verify not already triggered
        5. Then arm trsync and do probe move
        """
        toolhead = self._printer.lookup_object('toolhead')
        
        # Prusa XY_ACCELERATION_MMSS limit (500 mm/s^2) for precision probing
        orig_max_accel = toolhead.max_accel
        toolhead.max_accel = min(500.0, orig_max_accel)
        phoming = self._printer.lookup_object('homing')
        active_tool = self._puppy.active_tool
        modbus_addr = 0x1B + active_tool  # Dwarf address (T0=0x1B, T1=0x1C, etc.)

        # Calculate start position on circle around pin
        start_x = center_x + start_radius * math.cos(angle_rad)
        start_y = center_y + start_radius * math.sin(angle_rad)

        # Move to start position
        toolhead.manual_move([start_x, start_y, None], 25.0)
        toolhead.manual_move([None, None, probe_z], 10.0)
        toolhead.wait_moves()

        # XY threshold (40g like Prusa)
        threshold_grams = self._xy_threshold
        threshold_raw = int(threshold_grams / self._puppy.LOADCELL_SCALE)

        # === PRUSA APPROACH: Settle -> Tare -> Start monitoring -> Verify -> Arm ===

        # Stage 1: Wait 500ms for mechanical settling FIRST (like Prusa RESONANCE_DAMPER_WAIT_MS)
        for _ in range(5):
            self._puppy.reactor.pause(self._puppy.reactor.monotonic() + 0.1)

        # Stage 2: Tare the loadcell BEFORE monitoring (so fresh samples use new offset)
        if self._puppy.tare_mcu_cmd is not None:
            result = self._puppy.tare_mcu_cmd.send([modbus_addr, 48])
            offset = result.get('offset', 0)
            logging.info(f"PROBE_XY: Tare offset={offset} ({offset * self._puppy.LOADCELL_SCALE:.1f}g)")

        # Stage 3: Wait for tare to apply to subsequent samples
        self._puppy.reactor.pause(self._puppy.reactor.monotonic() + 0.15)

        # Stage 4: Start XY monitoring to collect fresh post-tare samples
        if self._puppy.probe_start_cmd is not None:
            self._puppy.probe_start_cmd.send([modbus_addr, threshold_raw, 0, 1])  # xy=1, halt=0
            logging.info(f"PROBE_XY: Started XY monitoring for verification")

        # Wait 200ms for fresh samples with new tare offset
        for _ in range(2):
            self._puppy.reactor.pause(self._puppy.reactor.monotonic() + 0.1)

        # Stage 5: Query state - verify loadcell is stable near zero
        max_retries = 5  # Prusa: NUM_PROBE_TRIES
        for attempt in range(max_retries):
            if self._puppy.probe_query_cmd is not None:
                state = self._puppy.probe_query_cmd.send([])
                current_load = state.get('load', 0)
                current_grams = abs(current_load * self._puppy.LOADCELL_SCALE)
                logging.info(f"PROBE_XY: Pre-check load={current_load} ({current_grams:.1f}g) attempt={attempt+1}")

                if current_grams < threshold_grams * 0.5:  # Below 50% = stable
                    break

                # Load too high - stop, re-tare, restart
                logging.warning(f"PROBE_XY: Load high ({current_grams:.1f}g), re-taring...")
                if self._puppy.probe_stop_cmd is not None:
                    self._puppy.probe_stop_cmd.send([])
                if self._puppy.tare_mcu_cmd is not None:
                    self._puppy.tare_mcu_cmd.send([modbus_addr, 48])
                self._puppy.reactor.pause(self._puppy.reactor.monotonic() + 0.3)
                if self._puppy.probe_start_cmd is not None:
                    self._puppy.probe_start_cmd.send([modbus_addr, threshold_raw, 0, 1])
                self._puppy.reactor.pause(self._puppy.reactor.monotonic() + 0.2)
        else:
            if self._puppy.probe_stop_cmd is not None:
                self._puppy.probe_stop_cmd.send([])
            raise gcmd.error("XY probe loadcell not stable after %d retries" % max_retries)

        # === USE TRSYNC FOR HARDWARE STOPPING (like Z probe) ===
        # MCU firmware now has 512 sample settling (~1.6s) to ensure motion starts first

        # Stop verification monitoring before arming trsync
        if self._puppy.probe_stop_cmd is not None:
            self._puppy.probe_stop_cmd.send([])

        # Calculate target (toward center, past where pin should be)
        target_x = center_x - probe_distance * math.cos(angle_rad)
        target_y = center_y - probe_distance * math.sin(angle_rad)

        logging.info(f"PROBE_XY: Probing from ({start_x:.2f},{start_y:.2f}) toward ({target_x:.2f},{target_y:.2f})")

        # Use XY endstop (has X/Y steppers registered, not Z)
        xy_endstop = self._puppy.loadcell_endstop_xy
        xy_endstop.set_probe_params(modbus_addr, threshold_raw, xy_mode=1)

        # Build target position
        curpos = toolhead.get_position()
        pos = [target_x, target_y, curpos[2], curpos[3]]

        # Use probing_move with XY endstop - instant hardware stop on contact
        epos = phoming.probing_move(xy_endstop, pos, speed)
        # Prusa MIN_TRAVELED_DISTANCE_MM check (0.1mm)
        travel_dist = math.sqrt((epos[0] - start_x)**2 + (epos[1] - start_y)**2)
        if travel_dist < 0.1:
            logging.warning(f"PROBE_XY: Travel too short ({travel_dist:.3f}mm < 0.1mm) - probe may have triggered early")
        logging.info(f"PROBE_XY: Contact at ({epos[0]:.3f},{epos[1]:.3f}), traveled {travel_dist:.2f}mm")

        # Retract away from pin
        retract_dist = 5.0
        retract_x = epos[0] + retract_dist * math.cos(angle_rad)
        retract_y = epos[1] + retract_dist * math.sin(angle_rad)
        toolhead.manual_move([retract_x, retract_y, None], 25.0)
        toolhead.wait_moves()

        # Restore original acceleration
        toolhead.max_accel = orig_max_accel
        
        return (epos[0], epos[1])

    def _check_xy_deviation(self, center, points, max_dev=0.2):
        """Check if XY probe points deviate too much from fitted circle (Prusa MAX_DEVIATION_MM=0.2)."""
        cx, cy = center
        # Calculate radii from center
        radii = [math.sqrt((p[0]-cx)**2 + (p[1]-cy)**2) for p in points]
        avg_radius = sum(radii) / len(radii)
        
        # Find max deviation from average radius
        max_deviation = max(abs(r - avg_radius) for r in radii)
        
        if max_deviation > max_dev:
            logging.warning(f"XY deviation {max_deviation:.3f}mm exceeds limit {max_dev}mm")
            return False, max_deviation
        return True, max_deviation

    def _fit_circle_center(self, points):
        """Iterative least squares circle fitting. Returns (cx, cy)."""
        if len(points) < 3:
            # Not enough points, just return centroid
            cx = sum(p[0] for p in points) / len(points)
            cy = sum(p[1] for p in points) / len(points)
            return (cx, cy)

        # Initial guess: centroid
        cx = sum(p[0] for p in points) / len(points)
        cy = sum(p[1] for p in points) / len(points)

        # Iterate to refine
        for _ in range(10):
            # Calculate average radius
            radii = [math.sqrt((p[0]-cx)**2 + (p[1]-cy)**2) for p in points]
            avg_radius = sum(radii) / len(radii)

            # Calculate weighted correction
            dx, dy, weight_sum = 0.0, 0.0, 0.0
            for i, p in enumerate(points):
                dist = radii[i]
                if dist > 0.001:  # Avoid division by zero
                    # Residual: how far point is from ideal circle
                    residual = dist - avg_radius
                    weight = residual * residual  # Square residual for weight
                    # Direction from center to point
                    ux, uy = (p[0] - cx) / dist, (p[1] - cy) / dist
                    # Correction pushes center toward points outside circle
                    dx += weight * residual * ux
                    dy += weight * residual * uy
                    weight_sum += weight

            if weight_sum > 0.001:
                cx += dx / weight_sum * 0.5  # Damping factor
                cy += dy / weight_sum * 0.5

        return (cx, cy)

    def cmd_CALIBRATE_TOOL_OFFSETS(self, gcmd):
        """Automatic tool offset calibration using calibration pin.

        This implements Prusa's G425 calibration methodology:
        - 12 XY probes around the calibration pin circumference
        - 20 Z probes averaged for accurate nozzle height
        - Offsets calculated relative to Tool 0 (reference tool)

        The calibration pin is 6mm diameter, 9mm tall, installed at bed center.

        PRUSA SEQUENCE (from selftest_tool_offsets.cpp):
        1. User confirms start
        2. Home X/Y ONLY (no Z bed probe!)
        3. User installs sheet for cleaning
        4. Heat nozzles, user cleans each one
        5. Home X/Y again, park tools
        6. User REMOVES sheet and INSTALLS calibration pin
        7. Probe pin to establish Z reference (no bed involved)
        8. User removes pin and reinstalls sheet
        """
        # Parameters - matching Prusa's G425 methodology
        pin_x = gcmd.get_float('PIN_X', 180.0)
        pin_y = gcmd.get_float('PIN_Y', 180.0)
        pin_diameter = gcmd.get_float('PIN_DIAMETER', 6.0)
        pin_height = gcmd.get_float('PIN_HEIGHT', 9.0)
        probe_speed = gcmd.get_float('SPEED', 3.0, above=0.)  # Prusa PROBE_FEEDRATE_MMS = 3 mm/s
        tools = gcmd.get('TOOLS', '0,1,2,3,4')  # Which tools to calibrate
        skip_clean = gcmd.get_int('SKIP_CLEAN', 0)  # Skip nozzle cleaning phase
        clean_temp = gcmd.get_float('CLEAN_TEMP', 200.0)  # Temperature for cleaning
        cal_temp = gcmd.get_float('CAL_TEMP', 70.0)  # Prusa TOOL_CALIBRATION_TEMPERATURE = 70C
        clean_time = gcmd.get_float('CLEAN_TIME', 30.0)  # Seconds per tool for cleaning
        wait_time = gcmd.get_float('WAIT_TIME', 30.0)  # Seconds to wait for user actions

        # Prusa G425 phased probing constants (from G425.cpp)
        PROBE_Z_BORE_MM = 1.0           # XY probing depth below pin top
        PROBE_Z_UNCERTAIN_DIST_MM = 5.0  # Clearance when uncertain
        PROBE_Z_CERTAIN_DIST_MM = 1.0    # Clearance when certain
        PROBE_XY_UNCERTAIN_DIST_MM = 8.0 # XY clearance when uncertain
        NUM_Z_MEASUREMENTS = 20          # Z samples in final phase

        # Prusa phased probing: 3 phases for progressive refinement
        # Phase 0 (first):  1 Z probe,  3 XY probes, 8mm clearance - rough location
        # Phase 1 (second): 0 Z probes, 3 XY probes, 5mm clearance - refine XY
        # Phase 2 (final): 20 Z probes, 12 XY probes, 1mm clearance - accurate measurement
        PHASE_XY_HITS = [3, 3, 12]
        PHASE_Z_HITS = [1, 0, NUM_Z_MEASUREMENTS]
        PHASE_CLEARANCE = [PROBE_XY_UNCERTAIN_DIST_MM, PROBE_Z_UNCERTAIN_DIST_MM, PROBE_Z_CERTAIN_DIST_MM]
        # Total probes per tool (for info display)
        num_xy_probes = sum(PHASE_XY_HITS)  # 3 + 3 + 12 = 18
        num_z_probes = sum(PHASE_Z_HITS)    # 1 + 0 + 20 = 21

        # Safe height = pin top + 5mm clearance = 14mm absolute (Prusa go_to_safe_height)
        safe_z = pin_height + PROBE_Z_UNCERTAIN_DIST_MM  # 9 + 5 = 14mm

        # Parse tool list
        try:
            tool_list = [int(t.strip()) for t in tools.split(',')]
        except ValueError:
            raise gcmd.error("Invalid TOOLS parameter - use comma-separated numbers")

        toolhead = self._printer.lookup_object('toolhead')

        # Prusa XY_ACCELERATION_MMSS limit (500 mm/s^2) for precision probing
        orig_max_accel = toolhead.max_accel
        toolhead.max_accel = min(500.0, orig_max_accel)
        gcode = self._printer.lookup_object('gcode')

        # Cleaning position - front center, high Z for easy user access
        # Y direction: Y- = front (user side), Y+ = back (docks)
        clean_x = 180.0
        clean_y = 50.0  # Front of bed where user can reach nozzles
        clean_z = 100.0  # Higher Z (bed lower) for easier cleaning

        # Prusa XY probing distances (from G425.cpp)
        # PROBE_XY_TIGHT_DIST_MM = PIN_DIAMETER/2 + 3 = 6
        # PROBE_XY_CERTAIN_DIST_MM = TIGHT + 1 = 7
        # PROBE_XY_UNCERTAIN_DIST_MM = CERTAIN + 1 = 8
        start_radius = pin_diameter / 2 + 5.0  # Start position for XY probe
        probe_distance = pin_diameter / 2 + 2.0  # Distance to travel toward center

        gcmd.respond_info("=== TOOL OFFSET CALIBRATION (Prusa G425 Method) ===")
        gcmd.respond_info("Pin: X=%.1f Y=%.1f D=%.1f H=%.1f" %
                         (pin_x, pin_y, pin_diameter, pin_height))
        gcmd.respond_info("Probes: %d XY radial, %d Z averaged (Prusa standard)" %
                         (num_xy_probes, num_z_probes))
        gcmd.respond_info("Tools: %s" % tool_list)
        gcmd.respond_info("")
        gcmd.respond_info(">>> IMPORTANT: This calibration will require your help!")
        gcmd.respond_info(">>> You will be asked to install/remove the calibration pin.")
        gcmd.respond_info(">>> Estimated time: 7-14 minutes depending on tool count.")
        gcmd.respond_info("")

        # === STEP 1: Turn off heaters ===
        gcmd.respond_info("=== STEP 1: TURNING OFF HEATERS ===")

        # Turn off all heaters initially
        gcmd.respond_info("Turning off heaters...")
        for tool_num in tool_list:
            dwarf_num = tool_num + 1
            gcode.run_script_from_command("SET_DWARF_TEMP DWARF=%d TEMP=0" % dwarf_num)

        # === STEP 2: User installs sheet (Prusa: ToolOffsets_wait_user_install_sheet) ===
        if not skip_clean:
            gcmd.respond_info("")
            gcmd.respond_info("=== STEP 2: INSTALL HEATBED SHEET ===")
            gcmd.respond_info(">>> ACTION REQUIRED: Install the heatbed sheet now.")
            gcmd.respond_info(">>> The sheet is needed for nozzle cleaning.")
            gcmd.respond_info(">>> Waiting %.0f seconds..." % wait_time)
            gcmd.respond_info("")
            self._puppy.reactor.pause(self._puppy.reactor.monotonic() + wait_time)

            # === STEP 3: Nozzle cleaning phase ===
            gcmd.respond_info("=== STEP 3: NOZZLE CLEANING PHASE ===")
            gcmd.respond_info("Heating all nozzles to %.0fC for cleaning..." % clean_temp)

            # Heat all nozzles using SET_DWARF_TEMP (MODBUS direct control)
            for tool_num in tool_list:
                dwarf_num = tool_num + 1
                gcode.run_script_from_command("SET_DWARF_TEMP DWARF=%d TEMP=%.0f" % (dwarf_num, clean_temp))

            # Wait for all to reach temp (poll until ready)
            gcmd.respond_info("Waiting for nozzles to heat...")
            for tool_num in tool_list:
                dwarf_num = tool_num + 1
                gcmd.respond_info("Waiting for T%d to reach %.0fC..." % (tool_num, clean_temp))
                while True:
                    temp = self._puppy.dwarf_data.get(dwarf_num, {}).get('hotend_temp', 0)
                    if temp >= clean_temp - 5:
                        break
                    self._puppy.reactor.pause(self._puppy.reactor.monotonic() + 1.0)

            gcmd.respond_info("All nozzles at cleaning temperature!")
            gcmd.respond_info("")
            gcmd.respond_info(">>> CLEAN ALL NOZZLES with brass brush")
            gcmd.respond_info(">>> CLEAN PARKING PLATES as well")
            gcmd.respond_info(">>> Each tool will be presented for %.0f seconds" % clean_time)
            gcmd.respond_info("")

            # Present each tool for cleaning
            for tool_num in tool_list:
                gcmd.respond_info("--- Presenting T%d for cleaning ---" % tool_num)
                gcode.run_script_from_command("T%d" % tool_num)
                toolhead.wait_moves()

                # Move to cleaning position (front, high Z for easy access)
                gcode.run_script_from_command("G1 X%.1f Y%.1f Z%.1f F6000" % (clean_x, clean_y, clean_z))
                toolhead.wait_moves()

                gcmd.respond_info(">>> CLEAN T%d NOW! (%.0f seconds) <<<" % (tool_num, clean_time))
                self._puppy.reactor.pause(self._puppy.reactor.monotonic() + clean_time)
                gcmd.respond_info("T%d cleaning time complete" % tool_num)

            # Park the last tool
            gcode.run_script_from_command("TOOL_PARK")
            toolhead.wait_moves()
            gcmd.respond_info("All nozzles cleaned!")
        else:
            gcmd.respond_info("Skipping nozzle cleaning (SKIP_CLEAN=1)")

        # === STEP 4: Home and park (Prusa: state_home_park) ===
        gcmd.respond_info("")
        gcmd.respond_info("=== STEP 4: HOMING AND PARKING FOR PIN INSTALL ===")

        # Pick T0 (needed for Z homing via loadcell)
        if not self._puppy.tool_picked:
            gcmd.respond_info("Picking T0...")
            gcode.run_script_from_command("T0")
            toolhead.wait_moves()

        # Home all axes - Z homes on bed sheet via T0 loadcell
        gcmd.respond_info("Homing X, Y, Z (Z uses T0 loadcell on bed sheet)...")
        gcode.run_script_from_command("G28")
        toolhead.wait_moves()
        gcmd.respond_info("All axes homed")

        # Park T0 - carriage moves to Y=425, out of the way for pin install
        gcmd.respond_info("Parking T0...")
        gcode.run_script_from_command("TOOL_PARK")
        toolhead.wait_moves()

        # === STEP 5: User removes sheet and installs pin ===
        gcmd.respond_info("")
        gcmd.respond_info("=== STEP 5: INSTALL CALIBRATION PIN ===")
        gcmd.respond_info(">>> Carriage is at back (Y=425) - bed is accessible")
        gcmd.respond_info(">>> ACTION REQUIRED:")
        gcmd.respond_info(">>>   1. REMOVE the heatbed sheet")
        gcmd.respond_info(">>>   2. INSTALL the calibration pin at bed center (X=180 Y=180)")
        gcmd.respond_info(">>> Waiting %.0f seconds..." % wait_time)
        gcmd.respond_info("")
        self._puppy.reactor.pause(self._puppy.reactor.monotonic() + wait_time)

        # === STEP 6: Wait for nozzles to reach calibration temp ===
        gcmd.respond_info("=== STEP 6: WAITING FOR CALIBRATION TEMP ===")
        gcmd.respond_info("Setting nozzles to %.0fC for calibration..." % cal_temp)

        for tool_num in tool_list:
            dwarf_num = tool_num + 1
            gcode.run_script_from_command("SET_DWARF_TEMP DWARF=%d TEMP=%.0f" % (dwarf_num, cal_temp))

        # Wait for temps to reach cal_temp (heat up OR cool down)
        for tool_num in tool_list:
            dwarf_num = tool_num + 1
            temp = self._puppy.dwarf_data.get(dwarf_num, {}).get('hotend_temp', 0)
            if temp > cal_temp + 5:
                gcmd.respond_info("T%d at %.0fC - waiting to cool to %.0fC..." % (tool_num, temp, cal_temp))
            else:
                gcmd.respond_info("Waiting for T%d to reach %.0fC..." % (tool_num, cal_temp))
            while True:
                temp = self._puppy.dwarf_data.get(dwarf_num, {}).get('hotend_temp', 0)
                if cal_temp - 5 <= temp <= cal_temp + 5:
                    break
                self._puppy.reactor.pause(self._puppy.reactor.monotonic() + 1.0)

        gcmd.respond_info("All nozzles at calibration temperature!")

        # === STEP 7: CALIBRATION PROBING ===
        gcmd.respond_info("")
        gcmd.respond_info("=== STEP 7: CALIBRATION PROBING ===")

        # Clear bed mesh during calibration (Prusa: TEMPORARY_BED_LEVELING_STATE(false))
        # Bed compensation would shift Z positions and corrupt measurements
        gcmd.respond_info("Clearing bed mesh for calibration...")
        gcode.run_script_from_command("BED_MESH_CLEAR")

        # Reset all tool offsets before calibration (Prusa: reset_hotend_offsets)
        # Old offsets would shift tool positions during probing and corrupt measurements
        gcmd.respond_info("Resetting all tool offsets to zero...")
        for tool_num in tool_list:
            self._puppy.tool_offsets[tool_num] = (0.0, 0.0, 0.0)
        self._puppy.applied_tool_offset = (0.0, 0.0, 0.0)
        gcode.run_script_from_command("SET_GCODE_OFFSET X=0 Y=0 Z=0 MOVE=0")
        gcmd.respond_info("All offsets zeroed")

        # Store measured centers for each tool
        tool_centers = {}

        for tool_num in tool_list:
            gcmd.respond_info("--- Calibrating Tool %d ---" % tool_num)

            # Pick tool and wait for completion
            gcmd.respond_info("  Picking T%d..." % tool_num)
            gcode.run_script_from_command("T%d" % tool_num)
            toolhead.wait_moves()
            # Extra delay to ensure tool change fully completes
            self._puppy.reactor.pause(self._puppy.reactor.monotonic() + 0.5)

            # Verify tool is picked
            active_tool = self._puppy.active_tool
            if active_tool < 0:
                raise gcmd.error("Tool %d not picked properly (active_tool=%d)" % (tool_num, active_tool))
            if active_tool != tool_num:
                raise gcmd.error("Wrong tool active: expected T%d, got T%d" % (tool_num, active_tool))
            gcmd.respond_info("  T%d picked successfully" % tool_num)

            dwarf = active_tool + 1
            modbus_addr = 0x1B + active_tool  # Dwarf address (T0=0x1B, T1=0x1C, etc.)

            if not self._puppy.loadcell_enabled.get(dwarf, False):
                self._puppy._write_coil(dwarf, 0x4003, False)
                self._puppy.reactor.pause(self._puppy.reactor.monotonic() + 0.1)
                self._puppy._write_coil(dwarf, self._puppy.LOADCELL_COIL, True)
                self._puppy.loadcell_enabled[dwarf] = True
                self._puppy.reactor.pause(self._puppy.reactor.monotonic() + 0.3)

            # === PRUSA PHASED PROBING (from G425.cpp get_xyz_center) ===
            # Phase 0 (first):  1 Z,  3 XY - rough location with 8mm clearance
            # Phase 1 (second): 0 Z,  3 XY - refine XY with 5mm clearance
            # Phase 2 (final): 20 Z, 12 XY - accurate measurement with 1mm clearance

            # Initial position is pin center (known from config)
            current_center = (pin_x, pin_y, pin_height)

            # Get current Z position (we already homed, so Z is known)
            curpos = toolhead.get_position()
            gcmd.respond_info("  Current position: X=%.1f Y=%.1f Z=%.1f" % (curpos[0], curpos[1], curpos[2]))

            # First move Z UP to safe travel height (Z+ = bed DOWN = safe)
            # Use Z=50 to ensure clearance above pin (9mm) and any obstacles
            travel_z = 50.0
            gcmd.respond_info("  Moving Z up to %.1fmm for safe travel..." % travel_z)
            toolhead.manual_move([None, None, travel_z], 10.0)
            toolhead.wait_moves()

            # Move XY to pin location at safe height
            gcmd.respond_info("  Moving to pin XY location (X=%.1f, Y=%.1f)..." % (pin_x, pin_y))
            toolhead.manual_move([pin_x, pin_y, None], 50.0)
            toolhead.wait_moves()

            # Now move down to safe_z (14mm = 5mm above 9mm pin)
            gcmd.respond_info("  Moving Z down to %.1fmm (above pin)..." % safe_z)
            toolhead.manual_move([None, None, safe_z], 10.0)
            toolhead.wait_moves()

            # Run 3 phases of probing
            for phase in range(3):
                phase_names = ["first (rough)", "second (refine)", "final (accurate)"]
                num_z = PHASE_Z_HITS[phase]
                num_xy = PHASE_XY_HITS[phase]
                clearance = PHASE_CLEARANCE[phase]

                gcmd.respond_info("  --- Phase %d: %s ---" % (phase, phase_names[phase]))
                gcmd.respond_info("      %d Z probes, %d XY probes, %.1fmm clearance" %
                                 (num_z, num_xy, clearance))

                # Z probing for this phase (if any)
                if num_z > 0:
                    gcmd.respond_info("    Probing Z (%d samples)..." % num_z)

                    # Move to current center XY
                    toolhead.manual_move([current_center[0], current_center[1], None], 25.0)
                    toolhead.wait_moves()

                    # Move to clearance above expected pin top
                    probe_start_z = pin_height + clearance
                    toolhead.manual_move([None, None, probe_start_z], 10.0)
                    toolhead.wait_moves()

                    z_samples = []
                    for z_i in range(num_z):
                        # Probe down from clearance position
                        z_result = self.run_probe(gcmd, travel=clearance + 5.0)
                        z_samples.append(z_result[2])

                        if num_z <= 3:
                            gcmd.respond_info("      Z sample %d/%d: %.4f" %
                                             (z_i+1, num_z, z_result[2]))

                        # Retract to clearance for next sample
                        toolhead.manual_move([None, None, z_result[2] + clearance], 10.0)
                        toolhead.wait_moves()

                    # Average Z
                    avg_z = sum(z_samples) / len(z_samples)
                    current_center = (current_center[0], current_center[1], avg_z)

                    if num_z > 3:
                        z_range = max(z_samples) - min(z_samples)
                        gcmd.respond_info("      Z average: %.4f (range: %.4f)" % (avg_z, z_range))
                    else:
                        gcmd.respond_info("      Z: %.4f" % avg_z)

                # XY probing for this phase
                gcmd.respond_info("    Probing XY (%d probes)..." % num_xy)

                # XY probe depth: 1mm below pin top (PROBE_Z_BORE_MM)
                xy_probe_z = current_center[2] - PROBE_Z_BORE_MM

                hits = []
                for i in range(num_xy):
                    angle = 2 * math.pi * i / num_xy

                    try:
                        hit = self._probe_xy_verified(
                            gcmd, current_center[0], current_center[1], xy_probe_z, angle,
                            start_radius, probe_distance, probe_speed, toolhead, current_center[2])
                        hits.append(hit)
                    except Exception as e:
                        gcmd.respond_info("      XY probe %d FAILED: %s" % (i+1, str(e)))
                        raise

                    # Retract to safe height after each XY probe
                    toolhead.manual_move([None, None, current_center[2] + clearance], 10.0)
                    toolhead.wait_moves()

                # Fit circle to find center
                center_xy = self._fit_circle_center(hits)
                current_center = (center_xy[0], center_xy[1], current_center[2])

                # Check deviation on final phase
                if phase == 2:
                    passed, max_dev = self._check_xy_deviation(center_xy, hits, 0.2)
                    if not passed:
                        gcmd.respond_info("    WARNING: XY deviation %.3fmm exceeds 0.2mm" % max_dev)
                    else:
                        gcmd.respond_info("    XY deviation: %.3fmm (OK)" % max_dev)

                gcmd.respond_info("    Center: X=%.4f Y=%.4f Z=%.4f" %
                                 (current_center[0], current_center[1], current_center[2]))

            # Store final result
            tool_centers[tool_num] = current_center
            gcmd.respond_info("  Tool %d COMPLETE: X=%.4f Y=%.4f Z=%.4f" %
                             (tool_num, current_center[0], current_center[1], current_center[2]))

            # Move to safe height Z=14mm before next tool (Prusa go_to_safe_height)
            gcmd.respond_info("  Moving to safe height...")
            toolhead.manual_move([None, None, safe_z], 10.0)
            toolhead.wait_moves()

        # Calculate offsets relative to Tool 0
        gcmd.respond_info("=== CALCULATING OFFSETS ===")
        if 0 not in tool_centers:
            raise gcmd.error("Tool 0 must be calibrated as reference")

        ref = tool_centers[0]
        offsets = {}
        for tool_num, center in tool_centers.items():
            # Offset = reference - measured (inverted per Prusa)
            offset_x = ref[0] - center[0]
            offset_y = ref[1] - center[1]
            offset_z = ref[2] - center[2]
            offsets[tool_num] = (offset_x, offset_y, offset_z)
            gcmd.respond_info("Tool %d offset: X=%.4f Y=%.4f Z=%.4f" %
                             (tool_num, offset_x, offset_y, offset_z))

        # Bounds check (Prusa: X_MIN/MAX_OFFSET, Y_MIN/MAX_OFFSET, Z_MIN/MAX_OFFSET)
        for tool_num, (ox, oy, oz) in offsets.items():
            if tool_num == 0:
                continue  # T0 is always 0,0,0
            if ox < -1.0 or ox > 1.0:
                gcmd.respond_info("WARNING: T%d X offset %.4f out of bounds (±1mm)" % (tool_num, ox))
            if oy < -1.0 or oy > 1.0:
                gcmd.respond_info("WARNING: T%d Y offset %.4f out of bounds (±1mm)" % (tool_num, oy))
            if oz < -2.0 or oz > 1.45:
                gcmd.respond_info("WARNING: T%d Z offset %.4f out of bounds (-2/+1.45mm)" % (tool_num, oz))

        # Park the last tool
        gcmd.respond_info("Parking tool...")
        gcode.run_script_from_command("TOOL_PARK")
        toolhead.wait_moves()

        # Turn off heaters
        gcmd.respond_info("Turning off heaters...")
        for tool_num in tool_list:
            dwarf_num = tool_num + 1
            gcode.run_script_from_command("SET_DWARF_TEMP DWARF=%d TEMP=0" % dwarf_num)

        # === STEP 8: User removes pin and reinstalls sheet ===
        gcmd.respond_info("")
        gcmd.respond_info("=== STEP 8: REMOVE PIN AND REINSTALL SHEET ===")
        gcmd.respond_info(">>> ACTION REQUIRED:")
        gcmd.respond_info(">>>   1. REMOVE the calibration pin")
        gcmd.respond_info(">>>   2. INSTALL the heatbed sheet")
        gcmd.respond_info(">>> Waiting %.0f seconds..." % wait_time)
        gcmd.respond_info("")
        self._puppy.reactor.pause(self._puppy.reactor.monotonic() + wait_time)

        gcmd.respond_info("=== CALIBRATION COMPLETE ===")

        # Apply offsets to working storage (used during tool changes)
        for tool_num, (ox, oy, oz) in offsets.items():
            self._puppy.tool_offsets[tool_num] = (ox, oy, oz)

        # Save to printer.cfg via SAVE_CONFIG for persistence
        gcmd.respond_info("Saving offsets to printer.cfg...")
        if self._puppy._save_tool_offsets_immediate():
            gcmd.respond_info("Tool offsets saved to printer.cfg [puppy_bootloader] section")
            gcmd.respond_info("Offsets will persist across restarts and power cycles")
        else:
            gcmd.respond_info("WARNING: Auto-save failed - run SAVE_CONFIG manually!")
            # Stage the offsets anyway so manual SAVE_CONFIG works
            self._puppy._save_tool_offsets()

        gcmd.respond_info("Offsets applied - will be used on next tool change")

        # Store in status for retrieval
        self._tool_offsets = offsets
        return offsets

    def cmd_PROBE_ACCURACY(self, gcmd):
        """Test probe accuracy with multiple samples."""
        samples = gcmd.get_int('SAMPLES', 10, minval=1)
        speed = gcmd.get_float('PROBE_SPEED', self._speed, above=0.)
        retract = gcmd.get_float('SAMPLE_RETRACT_DIST', 2.0, above=0.)

        toolhead = self._printer.lookup_object('toolhead')
        
        # Prusa XY_ACCELERATION_MMSS limit (500 mm/s^2) for precision probing
        orig_max_accel = toolhead.max_accel
        toolhead.max_accel = min(500.0, orig_max_accel)

        # Multi-probe begin
        self.multi_probe_begin()

        positions = []
        for i in range(samples):
            # Move up before each probe
            curpos = toolhead.get_position()
            curpos[2] += retract
            toolhead.move(curpos, self._lift_speed)
            toolhead.wait_moves()

            # Probe
            pos = self.run_probe(gcmd)
            positions.append(pos[2])
            gcmd.respond_info("Sample %d: Z=%.4f" % (i + 1, pos[2]))

        self.multi_probe_end()

        # Calculate statistics
        avg = sum(positions) / len(positions)
        min_z = min(positions)
        max_z = max(positions)
        range_z = max_z - min_z

        gcmd.respond_info(
            "Probe accuracy: avg=%.4f min=%.4f max=%.4f range=%.4f (%d samples)" %
            (avg, min_z, max_z, range_z, samples))


class PuppyBootloader:
    """Sends bootloader and MODBUS commands to Prusa Dwarfs"""
    
    # Bootloader commands
    CMD_GET_PROTOCOL_VERSION = 0x00
    CMD_SET_ADDRESS = 0x01
    CMD_GET_HARDWARE_INFO = 0x03
    CMD_START_APPLICATION = 0x05
    CMD_GET_FINGERPRINT = 0x0E
    CMD_COMPUTE_FINGERPRINT = 0x0F
    
    # MODBUS function codes
    MODBUS_READ_COILS = 0x01
    MODBUS_READ_DISCRETE = 0x02
    MODBUS_READ_HOLDING = 0x03
    MODBUS_READ_INPUT = 0x04
    MODBUS_WRITE_COIL = 0x05
    MODBUS_WRITE_SINGLE = 0x06
    MODBUS_WRITE_MULTIPLE = 0x10  # Write multiple holding registers
    MODBUS_READ_FIFO = 0x18

    # Protocol validation constants
    _PROTO_IDENT = "WGxLbGlwcGVyIGJ5IFJpY2hhcmQgQ3Jvb2sgMjAyNg=="
    _PROTO_CHECKSUM = 0x04DB

    # Loadcell constants
    LOADCELL_COIL = 0x4002         # Enable loadcell on Dwarf
    LOADCELL_FIFO_ADDR = 0x0000    # FIFO stream address
    LOADCELL_MSG_TYPE = 0x02       # Message type in FIFO stream
    LOADCELL_SCALE = 0.0192        # grams per raw ADC unit
    LOADCELL_THRESHOLD = 25.0      # grams - Z probe trigger (absolute value)
    LOADCELL_TARE_SAMPLES = 48     # samples for tare averaging (~150ms @ 320Hz)
    LOADCELL_PROBE_SPEED = 5.0     # mm/s for Z probing
    LOADCELL_PROBE_STEP = 0.2      # mm per probe step

    # Dock positions (from Prusa XL toolchanger_utils.h - use directly, no conversion needed)
    # Y axis matches Prusa: Y=-9=FRONT (home), Y=461=BACK, Y+ toward BACK (toward docks)
    # Prusa Y_MIN_POS = -9, same as our position_endstop, so coordinates match directly
    # Tool 0 at X=25, each subsequent +82mm
    DOCK_FIRST_X = 25.0
    DOCK_OFFSET_X = 82.0
    DOCK_Y = 455.0              # DOCK_DEFAULT_Y_MM from Prusa (back of printer, near Y_MAX=460)
    SAFE_Y_WITH_TOOL = 360.0    # SAFE_Y_WITH_TOOL from Prusa
    SAFE_Y_WITHOUT_TOOL = 425.0 # SAFE_Y_WITHOUT_TOOL from Prusa

    # Pick/Park motion offsets (from Prusa toolchanger_utils.h)
    PICK_Y_OFFSET = -5.0       # PICK_Y_OFFSET from Prusa (5mm toward front from dock)
    PICK_X_OFFSET_1 = -11.8    # Lock engage start
    PICK_X_OFFSET_2 = -12.8    # Lock fully engaged
    PICK_X_OFFSET_3 = -9.9     # Lock final position
    PARK_X_OFFSET_1 = -10.0    # Approach dock front
    PARK_X_OFFSET_2 = -9.0     # Unlock start
    PARK_X_OFFSET_3 = 0.5      # Unlock complete
    DOCK_WIGGLE = 0.5          # Fallback wiggle distance

    # Cheese LED register (hotend illumination on each Dwarf)
    LED_REG_CHEESE = 0xE004        # (selected_pwm << 8) | not_selected_pwm
    LED_SELECTED_PWM = 255         # Full brightness when tool is active
    LED_NOT_SELECTED_PWM = 0       # Off when tool is inactive

    # Motion speeds
    SLOW_SPEED = 50            # mm/s for dock insert/remove
    TRAVEL_SPEED = 200         # mm/s for travel moves
    PARKING_CURRENT = 950      # mA during lock/unlock
    NORMAL_CURRENT = 650       # mA normal operation (matches printer.cfg)
    
    # Addresses (Prusa standard: bed=0x1A, dwarfs=0x1B-0x1F)
    # Address = ADDR_MODBUS_OFFSET + dock_index
    # Bed dock_index=0, Dwarf N dock_index=N
    ADDR_DEFAULT = 0x00
    ADDR_FIRST_BOOT = 0x0A
    ADDR_MODBUS_OFFSET = 0x1A  # Base MODBUS address
    ADDR_MODULAR_BED = 0x1A    # Bed at 0x1A (dock_index=0)
    ADDR_DWARF_BASE = 0x1B     # Dwarf 1 at 0x1B (dock_index=1)
    # Dwarf address = ADDR_MODBUS_OFFSET + dwarf, e.g., Dwarf 1 = 0x1B

    # PCA9557 reset pin mapping: bit1=dwarf1, bit2=dwarf2, ..., bit5=dwarf5, bit7=modularBed
    NUM_DWARFS = 5
    ALL_DWARF_BITS = 0x3E  # bits 1-5
    MODULAR_BED_BIT = 0x80  # bit 7

    # Modular bed MODBUS registers (Prusa XL: 16 bedlets in 4x4 grid)
    NUM_BEDLETS = 16
    BED_REG_TARGET_TEMP = 0xE010     # Holding register: target temps (16 regs)
    BED_REG_MEASURED_TEMP = 0xA010   # Input register: measured temps (16 regs)
    BED_REG_FAULT_STATUS = 0x8000    # Input register: system fault status
    BED_REG_BEDLET_FAULT = 0xA000    # Input register: bedlet fault status (16 regs)
    BED_REG_HEATBEDLET_COUNT = 0x8010  # Input register: number of bedlets
    BED_TEMP_SCALE = 10              # Temps in 1/10 degree (60C = 600)
    # Bed coils (for clearing faults - same as Prusa refresh cycle)
    BED_COIL_CLEAR_FAULT = 0x4000    # SystemCoil: clear_fault_status
    BED_COIL_RESET_OVERCURRENT = 0x4001  # SystemCoil: reset_overcurrent_fault
    BED_COIL_PRINT_FAN_ACTIVE = 0x4005   # SystemCoil: print_fan_active

    # Bedlet physical wiring map: grid (row, col) -> MODBUS register index (0-based)
    # From Prusa modular_bed.cpp line 344 (values are 1-indexed there, we subtract 1)
    # Row 0 = front (Y=0..90), Row 3 = back (Y=270..360)
    # Col 0 = left (X=0..90), Col 3 = right (X=270..360)
    BEDLET_MAP = [
        [6,  7,  8,  9 ],   # Row 0 (front, Y 0-90)
        [5,  4,  11, 10],   # Row 1 (Y 90-180)
        [2,  3,  12, 13],   # Row 2 (Y 180-270)
        [1,  0,  15, 14],   # Row 3 (back, Y 270-360)
    ]
    # Reverse map: MODBUS register index -> (col, row)
    BEDLET_GRID = {}
    for _row in range(4):
        for _col in range(4):
            BEDLET_GRID[BEDLET_MAP[_row][_col]] = (_col, _row)
    BEDLET_SIZE = 90  # mm per bedlet (Prusa: X_HBL_SIZE = Y_HBL_SIZE = 90)

    # Dwarf fault status register (from Prusa dwarf_registers.hpp)
    DWARF_REG_FAULT_STATUS = 0x8060  # Input register: fault_status
    # Fault status bits (FaultStatusMask enum)
    FAULT_NO_FAULT = 0
    FAULT_MARLIN_KILLED = (1 << 1)   # Thermal runaway, min/max temp, etc.
    FAULT_TMC_FAULT = (1 << 2)       # TMC driver fault

    # Modular bed fault status (from Prusa modular_bed_errors.hpp)
    # System faults (register 0x8000)
    BED_FAULT_PREHEAT_ERROR = 0x0001      # Bed not reaching temperature
    BED_FAULT_MINTEMP = 0x0002            # Temperature below minimum
    BED_FAULT_MAXTEMP = 0x0004            # Temperature above maximum
    BED_FAULT_THERMAL_RUNAWAY = 0x0008    # Thermal runaway detected
    BED_FAULT_HEATBEDLET_ERROR = 0x0010   # Individual bedlet fault
    BED_FAULT_OVERCURRENT = 0x0020        # Overcurrent protection
    BED_FAULT_UNEXPECTED_CURRENT = 0x0040 # Unexpected current draw
    # Critical faults that require emergency stop
    BED_CRITICAL_FAULTS = (BED_FAULT_THERMAL_RUNAWAY | BED_FAULT_MAXTEMP |
                           BED_FAULT_OVERCURRENT)
    # Bedlet-level faults (register 0xA000 + bedlet)
    BEDLET_FAULT_HEATER_DISCONNECTED = 0x0001
    BEDLET_FAULT_SHORT_CIRCUIT = 0x0002
    BEDLET_FAULT_TEMP_BELOW_MIN = 0x0004
    BEDLET_FAULT_TEMP_ABOVE_MAX = 0x0008
    BEDLET_FAULT_TEMP_DROP = 0x0010
    BEDLET_FAULT_TEMP_PEAK = 0x0020
    BEDLET_FAULT_PREHEAT = 0x0040
    BEDLET_FAULT_TEST_HEATING = 0x0080
    # Critical bedlet faults
    BEDLET_CRITICAL_FAULTS = (BEDLET_FAULT_SHORT_CIRCUIT | BEDLET_FAULT_TEMP_ABOVE_MAX)

    # CRC16 table (works for both bootloader and MODBUS - same polynomial)
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
        self.config = config  # Store config for _load_tool_offsets
        self.printer = config.get_printer()
        self.gcode = self.printer.lookup_object('gcode')
        self.mcu = self.printer.lookup_object('mcu')
        self.send_raw_cmd = None
        self.loadcell_cmd = None
        self.probe_start_cmd = None
        self.probe_stop_cmd = None
        self.probe_query_cmd = None
        self.tare_mcu_cmd = None
        self.probe_has_xy = False
        self.loadcell_home_cmd = None
        # Create LoadcellEndstops NOW during config phase so TriggerDispatch/MCU_trsync
        # can register their config callbacks and _build_config runs at the right time
        # We need separate endstops for Z probing and XY probing (different steppers)
        self.loadcell_endstop = LoadcellEndstop(self.printer, self.mcu, self)  # For Z probing
        self.loadcell_endstop_xy = LoadcellEndstop(self.printer, self.mcu, self)  # For XY probing
        logging.info("PuppyBootloader: LoadcellEndstops created (Z and XY) during config phase")

        # Create LoadcellProbe and register as 'probe' for bed_mesh, etc.
        self.loadcell_probe = LoadcellProbe(self.printer, self.loadcell_endstop, self)
        self.printer.add_object('probe', self.loadcell_probe)
        logging.info("PuppyBootloader: LoadcellProbe registered as 'probe'")

        # Polling state
        self.reactor = self.printer.get_reactor()
        self.booted_dwarfs = []  # List of booted Dwarf numbers
        self.polling_active = False
        self.poll_timer = None
        self.poll_dwarf_index = 0  # Which Dwarf to poll next
        self.dwarf_data = {}  # Latest register data per Dwarf
        self.poll_interval = 5  # Seconds between full poll cycles (watchdog is 30s)

        # Tool state
        self.active_tool = 0  # -1 = no tool picked, 0-4 = T0-T4 (default 0 for dev)
        self.target_temps = {}  # {dwarf_num: target_temp}
        self.fan_speeds = {}  # {dwarf_num: speed 0-255}
        self.tmc_enabled = {}  # {dwarf_num: True/False}
        self.tool_picked = False  # True if a tool is on the carriage
        self.led_pending = set()  # Dwarfs needing cheese LED config retry
        self.tool_remap = {}  # {gcode_tool: physical_tool} for spool join
        self._button_extrude_active = False  # True while button-driven extrude is in progress
        self._pending_offset_tool = None  # Tool needing offset applied after ready

        # Heater sync state - syncs [extruder] heater target to MODBUS
        self._extruder_heater = None  # Reference to [extruder] heater object
        self._last_synced_temp = 0    # Last temp we sent to MODBUS
        self._heater_sync_timer = None  # Separate timer for heater sync (not nested in poll)

        # MODBUS health monitoring (Phase 2)
        self._modbus_consecutive_failures = {}  # {dwarf: failure_count}
        self._modbus_failure_threshold = 10     # Failures before escalation (increased from 5)
        self._modbus_last_success = {}          # {dwarf: monotonic_time}

        # Modular bed state (Prusa XL: 16 bedlets in 4x4 grid)
        self.modular_bed_booted = False
        self.bed_target_temps = {}  # {bedlet: target_temp} (0-15)
        self.bed_measured_temps = {}  # {bedlet: measured_temp}
        self.bed_last_modbus_time = 0  # Rate limiting for bed MODBUS commands
        self.bed_modbus_min_interval = 0.1  # Minimum seconds between bed commands
        self.bed_faults_cleared = False  # Track if we've cleared faults since boot

        # Adaptive bed heating state (Prusa modular_bed.cpp)
        self.bed_enabled_mask = 0xFFFF       # 16-bit mask, all enabled by default
        self.bed_print_area = None           # (x0, y0, x1, y1) or None = whole bed
        self.bed_global_target = 0.0         # The "logical" bed target temp
        self.bed_gradient_cutoff = 2.0       # Prusa default (bedlet units)
        self.bed_gradient_exponent = 2.0     # Prusa default
        self.bed_expand_to_sides = True      # Prusa default

        # Tool offsets from calibration (relative to T0)
        # Format: {tool_num: (x_offset, y_offset, z_offset)}
        # Loaded from file on startup, saved after calibration
        self.tool_offsets_file = "/home/pi/printer_data/config/tool_offsets.cfg"
        self.tool_offsets = self._load_tool_offsets()

        logging.info(f"PuppyBootloader: tool_offsets after load: {self.tool_offsets}")
        # Prusa-style applied offset tracking (like hotend_currently_applied_offset)
        # This tracks what XYZ offset is currently applied to the coordinate system.
        # On tool change, we calculate the DIFFERENCE between old and new tool offsets
        # and apply PHYSICAL Z movement to compensate (not just coordinate shift).
        # Format: (x, y, z) - the total offset currently applied
        self.applied_tool_offset = (0.0, 0.0, 0.0)

        # NOTE: No loadcell_z_offset needed!
        # With Prusa XL loadcell probing, the nozzle IS the probe.
        # When loadcell triggers = nozzle at bed surface = Z=0
        # Tool offsets from G425 calibration handle all tool-to-tool differences.
        # This matches how Prusa XL works - no Live-Z or probe offset needed.

        # Filament sensor state (tool sensors via MODBUS 0x8063)
        # Calibration: {tool_num: [ref_nins, ref_ins]} loaded from variables.cfg
        self.fs_calibration = {}
        # Current state per tool: "has_filament", "no_filament", "not_calibrated", "not_connected"
        self.fs_state = {}
        # Debounce counter for runout (need 2 consecutive no_filament readings)
        self.fs_runout_count = {}
        self.filament_runout_enabled = True
        # Cal loaded at connect time (save_variables not ready during __init__)

        # Side filament sensor state (ADC3 + mux on XLBuddy sandwich board)
        # These are the PRIMARY runout sensors (matching Prusa stock firmware)
        # 5 sensors: 3 left (T0,T1,T2) + 2 right (T3,T4) near spools
        self.side_fs_cmd = None
        self.side_fs_calibration = {}    # {tool: [ref_nins, ref_ins]}
        self.side_fs_state = {}          # {tool: state_string}
        self.side_fs_raw = {}            # {tool: last_raw_adc}
        self.side_fs_runout_count = {}   # {tool: consecutive_no_filament_count}
        self.autoload_enabled = True     # Autoload when side sensor detects insertion
        self._autoload_in_progress = False
        self._autoload_insert_count = {}  # {tool: consecutive has_filament count}

        # Loadcell state
        self.loadcell_enabled = {}   # {dwarf_num: True/False}
        self.loadcell_offset = {}    # {dwarf_num: raw_offset} for tare
        self.loadcell_last_raw = {}  # {dwarf_num: last_raw_value}
        self.last_probe_z = None     # Last Z probe result

        # Dock positions (can be overridden by DOCK_CALIBRATE)
        self.dock_positions = {}
        for i in range(5):
            self.dock_positions[i] = {
                'x': self.DOCK_FIRST_X + i * self.DOCK_OFFSET_X,
                'y': self.DOCK_Y
            }

        # Register config callback for MCU setup
        self.mcu.register_config_callback(self._build_config)
        self.printer.register_event_handler("klippy:connect", self._handle_connect)
        self.printer.register_event_handler("klippy:disconnect", self._handle_disconnect)
        self.printer.register_event_handler("klippy:shutdown", self._handle_shutdown)
        
        # Bootloader commands
        self.gcode.register_command("BOOTLOADER_SEND", self.cmd_BOOTLOADER_SEND,
            desc="Send raw bootloader command")
        self.gcode.register_command("BOOTLOADER_START_APP", self.cmd_BOOTLOADER_START_APP,
            desc="Send START_APPLICATION to boot Dwarf")
        self.gcode.register_command("BOOTLOADER_DISCOVER", self.cmd_BOOTLOADER_DISCOVER,
            desc="Try to discover Dwarf in bootloader mode")
        self.gcode.register_command("BOOTLOADER_HWINFO", self.cmd_BOOTLOADER_HWINFO,
            desc="Get hardware info from Dwarf in bootloader mode")
        self.gcode.register_command("BOOTLOADER_ASSIGN", self.cmd_BOOTLOADER_ASSIGN,
            desc="Assign new address to Dwarf")
        self.gcode.register_command("BOOTLOADER_COMPUTE_FP", self.cmd_BOOTLOADER_COMPUTE_FP,
            desc="Ask Dwarf to compute fingerprint")
        self.gcode.register_command("BOOTLOADER_GET_FP", self.cmd_BOOTLOADER_GET_FP,
            desc="Get computed fingerprint from Dwarf")
        self.gcode.register_command("BOOTLOADER_BOOT_DWARF", self.cmd_BOOTLOADER_BOOT_DWARF,
            desc="Full boot sequence for a single Dwarf")
        
        # MODBUS commands
        self.gcode.register_command("MODBUS_READ", self.cmd_MODBUS_READ,
            desc="Read MODBUS holding registers")
        self.gcode.register_command("MODBUS_SCAN", self.cmd_MODBUS_SCAN,
            desc="Scan for MODBUS devices")
        self.gcode.register_command("MODBUS_WRITE", self.cmd_MODBUS_WRITE,
            desc="Write MODBUS register")

        # Standard gcode commands for heater/fan/tool control
        # Override M104/M109 at klippy:ready to handle T parameter for multi-tool
        # (Klipper's default handlers don't understand our Dwarf/MODBUS tool system)
        self._has_klipper_extruder = config.has_section('extruder')
        self._m104_overridden = False
        self.printer.register_event_handler("klippy:ready", self._override_m104_m109)
        self.printer.register_event_handler("klippy:ready", self._override_turn_off_heaters)
        self.printer.register_event_handler("klippy:ready", self._apply_pending_offset)
        self.gcode.register_command("M106", self.cmd_M106,
            desc="Set fan speed")
        self.gcode.register_command("M107", self.cmd_M107,
            desc="Turn fan off")
        self.gcode.register_command("DWARF_STATUS", self.cmd_DWARF_STATUS,
            desc="Show all Dwarf temperatures and status")

        # Modular bed commands
        self.gcode.register_command("M140", self.cmd_M140,
            desc="Set bed temperature")
        self.gcode.register_command("M190", self.cmd_M190,
            desc="Set bed temperature and wait")
        self.gcode.register_command("BED_STATUS", self.cmd_BED_STATUS,
            desc="Show modular bed temperatures")
        self.gcode.register_command("BED_DIAG", self.cmd_BED_DIAG,
            desc="Show modular bed fault diagnostics")
        self.gcode.register_command("SET_BEDLET_TEMP", self.cmd_SET_BEDLET_TEMP,
            desc="Set individual bedlet temperature")
        self.gcode.register_command("SET_DWARF_TEMP", self.cmd_SET_DWARF_TEMP,
            desc="Set Dwarf heater temperature directly")
        # Adaptive bed heating commands
        self.gcode.register_command("SET_BED_AREA", self.cmd_SET_BED_AREA,
            desc="Set print area for adaptive bed heating")
        self.gcode.register_command("CLEAR_BED_AREA", self.cmd_CLEAR_BED_AREA,
            desc="Reset to full bed heating")

        # Prusa XL slicer compatibility commands
        self.gcode.register_command("M151", self.cmd_M151,
            desc="Set LED strip color (stub)")
        self.gcode.register_command("P0", self.cmd_P0,
            desc="Park current tool")
        self.gcode.register_command("M17", self.cmd_M17,
            desc="Enable steppers (stub)")
        # Marlin motion settings (Klipper uses config instead)
        self.gcode.register_command("M201", self.cmd_M201_stub,
            desc="Max acceleration (stub)")
        self.gcode.register_command("M203", self.cmd_M203_stub,
            desc="Max feedrate (stub)")
        self.gcode.register_command("M205", self.cmd_M205_stub,
            desc="Jerk settings (stub)")
        # Prusa firmware check stubs (accept but ignore)
        self.gcode.register_command("M862.3", self.cmd_M862_stub,
            desc="Printer model check (stub)")
        self.gcode.register_command("M862.5", self.cmd_M862_stub,
            desc="G-code level check (stub)")
        self.gcode.register_command("M862.6", self.cmd_M862_stub,
            desc="FW feature check (stub)")
        self.gcode.register_command("M862.1", self.cmd_M862_stub,
            desc="Nozzle diameter check (stub)")
        self.gcode.register_command("M555", self.cmd_M555,
            desc="Set print area (stub)")
        self.gcode.register_command("M217", self.cmd_M217,
            desc="Toolchange settings (stub)")
        self.gcode.register_command("M591", self.cmd_M591,
            desc="Filament runout detection enable/disable")
        self.gcode.register_command("M302", self.cmd_M302,
            desc="Cold extrusion setting (stub)")
        # M84/M18 - Override Klipper's default to handle Prusa's axis parameters
        # This preserves homing state when only E is specified (M84 E)
        self._original_m84 = None
        self.printer.register_event_handler("klippy:ready", self._override_m84)
        # Prusa G29 variants (Klipper uses BED_MESH_CALIBRATE)
        self.gcode.register_command("G29", self.cmd_G29,
            desc="Prusa bed mesh / nozzle cleanup")
        # Prusa print timer commands
        self.gcode.register_command("M77", self.cmd_M77,
            desc="Stop print timer (stub)")
        # Prusa pause/resume
        self.gcode.register_command("M601", self.cmd_M601,
            desc="Pause print")
        self.gcode.register_command("M600", self.cmd_M600,
            desc="Filament change (pause + unload + load)")

        # Filament sensor commands
        self.gcode.register_command("CALIBRATE_FILAMENT_SENSOR",
            self.cmd_CALIBRATE_FILAMENT_SENSOR,
            desc="Calibrate filament sensor (step 1: remove filament)")
        self.gcode.register_command("CALIBRATE_FILAMENT_SENSOR_STEP2",
            self.cmd_CALIBRATE_FILAMENT_SENSOR_STEP2,
            desc="Calibrate filament sensor (step 2: insert filament)")
        self.gcode.register_command("SHOW_FILAMENT_SENSORS",
            self.cmd_SHOW_FILAMENT_SENSORS,
            desc="Show filament sensor states and calibration")
        self.gcode.register_command("LOAD_FILAMENT", self.cmd_LOAD_FILAMENT,
            desc="Load filament into tool (Prusa M701)")
        self.gcode.register_command("UNLOAD_FILAMENT", self.cmd_UNLOAD_FILAMENT,
            desc="Unload filament from tool (Prusa M702)")

        # Side filament sensor commands (PRIMARY runout detection)
        self.gcode.register_command("CALIBRATE_SIDE_SENSOR",
            self.cmd_CALIBRATE_SIDE_SENSOR,
            desc="Calibrate side filament sensor (step 1: remove filament)")
        self.gcode.register_command("CALIBRATE_SIDE_SENSOR_STEP2",
            self.cmd_CALIBRATE_SIDE_SENSOR_STEP2,
            desc="Calibrate side filament sensor (step 2: insert filament)")
        self.gcode.register_command("SHOW_SIDE_SENSORS",
            self.cmd_SHOW_SIDE_SENSORS,
            desc="Show side filament sensor states and calibration")

        # Autoload control
        self.gcode.register_command("AUTOLOAD_FILAMENT",
            self.cmd_AUTOLOAD_FILAMENT,
            desc="Manually trigger filament autoload for a tool")
        self.gcode.register_command("SET_AUTOLOAD",
            self.cmd_SET_AUTOLOAD,
            desc="Enable/disable filament autoload (SET_AUTOLOAD ENABLE=0/1)")

        # Tool select commands T0-T4
        for i in range(5):
            self.gcode.register_command(f"T{i}", self._make_tool_cmd(i),
                desc=f"Select tool {i}")

        # Extruder/TMC commands
        self.gcode.register_command("EXTRUDER_ENABLE", self.cmd_EXTRUDER_ENABLE,
            desc="Enable extruder TMC on active tool")
        self.gcode.register_command("EXTRUDER_DISABLE", self.cmd_EXTRUDER_DISABLE,
            desc="Disable extruder TMC on active tool")
        self.gcode.register_command("TMC_STATUS_DWARF", self.cmd_TMC_STATUS_DWARF,
            desc="Read TMC2130 status from Dwarf")
        self.gcode.register_command("TMC_WRITE_DWARF", self.cmd_TMC_WRITE_DWARF,
            desc="Write TMC2130 register on Dwarf")
        self.gcode.register_command("DWARF_SELECT", self.cmd_DWARF_SELECT,
            desc="Set is_selected coil on Dwarf (enables remote step/dir)")
        self.gcode.register_command("MSCNT_TEST", self.cmd_MSCNT_TEST,
            desc="Test if step pulses reach Dwarf TMC via MSCNT register")

        # Tool change commands
        self.gcode.register_command("TOOL_PICK", self.cmd_TOOL_PICK,
            desc="Pick up a tool from its dock")
        self.gcode.register_command("TOOL_PARK", self.cmd_TOOL_PARK,
            desc="Park the current tool in its dock")
        self.gcode.register_command("DOCK_STATUS", self.cmd_DOCK_STATUS,
            desc="Read dock Hall sensors from Dwarf")
        self.gcode.register_command("DOCK_POSITIONS", self.cmd_DOCK_POSITIONS,
            desc="Show configured dock positions")
        self.gcode.register_command("DOCK_CALIBRATE", self.cmd_DOCK_CALIBRATE,
            desc="Calibrate dock position (tool must be manually attached)")

        # Spool join commands
        self.gcode.register_command("SPOOL_JOIN", self.cmd_SPOOL_JOIN,
            desc="Join spool: remap tool and transfer temp (use while paused)")
        self.gcode.register_command("SPOOL_JOIN_STATUS", self.cmd_SPOOL_JOIN_STATUS,
            desc="Show active spool join remaps")
        self.gcode.register_command("SPOOL_JOIN_RESET", self.cmd_SPOOL_JOIN_RESET,
            desc="Clear all spool join remaps")


        # Tool state override
        self.gcode.register_command("SET_TOOL_STATE", self.cmd_SET_TOOL_STATE,
            desc="Force tool state (for recovery after restart)")

        # Tool offset commands
        self.gcode.register_command("GET_TOOL_OFFSETS", self.cmd_GET_TOOL_OFFSETS,
            desc="Display current tool offsets")
        self.gcode.register_command("SHOW_TOOL_OFFSETS", self.cmd_GET_TOOL_OFFSETS,
            desc="Display current tool offsets (alias)")
        # NOTE: SAVE_TOOL_OFFSETS removed - CALIBRATE_TOOL_OFFSETS auto-saves

        # NOTE: Z offset commands removed - not needed with Prusa-style loadcell probing
        # The loadcell probes with the nozzle itself, so Z=0 = nozzle at bed surface
        # Tool offsets from CALIBRATE_TOOL_OFFSETS handle all tool-to-tool differences

        # Loadcell commands
        self.gcode.register_command("LOADCELL_ENABLE", self.cmd_LOADCELL_ENABLE,
            desc="Enable loadcell on tool")
        self.gcode.register_command("LOADCELL_DISABLE", self.cmd_LOADCELL_DISABLE,
            desc="Disable loadcell on tool")
        self.gcode.register_command("LOADCELL_READ", self.cmd_LOADCELL_READ,
            desc="Read loadcell value from tool")
        self.gcode.register_command("LOADCELL_TARE", self.cmd_LOADCELL_TARE,
            desc="Tare (zero) loadcell on tool")
        self.gcode.register_command("LOADCELL_PROBE", self.cmd_LOADCELL_PROBE,
            desc="Z probe using loadcell on active tool")
        self.gcode.register_command("Z_CALIBRATION_RUN", self.cmd_Z_CALIBRATION,
            desc="Z Calibration internal command (use Z_CALIBRATION macro instead)")

        # Initialize accelerometer support for input shaper calibration
        self.accelerometer = None
        if DwarfAccelerometer is not None:
            try:
                self.accelerometer = DwarfAccelerometer(self)
                logging.info("PuppyBootloader: DwarfAccelerometer initialized")
            except Exception as e:
                logging.warning("PuppyBootloader: Failed to initialize accelerometer: %s" % e)
        else:
            logging.info("PuppyBootloader: Accelerometer support not available")

        logging.info("PuppyBootloader: initialized")
    
    def _build_config(self):
        """Called during MCU config phase - add config commands here"""
        self.mcu.add_config_cmd("config_modbus")
        logging.info("PuppyBootloader: Added config_modbus command")

    def _load_tool_offsets(self):
        """Load tool offsets from printer.cfg [puppy_bootloader] section.

        Offsets are stored in SAVE_CONFIG section as:
        [puppy_bootloader]
        tool_offset_t0 = 0.0000, 0.0000, 0.0000
        tool_offset_t1 = 0.0263, 0.1986, 2.7650
        etc.
        """
        offsets = {}
        
        # Declare options with defaults and read values
        for tool in range(5):
            key = f'tool_offset_t{tool}'
            default = '0.0, 0.0, 0.0'
            try:
                value = self.config.get(key, default)
                parts = [float(v.strip()) for v in value.split(',')]
                if len(parts) == 3:
                    offsets[tool] = (parts[0], parts[1], parts[2])
                    if offsets[tool] != (0.0, 0.0, 0.0):
                        logging.info(f"PuppyBootloader: Loaded T{tool} offset: "
                                   f"X={parts[0]:.4f} Y={parts[1]:.4f} Z={parts[2]:.4f}")
                else:
                    offsets[tool] = (0.0, 0.0, 0.0)
            except Exception as e:
                logging.info(f"PuppyBootloader: Error loading T{tool} offset: {e}")
                offsets[tool] = (0.0, 0.0, 0.0)

        # Also try loading from legacy file for migration
        try:
            if os.path.exists(self.tool_offsets_file):
                with open(self.tool_offsets_file, 'r') as f:
                    for line in f:
                        line = line.strip()
                        if line.startswith('#') or not line:
                            continue
                        if line.startswith('T') and ':' in line:
                            parts = line.split(':')
                            tool = int(parts[0][1:])
                            values = parts[1].strip().split()
                            x = float(values[0].split('=')[1])
                            y = float(values[1].split('=')[1])
                            z = float(values[2].split('=')[1])
                            # Only use legacy if not already loaded from config
                            if offsets[tool] == (0.0, 0.0, 0.0) or tool == 0:
                                if (x, y, z) != (0.0, 0.0, 0.0):
                                    offsets[tool] = (x, y, z)
                                    logging.info(f"PuppyBootloader: Migrated T{tool} offset from legacy file")
        except:
            pass

        return offsets

    def _save_tool_offsets(self):
        """Save tool offsets to printer.cfg via Klipper's SAVE_CONFIG mechanism.

        This saves offsets persistently in the [puppy_bootloader] section
        at the bottom of printer.cfg (in the auto-generated SAVE_CONFIG area).
        """
        try:
            configfile = self.printer.lookup_object('configfile')

            # Save each tool offset
            for tool in sorted(self.tool_offsets.keys()):
                x, y, z = self.tool_offsets[tool]
                key = f'tool_offset_t{tool}'
                value = f'{x:.4f}, {y:.4f}, {z:.4f}'
                configfile.set('puppy_bootloader', key, value)

            logging.info(f"PuppyBootloader: Tool offsets staged for SAVE_CONFIG")
            logging.info(f"PuppyBootloader: Run SAVE_CONFIG to persist to printer.cfg")
            return True
        except Exception as e:
            logging.error(f"PuppyBootloader: Failed to stage tool offsets: {e}")
            return False

    def _save_tool_offsets_immediate(self):
        """Save tool offsets and trigger SAVE_CONFIG automatically."""
        if self._save_tool_offsets():
            try:
                # Trigger save_config to write to printer.cfg
                gcode = self.printer.lookup_object('gcode')
                gcode.run_script_from_command("SAVE_CONFIG")
                return True
            except Exception as e:
                logging.error(f"PuppyBootloader: SAVE_CONFIG failed: {e}")
                return False
        return False

    # NOTE: _load_z_offset, _save_z_offset, _get_effective_z_offset, _apply_z_offsets removed
    # With Prusa-style loadcell probing, no separate Z offset is needed:
    # - Loadcell probes WITH the nozzle itself
    # - Z=0 = where nozzle touches bed surface
    # - Tool offsets from CALIBRATE_TOOL_OFFSETS handle all tool-to-tool differences
    # This matches Prusa XL's approach - no Live-Z needed

    # --- Filament Sensor Methods (Prusa FSensorADC equivalent) ---
    # Matches Prusa's filament_sensor_adc_eval.cpp algorithm:
    # - Two-value calibration: ref_nins (no filament) and ref_ins (filament inserted)
    # - Midpoint detection with 1/6-span hysteresis zone
    # - Orientation-agnostic (works regardless of magnet polarity)

    # Prusa constants
    FS_VALUE_SPAN = 1000        # extruder_fs_value_span (hysteresis base for uncalibrated)
    FS_CAL_MIN_DIFF = 200       # Minimum ADC difference for valid calibration
                                # (Prusa uses span*1.2=1200 but some sensors swing less)
    FS_ADC_MIN = 20             # Below this = sensor not connected
    FS_ADC_MAX = 4096           # 12-bit ADC maximum
    FS_RUNOUT_DEBOUNCE = 2      # Consecutive no_filament readings before triggering

    # Side sensor constants (Prusa: side filament sensors near spools)
    SIDE_FS_VALUE_SPAN = 310    # side_fs_value_span (Prusa constant for side sensors)
    SIDE_FS_CAL_MIN_DIFF = 200  # Side sensors may have smaller swing than tool sensors

    def _load_filament_cal(self):
        """Load filament sensor calibration from variables.cfg."""
        try:
            save_vars = self.printer.lookup_object("save_variables", None)
            if save_vars is not None:
                variables = save_vars.allVariables
                for tool in range(5):
                    key = f"fs_cal_t{tool}"
                    cal = variables.get(key, None)
                    if cal is not None and isinstance(cal, list) and len(cal) == 2:
                        self.fs_calibration[tool] = cal
                        logging.info(f"PuppyBootloader: Loaded filament cal T{tool}: "
                                   f"nins={cal[0]} ins={cal[1]}")
                # Load runout enabled state
                enabled = variables.get("fs_runout_enabled", 1)
                self.filament_runout_enabled = bool(enabled)
                logging.info(f"PuppyBootloader: Filament runout detection: "
                           f"{'enabled' if self.filament_runout_enabled else 'disabled'}")
        except Exception as e:
            logging.info(f"PuppyBootloader: No filament cal loaded: {e}")

    def _save_filament_cal(self, tool, ref_nins, ref_ins):
        """Save filament sensor calibration for one tool to variables.cfg."""
        save_vars = self.printer.lookup_object("save_variables", None)
        if save_vars is None:
            raise self.gcode.error("save_variables not configured")
        key = f"fs_cal_t{tool}"
        value = f"[{ref_nins}, {ref_ins}]"
        save_vars.cmd_SAVE_VARIABLE(self.gcode.create_gcode_command(
            "SAVE_VARIABLE", "SAVE_VARIABLE", {"VARIABLE": key, "VALUE": value}))
        self.fs_calibration[tool] = [ref_nins, ref_ins]
        logging.info(f"PuppyBootloader: Saved filament cal T{tool}: nins={ref_nins} ins={ref_ins}")

    def _save_fs_runout_enabled(self):
        """Save filament runout enabled state to variables.cfg."""
        save_vars = self.printer.lookup_object("save_variables", None)
        if save_vars is None:
            return
        value = "1" if self.filament_runout_enabled else "0"
        save_vars.cmd_SAVE_VARIABLE(self.gcode.create_gcode_command(
            "SAVE_VARIABLE", "SAVE_VARIABLE",
            {"VARIABLE": "fs_runout_enabled", "VALUE": value}))

    def _evaluate_filament_state(self, filtered_value, tool):
        """Evaluate filament sensor state matching Prusa's evaluate_state().

        Algorithm from Prusa filament_sensor_adc_eval.cpp:
        1. Check ADC range (20-4096) for connectivity
        2. Check if calibrated (ref_nins exists)
        3. Half-calibrated: use span-based detection
        4. Full calibration: midpoint with 1/6-span hysteresis
        """
        # Not connected check (Prusa: lower_limit=20, upper_limit=4096)
        if filtered_value < self.FS_ADC_MIN or filtered_value > self.FS_ADC_MAX:
            return "not_connected"

        cal = self.fs_calibration.get(tool, None)
        if cal is None:
            return "not_calibrated"

        ref_nins = cal[0]
        ref_ins = cal[1] if len(cal) > 1 else None

        # Half-calibrated (only nins set) - use span-based detection
        if ref_ins is None:
            if (filtered_value < ref_nins - self.FS_VALUE_SPAN or
                    filtered_value > ref_nins + self.FS_VALUE_SPAN):
                return "has_filament"
            return "no_filament"

        # Full calibration - midpoint with 1/6 hysteresis (Prusa algorithm)
        midpoint = (ref_nins + ref_ins) // 2
        hysteresis = abs(ref_nins - ref_ins) // 6

        # In hysteresis zone - maintain previous state to prevent jitter
        prev_state = self.fs_state.get(tool, "not_calibrated")
        if prev_state in ("has_filament", "no_filament"):
            if abs(filtered_value - midpoint) < hysteresis:
                return prev_state

        # Clear transition - which side of midpoint matches the inserted reference?
        if (filtered_value > midpoint) == (ref_ins > midpoint):
            return "has_filament"
        return "no_filament"

    def _check_filament_runout(self, tool, new_state):
        """Tool sensor state tracking (logging only).

        Tool sensors are NOT used for runout detection (Prusa uses side sensors).
        This just logs state transitions for diagnostics. Side sensor
        _check_side_filament_runout() handles the actual runout PAUSE.
        """
        pass

    def _read_tool_buttons(self, dwarf):
        """Read button state from Dwarf discrete inputs.
        Returns (btn_up, btn_down) or (False, False) on error."""
        addr = self.ADDR_MODBUS_OFFSET + dwarf
        data = [0x00, 0x00, 0x00, 0x04]  # Start at 0x0000, read 4 bits
        frame = self._build_modbus_frame(addr, self.MODBUS_READ_DISCRETE, data)
        try:
            response = self._send_frame(frame)
            status = response.get('status', -1)
            resp_data = response.get('data', b'')
            if status != 0 or not resp_data or len(resp_data) < 4:
                return False, False
            bits = resp_data[3]
            return bool(bits & 0x04), bool(bits & 0x08)
        except Exception:
            return False, False

    def _check_tool_buttons(self, dwarf, tool):
        """Check tool buttons on active picked tool.
        Button up = retract, button down = extrude.
        Loops tight while button is held for responsive feel."""
        btn_up, btn_down = self._read_tool_buttons(dwarf)

        if not btn_up and not btn_down:
            self._button_extrude_active = False
            return

        # Check tool is hot enough to extrude (170C minimum, matches Prusa EXTRUDE_MINTEMP)
        temp = self.dwarf_data.get(dwarf, {}).get('hotend_temp', 0)
        if temp < 170:
            if not self._button_extrude_active:
                self.gcode.respond_info(
                    f"T{tool} too cold ({temp}C) - heat above 170C to use buttons")
            return

        # Tight loop while button is held - re-read button state between chunks
        self._button_extrude_active = True
        direction = -1.0 if btn_up else 1.0  # Up = retract, Down = extrude
        chunk = 2.0  # 2mm per chunk
        speed = 5.0  # 5mm/s
        try:
            while True:
                self.gcode.run_script_from_command(
                    f"G91\nG1 E{chunk * direction:.1f} F{speed * 60:.0f}\nG90")
                # Re-read button state immediately after move completes
                btn_up, btn_down = self._read_tool_buttons(dwarf)
                if not btn_up and not btn_down:
                    break
        except Exception as e:
            logging.info(f"PuppyBootloader: Button extrude error: {e}")
        self._button_extrude_active = False

    # --- Side Filament Sensor Methods (PRIMARY runout detection, Prusa stock) ---
    # Side sensors are mounted near the spools: 3 left (T0,T1,T2) + 2 right (T3,T4)
    # Read via ADC3 + GPIO-controlled mux on the XLBuddy sandwich board.
    # Prusa uses these as the PRIMARY runout trigger (not the tool sensors).

    def _load_side_filament_cal(self):
        """Load side filament sensor calibration from variables.cfg."""
        try:
            save_vars = self.printer.lookup_object("save_variables", None)
            if save_vars is not None:
                variables = save_vars.allVariables
                for tool in range(5):
                    key = f"side_fs_cal_t{tool}"
                    cal = variables.get(key, None)
                    if cal is not None and isinstance(cal, list) and len(cal) == 2:
                        self.side_fs_calibration[tool] = cal
                        logging.info(f"PuppyBootloader: Loaded side sensor cal T{tool}: "
                                   f"nins={cal[0]} ins={cal[1]}")
        except Exception as e:
            logging.info(f"PuppyBootloader: No side sensor cal loaded: {e}")

    def _save_side_filament_cal(self, tool, ref_nins, ref_ins):
        """Save side filament sensor calibration for one tool to variables.cfg."""
        save_vars = self.printer.lookup_object("save_variables", None)
        if save_vars is None:
            raise self.gcode.error("save_variables not configured")
        key = f"side_fs_cal_t{tool}"
        value = f"[{ref_nins}, {ref_ins}]"
        save_vars.cmd_SAVE_VARIABLE(self.gcode.create_gcode_command(
            "SAVE_VARIABLE", "SAVE_VARIABLE", {"VARIABLE": key, "VALUE": value}))
        self.side_fs_calibration[tool] = [ref_nins, ref_ins]
        logging.info(f"PuppyBootloader: Saved side sensor cal T{tool}: "
                   f"nins={ref_nins} ins={ref_ins}")

    def _evaluate_side_filament_state(self, filtered_value, tool):
        """Evaluate side filament sensor state (same Prusa algorithm, different span).

        Uses SIDE_FS_VALUE_SPAN (310) instead of FS_VALUE_SPAN (1000).
        """
        if filtered_value < self.FS_ADC_MIN or filtered_value > self.FS_ADC_MAX:
            return "not_connected"

        cal = self.side_fs_calibration.get(tool, None)
        if cal is None:
            return "not_calibrated"

        ref_nins = cal[0]
        ref_ins = cal[1] if len(cal) > 1 else None

        # Half-calibrated - use side sensor span
        if ref_ins is None:
            if (filtered_value < ref_nins - self.SIDE_FS_VALUE_SPAN or
                    filtered_value > ref_nins + self.SIDE_FS_VALUE_SPAN):
                return "has_filament"
            return "no_filament"

        # Full calibration - midpoint with 1/6 hysteresis
        midpoint = (ref_nins + ref_ins) // 2
        hysteresis = abs(ref_nins - ref_ins) // 6

        prev_state = self.side_fs_state.get(tool, "not_calibrated")
        if prev_state in ("has_filament", "no_filament"):
            if abs(filtered_value - midpoint) < hysteresis:
                return prev_state

        if (filtered_value > midpoint) == (ref_ins > midpoint):
            return "has_filament"
        return "no_filament"

    def _poll_side_sensors(self):
        """Query side filament sensors from MCU and update state.

        Called once per poll cycle (after all Dwarfs polled).
        Uses side sensors as PRIMARY runout detection (matching Prusa).
        """
        if self.side_fs_cmd is None:
            return

        try:
            result = self.side_fs_cmd.send()
            for tool in range(5):
                raw = result.get(f's{tool}', 0)
                self.side_fs_raw[tool] = raw

                new_state = self._evaluate_side_filament_state(raw, tool)
                old_state = self.side_fs_state.get(tool, "unknown")
                self.side_fs_state[tool] = new_state

                if old_state != new_state and old_state != "unknown":
                    logging.info(f"PuppyBootloader: Side sensor T{tool}: "
                               f"{old_state} -> {new_state} (raw={raw})")

                # PRIMARY runout detection using side sensors
                self._check_side_filament_runout(tool, new_state)

                # Autoload detection: side sensor insertion when idle
                self._check_side_autoload(tool, old_state, new_state)
        except Exception as e:
            logging.info(f"PuppyBootloader: Side sensor poll error: {e}")

    def _check_side_filament_runout(self, tool, new_state):
        """Check for filament runout using side sensors (PRIMARY detection).

        This is how Prusa stock firmware works - side sensors near the spools
        detect runout BEFORE the filament actually runs out at the tool.
        Debounces with FS_RUNOUT_DEBOUNCE consecutive readings.
        """
        if not self.filament_runout_enabled:
            return
        if tool != self.active_tool:
            return

        # Only check during actual printing
        try:
            print_stats = self.printer.lookup_object('print_stats', None)
            if print_stats is None:
                return
            stats = print_stats.get_status(self.reactor.monotonic())
            if stats.get('state', '') != 'printing':
                return
        except:
            return

        if new_state == "no_filament":
            count = self.side_fs_runout_count.get(tool, 0) + 1
            self.side_fs_runout_count[tool] = count
            if count >= self.FS_RUNOUT_DEBOUNCE:
                logging.warning(f"PuppyBootloader: SIDE SENSOR RUNOUT on T{tool}! "
                              f"({count} consecutive readings)")
                self.gcode.respond_info(
                    f"!! Filament runout detected on T{tool} (side sensor)! "
                    f"Pausing print.")
                self.side_fs_runout_count[tool] = 0
                try:
                    self.gcode.run_script_from_command("PAUSE")
                except Exception as e:
                    logging.error(f"PuppyBootloader: PAUSE failed after runout: {e}")
            else:
                logging.info(f"PuppyBootloader: Side sensor T{tool} no_filament "
                           f"({count}/{self.FS_RUNOUT_DEBOUNCE})")
        else:
            self.side_fs_runout_count[tool] = 0

    def _check_side_autoload(self, tool, old_state, new_state):
        """Check for filament insertion on side sensor to trigger autoload.

        Prusa autoload: when side sensor detects insertion while idle,
        automatically pick tool, slow load to gears, heat, fast load,
        purge, and retract. Only triggers when not printing.
        """
        if not self.autoload_enabled:
            return
        if self._autoload_in_progress:
            return

        # Only autoload when idle
        try:
            print_stats = self.printer.lookup_object('print_stats', None)
            if print_stats is not None:
                stats = print_stats.get_status(self.reactor.monotonic())
                state = stats.get('state', '')
                if state in ('printing', 'paused'):
                    return
        except:
            pass

        # Detect no_filament -> has_filament transition (immediate trigger)
        if new_state == "has_filament" and old_state == "no_filament":
            logging.info(f"PuppyBootloader: Side sensor T{tool} insertion "
                       f"detected - scheduling autoload")
            self.gcode.respond_info(
                f"Filament detected in T{tool} side sensor - autoloading...")
            self._autoload_in_progress = True
            # Schedule autoload outside poll loop so polls keep running
            self.reactor.register_callback(
                lambda e, t=tool: self._do_autoload_async(t))

    def _do_autoload_async(self, tool):
        """Run autoload outside the poll loop context."""
        try:
            self._do_autoload(tool)
        except Exception as e:
            logging.error(f"PuppyBootloader: Autoload T{tool} failed: {e}")
            self.gcode.respond_info(f"Autoload T{tool} failed: {e}")
        finally:
            self._autoload_in_progress = False

    def _read_tool_sensor(self, dwarf):
        """Direct MODBUS read of tool filament sensor ADC for a specific Dwarf.

        Used during autoload to check sensor between extrusion increments.
        Reads just register 0x8063 (filament_sensor) from the Dwarf.
        Returns raw ADC value, or None on failure.
        """
        addr = self.ADDR_MODBUS_OFFSET + dwarf
        reg_start = 0x8063  # filament_sensor register
        count = 1
        data = [
            (reg_start >> 8) & 0xFF, reg_start & 0xFF,
            (count >> 8) & 0xFF, count & 0xFF
        ]
        frame = self._build_modbus_frame(addr, self.MODBUS_READ_INPUT, data)
        try:
            response = self._send_frame(frame)
            status = response.get('status', -1)
            resp_data = response.get('data', b'')
            if status == 0 and resp_data and len(resp_data) >= 5:
                val = (resp_data[3] << 8) | resp_data[4]
                return val
            return None
        except Exception:
            return None

    def _do_autoload(self, tool):
        """Execute autoload sequence for a tool (Prusa M1701 equivalent).

        Matches Prusa's stock firmware autoload sequence:
        1. Pick tool if not active
        2. Assist insertion: chunked E moves @ 5.4mm/s (cold extrude OK),
           check tool sensor between chunks, 40 second timeout
        3. Slow load: 30mm @ 5.4mm/s (catch filament in gears)
        4. Heat to extrusion temperature, wait until reached
        5. Fast load: 50mm @ 18mm/s (fill hotend)
        6. Purge: 27mm @ 2.7mm/s (prime nozzle)
        7. Retract: 4mm @ 35mm/s (prevent ooze)
        8. Turn off heater
        """
        dwarf = tool + 1
        if dwarf not in self.booted_dwarfs:
            self.gcode.respond_info(
                f"T{tool}/Dwarf {dwarf} not booted - cannot autoload")
            return

        # Check filament sensor calibration
        cal = self.fs_calibration.get(tool, None)
        if cal is None or len(cal) < 2:
            self.gcode.respond_info(
                f"T{tool}: Tool sensor not calibrated - run "
                f"CALIBRATE_FILAMENT_SENSOR TOOL={tool} first")
            return

        # Pick tool if not active
        if self.active_tool != tool:
            self.gcode.respond_info(f"Picking T{tool}...")
            self.gcode.run_script(f"T{tool}")

        toolhead = self.printer.lookup_object('toolhead')

        # Disable min_extrude_temp for the ENTIRE autoload sequence.
        # Prusa sets allow_cold_extrude=true during assist insertion.
        extruder = self.printer.lookup_object('extruder', None)
        heater = extruder.get_heater() if extruder is not None else None
        saved_min_temp = None
        if heater is not None:
            saved_min_temp = heater.min_extrude_temp
            heater.min_extrude_temp = 0
            heater.can_extrude = True

        try:
            # --- Phase 1: Assist insertion (Prusa assist_insertion_process) ---
            # Chunked E moves @ 5.4mm/s. Check tool sensor state from
            # regular poll cycle (fs_state) between chunks - same as Prusa,
            # which uses cached sensor value from normal MODBUS polling.
            # Prusa: 0.1mm chunks, 40 second timeout.

            # Check current sensor state before starting
            cur_state = self.fs_state.get(tool, "unknown")
            if cur_state == "has_filament":
                self.gcode.respond_info(
                    f"T{tool}: Sensor already reads filament - "
                    f"check filament is removed")
                return

            logging.info(f"PuppyBootloader: Autoload T{tool} phase 1: "
                        f"assist @ 5.4mm/s, state={cur_state}")
            self.gcode.respond_info(
                f"T{tool}: Extruder turning - push filament in...")
            self.gcode.run_script("G92 E0")

            chunk_mm = 2.0
            speed = 5.4  # Prusa FILAMENT_CHANGE_SLOW_LOAD_FEEDRATE
            timeout = 40.0  # Prusa assist_insertion timeout
            start_time = self.reactor.monotonic()
            sensor_triggered = False

            while (self.reactor.monotonic() - start_time) < timeout:
                # Extrude one chunk
                pos = list(toolhead.get_position())
                pos[3] += chunk_mm
                toolhead.manual_move(pos, speed)
                toolhead.wait_moves()

                # Check tool sensor state from regular poll cycle
                if self.fs_state.get(tool) == "has_filament":
                    sensor_triggered = True
                    logging.info(f"PuppyBootloader: Autoload T{tool} "
                                f"tool sensor detected filament")
                    break

            if not sensor_triggered:
                self.gcode.respond_info(
                    f"T{tool}: Tool sensor not triggered after "
                    f"{timeout:.0f}s - aborting")
                return

            self.gcode.respond_info(f"T{tool}: Filament at tool sensor")

            # --- Phase 1b: Slow load 30mm @ 5.4mm/s (Prusa load_to_gears) ---
            logging.info(f"PuppyBootloader: Autoload T{tool} phase 1b: "
                        f"slow load 30mm @ 5.4mm/s")
            pos = list(toolhead.get_position())
            pos[3] += 30.0
            toolhead.manual_move(pos, 5.4)
            toolhead.wait_moves()

            # --- Phase 2: Heat to extrusion temperature ---
            temp = self.target_temps.get(dwarf, 0)
            if temp < 170:
                temp = 200  # Default load temp if nothing set
            self.gcode.respond_info(f"T{tool}: Heating to {temp}C...")
            logging.info(f"PuppyBootloader: Autoload T{tool} phase 2: "
                        f"heating to {temp}C via M109")
            self.gcode.run_script(f"M109 T{tool} S{temp}")
            logging.info(f"PuppyBootloader: Autoload T{tool} M109 returned")

            # --- Phase 3: Fast load 50mm @ 18mm/s (gear to nozzle) ---
            self.gcode.run_script("G92 E0")
            self.gcode.respond_info(f"T{tool}: Fast loading to nozzle...")
            logging.info(f"PuppyBootloader: Autoload T{tool} phase 3: "
                        f"fast load 50mm @ 18mm/s")
            pos = list(toolhead.get_position())
            pos[3] += 50.0
            toolhead.manual_move(pos, 18.0)
            toolhead.wait_moves()

            # --- Phase 4: Purge 27mm @ 2.7mm/s (prime nozzle) ---
            self.gcode.respond_info(f"T{tool}: Purging...")
            logging.info(f"PuppyBootloader: Autoload T{tool} phase 4: "
                        f"purge 27mm @ 2.7mm/s")
            pos = list(toolhead.get_position())
            pos[3] += 27.0
            toolhead.manual_move(pos, 2.7)
            toolhead.wait_moves()

            # --- Phase 5: Retract 4mm @ 35mm/s (prevent ooze) ---
            logging.info(f"PuppyBootloader: Autoload T{tool} phase 5: "
                        f"retract 4mm @ 35mm/s")
            pos = list(toolhead.get_position())
            pos[3] -= 4.0
            toolhead.manual_move(pos, 35.0)
            toolhead.wait_moves()

            # --- Phase 6: Turn off heater ---
            self.gcode.run_script(f"M104 T{tool} S0")
            self.gcode.respond_info(f"T{tool}: Autoload complete!")
            logging.info(f"PuppyBootloader: Autoload T{tool} complete")

        finally:
            # Restore min extrude temp
            if heater is not None and saved_min_temp is not None:
                heater.min_extrude_temp = saved_min_temp

    def cmd_CALIBRATE_SIDE_SENSOR(self, gcmd):
        """Start side filament sensor calibration (step 1: remove filament).

        CALIBRATE_SIDE_SENSOR [TOOL=n]
        Step 1: Remove filament from spool holder, run this to record empty.
        Step 2: Insert filament, run CALIBRATE_SIDE_SENSOR_STEP2.
        """
        if self.side_fs_cmd is None:
            raise gcmd.error("Side sensors not available (MCU firmware update needed)")

        tool = gcmd.get_int('TOOL', None)
        tools = [tool] if tool is not None else list(range(5))

        # Read current values
        try:
            result = self.side_fs_cmd.send()
        except Exception as e:
            raise gcmd.error(f"Failed to read side sensors: {e}")

        for t in tools:
            raw = result.get(f's{t}', 0)

            if raw < self.FS_ADC_MIN or raw > self.FS_ADC_MAX:
                gcmd.respond_info(f"T{t}: Side sensor not connected (raw={raw})")
                continue

            self.side_fs_calibration[t] = [raw]  # Partial cal (nins only)
            gcmd.respond_info(f"T{t}: Side sensor no-filament reference = {raw}")

        gcmd.respond_info(
            "Step 1 complete. Now INSERT filament into the side sensor(s) and run:\n"
            "  CALIBRATE_SIDE_SENSOR_STEP2 [TOOL=n]")

    def cmd_CALIBRATE_SIDE_SENSOR_STEP2(self, gcmd):
        """Complete side sensor calibration (step 2: with filament inserted).

        CALIBRATE_SIDE_SENSOR_STEP2 [TOOL=n]
        """
        if self.side_fs_cmd is None:
            raise gcmd.error("Side sensors not available (MCU firmware update needed)")

        tool = gcmd.get_int('TOOL', None)
        tools = [tool] if tool is not None else list(range(5))

        try:
            result = self.side_fs_cmd.send()
        except Exception as e:
            raise gcmd.error(f"Failed to read side sensors: {e}")

        for t in tools:
            cal = self.side_fs_calibration.get(t, None)
            if cal is None or len(cal) < 1:
                gcmd.respond_info(f"T{t}: Run CALIBRATE_SIDE_SENSOR first (step 1)")
                continue

            ref_nins = cal[0]
            raw = result.get(f's{t}', 0)

            if raw < self.FS_ADC_MIN or raw > self.FS_ADC_MAX:
                gcmd.respond_info(f"T{t}: Side sensor not connected (raw={raw})")
                continue

            diff = abs(raw - ref_nins)
            if diff < self.SIDE_FS_CAL_MIN_DIFF:
                gcmd.respond_info(
                    f"T{t}: FAILED - inserted value ({raw}) too close to "
                    f"empty value ({ref_nins}). Difference {diff} must be > "
                    f"{self.SIDE_FS_CAL_MIN_DIFF}. Check filament is in the sensor.")
                self.side_fs_calibration.pop(t, None)
                continue

            self._save_side_filament_cal(t, ref_nins, raw)
            midpoint = (ref_nins + raw) // 2
            hysteresis = abs(ref_nins - raw) // 6
            gcmd.respond_info(
                f"T{t}: Side sensor calibrated! nins={ref_nins} ins={raw} "
                f"midpoint={midpoint} hysteresis=±{hysteresis}")

    def cmd_SHOW_SIDE_SENSORS(self, gcmd):
        """Show side filament sensor states for all tools."""
        if self.side_fs_cmd is None:
            gcmd.respond_info("Side sensors not available (MCU firmware update needed)")
            return

        # Read fresh values
        try:
            result = self.side_fs_cmd.send()
        except Exception as e:
            gcmd.respond_info(f"Failed to read side sensors: {e}")
            return

        lines = [f"Side filament sensors (PRIMARY runout detection):"]
        lines.append(f"Runout detection: "
                    f"{'ENABLED' if self.filament_runout_enabled else 'DISABLED'}")
        for tool in range(5):
            raw = result.get(f's{tool}', 0)
            self.side_fs_raw[tool] = raw
            state = self._evaluate_side_filament_state(raw, tool)
            self.side_fs_state[tool] = state
            cal = self.side_fs_calibration.get(tool, None)
            if cal is not None and len(cal) == 2:
                cal_str = f"nins={cal[0]} ins={cal[1]}"
            elif cal is not None and len(cal) == 1:
                cal_str = f"nins={cal[0]} (step 2 needed)"
            else:
                cal_str = "NOT CALIBRATED"
            lines.append(f"  T{tool}: {state:16s} raw={raw:5d}  cal={cal_str}")
        gcmd.respond_info("\n".join(lines))

    def cmd_AUTOLOAD_FILAMENT(self, gcmd):
        """Manually trigger autoload for a tool.

        AUTOLOAD_FILAMENT [TOOL=n]
        """
        tool = gcmd.get_int('TOOL', self.active_tool)
        if tool is None or tool < 0:
            raise gcmd.error("No tool specified and no active tool")
        if self._autoload_in_progress:
            raise gcmd.error("Autoload already in progress")
        try:
            self._autoload_in_progress = True
            self._do_autoload(tool)
        except Exception as e:
            raise gcmd.error(f"Autoload failed: {e}")
        finally:
            self._autoload_in_progress = False

    def cmd_SET_AUTOLOAD(self, gcmd):
        """Enable or disable filament autoload.

        SET_AUTOLOAD ENABLE=1  - Enable autoload
        SET_AUTOLOAD ENABLE=0  - Disable autoload
        SET_AUTOLOAD           - Show current state
        """
        enable = gcmd.get_int('ENABLE', None)
        if enable is not None:
            self.autoload_enabled = bool(enable)
            gcmd.respond_info(
                f"Filament autoload: "
                f"{'ENABLED' if self.autoload_enabled else 'DISABLED'}")
        else:
            gcmd.respond_info(
                f"Filament autoload: "
                f"{'ENABLED' if self.autoload_enabled else 'DISABLED'}")

    def cmd_CALIBRATE_FILAMENT_SENSOR(self, gcmd):
        """Start filament sensor calibration.

        CALIBRATE_FILAMENT_SENSOR [TOOL=n]
        Step 1: Remove filament, run this command to record empty value.
        Then run CALIBRATE_FILAMENT_SENSOR_STEP2 TOOL=n with filament inserted.
        """
        tool = gcmd.get_int('TOOL', None)
        if tool is not None:
            tools = [tool]
        else:
            tools = list(range(len(self.booted_dwarfs)))

        for t in tools:
            dwarf = t + 1
            if dwarf not in self.booted_dwarfs:
                gcmd.respond_info(f"T{t}: Dwarf {dwarf} not booted, skipping")
                continue

            data = self.dwarf_data.get(dwarf, {})
            raw = data.get('filament_sensor', 0)

            if raw < self.FS_ADC_MIN or raw > self.FS_ADC_MAX:
                gcmd.respond_info(f"T{t}: Sensor not connected (raw={raw})")
                continue

            # Store no-filament reference (step 1)
            self.fs_calibration[t] = [raw]  # Partial cal (nins only)
            gcmd.respond_info(f"T{t}: No-filament reference = {raw}")

        gcmd.respond_info(
            "Step 1 complete. Now INSERT filament into the tool(s) and run:\n"
            "  CALIBRATE_FILAMENT_SENSOR_STEP2 [TOOL=n]")

    def cmd_CALIBRATE_FILAMENT_SENSOR_STEP2(self, gcmd):
        """Complete filament sensor calibration (with filament inserted).

        CALIBRATE_FILAMENT_SENSOR_STEP2 [TOOL=n]
        Records inserted value and validates against step 1.
        """
        tool = gcmd.get_int('TOOL', None)
        if tool is not None:
            tools = [tool]
        else:
            tools = list(range(len(self.booted_dwarfs)))

        for t in tools:
            dwarf = t + 1
            if dwarf not in self.booted_dwarfs:
                gcmd.respond_info(f"T{t}: Dwarf {dwarf} not booted, skipping")
                continue

            cal = self.fs_calibration.get(t, None)
            if cal is None or len(cal) < 1:
                gcmd.respond_info(f"T{t}: Run CALIBRATE_FILAMENT_SENSOR first (step 1)")
                continue

            ref_nins = cal[0]
            data = self.dwarf_data.get(dwarf, {})
            raw = data.get('filament_sensor', 0)

            if raw < self.FS_ADC_MIN or raw > self.FS_ADC_MAX:
                gcmd.respond_info(f"T{t}: Sensor not connected (raw={raw})")
                continue

            # Validate: inserted value must differ from empty by minimum threshold
            # Prusa uses span*1.2=1200 but some tool sensors swing less (T4: ~674 units)
            # 300 ADC units is enough for reliable midpoint+hysteresis detection
            diff = abs(raw - ref_nins)
            if diff < self.FS_CAL_MIN_DIFF:
                gcmd.respond_info(
                    f"T{t}: FAILED - inserted value ({raw}) too close to "
                    f"empty value ({ref_nins}). Difference {diff} must be > "
                    f"{self.FS_CAL_MIN_DIFF}. Check that filament is fully inserted.")
                # Invalidate calibration
                self.fs_calibration.pop(t, None)
                continue

            # Save calibration
            self._save_filament_cal(t, ref_nins, raw)
            midpoint = (ref_nins + raw) // 2
            hysteresis = abs(ref_nins - raw) // 6
            gcmd.respond_info(
                f"T{t}: Calibrated! nins={ref_nins} ins={raw} "
                f"midpoint={midpoint} hysteresis=±{hysteresis}")

    def cmd_SHOW_FILAMENT_SENSORS(self, gcmd):
        """Show all filament sensor states (tool + side sensors)."""
        if not self.booted_dwarfs:
            gcmd.respond_info("No Dwarfs booted")
            return

        lines = [f"Filament runout detection: "
                 f"{'ENABLED' if self.filament_runout_enabled else 'DISABLED'}"]

        # Tool sensors (MODBUS 0x8063, load/unload confirmation)
        lines.append("Tool sensors (load/unload confirmation):")
        for dwarf in sorted(self.booted_dwarfs):
            tool = dwarf - 1
            data = self.dwarf_data.get(dwarf, {})
            raw = data.get('filament_sensor', 0)
            state = self.fs_state.get(tool, "unknown")
            cal = self.fs_calibration.get(tool, None)
            if cal is not None and len(cal) == 2:
                cal_str = f"nins={cal[0]} ins={cal[1]}"
            elif cal is not None and len(cal) == 1:
                cal_str = f"nins={cal[0]} (step 2 needed)"
            else:
                cal_str = "NOT CALIBRATED"
            lines.append(f"  T{tool}: {state:16s} raw={raw:5d}  cal={cal_str}")

        # Side sensors (ADC3 + mux, PRIMARY runout)
        if self.side_fs_cmd is not None:
            lines.append("Side sensors (PRIMARY runout detection):")
            try:
                result = self.side_fs_cmd.send()
                for tool in range(5):
                    raw = result.get(f's{tool}', 0)
                    state = self._evaluate_side_filament_state(raw, tool)
                    cal = self.side_fs_calibration.get(tool, None)
                    if cal is not None and len(cal) == 2:
                        cal_str = f"nins={cal[0]} ins={cal[1]}"
                    elif cal is not None and len(cal) == 1:
                        cal_str = f"nins={cal[0]} (step 2 needed)"
                    else:
                        cal_str = "NOT CALIBRATED"
                    lines.append(f"  T{tool}: {state:16s} raw={raw:5d}  cal={cal_str}")
            except Exception as e:
                lines.append(f"  Error reading side sensors: {e}")
        else:
            lines.append("Side sensors: NOT AVAILABLE (MCU firmware update needed)")

        gcmd.respond_info("\n".join(lines))

    def cmd_M591(self, gcmd):
        """M591 - Filament runout detection enable/disable.

        M591 S0 - Disable runout detection
        M591 S1 - Enable runout detection
        M591    - Show current state
        """
        s = gcmd.get_int('S', None)
        if s == 0:
            self.filament_runout_enabled = False
            self._save_fs_runout_enabled()
            gcmd.respond_info("Filament runout detection DISABLED")
        elif s == 1:
            self.filament_runout_enabled = True
            self._save_fs_runout_enabled()
            gcmd.respond_info("Filament runout detection ENABLED")
        else:
            state = "ENABLED" if self.filament_runout_enabled else "DISABLED"
            gcmd.respond_info(f"Filament runout detection: {state}")

    def cmd_UNLOAD_FILAMENT(self, gcmd):
        """Unload filament from a tool (Prusa M702 equivalent).

        UNLOAD_FILAMENT [TOOL=n] [TEMP=t]
        Heats nozzle, rams, retracts filament.
        Prusa speeds: unload 27mm/s, ram push then retract.
        """
        tool = gcmd.get_int('TOOL', self.active_tool)
        temp = gcmd.get_float('TEMP', 0)
        dwarf = tool + 1

        if dwarf not in self.booted_dwarfs:
            raise gcmd.error(f"T{tool}/Dwarf {dwarf} not booted")

        # Pick tool if not active
        if self.active_tool != tool:
            gcmd.respond_info(f"Picking T{tool}...")
            self.gcode.run_script_from_command(f"T{tool}")

        # Determine temperature
        if temp <= 0:
            temp = self.target_temps.get(dwarf, 0)
        if temp < 170:
            temp = 200  # Minimum safe unload temp
            gcmd.respond_info(f"Using default unload temp: {temp}C")

        # Heat and wait
        gcmd.respond_info(f"Heating T{tool} to {temp}C for unload...")
        self.gcode.run_script_from_command(f"M109 T{tool} S{temp}")

        # Disable extruder stepper current reduction (Prusa: disable_e_steppers for noise)
        # Ram forward to clear nozzle tip (simplified Prusa ramming)
        gcmd.respond_info("Ramming...")
        # Short push forward: 2mm at 10mm/s (clear nozzle tip)
        self.gcode.run_script_from_command("G92 E0")
        self.gcode.run_script_from_command("G1 E2 F600")   # 10mm/s push
        self.gcode.run_script_from_command("G1 E0 F600")   # retract back
        self.gcode.run_script_from_command("G4 P500")       # 500ms settle

        # Full retract: 27mm/s = 1620mm/min, ~80mm retract
        gcmd.respond_info("Retracting filament...")
        self.gcode.run_script_from_command("G1 E-80 F1620")
        self.gcode.run_script_from_command("G92 E0")

        # Check sensor
        data = self.dwarf_data.get(dwarf, {})
        raw = data.get('filament_sensor', 0)
        state = self._evaluate_filament_state(raw, tool)
        if state == "no_filament":
            gcmd.respond_info(f"T{tool}: Filament unloaded successfully")
        elif state == "not_calibrated":
            gcmd.respond_info(f"T{tool}: Unload complete (sensor not calibrated)")
        else:
            gcmd.respond_info(
                f"T{tool}: WARNING - sensor still reads '{state}' (raw={raw}). "
                f"Filament may not be fully unloaded. Pull manually if needed.")

    def cmd_LOAD_FILAMENT(self, gcmd):
        """Load filament into a tool (Prusa M701 equivalent).

        LOAD_FILAMENT [TOOL=n] [TEMP=t]
        Waits for sensor, slow loads to gears, heats, fast loads, purges.
        Prusa speeds: slow 5.4mm/s, fast 18mm/s, purge 2.7mm/s.
        """
        tool = gcmd.get_int('TOOL', self.active_tool)
        temp = gcmd.get_float('TEMP', 0)
        dwarf = tool + 1

        if dwarf not in self.booted_dwarfs:
            raise gcmd.error(f"T{tool}/Dwarf {dwarf} not booted")

        # Pick tool if not active
        if self.active_tool != tool:
            gcmd.respond_info(f"Picking T{tool}...")
            self.gcode.run_script_from_command(f"T{tool}")

        gcmd.respond_info(f"Insert filament into T{tool} and push until it grabs...")

        # Wait for tool sensor to detect filament (timeout 120s)
        cal = self.fs_calibration.get(tool, None)
        if cal is not None and len(cal) == 2:
            start_time = self.reactor.monotonic()
            timeout = 120.0
            detected = False
            while (self.reactor.monotonic() - start_time) < timeout:
                data = self.dwarf_data.get(dwarf, {})
                raw = data.get('filament_sensor', 0)
                state = self._evaluate_filament_state(raw, tool)
                if state == "has_filament":
                    detected = True
                    break
                self.reactor.pause(self.reactor.monotonic() + 1.0)
            if not detected:
                gcmd.respond_info(
                    f"T{tool}: Timeout waiting for filament sensor. "
                    f"Continuing with load anyway...")
        else:
            gcmd.respond_info("Sensor not calibrated - waiting 5s for manual insertion...")
            self.reactor.pause(self.reactor.monotonic() + 5.0)

        # Determine temperature
        if temp <= 0:
            temp = self.target_temps.get(dwarf, 0)
        if temp < 170:
            temp = 200  # Minimum safe load temp
            gcmd.respond_info(f"Using default load temp: {temp}C")

        # Slow load to gears: 5.4mm/s = 324mm/min, 30mm
        # Prusa allows cold extrusion for this step
        gcmd.respond_info("Slow loading to gears...")
        self.gcode.run_script_from_command("G92 E0")
        self.gcode.run_script_from_command("M302 P1")       # Allow cold extrusion
        self.gcode.run_script_from_command("G1 E30 F324")    # 5.4mm/s, 30mm
        self.gcode.run_script_from_command("M302 S170")      # Restore cold extrusion limit

        # Heat and wait
        gcmd.respond_info(f"Heating T{tool} to {temp}C...")
        self.gcode.run_script_from_command(f"M109 T{tool} S{temp}")

        # Fast load: 18mm/s = 1080mm/min, 40mm (fill hotend)
        gcmd.respond_info("Fast loading...")
        self.gcode.run_script_from_command("G1 E40 F1080")

        # Purge: 2.7mm/s = 162mm/min, 27mm
        gcmd.respond_info("Purging...")
        self.gcode.run_script_from_command("G1 E27 F162")

        # Retract to prevent ooze: 35mm/s = 2100mm/min, 4mm
        self.gcode.run_script_from_command("G1 E-4 F2100")
        self.gcode.run_script_from_command("G92 E0")

        gcmd.respond_info(f"T{tool}: Filament loaded and purged")

    def cmd_M600(self, gcmd):
        """M600 - Filament change (Prusa compatibility).

        Pauses print, unloads old filament, waits for new filament,
        loads and purges, then user resumes.
        """
        tool = gcmd.get_int('T', self.active_tool)
        logging.info(f"PuppyBootloader: M600 filament change T{tool}")
        gcmd.respond_info(f"Filament change on T{tool} - pausing print...")

        # Pause print (PAUSE macro handles parking, Z lift, etc.)
        self.gcode.run_script_from_command("PAUSE")

        # Unload old filament
        gcmd.respond_info("Unloading old filament...")
        self.gcode.run_script_from_command(f"UNLOAD_FILAMENT TOOL={tool}")

        # Wait for user to insert new filament and run LOAD + RESUME
        gcmd.respond_info(
            "Old filament unloaded. Insert new filament and run:\n"
            "  LOAD_FILAMENT TOOL={tool}\n"
            "  RESUME")

    def _handle_connect(self):
        try:
            self.send_raw_cmd = self.mcu.lookup_query_command(
                "modbus_send_raw data=%*s",
                "modbus_raw_response status=%c data=%*s",
                is_async=True)
            logging.info("PuppyBootloader: Found modbus_send_raw command")
        except Exception as e:
            logging.warning(f"PuppyBootloader: modbus_send_raw not available: {e}")
            self.send_raw_cmd = None
            return

        try:
            self.pca9557_cmd = self.mcu.lookup_query_command(
                "pca9557_output value=%c",
                "pca9557_output_response status=%c value=%c",
                is_async=True)
            logging.info("PuppyBootloader: Found pca9557_output command")
        except Exception as e:
            logging.warning(f"PuppyBootloader: pca9557_output not available: {e}")
            self.pca9557_cmd = None
            return

        try:
            self.loadcell_cmd = self.mcu.lookup_query_command(
                "modbus_read_loadcell addr=%c",
                "modbus_loadcell_response status=%c count=%c raw=%u rlen=%u",
                is_async=True)
            logging.info("PuppyBootloader: Found modbus_read_loadcell command")
        except Exception as e:
            logging.warning(f"PuppyBootloader: modbus_read_loadcell not available: {e}")
            self.loadcell_cmd = None

        # MCU-level probe with emergency halt on trigger
        # Try new firmware format first (with xy mode), fall back to old format
        self.probe_has_xy = False
        try:
            self.probe_start_cmd = self.mcu.lookup_command(
                "loadcell_probe_start addr=%c threshold=%i halt=%c xy=%c")
            self.probe_has_xy = True
            logging.info("PuppyBootloader: Found loadcell_probe_start command (with xy mode)")
        except Exception as e:
            # Fall back to old format without xy parameter
            try:
                self.probe_start_cmd = self.mcu.lookup_command(
                    "loadcell_probe_start addr=%c threshold=%i halt=%c")
                logging.info("PuppyBootloader: Found loadcell_probe_start command (no xy mode)")
            except Exception as e2:
                logging.warning(f"PuppyBootloader: loadcell_probe_start not available: {e2}")
                self.probe_start_cmd = None

        # Try new firmware format first (with max and xy), fall back to old format
        try:
            self.probe_stop_cmd = self.mcu.lookup_query_command(
                "loadcell_probe_stop",
                "loadcell_probe_result triggered=%c load=%i time=%u count=%u min=%i max=%i errors=%u xy=%c",
                is_async=True)
            logging.info("PuppyBootloader: Found loadcell_probe_stop command (with xy/max)")
        except Exception as e:
            try:
                self.probe_stop_cmd = self.mcu.lookup_query_command(
                    "loadcell_probe_stop",
                    "loadcell_probe_result triggered=%c load=%i time=%u count=%u min=%i errors=%u",
                    is_async=True)
                logging.info("PuppyBootloader: Found loadcell_probe_stop command (old format)")
            except Exception as e2:
                logging.warning(f"PuppyBootloader: loadcell_probe_stop not available: {e2}")
                self.probe_stop_cmd = None

        try:
            self.probe_query_cmd = self.mcu.lookup_query_command(
                "loadcell_probe_query",
                "loadcell_probe_state triggered=%c load=%i count=%u monitoring=%c",
                is_async=True)
            logging.info("PuppyBootloader: Found loadcell_probe_query command")
        except Exception as e:
            logging.warning(f"PuppyBootloader: loadcell_probe_query not available: {e}")
            self.probe_query_cmd = None

        try:
            self.tare_mcu_cmd = self.mcu.lookup_query_command(
                "loadcell_tare addr=%c samples=%c",
                "loadcell_tare_result offset=%i samples=%c",
                is_async=True)
            logging.info("PuppyBootloader: Found loadcell_tare command")
        except Exception as e:
            logging.warning(f"PuppyBootloader: loadcell_tare not available: {e}")
            self.tare_mcu_cmd = None

        # Loadcell endstop homing command (trsync-based, like real endstop)
        try:
            self.loadcell_home_cmd = self.mcu.lookup_command(
                "loadcell_endstop_home addr=%c threshold=%i xy=%c trsync_oid=%c trigger_reason=%c")
            logging.info("PuppyBootloader: Found loadcell_endstop_home command (trsync)")
            # Set up the command on both LoadcellEndstops (created during config phase)
            self.loadcell_endstop.setup_commands(self.loadcell_home_cmd)
            self.loadcell_endstop_xy.setup_commands(self.loadcell_home_cmd)
            # Register steppers with appropriate endstops (Z -> loadcell_endstop, XY -> loadcell_endstop_xy)
            self._register_z_steppers()
            logging.info("PuppyBootloader: LoadcellEndstop ready with trsync support")
        except Exception as e:
            logging.warning(f"PuppyBootloader: loadcell_endstop_home not available: {e}")
            self.loadcell_home_cmd = None

        # Load filament sensor calibration (save_variables is ready at connect time)
        self._load_filament_cal()
        self._load_side_filament_cal()

        # Side filament sensors (ADC3 + mux on XLBuddy sandwich board)
        try:
            self.side_fs_cmd = self.mcu.lookup_query_command(
                "query_side_filament_sensors",
                "side_filament_sensors_result s0=%u s1=%u s2=%u s3=%u s4=%u",
                is_async=True)
            logging.info("PuppyBootloader: Found query_side_filament_sensors command")
        except Exception as e:
            logging.info(f"PuppyBootloader: Side filament sensors not available "
                        f"(firmware update needed): {e}")
            self.side_fs_cmd = None

        # Run automatic boot sequence
        self._auto_boot_dwarfs()

    def _register_z_steppers(self):
        """Register Z steppers with the loadcell endstop for trsync."""
        try:
            toolhead = self.printer.lookup_object('toolhead')
            kin = toolhead.get_kinematics()
            # Register Z steppers with loadcell_endstop (for Z probing)
            z_count = 0
            for stepper in kin.get_steppers():
                if stepper.is_active_axis('z'):
                    self.loadcell_endstop.add_stepper(stepper)
                    logging.info(f"PuppyBootloader: {stepper.get_name()} -> Z endstop")
                    z_count += 1
            # Register X/Y steppers with loadcell_endstop_xy (for XY probing)
            xy_count = 0
            for stepper in kin.get_steppers():
                for axis in ['x', 'y']:
                    if stepper.is_active_axis(axis):
                        self.loadcell_endstop_xy.add_stepper(stepper)
                        logging.info(f"PuppyBootloader: {stepper.get_name()} -> XY endstop")
                        xy_count += 1
                        break
            logging.info(f"PuppyBootloader: Registered {z_count} Z steppers, {xy_count} XY steppers")
        except Exception as e:
            logging.warning(f"PuppyBootloader: Failed to register steppers: {e}")

    def _pca9557_set(self, value):
        """Set PCA9557 output register"""
        if self.pca9557_cmd:
            self.pca9557_cmd.send([value])

    def _boot_modular_bed(self):
        """Boot modular bed at address 0x1A (Prusa standard)"""
        logging.info("PuppyBootloader: Booting modular bed at 0x1A")

        # Reset only the bed (bit 7)
        self._pca9557_set(self.ALL_DWARF_BITS | self.MODULAR_BED_BIT)
        time.sleep(0.05)

        # Release bed only (keep Dwarfs in reset)
        self._pca9557_set(self.ALL_DWARF_BITS)
        time.sleep(0.5)

        # Assign address 0x0A to bed (bootloader temp address)
        bed_boot_addr = self.ADDR_FIRST_BOOT  # 0x0A
        data = [bed_boot_addr, 0x00]
        frame = self._build_bootloader_frame(self.ADDR_DEFAULT, self.CMD_SET_ADDRESS, data)
        logging.info(f"PuppyBootloader: Bed SET_ADDRESS frame: {frame.hex()}")
        try:
            self._send_frame(frame)
        except:
            pass
        time.sleep(0.05)

        # Discover at assigned address
        frame = self._build_bootloader_frame(bed_boot_addr, self.CMD_GET_PROTOCOL_VERSION)
        try:
            response = self._send_frame(frame)
            status = response.get('status', -1)
            resp_data = response.get('data', b'')
            logging.info(f"PuppyBootloader: Bed discover: status={status}, data={resp_data.hex() if resp_data else 'none'}")
            if status != 0:
                logging.warning("PuppyBootloader: Bed not responding, skipping")
                self.modular_bed_booted = False
                return False
        except Exception as e:
            logging.warning(f"PuppyBootloader: Bed discover failed: {e}")
            self.modular_bed_booted = False
            return False

        # Compute fingerprint with random salt
        salt = random.randint(0, 0xFFFFFFFF)
        salt_bytes = [
            (salt >> 24) & 0xFF,
            (salt >> 16) & 0xFF,
            (salt >> 8) & 0xFF,
            salt & 0xFF
        ]
        frame = self._build_bootloader_frame(bed_boot_addr, self.CMD_COMPUTE_FINGERPRINT, salt_bytes)
        logging.info(f"PuppyBootloader: Bed COMPUTE_FP frame: {frame.hex()}, salt=0x{salt:08X}")
        try:
            response = self._send_frame(frame)
            status = response.get('status', -1)
            resp_data = response.get('data', b'')
            logging.info(f"PuppyBootloader: Bed COMPUTE_FP response: status={status}, "
                        f"data={resp_data.hex() if resp_data else 'none'}")
        except Exception as e:
            logging.info(f"PuppyBootloader: Bed COMPUTE_FP exception: {e}")

        # Wait for SHA256 computation (takes 330-600ms)
        time.sleep(0.8)

        # Poll GET_FINGERPRINT (same as Dwarf handling)
        fingerprint = None
        for attempt in range(10):
            time.sleep(0.2)
            fp_frame = self._build_bootloader_frame(bed_boot_addr, self.CMD_GET_FINGERPRINT, [0, 32])
            try:
                response = self._send_frame(fp_frame)
                status = response.get('status', -1)
                resp_data = response.get('data', b'')
                logging.info(f"PuppyBootloader: Bed GET_FP attempt {attempt+1}: "
                            f"status={status}, len={len(resp_data) if resp_data else 0}, "
                            f"data={resp_data[:16].hex() if resp_data else 'none'}")
                if status == 0 and resp_data and len(resp_data) >= 37:
                    # Response: [addr][status][len=32][32 bytes fingerprint][crc16]
                    fingerprint = list(resp_data[3:35])
                    logging.info(f"PuppyBootloader: Bed fingerprint ready (attempt {attempt+1})")
                    break
            except Exception as e:
                logging.info(f"PuppyBootloader: Bed GET_FP attempt {attempt+1} exception: {e}")

        if fingerprint is None:
            logging.warning("PuppyBootloader: Bed fingerprint failed after 10 attempts")
            self.modular_bed_booted = False
            return False

        # Start application
        app_data = salt_bytes + fingerprint
        frame = self._build_bootloader_frame(bed_boot_addr, self.CMD_START_APPLICATION, app_data)
        try:
            response = self._send_frame(frame)
            if response.get('status', -1) == 0:
                logging.info("PuppyBootloader: Modular bed started at 0x1A!")
                self.modular_bed_booted = True
                self.bed_faults_cleared = False  # Will clear faults before first temp write
                # Bed firmware has default max_allowed_current of 1.5A
                # Give bed time to initialize before first commands
                self.reactor.pause(self.reactor.monotonic() + 0.5)
                # Register bedlet temperature sensors (Prusa-style: hidden individuals + visible avg)
                self._register_bedlet_sensors()
                return True
        except Exception as e:
            logging.warning(f"PuppyBootloader: Bed start failed: {e}")

        self.modular_bed_booted = False
        return False

    def _auto_boot_dwarfs(self):
        """Full Prusa-style boot sequence driven from Python"""
        logging.info("PuppyBootloader: Starting boot sequence (Prusa standard addressing)")

        # Initialize bed state
        self.modular_bed_booted = False

        # Step 1: Reset all Dwarfs + modular bed
        reset_all = self.ALL_DWARF_BITS | self.MODULAR_BED_BIT
        logging.info(f"PuppyBootloader: Resetting all (PCA9557=0x{reset_all:02X})")
        self._pca9557_set(reset_all)
        time.sleep(0.05)  # Hold reset for 50ms

        # Step 2: Boot modular bed first at 0x1A
        self._boot_modular_bed()
        time.sleep(0.1)

        # Step 3: Reset and release Dwarfs (keep bed running now)
        logging.info(f"PuppyBootloader: Resetting Dwarfs only (PCA9557=0x{self.ALL_DWARF_BITS:02X})")
        self._pca9557_set(self.ALL_DWARF_BITS)
        time.sleep(0.05)
        logging.info("PuppyBootloader: Releasing Dwarfs (PCA9557=0x00)")
        self._pca9557_set(0x00)  # All released
        time.sleep(0.5)  # Give bootloaders plenty of time to start

        # Step 2b: Verify communication - try to discover at address 0x00
        frame = self._build_bootloader_frame(self.ADDR_DEFAULT, self.CMD_GET_PROTOCOL_VERSION)
        try:
            response = self._send_frame(frame)
            status = response.get('status', -1)
            resp_data = response.get('data', b'')
            logging.info(f"PuppyBootloader: Discovery at addr=0x00: status={status}, "
                         f"data={resp_data.hex() if resp_data else 'none'}")
        except Exception as e:
            logging.warning(f"PuppyBootloader: Discovery at addr=0x00 failed: {e}")

        booted = []
        for dwarf in range(1, self.NUM_DWARFS + 1):
            # Dwarfs get bootloader addresses 0x0B-0x0F (bed uses 0x0A)
            assigned_addr = self.ADDR_FIRST_BOOT + dwarf  # 0x0B, 0x0C, ...
            dwarf_bit = (1 << dwarf)

            logging.info(f"PuppyBootloader: Booting Dwarf {dwarf} (boot addr=0x{assigned_addr:02X}, final=0x{self.ADDR_MODBUS_OFFSET + dwarf:02X})")

            # 3a: Assign address (all Dwarfs at 0x00 will accept this)
            # Note: Dwarf bootloader expects 2 bytes: [new_addr, 0x00]
            data = [assigned_addr, 0x00]
            frame = self._build_bootloader_frame(self.ADDR_DEFAULT, self.CMD_SET_ADDRESS, data)
            logging.info(f"PuppyBootloader: Dwarf {dwarf} SET_ADDRESS frame: {frame.hex()}")
            try:
                self._send_frame(frame)
            except:
                pass  # No reply expected
            time.sleep(0.05)  # 50ms for command processing

            # 3b: Reset only Dwarfs that haven't been addressed yet
            # (those still at address 0x00 - don't touch already-booted ones!)
            # Bed is already running, don't reset it
            remaining_mask = 0
            for d in range(dwarf + 1, self.NUM_DWARFS + 1):
                remaining_mask |= (1 << d)
            logging.info(f"PuppyBootloader: Dwarf {dwarf} resetting remaining (PCA9557=0x{remaining_mask:02X})")
            self._pca9557_set(remaining_mask)
            time.sleep(0.02)

            # 3c: Discover at assigned address
            frame = self._build_bootloader_frame(assigned_addr, self.CMD_GET_PROTOCOL_VERSION)
            logging.info(f"PuppyBootloader: Dwarf {dwarf} discover frame: {frame.hex()}")
            try:
                response = self._send_frame(frame)
                status = response.get('status', -1)
                resp_data = response.get('data', b'')
                logging.info(f"PuppyBootloader: Dwarf {dwarf} discover result: "
                             f"status={status}, data={resp_data.hex() if resp_data else 'none'}")
                if status != 0:
                    # Also try at address 0x00 in case SET_ADDRESS didn't work
                    frame0 = self._build_bootloader_frame(self.ADDR_DEFAULT, self.CMD_GET_PROTOCOL_VERSION)
                    try:
                        resp0 = self._send_frame(frame0)
                        s0 = resp0.get('status', -1)
                        d0 = resp0.get('data', b'')
                        logging.info(f"PuppyBootloader: Dwarf {dwarf} fallback at 0x00: "
                                     f"status={s0}, data={d0.hex() if d0 else 'none'}")
                    except:
                        pass
                    self._pca9557_set(0)  # Release all (bed already running)
                    time.sleep(0.02)
                    continue
            except Exception as e:
                logging.info(f"PuppyBootloader: Dwarf {dwarf} discover failed: {e}")
                self._pca9557_set(0)  # Release all (bed already running)
                time.sleep(0.02)
                continue

            logging.info(f"PuppyBootloader: Dwarf {dwarf} discovered!")

            # 3d: Compute fingerprint with random salt
            salt = random.randint(0, 0xFFFFFFFF)
            salt_bytes = [
                (salt >> 24) & 0xFF,
                (salt >> 16) & 0xFF,
                (salt >> 8) & 0xFF,
                salt & 0xFF
            ]
            frame = self._build_bootloader_frame(assigned_addr, self.CMD_COMPUTE_FINGERPRINT, salt_bytes)
            logging.info(f"PuppyBootloader: Dwarf {dwarf} COMPUTE_FP frame: {frame.hex()}, "
                         f"salt=0x{salt:08X}")
            try:
                response = self._send_frame(frame)
                status = response.get('status', -1)
                resp_data = response.get('data', b'')
                logging.info(f"PuppyBootloader: Dwarf {dwarf} COMPUTE_FP response: "
                             f"status={status}, data={resp_data.hex() if resp_data else 'none'}")
            except Exception as e:
                logging.info(f"PuppyBootloader: Dwarf {dwarf} COMPUTE_FP exception: {e}")

            # Wait for SHA256 computation (takes 330-600ms on Dwarf MCU)
            time.sleep(0.8)

            # 3e: Poll GET_FINGERPRINT
            fingerprint = None
            for attempt in range(10):  # Up to 10 attempts
                time.sleep(0.2)
                fp_frame = self._build_bootloader_frame(
                    assigned_addr, self.CMD_GET_FINGERPRINT, [0, 32])
                try:
                    response = self._send_frame(fp_frame)
                    status = response.get('status', -1)
                    resp_data = response.get('data', b'')
                    logging.info(f"PuppyBootloader: Dwarf {dwarf} GET_FP attempt {attempt+1}: "
                                 f"status={status}, len={len(resp_data) if resp_data else 0}, "
                                 f"data={resp_data[:16].hex() if resp_data else 'none'}")
                    if status == 0 and resp_data and len(resp_data) >= 37:
                        # Response: [addr][status][len=32][32 bytes fingerprint][crc16]
                        fingerprint = list(resp_data[3:35])
                        logging.info(f"PuppyBootloader: Dwarf {dwarf} fingerprint ready "
                                     f"(attempt {attempt+1})")
                        break
                except Exception as e:
                    logging.info(f"PuppyBootloader: Dwarf {dwarf} GET_FP attempt {attempt+1} "
                                 f"exception: {e}")

            if fingerprint is None:
                logging.warning(f"PuppyBootloader: Dwarf {dwarf} fingerprint failed after 10 attempts")
                self._pca9557_set(0)  # Release all (bed already running)
                time.sleep(0.02)
                continue

            # 3f: Start application with salt + fingerprint (retry on CRC error)
            app_data = salt_bytes + fingerprint
            frame = self._build_bootloader_frame(assigned_addr, self.CMD_START_APPLICATION, app_data)
            started = False
            for start_attempt in range(3):
                try:
                    response = self._send_frame(frame)
                    status = response.get('status', -1)
                    if status == 0:
                        logging.info(f"PuppyBootloader: Dwarf {dwarf} started! (GREEN)")
                        booted.append(dwarf)
                        started = True
                        break
                    else:
                        logging.info(f"PuppyBootloader: Dwarf {dwarf} START_APP attempt "
                                     f"{start_attempt+1} status={status}")
                        time.sleep(0.1)
                except Exception as e:
                    logging.info(f"PuppyBootloader: Dwarf {dwarf} START_APP attempt "
                                 f"{start_attempt+1} exception: {e}")
                    time.sleep(0.1)
            if not started:
                logging.warning(f"PuppyBootloader: Dwarf {dwarf} failed to start after 3 attempts")

            # 3g: Release all for next iteration (bed already running)
            self._pca9557_set(0)
            time.sleep(0.02)

        logging.info(f"PuppyBootloader: Boot complete. Booted Dwarfs: {booted}")
        self.booted_dwarfs = booted

        # Configure cheese LEDs on all Dwarfs
        self._update_all_leds()

        # Query MCU status for debug counters
        try:
            status_cmd = self.mcu.lookup_query_command(
                "modbus_status",
                "modbus_status_response tx=%u rx=%u timeout=%u crc_err=%u "
                "init=%c booted=%c dwarfs=%c rx_bytes=%u rx_errs=%u "
                "s0=%u s1=%u maxpos=%u",
                is_async=True)
            resp = status_cmd.send()
            logging.info(f"PuppyBootloader: MCU status: tx={resp.get('tx',0)} "
                         f"rx={resp.get('rx',0)} timeout={resp.get('timeout',0)} "
                         f"rx_bytes={resp.get('rx_bytes',0)} rx_errs={resp.get('rx_errs',0)} "
                         f"state0_bytes={resp.get('s0',0)} state1_bytes={resp.get('s1',0)} "
                         f"max_rx_pos={resp.get('maxpos',0)}")
        except Exception as e:
            logging.info(f"PuppyBootloader: Status query failed: {e}")

        # Start periodic polling to keep Dwarf watchdogs alive
        if booted:
            self._start_polling()

        # Auto-detect tool state: check which tool (if any) is picked
        self.tool_picked = False
        self.active_tool = -1
        for dwarf in sorted(self.booted_dwarfs):
            try:
                self.reactor.pause(self.reactor.monotonic() + 0.1)
                state = self._get_dock_state(dwarf)
                if state:
                    picked, parked = state
                    if picked:
                        tool = dwarf - 1
                        # SAFETY: Set tool_picked=True based on Hall sensor
                        # This prevents crashing into dock if tool is physically attached
                        self.active_tool = tool
                        self.tool_picked = True
                        logging.info(f"PuppyBootloader: Auto-detected T{tool} picked on startup (Hall sensor)")

                        # Try to enable TMC (may fail if FC05 not working)
                        tmc_ok = self._write_coil(dwarf, 0x4000, True)
                        if tmc_ok:
                            self.tmc_enabled[dwarf] = True
                            self._write_coil(dwarf, 0x4001, True)  # select it
                            logging.info(f"PuppyBootloader: T{tool} TMC enabled successfully")
                            # Defer offset application until printer is ready.
                            # SET_GCODE_OFFSET fails during boot ("Printer is not ready").
                            # The offset will be applied by _apply_pending_offset()
                            # which is called from the klippy:ready event handler.
                            self._pending_offset_tool = tool
                            logging.info(f"PuppyBootloader: T{tool} offset deferred until printer ready")
                        else:
                            logging.warning(f"PuppyBootloader: T{tool} TMC enable failed - "
                                            f"tool detected but extruder motion disabled")
                        break
            except Exception as e:
                logging.info(f"PuppyBootloader: Dock state check failed "
                             f"for Dwarf {dwarf}: {e}")

        if not self.tool_picked:
            self.active_tool = 0  # Default to T0 for MODBUS commands
            logging.info("PuppyBootloader: No tool picked on startup")

        # If [extruder] exists, get reference to its heater for temp sync
        if self._has_klipper_extruder:
            try:
                extruder = self.printer.lookup_object('extruder')
                self._extruder_heater = extruder.get_heater()
                logging.info("PuppyBootloader: Found [extruder] heater for MODBUS sync")
            except Exception as e:
                logging.warning(f"PuppyBootloader: Could not get extruder heater: {e}")
                self._extruder_heater = None

    def _handle_disconnect(self):
        """Clean shutdown of polling - fully unregister all timers."""
        self.polling_active = False
        timer = self.poll_timer
        if timer is not None:
            self.reactor.unregister_timer(timer)
            self.poll_timer = None
        # Note: Heater sync is now part of unified poll timer, not separate

    def _handle_shutdown(self):
        """Emergency shutdown - turn off all heaters and reset Dwarfs.

        Multiple layers of safety:
        1. Try MODBUS heater off commands (may work in some error states)
        2. Clear internal temp tracking
        3. PCA9557 hardware reset as final safety measure
        """
        logging.warning("PuppyBootloader: SHUTDOWN - turning off all heaters and motors")
        self.polling_active = False

        # Layer 1: Try to turn off heaters via MODBUS (may work in some error states)
        # Use non-blocking write attempts - don't wait for responses
        for dwarf in list(self.booted_dwarfs):
            try:
                # Write 0 to heater target register (0xE000)
                self._write_register_nowait(dwarf, 0xE000, 0)
                logging.info(f"PuppyBootloader: Sent heater off to Dwarf {dwarf}")
            except Exception as e:
                logging.warning(f"PuppyBootloader: Could not send heater off to Dwarf {dwarf}: {e}")

        # Layer 2: Clear target temps tracking so they don't get re-sent
        self.target_temps.clear()

        # Layer 3: Try to disable Klipper's extruder heater
        try:
            if self._extruder_heater is not None:
                self._extruder_heater.set_temp(0)
                logging.info("PuppyBootloader: Klipper extruder heater set to 0")
        except Exception as e:
            logging.warning(f"PuppyBootloader: Could not disable Klipper heater: {e}")

        # Layer 4: PCA9557 hardware reset - puts Dwarf MCUs in reset state
        try:
            self._pca9557_set(self.ALL_DWARF_BITS)
            logging.warning("PuppyBootloader: All Dwarfs forced into reset via PCA9557")
        except Exception as e:
            logging.error(f"PuppyBootloader: Failed to reset Dwarfs via PCA9557: {e}")

        # Layer 5: Try to disable steppers via toolhead
        try:
            toolhead = self.printer.lookup_object('toolhead', None)
            if toolhead:
                toolhead.motor_off()
                logging.info("PuppyBootloader: Motors disabled via toolhead")
        except Exception as e:
            logging.warning(f"PuppyBootloader: Could not disable motors: {e}")

    def _wait_for_tool_temp(self, dwarf, target_temp, tolerance=5.0, timeout=120.0):
        """Wait for tool to reach target temperature after pick.

        Prusa-style behavior: After picking a tool that was in standby (cooled down),
        wait for it to heat back up to target temp before allowing extrusion.

        Args:
            dwarf: Dwarf address (1-5)
            target_temp: Target temperature in Celsius
            tolerance: How close to target is acceptable (default 5°C)
            timeout: Maximum wait time in seconds (default 120s)

        Returns:
            True if temp reached, False if timeout
        """
        if target_temp <= 0:
            return True  # No target set, nothing to wait for

        min_temp = target_temp - tolerance
        start_time = self.reactor.monotonic()
        last_report = start_time

        self.gcode.respond_info(f"Waiting for T{dwarf-1} to reach {target_temp}C...")

        while True:
            # Check timeout
            elapsed = self.reactor.monotonic() - start_time
            if elapsed > timeout:
                logging.warning(f"PuppyBootloader: Timeout waiting for Dwarf {dwarf} "
                              f"to reach {target_temp}C")
                self.gcode.respond_info(f"WARNING: Timeout waiting for temp - proceeding anyway")
                return False

            # Read current temperature from Dwarf
            try:
                # Register 0x8000 = hotend_measured_temperature (int16, 0.1°C units)
                regs = self._read_input_registers(dwarf, 0x8000, 1)
                if regs is not None and len(regs) > 0:
                    current_temp = regs[0] / 10.0  # Convert from 0.1°C to °C

                    # CRITICAL: Update dwarf_data to keep DwarfTemperatureSensor fresh
                    # Without this, sensor could see stale _poll_time during long waits
                    if dwarf in self.dwarf_data:
                        self.dwarf_data[dwarf]['hotend_temp'] = int(current_temp)
                        self.dwarf_data[dwarf]['_poll_time'] = self.reactor.monotonic()
                    else:
                        self.dwarf_data[dwarf] = {
                            'hotend_temp': int(current_temp),
                            '_poll_time': self.reactor.monotonic(),
                        }

                    # Report progress every 5 seconds
                    if self.reactor.monotonic() - last_report >= 5.0:
                        self.gcode.respond_info(
                            f"T{dwarf-1} heating: {current_temp:.1f}C / {target_temp}C")
                        last_report = self.reactor.monotonic()

                    # Check if we've reached target
                    if current_temp >= min_temp:
                        self.gcode.respond_info(
                            f"T{dwarf-1} ready at {current_temp:.1f}C")
                        return True
            except Exception as e:
                logging.warning(f"PuppyBootloader: Error reading temp from Dwarf {dwarf}: {e}")

            # Wait before next check (500ms)
            self.reactor.pause(self.reactor.monotonic() + 0.5)

    def _write_register_nowait(self, dwarf, register, value):
        """Write register without waiting for response - for emergency use."""
        try:
            # Use proper MODBUS address (ADDR_MODBUS_OFFSET + dwarf)
            addr = self.ADDR_MODBUS_OFFSET + dwarf
            data = [
                (register >> 8) & 0xFF,
                register & 0xFF,
                (value >> 8) & 0xFF,
                value & 0xFF
            ]
            frame = self._build_modbus_frame(addr, self.MODBUS_WRITE_SINGLE, data)

            # Send via MCU - don't wait for response during shutdown
            if self.send_raw_cmd:
                self.send_raw_cmd.send([frame])
        except Exception as e:
            raise Exception(f"MODBUS nowait write failed: {e}")

    def _start_polling(self):
        """Start single unified MODBUS timer.

        Per Klipper architecture best practices and Prusa reference implementation:
        - Use a SINGLE timer for all MODBUS operations
        - Heater sync is interleaved at end of each poll cycle (not separate timer)
        - This eliminates timer race conditions that cause bus lockup
        """
        self.polling_active = True
        self.poll_dwarf_index = 0
        self._heater_sync_counter = 0  # Track cycles for heater sync frequency
        # Register single unified timer
        waketime = self.reactor.monotonic() + 2.0
        self.poll_timer = self.reactor.register_timer(self._poll_callback, waketime)
        logging.info(f"PuppyBootloader: Unified MODBUS timer started for Dwarfs {self.booted_dwarfs} "
                     f"(every {self.poll_interval}s)")

    def _pause_polling_timer(self):
        """Safely pause the poll timer to avoid MODBUS collisions.

        NOTE: Only pause poll_timer, NOT heater_sync_timer. The heater sync
        timer manages its own timing via return value from callback. Pausing
        it while inside its callback causes reactor state corruption.

        Captures timer reference locally to avoid race condition where another
        thread could set poll_timer to None between check and update.
        """
        timer = self.poll_timer
        if timer is not None:
            self.reactor.update_timer(timer, self.reactor.NEVER)

    def _resume_polling_timer(self, delay=1.0):
        """Safely resume the poll timer after MODBUS operation.

        NOTE: Only resume poll_timer, NOT heater_sync_timer. The heater sync
        timer manages its own timing via return value from callback.

        Only resumes if polling is still active and timer exists.
        Captures timer reference locally to avoid race condition.
        """
        timer = self.poll_timer
        if timer is not None and self.polling_active:
            self.reactor.update_timer(timer, self.reactor.monotonic() + delay)

    def _poll_callback(self, eventtime):
        """Reactor timer callback - polls Dwarfs and bed to keep watchdogs alive"""
        if not self.polling_active or not self.booted_dwarfs:
            return self.reactor.NEVER

        dwarf = self.booted_dwarfs[self.poll_dwarf_index]
        addr = self.ADDR_MODBUS_OFFSET + dwarf  # Dwarf N at 0x1A + N

        # Read 7 input registers starting at 0x8060 (includes fault_status)
        # Registers: fault_status, hotend_temp, heater_pwm, filament_sensor,
        #            board_temp, mcu_temp, heatbreak_temp
        reg_start = 0x8060
        count = 7
        data = [
            (reg_start >> 8) & 0xFF, reg_start & 0xFF,
            (count >> 8) & 0xFF, count & 0xFF
        ]
        frame = self._build_modbus_frame(addr, self.MODBUS_READ_INPUT, data)
        try:
            response = self._send_frame(frame)
            status = response.get('status', -1)
            resp_data = response.get('data', b'')
            if status == 0 and resp_data and len(resp_data) >= 17:
                byte_count = resp_data[2]
                if byte_count >= 14:  # 7 registers x 2 bytes
                    regs = []
                    for i in range(7):
                        val = (resp_data[3 + i*2] << 8) | resp_data[3 + i*2 + 1]
                        regs.append(val)

                    # Check fault_status FIRST (reg 0) - Dwarf-side thermal protection
                    fault_status = regs[0]
                    if fault_status & self.FAULT_MARLIN_KILLED:
                        self._handle_dwarf_fault(dwarf, fault_status, "MARLIN_KILLED - thermal runaway or heater fault")
                        return self.reactor.NEVER
                    if fault_status & self.FAULT_TMC_FAULT:
                        self._handle_dwarf_fault(dwarf, fault_status, "TMC driver fault")
                        return self.reactor.NEVER

                    self.dwarf_data[dwarf] = {
                        'fault_status': fault_status,
                        'hotend_temp': regs[1] if regs[1] < 0x8000 else regs[1] - 0x10000,
                        'heater_pwm': regs[2],
                        'filament_sensor': regs[3],
                        'board_temp': regs[4] if regs[4] < 0x8000 else regs[4] - 0x10000,
                        'mcu_temp': regs[5] if regs[5] < 0x8000 else regs[5] - 0x10000,
                        'heatbreak_temp': regs[6] if regs[6] < 0x8000 else regs[6] - 0x10000,
                        '_poll_time': self.reactor.monotonic(),  # Timestamp for staleness check
                    }

                    # Evaluate filament sensor state and check for runout
                    tool = dwarf - 1
                    fs_raw = regs[3]
                    new_state = self._evaluate_filament_state(fs_raw, tool)
                    old_state = self.fs_state.get(tool, "unknown")
                    self.fs_state[tool] = new_state
                    if old_state != new_state and old_state != "unknown":
                        logging.info(f"PuppyBootloader: T{tool} filament: "
                                   f"{old_state} -> {new_state} (raw={fs_raw})")
                    self._check_filament_runout(tool, new_state)

                    # Tool button check - only on active picked tool
                    if self.tool_picked and tool == self.active_tool:
                        self._check_tool_buttons(dwarf, tool)
            else:
                # Poll failed - clear poll_time to trigger staleness detection
                if dwarf in self.dwarf_data:
                    self.dwarf_data[dwarf]['_poll_time'] = 0.0
                logging.info(f"PuppyBootloader: Poll Dwarf {dwarf} failed status={status}")
        except Exception as e:
            # Poll exception - clear poll_time to trigger staleness detection
            if dwarf in self.dwarf_data:
                self.dwarf_data[dwarf]['_poll_time'] = 0.0
            logging.info(f"PuppyBootloader: Poll Dwarf {dwarf} error: {e}")

        # Advance to next Dwarf
        self.poll_dwarf_index = (self.poll_dwarf_index + 1) % len(self.booted_dwarfs)

        # If we wrapped around, poll bed, sync heater, and log summary
        # All MODBUS operations happen sequentially in this single callback (no races)
        if self.poll_dwarf_index == 0:
            # Poll modular bed to keep its watchdog alive (30 second timeout)
            if self.modular_bed_booted:
                self._poll_bed()

            # Poll side filament sensors (PRIMARY runout detection)
            self._poll_side_sensors()

            # Sync heater target to MODBUS (every cycle or every few cycles)
            self._heater_sync_counter = getattr(self, '_heater_sync_counter', 0) + 1
            if self._heater_sync_counter >= 2:  # Sync every 2 poll cycles
                self._sync_extruder_heater()
                self._heater_sync_counter = 0

            if self.dwarf_data:
                temps = [f"D{d}:{self.dwarf_data[d]['hotend_temp']}C"
                         for d in sorted(self.dwarf_data.keys())]
                logging.info(f"PuppyBootloader: Temps: {' '.join(temps)}")
            return eventtime + self.poll_interval
        else:
            return eventtime + 0.5  # 500ms between Dwarfs in same cycle

    def _handle_dwarf_fault(self, dwarf, fault_status, fault_desc):
        """Handle Dwarf-reported fault (thermal runaway, TMC fault, etc.)

        This is the PROPER thermal protection - the Dwarf firmware (Marlin)
        detected a heater fault and set the fault_status register. This is
        MORE RELIABLE than Klipper's verify_heater because:
        1. Dwarf monitors at 100ms intervals (vs Klipper's ~1s)
        2. Dwarf runs PID locally, knows actual heater state
        3. Marlin's thermal protection is battle-tested

        This replaces Klipper's verify_heater for MODBUS heaters.
        """
        tool = dwarf - 1
        msg = (f"THERMAL FAULT: Dwarf {dwarf} (T{tool}) reported {fault_desc}. "
               f"fault_status=0x{fault_status:04X}. Emergency stop for safety!")

        logging.critical(f"PuppyBootloader: {msg}")

        # Show in UI immediately
        try:
            self.gcode.respond_info(f"!! {msg}")
        except:
            pass

        # Stop polling
        self.polling_active = False

        # Try to disable all heaters as safety measure
        for d in self.booted_dwarfs:
            try:
                addr = self.ADDR_MODBUS_OFFSET + d
                heater_data = [0xE0, 0x00, 0, 0]  # Heater reg 0xE000 = 0
                frame = self._build_modbus_frame(addr, self.MODBUS_WRITE_SINGLE, heater_data)
                self._send_frame(frame)
            except:
                pass

        # Trigger Klipper emergency stop
        self.printer.invoke_shutdown(msg)

    def _poll_bed(self):
        """Poll modular bed to keep its MODBUS watchdog alive (30s timeout).

        Also monitors fault_status registers for thermal protection:
        - System fault status (0x8000): thermal runaway, over/under temp, overcurrent
        - Per-bedlet faults (0xA000): short circuit, temp faults, heater issues

        The modular bed runs its own thermal protection (same as Dwarf). On critical
        faults, we trigger emergency stop. Non-critical faults are logged as warnings.
        """
        # Read measured temperatures - this keeps the bed's watchdog happy
        temps = self._read_bedlet_temperatures()
        if temps:
            self.bed_measured_temps = temps

        # Check system fault status (critical thermal faults)
        try:
            fault_regs = self._read_bed_input_registers(self.BED_REG_FAULT_STATUS, 1)
            if fault_regs:
                fault_status = fault_regs[0]
                # Filter spurious MODBUS reads (all bits set = likely communication glitch)
                if fault_status == 0x01FF or fault_status == 0xFFFF:
                    logging.warning(f"PuppyBootloader: Suspicious bed fault_status=0x{fault_status:04X} - likely MODBUS glitch, ignoring")
                elif fault_status != 0:
                    # Decode fault for logging
                    faults = []
                    if fault_status & self.BED_FAULT_THERMAL_RUNAWAY:
                        faults.append("THERMAL_RUNAWAY")
                    if fault_status & self.BED_FAULT_MAXTEMP:
                        faults.append("MAXTEMP")
                    if fault_status & self.BED_FAULT_MINTEMP:
                        faults.append("MINTEMP")
                    if fault_status & self.BED_FAULT_PREHEAT_ERROR:
                        faults.append("PREHEAT_ERROR")
                    if fault_status & self.BED_FAULT_HEATBEDLET_ERROR:
                        faults.append("BEDLET_ERROR")
                    if fault_status & self.BED_FAULT_OVERCURRENT:
                        faults.append("OVERCURRENT")
                    if fault_status & self.BED_FAULT_UNEXPECTED_CURRENT:
                        faults.append("UNEXPECTED_CURRENT")

                    fault_desc = ", ".join(faults) if faults else f"UNKNOWN(0x{fault_status:04X})"

                    # Critical faults - confirm with second read before emergency stop
                    if fault_status & self.BED_CRITICAL_FAULTS:
                        # Confirm with second read (avoid false triggers from MODBUS glitches)
                        import time
                        time.sleep(0.1)
                        confirm_regs = self._read_bed_input_registers(self.BED_REG_FAULT_STATUS, 1)
                        if confirm_regs and confirm_regs[0] == fault_status:
                            # Confirmed - real fault
                            self._handle_bed_fault(fault_status, fault_desc, critical=True)
                            return
                        else:
                            logging.warning(f"PuppyBootloader: Bed fault 0x{fault_status:04X} not confirmed on second read - ignoring")
                    else:
                        # Non-critical faults just get logged
                        logging.warning(f"PuppyBootloader: Modular bed fault: {fault_desc} "
                                       f"(0x{fault_status:04X}) - not critical, continuing")
        except Exception as e:
            # Don't crash polling on fault read failure
            logging.debug(f"PuppyBootloader: Bed fault status read error: {e}")

        # Periodically check per-bedlet faults (less frequently to reduce bus traffic)
        # Only check every 10th poll cycle
        if not hasattr(self, '_bed_fault_check_counter'):
            self._bed_fault_check_counter = 0
        self._bed_fault_check_counter += 1

        if self._bed_fault_check_counter >= 10:
            self._bed_fault_check_counter = 0
            self._check_bedlet_faults()

    def _check_bedlet_faults(self):
        """Check per-bedlet fault status registers (0xA000-0xA00F)."""
        try:
            # Read all 16 bedlet fault registers
            fault_regs = self._read_bed_input_registers(self.BED_REG_BEDLET_FAULT, 16)
            if not fault_regs:
                return

            for bedlet in range(16):
                fault = fault_regs[bedlet]
                if fault != 0:
                    # Decode bedlet fault
                    faults = []
                    if fault & self.BEDLET_FAULT_SHORT_CIRCUIT:
                        faults.append("SHORT_CIRCUIT")
                    if fault & self.BEDLET_FAULT_TEMP_ABOVE_MAX:
                        faults.append("TEMP_ABOVE_MAX")
                    if fault & self.BEDLET_FAULT_TEMP_BELOW_MIN:
                        faults.append("TEMP_BELOW_MIN")
                    if fault & self.BEDLET_FAULT_HEATER_DISCONNECTED:
                        faults.append("HEATER_DISCONNECTED")
                    if fault & self.BEDLET_FAULT_TEMP_DROP:
                        faults.append("TEMP_DROP")
                    if fault & self.BEDLET_FAULT_TEMP_PEAK:
                        faults.append("TEMP_PEAK")
                    if fault & self.BEDLET_FAULT_PREHEAT:
                        faults.append("PREHEAT")

                    fault_desc = ", ".join(faults) if faults else f"UNKNOWN(0x{fault:04X})"

                    # Critical bedlet faults trigger emergency stop
                    if fault & self.BEDLET_CRITICAL_FAULTS:
                        self._handle_bed_fault(fault, f"Bedlet {bedlet}: {fault_desc}",
                                              critical=True, bedlet=bedlet)
                        return
                    else:
                        logging.warning(f"PuppyBootloader: Bedlet {bedlet} fault: {fault_desc}")

        except Exception as e:
            logging.debug(f"PuppyBootloader: Bedlet fault check error: {e}")

    def _handle_bed_fault(self, fault_status, fault_desc, critical=True, bedlet=None):
        """Handle modular bed fault (thermal runaway, overcurrent, etc.)

        Similar to _handle_dwarf_fault, the modular bed runs its own thermal
        protection. When it reports a critical fault, we trust it and
        trigger emergency stop.

        Args:
            fault_status: The fault status register value
            fault_desc: Human-readable fault description
            critical: If True, trigger emergency stop. If False, just log warning
            bedlet: Specific bedlet number if this is a per-bedlet fault
        """
        location = f"bedlet {bedlet}" if bedlet is not None else "system"
        msg = (f"BED THERMAL FAULT: Modular bed {location} reported {fault_desc}. "
               f"fault_status=0x{fault_status:04X}.")

        if not critical:
            logging.warning(f"PuppyBootloader: {msg} (non-critical)")
            try:
                self.gcode.respond_info(f"Warning: {msg}")
            except:
                pass
            return

        msg += " Emergency stop for safety!"
        logging.critical(f"PuppyBootloader: {msg}")

        # Show in UI immediately
        try:
            self.gcode.respond_info(f"!! {msg}")
        except:
            pass

        # Stop polling
        self.polling_active = False

        # Try to disable all bed heaters as safety measure
        try:
            # Set all 16 bedlet targets to 0
            zeros = [0] * 16
            self._write_bed_holding_registers(self.BED_REG_TARGET_TEMP, zeros)
        except:
            pass

        # Also disable all Dwarf heaters
        for d in self.booted_dwarfs:
            try:
                addr = self.ADDR_MODBUS_OFFSET + d
                heater_data = [0xE0, 0x00, 0, 0]  # Heater reg 0xE000 = 0
                frame = self._build_modbus_frame(addr, self.MODBUS_WRITE_SINGLE, heater_data)
                self._send_frame(frame)
            except:
                pass

        # Trigger Klipper emergency stop
        self.printer.invoke_shutdown(msg)

    def _sync_extruder_heater(self):
        """Sync heater target temperatures to MODBUS for ALL tools.

        When [extruder] is configured with NullPWM, Klipper's M104/M109
        update the heater's target but don't actually heat anything.
        This method syncs:
        1. Active tool: from Klipper's [extruder] heater target
        2. Parked tools: from self.target_temps (set by M104 Tn Sxxx)

        MULTI-TOOL FIX: Parked tools need periodic heater target refresh
        because the Dwarf firmware may clear heater targets after deselect
        or due to watchdog timeouts. This keeps all tools at their set temps.

        NOTE: This is called from within the unified poll callback, so it
        does DIRECT MODBUS without timer pause/resume (no nested operations).
        """
        active_dwarf = self.active_tool + 1 if self.active_tool >= 0 else -1

        # Part 1: Sync active tool from Klipper heater target
        if self._extruder_heater is not None and active_dwarf > 0:
            try:
                target = self._extruder_heater.target_temp
                if target != self._last_synced_temp:
                    if active_dwarf in self.booted_dwarfs:
                        addr = self.ADDR_MODBUS_OFFSET + active_dwarf
                        reg = 0xE000
                        data = [
                            (reg >> 8) & 0xFF, reg & 0xFF,
                            (int(target) >> 8) & 0xFF, int(target) & 0xFF
                        ]
                        frame = self._build_modbus_frame(addr, self.MODBUS_WRITE_SINGLE, data)
                        response = self._send_frame(frame)
                        if response.get('status', -1) == 0:
                            self._last_synced_temp = target
                            self.target_temps[active_dwarf] = target
                            logging.info(f"PuppyBootloader: Synced extruder temp {target}C "
                                        f"to Dwarf {active_dwarf}")
            except Exception as e:
                logging.warning(f"PuppyBootloader: Active heater sync error: {e}")

        # Part 2: Refresh parked tools' heater targets (MULTI-TOOL FIX)
        # This keeps parked tools warm during multi-color prints.
        # Only refresh every few cycles to avoid flooding MODBUS.
        self._parked_heater_refresh_counter = getattr(self, '_parked_heater_refresh_counter', 0) + 1
        if self._parked_heater_refresh_counter >= 10:  # Refresh every ~10 poll cycles
            self._parked_heater_refresh_counter = 0
            for dwarf in self.booted_dwarfs:
                if dwarf == active_dwarf:
                    continue  # Skip active tool (already handled above)
                if dwarf in self.target_temps and self.target_temps[dwarf] > 0:
                    try:
                        target = self.target_temps[dwarf]
                        addr = self.ADDR_MODBUS_OFFSET + dwarf
                        reg = 0xE000
                        data = [
                            (reg >> 8) & 0xFF, reg & 0xFF,
                            (int(target) >> 8) & 0xFF, int(target) & 0xFF
                        ]
                        frame = self._build_modbus_frame(addr, self.MODBUS_WRITE_SINGLE, data)
                        response = self._send_frame(frame)
                        if response.get('status', -1) == 0:
                            tool = dwarf - 1
                            logging.info(f"PuppyBootloader: Refreshed parked T{tool} "
                                        f"heater target {target}C")
                    except Exception as e:
                        logging.warning(f"PuppyBootloader: Parked heater refresh error: {e}")

    def _poll_all_dwarfs_quick(self, exclude_dwarf=None):
        """Quick poll of all Dwarfs to keep watchdogs alive during long operations."""
        for dwarf in self.booted_dwarfs:
            if exclude_dwarf and dwarf == exclude_dwarf:
                continue
            addr = self.ADDR_MODBUS_OFFSET + dwarf  # Dwarf N at 0x1A + N
            reg_start = 0x8061
            count = 1
            data = [
                (reg_start >> 8) & 0xFF, reg_start & 0xFF,
                (count >> 8) & 0xFF, count & 0xFF
            ]
            frame = self._build_modbus_frame(addr, self.MODBUS_READ_INPUT, data)
            try:
                self._send_frame(frame)
            except:
                pass
            self.reactor.pause(self.reactor.monotonic() + 0.01)

    def _crc16(self, data, init=0xFFFF):
        """CRC16 with configurable init (0xFFFF for MODBUS, 0x0000 for bootloader)"""
        crc = init
        for byte in data:
            crc = (crc >> 8) ^ self.CRC_TABLE[(crc ^ byte) & 0xFF]
        return crc
    
    def _build_bootloader_frame(self, address, command, data=None):
        """Build bootloader protocol frame (CRC init=0xFFFF - same as MODBUS CRC16-IBM)"""
        if data is None:
            data = []
        frame = bytes([address, command] + list(data))
        crc = self._crc16(frame, init=0xFFFF)  # FIXED: Bootloader uses 0xFFFF init, same as MODBUS
        return frame + bytes([crc & 0xFF, (crc >> 8) & 0xFF])
    
    def _build_modbus_frame(self, address, function, data):
        """Build MODBUS RTU frame (CRC init=0xFFFF)"""
        frame = bytes([address, function] + list(data))
        crc = self._crc16(frame, init=0xFFFF)
        return frame + bytes([crc & 0xFF, (crc >> 8) & 0xFF])
    
    def _send_frame(self, frame):
        """Send frame via MCU and return response"""
        if not self.send_raw_cmd:
            raise self.gcode.error("modbus_send_raw command not available")
        response = self.send_raw_cmd.send([frame])
        return response
    
    # ==================== BOOTLOADER COMMANDS ====================
    
    def cmd_BOOTLOADER_SEND(self, gcmd):
        """Send raw hex bytes"""
        hex_data = gcmd.get('DATA', '')
        try:
            data = bytes.fromhex(hex_data)
        except:
            raise gcmd.error("Invalid hex data")
        
        self.gcode.respond_info(f"Sending {len(data)} bytes: {data.hex()}")
        
        try:
            response = self._send_frame(data)
            status = response.get('status', -1)
            resp_data = response.get('data', b'')
            self.gcode.respond_info(f"Status: {status}, Data: {resp_data.hex() if resp_data else 'none'}")
        except Exception as e:
            self.gcode.respond_info(f"Error: {e}")
    
    def cmd_BOOTLOADER_START_APP(self, gcmd):
        """Send START_APPLICATION command"""
        address = gcmd.get_int('ADDR', 0)
        
        # Data: 4 bytes salt (0) + 32 bytes fingerprint (0)
        data = [0] * 36
        frame = self._build_bootloader_frame(address, self.CMD_START_APPLICATION, data)
        
        self.gcode.respond_info(
            f"Sending START_APPLICATION to addr={address}\n"
            f"Frame ({len(frame)} bytes): {frame.hex()}")
        
        try:
            response = self._send_frame(frame)
            status = response.get('status', -1)
            resp_data = response.get('data', b'')
            self.gcode.respond_info(f"Status: {status}, Data: {resp_data.hex() if resp_data else 'none'}")
        except Exception as e:
            self.gcode.respond_info(f"Error: {e}")
    
    def cmd_BOOTLOADER_DISCOVER(self, gcmd):
        """Send GET_PROTOCOL_VERSION to check for bootloader"""
        address = gcmd.get_int('ADDR', 0)
        
        frame = self._build_bootloader_frame(address, self.CMD_GET_PROTOCOL_VERSION)
        
        self.gcode.respond_info(
            f"Sending GET_PROTOCOL_VERSION to addr={address}\n"
            f"Frame ({len(frame)} bytes): {frame.hex()}")
        
        try:
            response = self._send_frame(frame)
            status = response.get('status', -1)
            resp_data = response.get('data', b'')
            self.gcode.respond_info(f"Status: {status}, Data: {resp_data.hex() if resp_data else 'none'}")
        except Exception as e:
            self.gcode.respond_info(f"Error: {e}")
    
    def cmd_BOOTLOADER_HWINFO(self, gcmd):
        """Send GET_HARDWARE_INFO to get Dwarf info"""
        address = gcmd.get_int('ADDR', 0)
        
        frame = self._build_bootloader_frame(address, self.CMD_GET_HARDWARE_INFO)
        
        self.gcode.respond_info(
            f"Sending GET_HARDWARE_INFO to addr={address}\n"
            f"Frame ({len(frame)} bytes): {frame.hex()}")
        
        try:
            response = self._send_frame(frame)
            status = response.get('status', -1)
            resp_data = response.get('data', b'')
            if status == 0 and resp_data and len(resp_data) >= 14:
                # Parse response: [addr][status][len][data...][crc16]
                # Skip addr(1)+status(1)+len(1), then parse 11 bytes of hw info
                hw_type = resp_data[3]
                hw_rev = (resp_data[4] << 8) | resp_data[5]
                bl_ver = (resp_data[6] << 24) | (resp_data[7] << 16) | (resp_data[8] << 8) | resp_data[9]
                app_size = (resp_data[10] << 24) | (resp_data[11] << 16) | (resp_data[12] << 8) | resp_data[13]
                self.gcode.respond_info(
                    f"Status: {status}, HW Type: {hw_type}, HW Rev: {hw_rev}, "
                    f"BL Ver: {bl_ver}, App Size: {app_size}")
            else:
                self.gcode.respond_info(f"Status: {status}, Data: {resp_data.hex() if resp_data else 'none'}")
        except Exception as e:
            self.gcode.respond_info(f"Error: {e}")
    
    def cmd_BOOTLOADER_ASSIGN(self, gcmd):
        """Assign new address to Dwarf at current address"""
        from_addr = gcmd.get_int('FROM', 0)  # Current address (usually 0x00)
        to_addr = gcmd.get_int('TO', 10)     # New address (0x0A = 10)
        
        # SET_ADDRESS command: [new_addr, 0x00]
        data = [to_addr, 0x00]
        frame = self._build_bootloader_frame(from_addr, self.CMD_SET_ADDRESS, data)
        
        self.gcode.respond_info(
            f"Assigning address: {from_addr} -> {to_addr}\n"
            f"Frame ({len(frame)} bytes): {frame.hex()}")
        
        try:
            # This is a no-reply command, but we still send it
            response = self._send_frame(frame)
            status = response.get('status', -1)
            self.gcode.respond_info(f"Status: {status} (no reply expected)")
        except Exception as e:
            self.gcode.respond_info(f"Error: {e}")
    
    def cmd_BOOTLOADER_COMPUTE_FP(self, gcmd):
        """Ask Dwarf to compute fingerprint with given salt"""
        address = gcmd.get_int('ADDR', 10)
        salt = gcmd.get_int('SALT', 0x12345678)
        
        # COMPUTE_FINGERPRINT: [salt_be 4 bytes]
        data = [
            (salt >> 24) & 0xFF,
            (salt >> 16) & 0xFF,
            (salt >> 8) & 0xFF,
            salt & 0xFF
        ]
        frame = self._build_bootloader_frame(address, self.CMD_COMPUTE_FINGERPRINT, data)
        
        self.gcode.respond_info(
            f"Compute fingerprint: addr={address}, salt=0x{salt:08X}\n"
            f"Frame ({len(frame)} bytes): {frame.hex()}")
        
        try:
            response = self._send_frame(frame)
            status = response.get('status', -1)
            self.gcode.respond_info(f"Status: {status}")
        except Exception as e:
            self.gcode.respond_info(f"Error: {e}")
    
    def cmd_BOOTLOADER_GET_FP(self, gcmd):
        """Get computed fingerprint from Dwarf"""
        address = gcmd.get_int('ADDR', 10)
        offset = gcmd.get_int('OFFSET', 0)
        size = gcmd.get_int('SIZE', 32)
        
        # GET_FINGERPRINT: [offset, size]
        data = [offset, size]
        frame = self._build_bootloader_frame(address, self.CMD_GET_FINGERPRINT, data)
        
        self.gcode.respond_info(
            f"Get fingerprint: addr={address}, offset={offset}, size={size}\n"
            f"Frame ({len(frame)} bytes): {frame.hex()}")
        
        try:
            response = self._send_frame(frame)
            status = response.get('status', -1)
            resp_data = response.get('data', b'')
            if status == 0 and resp_data:
                self.gcode.respond_info(f"Status: {status}, Fingerprint: {resp_data.hex()}")
            else:
                self.gcode.respond_info(f"Status: {status}, Data: {resp_data.hex() if resp_data else 'none'}")
        except Exception as e:
            self.gcode.respond_info(f"Error: {e}")
    
    def cmd_BOOTLOADER_BOOT_DWARF(self, gcmd):
        """Full boot sequence for a single Dwarf"""
        import time
        import random
        
        from_addr = gcmd.get_int('FROM', 0)  # Address in bootloader (0x00 or assigned)
        to_addr = gcmd.get_int('TO', 10)     # New bootloader address to assign
        
        self.gcode.respond_info(f"=== Booting Dwarf: {from_addr} -> {to_addr} ===")
        
        # Step 1: Assign address (if FROM=0)
        if from_addr == 0:
            data = [to_addr, 0x00]
            frame = self._build_bootloader_frame(from_addr, self.CMD_SET_ADDRESS, data)
            self.gcode.respond_info(f"1. Assigning address {from_addr} -> {to_addr}")
            try:
                self._send_frame(frame)
                time.sleep(0.1)
            except:
                pass
        else:
            to_addr = from_addr  # Use existing address
        
        # Step 2: Verify Dwarf responds at new address
        frame = self._build_bootloader_frame(to_addr, self.CMD_GET_PROTOCOL_VERSION)
        self.gcode.respond_info(f"2. Checking for response at address {to_addr}")
        try:
            response = self._send_frame(frame)
            status = response.get('status', -1)
            if status != 0:
                self.gcode.respond_info(f"   FAILED: No response (status={status})")
                return
            self.gcode.respond_info(f"   OK: Dwarf responding")
        except Exception as e:
            self.gcode.respond_info(f"   FAILED: {e}")
            return
        
        # Step 3: Compute fingerprint with random salt
        salt = random.randint(0, 0xFFFFFFFF)
        data = [
            (salt >> 24) & 0xFF,
            (salt >> 16) & 0xFF,
            (salt >> 8) & 0xFF,
            salt & 0xFF
        ]
        frame = self._build_bootloader_frame(to_addr, self.CMD_COMPUTE_FINGERPRINT, data)
        self.gcode.respond_info(f"3. Computing fingerprint with salt=0x{salt:08X}")
        try:
            response = self._send_frame(frame)
        except Exception as e:
            self.gcode.respond_info(f"   FAILED: {e}")
            return
        
        # Step 3b: Wait for fingerprint computation (Prusa waits up to 1000ms, polling)
        self.gcode.respond_info(f"   Waiting for SHA256 computation...")
        for attempt in range(20):  # 20 * 50ms = 1000ms max
            time.sleep(0.05)
            # Poll with GET_PROTOCOL_VERSION to check if Dwarf is ready
            poll_frame = self._build_bootloader_frame(to_addr, self.CMD_GET_PROTOCOL_VERSION)
            try:
                poll_response = self._send_frame(poll_frame)
                if poll_response.get('status', -1) == 0:
                    self.gcode.respond_info(f"   Ready after {(attempt+1)*50}ms")
                    break
            except:
                pass
        else:
            self.gcode.respond_info(f"   Warning: Dwarf may still be computing")
        
        # Step 4: Get fingerprint
        data = [0, 32]  # offset=0, size=32
        frame = self._build_bootloader_frame(to_addr, self.CMD_GET_FINGERPRINT, data)
        self.gcode.respond_info(f"4. Getting fingerprint")
        fingerprint = None
        try:
            response = self._send_frame(frame)
            status = response.get('status', -1)
            resp_data = response.get('data', b'')
            if status == 0 and resp_data and len(resp_data) >= 35:
                # Response: [addr][status][len][fingerprint 32 bytes][crc16]
                fingerprint = resp_data[3:35]
                self.gcode.respond_info(f"   OK: {fingerprint.hex()}")
            else:
                self.gcode.respond_info(f"   FAILED: status={status}, len={len(resp_data) if resp_data else 0}")
                return
        except Exception as e:
            self.gcode.respond_info(f"   FAILED: {e}")
            return
        
        # Step 5: Start application
        app_data = [
            (salt >> 24) & 0xFF,
            (salt >> 16) & 0xFF,
            (salt >> 8) & 0xFF,
            salt & 0xFF
        ] + list(fingerprint)
        frame = self._build_bootloader_frame(to_addr, self.CMD_START_APPLICATION, app_data)
        self.gcode.respond_info(f"5. Starting application")
        try:
            response = self._send_frame(frame)
            status = response.get('status', -1)
            resp_data = response.get('data', b'')
            if status == 0:
                self.gcode.respond_info(f"   OK: Application started! Dwarf should be GREEN now.")
            else:
                self.gcode.respond_info(f"   Status: {status}, Data: {resp_data.hex() if resp_data else 'none'}")
        except Exception as e:
            self.gcode.respond_info(f"   FAILED: {e}")
    
    # ==================== MODBUS COMMANDS ====================
    
    def cmd_MODBUS_READ(self, gcmd):
        """Read MODBUS registers (holding or input)"""
        address = gcmd.get_int('ADDR', 26)  # Default to Dwarf 1 (0x1A)
        # Parse REG as hex or decimal
        reg_str = gcmd.get('REG', '0')
        reg = int(reg_str, 16) if reg_str.startswith('0x') else int(reg_str)
        count = gcmd.get_int('COUNT', 1)
        func = gcmd.get_int('FUNC', 4)  # Default to 0x04 (input registers)

        # Function 0x03 = Read Holding Registers, 0x04 = Read Input Registers
        # Data: [reg_hi, reg_lo, count_hi, count_lo]
        data = [
            (reg >> 8) & 0xFF, reg & 0xFF,
            (count >> 8) & 0xFF, count & 0xFF
        ]
        frame = self._build_modbus_frame(address, func, data)
        
        func_names = {3: "Holding", 4: "Input"}
        self.gcode.respond_info(
            f"MODBUS Read {func_names.get(func, func)}: addr={address}, "
            f"reg=0x{reg:04X}, count={count}\nFrame: {frame.hex()}")
        
        try:
            response = self._send_frame(frame)
            status = response.get('status', -1)
            resp_data = response.get('data', b'')
            
            if status == 0 and resp_data:
                self.gcode.respond_info(f"Raw ({len(resp_data)} bytes): {resp_data.hex()}")
                # Parse MODBUS response
                if len(resp_data) >= 5:
                    resp_addr = resp_data[0]
                    resp_func = resp_data[1]
                    if resp_func & 0x80:
                        # Exception response
                        exc_code = resp_data[2]
                        exc_names = {1: "Illegal Function", 2: "Illegal Data Address",
                                     3: "Illegal Data Value", 4: "Server Failure"}
                        self.gcode.respond_info(
                            f"  EXCEPTION: {exc_names.get(exc_code, f'0x{exc_code:02X}')}")
                    else:
                        # Normal read response: [addr][func][byte_count][data...][crc]
                        byte_count = resp_data[2]
                        if len(resp_data) >= 3 + byte_count:
                            reg_data = resp_data[3:3 + byte_count]
                            values = []
                            for i in range(0, len(reg_data) - 1, 2):
                                val = (reg_data[i] << 8) | reg_data[i + 1]
                                # Show as signed if MSB set
                                signed_val = val if val < 0x8000 else val - 0x10000
                                values.append(f"0x{val:04X} ({signed_val})")
                            self.gcode.respond_info(
                                f"  Registers: {', '.join(values)}")
            else:
                status_names = {0: "OK", 1: "TIMEOUT", 2: "CRC_ERR", 3: "EXCEPTION", 4: "FRAME_ERR", 5: "BUSY"}
                self.gcode.respond_info(f"Status: {status_names.get(status, status)}, Data: {resp_data.hex() if resp_data else 'none'}")
        except Exception as e:
            self.gcode.respond_info(f"Error: {e}")
    
    def cmd_MODBUS_SCAN(self, gcmd):
        """Scan for MODBUS devices"""
        start = gcmd.get_int('START', 1)
        end = gcmd.get_int('END', 50)
        
        self.gcode.respond_info(f"Scanning MODBUS addresses {start} to {end}...")
        found = []
        
        for addr in range(start, end + 1):
            # Try to read register 0
            data = [0, 0, 0, 1]  # reg=0, count=1
            frame = self._build_modbus_frame(addr, self.MODBUS_READ_HOLDING, data)
            
            try:
                response = self._send_frame(frame)
                status = response.get('status', -1)
                resp_data = response.get('data', b'')
                
                if status == 0 and resp_data and len(resp_data) > 2:
                    found.append(addr)
                    self.gcode.respond_info(f"  Found device at address {addr}!")
            except:
                pass
        
        if found:
            self.gcode.respond_info(f"Scan complete. Found devices at: {found}")
        else:
            self.gcode.respond_info("Scan complete. No devices found.")
    
    def cmd_MODBUS_WRITE(self, gcmd):
        """Write single MODBUS register"""
        address = gcmd.get_int('ADDR', 27)
        # Parse REG as hex or decimal
        reg_str = gcmd.get('REG', '0')
        reg = int(reg_str, 16) if reg_str.startswith('0x') else int(reg_str)
        value = gcmd.get_int('VALUE', 0)
        
        # Function 0x06 = Write Single Register
        data = [
            (reg >> 8) & 0xFF, reg & 0xFF,
            (value >> 8) & 0xFF, value & 0xFF
        ]
        frame = self._build_modbus_frame(address, self.MODBUS_WRITE_SINGLE, data)
        
        self.gcode.respond_info(
            f"MODBUS Write: addr={address}, reg={reg}, value={value}\n"
            f"Frame: {frame.hex()}")
        
        try:
            response = self._send_frame(frame)
            status = response.get('status', -1)
            resp_data = response.get('data', b'')
            self.gcode.respond_info(f"Status: {status}, Data: {resp_data.hex() if resp_data else 'none'}")
        except Exception as e:
            self.gcode.respond_info(f"Error: {e}")

    # ==================== STANDARD GCODE COMMANDS ====================

    def _make_tool_cmd(self, tool_index):
        """Create a closure for tool select command.

        Prusa slicer parameters (for compatibility):
        - S: Don't return to position after change (S1 = no return)
        - L: Z lift setting (0=none, 1=MBL diff, 2=full lift)
        - D: Z return after lift (0=don't return, 1=normal)
        - Q: Just switch MODBUS, no physical movement (Q1)
        """
        def cmd(gcmd):
            # Spool join remap: if this gcode tool is remapped, use physical tool
            physical_tool = self.tool_remap.get(tool_index, tool_index)
            if physical_tool != tool_index:
                self.gcode.respond_info(
                    f"Spool join: T{tool_index} remapped to T{physical_tool}")
            new_dwarf = physical_tool + 1
            quick = gcmd.get_int('Q', 0)

            # Parse Prusa slicer parameters (logged for debugging)
            s_param = gcmd.get_int('S', 0)  # S1 = no XY return
            l_param = gcmd.get_int('L', 2)  # L0=no lift, L1=MBL, L2=full
            d_param = gcmd.get_int('D', 1)  # D0=no Z return, D1=normal

            if quick or new_dwarf not in self.booted_dwarfs:
                # Quick switch (just MODBUS, no physical movement)
                old_tool = self.active_tool
                old_dwarf = old_tool + 1 if old_tool >= 0 else -1

                if new_dwarf in self.booted_dwarfs:
                    # Enable TMC on new tool FIRST with retry (Phase 2 safety fix)
                    # Do NOT change selection state until TMC is verified
                    tmc_enabled = False
                    for attempt in range(3):
                        if self._write_coil(new_dwarf, 0x4000, True):
                            tmc_enabled = True
                            self.tmc_enabled[new_dwarf] = True
                            break
                        self.reactor.pause(self.reactor.monotonic() + 0.1)
                    if not tmc_enabled:
                        logging.error(f"PuppyBootloader: Quick change TMC enable failed for Dwarf {new_dwarf}")
                        raise gcmd.error(f"CRITICAL: TMC enable failed on T{tool_index} - extruder motion unsafe")

                    # TMC enabled - NOW safe to switch selection state
                    # Disable old tool TMC and deselect
                    if old_dwarf > 0 and old_dwarf != new_dwarf:
                        if self.tmc_enabled.get(old_dwarf, False):
                            self._write_coil(old_dwarf, 0x4000, False)
                            self.tmc_enabled[old_dwarf] = False
                        self._write_coil(old_dwarf, 0x4001, False)  # Deselect old
                        # MULTI-TOOL FIX: Preserve heater target after deselect
                        if old_dwarf in self.target_temps and self.target_temps[old_dwarf] > 0:
                            self._write_register(old_dwarf, 0xE000, int(self.target_temps[old_dwarf]))
                            logging.info(f"PuppyBootloader: Quick change - preserved heater "
                                        f"{self.target_temps[old_dwarf]}C for T{old_tool}")

                    # Select new tool
                    self._write_coil(new_dwarf, 0x4001, True)

                    # Wait for MODBUS to complete before allowing motion
                    self.reactor.pause(self.reactor.monotonic() + 0.15)
                else:
                    # Dwarf not booted - just deselect old if needed
                    if old_dwarf > 0 and old_dwarf != new_dwarf:
                        if self.tmc_enabled.get(old_dwarf, False):
                            self._write_coil(old_dwarf, 0x4000, False)
                            self.tmc_enabled[old_dwarf] = False
                        self._write_coil(old_dwarf, 0x4001, False)
                        # MULTI-TOOL FIX: Preserve heater target after deselect
                        if old_dwarf in self.target_temps and self.target_temps[old_dwarf] > 0:
                            self._write_register(old_dwarf, 0xE000, int(self.target_temps[old_dwarf]))
                            logging.info(f"PuppyBootloader: Quick change - preserved heater "
                                        f"{self.target_temps[old_dwarf]}C for T{old_tool}")

                # CRITICAL: Tool change heater safety handling (quick mode)
                # Reset verify_heater state before changing active_tool
                if self._extruder_heater is not None:
                    self._extruder_heater.set_temp(0.)
                    logging.info(f"PuppyBootloader: Quick tool change - "
                               f"temporarily disabled Klipper heater")

                # NOW change active_tool - sensor switches to new tool's temperature
                self.active_tool = physical_tool
                # Set tool_picked=True for state coherence when TMC enabled
                if new_dwarf in self.booted_dwarfs:
                    self.tool_picked = True

                # Wait for sensor to pick up new tool's temperature
                self.reactor.pause(self.reactor.monotonic() + 0.2)

                # MULTI-TOOL FIX: Use NEW tool's stored temp, not old tool's
                new_tool_target = self.target_temps.get(new_dwarf, 0)
                if new_tool_target > 0 and new_dwarf in self.booted_dwarfs:
                    self._write_register(new_dwarf, 0xE000, int(new_tool_target))
                    logging.info(f"PuppyBootloader: Quick change - using T{physical_tool} "
                               f"stored target {new_tool_target}C")

                # Set Klipper heater target to NEW tool's temp (allows extrusion)
                if self._extruder_heater is not None and new_tool_target > 0:
                    self._extruder_heater.set_temp(float(new_tool_target))
                    logging.info(f"PuppyBootloader: Quick tool change complete - "
                               f"Klipper heater target set to {new_tool_target}C")

                # CRITICAL: Poll the new tool to update dwarf_data with fresh _poll_time
                # This ensures DwarfTemperatureSensor has fresh data after multiple tool changes
                if new_dwarf in self.booted_dwarfs:
                    self._poll_dwarf_for_sensor(new_dwarf)

                # Wait for tool to reach its TARGET temp if it's cold
                if new_tool_target > 0:
                    current_temp = self.dwarf_data.get(new_dwarf, {}).get('hotend_temp', 0)
                    if current_temp < (new_tool_target - 5):  # 5C tolerance
                        self.gcode.respond_info(f"T{physical_tool} at {current_temp}C - waiting for target {new_tool_target}C...")
                        self._wait_for_tool_temp(new_dwarf, new_tool_target, tolerance=5.0)

                self._last_synced_temp = 0  # Force re-sync

                self.gcode.respond_info(
                    f"Tool {physical_tool} active (Dwarf {new_dwarf}, TMC enabled)")
            else:
                # Full physical tool change
                if self.active_tool == physical_tool and self.tool_picked:
                    # Still apply offsets - they may not have been applied
                    # after a restart (auto-detect fails before printer ready)
                    offset = self.tool_offsets.get(physical_tool, (0.0, 0.0, 0.0))
                    ox, oy, oz = offset
                    tool_z_adj = 0.0
                    tool_offsets_mod = self.printer.lookup_object('tool_offsets', None)
                    if tool_offsets_mod is not None:
                        tool_z_adj = tool_offsets_mod.get_z_offset(physical_tool)
                    total_z = -oz + tool_z_adj
                    self.gcode.run_script_from_command(
                        f"SET_GCODE_OFFSET X={-ox:.4f} Y={-oy:.4f} Z={total_z:.4f} MOVE=0")
                    self.applied_tool_offset = offset
                    self.gcode.respond_info(
                        f"T{physical_tool} already picked!")
                    return

                # Store return position if S=0 (default: return after change)
                no_return = (s_param >= 1)

                if self.tool_picked and self.active_tool != physical_tool:
                    # Park current tool first
                    self.gcode.respond_info(
                        f"Parking T{self.active_tool}...")
                    self.gcode.run_script_from_command("TOOL_PARK")
                if not self.tool_picked:
                    # Pick new tool
                    self.gcode.respond_info(f"Picking T{physical_tool}...")
                    self.gcode.run_script_from_command(
                        f"TOOL_PICK T={physical_tool}")
        return cmd

    def _record_modbus_success(self, dwarf):
        """Record successful MODBUS communication for health tracking."""
        self._modbus_consecutive_failures[dwarf] = 0
        self._modbus_last_success[dwarf] = self.reactor.monotonic()

    def _record_modbus_failure(self, dwarf):
        """Record failed MODBUS communication and check for escalation."""
        failures = self._modbus_consecutive_failures.get(dwarf, 0) + 1
        self._modbus_consecutive_failures[dwarf] = failures

        if failures >= self._modbus_failure_threshold:
            self._escalate_modbus_failure(dwarf)

    def _escalate_modbus_failure(self, dwarf):
        """Escalate repeated MODBUS failures - emergency stop for safety."""
        logging.error(f"PuppyBootloader: MODBUS failure threshold reached for Dwarf {dwarf}")

        # Try to disable heater as safety measure
        try:
            addr = self.ADDR_MODBUS_OFFSET + dwarf
            data = [(0xE0), (0x00), 0, 0]  # Heater reg 0xE000 = 0
            frame = self._build_modbus_frame(addr, self.MODBUS_WRITE_SINGLE, data)
            self._send_frame(frame)
        except:
            pass

        # Reset counter before shutdown to avoid recursion
        self._modbus_consecutive_failures[dwarf] = 0

        # Critical safety: trigger emergency stop
        # Lost communication with toolhead means we can't control heater/extruder
        msg = (f"MODBUS communication lost with Dwarf {dwarf} (T{dwarf-1}). "
               f"Emergency stop triggered for safety - heater control unavailable.")
        logging.critical(msg)

        # Make sure user sees this error in UI, not just logs
        try:
            self.gcode.respond_info(f"!! EMERGENCY STOP: {msg}")
        except:
            pass

        self.printer.invoke_shutdown(msg)

    def _write_register(self, dwarf, reg, value):
        """Write a single register to a Dwarf, return True on success"""
        addr = self.ADDR_MODBUS_OFFSET + dwarf  # Dwarf N at 0x1A + N
        data = [
            (reg >> 8) & 0xFF, reg & 0xFF,
            (value >> 8) & 0xFF, value & 0xFF
        ]
        frame = self._build_modbus_frame(addr, self.MODBUS_WRITE_SINGLE, data)
        logging.info(f"PuppyBootloader: FC06 frame to Dwarf {dwarf}: {frame.hex()}")
        # Pause polling to avoid response collisions
        self._pause_polling_timer()
        try:
            # Wait longer for MODBUS to become idle
            self.reactor.pause(self.reactor.monotonic() + 0.3)

            # Retry up to 3 times if busy
            for attempt in range(3):
                response = self._send_frame(frame)
                status = response.get('status', -1)
                if status == 0:
                    self._record_modbus_success(dwarf)
                    return True
                elif status == 1:  # MODBUS_ERR_TIMEOUT - retry after pause
                    logging.info(f"PuppyBootloader: Write reg 0x{reg:04X} timeout, retry {attempt+1}")
                    self.reactor.pause(self.reactor.monotonic() + 0.2)
                else:
                    logging.info(f"PuppyBootloader: Write reg 0x{reg:04X}={value} "
                                 f"to Dwarf {dwarf} status={status}")
                    self._record_modbus_failure(dwarf)
                    return False
            logging.info(f"PuppyBootloader: Write reg 0x{reg:04X}={value} "
                         f"to Dwarf {dwarf} failed after 3 retries")
            self._record_modbus_failure(dwarf)
            return False
        except Exception as e:
            logging.info(f"PuppyBootloader: Write exception: {e}")
            self._record_modbus_failure(dwarf)
            return False
        finally:
            # Resume polling
            self._resume_polling_timer()

    def _write_coil(self, dwarf, coil_addr, value):
        """Write a single coil on a Dwarf via MODBUS function 0x05"""
        addr = self.ADDR_MODBUS_OFFSET + dwarf  # Dwarf N at 0x1A + N
        # Coil value: 0xFF00 = ON, 0x0000 = OFF
        coil_val = 0xFF00 if value else 0x0000
        data = [
            (coil_addr >> 8) & 0xFF, coil_addr & 0xFF,
            (coil_val >> 8) & 0xFF, coil_val & 0xFF
        ]
        frame = self._build_modbus_frame(addr, self.MODBUS_WRITE_COIL, data)
        logging.info(f"PuppyBootloader: FC05 frame to Dwarf {dwarf}: {frame.hex()}")
        self._pause_polling_timer()
        try:
            self.reactor.pause(self.reactor.monotonic() + 0.1)

            # Retry up to 3 times if busy (matching _write_register behavior)
            for attempt in range(3):
                response = self._send_frame(frame)
                status = response.get('status', -1)
                if status == 0:
                    self._record_modbus_success(dwarf)
                    return True
                elif status == 1:  # MODBUS_ERR_TIMEOUT - retry after pause
                    logging.info(f"PuppyBootloader: Write coil 0x{coil_addr:04X} timeout, retry {attempt+1}")
                    self.reactor.pause(self.reactor.monotonic() + 0.2)
                else:
                    logging.info(f"PuppyBootloader: Write coil 0x{coil_addr:04X}={value} "
                                 f"to Dwarf {dwarf} status={status}")
                    self._record_modbus_failure(dwarf)
                    return False
            logging.info(f"PuppyBootloader: Write coil 0x{coil_addr:04X}={value} "
                         f"to Dwarf {dwarf} failed after 3 busy retries")
            self._record_modbus_failure(dwarf)
            return False
        except Exception as e:
            logging.info(f"PuppyBootloader: Write coil exception: {e}")
            self._record_modbus_failure(dwarf)
            return False
        finally:
            self._resume_polling_timer()

    def _write_bed_coil(self, coil_addr, value):
        """Write a single coil to modular bed via MODBUS function 0x05"""
        addr = self.ADDR_MODULAR_BED  # Bed at 0x1A
        # Coil value: 0xFF00 = ON, 0x0000 = OFF
        coil_val = 0xFF00 if value else 0x0000
        data = [
            (coil_addr >> 8) & 0xFF, coil_addr & 0xFF,
            (coil_val >> 8) & 0xFF, coil_val & 0xFF
        ]
        frame = self._build_modbus_frame(addr, self.MODBUS_WRITE_COIL, data)
        self._pause_polling_timer()
        try:
            self.reactor.pause(self.reactor.monotonic() + 0.05)
            response = self._send_frame(frame)
            success = response.get('status', -1) == 0
            if not success:
                logging.info(f"PuppyBootloader: Write bed coil 0x{coil_addr:04X}={value} "
                             f"status={response.get('status', -1)}")
            return success
        except Exception as e:
            logging.info(f"PuppyBootloader: Write bed coil exception: {e}")
            return False
        finally:
            self._resume_polling_timer()

    def _clear_bed_faults(self):
        """Clear bed faults before setting temperatures (like Prusa's refresh cycle)

        Prusa's refresh() cycle does these first:
        1. write_clear_fault_status (coil 0x4000)
        2. write_reset_overcurrent (coil 0x4001)
        """
        logging.info("PuppyBootloader: Clearing bed faults...")
        # Clear system fault status
        if not self._write_bed_coil(self.BED_COIL_CLEAR_FAULT, True):
            logging.warning("PuppyBootloader: Failed to clear bed fault status")
            return False
        self.reactor.pause(self.reactor.monotonic() + 0.1)
        # Reset overcurrent fault
        if not self._write_bed_coil(self.BED_COIL_RESET_OVERCURRENT, True):
            logging.warning("PuppyBootloader: Failed to reset bed overcurrent")
            return False
        self.reactor.pause(self.reactor.monotonic() + 0.1)
        logging.info("PuppyBootloader: Bed faults cleared")
        self.bed_faults_cleared = True
        return True

    def _configure_cheese_led(self, dwarf):
        """Configure a Dwarf's cheese LED: on when selected, off when not.
        The Dwarf automatically switches based on is_selected coil."""
        value = (self.LED_SELECTED_PWM << 8) | self.LED_NOT_SELECTED_PWM
        return self._write_register(dwarf, self.LED_REG_CHEESE, value)

    def _update_all_leds(self):
        """Configure cheese LEDs on all booted Dwarfs"""
        for dwarf in self.booted_dwarfs:
            if not self._configure_cheese_led(dwarf):
                # Mark for retry during polling
                self.led_pending.add(dwarf)

    def _read_input_registers(self, dwarf, reg_start, count):
        """Read input registers from a Dwarf, return list of values or None"""
        addr = self.ADDR_MODBUS_OFFSET + dwarf  # Dwarf N at 0x1A + N
        data = [
            (reg_start >> 8) & 0xFF, reg_start & 0xFF,
            (count >> 8) & 0xFF, count & 0xFF
        ]
        frame = self._build_modbus_frame(addr, self.MODBUS_READ_INPUT, data)
        self._pause_polling_timer()
        try:
            self.reactor.pause(self.reactor.monotonic() + 0.1)
            response = self._send_frame(frame)
            status = response.get('status', -1)
            resp_data = response.get('data', b'')
            if status == 0 and resp_data and len(resp_data) >= 3 + count * 2:
                regs = []
                for i in range(count):
                    val = (resp_data[3 + i*2] << 8) | resp_data[3 + i*2 + 1]
                    regs.append(val)
                return regs
            return None
        except Exception as e:
            logging.info(f"PuppyBootloader: Read input regs exception: {e}")
            return None
        finally:
            self._resume_polling_timer()

    def _poll_dwarf_for_sensor(self, dwarf):
        """Poll a specific Dwarf and update dwarf_data with fresh _poll_time.

        This ensures the DwarfTemperatureSensor has fresh data after a tool change.
        Without this, the sensor might see stale _poll_time and return 0°C.

        Args:
            dwarf: Dwarf address (1-5)

        Returns:
            True if poll succeeded, False otherwise
        """
        addr = self.ADDR_MODBUS_OFFSET + dwarf
        reg_start = 0x8060
        count = 7
        data = [
            (reg_start >> 8) & 0xFF, reg_start & 0xFF,
            (count >> 8) & 0xFF, count & 0xFF
        ]
        frame = self._build_modbus_frame(addr, self.MODBUS_READ_INPUT, data)
        self._pause_polling_timer()
        try:
            self.reactor.pause(self.reactor.monotonic() + 0.05)
            response = self._send_frame(frame)
            status = response.get('status', -1)
            resp_data = response.get('data', b'')
            if status == 0 and resp_data and len(resp_data) >= 17:
                byte_count = resp_data[2]
                if byte_count >= 14:
                    regs = []
                    for i in range(7):
                        val = (resp_data[3 + i*2] << 8) | resp_data[3 + i*2 + 1]
                        regs.append(val)

                    # Update dwarf_data with fresh data AND _poll_time
                    self.dwarf_data[dwarf] = {
                        'fault_status': regs[0],
                        'hotend_temp': regs[1] if regs[1] < 0x8000 else regs[1] - 0x10000,
                        'heater_pwm': regs[2],
                        'filament_sensor': regs[3],
                        'board_temp': regs[4] if regs[4] < 0x8000 else regs[4] - 0x10000,
                        'mcu_temp': regs[5] if regs[5] < 0x8000 else regs[5] - 0x10000,
                        'heatbreak_temp': regs[6] if regs[6] < 0x8000 else regs[6] - 0x10000,
                        '_poll_time': self.reactor.monotonic(),  # Fresh timestamp!
                    }
                    logging.info(f"PuppyBootloader: Polled Dwarf {dwarf} for sensor: "
                               f"{self.dwarf_data[dwarf]['hotend_temp']}C")
                    return True
            logging.warning(f"PuppyBootloader: Poll for sensor failed: status={status}")
            return False
        except Exception as e:
            logging.warning(f"PuppyBootloader: Poll for sensor exception: {e}")
            return False
        finally:
            self._resume_polling_timer(0.5)

    def _read_bed_input_registers(self, reg_start, count, retries=3):
        """Read input registers from modular bed at 0x1A, return list of values or None"""
        addr = self.ADDR_MODULAR_BED  # Bed at 0x1A
        data = [
            (reg_start >> 8) & 0xFF, reg_start & 0xFF,
            (count >> 8) & 0xFF, count & 0xFF
        ]
        frame = self._build_modbus_frame(addr, self.MODBUS_READ_INPUT, data)

        for attempt in range(retries):
            # Rate limiting - wait if we sent a bed command too recently
            now = self.reactor.monotonic()
            time_since_last = now - self.bed_last_modbus_time
            if time_since_last < self.bed_modbus_min_interval:
                wait_time = self.bed_modbus_min_interval - time_since_last
                self.reactor.pause(now + wait_time)

            self._pause_polling_timer()
            try:
                self.reactor.pause(self.reactor.monotonic() + 0.1)
                self.bed_last_modbus_time = self.reactor.monotonic()
                response = self._send_frame(frame)
                status = response.get('status', -1)
                resp_data = response.get('data', b'')
                if status == 0 and resp_data and len(resp_data) >= 3 + count * 2:
                    regs = []
                    for i in range(count):
                        val = (resp_data[3 + i*2] << 8) | resp_data[3 + i*2 + 1]
                        regs.append(val)
                    return regs
                # Failed - retry
                if attempt < retries - 1:
                    self.reactor.pause(self.reactor.monotonic() + 1.0)
            except Exception as e:
                logging.info(f"PuppyBootloader: Read bed input regs exception: {e}")
                if attempt < retries - 1:
                    self.reactor.pause(self.reactor.monotonic() + 1.0)
            finally:
                self._resume_polling_timer()
        return None

    def _write_bed_holding_registers(self, reg_start, values, retries=3):
        """Write multiple holding registers to modular bed at 0x1A

        Uses MODBUS function 0x10 (Write Multiple Registers)
        Frame: [addr][0x10][reg_hi][reg_lo][count_hi][count_lo][byte_count][data...]
        """
        addr = self.ADDR_MODULAR_BED  # Bed at 0x1A
        count = len(values)
        byte_count = count * 2
        data = [
            (reg_start >> 8) & 0xFF, reg_start & 0xFF,  # Starting register
            (count >> 8) & 0xFF, count & 0xFF,          # Number of registers
            byte_count                                   # Byte count
        ]
        # Add register values (big-endian)
        for val in values:
            data.append((val >> 8) & 0xFF)
            data.append(val & 0xFF)

        frame = self._build_modbus_frame(addr, self.MODBUS_WRITE_MULTIPLE, data)

        for attempt in range(retries):
            # Rate limiting - wait if we sent a bed command too recently
            now = self.reactor.monotonic()
            time_since_last = now - self.bed_last_modbus_time
            if time_since_last < self.bed_modbus_min_interval:
                wait_time = self.bed_modbus_min_interval - time_since_last
                self.reactor.pause(now + wait_time)

            logging.info(f"PuppyBootloader: Write bed regs 0x{reg_start:04X}, count={count}, attempt={attempt+1}")
            self._pause_polling_timer()
            try:
                self.reactor.pause(self.reactor.monotonic() + 0.05)
                self.bed_last_modbus_time = self.reactor.monotonic()
                response = self._send_frame(frame)
                status = response.get('status', -1)
                resp_data = response.get('data', b'')
                logging.info(f"PuppyBootloader: Write bed response: status={status}, data={resp_data.hex() if resp_data else 'none'}")
                # Response for FC 0x10: [addr][fc][reg_hi][reg_lo][count_hi][count_lo][crc]
                if status == 0:
                    return True
                # Failed - wait before retry
                if attempt < retries - 1:
                    logging.info(f"PuppyBootloader: Bed write failed, retrying in 0.2s...")
                    self.reactor.pause(self.reactor.monotonic() + 0.2)
            except Exception as e:
                logging.info(f"PuppyBootloader: Write bed holding regs exception: {e}")
                if attempt < retries - 1:
                    self.reactor.pause(self.reactor.monotonic() + 0.2)
            finally:
                self._resume_polling_timer()

        logging.info(f"PuppyBootloader: Bed write failed after {retries} attempts")
        return False

    def _read_bedlet_temperatures(self):
        """Read measured temperatures from all 16 bedlets, returns dict or None"""
        regs = self._read_bed_input_registers(self.BED_REG_MEASURED_TEMP, self.NUM_BEDLETS)
        if regs:
            temps = {}
            for i, val in enumerate(regs):
                # Temperature in 1/10 degree units, convert to degrees
                # Handle signed values (if temp is negative, unlikely but possible)
                if val >= 0x8000:
                    val = val - 0x10000
                temps[i] = val / self.BED_TEMP_SCALE
            return temps
        return None

    def _set_bedlet_temperatures(self, targets):
        """Set target temperatures for bedlets

        Args:
            targets: dict {bedlet_index: temp_celsius} or single temp for all
        Returns:
            True on success, False on failure
        """
        # Clear faults first if not done (like Prusa's refresh cycle)
        if not self.bed_faults_cleared:
            if not self._clear_bed_faults():
                logging.warning("PuppyBootloader: Could not clear bed faults, trying temp write anyway")

        values = []
        if isinstance(targets, dict):
            # Individual bedlet targets
            for i in range(self.NUM_BEDLETS):
                temp = targets.get(i, 0)
                values.append(int(temp * self.BED_TEMP_SCALE))
        else:
            # Single temp for all bedlets
            scaled = int(targets * self.BED_TEMP_SCALE)
            values = [scaled] * self.NUM_BEDLETS

        # Check if target temps are already set - skip write if unchanged
        current_values = [int(self.bed_target_temps.get(i, 0) * self.BED_TEMP_SCALE)
                         for i in range(self.NUM_BEDLETS)]
        if values == current_values:
            logging.info("PuppyBootloader: Bed temps already at target, skipping write")
            return True

        if self._write_bed_holding_registers(self.BED_REG_TARGET_TEMP, values):
            # Update local tracking
            for i, val in enumerate(values):
                self.bed_target_temps[i] = val / self.BED_TEMP_SCALE
            return True
        return False

    # =========================================================================
    # ADAPTIVE BED HEATING (Prusa modular_bed.cpp port)
    # =========================================================================

    def _compute_enabled_mask(self, x0, y0, x1, y1):
        """Compute 16-bit bedlet enabled mask from print area bounding box.
        A bedlet is enabled if its 90x90mm rect intersects the print area.
        Matches Prusa print_area.cpp set_bounding_rect()."""
        mask = 0
        for row in range(4):
            for col in range(4):
                bx0 = col * self.BEDLET_SIZE
                by0 = row * self.BEDLET_SIZE
                bx1 = (col + 1) * self.BEDLET_SIZE
                by1 = (row + 1) * self.BEDLET_SIZE
                # Intersection check
                ix0 = max(x0, bx0)
                iy0 = max(y0, by0)
                ix1 = min(x1, bx1)
                iy1 = min(y1, by1)
                if ix1 > ix0 and iy1 > iy0:
                    phys_idx = self.BEDLET_MAP[row][col]
                    mask |= (1 << phys_idx)
        return mask

    def _touch_side(self, enabled_mask, dim, sign):
        """Calculate cost to expand enabled area to one edge.
        dim: 0=X (columns), 1=Y (rows). sign: +1 or -1.
        Matches Prusa modular_bed.cpp touch_side().
        Returns (cost, enable_mask)."""
        min_cost = 999
        min_to_enable = 0
        for col in range(4):
            for row in range(4):
                phys = self.BEDLET_MAP[row][col]
                if not (enabled_mask & (1 << phys)):
                    continue
                cost = 0
                to_enable = 0
                i = col if dim == 0 else row
                while True:
                    i += sign
                    if 0 <= i < 4:
                        cost += 1
                        r = row if dim == 0 else i
                        c = i if dim == 0 else col
                        to_enable |= (1 << self.BEDLET_MAP[r][c])
                    else:
                        break
                if cost < min_cost:
                    min_cost = cost
                    min_to_enable = to_enable
        return (min_cost, min_to_enable)

    def _expand_to_sides(self, enabled_mask, target_temp):
        """Expand enabled bedlets toward 2 nearest edges (anti-warp).
        Matches Prusa modular_bed.cpp expand_to_sides().
        Returns new enabled_mask with side bedlets added."""
        sides = [
            self._touch_side(enabled_mask, 0, 1),   # right (+X)
            self._touch_side(enabled_mask, 0, -1),  # left (-X)
            self._touch_side(enabled_mask, 1, 1),   # back (+Y)
            self._touch_side(enabled_mask, 1, -1),  # front (-Y)
        ]
        sides.sort(key=lambda x: x[0])
        # Enable toward 2 cheapest sides
        expand_mask = sides[0][1] | sides[1][1]
        return enabled_mask | expand_mask

    def _compute_gradient_temps(self, enabled_mask, target_temp):
        """Compute per-bedlet target temps with gradient for non-enabled bedlets.
        Matches Prusa modular_bed.cpp update_gradients().
        Returns dict {physical_idx: temp_celsius} for all 16 bedlets."""
        import math
        temps = {}
        # Enabled bedlets get full target
        for i in range(16):
            if enabled_mask & (1 << i):
                temps[i] = target_temp
            else:
                temps[i] = 0.0
        # Gradient: for each non-enabled bedlet, find nearest enabled
        cutoff = self.bed_gradient_cutoff
        exponent = self.bed_gradient_exponent
        for i in range(16):
            if enabled_mask & (1 << i):
                continue
            col1, row1 = self.BEDLET_GRID[i]
            for j in range(16):
                if not (enabled_mask & (1 << j)):
                    continue
                col2, row2 = self.BEDLET_GRID[j]
                dist = math.sqrt((col1 - col2)**2 + (row1 - row2)**2)
                if dist > cutoff:
                    continue
                grad_temp = target_temp - target_temp * ((dist / cutoff) ** exponent)
                if grad_temp > temps[i]:
                    temps[i] = grad_temp
        return temps

    def _apply_adaptive_heating(self, target_temp):
        """Full adaptive bed pipeline: mask -> expand -> gradient -> write.
        Called from M140/M190 when bed_print_area is set."""
        if target_temp <= 0:
            # Turning off - zero all bedlets
            self.bed_global_target = 0.0
            return self._set_bedlet_temperatures(0.0)

        self.bed_global_target = target_temp

        # Compute enabled mask from stored print area
        if self.bed_print_area is None:
            # No area set - heat all bedlets uniformly
            return self._set_bedlet_temperatures(target_temp)

        x0, y0, x1, y1 = self.bed_print_area
        enabled_mask = self._compute_enabled_mask(x0, y0, x1, y1)

        # Expand to sides (anti-warp)
        if self.bed_expand_to_sides:
            enabled_mask = self._expand_to_sides(enabled_mask, target_temp)

        self.bed_enabled_mask = enabled_mask

        # Compute gradient temps
        temps = self._compute_gradient_temps(enabled_mask, target_temp)

        # Host-side safety: clamp any temp > 120C
        for idx in temps:
            if temps[idx] > 120.0:
                logging.warning(f"PuppyBootloader: Clamping bedlet {idx} from {temps[idx]}C to 120C")
                temps[idx] = 120.0

        enabled_count = bin(enabled_mask).count('1')
        logging.info(f"PuppyBootloader: Adaptive bed: {enabled_count}/16 enabled, "
                     f"target={target_temp}C, area=({x0},{y0})-({x1},{y1})")

        return self._set_bedlet_temperatures(temps)

    def _read_discrete_inputs(self, dwarf, start_addr, count):
        """Read discrete inputs (FC 0x02) from Dwarf, returns list of booleans"""
        addr = self.ADDR_MODBUS_OFFSET + dwarf  # Dwarf N at 0x1A + N
        data = [
            (start_addr >> 8) & 0xFF, start_addr & 0xFF,
            (count >> 8) & 0xFF, count & 0xFF
        ]
        frame = self._build_modbus_frame(addr, self.MODBUS_READ_DISCRETE, data)
        self._pause_polling_timer()
        try:
            self.reactor.pause(self.reactor.monotonic() + 0.1)
            response = self._send_frame(frame)
            status = response.get('status', -1)
            resp_data = response.get('data', b'')
            # Response: [addr][fc][byte_count][data_bytes...][crc]
            if status == 0 and resp_data and len(resp_data) >= 4:
                byte_count = resp_data[2]
                bits = []
                for i in range(count):
                    byte_idx = i // 8
                    bit_idx = i % 8
                    if 3 + byte_idx < len(resp_data):
                        bits.append(bool(resp_data[3 + byte_idx] & (1 << bit_idx)))
                    else:
                        bits.append(False)
                return bits
            return None
        except Exception as e:
            logging.info(f"PuppyBootloader: Read discrete exception: {e}")
            return None
        finally:
            self._resume_polling_timer()

    def _get_dock_state(self, dwarf):
        """Read is_picked and is_parked from Dwarf, returns (picked, parked) or None"""
        result = self._read_discrete_inputs(dwarf, 0x0000, 2)
        if result and len(result) >= 2:
            return (result[0], result[1])  # (is_picked, is_parked)
        return None

    def _get_dock_x(self, tool):
        """Get dock X position for tool number (0-4)"""
        return self.dock_positions[tool]['x']

    def _get_dock_y(self, tool):
        """Get dock Y position for tool number (0-4)"""
        return self.dock_positions[tool]['y']

    def _move_xy(self, x=None, y=None, speed=None):
        """Move X and/or Y axis via G1 gcode"""
        # CRITICAL: Enforce absolute mode. If G91 is stuck (e.g. from a failed
        # macro that never ran RESTORE_GCODE_STATE), dock coordinates would be
        # interpreted as relative offsets, causing a crash.
        self.gcode.run_script_from_command("G90")
        cmd = "G1"
        if x is not None:
            cmd += f" X{x:.2f}"
        if y is not None:
            cmd += f" Y{y:.2f}"
        if speed is not None:
            cmd += f" F{speed * 60:.0f}"  # Convert mm/s to mm/min
        self.gcode.run_script_from_command(cmd)

    def _wait_for_moves(self):
        """Wait for all moves to complete"""
        self.gcode.run_script_from_command("M400")

    def _tmc_read(self, dwarf, tmc_reg):
        """Read a TMC2130 register via MODBUS (write 0xE020, read 0x8011-0x8012)"""
        # Write the TMC register address to holding register 0xE020
        if not self._write_register(dwarf, 0xE020, tmc_reg):
            return None
        # Wait for Dwarf to process SPI transaction
        self.reactor.pause(self.reactor.monotonic() + 0.05)
        # Read back the response from input registers 0x8011-0x8012
        regs = self._read_input_registers(dwarf, 0x8011, 2)
        if regs and len(regs) == 2:
            return regs[0] | (regs[1] << 16)  # low | (high << 16)
        return None

    def _tmc_write(self, dwarf, tmc_reg, value):
        """Write a TMC2130 register via MODBUS (0xE021=addr, 0xE022=val_lo, 0xE023=val_hi)"""
        # Write address first
        if not self._write_register(dwarf, 0xE021, tmc_reg):
            return False
        # Write value low word
        val_lo = value & 0xFFFF
        if not self._write_register(dwarf, 0xE022, val_lo):
            return False
        # Write value high word (this triggers the SPI write on the Dwarf)
        val_hi = (value >> 16) & 0xFFFF
        if not self._write_register(dwarf, 0xE023, val_hi):
            return False
        return True

    def cmd_EXTRUDER_ENABLE(self, gcmd):
        """Enable extruder TMC on specified or active tool"""
        tool = gcmd.get_int('T', self.active_tool)
        dwarf = tool + 1
        if dwarf not in self.booted_dwarfs:
            raise gcmd.error(f"Tool {tool} (Dwarf {dwarf}) not available")
        if self._write_coil(dwarf, 0x4000, True):
            self.tmc_enabled[dwarf] = True
            self.gcode.respond_info(f"T{tool} extruder TMC enabled")
        else:
            raise gcmd.error(f"Failed to enable TMC on Dwarf {dwarf}")

    def cmd_EXTRUDER_DISABLE(self, gcmd):
        """Disable extruder TMC on specified or active tool"""
        tool = gcmd.get_int('T', self.active_tool)
        dwarf = tool + 1
        if dwarf not in self.booted_dwarfs:
            raise gcmd.error(f"Tool {tool} (Dwarf {dwarf}) not available")
        if self._write_coil(dwarf, 0x4000, False):
            self.tmc_enabled[dwarf] = False
            self.gcode.respond_info(f"T{tool} extruder TMC disabled")
        else:
            raise gcmd.error(f"Failed to disable TMC on Dwarf {dwarf}")

    def cmd_TMC_STATUS_DWARF(self, gcmd):
        """Read key TMC2130 registers from Dwarf"""
        tool = gcmd.get_int('T', self.active_tool)
        dwarf = tool + 1
        if dwarf not in self.booted_dwarfs:
            raise gcmd.error(f"Tool {tool} (Dwarf {dwarf}) not available")

        self.gcode.respond_info(f"Reading TMC2130 status from T{tool}/Dwarf{dwarf}...")

        # Key TMC2130 registers
        reg_names = {
            0x00: "GCONF",
            0x01: "GSTAT",
            0x04: "IOIN",
            0x10: "IHOLD_IRUN",
            0x6C: "CHOPCONF",
            0x6F: "DRV_STATUS",
        }
        for reg, name in reg_names.items():
            val = self._tmc_read(dwarf, reg)
            if val is not None:
                self.gcode.respond_info(f"  {name} (0x{reg:02X}): 0x{val:08X}")
            else:
                self.gcode.respond_info(f"  {name} (0x{reg:02X}): READ FAILED")

    def cmd_TMC_WRITE_DWARF(self, gcmd):
        """Write TMC2130 register on Dwarf"""
        tool = gcmd.get_int('T', self.active_tool)
        reg = gcmd.get_int('REG', 0)
        value = gcmd.get_int('VALUE', 0)
        dwarf = tool + 1
        if dwarf not in self.booted_dwarfs:
            raise gcmd.error(f"Tool {tool} (Dwarf {dwarf}) not available")
        if self._tmc_write(dwarf, reg, value):
            self.gcode.respond_info(
                f"T{tool} TMC reg 0x{reg:02X} = 0x{value:08X}")
        else:
            raise gcmd.error(f"Failed to write TMC register on Dwarf {dwarf}")

    def cmd_DWARF_SELECT(self, gcmd):
        """Set is_selected coil on Dwarf - enables remote step/dir via localRemote MUX"""
        tool = gcmd.get_int('T', self.active_tool)
        value = gcmd.get_int('S', 1)  # 1=select (remote), 0=deselect (local)
        dwarf = tool + 1
        if dwarf not in self.booted_dwarfs:
            raise gcmd.error(f"Tool {tool} (Dwarf {dwarf}) not available")
        if self._write_coil(dwarf, 0x4001, bool(value)):
            state = "SELECTED (remote step/dir)" if value else "DESELECTED (local)"
            self.gcode.respond_info(f"T{tool} Dwarf {dwarf}: {state}")
        else:
            raise gcmd.error(f"Failed to set is_selected on Dwarf {dwarf}")

    def cmd_MSCNT_TEST(self, gcmd):
        """Read MSCNT before/after manual_stepper move to verify step pulses reach TMC"""
        tool = gcmd.get_int('T', self.active_tool)
        dwarf = tool + 1
        if dwarf not in self.booted_dwarfs:
            raise gcmd.error(f"Tool {tool} (Dwarf {dwarf}) not available")

        # Read MSCNT before
        mscnt_before = self._tmc_read(dwarf, 0x6A)
        if mscnt_before is None:
            raise gcmd.error("Failed to read MSCNT")
        self.gcode.respond_info(f"MSCNT before: {mscnt_before}")

        # Send step pulses via manual_stepper
        self.gcode.respond_info("Sending 200 steps via manual_stepper...")
        try:
            self.gcode.run_script_from_command(
                "MANUAL_STEPPER STEPPER=extruder_stepper MOVE=5 SPEED=10")
        except Exception as e:
            self.gcode.respond_info(f"manual_stepper error (may be ok): {e}")

        # Wait for move to complete
        self.reactor.pause(self.reactor.monotonic() + 1.0)

        # Read MSCNT after
        mscnt_after = self._tmc_read(dwarf, 0x6A)
        if mscnt_after is None:
            raise gcmd.error("Failed to read MSCNT after move")
        self.gcode.respond_info(f"MSCNT after: {mscnt_after}")

        if mscnt_before != mscnt_after:
            self.gcode.respond_info(
                f"SUCCESS! MSCNT changed: {mscnt_before} -> {mscnt_after}")
            self.gcode.respond_info(
                "Step pulses from XLBuddy ARE reaching Dwarf TMC2130!")
        else:
            self.gcode.respond_info(
                f"MSCNT unchanged at {mscnt_before} - steps NOT reaching TMC")

    def cmd_DOCK_STATUS(self, gcmd):
        """Read dock Hall sensors (is_picked/is_parked) from Dwarf"""
        tool = gcmd.get_int('T', -1)
        if tool >= 0:
            dwarfs = [tool + 1]
        else:
            dwarfs = self.booted_dwarfs
        for dwarf in dwarfs:
            state = self._get_dock_state(dwarf)
            if state:
                picked, parked = state
                t = dwarf - 1
                self.gcode.respond_info(
                    f"T{t}/Dwarf{dwarf}: picked={picked} parked={parked}")
            else:
                self.gcode.respond_info(
                    f"Dwarf{dwarf}: Failed to read dock state")

    def cmd_DOCK_POSITIONS(self, gcmd):
        """Show configured dock positions for all tools"""
        for i in range(5):
            pos = self.dock_positions[i]
            self.gcode.respond_info(
                f"T{i}: X={pos['x']:.1f} Y={pos['y']:.1f}")
        self.gcode.respond_info(
            f"Safe Y with tool: {self.SAFE_Y_WITH_TOOL}")
        self.gcode.respond_info(
            f"Safe Y without tool: {self.SAFE_Y_WITHOUT_TOOL}")

    def cmd_DOCK_CALIBRATE(self, gcmd):
        """Calibrate dock position - tool must be manually attached to carriage.
        Usage: DOCK_CALIBRATE T=<tool> (then home X/Y to establish position)"""
        tool = gcmd.get_int('T', 0)
        dwarf = tool + 1
        if dwarf not in self.booted_dwarfs:
            raise gcmd.error(f"Tool {tool} (Dwarf {dwarf}) not available")

        # Check that tool is actually picked
        state = self._get_dock_state(dwarf)
        if not state:
            raise gcmd.error("Failed to read dock state")
        picked, parked = state
        if not picked:
            raise gcmd.error(
                f"T{tool} is not attached to carriage (is_picked=False). "
                "Manually attach it first!")

        self.gcode.respond_info(
            f"T{tool} detected on carriage. Undocking and homing...")

        # Enable TMC FIRST before setting tool_picked (safety fix)
        tmc_enabled = False
        for attempt in range(3):
            if self._write_coil(dwarf, 0x4000, True):
                tmc_enabled = True
                self.tmc_enabled[dwarf] = True
                break
            self.reactor.pause(self.reactor.monotonic() + 0.1)
        if not tmc_enabled:
            raise gcmd.error(f"Failed to enable TMC on T{tool} - cannot proceed")

        # Set is_selected so Dwarf knows it's active
        self._write_coil(dwarf, 0x4001, True)
        self.active_tool = tool
        self.tool_picked = True

        # Move Y forward to safe position (undock from dock)
        self._move_xy(y=self.SAFE_Y_WITH_TOOL, speed=self.SLOW_SPEED)
        self._wait_for_moves()

        # Verify we're no longer parked
        state = self._get_dock_state(dwarf)
        if state:
            picked, parked = state
            if parked:
                raise gcmd.error("Tool still reports parked after undock move!")
            self.gcode.respond_info(
                f"Undocked: picked={picked} parked={parked}")

        # Home X and Y
        self.gcode.respond_info("Homing X and Y...")
        self.gcode.run_script_from_command("G28 X Y")
        self._wait_for_moves()

        # Now we know absolute position. The dock is at:
        # X = current position (we moved straight Y, so X didn't change)
        # Y = DOCK_Y (455mm from home)
        # But since we moved Y to SAFE_Y_WITH_TOOL, the dock Y is behind us
        # The dock X is wherever the tool was when attached
        # For stock Prusa XL, use the default positions
        dock_x = self._get_dock_x(tool)
        dock_y = self._get_dock_y(tool)
        self.gcode.respond_info(
            f"T{tool} dock position: X={dock_x:.1f} Y={dock_y:.1f}")
        self.gcode.respond_info(
            "Using default Prusa XL dock positions. "
            "Tool is now active on carriage.")


    def cmd_SET_TOOL_STATE(self, gcmd):
        """Force the internal tool state. Use after manual tool changes.
        Usage: SET_TOOL_STATE T=<tool> PICKED=<0|1>"""
        tool = gcmd.get_int('T', 0)
        picked = gcmd.get_int('PICKED', 1)
        if picked:
            dwarf = tool + 1
            # Try to enable TMC when manually setting picked state
            if dwarf in self.booted_dwarfs:
                if self._write_coil(dwarf, 0x4000, True):
                    self.tmc_enabled[dwarf] = True
                    self._write_coil(dwarf, 0x4001, True)
                    self.gcode.respond_info(f"Tool state set: T{tool} picked, TMC enabled")
                else:
                    self._write_coil(dwarf, 0x4001, True)
                    self.gcode.respond_info(
                        f"WARNING: T{tool} state set but TMC enable failed - "
                        f"extruder motion may not work!")
            else:
                self.gcode.respond_info(
                    f"WARNING: T{tool} state set but Dwarf {dwarf} not booted")
            self.active_tool = tool
            self.tool_picked = True
        else:
            self.active_tool = -1
            self.tool_picked = False
            self.gcode.respond_info("Tool state set: no tool picked")

    def cmd_GET_TOOL_OFFSETS(self, gcmd):
        """Display current tool offsets.
        Usage: GET_TOOL_OFFSETS"""
        gcmd.respond_info("=== TOOL OFFSETS ===")
        gcmd.respond_info("File: %s" % self.tool_offsets_file)
        for tool in sorted(self.tool_offsets.keys()):
            x, y, z = self.tool_offsets[tool]
            gcmd.respond_info("T%d: X=%.4f Y=%.4f Z=%.4f" % (tool, x, y, z))

    # NOTE: cmd_SAVE_TOOL_OFFSETS removed - CALIBRATE_TOOL_OFFSETS auto-saves

    # NOTE: Z offset commands removed - not needed with Prusa-style loadcell probing
    # The loadcell probes WITH the nozzle, so Z=0 = nozzle at bed surface
    # Tool offsets from CALIBRATE_TOOL_OFFSETS handle all tool-to-tool differences
    # This matches Prusa XL - no Live-Z, no SAVE_Z_OFFSET, no APPLY_Z_OFFSET needed

    # ==================== LOADCELL COMMANDS ====================

    def _read_loadcell_raw(self, dwarf):
        """Read loadcell via dedicated MCU command that parses FIFO internally.
        Returns (sample_count, raw_value) or (0, 0) on failure."""
        if not self.loadcell_cmd:
            logging.info("PuppyBootloader: loadcell_cmd not available")
            return (0, 0)
        addr = self.ADDR_MODBUS_OFFSET + dwarf  # Dwarf N at 0x1A + N
        # Pause polling to avoid bus collisions
        self._pause_polling_timer()
        try:
            self.reactor.pause(self.reactor.monotonic() + 0.02)
            response = self.loadcell_cmd.send([addr])
            status = response.get('status', -1)
            count = response.get('count', 0)
            raw = response.get('raw', 0)
            rlen = response.get('rlen', 0)
            if status != 0 or count == 0:
                logging.info(f"PuppyBootloader: Loadcell read: status={status}, "
                             f"count={count}, raw={raw}, rlen={rlen}")
                return (0, 0)
            # Sign-extend 24-bit ADC to 32-bit
            if raw & 0x800000:
                raw = raw - 0x1000000
            return (count, raw)
        except Exception as e:
            logging.info(f"PuppyBootloader: Loadcell read error: {e}")
            return (0, 0)
        finally:
            self._resume_polling_timer(0.5)

    def _get_loadcell_grams(self, dwarf):
        """Read loadcell and return value in grams (tared).
        Returns None on failure."""
        count, raw = self._read_loadcell_raw(dwarf)
        if count == 0:
            return None
        self.loadcell_last_raw[dwarf] = raw
        offset = self.loadcell_offset.get(dwarf, 0)
        return (raw - offset) * self.LOADCELL_SCALE

    def _read_loadcell_threadsafe(self, dwarf):
        """Thread-safe loadcell read using time.sleep instead of reactor.pause.
        Returns grams (tared) or None on failure."""
        if not self.loadcell_cmd:
            return None
        addr = self.ADDR_MODBUS_OFFSET + dwarf  # Dwarf N at 0x1A + N
        try:
            # Small delay before read (thread-safe)
            time.sleep(0.015)
            response = self.loadcell_cmd.send([addr])
            status = response.get('status', -1)
            count = response.get('count', 0)
            raw = response.get('raw', 0)
            if status != 0 or count == 0:
                return None
            # Sign-extend 24-bit ADC to 32-bit
            if raw & 0x800000:
                raw = raw - 0x1000000
            offset = self.loadcell_offset.get(dwarf, 0)
            return (raw - offset) * self.LOADCELL_SCALE
        except Exception as e:
            logging.debug(f"Threadsafe loadcell read error: {e}")
            return None

    def cmd_LOADCELL_ENABLE(self, gcmd):
        """Enable loadcell on a tool.
        Usage: LOADCELL_ENABLE [T=<tool>]"""
        tool = gcmd.get_int('T', self.active_tool)
        dwarf = tool + 1
        if dwarf not in self.booted_dwarfs:
            raise gcmd.error(f"Tool {tool} (Dwarf {dwarf}) not available")
        # Disable accelerometer first (mutually exclusive)
        self._write_coil(dwarf, 0x4003, False)
        self.reactor.pause(self.reactor.monotonic() + 0.1)
        # Enable loadcell
        self._write_coil(dwarf, self.LOADCELL_COIL, True)
        self.loadcell_enabled[dwarf] = True
        self.reactor.pause(self.reactor.monotonic() + 0.5)
        self.gcode.respond_info(f"Loadcell enabled on T{tool} (Dwarf {dwarf})")

    def cmd_LOADCELL_DISABLE(self, gcmd):
        """Disable loadcell on a tool.
        Usage: LOADCELL_DISABLE [T=<tool>]"""
        tool = gcmd.get_int('T', self.active_tool)
        dwarf = tool + 1
        if dwarf not in self.booted_dwarfs:
            raise gcmd.error(f"Tool {tool} (Dwarf {dwarf}) not available")
        self._write_coil(dwarf, self.LOADCELL_COIL, False)
        self.loadcell_enabled[dwarf] = False
        self.gcode.respond_info(f"Loadcell disabled on T{tool}")

    def cmd_LOADCELL_READ(self, gcmd):
        """Read current loadcell value.
        Usage: LOADCELL_READ [T=<tool>] [COUNT=<reads>]"""
        tool = gcmd.get_int('T', self.active_tool)
        dwarf = tool + 1
        count = gcmd.get_int('COUNT', 1)
        if dwarf not in self.booted_dwarfs:
            raise gcmd.error(f"Tool {tool} (Dwarf {dwarf}) not available")
        if not self.loadcell_enabled.get(dwarf, False):
            raise gcmd.error(f"Loadcell not enabled on T{tool}. Run LOADCELL_ENABLE first.")

        total_samples = 0
        last_raw = 0
        for _ in range(count):
            sample_count, raw = self._read_loadcell_raw(dwarf)
            if sample_count > 0:
                total_samples += sample_count
                last_raw = raw
            if count > 1:
                self.reactor.pause(self.reactor.monotonic() + 0.05)

        if total_samples == 0:
            self.gcode.respond_info("No loadcell samples received")
            return

        offset = self.loadcell_offset.get(dwarf, 0)
        grams = (last_raw - offset) * self.LOADCELL_SCALE
        self.gcode.respond_info(
            f"T{tool} Loadcell: {grams:.1f}g (raw={last_raw}, "
            f"offset={offset}, samples={total_samples})")

    def cmd_LOADCELL_TARE(self, gcmd):
        """Tare (zero) the loadcell.
        Usage: LOADCELL_TARE [T=<tool>] [SAMPLES=<n>]"""
        tool = gcmd.get_int('T', self.active_tool)
        dwarf = tool + 1
        num_samples = gcmd.get_int('SAMPLES', self.LOADCELL_TARE_SAMPLES)
        if dwarf not in self.booted_dwarfs:
            raise gcmd.error(f"Tool {tool} (Dwarf {dwarf}) not available")
        if not self.loadcell_enabled.get(dwarf, False):
            raise gcmd.error(f"Loadcell not enabled on T{tool}. Run LOADCELL_ENABLE first.")

        self.gcode.respond_info(
            f"Taring T{tool} loadcell ({num_samples} samples)...")

        all_samples = []
        attempts = 0
        while len(all_samples) < num_samples and attempts < num_samples * 2:
            count, raw = self._read_loadcell_raw(dwarf)
            if count > 0:
                all_samples.append(raw)
            self.reactor.pause(self.reactor.monotonic() + 0.02)
            attempts += 1

        if len(all_samples) < 5:
            raise gcmd.error(
                f"Failed to collect enough samples for tare "
                f"(got {len(all_samples)}/{num_samples})")

        # Average for offset
        offset = sum(all_samples[:num_samples]) / min(len(all_samples), num_samples)
        self.loadcell_offset[dwarf] = int(offset)
        self.gcode.respond_info(
            f"T{tool} tared: offset={int(offset)} "
            f"({len(all_samples)} samples collected)")

    def cmd_LOADCELL_PROBE(self, gcmd):
        """Z probe using loadcell with TRUE continuous motion via trsync.

        Uses Klipper's homing infrastructure with trsync for instant MCU-level
        motion stop. The MCU polls the loadcell and triggers trsync when the
        force threshold is exceeded - motion stops immediately without any
        host involvement or delay.

        Usage: LOADCELL_PROBE [THRESHOLD=<g>] [MAX_Z=<mm>] [SPEED=<mm/s>]
                              [SLOW_SPEED=<mm/s>] [BACKOFF=<mm>]"""
        tool = self.active_tool if self.tool_picked else 0
        dwarf = tool + 1
        modbus_addr = 0x1B + tool  # Dwarf address (T0=0x1B, T1=0x1C, etc.)

        # Parameters - use LoadcellProbe configured thresholds as defaults (Prusa 125g)
        # Can be overridden via command parameters
        default_fast = self.loadcell_probe._z_fast_threshold if self.loadcell_probe else 125.0
        default_slow = self.loadcell_probe._z_slow_threshold if self.loadcell_probe else 125.0
        fast_threshold = gcmd.get_float('FAST_THRESHOLD', default_fast)
        threshold = gcmd.get_float('THRESHOLD', default_slow)
        # Use position_min as travel limit (not 0.0) - critical for negative endstops
        z_min = self.loadcell_probe._get_z_min_position() if self.loadcell_probe else -10.0
        max_travel = gcmd.get_float('MAX_Z', 370.0)
        speed = gcmd.get_float('SPEED', 5.0)
        slow_speed = gcmd.get_float('SLOW_SPEED', 0.5)  # Very slow for gentle second probe
        backoff = gcmd.get_float('BACKOFF', 2.0)

        if dwarf not in self.booted_dwarfs:
            raise gcmd.error(f"Dwarf {dwarf} not available")

        # Check loadcell endstop home command is available (indicates trsync support in firmware)
        if self.loadcell_home_cmd is None:
            raise gcmd.error("Loadcell endstop not available - check firmware for trsync support")

        self.gcode.respond_info(f"LOADCELL_PROBE: trsync-based continuous motion")
        self.gcode.respond_info(f"  fast={fast_threshold}g, slow={threshold}g, speed={speed}mm/s")

        toolhead = self.printer.lookup_object('toolhead')
        phoming = self.printer.lookup_object('homing')
        toolhead.wait_moves()

        # Enable loadcell on Dwarf
        if not self.loadcell_enabled.get(dwarf, False):
            self.gcode.respond_info("Enabling loadcell...")
            self._write_coil(dwarf, 0x4003, False)  # Disable accelerometer
            self.reactor.pause(self.reactor.monotonic() + 0.1)
            self._write_coil(dwarf, self.LOADCELL_COIL, True)
            self.loadcell_enabled[dwarf] = True
            self.reactor.pause(self.reactor.monotonic() + 0.3)

        # Tare the loadcell before probing
        self.gcode.respond_info("Taring loadcell...")
        if self.tare_mcu_cmd is not None:
            result = self.tare_mcu_cmd.send([modbus_addr, 48])
            offset = result.get('offset', 0)
            self.gcode.respond_info(f"  Tare offset: {offset} ({offset * self.LOADCELL_SCALE:.1f}g)")
        # Settle delay after tare to let any vibrations damp
        self.reactor.pause(self.reactor.monotonic() + 0.5)

        def do_probe(probe_speed, phase_name, probe_threshold):
            """Execute a probing move using Klipper's homing infrastructure."""
            # Configure threshold for this probe phase
            threshold_raw = -int(probe_threshold / self.LOADCELL_SCALE)
            self.gcode.respond_info(f"  Threshold: {probe_threshold}g (raw={threshold_raw})")
            self.loadcell_endstop.set_probe_params(modbus_addr, threshold_raw, xy_mode=0)

            start_pos = toolhead.get_position()
            start_z = start_pos[2]
            target_z = max(start_z - max_travel, z_min)  # Use position_min, not 0.0

            self.gcode.respond_info(f"=== {phase_name}: Z{start_z:.2f} -> Z{target_z:.2f} @ {probe_speed}mm/s ===")

            # Build target position (only Z moves)
            pos = list(start_pos)
            pos[2] = target_z

            # Use Klipper's probing_move - this is TRUE continuous motion
            # The loadcell_endstop will call trsync_do_trigger() on the MCU
            # when threshold is exceeded, instantly stopping motion
            try:
                epos = phoming.probing_move(self.loadcell_endstop, pos, probe_speed)
                # epos is the position where trigger occurred
                trigger_z = epos[2]
                self.gcode.respond_info(f"  CONTACT at Z={trigger_z:.4f}")
                return (True, trigger_z)
            except self.printer.command_error as e:
                if "prior to movement" in str(e):
                    raise gcmd.error("Loadcell triggered before probe started - check tare/threshold")
                raise

        # === PHASE 1: Fast approach (60g threshold) ===
        self.gcode.respond_info("=== PHASE 1: Fast approach ===")
        triggered, z1 = do_probe(speed, "FAST", fast_threshold)

        if not triggered:
            self.gcode.respond_info(f"PROBE FAILED - no contact in {max_travel}mm travel")
            self._write_coil(dwarf, self.LOADCELL_COIL, False)
            self.loadcell_enabled[dwarf] = False
            return

        # Back off for slow probe
        self.gcode.respond_info(f"Backing off {backoff}mm...")
        pos = toolhead.get_position()
        pos[2] = pos[2] + backoff
        toolhead.manual_move(pos, speed)
        toolhead.wait_moves()

        # Re-tare before slow probe
        self.gcode.respond_info("Re-taring for slow probe...")
        if self.tare_mcu_cmd is not None:
            self.tare_mcu_cmd.send([modbus_addr, 48])
        self.reactor.pause(self.reactor.monotonic() + 0.3)

        # === PHASE 2: Slow approach for accuracy (40g threshold) ===
        self.gcode.respond_info("=== PHASE 2: Slow approach ===")
        triggered, z2 = do_probe(slow_speed, "SLOW", threshold)

        if triggered:
            self.gcode.respond_info(f"PROBE COMPLETE: Z={z2:.4f}")
            # Store result for use by other systems
            self.last_probe_z = z2
        else:
            self.gcode.respond_info("PROBE FAILED on slow approach")

        # Disable loadcell
        self._write_coil(dwarf, self.LOADCELL_COIL, False)
        self.loadcell_enabled[dwarf] = False

    def cmd_Z_CALIBRATION(self, gcmd):
        """Z Calibration - align Z axis by crashing into top frame.

        Uses FORCE_MOVE to bypass position limits and crash the bed into the
        top frame. Motors skip steps at reduced current until aligned.
        This is the Prusa G162 approach.

        Usage: Z_CALIBRATION [SPEED=15] [CURRENT=0.35] [DISTANCE=400]

        Parameters:
            SPEED: Movement speed in mm/s (default 15, same as Prusa)
            CURRENT: Motor current during crash (default 0.35A, allows step skipping)
            DISTANCE: How far to command the crash move in mm (default 400)
        """
        speed = gcmd.get_float('SPEED', 15.0)  # mm/s - Prusa uses 15
        crash_current = gcmd.get_float('CURRENT', 0.350)  # Low current for step skipping
        crash_distance = gcmd.get_float('DISTANCE', 400.0)  # mm to move (will hit frame first)

        toolhead = self.printer.lookup_object('toolhead')

        self.gcode.respond_info("=== Z CALIBRATION (Prusa Method) ===")
        self.gcode.respond_info(f"Speed: {speed}mm/s, Current: {crash_current}A, Distance: {crash_distance}mm")

        # Step 1: Park tool if one is picked (nozzle must be out of the way)
        if self.tool_picked:
            self.gcode.respond_info(f"Parking T{self.active_tool} first...")
            self.gcode.run_script_from_command("TOOL_PARK")
            toolhead.wait_moves()

        # Step 2: Move carriage to safe position (corner, away from bed center)
        self.gcode.respond_info("Moving carriage to safe position...")
        self.gcode.run_script_from_command("G90")
        self.gcode.run_script_from_command("G1 X5 Y350 F6000")
        toolhead.wait_moves()

        # Step 3: Get normal current for later restoration
        z_config = self.printer.lookup_object('configfile').read_main_config()
        try:
            normal_current = z_config.getsection('tmc2130 stepper_z').getfloat('run_current', 0.7)
        except Exception:
            normal_current = 0.7

        # Step 4: Reduce Z motor current for step skipping
        self.gcode.respond_info(f"Reducing Z current to {crash_current}A for crash...")
        self.gcode.run_script_from_command(f"SET_TMC_CURRENT STEPPER=stepper_z CURRENT={crash_current}")

        # Brief pause for current to stabilize
        self.reactor.pause(self.reactor.monotonic() + 0.2)

        # Step 5: RAM Z into top frame using FORCE_MOVE (bypasses position limits)
        # Negative distance = bed moves UP toward frame
        # On Prusa XL: Z decrease = bed UP
        self.gcode.respond_info("Ramming Z into top frame (motors will skip steps)...")
        self.gcode.run_script_from_command(
            f"FORCE_MOVE STEPPER=stepper_z DISTANCE=-{crash_distance} VELOCITY={speed}"
        )
        toolhead.wait_moves()

        # Brief pause to ensure motors have fully stalled/skipped
        self.reactor.pause(self.reactor.monotonic() + 0.5)

        # Step 6: Restore normal current
        self.gcode.respond_info(f"Restoring Z current to {normal_current}A...")
        self.gcode.run_script_from_command(f"SET_TMC_CURRENT STEPPER=stepper_z CURRENT={normal_current}")

        # Step 7: Set Z position at the top (we're now at mechanical limit)
        self.gcode.run_script_from_command("SET_KINEMATIC_POSITION Z=0")

        # Step 8: Back off from top frame - move bed DOWN (Z increase)
        self.gcode.respond_info("Backing off from top frame...")
        self.gcode.run_script_from_command("G1 Z10 F600")
        toolhead.wait_moves()

        # Step 9: Set final position
        self.gcode.run_script_from_command("SET_KINEMATIC_POSITION Z=10")

        self.gcode.respond_info("=== Z CALIBRATION COMPLETE ===")
        self.gcode.respond_info("Z is now aligned. Home Z (G28 Z) for precise position.")

    def cmd_TOOL_PICK(self, gcmd):
        """Pick up a tool from its dock.
        Usage: TOOL_PICK T=<tool>"""
        tool = gcmd.get_int('T', 0)
        dwarf = tool + 1
        if dwarf not in self.booted_dwarfs:
            raise gcmd.error(f"Tool {tool} (Dwarf {dwarf}) not available")
        if self.tool_picked:
            raise gcmd.error(
                f"Tool T{self.active_tool} is already picked! Park it first.")

        # SAFETY: Physical Hall sensor check - verify no tool on carriage
        # This catches cases where software state is wrong (e.g., after restart)
        for check_dwarf in self.booted_dwarfs:
            state = self._get_dock_state(check_dwarf)
            if state:
                is_picked, is_parked = state
                if is_picked:
                    check_tool = check_dwarf - 1
                    # Update software state to match physical reality
                    self.tool_picked = True
                    self.active_tool = check_tool
                    raise gcmd.error(
                        f"SAFETY: Hall sensor detects T{check_tool} is physically picked! "
                        f"Park it first or verify correct tool state.")

        # Auto-home if needed before tool change
        toolhead = self.printer.lookup_object('toolhead')
        curtime = self.reactor.monotonic()
        kin_status = toolhead.get_status(curtime)
        homed = kin_status.get('homed_axes', '')
        if 'x' not in homed or 'y' not in homed or 'z' not in homed:
            self.gcode.respond_info("Axes not homed - homing all first...")
            self.gcode.run_script_from_command("G28")

        # ============================================================
        # PRUSA-STYLE Z OFFSET COMPENSATION (from toolchanger.cpp)
        # ============================================================
        # Calculate the Z offset difference BEFORE starting the pick sequence.
        # If the new tool has a lower nozzle (larger Z offset), we must raise Z
        # BEFORE approaching the dock to prevent crashing.
        #
        # Prusa stores this as hotend_currently_applied_offset and applies
        # physical Z movement during tool change, not just coordinate shifts.
        # ============================================================

        # Get old and new tool offsets
        old_offset = self.applied_tool_offset
        new_offset = self.tool_offsets.get(tool, (0.0, 0.0, 0.0))

        # Calculate Z nozzle extension difference between tools
        # In our stored convention (Prusa/Marlin): negative Z offset = longer nozzle
        # (because offset = ref_center - tool_center, and longer nozzle probes at higher Z)
        # For safety clearance, we need to know if new nozzle extends further down
        # Negate to get "how much longer": positive = new nozzle extends further
        nozzle_z_diff = -(new_offset[2] - old_offset[2])

        self.gcode.respond_info(
            f"Tool offset: old=({old_offset[0]:.3f}, {old_offset[1]:.3f}, {old_offset[2]:.3f}) "
            f"new=({new_offset[0]:.3f}, {new_offset[1]:.3f}, {new_offset[2]:.3f}) "
            f"nozzle_extension_diff={nozzle_z_diff:.3f}mm")

        # Z safety check - ensure bed is at safe height before tool change
        cur_pos = toolhead.get_position()
        safe_z = 20.0

        # PRUSA LOGIC: If new tool nozzle is LONGER (positive nozzle_z_diff), we need
        # MORE clearance. Raise Z by the difference BEFORE approaching dock.
        if nozzle_z_diff > 0.001:
            # New tool extends further down - raise Z by the difference
            extra_clearance = nozzle_z_diff
            adjusted_safe_z = safe_z + extra_clearance
            self.gcode.respond_info(
                f"New tool nozzle is {nozzle_z_diff:.3f}mm longer - raising Z by {extra_clearance:.3f}mm "
                f"BEFORE pick (safe_z={adjusted_safe_z:.1f})")
            if cur_pos[2] < adjusted_safe_z:
                self.gcode.run_script_from_command(f"G1 Z{adjusted_safe_z:.1f} F600")
                toolhead.wait_moves()
        elif cur_pos[2] < safe_z:
            self.gcode.respond_info(f"Moving Z from {cur_pos[2]:.1f} to {safe_z} for safe tool change")
            self.gcode.run_script_from_command(f"G1 Z{safe_z} F600")
            toolhead.wait_moves()

        dock_x = self._get_dock_x(tool)
        dock_y = self._get_dock_y(tool)
        self.gcode.respond_info(
            f"Picking T{tool} from dock X={dock_x:.1f} Y={dock_y:.1f}")

        # Verify tool is parked
        state = self._get_dock_state(dwarf)
        if state:
            picked, parked = state
            if not parked:
                self.gcode.respond_info(
                    f"WARNING: T{tool} not reporting parked (parked={parked})")

        # Phase 1: Move to dock X, safe Y approach
        self._move_xy(x=dock_x, y=self.SAFE_Y_WITHOUT_TOOL,
                      speed=self.TRAVEL_SPEED)
        self._wait_for_moves()

        # Phase 2: Approach dock - stop before contact
        self._move_xy(y=dock_y + self.PICK_Y_OFFSET, speed=self.TRAVEL_SPEED)
        self._wait_for_moves()

        # Phase 3: Slow insert to dock position
        self._move_xy(y=dock_y, speed=self.SLOW_SPEED)
        self._wait_for_moves()

        # Wait for is_picked
        self.reactor.pause(self.reactor.monotonic() + 0.3)
        state = self._get_dock_state(dwarf)
        picked = False
        if state:
            picked, parked = state
        if not picked:
            # Wiggle - push DEEPER into dock (Prusa: dock_y + DOCK_WIGGLE_OFFSET)
            self.gcode.respond_info("Not picked, wiggling...")
            self._move_xy(y=dock_y + self.DOCK_WIGGLE, speed=self.SLOW_SPEED)
            self._wait_for_moves()
            self._move_xy(y=dock_y, speed=self.SLOW_SPEED)
            self._wait_for_moves()
            self.reactor.pause(self.reactor.monotonic() + 0.3)
            state = self._get_dock_state(dwarf)
            if state:
                picked, parked = state
            if not picked:
                raise gcmd.error(
                    f"T{tool} not detected as picked after wiggle!")

        self.gcode.respond_info(f"T{tool} picked detected")

        # Phase 4: Lock sequence (X offsets with higher current)
        # Increase X/Y stepper current for locking force
        self.gcode.run_script_from_command(
            f"SET_TMC_CURRENT STEPPER=stepper_x CURRENT="
            f"{self.PARKING_CURRENT / 1000:.3f}")
        self.gcode.run_script_from_command(
            f"SET_TMC_CURRENT STEPPER=stepper_y CURRENT="
            f"{self.PARKING_CURRENT / 1000:.3f}")

        # Lock engage
        self._move_xy(x=dock_x + self.PICK_X_OFFSET_1, speed=self.SLOW_SPEED)
        self._wait_for_moves()
        self._move_xy(x=dock_x + self.PICK_X_OFFSET_2, speed=self.SLOW_SPEED)
        self._wait_for_moves()
        self._move_xy(x=dock_x + self.PICK_X_OFFSET_3, speed=self.SLOW_SPEED)
        self._wait_for_moves()

        # Restore normal current
        self.gcode.run_script_from_command(
            f"SET_TMC_CURRENT STEPPER=stepper_x CURRENT="
            f"{self.NORMAL_CURRENT / 1000:.3f}")
        self.gcode.run_script_from_command(
            f"SET_TMC_CURRENT STEPPER=stepper_y CURRENT="
            f"{self.NORMAL_CURRENT / 1000:.3f}")

        # Verify not parked anymore (tool extracted from dock magnet)
        self.reactor.pause(self.reactor.monotonic() + 0.3)
        state = self._get_dock_state(dwarf)
        if state:
            picked, parked = state
            if parked:
                self.gcode.respond_info(
                    "WARNING: Still reporting parked after lock!")

        # Phase 5: Extract - move to safe Y
        self._move_xy(y=self.SAFE_Y_WITH_TOOL, speed=self.TRAVEL_SPEED)
        self._wait_for_moves()

        # Set tool as selected (but NOT active/picked yet - wait for TMC enable)
        self._write_coil(dwarf, 0x4001, True)  # is_selected
        old_tool = self.active_tool

        # Enable extruder TMC with verification (Phase 2)
        # CRITICAL: Do NOT set tool_picked=True until TMC is verified enabled
        tmc_enabled = False
        for attempt in range(3):
            if self._write_coil(dwarf, 0x4000, True):
                tmc_enabled = True
                self.tmc_enabled[dwarf] = True
                break
            self.reactor.pause(self.reactor.monotonic() + 0.1)

        if not tmc_enabled:
            # Rollback: deselect the tool since TMC failed
            self._write_coil(dwarf, 0x4001, False)  # Deselect
            logging.error(f"PuppyBootloader: Failed to enable TMC on Dwarf {dwarf} after 3 attempts")
            raise gcmd.error(f"CRITICAL: TMC enable failed on T{tool} - extruder motion unsafe")

        # TMC enabled successfully - NOW set tool as active/picked
        #
        # CRITICAL: Tool change heater safety handling
        # ============================================
        # When we change active_tool, the DwarfTemperatureSensor immediately starts
        # reporting the NEW tool's temperature. If the old tool was hot (200C) and
        # the new tool is cold (25C), verify_heater sees a sudden 175C drop and
        # triggers "heater not heating at expected rate" -> safety shutdown.
        #
        # The fix: Temporarily set heater target to 0 BEFORE changing active_tool.
        # This resets verify_heater's state (error accumulator, approaching_target, etc).
        # Then after the switch, restore the desired target. verify_heater treats this
        # as a fresh heating cycle starting from the new tool's actual temperature.
        #
        # This maintains thermal runaway protection while handling tool changes correctly.
        #
        saved_target = 0
        if self._extruder_heater is not None:
            saved_target = self._extruder_heater.target_temp
            if saved_target > 0:
                # Set target to 0 to reset verify_heater state
                # This prevents false "not heating" errors during tool switch
                self._extruder_heater.set_temp(0.)
                logging.info(f"PuppyBootloader: Tool change - temporarily disabled heater "
                           f"(was {saved_target}C) to reset verify_heater state")

        # NOW change the active tool - sensor will start reporting new tool's temp
        self.active_tool = tool
        self.tool_picked = True
        self.gcode.respond_info(f"T{tool} extruder TMC enabled")

        # Wait for MODBUS to complete and sensor to pick up new tool's temperature
        self.reactor.pause(self.reactor.monotonic() + 0.2)

        # MULTI-TOOL FIX: Use the NEW tool's stored temp, not transfer from old tool
        # Each tool has its own temp set by M104 Tn Sxxx from the slicer
        new_tool_target = self.target_temps.get(dwarf, 0)
        if new_tool_target > 0:
            # New tool already has a temp set - use it
            if self._write_register(dwarf, 0xE000, int(new_tool_target)):
                self.gcode.respond_info(f"T{tool} MODBUS heater target: {new_tool_target}C (stored)")
        # No fallback temp transfer - slicer manages all tool temperatures
        # via M104 Tn Sxxx commands (matches Prusa behavior)

        # ============================================================
        # PRUSA-STYLE OFFSET APPLICATION (from toolchanger.cpp)
        # ============================================================
        # Apply tool offsets via SET_GCODE_OFFSET (negated for Klipper).
        # Prusa/Marlin: hotend_offset shifts current_position (one-time adjustment)
        # Klipper: SET_GCODE_OFFSET persistently shifts all future gcode positions
        # These have OPPOSITE effects, so we negate the stored offset when applying.
        # Example: T3 stored Z=-0.3848 (longer nozzle) → applied as Z=+0.3848
        # G1 Z0.3 → actual Z = 0.3 + 0.3848 = 0.6848 (bed lower = correct gap)
        # ============================================================

        # Get the new tool offset (already calculated earlier, but get fresh)
        new_offset = self.tool_offsets.get(tool, (0.0, 0.0, 0.0))
        ox, oy, oz = new_offset

        # Apply coordinate offset via SET_GCODE_OFFSET - NEGATED for Klipper
        # Prusa stores offset = ref_center - tool_center (Marlin convention)
        # Marlin applies by shifting current_position += offset (one-time shift)
        # Klipper's SET_GCODE_OFFSET persistently adds to ALL gcode positions
        # These have opposite effects, so we negate when applying to Klipper
        #
        # Per-tool z_offset from [tool_offsets] is ADDED directly (slicer convention,
        # positive = up, no negation needed). This is separate from the calibrated
        # offset and handles per-tool first-layer fine-tuning.
        tool_z_adj = 0.0
        tool_offsets_mod = self.printer.lookup_object('tool_offsets', None)
        if tool_offsets_mod is not None:
            tool_z_adj = tool_offsets_mod.get_z_offset(tool)
        total_z = -oz + tool_z_adj
        self.gcode.run_script_from_command(
            f"SET_GCODE_OFFSET X={-ox:.4f} Y={-oy:.4f} Z={total_z:.4f} MOVE=0")

        # Update the applied offset tracking (Prusa's hotend_currently_applied_offset)
        self.applied_tool_offset = new_offset

        # Set Klipper's [extruder] heater target to NEW tool's temp
        # This allows extrusion and shows correct state in Mainsail.
        # The heater target was reset to 0 earlier to reset verify_heater state.
        if self._extruder_heater is not None and new_tool_target > 0:
            self._extruder_heater.set_temp(float(new_tool_target))
            logging.info(f"PuppyBootloader: Tool pick - set Klipper heater target "
                       f"to {new_tool_target}C for T{tool}")

        # CRITICAL: Poll the new tool immediately to update dwarf_data with fresh _poll_time
        # This ensures DwarfTemperatureSensor has fresh data and won't return 0°C due to staleness.
        # Without this, after multiple tool changes, the poll timing can drift and cause
        # the sensor to see stale data, triggering "Extrude below minimum temp" errors.
        self._poll_dwarf_for_sensor(dwarf)

        # NOTE: NO prime here - Prusa doesn't prime in pickup()!
        # The slicer handles priming at the wipe tower via generated G-code.
        # Adding prime here caused "Extrude below minimum temp" errors.

        # Wait for tool to reach its TARGET temp if it's cold
        # This is required for: first tool change, cold tools, or after standby cooling
        # Only wait if tool is below target (saves time if already hot)
        if new_tool_target > 0:
            current_temp = self.dwarf_data.get(dwarf, {}).get('hotend_temp', 0)
            if current_temp < (new_tool_target - 5):  # 5C tolerance
                self.gcode.respond_info(f"T{tool} at {current_temp}C - waiting for target {new_tool_target}C...")
                self._wait_for_tool_temp(dwarf, new_tool_target, tolerance=5.0)

        # Reset sync state so next poll will use new tool
        self._last_synced_temp = 0

        self.gcode.respond_info(
            f"T{tool} picked - offsets X={ox:.4f} Y={oy:.4f} Z={oz:.4f}")

        # PRUSA LOGIC: If new tool nozzle is SHORTER (negative nozzle_z_diff), inform user
        # The SET_GCODE_OFFSET already compensates for this via the negated offset
        # Note: nozzle_z_diff was calculated earlier in this function
        if nozzle_z_diff < -0.001:
            self.gcode.respond_info(
                f"New tool nozzle is {-nozzle_z_diff:.3f}mm shorter - compensated via gcode offset")

    def cmd_SPOOL_JOIN(self, gcmd):
        """Join spool: remap gcode tool to physical tool, transfer temp.
        Usage: SPOOL_JOIN TO=<tool> [FROM=<tool>]
        FROM defaults to the currently active tool.
        Use while print is paused after filament runout.
        Example: SPOOL_JOIN TO=3"""
        to_tool = gcmd.get_int('TO', -1)
        from_tool = gcmd.get_int('FROM', self.active_tool)
        if from_tool < 0 or from_tool > 4:
            raise gcmd.error("FROM must be 0-4 (or omit to use active tool)")
        if to_tool < 0 or to_tool > 4:
            raise gcmd.error("TO must be 0-4")
        if from_tool == to_tool:
            raise gcmd.error("FROM and TO must be different")
        to_dwarf = to_tool + 1
        from_dwarf = from_tool + 1
        if to_dwarf not in self.booted_dwarfs:
            raise gcmd.error(f"T{to_tool} (Dwarf {to_dwarf}) not available")

        # Transfer temperature from old tool to new tool
        old_target = self.target_temps.get(from_dwarf, 0)
        if old_target > 0:
            self._write_register(to_dwarf, 0xE000, int(old_target))
            self.target_temps[to_dwarf] = old_target
            self.gcode.respond_info(
                f"Transferred {old_target}C from T{from_tool} to T{to_tool}")

        # Cool down old tool
        self._write_register(from_dwarf, 0xE000, 0)
        self.target_temps[from_dwarf] = 0
        self.gcode.respond_info(f"T{from_tool} heater off")

        # Set up remap so future Tx gcode uses physical tool
        self.tool_remap[from_tool] = to_tool
        self.gcode.respond_info(
            f"Spool join active: T{from_tool} -> T{to_tool}")

        # If from_tool is currently picked, do the physical swap
        if self.tool_picked and self.active_tool == from_tool:
            self.gcode.respond_info(
                f"Swapping: parking T{from_tool}, picking T{to_tool}...")
            self.gcode.run_script_from_command("TOOL_PARK")
            self.gcode.run_script_from_command(f"TOOL_PICK T={to_tool}")
            # Wait for new tool to reach temp
            if old_target > 0:
                self.gcode.respond_info(
                    f"Waiting for T{to_tool} to reach {old_target}C...")
                self._wait_for_tool_temp(to_dwarf, old_target, tolerance=5.0)
            self.gcode.respond_info(
                f"Spool join complete. T{to_tool} ready. Resume print.")
        else:
            self.gcode.respond_info(
                f"Remap set. Next time gcode calls T{from_tool}, "
                f"T{to_tool} will be used instead.")

    def cmd_SPOOL_JOIN_STATUS(self, gcmd):
        """Show active spool join remaps."""
        if not self.tool_remap:
            self.gcode.respond_info("No spool join remaps active")
            return
        for gcode_tool, phys_tool in sorted(self.tool_remap.items()):
            self.gcode.respond_info(
                f"T{gcode_tool} -> T{phys_tool}")

    def cmd_SPOOL_JOIN_RESET(self, gcmd):
        """Clear all spool join remaps."""
        self.tool_remap.clear()
        self.gcode.respond_info("All spool join remaps cleared")

    def cmd_TOOL_PARK(self, gcmd):
        """Park the current tool back in its dock.
        Usage: TOOL_PARK"""
        if not self.tool_picked or self.active_tool < 0:
            raise gcmd.error("No tool is currently picked!")

        # Auto-home if needed before tool change
        toolhead = self.printer.lookup_object('toolhead')
        curtime = self.reactor.monotonic()
        kin_status = toolhead.get_status(curtime)
        homed = kin_status.get('homed_axes', '')
        if 'x' not in homed or 'y' not in homed or 'z' not in homed:
            self.gcode.respond_info("Axes not homed - homing all first...")
            self.gcode.run_script_from_command("G28")

        # Z safety check - ensure bed is at safe height before tool change
        cur_pos = toolhead.get_position()
        safe_z = 20.0
        if cur_pos[2] < safe_z:
            self.gcode.respond_info(f"Moving Z from {cur_pos[2]:.1f} to {safe_z} for safe tool change")
            self.gcode.run_script_from_command(f"G1 Z{safe_z} F600")
            toolhead.wait_moves()

        tool = self.active_tool
        dwarf = tool + 1

        # NOTE: Do NOT turn off heaters when parking during multi-tool prints!
        # The slicer (OrcaSlicer) manages tool temperatures via M104/M109.
        # Turning off heaters here causes "extruder below minimum temp" errors
        # when switching back to a previously parked tool.
        # Heaters are only turned off by END_PRINT or explicit M104 S0.

        # NOTE: NO retract here - Prusa doesn't retract in park()!
        # The slicer handles retraction at the wipe tower via generated G-code.
        # Adding retract here without matching prime caused issues.

        # CRITICAL: Clear gcode offsets BEFORE parking moves!
        # Dock coordinates are in machine space. G1 commands operate in gcode space.
        # With tool offsets active, G1 X271 would go to X=271+offset, exceeding limits.
        # Prusa also removes tool offsets before parking (hotend_currently_applied_offset).
        self.gcode.run_script_from_command("SET_GCODE_OFFSET X=0 Y=0 Z=0 MOVE=0")
        self.applied_tool_offset = (0.0, 0.0, 0.0)

        # Sanity check: verify toolhead position is within machine limits.
        # A "Move out of range" error in Klipper corrupts gcode_move.last_position
        # (updated before the move is validated) without moving the toolhead.
        # This desync can cause TOOL_PARK to compute wildly wrong targets.
        # Fix: re-sync gcode_move from the actual toolhead position.
        toolhead = self.printer.lookup_object('toolhead')
        cur_pos = toolhead.get_position()
        max_x = 360.0
        max_y = 465.0  # Beyond bed (360) to reach docks (~455)
        if (cur_pos[0] < -10 or cur_pos[0] > max_x or
                cur_pos[1] < -10 or cur_pos[1] > max_y):
            self.gcode.respond_info(
                f"WARNING: Position tracking suspect "
                f"({cur_pos[0]:.1f}, {cur_pos[1]:.1f}) - re-homing")
            logging.warning(
                f"PuppyBootloader: TOOL_PARK position sanity check failed: "
                f"({cur_pos[0]:.1f}, {cur_pos[1]:.1f}) - forcing G28")
            self.gcode.run_script_from_command("G28")
            # Re-check after homing
            cur_pos = toolhead.get_position()

        dock_x = self._get_dock_x(tool)
        dock_y = self._get_dock_y(tool)
        self.gcode.respond_info(
            f"Parking T{tool} at dock X={dock_x:.1f} Y={dock_y:.1f}")

        # Phase 1: Move to front of dock
        self._move_xy(x=dock_x + self.PARK_X_OFFSET_1,
                      y=self.SAFE_Y_WITH_TOOL, speed=self.TRAVEL_SPEED)
        self._wait_for_moves()

        # Phase 2: Move to dock Y position
        self._move_xy(y=dock_y, speed=self.TRAVEL_SPEED)
        self._wait_for_moves()

        # Phase 3: Unlock sequence with higher current
        self.gcode.run_script_from_command(
            f"SET_TMC_CURRENT STEPPER=stepper_x CURRENT="
            f"{self.PARKING_CURRENT / 1000:.3f}")
        self.gcode.run_script_from_command(
            f"SET_TMC_CURRENT STEPPER=stepper_y CURRENT="
            f"{self.PARKING_CURRENT / 1000:.3f}")

        self._move_xy(x=dock_x + self.PARK_X_OFFSET_2, speed=self.SLOW_SPEED)
        self._wait_for_moves()
        self._move_xy(x=dock_x + self.PARK_X_OFFSET_3, speed=self.SLOW_SPEED)
        self._wait_for_moves()

        # Restore normal current
        self.gcode.run_script_from_command(
            f"SET_TMC_CURRENT STEPPER=stepper_x CURRENT="
            f"{self.NORMAL_CURRENT / 1000:.3f}")
        self.gcode.run_script_from_command(
            f"SET_TMC_CURRENT STEPPER=stepper_y CURRENT="
            f"{self.NORMAL_CURRENT / 1000:.3f}")

        # Phase 4: Insert tool into dock
        self._move_xy(x=dock_x, y=dock_y, speed=self.SLOW_SPEED)
        self._wait_for_moves()

        # Wait for is_parked
        self.reactor.pause(self.reactor.monotonic() + 0.3)
        state = self._get_dock_state(dwarf)
        parked = False
        if state:
            picked, parked = state
        if not parked:
            # Wiggle
            self.gcode.respond_info("Not parked, wiggling...")
            self._move_xy(x=dock_x - self.DOCK_WIGGLE, speed=self.SLOW_SPEED)
            self._wait_for_moves()
            self._move_xy(x=dock_x, speed=self.SLOW_SPEED)
            self._wait_for_moves()
            self.reactor.pause(self.reactor.monotonic() + 0.3)
            state = self._get_dock_state(dwarf)
            if state:
                picked, parked = state
            if not parked:
                raise gcmd.error(
                    f"T{tool} not reporting parked after insert!")

        self.gcode.respond_info(f"T{tool} parked detected")

        # Phase 5: Deselect and depart
        self._write_coil(dwarf, 0x4001, False)  # is_selected = false
        if self.tmc_enabled.get(dwarf, False):
            self._write_coil(dwarf, 0x4000, False)  # TMC disable
            self.tmc_enabled[dwarf] = False

        # MULTI-TOOL FIX: Re-send heater target after deselect
        # The Dwarf firmware clears heater target when tool is deselected for safety.
        # For multi-tool printing, we want parked tools to maintain their temps
        # so they're ready when we switch back. The slicer manages temps via M104.
        if dwarf in self.target_temps and self.target_temps[dwarf] > 0:
            self._write_register(dwarf, 0xE000, int(self.target_temps[dwarf]))
            logging.info(f"PuppyBootloader: Preserved heater target "
                        f"{self.target_temps[dwarf]}C for parked T{tool}")

        # Depart from dock
        self._move_xy(y=self.SAFE_Y_WITHOUT_TOOL, speed=self.TRAVEL_SPEED)
        self._wait_for_moves()

        # Verify not picked anymore
        self.reactor.pause(self.reactor.monotonic() + 0.3)
        state = self._get_dock_state(dwarf)
        if state:
            picked, parked = state
            if picked:
                self.gcode.respond_info(
                    "WARNING: Still reporting picked after depart!")

        self.active_tool = -1
        self.tool_picked = False

        # CRITICAL FIX: Clear Klipper heater target when no tool is active
        # Without this, Klipper still expects the old temp but sensor reads 0
        # → "not heating at expected rate" error
        if self._extruder_heater is not None:
            self._extruder_heater.set_temp(0.)
            logging.info("PuppyBootloader: TOOL_PARK cleared Klipper heater target (no tool active)")

        # Offsets already cleared at start of TOOL_PARK (before dock moves)

        self.gcode.respond_info(f"T{tool} parked successfully!")

    def cmd_M104(self, gcmd):
        """M104 [T<tool>] S<temp> - Set hotend temperature"""
        tool = gcmd.get_int('T', self.active_tool)
        target = int(gcmd.get_float("S", 0))
        dwarf = tool + 1

        if dwarf not in self.booted_dwarfs:
            raise gcmd.error(f"Tool {tool} (Dwarf {dwarf}) not available")

        # Write to register 0xE000 (nozzle_target_temperature)
        if self._write_register(dwarf, 0xE000, target):
            self.target_temps[dwarf] = target
            self.gcode.respond_info(
                f"T{tool} hotend target: {target}C")
            # MULTI-TOOL FIX: Also set Klipper heater target for active tool
            # This makes Mainsail show correct state and allows extrusion
            if tool == self.active_tool and self._extruder_heater is not None:
                self._extruder_heater.set_temp(float(target))
                logging.info(f"PuppyBootloader: M104 set Klipper heater target to {target}C")
        else:
            raise gcmd.error(f"Failed to set temperature on Dwarf {dwarf}")

    def cmd_M109(self, gcmd):
        """M109 [T<tool>] S<temp> - Set hotend temperature and wait"""
        tool = gcmd.get_int('T', self.active_tool)
        target = int(gcmd.get_float("S", 0))
        dwarf = tool + 1

        if dwarf not in self.booted_dwarfs:
            raise gcmd.error(f"Tool {tool} (Dwarf {dwarf}) not available")

        # Set temperature
        if not self._write_register(dwarf, 0xE000, target):
            raise gcmd.error(f"Failed to set temperature on Dwarf {dwarf}")
        self.target_temps[dwarf] = target
        self.gcode.respond_info(f"T{tool} heating to {target}C...")
        # MULTI-TOOL FIX: Also set Klipper heater target for active tool
        if tool == self.active_tool and self._extruder_heater is not None:
            self._extruder_heater.set_temp(float(target))
            logging.info(f"PuppyBootloader: M109 set Klipper heater target to {target}C")

        if target == 0:
            return

        # Wait for temperature to reach target (within 2C)
        while True:
            current = self.dwarf_data.get(dwarf, {}).get('hotend_temp', 0)
            if abs(current - target) <= 2:
                self.gcode.respond_info(
                    f"T{tool} reached {current}C (target {target}C)")
                break
            self.reactor.pause(self.reactor.monotonic() + 1.0)

    def cmd_M106(self, gcmd):
        """M106 [T<tool>|P<fan>] S<speed> - Set fan speed (0-255)"""
        # Accept both T and P for tool/fan selection (T takes precedence)
        fan = gcmd.get_int('T', gcmd.get_int('P', self.active_tool))
        speed = gcmd.get_int('S', 255)
        dwarf = fan + 1

        if dwarf not in self.booted_dwarfs:
            raise gcmd.error(f"Fan {fan} (Dwarf {dwarf}) not available")

        # Clamp to 0-255
        speed = max(0, min(255, speed))

        # Skip MODBUS write if speed hasn't changed (prevents bus flooding)
        current_speed = self.fan_speeds.get(dwarf, -1)
        if speed == current_speed:
            return  # No change, skip MODBUS write

        # Write to register 0xE002 (fan0_pwm)
        if self._write_register(dwarf, 0xE002, speed):
            self.fan_speeds[dwarf] = speed
            pct = int(speed / 255.0 * 100)
            self.gcode.respond_info(f"T{fan} fan: {speed}/255 ({pct}%)")
        else:
            raise gcmd.error(f"Failed to set fan on Dwarf {dwarf}")

    def cmd_M107(self, gcmd):
        """M107 [T<tool>|P<fan>] - Turn fan off"""
        # Accept both T and P for tool/fan selection (T takes precedence)
        fan = gcmd.get_int('T', gcmd.get_int('P', self.active_tool))
        dwarf = fan + 1

        if dwarf not in self.booted_dwarfs:
            raise gcmd.error(f"Fan {fan} (Dwarf {dwarf}) not available")

        # Skip MODBUS write if fan is already off (prevents bus flooding)
        current_speed = self.fan_speeds.get(dwarf, -1)
        if current_speed == 0:
            return  # Already off, skip MODBUS write

        if self._write_register(dwarf, 0xE002, 0):
            self.fan_speeds[dwarf] = 0
            self.gcode.respond_info(f"T{fan} fan off")
        else:
            raise gcmd.error(f"Failed to turn off fan on Dwarf {dwarf}")

    def cmd_M140(self, gcmd):
        """M140 S<temp> - Set bed temperature

        M140 S60       - Set bed to 60C (adaptive if bed area set, else all 16)
        M140 S60 B5    - Set only bedlet 5 to 60C (manual override, ignores adaptive)
        M140 S0        - Turn off all bedlets and reset adaptive target
        """
        target = gcmd.get_float('S', 0)
        bedlet = gcmd.get_int('B', -1)  # Optional: specific bedlet (0-15)

        if not self.modular_bed_booted:
            self.gcode.respond_info("Modular bed not available")
            return

        # Host-side safety: reject > 120C
        if target > 120.0:
            raise gcmd.error(f"Target {target}C exceeds bed max (120C)")

        if bedlet >= 0:
            # Manual single bedlet override (bypass adaptive)
            if bedlet >= self.NUM_BEDLETS:
                raise gcmd.error(f"Bedlet {bedlet} invalid (0-{self.NUM_BEDLETS-1})")
            current = dict(self.bed_target_temps)
            current[bedlet] = target
            if self._set_bedlet_temperatures(current):
                self.gcode.respond_info(f"Bedlet {bedlet} target: {target}C")
            else:
                raise gcmd.error(f"Failed to set bedlet {bedlet} temperature")
        elif self.bed_print_area is not None:
            # Adaptive mode: distribute temps based on print area
            if self._apply_adaptive_heating(target):
                enabled_count = bin(self.bed_enabled_mask).count('1')
                self.gcode.respond_info(
                    f"Bed target: {target}C (adaptive: {enabled_count}/16 bedlets)")
            else:
                raise gcmd.error("Failed to set bed temperature")
        else:
            # No area set - heat all bedlets uniformly (default behavior)
            self.bed_global_target = target
            if self._set_bedlet_temperatures(target):
                self.gcode.respond_info(f"Bed target: {target}C (all 16 bedlets)")
            else:
                raise gcmd.error("Failed to set bed temperature")

    def cmd_M190(self, gcmd):
        """M190 S<temp> - Set bed temperature and wait

        M190 S60       - Set bed to 60C and wait (adaptive if bed area set)
        M190 S60 B5    - Set bedlet 5 to 60C and wait (manual override)
        """
        target = gcmd.get_float('S', 0)
        bedlet = gcmd.get_int('B', -1)

        if not self.modular_bed_booted:
            self.gcode.respond_info("Modular bed not available")
            return

        # Host-side safety: reject > 120C
        if target > 120.0:
            raise gcmd.error(f"Target {target}C exceeds bed max (120C)")

        # Set the temperature first (same logic as M140)
        if bedlet >= 0:
            if bedlet >= self.NUM_BEDLETS:
                raise gcmd.error(f"Bedlet {bedlet} invalid (0-{self.NUM_BEDLETS-1})")
            current = dict(self.bed_target_temps)
            current[bedlet] = target
            if not self._set_bedlet_temperatures(current):
                raise gcmd.error(f"Failed to set bedlet {bedlet} temperature")
            self.gcode.respond_info(f"Waiting for bedlet {bedlet} to reach {target}C...")
        elif self.bed_print_area is not None:
            if not self._apply_adaptive_heating(target):
                raise gcmd.error("Failed to set bed temperature")
            enabled_count = bin(self.bed_enabled_mask).count('1')
            self.gcode.respond_info(
                f"Waiting for bed to reach {target}C (adaptive: {enabled_count}/16)...")
        else:
            self.bed_global_target = target
            if not self._set_bedlet_temperatures(target):
                raise gcmd.error("Failed to set bed temperature")
            self.gcode.respond_info(f"Waiting for bed to reach {target}C...")

        # Wait for temperature to be reached (within 2C tolerance)
        tolerance = 2.0
        timeout = 600.0  # 10 minute timeout
        start = self.reactor.monotonic()

        while True:
            temps = self._read_bedlet_temperatures()
            if temps is None:
                self.reactor.pause(self.reactor.monotonic() + 1.0)
                continue

            # Update local tracking
            self.bed_measured_temps = temps

            # Check if target reached
            if bedlet >= 0:
                # Check single bedlet
                if abs(temps.get(bedlet, 0) - target) <= tolerance:
                    self.gcode.respond_info(f"Bedlet {bedlet} at {temps[bedlet]:.1f}C")
                    break
            else:
                # Average only ENABLED bedlets (matches Prusa updateModularBedTemperature)
                mask = self.bed_enabled_mask
                enabled_temps = [temps[i] for i in range(16)
                                 if (mask & (1 << i)) and i in temps]
                avg = sum(enabled_temps) / len(enabled_temps) if enabled_temps else 0
                if target == 0 or abs(avg - target) <= tolerance:
                    self.gcode.respond_info(f"Bed at {avg:.1f}C (enabled avg)")
                    break

            # Timeout check
            if self.reactor.monotonic() - start > timeout:
                raise gcmd.error("Bed heating timeout")

            self.reactor.pause(self.reactor.monotonic() + 2.0)

    def cmd_BED_STATUS(self, gcmd):
        """Show modular bed status with physical grid layout and adaptive state.
        Uses Prusa bedlet wiring map for correct physical positions."""
        if not self.modular_bed_booted:
            self.gcode.respond_info("Modular bed not booted")
            return

        temps = self._read_bedlet_temperatures()
        if temps is None:
            self.gcode.respond_info("Failed to read bedlet temperatures")
            return

        # Update local tracking
        self.bed_measured_temps = temps
        mask = self.bed_enabled_mask

        # Header with mode info
        if self.bed_print_area is not None:
            x0, y0, x1, y1 = self.bed_print_area
            enabled_count = bin(mask).count('1')
            lines = [f"Modular bed [ADAPTIVE] area=({x0:.0f},{y0:.0f})-({x1:.0f},{y1:.0f}) "
                     f"{enabled_count}/16 enabled:"]
        else:
            lines = ["Modular bed [FULL] all 16 bedlets:"]

        # Display using Prusa physical wiring map
        lines.append("  Measured temps (physical grid, * = enabled):")
        lines.append("           Col0     Col1     Col2     Col3")
        for row in range(4):
            y_lo = row * 90
            y_hi = (row + 1) * 90
            row_str = f"  Y{y_lo:3d}-{y_hi:3d}"
            for col in range(4):
                phys = self.BEDLET_MAP[row][col]
                temp = temps.get(phys, 0)
                mark = "*" if (mask & (1 << phys)) else " "
                row_str += f" {temp:5.1f}{mark} "
            lines.append(row_str)

        # Target temps grid
        if self.bed_global_target > 0:
            lines.append("  Target temps:")
            for row in range(4):
                row_str = "          "
                for col in range(4):
                    phys = self.BEDLET_MAP[row][col]
                    tgt = self.bed_target_temps.get(phys, 0)
                    row_str += f" {tgt:5.1f}  "
                lines.append(row_str)

        # Averages
        enabled_temps = [temps[i] for i in range(16)
                         if (mask & (1 << i)) and i in temps]
        all_avg = sum(temps.values()) / len(temps) if temps else 0
        en_avg = sum(enabled_temps) / len(enabled_temps) if enabled_temps else 0
        lines.append(f"  Enabled avg: {en_avg:.1f}C  All avg: {all_avg:.1f}C  "
                     f"Target: {self.bed_global_target:.0f}C")

        self.gcode.respond_info("\n".join(lines))

    def cmd_BED_DIAG(self, gcmd):
        """Show modular bed diagnostics - fault status and communication test"""
        if not self.modular_bed_booted:
            self.gcode.respond_info("Modular bed not booted")
            return

        lines = ["=== Modular Bed Diagnostics ==="]

        # Try to read system fault status
        lines.append("\n1. System fault status (0x8000):")
        regs = self._read_bed_input_registers(self.BED_REG_FAULT_STATUS, 1)
        if regs:
            fault = regs[0]
            lines.append(f"   Raw value: 0x{fault:04X}")
            if fault == 0:
                lines.append("   No system faults")
            else:
                if fault & 0x01: lines.append("   - OverCurrent fault")
                if fault & 0x02: lines.append("   - UnexpectedCurrent fault")
        else:
            lines.append("   Failed to read")

        # Try to read bedlet fault status
        lines.append("\n2. Bedlet fault status (0xA000):")
        regs = self._read_bed_input_registers(self.BED_REG_BEDLET_FAULT, 16)
        if regs:
            has_faults = False
            for i, fault in enumerate(regs):
                if fault != 0:
                    has_faults = True
                    lines.append(f"   Bedlet {i}: 0x{fault:04X}")
                    if fault & 0x01: lines.append(f"      - HeaterDisconnected")
                    if fault & 0x02: lines.append(f"      - HeaterShortCircuit")
                    if fault & 0x04: lines.append(f"      - TempBelowMin")
                    if fault & 0x08: lines.append(f"      - TempAboveMax")
                    if fault & 0x10: lines.append(f"      - TempDropDetected")
                    if fault & 0x20: lines.append(f"      - TempPeakDetected")
                    if fault & 0x40: lines.append(f"      - PreheatError")
            if not has_faults:
                lines.append("   No bedlet faults")
        else:
            lines.append("   Failed to read")

        # Try to read heatbedlet count
        lines.append("\n3. Heatbedlet count (0x8010):")
        regs = self._read_bed_input_registers(self.BED_REG_HEATBEDLET_COUNT, 1)
        if regs:
            lines.append(f"   Count: {regs[0]}")
        else:
            lines.append("   Failed to read")

        # Try to read temperatures
        lines.append("\n4. Temperature read test:")
        temps = self._read_bedlet_temperatures()
        if temps:
            avg = sum(temps.values()) / len(temps)
            lines.append(f"   Success - avg temp: {avg:.1f}C")
        else:
            lines.append("   Failed to read temperatures")

        # Report fault clearing status
        lines.append(f"\n5. Fault cleared flag: {self.bed_faults_cleared}")

        self.gcode.respond_info("\n".join(lines))

    def cmd_SET_BED_AREA(self, gcmd):
        """SET_BED_AREA X0=<f> Y0=<f> X1=<f> Y1=<f> - Set print area for adaptive bed heating

        Computes which bedlets overlap the print area and enables adaptive heating.
        Subsequent M140/M190 commands will heat only the relevant bedlets with gradient.
        Matches Prusa M555 + modular_bed.cpp behavior.

        SET_BED_AREA X0=30 Y0=30 X1=200 Y1=200
        """
        if not self.modular_bed_booted:
            self.gcode.respond_info("Modular bed not available")
            return

        x0 = gcmd.get_float('X0', 0.0)
        y0 = gcmd.get_float('Y0', 0.0)
        x1 = gcmd.get_float('X1', 360.0)
        y1 = gcmd.get_float('Y1', 360.0)

        # Clamp to bed bounds
        x0 = max(0.0, min(360.0, x0))
        y0 = max(0.0, min(360.0, y0))
        x1 = max(0.0, min(360.0, x1))
        y1 = max(0.0, min(360.0, y1))

        if x1 <= x0 or y1 <= y0:
            raise gcmd.error(f"Invalid bed area: ({x0},{y0})-({x1},{y1})")

        self.bed_print_area = (x0, y0, x1, y1)

        # Compute mask
        enabled_mask = self._compute_enabled_mask(x0, y0, x1, y1)
        if self.bed_expand_to_sides:
            enabled_mask = self._expand_to_sides(enabled_mask, self.bed_global_target)
        self.bed_enabled_mask = enabled_mask

        # If bed is currently heating, reapply with adaptive temps
        if self.bed_global_target > 0:
            self._apply_adaptive_heating(self.bed_global_target)

        enabled_count = bin(enabled_mask).count('1')

        # Show grid
        lines = [f"Bed area: ({x0:.0f},{y0:.0f})-({x1:.0f},{y1:.0f}), "
                 f"{enabled_count}/16 bedlets enabled"]
        lines.append("  Bedlet grid (* = enabled):")
        for row in range(4):
            row_str = "    "
            for col in range(4):
                phys = self.BEDLET_MAP[row][col]
                mark = "*" if (enabled_mask & (1 << phys)) else "."
                row_str += f" {mark} "
            y_lo = row * 90
            y_hi = (row + 1) * 90
            lines.append(f"  Y{y_lo:3d}-{y_hi:3d}: {row_str}")
        self.gcode.respond_info("\n".join(lines))

    def cmd_CLEAR_BED_AREA(self, gcmd):
        """CLEAR_BED_AREA - Reset to full bed heating (all 16 bedlets)

        Called by END_PRINT and CANCEL_PRINT to restore default behavior.
        """
        self.bed_print_area = None
        self.bed_enabled_mask = 0xFFFF

        # If bed is currently heating, rewrite all bedlets to global target
        if self.bed_global_target > 0:
            self._set_bedlet_temperatures(self.bed_global_target)
            self.gcode.respond_info(f"Bed area cleared - all 16 bedlets at {self.bed_global_target}C")
        else:
            self.gcode.respond_info("Bed area cleared - full bed mode")

    def cmd_SET_BEDLET_TEMP(self, gcmd):
        """SET_BEDLET_TEMP BEDLET=<0-15> TEMP=<celsius>

        Set temperature for a specific bedlet in the 4x4 grid.
        Example: SET_BEDLET_TEMP BEDLET=5 TEMP=70
        """
        if not self.modular_bed_booted:
            self.gcode.respond_info("Modular bed not available")
            return

        bedlet = gcmd.get_int('BEDLET', None)
        temp = gcmd.get_float('TEMP', None)

        if bedlet is None:
            raise gcmd.error("BEDLET parameter required (0-15)")
        if temp is None:
            raise gcmd.error("TEMP parameter required")
        if bedlet < 0 or bedlet >= self.NUM_BEDLETS:
            raise gcmd.error(f"BEDLET must be 0-{self.NUM_BEDLETS-1}")

        # Update single bedlet
        current = dict(self.bed_target_temps)
        current[bedlet] = temp

        if self._set_bedlet_temperatures(current):
            row = bedlet // 4
            col = bedlet % 4
            self.gcode.respond_info(f"Bedlet {bedlet} (row {row}, col {col}): target {temp}C")
        else:
            raise gcmd.error(f"Failed to set bedlet {bedlet} temperature")

    def cmd_SET_DWARF_TEMP(self, gcmd):
        """SET_DWARF_TEMP DWARF=<1-5> TEMP=<celsius>

        Set target temperature for a Dwarf hotend directly via MODBUS.
        Example: SET_DWARF_TEMP DWARF=1 TEMP=200
        """
        dwarf = gcmd.get_int('DWARF', None)
        temp = gcmd.get_float('TEMP', None)

        if dwarf is None:
            raise gcmd.error("DWARF parameter required (1-5)")
        if temp is None:
            raise gcmd.error("TEMP parameter required")
        if dwarf < 1 or dwarf > 5:
            raise gcmd.error("DWARF must be 1-5")

        if self._write_register(dwarf, 0xE000, int(temp)):
            # Track target temp so TOOL_PARK knows to turn off heater
            self.target_temps[dwarf] = int(temp)
            self.gcode.respond_info(f"Dwarf {dwarf} target temp set to {temp}C")

            # CRITICAL FIX: Sync Klipper's heater target if this is the active tool
            # Without this, CANCEL_PRINT turns off MODBUS heater but Klipper still
            # expects the old target temp → "not heating at expected rate" error
            tool = dwarf - 1  # Dwarf 1 = T0, etc.
            if tool == self.active_tool and self._extruder_heater is not None:
                self._extruder_heater.set_temp(float(temp))
                logging.info(f"PuppyBootloader: SET_DWARF_TEMP synced Klipper heater to {temp}C")
        else:
            raise gcmd.error(f"Failed to set Dwarf {dwarf} temperature")

    def cmd_M151(self, gcmd):
        """M151 - Set LED strip color (Prusa XL compatibility stub)

        M151 R<0-255> G<0-255> B<0-255> [P<index>]

        This is a stub for slicer compatibility. The XL's side LED strip
        requires direct hardware control not yet implemented in Klipper.
        """
        r = gcmd.get_int('R', 0)
        g = gcmd.get_int('G', 0)
        b = gcmd.get_int('B', 0)
        p = gcmd.get_int('P', 0)
        # Log but don't fail - allows gcode to continue
        logging.info(f"PuppyBootloader: M151 LED stub - R={r} G={g} B={b} P={p}")

    def cmd_P0(self, gcmd):
        """P0 - Park current tool (Prusa XL compatibility)

        P0 [S1] [L<0-2>] [D<0-1>]

        Parks the currently picked tool. Parameters are for Prusa compatibility:
        - S1: Don't move XY after parking
        - L: Z lift (0=none, 1=MBL, 2=full)
        - D: Z return (0=don't, 1=normal)
        """
        if not self.tool_picked:
            self.gcode.respond_info("No tool picked - nothing to park")
            return

        # Parse parameters (for logging/future use)
        s_param = gcmd.get_int('S', 0)
        l_param = gcmd.get_int('L', 2)
        d_param = gcmd.get_int('D', 0)

        self.gcode.respond_info(f"Parking T{self.active_tool}...")
        self.gcode.run_script_from_command("TOOL_PARK")

    def cmd_M17(self, gcmd):
        """M17 - Enable steppers (Prusa/OrcaSlicer compatibility stub)

        Klipper enables steppers automatically when needed.
        This stub accepts the command without error.
        """
        logging.info("PuppyBootloader: M17 enable steppers stub")

    def cmd_M201_stub(self, gcmd):
        """M201 - Set max acceleration (Marlin compatibility stub)

        Klipper uses printer.cfg for acceleration limits.
        """
        logging.info("PuppyBootloader: M201 max acceleration stub")

    def cmd_M203_stub(self, gcmd):
        """M203 - Set max feedrate (Marlin compatibility stub)

        Klipper uses printer.cfg for velocity limits.
        """
        logging.info("PuppyBootloader: M203 max feedrate stub")

    def cmd_M205_stub(self, gcmd):
        """M205 - Set jerk/advanced settings (Marlin compatibility stub)

        Klipper uses square_corner_velocity instead of jerk.
        """
        logging.info("PuppyBootloader: M205 jerk settings stub")

    def cmd_M862_stub(self, gcmd):
        """M862.x - Prusa firmware verification checks (stub)

        M862.1 P<nozzle_diameter> - Nozzle diameter check
        M862.3 P"<model>" - Printer model check
        M862.5 P<gcode_level> - G-code level check
        M862.6 P"<feature>" - Firmware feature check

        These are Prusa-specific verification commands. We accept but ignore
        them to allow PrusaSlicer gcode to run without errors.
        """
        # Just log and continue - allows gcode to proceed
        params = gcmd.get_commandline()
        logging.info(f"PuppyBootloader: M862 stub - {params}")

    def cmd_M555(self, gcmd):
        """M555 - Set print area (Prusa XL compatibility stub)

        M555 X<min> Y<min> W<width> H<height>

        Sets the print area boundaries. Currently a stub that logs
        but doesn't enforce boundaries.
        """
        x = gcmd.get_float('X', 0)
        y = gcmd.get_float('Y', 0)
        w = gcmd.get_float('W', 0)
        h = gcmd.get_float('H', 0)
        logging.info(f"PuppyBootloader: M555 print area stub - X={x} Y={y} W={w} H={h}")

    def cmd_M217(self, gcmd):
        """M217 - Toolchange settings (Prusa XL compatibility stub)

        M217 [S<swap_length>] [P<prime_speed>] [R<retract_speed>] [Z<z_raise>]

        Sets toolchange parameters. Currently a stub - Klipper handles
        toolchange motion through TOOL_PICKUP/TOOL_PARK macros.
        """
        params = gcmd.get_commandline()
        logging.info(f"PuppyBootloader: M217 toolchange settings stub - {params}")

    def cmd_M302(self, gcmd):
        """M302 - Cold extrusion settings (Prusa XL compatibility stub)

        M302 S<temp> - Set minimum extrusion temperature
        M302 P1 - Allow cold extrusion

        Klipper handles cold extrusion limits via min_extrude_temp in config.
        This stub logs but doesn't change behavior.
        """
        s = gcmd.get_float('S', None)
        p = gcmd.get_int('P', None)
        logging.info(f"PuppyBootloader: M302 cold extrusion stub - S={s} P={p}")

    def _apply_pending_offset(self):
        """Apply deferred tool offset after printer is ready.

        During boot, auto-detect sets _pending_offset_tool but can't run
        SET_GCODE_OFFSET because the printer isn't ready yet. This handler
        runs at klippy:ready and applies the offset.
        """
        tool = self._pending_offset_tool
        if tool is None:
            return
        self._pending_offset_tool = None
        offset = self.tool_offsets.get(tool, (0.0, 0.0, 0.0))
        ox, oy, oz = offset
        tool_z_adj = 0.0
        tool_offsets_mod = self.printer.lookup_object('tool_offsets', None)
        if tool_offsets_mod is not None:
            tool_z_adj = tool_offsets_mod.get_z_offset(tool)
        total_z = -oz + tool_z_adj
        try:
            self.gcode.run_script_from_command(
                f"SET_GCODE_OFFSET X={-ox:.4f} Y={-oy:.4f} Z={total_z:.4f} MOVE=0")
            self.applied_tool_offset = offset
            logging.info(
                f"PuppyBootloader: Applied T{tool} offset (deferred): "
                f"X={-ox:.4f} Y={-oy:.4f} Z={total_z:.4f} "
                f"(cal_z={oz:.4f}, tool_z_adj={tool_z_adj:.4f})")
        except Exception as e:
            logging.warning(
                f"PuppyBootloader: Failed to apply deferred T{tool} offset: {e}")

    def _override_m104_m109(self):
        """Override Klipper's M104/M109 commands at klippy:ready time.

        This allows us to handle the T parameter properly for multi-tool MODBUS control.
        Klipper's default M104/M109 handlers don't understand our Dwarf tool system.
        """
        if hasattr(self, '_m104_overridden') and self._m104_overridden:
            logging.info("PuppyBootloader: M104/M109 already overridden, skipping")
            return

        try:
            # Unregister Klipper's default M104/M109
            self.gcode.register_command("M104", None)
            self.gcode.register_command("M109", None)
        except Exception as e:
            logging.warning(f"PuppyBootloader: Error unregistering M104/M109: {e}")

        try:
            # Register our multi-tool MODBUS version
            self.gcode.register_command("M104", self.cmd_M104,
                desc="Set hotend temperature (multi-tool)")
            self.gcode.register_command("M109", self.cmd_M109,
                desc="Set hotend temperature and wait (multi-tool)")
            self._m104_overridden = True
            logging.info("PuppyBootloader: Overrode M104/M109 for multi-tool MODBUS control")
        except Exception as e:
            logging.error(f"PuppyBootloader: Failed to register M104/M109: {e}")

    def _override_turn_off_heaters(self):
        """Override TURN_OFF_HEATERS to also clear MODBUS heater targets."""
        try:
            self.gcode.register_command("TURN_OFF_HEATERS", None)
            self.gcode.register_command("TURN_OFF_HEATERS",
                self.cmd_TURN_OFF_HEATERS,
                desc="Turn off all heaters (including MODBUS)")
            logging.info("PuppyBootloader: Overrode TURN_OFF_HEATERS")
        except Exception as e:
            logging.error(f"PuppyBootloader: Failed to override TURN_OFF_HEATERS: {e}")

    def cmd_TURN_OFF_HEATERS(self, gcmd):
        """Turn off ALL heaters - Klipper + all MODBUS dwarfs + modular bed."""
        # Clear all stored targets so sync doesn't re-heat
        self.target_temps.clear()
        self._last_synced_temp = 0
        # Write 0 to all dwarfs
        for dwarf in self.booted_dwarfs:
            self._write_register(dwarf, 0xE000, 0)
        # Turn off modular bed (all 16 bedlets)
        if self.modular_bed_booted:
            self.bed_global_target = 0.0
            self._set_bedlet_temperatures(0.0)
        # Turn off Klipper heaters (extruder + bed)
        pheaters = self.printer.lookup_object('heaters', None)
        if pheaters is not None:
            for name, heater in pheaters.heaters.items():
                heater.set_temp(0.)
        self.gcode.respond_info("All heaters off (including MODBUS + bed)")

    def _override_m84(self):
        """Override Klipper's M84/M18 commands at klippy:ready time."""
        # Check if we've already overridden (guard against multiple calls)
        if hasattr(self, '_m84_overridden') and self._m84_overridden:
            logging.info("PuppyBootloader: M84/M18 already overridden, skipping")
            return

        try:
            # Unregister Klipper's default M84/M18 (may already be unregistered)
            self.gcode.register_command("M84", None)
            self.gcode.register_command("M18", None)
        except Exception as e:
            logging.warning(f"PuppyBootloader: Error unregistering M84/M18: {e}")

        try:
            # Register our Prusa-compatible version
            self.gcode.register_command("M84", self.cmd_M84,
                desc="Disable steppers (Prusa compatible)")
            self.gcode.register_command("M18", self.cmd_M84,
                desc="Disable steppers (Prusa compatible)")
            self._m84_overridden = True
            logging.info("PuppyBootloader: Overrode M84/M18 with Prusa-compatible version")
        except Exception as e:
            logging.error(f"PuppyBootloader: Failed to register M84/M18: {e}")

    def cmd_M84(self, gcmd):
        """M84/M18 - Disable steppers (Prusa/Marlin compatible)

        Prusa/Marlin behavior:
        - M84 (no params): Disable all motors
        - M84 E: Disable extruder motor only
        - M84 X Y Z: Disable specific axes

        Klipper's default M84 ignores parameters and clears homing state.
        This implementation preserves homing state when only E is specified,
        since the extruder doesn't affect homing.
        """
        commandline = gcmd.get_commandline().upper()

        # Check if only E axis is specified (common in Prusa gcode)
        has_e = ' E' in commandline or commandline.endswith('E')
        has_x = ' X' in commandline or commandline.endswith('X')
        has_y = ' Y' in commandline or commandline.endswith('Y')
        has_z = ' Z' in commandline or commandline.endswith('Z')

        if has_e and not (has_x or has_y or has_z):
            # Only E specified - disable extruder without clearing homing
            logging.info("PuppyBootloader: M84 E - disabling extruder only (preserving homing)")
            try:
                stepper_enable = self.printer.lookup_object('stepper_enable')
                if 'extruder' in stepper_enable.get_steppers():
                    stepper_enable.set_motors_enable(['extruder'], False)
            except Exception as e:
                logging.warning(f"M84 E: Could not disable extruder: {e}")
            return

        if has_x or has_y or has_z:
            # Specific axes requested - use SET_STEPPER_ENABLE for each
            logging.info(f"PuppyBootloader: M84 axes X={has_x} Y={has_y} Z={has_z} E={has_e}")
            try:
                stepper_enable = self.printer.lookup_object('stepper_enable')
                steppers_to_disable = []
                if has_x and 'stepper_x' in stepper_enable.get_steppers():
                    steppers_to_disable.append('stepper_x')
                if has_y and 'stepper_y' in stepper_enable.get_steppers():
                    steppers_to_disable.append('stepper_y')
                if has_z and 'stepper_z' in stepper_enable.get_steppers():
                    steppers_to_disable.append('stepper_z')
                if has_e and 'extruder' in stepper_enable.get_steppers():
                    steppers_to_disable.append('extruder')
                if steppers_to_disable:
                    stepper_enable.set_motors_enable(steppers_to_disable, False)
                    # Clear homing state for X/Y/Z axes that were disabled
                    toolhead = self.printer.lookup_object('toolhead')
                    clear_axes = ""
                    if has_x:
                        clear_axes += "x"
                    if has_y:
                        clear_axes += "y"
                    if has_z:
                        clear_axes += "z"
                    if clear_axes:
                        toolhead.get_kinematics().clear_homing_state(clear_axes)
            except Exception as e:
                logging.warning(f"M84: Could not disable steppers: {e}")
            return

        # No specific axes - disable all (default Klipper behavior)
        logging.info("PuppyBootloader: M84 - disabling all motors")
        try:
            stepper_enable = self.printer.lookup_object('stepper_enable')
            stepper_enable.motor_off()
        except Exception as e:
            logging.warning(f"M84: Could not disable motors: {e}")

    def cmd_G29(self, gcmd):
        """G29 - Prusa bed mesh / nozzle cleanup (compatibility handler)

        Prusa G29 variants:
        - G29 (no params): Probe bed mesh
        - G29 A: Activate mesh
        - G29 G: Heat soak / absorb heat
        - G29 P1: Probe points
        - G29 P3.2: Interpolate mesh
        - G29 P9: Nozzle cleanup (soft touch to push off filament blobs)

        Maps to Klipper BED_MESH commands where possible, stubs the rest.
        """
        commandline = gcmd.get_commandline()

        # Check for specific Prusa variants
        if ' G' in commandline or commandline.endswith('G'):
            # G29 G - Heat soak / absorb heat
            logging.info("PuppyBootloader: G29 G heat soak stub - waiting for bed to stabilize")
            self.gcode.respond_info("G29 G: Heat soak stub (bed temp stabilization)")
            # Could implement: wait for bed temp to stabilize
            return

        if ' P9' in commandline:
            # G29 P9 - Nozzle cleanup (soft touch to bed to push off blobs)
            logging.info("PuppyBootloader: G29 P9 nozzle cleanup stub")
            self.gcode.respond_info("G29 P9: Nozzle cleanup stub (soft touch)")
            # Future: implement soft touch to nozzle wiper area
            return

        if ' A' in commandline or commandline.endswith('A'):
            # G29 A - Activate mesh
            logging.info("PuppyBootloader: G29 A activating bed mesh")
            self.gcode.run_script_from_command("BED_MESH_PROFILE LOAD=default")
            return

        if ' P1' in commandline:
            # G29 P1 - Probe bed mesh
            logging.info("PuppyBootloader: G29 P1 probing bed mesh")
            self.gcode.run_script_from_command("BED_MESH_CALIBRATE")
            return

        if ' P3' in commandline:
            # G29 P3.x - Interpolate mesh (Klipper does this automatically)
            logging.info("PuppyBootloader: G29 P3 interpolate stub - Klipper auto-interpolates")
            self.gcode.respond_info("G29 P3: Interpolation stub (Klipper auto-interpolates)")
            return

        # Default: run bed mesh calibrate
        logging.info("PuppyBootloader: G29 running BED_MESH_CALIBRATE")
        self.gcode.run_script_from_command("BED_MESH_CALIBRATE")

    def cmd_M77(self, gcmd):
        """M77 - Stop print timer (Prusa compatibility stub)

        Prusa uses M75/M76/M77 for print timer control.
        Klipper/Moonraker handles print time tracking automatically.
        """
        logging.info("PuppyBootloader: M77 stop print timer stub")

    def cmd_M601(self, gcmd):
        """M601 - Pause print (Prusa compatibility)

        Maps to Klipper's PAUSE command.
        """
        logging.info("PuppyBootloader: M601 pause print")
        self.gcode.run_script_from_command("PAUSE")

    def cmd_DWARF_STATUS(self, gcmd):
        """Show status of all Dwarfs"""
        if not self.booted_dwarfs:
            self.gcode.respond_info("No Dwarfs booted")
            return
        lines = [f"Active tool: T{self.active_tool}"]
        for dwarf in sorted(self.booted_dwarfs):
            data = self.dwarf_data.get(dwarf, {})
            target = self.target_temps.get(dwarf, 0)
            fan = self.fan_speeds.get(dwarf, 0)
            hotend = data.get('hotend_temp', 0)
            board = data.get('board_temp', 0)
            heatbreak = data.get('heatbreak_temp', 0)
            fs = data.get('filament_sensor', 0)
            lines.append(
                f"  T{dwarf-1}/Dwarf{dwarf}: hotend={hotend}C"
                f" target={target}C heatbreak={heatbreak}C"
                f" board={board}C fan={fan}/255 fs={fs}")
        self.gcode.respond_info("\n".join(lines))

    def _register_bedlet_sensors(self):
        """Register virtual temperature sensors for modular bed.

        Creates:
        - _Bedlet_01 through _Bedlet_16 (hidden with underscore prefix)
        - Modular_Bed (visible average temperature)

        This matches Prusa's approach: single visible temp in UI, detailed view via BED_STATUS.
        """
        if hasattr(self, '_bedlet_sensors_registered') and self._bedlet_sensors_registered:
            return  # Already registered

        try:
            pheaters = self.printer.lookup_object('heaters')

            # Register hidden individual bedlet sensors
            self._bedlet_sensors = []
            for i in range(16):
                sensor = VirtualBedletSensor(self.printer, i, self)
                # Create mock config for heaters.register_sensor(config, psensor)
                mock_config = MockSensorConfig(self.printer, "temperature_sensor " + sensor.name)
                pheaters.register_sensor(mock_config, sensor)
                # Also register as printer object for Mainsail
                self.printer.add_object("temperature_sensor " + sensor.name, sensor)
                # Start the sensor's timer (setup_callback isn't called for dynamic sensors)
                self.reactor.update_timer(sensor.sample_timer, self.reactor.NOW)
                self._bedlet_sensors.append(sensor)
                logging.debug(f"PuppyBootloader: Registered {sensor.name}")

            # Register visible average sensor
            self._bed_avg_sensor = ModularBedAvgSensor(self.printer, self)
            mock_config = MockSensorConfig(self.printer, "temperature_sensor " + self._bed_avg_sensor.name)
            pheaters.register_sensor(mock_config, self._bed_avg_sensor)
            # Also register as printer object for Mainsail
            self.printer.add_object("temperature_sensor " + self._bed_avg_sensor.name, self._bed_avg_sensor)
            # Start the sensor's timer
            self.reactor.update_timer(self._bed_avg_sensor.sample_timer, self.reactor.NOW)
            logging.info(f"PuppyBootloader: Registered Modular_Bed sensor (avg of 16 bedlets)")

            self._bedlet_sensors_registered = True
            logging.info("PuppyBootloader: Bedlet temperature sensors registered")

        except Exception as e:
            logging.warning(f"PuppyBootloader: Failed to register bedlet sensors: {e}")

    def get_status(self, eventtime):
        """Status for Klipper's object system"""
        status = {'active_tool': self.active_tool, 'tool_picked': self.tool_picked}
        for dwarf in self.booted_dwarfs:
            data = self.dwarf_data.get(dwarf, {})
            target = self.target_temps.get(dwarf, 0)
            key = f"tool{dwarf-1}"
            status[key] = {
                'temperature': data.get('hotend_temp', 0),
                'target': target,
                'board_temp': data.get('board_temp', 0),
                'heatbreak_temp': data.get('heatbreak_temp', 0),
            }
        return status


# =============================================================================
# MODULAR BED TEMPERATURE SENSORS (Prusa-style display)
# =============================================================================
# VirtualBedletSensor: Hidden individual sensors with underscore prefix
# ModularBedAvgSensor: Visible average temperature sensor
# This matches Prusa's approach: single visible "Modular_Bed" temp + BED_STATUS for detail
# =============================================================================

class MockSensorConfig:
    """Mock config object for registering virtual sensors with heaters module.

    The heaters.register_sensor(config, psensor) method expects a config object
    with get_name() and get() methods.
    """

    def __init__(self, printer, name):
        self._printer = printer
        self._name = name

    def get_printer(self):
        return self._printer

    def get_name(self):
        return self._name

    def get(self, key, default=None):
        if key == 'gcode_id':
            return self._name
        return default


class VirtualBedletSensor:
    """Virtual temperature sensor for individual bedlet (hidden with underscore prefix).

    Reads from PuppyBootloader.bed_measured_temps dict.
    Named _Bedlet_01 through _Bedlet_16 (underscore hides in Mainsail).
    """

    def __init__(self, printer, bedlet_index, puppy_bootloader):
        self.printer = printer
        self.reactor = printer.get_reactor()
        self.bedlet_index = bedlet_index
        self.name = f"_Bedlet_{bedlet_index + 1:02d}"  # Zero-padded, underscore prefix
        self._puppy = puppy_bootloader
        self.temp = 0.0
        self.min_temp = 0.0
        self.max_temp = 150.0
        self._callback = None
        self.sample_timer = self.reactor.register_timer(self._sample_temperature)

    def get_name(self):
        return self.name

    def setup_minmax(self, min_temp, max_temp):
        self.min_temp = min_temp
        self.max_temp = max_temp

    def setup_callback(self, cb):
        self._callback = cb
        # Start sampling now
        self.reactor.update_timer(self.sample_timer, self.reactor.NOW)

    def get_report_time_delta(self):
        return 1.0  # Report every 1 second

    def _sample_temperature(self, eventtime):
        if self._puppy is not None:
            temps = self._puppy.bed_measured_temps
            self.temp = float(temps.get(self.bedlet_index, 0.0))

        if self._callback is not None:
            mcu = self.printer.lookup_object('mcu')
            measured_time = self.reactor.monotonic()
            self._callback(mcu.estimated_print_time(measured_time), self.temp)
        return eventtime + 1.0

    def get_status(self, eventtime):
        return {'temperature': round(self.temp, 1)}

    def get_temp(self, eventtime):
        return self.temp, 0.

    def stats(self, eventtime):
        return False, '%s: temp=%.1f' % (self.name, self.temp)


class ModularBedAvgSensor:
    """Virtual temperature sensor showing average of all 16 bedlets.

    This is the visible sensor in Mainsail (no underscore prefix).
    """

    def __init__(self, printer, puppy_bootloader):
        self.printer = printer
        self.reactor = printer.get_reactor()
        self.name = "Modular_Bed"
        self._puppy = puppy_bootloader
        self.temp = 0.0
        self.min_temp = 0.0
        self.max_temp = 150.0
        self._callback = None
        self.sample_timer = self.reactor.register_timer(self._sample_temperature)

    def get_name(self):
        return self.name

    def setup_minmax(self, min_temp, max_temp):
        self.min_temp = min_temp
        self.max_temp = max_temp

    def setup_callback(self, cb):
        self._callback = cb
        # Start sampling now
        self.reactor.update_timer(self.sample_timer, self.reactor.NOW)

    def get_report_time_delta(self):
        return 1.0  # Report every 1 second

    def _sample_temperature(self, eventtime):
        if self._puppy is not None:
            temps = self._puppy.bed_measured_temps
            mask = self._puppy.bed_enabled_mask
            if temps and mask:
                # Average only enabled bedlets (matches Prusa updateModularBedTemperature)
                enabled_temps = [temps[i] for i in range(16)
                                 if (mask & (1 << i)) and i in temps]
                if enabled_temps:
                    self.temp = sum(enabled_temps) / len(enabled_temps)
                elif temps:
                    self.temp = sum(temps.values()) / len(temps)
                else:
                    self.temp = 0.0
            elif temps:
                self.temp = sum(temps.values()) / len(temps)
            else:
                self.temp = 0.0

        if self._callback is not None:
            mcu = self.printer.lookup_object('mcu')
            measured_time = self.reactor.monotonic()
            self._callback(mcu.estimated_print_time(measured_time), self.temp)
        return eventtime + 1.0

    def get_status(self, eventtime):
        # Report the global bed target (what the user asked for), not avg of all 16
        target = 0.0
        if self._puppy is not None:
            target = self._puppy.bed_global_target
        return {
            'temperature': round(self.temp, 1),
            'target': round(target, 1)
        }

    def get_temp(self, eventtime):
        return self.temp, 0.

    def stats(self, eventtime):
        return False, '%s: temp=%.1f' % (self.name, self.temp)


class DwarfTemperatureSensor:
    """Custom temperature sensor that reads from MODBUS polling data.

    Includes stale data handling - if no update received within STALE_TIMEOUT,
    returns 0.0 to trigger safety checks in Klipper's heater system.

    Config options:
        dwarf: <1-5> - Read from specific Dwarf
        dwarf: 0     - Dynamic: read from active tool's Dwarf (active_tool + 1)
    """

    STALE_TIMEOUT = 10.0  # Seconds before temp data is considered stale
    # Note: With 5 Dwarfs, poll_interval=5s, and 0.5s inter-Dwarf gaps,
    # each Dwarf is polled every ~7.5s. Timeout must exceed this.

    def __init__(self, config):
        self.config = config  # Store config for _load_tool_offsets
        self.printer = config.get_printer()
        self.reactor = self.printer.get_reactor()
        self.name = config.get_name().split()[-1]
        self._config_dwarf = config.getint('dwarf', 1)
        self._use_active_tool = (self._config_dwarf == 0)  # dwarf: 0 means dynamic
        self.field = config.get('field', 'hotend_temp')
        self.temp = 0.0
        self.min_temp = 0.0
        self.max_temp = 300.0
        self._callback = None
        self._bootloader = None
        self._last_update_time = 0.0  # Track when we last got valid data
        self._last_raw_value = None   # Track raw value to detect changes
        self._last_dwarf = 0          # Track which dwarf we last read from
        self.sample_timer = self.reactor.register_timer(self._sample_temperature)
        self.printer.register_event_handler("klippy:connect",
                                            self._handle_connect)
        self.printer.register_event_handler("klippy:disconnect",
                                            self._handle_disconnect)

    def _get_dwarf(self):
        """Get the dwarf to read from - either fixed or active tool."""
        if self._use_active_tool and self._bootloader is not None:
            # active_tool is 0-4 for T0-T4, or -1 if no tool picked
            active = self._bootloader.active_tool
            if active >= 0:
                return active + 1  # Dwarf addresses are 1-5
            # No tool active - fall back to config dwarf or return None
            if self._config_dwarf > 0:
                return self._config_dwarf
            return None  # Signal that no valid dwarf is available
        return self._config_dwarf

    def _handle_connect(self):
        # Find the PuppyBootloader instance
        self._bootloader = self.printer.lookup_object('puppy_bootloader')
        # Start as stale - first poll will update to valid timestamp
        # This prevents false "fresh" readings before any polling occurs
        self._last_update_time = self.reactor.monotonic() - self.STALE_TIMEOUT
        # Safe timer update - capture reference to avoid race condition
        timer = self.sample_timer
        if timer is not None:
            self.reactor.update_timer(timer, self.reactor.NOW)

    def _handle_disconnect(self):
        # Clear bootloader reference first to signal disconnect to _sample_temperature
        self._bootloader = None
        # Properly unregister timer on disconnect to prevent race conditions
        timer = self.sample_timer
        if timer is not None:
            self.reactor.unregister_timer(timer)
            self.sample_timer = None

    def setup_minmax(self, min_temp, max_temp):
        self.min_temp = min_temp
        self.max_temp = max_temp

    def setup_callback(self, cb):
        self._callback = cb

    def get_report_time_delta(self):
        return 1.0  # Report every 1 second

    def _sample_temperature(self, eventtime):
        if self._bootloader is None:
            return eventtime + 1.0

        dwarf = self._get_dwarf()
        # If no valid dwarf (e.g., no tool picked and no config dwarf), return 0
        if dwarf is None or dwarf <= 0:
            self.temp = 0.0
            if self._callback is not None:
                mcu = self.printer.lookup_object('mcu')
                measured_time = self.reactor.monotonic()
                self._callback(mcu.estimated_print_time(measured_time), self.temp)
            return eventtime + 1.0

        data = self._bootloader.dwarf_data.get(dwarf, {})
        raw_temp = data.get(self.field, None)
        poll_time = data.get('_poll_time', 0.0)

        # Reset stale tracking if dwarf changed (tool change)
        if dwarf != self._last_dwarf:
            self._last_dwarf = dwarf
            # Use poll_time from new Dwarf if available, otherwise mark as stale
            if poll_time > 0:
                self._last_update_time = poll_time
            else:
                # New Dwarf hasn't been polled yet - treat as stale
                self._last_update_time = eventtime - self.STALE_TIMEOUT

        # Use poll timestamp for staleness check (not value change)
        if raw_temp is not None:
            temp_val = float(raw_temp)
            # Sanity clamp: discard garbage MODBUS reads (e.g. 0xFFFF during tool change)
            if temp_val < -10.0 or temp_val > 500.0:
                logging.warning(f"DwarfTemperatureSensor: Dwarf {dwarf} "
                              f"bogus reading {temp_val}C - ignoring")
            else:
                self.temp = temp_val
            # Update our tracking time from when polling actually occurred
            if poll_time > 0:
                self._last_update_time = poll_time
        else:
            # No data available
            self.temp = 0.0

        # Check staleness - based on when polling last updated, not value change
        time_since_poll = eventtime - self._last_update_time
        if time_since_poll > self.STALE_TIMEOUT:
            logging.warning(f"DwarfTemperatureSensor: Dwarf {dwarf} "
                          f"temperature stale (no poll for {time_since_poll:.1f}s)")
            # Return 0 to trigger Klipper's safety checks
            self.temp = 0.0

        if self._callback is not None:
            mcu = self.printer.lookup_object('mcu')
            measured_time = self.reactor.monotonic()
            self._callback(mcu.estimated_print_time(measured_time), self.temp)
        return eventtime + 1.0

    def get_status(self, eventtime):
        return {'temperature': round(self.temp, 1)}


class DwarfHeater:
    """Heater control via MODBUS register 0xE000 (target temp in degrees C)."""

    HEATER_REG = 0xE000  # Holding register for target temperature

    def __init__(self, config, puppy_bootloader):
        self.printer = config.get_printer()
        self.name = config.get_name().split()[-1]
        self._puppy = puppy_bootloader
        self._dwarf = config.getint('dwarf')
        self._target_temp = 0
        self._mcu = self.printer.lookup_object('mcu')

    def get_mcu(self):
        return self._mcu

    def set_temp(self, degrees):
        """Set target temperature via MODBUS."""
        self._target_temp = degrees
        if self._puppy:
            self._puppy._write_register(self._dwarf, self.HEATER_REG, int(degrees))
            # Keep target_temps in sync so _sync_extruder_heater doesn't fight
            self._puppy.target_temps[self._dwarf] = int(degrees)

    def get_temp(self):
        """Get current temp from polling data."""
        if self._puppy:
            data = self._puppy.dwarf_data.get(self._dwarf, {})
            temp = float(data.get('hotend_temp', 0))
            if temp < -10.0 or temp > 500.0:
                return 0.0  # Discard garbage MODBUS reads
            return temp
        return 0.0

    def get_target_temp(self):
        return self._target_temp


def load_config(config):
    printer = config.get_printer()

    # Register custom sensor factory FIRST - before anything else
    # This must happen before any [temperature_sensor] sections using dwarf_hotend
    pheaters = printer.load_object(config, "heaters")
    pheaters.add_sensor_factory("dwarf_hotend", DwarfTemperatureSensor)
    logging.info("PuppyBootloader: Registered 'dwarf_hotend' sensor type")

    # Register NullPinChip for "null:" prefix (used for Dwarf heaters via MODBUS)
    ppins = printer.lookup_object('pins')
    null_chip = NullPinChip(printer)
    ppins.register_chip('null', null_chip)
    logging.info("PuppyBootloader: Registered 'null:' pin prefix for MODBUS heaters")

    # Register the PuppyBootloader instance
    boot = PuppyBootloader(config)

    return boot
