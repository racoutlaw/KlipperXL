# MCU-Level Z Babystep support for instant Z offset adjustment
#
# This module sends commands directly to the babystep.c MCU code for
# true instant Z adjustment that bypasses Klipper's lookahead buffer.
#
# Features:
# - Instant Z adjustment (bypasses 1-2 sec lookahead buffer)
# - Auto-saves offset to variables.cfg (survives restarts)
# - Auto-loads saved offset on startup (like Prusa stock firmware)
#
# Copyright (C) 2024
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import logging

SAVE_VARIABLE_NAME = "babystep_z_offset"

class MCUBabyStep:
    """MCU-level babystep implementation using babystep.c firmware code."""

    def __init__(self, config):
        self.printer = config.get_printer()
        self.name = config.get_name()
        self.gcode = self.printer.lookup_object('gcode')

        # Configuration
        self.max_offset = config.getfloat('max_offset', 1.0, above=0.)
        self.auto_save = config.getboolean('auto_save', True)

        # Get MCU - we need the same MCU as the Z stepper
        self.mcu = None
        self.z_stepper = None
        self.z_stepper_oid = None
        self.step_dist = None

        # MCU commands
        self.config_babystep_cmd = None
        self.babystep_cmd = None

        # State tracking
        self.z_offset = 0.
        self.configured = False
        self.startup_applied = False

        # Register callbacks
        self.printer.register_event_handler("klippy:mcu_identify",
                                            self._handle_mcu_identify)
        self.printer.register_event_handler("klippy:ready",
                                            self._handle_ready)
        self.printer.register_event_handler("homing:home_rails_end",
                                            self._handle_home_rails_end)

        # Register G-Code commands
        self.gcode.register_command('BABYSTEP_Z', self.cmd_BABYSTEP_Z,
                               desc=self.cmd_BABYSTEP_Z_help)
        self.gcode.register_command('INSTANT_Z_ADJUST', self.cmd_INSTANT_Z_ADJUST,
                               desc=self.cmd_INSTANT_Z_ADJUST_help)
        self.gcode.register_command('GET_BABYSTEP', self.cmd_GET_BABYSTEP,
                               desc=self.cmd_GET_BABYSTEP_help)
        self.gcode.register_command('RESET_BABYSTEP', self.cmd_RESET_BABYSTEP,
                               desc=self.cmd_RESET_BABYSTEP_help)
        self.gcode.register_command('SAVE_BABYSTEP', self.cmd_SAVE_BABYSTEP,
                               desc=self.cmd_SAVE_BABYSTEP_help)

    def _handle_mcu_identify(self):
        """Called after MCU identification - find Z stepper and register config."""
        toolhead = self.printer.lookup_object('toolhead')
        kin = toolhead.get_kinematics()

        # Find first Z stepper
        for stepper in kin.get_steppers():
            if stepper.is_active_axis('z'):
                self.z_stepper = stepper
                self.z_stepper_oid = stepper.get_oid()
                self.step_dist = stepper.get_step_dist()
                self.mcu = stepper.get_mcu()
                logging.info("babystep: Using Z stepper '%s' (oid=%d, step_dist=%.6f)",
                             stepper.get_name(), self.z_stepper_oid, self.step_dist)
                break

        if self.z_stepper is None:
            logging.warning("babystep: No Z stepper found!")
            return

        # Register MCU config callback to send config_babystep during init
        self.mcu.register_config_callback(self._build_config)

    def _build_config(self):
        """Called during MCU config phase - register babystep commands."""
        # Add config command to MCU init sequence
        # Use is_init=True to ensure this runs AFTER all regular config commands
        # (including stepper configs) so the stepper OID is valid
        self.mcu.add_config_cmd(
            "config_babystep stepper_oid=%d" % (self.z_stepper_oid,),
            is_init=True)

        # Look up the babystep command for runtime use
        self.babystep_cmd = self.mcu.lookup_command(
            "babystep count=%hi direction=%c")

        self.configured = True
        logging.info("babystep: Configured MCU babystep for stepper oid=%d",
                     self.z_stepper_oid)

    def _handle_ready(self):
        """Called when printer is ready - auto-load saved offset."""
        if self.startup_applied:
            return
        self.startup_applied = True

        # Load saved offset from variables.cfg
        saved_offset = self._load_offset()
        if saved_offset != 0.:
            logging.info("babystep: Auto-loading saved Z offset: %.4f mm", saved_offset)
            # Don't use apply_babystep here - just set the internal tracking
            # The physical offset will be applied relative to the homed position
            self.z_offset = saved_offset

    def _load_offset(self):
        """Load saved offset from save_variables."""
        try:
            save_vars = self.printer.lookup_object("save_variables", None)
            if save_vars is not None:
                variables = save_vars.allVariables
                return float(variables.get(SAVE_VARIABLE_NAME, 0.))
        except Exception as e:
            logging.info("babystep: Could not load saved offset: %s", e)
        return 0.

    def _save_offset(self):
        """Save current offset to save_variables."""
        try:
            save_vars = self.printer.lookup_object("save_variables", None)
            if save_vars is not None:
                save_vars.cmd_SAVE_VARIABLE(self.gcode.create_gcode_command(
                    "SAVE_VARIABLE", "SAVE_VARIABLE",
                    {"VARIABLE": SAVE_VARIABLE_NAME, "VALUE": str(self.z_offset)}))
                logging.info("babystep: Saved Z offset %.4f to variables.cfg",
                             self.z_offset)
        except Exception as e:
            logging.warning("babystep: Could not save offset: %s", e)

    def _handle_home_rails_end(self, homing_state, rails):
        """After Z homing, apply saved offset automatically."""
        if 2 in homing_state.get_axes():  # Z axis
            # After homing, apply the saved offset (like Prusa does)
            saved_offset = self._load_offset()
            if saved_offset != 0. and self.configured:
                logging.info("babystep: Applying saved Z offset %.4f after homing",
                             saved_offset)
                # Reset internal tracking first
                self.z_offset = 0.
                # Apply the saved offset
                self.apply_babystep(saved_offset, auto_save=False)

    def _mm_to_steps(self, mm):
        """Convert millimeters to step count."""
        if self.step_dist is None or self.step_dist == 0:
            return 0
        return int(mm / self.step_dist + (0.5 if mm >= 0 else -0.5))

    def _send_babystep(self, step_count, direction):
        """Send babystep command to MCU for instant execution."""
        if not self.configured:
            raise self.printer.command_error("Babystep not configured - MCU may not support it")

        if self.babystep_cmd is None:
            raise self.printer.command_error("Babystep MCU command not available")

        # Send immediately - no scheduling, no queue
        self.babystep_cmd.send([step_count, direction])

    def apply_babystep(self, delta_mm, auto_save=None):
        """Apply a babystep adjustment in millimeters."""
        if delta_mm == 0:
            return

        # Check limits
        new_offset = self.z_offset + delta_mm
        if abs(new_offset) > self.max_offset:
            raise self.printer.command_error(
                "Babystep offset %.4f would exceed max_offset %.4f"
                % (new_offset, self.max_offset))

        # Convert to steps
        step_count = self._mm_to_steps(abs(delta_mm))
        if step_count == 0:
            logging.info("babystep: Delta %.6f mm is less than one step", delta_mm)
            return

        # Determine direction (0 = one way, 1 = other way)
        # On Prusa XL: Z+ = bed moves down (away from nozzle)
        # For babystep: positive delta should raise nozzle relative to bed
        direction = 0 if delta_mm > 0 else 1

        # Send to MCU - this executes IMMEDIATELY
        self._send_babystep(step_count, direction)

        # Update tracked offset
        self.z_offset = new_offset
        logging.info("babystep: Applied %d steps (%.4f mm), total offset: %.4f mm",
                     step_count, delta_mm, self.z_offset)

        # Auto-save if enabled (default to config setting)
        if auto_save is None:
            auto_save = self.auto_save
        if auto_save:
            self._save_offset()

    def get_status(self, eventtime=None):
        """Return current babystep status for Moonraker/display."""
        return {
            'z_offset': self.z_offset,
            'max_offset': self.max_offset,
            'configured': self.configured,
            'step_dist': self.step_dist,
        }

    cmd_BABYSTEP_Z_help = "Apply instant Z babystep (MCU-level, no delay)"
    def cmd_BABYSTEP_Z(self, gcmd):
        """BABYSTEP_Z DELTA=<mm> - Apply instant Z offset adjustment."""
        delta = gcmd.get_float('DELTA')
        if delta == 0.:
            gcmd.respond_info("BABYSTEP_Z: No change (DELTA=0)")
            return

        self.apply_babystep(delta)
        gcmd.respond_info("Instant babystep: %.4f mm (total: %.4f mm)"
                          % (delta, self.z_offset))

    cmd_INSTANT_Z_ADJUST_help = "Alias for BABYSTEP_Z for SET_GCODE_OFFSET wrapper"
    def cmd_INSTANT_Z_ADJUST(self, gcmd):
        """INSTANT_Z_ADJUST Z=<mm> - Apply instant Z offset (for Mainsail wrapper)."""
        z_adjust = gcmd.get_float('Z', None)
        if z_adjust is None:
            z_adjust = gcmd.get_float('Z_ADJUST', 0.)

        if z_adjust == 0.:
            gcmd.respond_info("INSTANT_Z_ADJUST: No change")
            return

        self.apply_babystep(z_adjust)
        gcmd.respond_info("Instant Z adjust: %.4f mm (total: %.4f mm)"
                          % (z_adjust, self.z_offset))

    cmd_GET_BABYSTEP_help = "Get current babystep Z offset"
    def cmd_GET_BABYSTEP(self, gcmd):
        """GET_BABYSTEP - Report current babystep status."""
        msg = "Babystep Z offset: %.4f mm (max: %.4f mm)\n" % (
            self.z_offset, self.max_offset)
        msg += "Step distance: %.6f mm\n" % (self.step_dist or 0)
        msg += "MCU configured: %s" % ("Yes" if self.configured else "No")
        gcmd.respond_info(msg)

    cmd_RESET_BABYSTEP_help = "Reset babystep Z offset to zero"
    def cmd_RESET_BABYSTEP(self, gcmd):
        """RESET_BABYSTEP - Move back to zero offset."""
        if self.z_offset == 0.:
            gcmd.respond_info("Babystep offset already at zero")
            return

        # Apply reverse babystep to return to zero
        self.apply_babystep(-self.z_offset)
        gcmd.respond_info("Babystep offset reset to zero")

    cmd_SAVE_BABYSTEP_help = "Manually save babystep Z offset to variables.cfg"
    def cmd_SAVE_BABYSTEP(self, gcmd):
        """SAVE_BABYSTEP - Save current offset to variables.cfg."""
        self._save_offset()
        gcmd.respond_info("Babystep offset %.4f saved to variables.cfg" % self.z_offset)


def load_config(config):
    return MCUBabyStep(config)
