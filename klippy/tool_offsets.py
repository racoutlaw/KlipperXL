# Per-tool Z offset fine-tuning for multi-tool printers
#
# Provides [tool_offsets] config section with per-tool z_offset values.
# Works with Mainsail's Z-offset +/- buttons and Save button.
# Integrates with puppy_bootloader.py for tool change offset application.
#
# Copyright (C) 2026  XlKlipper Contributors
# Licensed under the GNU GPLv3
import logging


class ToolOffsets:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.name = config.get_name()
        self.gcode = self.printer.lookup_object('gcode')

        # Safety bounds (Prusa: Z -2.0 to +1.45, we use tighter for fine-tuning)
        self.z_min = config.getfloat('z_min', -0.3)
        self.z_max = config.getfloat('z_max', 0.5)

        # Load per-tool z_offsets from config
        self.z_offsets = {}
        for tool in range(5):
            key = 't%d_z_offset' % tool
            val = config.getfloat(key, 0.)
            # Clamp on load
            if val < self.z_min or val > self.z_max:
                logging.warning(
                    "tool_offsets: T%d z_offset %.4f out of bounds "
                    "(%.1f to %.1f) - clamping" % (tool, val,
                                                    self.z_min, self.z_max))
                val = max(self.z_min, min(self.z_max, val))
            self.z_offsets[tool] = val

        # Register G-code commands
        self.gcode.register_command(
            'Z_OFFSET_APPLY_PROBE',
            self.cmd_Z_OFFSET_APPLY_PROBE,
            desc="Save current Z adjustment to active tool's z_offset")
        # Defer override of Z_OFFSET_APPLY_ENDSTOP until all modules loaded.
        # Mainsail's Save button calls Z_OFFSET_APPLY_ENDSTOP; we replace it
        # with our per-tool save since we don't need global endstop adjustment.
        self.printer.register_event_handler(
            "klippy:ready", self._override_endstop_cmd)
        self.gcode.register_command(
            'SAVE_TOOL_Z_OFFSET',
            self.cmd_SAVE_TOOL_Z_OFFSET,
            desc="Save Z offset for a specific tool")
        self.gcode.register_command(
            'GET_TOOL_Z_OFFSETS',
            self.cmd_GET_TOOL_Z_OFFSETS,
            desc="Display all per-tool Z offsets")
        self.gcode.register_command(
            'SET_TOOL_Z_OFFSET',
            self.cmd_SET_TOOL_Z_OFFSET,
            desc="Set Z offset for a specific tool")

        logging.info("tool_offsets: Loaded Z offsets: %s" % (
            ', '.join('T%d=%.4f' % (t, v)
                      for t, v in sorted(self.z_offsets.items()))))

    def _override_endstop_cmd(self):
        self.gcode.register_command('Z_OFFSET_APPLY_ENDSTOP', None)
        self.gcode.register_command(
            'Z_OFFSET_APPLY_ENDSTOP',
            self.cmd_Z_OFFSET_APPLY_PROBE,
            desc="Save current Z adjustment to active tool's z_offset")
        logging.info("tool_offsets: Overrode Z_OFFSET_APPLY_ENDSTOP "
                     "with per-tool save")

    def get_z_offset(self, tool):
        """Get the per-tool z_offset for a given tool number."""
        return self.z_offsets.get(tool, 0.)

    def get_status(self, eventtime=None):
        """Status for Jinja2 templates and Moonraker."""
        return {
            't0_z_offset': self.z_offsets.get(0, 0.),
            't1_z_offset': self.z_offsets.get(1, 0.),
            't2_z_offset': self.z_offsets.get(2, 0.),
            't3_z_offset': self.z_offsets.get(3, 0.),
            't4_z_offset': self.z_offsets.get(4, 0.),
            'z_min': self.z_min,
            'z_max': self.z_max,
        }

    def _get_active_tool_and_cal_offset(self):
        """Get active tool number and its calibrated Z offset from puppy_bootloader."""
        puppy = self.printer.lookup_object('puppy_bootloader', None)
        if puppy is None:
            return -1, 0.
        tool = puppy.active_tool
        if tool < 0:
            return -1, 0.
        cal_offset = puppy.tool_offsets.get(tool, (0., 0., 0.))
        return tool, cal_offset[2]  # Return tool number and stored cal Z

    def cmd_Z_OFFSET_APPLY_PROBE(self, gcmd):
        """Called by Mainsail's Save button.

        Reads the current Z gcode offset (homing_origin.z), subtracts
        the calibrated tool offset, and saves the remainder as the
        active tool's per-tool z_offset.
        """
        tool, cal_z = self._get_active_tool_and_cal_offset()
        if tool < 0:
            gcmd.respond_info("No tool active - cannot save Z offset")
            return

        # Read total Z gcode offset from Klipper
        gcode_move = self.printer.lookup_object("gcode_move")
        total_z = gcode_move.homing_position[2]

        # The total gcode Z offset = (-cal_z) + per_tool_z + user_adjustment
        # We want to capture the NEW per_tool_z (which includes user adjustment)
        # new_per_tool_z = total_z - (-cal_z) = total_z + cal_z
        new_z_offset = total_z + cal_z

        # Safety clamp
        if new_z_offset < self.z_min:
            gcmd.respond_info(
                "WARNING: T%d z_offset %.4f below minimum (%.1f) - clamping"
                % (tool, new_z_offset, self.z_min))
            new_z_offset = self.z_min
        elif new_z_offset > self.z_max:
            gcmd.respond_info(
                "WARNING: T%d z_offset %.4f above maximum (%.1f) - clamping"
                % (tool, new_z_offset, self.z_max))
            new_z_offset = self.z_max

        old_z_offset = self.z_offsets.get(tool, 0.)
        self.z_offsets[tool] = new_z_offset

        # Stage for SAVE_CONFIG
        configfile = self.printer.lookup_object('configfile')
        key = 't%d_z_offset' % tool
        configfile.set(self.name, key, "%.4f" % new_z_offset)

        gcmd.respond_info(
            "T%d z_offset: %.4f (was %.4f)\n"
            "The SAVE_CONFIG command will update the printer config file\n"
            "and restart the printer."
            % (tool, new_z_offset, old_z_offset))

    def cmd_SAVE_TOOL_Z_OFFSET(self, gcmd):
        """Save Z offset for a specific tool (or active tool).

        Usage: SAVE_TOOL_Z_OFFSET [TOOL=0] [Z=0.035]
        If Z is omitted, captures current adjustment like Z_OFFSET_APPLY_PROBE.
        """
        tool = gcmd.get_int('TOOL', -1)
        if tool < 0:
            # Use active tool
            tool, cal_z = self._get_active_tool_and_cal_offset()
            if tool < 0:
                gcmd.respond_info("No tool active and no TOOL= specified")
                return
        else:
            _, cal_z_tuple = self.printer.lookup_object(
                'puppy_bootloader', None), None
            puppy = self.printer.lookup_object('puppy_bootloader', None)
            if puppy:
                cal_z = puppy.tool_offsets.get(tool, (0., 0., 0.))[2]
            else:
                cal_z = 0.

        z_val = gcmd.get_float('Z', None)
        if z_val is not None:
            new_z_offset = z_val
        else:
            # Capture from current gcode offset
            gcode_move = self.printer.lookup_object("gcode_move")
            total_z = gcode_move.homing_position[2]
            new_z_offset = total_z + cal_z

        # Clamp
        new_z_offset = max(self.z_min, min(self.z_max, new_z_offset))

        old_z_offset = self.z_offsets.get(tool, 0.)
        self.z_offsets[tool] = new_z_offset

        configfile = self.printer.lookup_object('configfile')
        key = 't%d_z_offset' % tool
        configfile.set(self.name, key, "%.4f" % new_z_offset)

        gcmd.respond_info(
            "T%d z_offset: %.4f (was %.4f)\n"
            "Run SAVE_CONFIG to persist."
            % (tool, new_z_offset, old_z_offset))

    def cmd_SET_TOOL_Z_OFFSET(self, gcmd):
        """Set Z offset for a tool without staging for SAVE_CONFIG.

        Usage: SET_TOOL_Z_OFFSET TOOL=0 Z=0.035
        Updates in memory only. Use SAVE_TOOL_Z_OFFSET to persist.
        """
        tool = gcmd.get_int('TOOL')
        z_val = gcmd.get_float('Z')
        z_val = max(self.z_min, min(self.z_max, z_val))
        old = self.z_offsets.get(tool, 0.)
        self.z_offsets[tool] = z_val
        gcmd.respond_info("T%d z_offset: %.4f (was %.4f)" % (tool, z_val, old))

    def cmd_GET_TOOL_Z_OFFSETS(self, gcmd):
        """Display all per-tool Z offsets."""
        lines = ["Per-tool Z offsets:"]
        for tool in range(5):
            z = self.z_offsets.get(tool, 0.)
            lines.append("  T%d: %.4f mm" % (tool, z))
        gcmd.respond_info('\n'.join(lines))


def load_config(config):
    return ToolOffsets(config)
