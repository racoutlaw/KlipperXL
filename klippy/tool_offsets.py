# KlipperXL - Per-tool Z offset fine-tuning for multi-tool printers
#
# Provides [tool_offsets] config section with per-tool z_offset values.
# Works with Mainsail's Z-offset +/- buttons and Save button.
# Integrates with puppy_bootloader.py for tool change offset application.
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
import logging


class ToolOffsets:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.name = config.get_name()
        self.gcode = self.printer.lookup_object('gcode')

        # Safety bounds (Prusa: Z -2.0 to +1.45, we use tighter for fine-tuning)
        self.z_min = config.getfloat('z_min', -0.3)
        self.z_max = config.getfloat('z_max', 0.5)

        # Load per-tool z_offsets from config.
        #
        # TWO SLOTS PER TOOL, because a squish value is only valid in the frame
        # it was measured in:
        #   tN_z_offset         multi-tool prints - Z=0 came from T0's nozzle,
        #                       so cal_z is applied on top of this
        #   tN_z_offset_single  single-tool prints - the tool homed itself, so
        #                       cal_z cancels out and is NOT applied
        # Any error in cal_z was absorbed into the multi-tool value (the two
        # always applied together). Remove cal_z and that error stops
        # cancelling, so the single-tool frame needs its own number.
        #
        # The single value DEFAULTS to the multi value, so the first single-tool
        # print starts from today's known-good figure rather than from zero.
        # Once they diverge, the difference is a direct measurement of the
        # cal_z error for that tool.
        self.z_offsets = {}
        self.z_offsets_single = {}
        self.single_explicit = {}
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

            skey = 't%d_z_offset_single' % tool
            sval = config.getfloat(skey, None)
            if sval is None:
                # Never tuned in the single-tool frame - inherit the multi value
                self.z_offsets_single[tool] = val
                self.single_explicit[tool] = False
            else:
                if sval < self.z_min or sval > self.z_max:
                    logging.warning(
                        "tool_offsets: T%d z_offset_single %.4f out of bounds "
                        "(%.1f to %.1f) - clamping" % (tool, sval,
                                                        self.z_min, self.z_max))
                    sval = max(self.z_min, min(self.z_max, sval))
                self.z_offsets_single[tool] = sval
                self.single_explicit[tool] = True

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

    def get_z_offset(self, tool, single=False):
        """Get the per-tool z_offset for a given tool number.

        single=True selects the single-tool-print slot, used when that tool
        established Z=0 itself and cal_z is therefore not applied.
        """
        if not single:
            return self.z_offsets.get(tool, 0.)
        if self.single_explicit.get(tool, False):
            return self.z_offsets_single.get(tool, 0.)
        # Never tuned in this frame - seed from the NET offset the multi-tool
        # frame applies, which is (-cal_z + squish), NOT the squish alone.
        #
        # MEASURED on a 5-tool XL 2026-08-20: cal_z(T3) was stored as 0.3848
        # while T3's nozzle is only 0.050 longer than T0's. The squish of
        # -0.300 existed almost entirely to cancel that bad cal_z; together
        # they gave the correct 0.0848. Single-tool homing correctly drops
        # cal_z, so inheriting the squish alone leaves -0.300 with nothing to
        # cancel and drives the nozzle into the bed.
        puppy = self.printer.lookup_object('puppy_bootloader', None)
        cal_z = 0.
        if puppy is not None:
            cal_z = puppy.tool_offsets.get(tool, (0., 0., 0.))[2]
        return -cal_z + self.z_offsets.get(tool, 0.)

    def get_status(self, eventtime=None):
        """Status for Jinja2 templates and Moonraker."""
        status = {
            'z_min': self.z_min,
            'z_max': self.z_max,
        }
        for tool in range(5):
            status['t%d_z_offset' % tool] = self.z_offsets.get(tool, 0.)
            status['t%d_z_offset_single' % tool] = \
                self.get_z_offset(tool, single=True)
            status['t%d_single_tuned' % tool] = self.single_explicit.get(
                tool, False)
        return status

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

    def _get_datum_and_mode(self, tool=None):
        """Return (cal_z of the tool that set Z=0, use_single_slot).

        The applied offset is -(cal_z - datum_cal_z) + squish, so recovering
        the squish from a live gcode offset needs the datum term too.
        Homing on T0 gives datum_cal_z = 0 and this reduces to the old maths.
        """
        puppy = self.printer.lookup_object('puppy_bootloader', None)
        if puppy is None:
            return 0., False
        datum_tool = getattr(puppy, 'z_home_tool', 0)
        datum_cal_z = puppy.tool_offsets.get(datum_tool, (0., 0., 0.))[2]
        if tool is None:
            single = bool(getattr(puppy, 'single_tool_mode', False))
        else:
            # Same rule the offset application uses: single-tool job AND this
            # tool is the datum tool.
            single = bool(puppy._use_single_slot(tool))
        return datum_cal_z, single

    def _store(self, tool, value, single):
        """Write a squish value into the correct slot and stage SAVE_CONFIG."""
        configfile = self.printer.lookup_object('configfile')
        if single:
            self.z_offsets_single[tool] = value
            self.single_explicit[tool] = True
            key = 't%d_z_offset_single' % tool
        else:
            self.z_offsets[tool] = value
            key = 't%d_z_offset' % tool
            # No need to touch the single slot: while untuned it is derived
            # live in get_z_offset(), so it follows this automatically.
        configfile.set(self.name, key, "%.4f" % value)
        return key

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
        datum_cal_z, single = self._get_datum_and_mode(tool)

        # Read total Z gcode offset from Klipper
        gcode_move = self.printer.lookup_object("gcode_move")
        total_z = gcode_move.homing_position[2]

        # Applied offset = -(cal_z - datum_cal_z) + per_tool_z + user_adjustment
        # so   new_per_tool_z = total_z + (cal_z - datum_cal_z)
        # Homing on T0 makes datum_cal_z 0 and this is the original formula.
        new_z_offset = total_z + (cal_z - datum_cal_z)

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

        old_z_offset = self.get_z_offset(tool, single=single)
        key = self._store(tool, new_z_offset, single)

        gcmd.respond_info(
            "T%d %s: %.4f (was %.4f)\n"
            "The SAVE_CONFIG command will update the printer config file\n"
            "and restart the printer."
            % (tool, key, new_z_offset, old_z_offset))

    def cmd_SAVE_TOOL_Z_OFFSET(self, gcmd):
        """Save Z offset for a specific tool (or active tool).

        Usage: SAVE_TOOL_Z_OFFSET [TOOL=0] [Z=0.035] [SINGLE=0|1]
        If Z is omitted, captures current adjustment like Z_OFFSET_APPLY_PROBE.
        SINGLE selects which slot to write; it defaults to the mode the printer
        is actually in, so a babystep saved during a single-tool print lands in
        the single-tool slot without the user having to think about it.
        """
        tool = gcmd.get_int('TOOL', -1)
        if tool < 0:
            # Use active tool
            tool, cal_z = self._get_active_tool_and_cal_offset()
            if tool < 0:
                gcmd.respond_info("No tool active and no TOOL= specified")
                return
        else:
            puppy = self.printer.lookup_object('puppy_bootloader', None)
            if puppy:
                cal_z = puppy.tool_offsets.get(tool, (0., 0., 0.))[2]
            else:
                cal_z = 0.

        datum_cal_z, mode_single = self._get_datum_and_mode(tool)
        single = bool(gcmd.get_int('SINGLE', 1 if mode_single else 0))

        z_val = gcmd.get_float('Z', None)
        if z_val is not None:
            new_z_offset = z_val
        else:
            # Capture from current gcode offset
            gcode_move = self.printer.lookup_object("gcode_move")
            total_z = gcode_move.homing_position[2]
            new_z_offset = total_z + (cal_z - datum_cal_z)

        # Clamp
        new_z_offset = max(self.z_min, min(self.z_max, new_z_offset))

        old_z_offset = self.get_z_offset(tool, single=single)
        key = self._store(tool, new_z_offset, single)

        gcmd.respond_info(
            "T%d %s: %.4f (was %.4f)\n"
            "Run SAVE_CONFIG to persist."
            % (tool, key, new_z_offset, old_z_offset))

    def cmd_SET_TOOL_Z_OFFSET(self, gcmd):
        """Set Z offset for a tool without staging for SAVE_CONFIG.

        Usage: SET_TOOL_Z_OFFSET TOOL=0 Z=0.035 [SINGLE=0|1]
        Updates in memory only. Use SAVE_TOOL_Z_OFFSET to persist.
        """
        tool = gcmd.get_int('TOOL')
        z_val = gcmd.get_float('Z')
        z_val = max(self.z_min, min(self.z_max, z_val))
        _, mode_single = self._get_datum_and_mode(tool)
        single = bool(gcmd.get_int('SINGLE', 1 if mode_single else 0))
        old = self.get_z_offset(tool, single=single)
        if single:
            self.z_offsets_single[tool] = z_val
            self.single_explicit[tool] = True
            label = 'z_offset_single'
        else:
            self.z_offsets[tool] = z_val
            label = 'z_offset'
        gcmd.respond_info("T%d %s: %.4f (was %.4f)"
                          % (tool, label, z_val, old))

    def cmd_GET_TOOL_Z_OFFSETS(self, gcmd):
        """Display all per-tool Z offsets, both frames."""
        datum_cal_z, mode_single = self._get_datum_and_mode()
        lines = ["Per-tool Z offsets      multi    single",
                 "                     (T0 datum) (self-homed)"]
        for tool in range(5):
            multi = self.z_offsets.get(tool, 0.)
            sing = self.get_z_offset(tool, single=True)
            mark = '' if self.single_explicit.get(tool, False) \
                else '  (inherited = -cal_z + squish)'
            lines.append("  T%d:            %8.4f  %8.4f%s"
                         % (tool, multi, sing, mark))
        lines.append("")
        lines.append("Active mode: %s"
                     % ("SINGLE-TOOL" if mode_single else "MULTI-TOOL"))
        gcmd.respond_info('\n'.join(lines))


def load_config(config):
    return ToolOffsets(config)
