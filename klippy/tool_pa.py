# KlipperXL - Per-tool Pressure Advance storage (XLPATuner Road B)
#
# Provides a [tool_pa] config section holding one K per tool, persisted via
# SAVE_CONFIG. Deliberately a close mirror of KlipperXL's own tool_offsets.py so it
# behaves the way the rest of this machine already does.
#
# THE POINT: KlipperXL runs a SINGLE [extruder] that follows the active tool, so
# "the tool's K" is just whatever SET_PRESSURE_ADVANCE was last given. Storing K per
# tool here, and re-applying it on every tool pick, is what turns that into real
# per-tool Pressure Advance -- five tools, five materials, each with its own K.
# That is the multi-tool case CNC Kitchen's README says nobody handles.
#
# PER-ROLL RE-TUNING (his spec): re-running the sweep for a tool simply overwrites
# that tool's K. An optional FILAMENT= label is stored alongside purely as a record
# of what the value was tuned against, so a later "what is T2 set for?" is answerable.
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
import logging

N_TOOLS = 5


class ToolPA:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.name = config.get_name()
        self.gcode = self.printer.lookup_object('gcode')

        # Sanity bounds. Pressure Advance is a time constant in seconds; anything
        # outside this is a typo or a bad fit, not a real value. Refuse rather than
        # quietly wreck a print.
        self.k_min = config.getfloat('k_min', 0.)
        self.k_max = config.getfloat('k_max', 1.0)

        self.pa = {}
        self.label = {}
        for tool in range(N_TOOLS):
            val = config.getfloat('t%d_pa' % tool, 0.)
            if val < self.k_min or val > self.k_max:
                logging.warning("tool_pa: T%d pa %.4f out of bounds (%.2f..%.2f)"
                                " - clamping" % (tool, val, self.k_min, self.k_max))
                val = max(self.k_min, min(self.k_max, val))
            self.pa[tool] = val
            self.label[tool] = config.get('t%d_label' % tool, '')

        self.gcode.register_command(
            'SAVE_TOOL_PA', self.cmd_SAVE_TOOL_PA,
            desc="Save Pressure Advance for a tool and stage it for SAVE_CONFIG")
        self.gcode.register_command(
            'SET_TOOL_PA', self.cmd_SET_TOOL_PA,
            desc="Set a tool's Pressure Advance in memory only (no SAVE_CONFIG)")
        self.gcode.register_command(
            'GET_TOOL_PA', self.cmd_GET_TOOL_PA,
            desc="Show stored per-tool Pressure Advance values")
        self.gcode.register_command(
            'APPLY_TOOL_PA', self.cmd_APPLY_TOOL_PA,
            desc="Apply a tool's stored PA to the extruder now")

        # Re-apply the tool's K on every pick. Done by WRAPPING the existing T0-T4
        # commands rather than editing puppy_bootloader.py, so this whole feature
        # stays additive and reversible -- delete the file, remove [tool_pa], gone.
        #
        # This is a supported operation, not a hack: gcode.py:133-141 shows
        # register_command(cmd, None) *returns the previous handler* and unregisters
        # it, so we can re-register a wrapper that calls the original first.
        # puppy registers T0-T4 in its config phase (puppy_bootloader.py:1958-1960),
        # so we must wrap at klippy:ready, once those exist.
        self.printer.register_event_handler("klippy:ready", self._wrap_tool_commands)
        # Apply the ALREADY-COUPLED tool's K at startup. Without this the tuned value
        # only takes effect on the first tool change, so a single-tool print that
        # never issues a T command would silently run on [extruder] pressure_advance
        # instead. Observed live: after SAVE_CONFIG the stored T0=0.0399 was correct
        # but the extruder was still on the config default of 0.025.
        # Deferred a few seconds because puppy determines active_tool during its own
        # startup, so it is not reliable at the instant klippy:ready fires.
        self.printer.register_event_handler("klippy:ready", self._schedule_initial)

    def _schedule_initial(self):
        reactor = self.printer.get_reactor()
        reactor.register_callback(self._apply_initial,
                                  reactor.monotonic() + 5.0)

    def _apply_initial(self, eventtime):
        try:
            puppy = self.printer.lookup_object('puppy_bootloader', None)
            if puppy is None:
                return
            tool = puppy.active_tool
            if tool is None or tool < 0 or not getattr(puppy, 'tool_picked', False):
                return
            k = self.get_pa(tool)
            if k is None:
                return
            self.gcode.run_script_from_command(
                "SET_PRESSURE_ADVANCE ADVANCE=%.4f" % (k,))
            logging.info("tool_pa: applied T%d startup pressure advance %.4f"
                         % (tool, k))
        except Exception as e:
            logging.warning("tool_pa: startup PA apply failed: %s" % (e,))

        logging.info("tool_pa: loaded %s" % (
            ', '.join('T%d=%.4f' % (t, v) for t, v in sorted(self.pa.items())),))

    def _make_tool_wrapper(self, tool, prev):
        def handler(gcmd):
            prev(gcmd)                       # the real tool change happens first
            k = self.get_pa(tool)
            if k is None:
                return                       # never tuned -- leave PA alone
            self.gcode.run_script_from_command(
                "SET_PRESSURE_ADVANCE ADVANCE=%.4f" % (k,))
        return handler

    def _wrap_tool_commands(self):
        wrapped = []
        for tool in range(N_TOOLS):
            name = 'T%d' % tool
            try:
                prev = self.gcode.register_command(name, None)
            except Exception as e:
                logging.warning("tool_pa: could not take over %s: %s" % (name, e))
                continue
            if prev is None:
                # Nothing was registered -- do not invent a T command of our own,
                # that would be a behaviour change rather than a recall hook.
                logging.info("tool_pa: %s not registered, skipping" % (name,))
                continue
            self.gcode.register_command(
                name, self._make_tool_wrapper(tool, prev),
                desc="Select tool %d (re-applies its stored pressure advance)" % tool)
            wrapped.append(name)
        if wrapped:
            logging.info("tool_pa: PA recall attached to %s" % (', '.join(wrapped),))

    # ---- API used by the tool-pick recall hook -------------------------------
    def get_pa(self, tool):
        """K for a tool, or None if that tool has never been tuned.

        Returns None rather than 0.0 for untuned tools: 0.0 is a LEGITIMATE PA value
        (it means "no advance"), so the caller must be able to tell "deliberately
        zero" from "never measured" and leave the extruder alone in the latter case.
        """
        v = self.pa.get(tool)
        if v is None:
            return None
        if not self.label.get(tool) and v == 0.:
            return None
        return v

    def get_status(self, eventtime=None):
        st = {'k_min': self.k_min, 'k_max': self.k_max}
        for t in range(N_TOOLS):
            st['t%d_pa' % t] = self.pa.get(t, 0.)
            st['t%d_label' % t] = self.label.get(t, '')
        return st

    def _check_tool(self, gcmd, tool):
        if tool < 0 or tool >= N_TOOLS:
            raise gcmd.error("tool_pa: TOOL must be 0-%d" % (N_TOOLS - 1,))

    def _check_k(self, gcmd, k):
        if k < self.k_min or k > self.k_max:
            raise gcmd.error("tool_pa: K=%.4f outside %.2f..%.2f - refusing"
                             % (k, self.k_min, self.k_max))

    # ---- commands ------------------------------------------------------------
    def cmd_SAVE_TOOL_PA(self, gcmd):
        tool = gcmd.get_int('TOOL')
        k = gcmd.get_float('K')
        label = gcmd.get('FILAMENT', '')
        self._check_tool(gcmd, tool)
        self._check_k(gcmd, k)
        old = self.pa.get(tool, 0.)
        self.pa[tool] = k
        if label:
            self.label[tool] = label
        configfile = self.printer.lookup_object('configfile')
        configfile.set(self.name, 't%d_pa' % tool, "%.4f" % k)
        if label:
            configfile.set(self.name, 't%d_label' % tool, label)
        gcmd.respond_info(
            "T%d pressure advance: %.4f (was %.4f)%s\n"
            "Run SAVE_CONFIG to persist."
            % (tool, k, old, (" [%s]" % label) if label else ""))

    def cmd_SET_TOOL_PA(self, gcmd):
        tool = gcmd.get_int('TOOL')
        k = gcmd.get_float('K')
        self._check_tool(gcmd, tool)
        self._check_k(gcmd, k)
        old = self.pa.get(tool, 0.)
        self.pa[tool] = k
        gcmd.respond_info("T%d pressure advance: %.4f (was %.4f) - memory only"
                          % (tool, k, old))

    def cmd_APPLY_TOOL_PA(self, gcmd):
        tool = gcmd.get_int('TOOL', -1)
        if tool < 0:
            puppy = self.printer.lookup_object('puppy_bootloader', None)
            tool = puppy.active_tool if puppy is not None else -1
            if tool is None or tool < 0:
                raise gcmd.error("APPLY_TOOL_PA: no tool active and no TOOL= given")
        self._check_tool(gcmd, tool)
        k = self.get_pa(tool)
        if k is None:
            gcmd.respond_info("T%d has no tuned PA - leaving the extruder alone"
                              % (tool,))
            return
        self.gcode.run_script_from_command(
            "SET_PRESSURE_ADVANCE ADVANCE=%.4f" % (k,))
        gcmd.respond_info("Applied T%d pressure advance %.4f" % (tool, k))

    def cmd_GET_TOOL_PA(self, gcmd):
        lines = ["Per-tool pressure advance:"]
        for t in range(N_TOOLS):
            k = self.get_pa(t)
            if k is None:
                # Print no number at all. 0.0 is a legitimate K, so showing it for an
                # untuned tool invites reading "T3 is tuned to zero" -- which is wrong;
                # T3 simply leaves whatever PA is already set alone.
                lines.append("  T%d:   --      (never tuned)" % (t,))
                continue
            lab = self.label.get(t, '')
            lines.append("  T%d: %.4f%s"
                         % (t, k, ("  [%s]" % lab) if lab else ""))
        gcmd.respond_info('\n'.join(lines))


def load_config(config):
    return ToolPA(config)
