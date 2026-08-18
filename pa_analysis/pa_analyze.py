#!/usr/bin/env python3
"""Turn a KlipperXL PA recording into a Pressure Advance value.

Runs CNC Kitchen's analyser (PrusaPATuner) against the trace captured by
dwarf_pa_tuner.py. Deliberately a SEPARATE process in its own venv:
  * scipy must never be able to disturb klippy-env, which runs the printer
  * Stefan's analyser is AGPL-3.0 while KlipperXL is GPL-3.0, so keeping it as a
    detached companion piece keeps the licences clean

INPUT  : /tmp/pa_recording.csv written by STOP_PA_RECORDING. Its header carries the
         sweep parameters as `# param NAME=VALUE`, so the plan reconstructed here
         cannot drift from what the machine actually ran.
OUTPUT : prints a summary and writes JSON to /tmp/pa_result.json

The analyser needs a SweepPlan describing where each K segment sits in time. We get
one by calling build_sweep() with SweepParams that mirror the macro, then discarding
the gcode it also produces -- only the plan is used.

NOTE ON UNITS: the macro parameterises flow VOLUMETRICALLY (mm^3/s), because that is
what Richard's actual stock runs did. SweepParams wants mm/s of filament, so convert
with the filament cross-section.
"""
import json
import math
import os
import sys

import numpy as np

# Resolve the vendored analyser relative to THIS file, never to a fixed home
# directory -- the install path differs per user and per distro.
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from prusa_pa_tuner.analysis import analyse_sweep          # noqa: E402
from prusa_pa_tuner.gcode_gen import SweepParams, build_sweep  # noqa: E402

CSV = sys.argv[1] if len(sys.argv) > 1 else "/tmp/pa_recording.csv"
OUT = "/tmp/pa_result.json"


def read_csv(path):
    params = {}
    t, g = [], []
    with open(path) as f:
        for line in f:
            line = line.strip()
            if line.startswith("# param "):
                k, _, v = line[len("# param "):].partition("=")
                params[k.strip()] = v.strip()
                continue
            if line.startswith("#") or line.startswith("time_s") or not line:
                continue
            a, _, b = line.partition(",")
            t.append(float(a))
            g.append(float(b))
    return params, np.asarray(t, dtype=float), np.asarray(g, dtype=float)


def f(params, key, default):
    try:
        return float(params[key])
    except (KeyError, ValueError, TypeError):
        return default


def main():
    params, force_t, force_y = read_csv(CSV)
    print("recording: %d samples, %.2fs, %.1f Hz"
          % (len(force_t), force_t[-1], (len(force_t) - 1) / force_t[-1]))
    print("params from header: %s" % (params,))

    start_k = f(params, "START_K", 0.02)
    end_k = f(params, "END_K", 0.07)
    step_k = f(params, "STEP_K", 0.002)
    cycles = int(f(params, "CYCLES", 10))
    slow_flow = f(params, "SLOW_FLOW", 3.0)
    fast_flow = f(params, "FAST_FLOW", 14.0)
    slow_half = f(params, "SLOW_HALF_S", 2.0)
    fast_half = f(params, "FAST_HALF_S", 0.714286)
    warmup = f(params, "WARMUP_FACTOR", 5.0)
    osc = f(params, "OSC_MM", 1.0)
    accel = f(params, "ACCEL", 5000.0)
    zmark = f(params, "Z_MARKER_MM", 2.0)
    baseline = f(params, "BASELINE_S", 2.0)
    fil_d = f(params, "FILAMENT_D", 1.75)
    temp = f(params, "TEMP", 215.0)

    # volumetric -> mm/s of filament
    area = math.pi * (fil_d / 2.0) ** 2
    slow_mm_s = slow_flow / area
    fast_mm_s = fast_flow / area

    n_k = int(round((end_k - start_k) / step_k)) + 1
    k_values = tuple(round(start_k + i * step_k, 6) for i in range(n_k))
    print("K values: %d from %.4f to %.4f" % (len(k_values), k_values[0], k_values[-1]))
    print("flow: slow %.4f mm/s (%.1f mm^3/s), fast %.4f mm/s (%.1f mm^3/s)"
          % (slow_mm_s, slow_flow, fast_mm_s, fast_flow))

    sp = SweepParams(
        nozzle_temp=temp,
        preheat_temp=temp + 10.0,
        filament_diameter=fil_d,
        slow_feed_mm_s=slow_mm_s,
        fast_feed_mm_s=fast_mm_s,
        slow_half_s=slow_half,
        fast_half_s=fast_half,
        cycles_per_K=cycles,
        accel_mm_s2=accel,
        K_values=k_values,
        purge_x=30.0, purge_y=30.0, purge_z=80.0,
        coupled_dx_mm=osc, coupled_dy_mm=0.0, coupled_dz_mm=0.0,
        first_slow_leg_factor=warmup,
        z_marker_lift_mm=zmark,
        baseline_dwell_s=baseline,
    )
    plan = build_sweep(sp)          # gcode discarded; we only want plan timings

    # We have no pos_x/pos_z streams (those were UDP metrics on stock), so let the
    # analyser locate sweep_t0 from the force signal itself.
    res = analyse_sweep(
        sweep_t0=0.0,
        force_t=force_t,
        force_y=force_y,
        plan=plan,
        auto_detect_t0=True,
        z_marker_lift_mm=zmark,
    )

    print("\n===== RESULT =====")
    out = {}
    for name in dir(res):
        if name.startswith("_"):
            continue
        val = getattr(res, name)
        if callable(val):
            continue
        if isinstance(val, np.ndarray):
            out[name] = "<array len=%d>" % (len(val),)
            print("  %-24s <array len=%d>" % (name, len(val)))
        elif isinstance(val, (int, float, str, bool)) or val is None:
            out[name] = val
            print("  %-24s %s" % (name, val))
        else:
            # Flatten FitResult-like objects into the JSON. Without this the only
            # number that survives is bd_k_opt, and the per-method k_opt + r_squared
            # -- which is how you tell a real fit from noise (phase_fit returned a
            # confident-looking 0.0652 at r2=0.008 on the first sweep) -- is lost.
            sub = {}
            for a in dir(val):
                if a.startswith("_"):
                    continue
                v2 = getattr(val, a)
                if callable(v2) or isinstance(v2, np.ndarray):
                    continue
                sub[a] = v2
            out[name] = sub if sub else str(type(val))
            print("  %-24s %s" % (name, type(val).__name__))

    # The analyser explains itself here -- when bd_k_opt comes back None this is
    # where it says what it could not do.
    notes = getattr(res, "notes", None) or []
    print("\n===== NOTES (%d) =====" % len(notes))
    for nline in notes:
        print("  %s" % (nline,))

    for fitname in ("phase_fit", "integral_fit", "integral_legacy_fit",
                    "bd_signed_rise_fit", "bd_signed_fall_fit"):
        fit = getattr(res, fitname, None)
        if fit is None:
            continue
        print("\n----- %s -----" % fitname)
        for a in dir(fit):
            if a.startswith("_"):
                continue
            v = getattr(fit, a)
            if callable(v):
                continue
            if isinstance(v, np.ndarray):
                print("  %-20s <array len=%d>" % (a, len(v)))
            else:
                print("  %-20s %s" % (a, v))

    perk = getattr(res, "per_k", None) or []
    print("\n===== PER-K (%d) =====" % len(perk))
    for row in perk[:12]:
        bits = []
        for a in dir(row):
            if a.startswith("_"):
                continue
            v = getattr(row, a)
            if callable(v) or isinstance(v, np.ndarray):
                continue
            bits.append("%s=%s" % (a, v))
        print("  " + ", ".join(bits))

    wins = getattr(res, "windows", None) or []
    print("\nwindows: %d" % len(wins))

    with open(OUT, "w") as fh:
        json.dump(out, fh, indent=2, default=str)
    print("\nwritten to %s" % OUT)


if __name__ == "__main__":
    main()
