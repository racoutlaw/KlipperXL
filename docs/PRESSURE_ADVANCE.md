# Automatic Per-Tool Pressure Advance

KlipperXL can measure Pressure Advance **for each tool independently**, using the
Dwarf's own loadcell as a back-pressure sensor. Click a button, wait about six
minutes, and that tool has its own K value — measured, not guessed.

Each of the five tools stores its own K. On a tool change, that tool's value is
applied automatically. **This is not one PA value shared across the toolhead
group** — every tool is measured and stored separately, and the machine switches
between them as it prints.

---

## Why per-tool, and not one value for all five

The tools are nominally identical: same Dwarf, same hotend, same nozzle size.
The reasonable expectation is that one number covers all of them.

Measured on a 5-tool XL, **same filament from the same spool, same nozzle
diameter, same temperature**, only the tool changed:

| Tool | Measured K (PLA @ 215 °C) |
|------|---------------------------|
| T0   | 0.0367 |
| T1   | 0.0552 |
| T2   | 0.0647 |

That is a **77 % spread** between the lowest and highest. Using T0's value on T2
under-compensates it badly; using T2's value on T0 over-compensates. A single
shared number is wrong for at least two tools no matter which one you pick.

Extruder gear wear, PTFE liner friction, heatbreak assembly tolerance and
idler-tension differences all land on the same knob. On a single-tool printer
they get folded into one calibration and never noticed. On a five-tool machine
they are five different printers sharing a frame.

So: measure each tool. The machine will remember.

> Upstream PrusaPATuner states that multi-tool machines are not handled. This is
> the part KlipperXL adds.

---

## What you need

| Requirement | Notes |
|---|---|
| KlipperXL firmware built from this repo | The MCU loadcell streaming support is in `src/modbus_stm32f4.c` — a firmware from before this release cannot do it |
| `~/pa_analysis_env` virtualenv | numpy + scipy, **not** in `klippy-env` |
| Filament loaded in the tool being tuned | ~4 g per run, extruded into free air |
| Nothing printing | The sweep homes the machine |

---

## Install

The install script does all of this. Manual equivalent:

```bash
# 1. Klipper extras
cp ~/KlipperXL/klippy/dwarf_pa_tuner.py ~/klipper/klippy/extras/
cp ~/KlipperXL/klippy/tool_pa.py        ~/klipper/klippy/extras/

# 2. Configs
cp ~/KlipperXL/config/pa_tuner.cfg ~/printer_data/config/
cp ~/KlipperXL/config/pa_menu.cfg  ~/printer_data/config/

# 3. Analyser + its own venv (scipy stays out of klippy-env on purpose)
cp -r ~/KlipperXL/pa_analysis ~/pa_analysis
python3 -m venv ~/pa_analysis_env
~/pa_analysis_env/bin/pip install -r ~/pa_analysis/requirements.txt
```

Then in `printer.cfg`, **above** the `#*# <--------- SAVE_CONFIG --------->`
marker:

```ini
[include pa_tuner.cfg]
[include pa_menu.cfg]

[dwarf_pa_tuner]
[tool_pa]
```

Restart Klipper. Five `PA_TUNE_T0` … `PA_TUNE_T4` buttons appear under Macros.

> **Config sections must go above the SAVE_CONFIG marker.** Klipper rewrites
> everything below it, and anything you put there will be silently discarded.

---

## Using it

### The short version

Load filament in the tool, click **`PA_TUNE_T1`**, wait ~6 minutes, then run
`SAVE_CONFIG`.

Each button drops down a small form:

| Field | Default | Meaning |
|---|---|---|
| `TEMP` | `215` | Nozzle temperature (PLA). PETG ≈ 240 |
| `FILAMENT` | `PLA` | A label stored with the K, so you know what it was measured on |
| `SAVECONFIG` | `0` | `1` = persist and restart automatically |

For PETG in tool 3: set `TEMP=240`, `FILAMENT=PETG`, press Send.

### What happens

1. Homes the machine **cold** — a failure must never leave a hot nozzle behind
2. Picks the tool (after homing, never before — see the note below)
3. Heats, records a static baseline
4. Sweeps K from 0.02 to 0.12, oscillating 1 mm in X while alternating slow
   (3 mm³/s) and fast (14 mm³/s) extrusion, capturing loadcell pressure at 300 Hz
5. Drops the heater, analyses in the background, applies the winning K to that tool
6. Prints a reminder to run `SAVE_CONFIG`

About 4 g of filament is extruded into free air at Z80 and drips onto the
unheated bed. This is normal and is what the stock tool does too. Clean it off.

### It does not auto-save, deliberately

The K goes live in memory immediately, so the tool is already using it and you
can print a test before committing. If the number looks wrong, simply never save
it — a restart discards it. Pass `SAVECONFIG=1` if you want it hands-off.

---

## Commands

| Command | What it does |
|---|---|
| `PA_TUNE_T0` … `PA_TUNE_T4` | Everything, end to end, for one tool |
| `GET_TOOL_PA` | Show all five stored values and their filament labels |
| `SET_TOOL_PA TOOL= K=` | Set a tool's K in memory |
| `SAVE_TOOL_PA TOOL= K= [LABEL=]` | Store a tool's K |
| `APPLY_TOOL_PA [TOOL=]` | Re-apply a tool's stored K |
| `CALIBRATE_PA [TOOL= TEMP= …]` | Sweep only — records, analyses nothing |
| `CALIBRATE_PA_FINE` | 0.002 step. **Narrow the range first** or it runs ~23 min |
| `ANALYZE_PA [TOOL= SAVE=1 …]` | Analyse the last recording |
| `START_PA_RECORDING` / `STOP_PA_RECORDING` | Raw loadcell capture |

`GET_TOOL_PA` output:

```
Per-tool pressure advance:
  T0: 0.0367  [PLA]
  T1: 0.0552  [PLA]
  T2: 0.0647  [PLA]
  T3:   --      (never tuned)
  T4:   --      (never tuned)
```

A never-tuned tool prints no number at all, because `0.0` is a legitimate K and
showing it would imply the tool was measured and came out at zero. Untuned tools
are left alone on a tool change rather than being forced to zero.

---

## Getting a better number

**Coarse first, then fine, on a narrow band.** The default 6-minute sweep tells
you roughly where K sits. To refine it:

```
CALIBRATE_PA_FINE TOOL=2 TEMP=215 START_K=0.05 END_K=0.08
```

That is 16 values at 0.002 resolution in about 7 minutes, concentrated where it
matters — far better than a 23-minute full-range fine sweep.

**Keep the optimum inside the swept range.** This is the single biggest source of
a wrong answer. When the true K lies outside the window, every estimator
extrapolates past the end of the data in its own way and they disagree. Measured
on one tool, same filament, only the range changed:

| Range | `bd_k_opt` | `integral_fit` | `signed_rise` | spread |
|---|---|---|---|---|
| 0.02–0.07 | 0.0472 | 0.0588 | 0.0632 | 0.016 |
| 0.02–0.12 | 0.0647 | 0.0579 | 0.0655 | **0.008** |

`bd_k_opt` moved from 0.047 to 0.065 purely from widening the range — it had
been clipped at the boundary. **Estimators agreeing is the quality signal.** If
they disagree, widen the range before trusting any of them.

---

## How it works

```
  Dwarf loadcell ──MODBUS/RS485──> XLBuddy ──USB──> Pi
       300 Hz                    (streams)     dwarf_pa_tuner.py
                                                     │ CSV
                                                     v
                                          pa_analyze.py (own venv)
                                                     │ K
                                                     v
                                          tool_pa.py  ── per-tool storage
                                                     │
                                              T0..T4 ─┘ auto-apply on tool change
```

**MCU-side streaming is required, not an optimisation.** Polling the loadcell
from the host works while the machine is idle, but fails under extrusion:
Klipper's MCU query timeout fires whenever a Dwarf misses a read, and the
capture stalls. Measured on the same hardware:

| Method | Rate | Gaps over a 12-minute run |
|---|---|---|
| Host polling @ 20 ms | 110 Hz | 5 stalls |
| **MCU streaming** | **300 Hz** | **zero** |

The MCU forwards loadcell records as they arrive instead of answering
one-at-a-time queries, so no host round-trip sits in the sampling path.

**Per-tool recall** wraps the `T0`–`T4` commands and re-applies that tool's K
after the tool change completes. It requires no edits to `puppy_bootloader.py` —
it captures the previous handler via `register_command(name, None)` and calls it
first. Values live in `[tool_pa]` in the autosave block.

**The analyser runs as a detached subprocess** in its own virtualenv, for two
independent reasons: scipy must never be able to disturb the environment driving
the printer, and it keeps the AGPL-licensed analysis code out of the Klipper
process. Because it is asynchronous, `SAVE_CONFIG` is issued from inside the
module once the result actually lands — a macro would reach `SAVE_CONFIG` while
analysis was still running and persist nothing.

---

## Troubleshooting

**"could not start analyser"** — the venv is missing or somewhere else:
```bash
ls ~/pa_analysis_env/bin/python ~/pa_analysis/pa_analyze.py
```
Point `[dwarf_pa_tuner]` at them with `analysis_python` / `analysis_script`.

**No K produced / every K rejected** — the analyser discards any K with fewer
than 4 included segments, and inclusion runs 67–85 %. `CYCLES=6` is a measured
floor, not a preference; at `CYCLES=4` every K is rejected. Do not lower it.

**Estimators disagree** — the optimum is probably outside the swept range. Widen
it. See above.

**Zero samples captured** — the loadcell is per-Dwarf and only streams for the
*coupled* tool. The macro handles this, but if you are driving the recorder by
hand, pick the tool first.

**The measured K was saved to the wrong tool** — `G28` must come *before* the
tool pick. `[homing_override]` couples T0 to run the Z probe, so picking T1 and
then homing silently switches back to T0: the sweep records T0's loadcell while
the result is stored as T1's K. It looks perfectly consistent and is wrong. The
shipped macro homes first for exactly this reason.

**Values vanish after a restart** — they were never saved. Run `SAVE_CONFIG`.

---

## Credit

The analysis mathematics — turning a pressure trace into an optimal K — is
**Stefan Hermann's (CNC Kitchen)** work, from
[PrusaPATuner](https://github.com/CNCKitchen/PrusaPATuner), used here unmodified
under AGPL-3.0-or-later. KlipperXL did not reinvent it and does not claim it.

KlipperXL contributes the parts around it: MCU-side loadcell streaming over
MODBUS, the host recorder, the sweep macro, and per-tool storage and recall.

See [`pa_analysis/prusa_pa_tuner/NOTICE`](../pa_analysis/prusa_pa_tuner/NOTICE)
for full attribution and licensing.

If this is useful to you, support the upstream project:
[CNC Kitchen](https://www.youtube.com/@CNCKitchen).
