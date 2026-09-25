## 2026-09-24

### FIXED: buttons on the display could wreck a running print

Touching `PA TUNE`, `LOADCELL TEST` and several other controls **while a print was
running** started the routine immediately. Nothing checked whether the machine was busy.

The worst case, hit on a real machine: `PA_TUNE` **homes Z before picking the tool**
(`homing_override` couples T0 for the loadcell probe). Mid-print that drives to bed
centre and probes down **onto the part**, re-datuming Z off the top of the print. The
job then resumes from the correct gcode position at a completely wrong Z and extrudes
into thin air. It cost an 88-minute print.

`LOADCELL_TEST` fails differently and just as badly: it is a bare passthrough to
`LOADCELL_PROBE`, which with no arguments probes straight down from wherever the nozzle
happens to be. Mid-print that is a 255 C nozzle driven into the part - or past it, into
the sheet.

- Added a shared `_ASSERT_NOT_PRINTING` macro in `config/printer.cfg`, called as the
  **first line** of every macro that homes, probes, calibrates or swaps tools:
  `CALIBRATE_PA`, `CALIBRATE_PA_FINE`, `CALIBRATE_INPUT_SHAPER`, `CALIBRATE_TOOLS`,
  `TEST_RESONANCES_X`, `TEST_RESONANCES_Y`, `LOADCELL_TEST`, `DOCK_CAL`, `PRUSA_Z_LEVEL`,
  `PARK_TOOL`, `STALLGUARD_Z_ALIGN`
- `paused` is refused as well as `printing`. Pausing does not take the part off the bed
- `_PA_TUNE_RUN` also carries its own guard with a message naming the Z-probe hazard

**Guard the command, not the button.** The first attempt guarded only `_PA_TUNE_RUN`,
the wrapper behind the `PA TUNE` buttons. `CALIBRATE_PA` is reachable on its own, and
one click ran straight past the guard and started a sweep on a live print. The refusal
now sits on the command that does the damage, so every caller inherits it.

**What is deliberately NOT guarded**, because these must move during a print and
guarding them breaks every job: `START_PRINT`, `END_PRINT`, `CANCEL_PRINT`, `PAUSE`,
`RESUME`, `_MELT_RETRACT_ON_CANCEL`, `_PARK_IF_TOOL_PICKED`, `_TOOLHEAD_PARK_PAUSE_CANCEL`,
`_CLIENT_EXTRUDE`, `_CLIENT_LINEAR_MOVE` and the timelapse macros. `START_PRINT` in
particular homes **and** probes while `print_stats` already reads `printing`.

The guard is also **not** in `loadcell_probe.py`. `BED_MESH_CALIBRATE` legitimately
calls the probe during `START_PRINT`, so a module-level refusal would break meshing on
every print. The user-facing macro is the correct level: the primitive has a valid
mid-print job, the button does not.

Naming is not a reliable filter, either. Eight of the eleven carry `TEST` or `CALIBRATE`
in the name - but `PARK_TOOL`, `PRUSA_Z_LEVEL` and `DOCK_CAL` do not, and `PARK_TOOL` is
a single innocuous-looking button that ejects the toolhead you are printing with.

### FIXED: a fan problem could cancel the print

`cmd_M106` raised on a failed MODBUS fan write, and a raise aborts the running job.
Seen live: the MCU repeated one MODBUS response nine times over four seconds, the query
returned `None`, and the print died with `buffer_time` already drained to zero.

Stock Prusa structurally cannot do this. `hwio_XLBuddy.cpp:398` calls
`Fans::print(active_extruder).set_pwm(ulValue)` and throws the returned bool away, as
does every other caller of `CFanCtl3Wire::set_pwm()`. No stock path lets a fan write
cancel a job.

There were **two** raises in that path, not one - the `dwarf not in booted_dwarfs` check
killed a print just as dead if a Dwarf dropped off the bus. Both are handled:

- **bare `M106`** - the only form a slicer emits, so the only one that can appear
  mid-print - warns and returns. The print continues
- **explicit `M106 T<n>` / `P<n>`** still raises. That form is hand- or macro-written,
  so a bad tool number is an authoring error worth surfacing, not swallowing
- warnings are **edge-triggered** via the new `_fan_warn_once()`: one line when it
  fails, one `fan write recovered` when it comes back, silence in between. PrusaSlicer
  emits 200+ `M106` per print, so warning on every failure would flood the console over
  the same RS485 bus that is already struggling

A missed fan update costs cooling on one layer. Killing the job costs the whole part.

---

## 2026-09-08

### FIXED: cancelling a print never emptied the melt zone

- `END_PRINT` pulls `E-20` and records it, so the next print on that tool gets it
  back. **Cancelling did neither** — it retracted 1 mm and parked. A cancelled
  print stops and parks exactly like a finished one, so it leaves the same 20 mm
  of void behind, and the next print on that tool paid for it
- The mechanism to do it already existed and had **never run**.
  `_CLIENT_VARIABLE` sets `user_cancel_macro: "_MELT_RETRACT_ON_CANCEL"`, but
  that hook only fires from **Mainsail's** `CANCEL_PRINT` — and
  `config/macros/print_macros.cfg` is included *after* `mainsail.cfg` and defines
  its own `CANCEL_PRINT`, which replaces Mainsail's outright. The hook was never
  consulted, so the macro behind it was dead code
- `CANCEL_PRINT` now calls `_MELT_RETRACT_ON_CANCEL` explicitly, **before** the
  heaters are turned off. The guard inside it checks actual nozzle temperature,
  and a nozzle below `min_extrude_temp` cannot retract at all — so ordering here
  is load-bearing
- The old 1 mm retract is removed. The melt retract already pulls 20 mm and books
  exactly 20; another 1 mm would put the filament 21 mm back while only 20 was
  recorded, leaving the next print 1 mm short — the same class of bug the
  melt-zone recovery exists to fix, just smaller

---

## 2026-09-05

### FIXED: the part cooling fan never turned off on a parked tool

- A tool parked mid-print kept its part cooling fan running for the **rest of the
  print, and after it**. On a five-tool job that meant four fans blowing into
  their docks for hours
- The fan speed lives in each Dwarf's own register (`0xE002`). Nothing on the
  Klipper side ever cleared it, so once a tool was parked at speed it stayed
  there until something else happened to address that tool
- The root cause was that this firmware had five fan registers and **no concept
  of "the print fan"**. Slicers only ever emit a bare `M106 S<x>` because they
  assume a single fan — one real five-tool file here contained **699 `M106` and
  zero `M107`** across 45 toolchanges. A bare `M106` was routed to whichever tool
  happened to be active, and the value was then stranded on that tool forever
- Stock Prusa does not work this way. `hwio_XLBuddy.cpp` routes the machine's one
  fan value to whichever tool is picked:

  ```cpp
  case MARLIN_PIN(FAN):
      Fans::print(active_extruder).set_pwm(ulValue);
  ```

  and `toolchanger.cpp` clears the outgoing tool on every change:

  ```cpp
  // Disable print fan on old dwarf, fan on new dwarf will be enabled by marlin
  Fans::print(old_dwarf->dwarf_index()).set_pwm(0);
  ```

- That model is now ported. There is one print fan speed; parking a tool zeroes
  its fan, picking a tool applies the stored speed. Marlin re-applies its value
  continuously through `active_extruder`; Klipper has no such loop, so it is
  written once at pick time
- `M106 T<n>` / `P<n>` still addresses one tool directly and deliberately does
  **not** disturb the print fan value, so a later toolchange will not carry it
  around. That is Prusa's rule too — `toolchanger.cpp` notes it uses the fanctl
  interface directly "without modifing marlin's value"
- Per-tool fans also gained a **selftest latch**, mirroring
  `CFanCtl3Wire::selftest_mode`: while a tool is latched, ordinary fan writes are
  ignored and the pre-latch speed is restored on release. This is what lets the
  tool-offset cooldown run fans on *parked* tools without a toolchange stomping
  them

### FIXED: END_PRINT left the bed hot on an XL with fewer than five tools

- `END_PRINT` and `CANCEL_PRINT` each carried five hardcoded lines:

  ```
  SET_DWARF_TEMP DWARF=1 TEMP=0
  ... through DWARF=5
  ```

- `cmd_SET_DWARF_TEMP` was the **only** Dwarf-addressing command in the module
  without a `booted_dwarfs` check — it validated the range 1-5 and then wrote.
  Every other such command guards on `if dwarf not in self.booted_dwarfs`
- On a two- or three-tool XL, the first absent Dwarf spent about a second in
  MODBUS retries, returned `False`, and raised. **A raised error aborts the rest
  of the macro**, which was confirmed by test: sending `M118 STEP-1` /
  `SET_DWARF_TEMP DWARF=9` / `M118 STEP-3` echoes `STEP-1`, reports the error,
  and never prints `STEP-3`
- So on a partial XL, `END_PRINT` died before reaching `CLEAR_BED_AREA`,
  `M140 S0`, `M84 X Y E` and `_status_complete`. **The bed stayed at print
  temperature with nothing on screen to say so.** `CANCEL_PRINT` survived only by
  luck — its `M140 S0` sits above the Dwarf block
- Replaced with `DISABLE_ALL_HOTENDS`, which iterates the Dwarfs that actually
  booted and is best-effort: a failure on one warns and the sweep continues. A
  shutdown path must never abort part way. This mirrors Marlin's
  `Temperature::disable_heaters()`, which does `HOTEND_LOOP() setTargetHotend(0, e)`
  — iterating the hotends that exist, never a hardcoded count
- `cmd_SET_DWARF_TEMP` also gained the missing `booted_dwarfs` guard
- **Order note for anyone editing `END_PRINT`:** `CLEAR_BED_AREA` must stay
  *before* `M140 S0`. `cmd_M140` only resets `bed_global_target` in its
  "no area set" branch; with an area still set it takes the adaptive branch and
  leaves the target at e.g. 60, and `CLEAR_BED_AREA` then re-heats all 16
  bedlets. Swapping them leaves the bed on

### FIXED: M106 could not parse a fractional fan speed, and it killed prints

- `cmd_M106` read its speed with `gcmd.get_int('S', 255)`. PrusaSlicer emits
  fractional fan speeds — `M106 S127.5` is 50% — and `get_int` raises
  `unable to parse 127.5`, **which aborts the running print**
- OrcaSlicer only ever emitted integers, which is why this went unnoticed. One
  PrusaSlicer file here contained 217 `M106` lines with values `S127.5`,
  `S224.4`, `S229.5`, `S252.45` and `S255`
- Now uses `get_float`, clamped and rounded — which is what Klipper's own fan
  `M106` does

### FIXED: a bare M106 with no tool picked spun T0

- On startup with nothing on the carriage, `active_tool` is set to `0`
  ("default to T0 for MODBUS commands"), so it is **never negative** in the
  parked state and the old `if fan < 0: return` guard could never fire. A bare
  `M106` typed at the console with an empty carriage spun T0's fan
- Stock gives "no tool" its own dummy extruder slot —
  `MARLIN_NO_TOOL_PICKED = EXTRUDERS - 1` in `toolchanger_utils.h` — so
  `Fans::print(active_extruder)` lands on a fan object that discards the write
- Now the bare form stores the print fan speed and writes nothing while no tool
  is picked. The stored value is applied on the next pick, which is the same end
  state Marlin reaches by re-applying `fan_speed[0]` continuously

### ADDED: M142 is accepted instead of erroring

- PrusaSlicer's XL start and end gcode emit `M142 S36` (heatbreak cooling
  target), which raised `Unknown command` twice per print
- It is now accepted and logged, and **explicitly documented as not acted on**.
  The Dwarf regulates its own heatbreak fan and exposes no writable register for
  it — the only writable Dwarf registers this module has are `0xE000` (nozzle
  target) and `0xE002` (print fan); `heatbreak_temp` is read-only
- Deliberately logged rather than silently swallowed. `M302` was a no-op stub
  whose callers assumed it worked, and that cost a night of debugging

---

## 2026-08-27

### FIXED: the first perimeters of a print ran dry

- `END_PRINT` pulls `E-20` to empty the melt zone while the nozzle is still hot.
  That is correct and deliberate — but it leaves **20 mm of void** above the
  nozzle, and nothing gave it back
- The prime line only returns about **13 mm** at these bead widths (7.48 mm on
  the fat pass, 5.61 mm on the clean pass). The remaining ~7 mm was paid for by
  **the part itself**: the first perimeters extruded into void and plastic
  arrived late
- The deep retract is now **recorded per tool** in `save_variables`
  (`melt_retract_t0` … `melt_retract_t4`), and the next `START_PRINT` on that
  tool gives back exactly what was taken, then clears the flag
- The refill happens at the start of the prime line with the nozzle lifted, so
  anything that does emerge lands at the head of the sacrificial purge line
  rather than on the part, and the fat pass immediately drags away from it
- Flags are per tool and survive restarts and power-off. They are cleared by that
  tool's own `START_PRINT` recovery, by `LOAD_FILAMENT`, by `UNLOAD_FILAMENT`, or
  by autoload — so a second print, a manual load, or a fresh spool never
  double-charges it

---

## 2026-08-20

### ADDED: single-tool prints home Z with the tool that is actually printing

- Previously every print homed Z on T0 regardless of which tool was printing.
  For a single-tool job on T3 that meant heating T0, picking it, probing with it,
  parking it, then picking T3 — and accepting T0's nozzle length as the datum
- `START_PRINT` now takes a `TOOLS` parameter (how many tools the job uses). When
  `TOOLS=1`, the printing nozzle establishes Z=0 itself: no T0 heat-and-swap, and
  no reliance on the stored tool-offset to translate between them
- When `TOOLS` is absent — any file sliced before this change — it defaults to 0
  and the multi-tool path runs exactly as before
- The datum choice reaches `[homing_override]` through a macro variable rather
  than a `G28` parameter, because **`G28` cannot take `KEY=VALUE`**. It is a
  standard gcode command, so Klipper parses it with `args_r` and `PROBE_TOOL=3`
  arrives as the literal string `'=3'`, which `|int` turns into 0. Only extended
  commands get `KEY=VALUE` handling
- The variable is one-shot: the override resets it after homing, so any other
  `G28` — manual, or the internal ones inside `puppy_bootloader.py` — still homes
  on T0 as it always did

### FIXED: START_PRINT could leave the wrong tool on the bed

- Step 8 used to be `{% if TOOL != 0 %} T{TOOL} {% endif %}` — it assumed homing
  had left T0 picked
- A macro template is rendered *before* it runs, so the macro cannot observe
  which tool homing actually left on the carriage. It can only assume, and
  assuming is what put T0 on the bed for a T3 job
- The pick is now unconditional. Picking an already-picked tool short-circuits
  inside `puppy_bootloader` (it re-applies offsets and returns), so it costs
  nothing when the assumption would have been right

### CHANGED: homing acceleration now matches stock

- Klipper was homing at the printer's `max_accel`. Stock Prusa uses a dedicated
  `XY_HOMING_ACCELERATION` of **1250** (`Configuration_XL_adv.h:1340`, under
  `IMPROVE_HOMING_RELIABILITY`)
- `[homing_override]` now sets 1250 for the XY homing moves and restores the
  configured `max_accel` afterwards. `XY_HOMING_JERK 8` already matched
  `square_corner_velocity: 8`

### ADDED: filament load / unload buttons in Mainsail

- `LOAD_FILAMENT` and `UNLOAD_FILAMENT` are registered in Python and have always
  worked, but Mainsail and Fluidd only list `[gcode_macro]` sections in their
  Macros panel — so a Python-registered command is invisible there no matter how
  well it works
- New `config/filament_macros.cfg` adds `FILAMENT_LOAD`, `FILAMENT_UNLOAD` and
  `FILAMENT_SENSORS` as thin wrappers with a parameter form
- They are deliberately **not** named `LOAD_FILAMENT` / `UNLOAD_FILAMENT`:
  Klipper refuses to start if a `[gcode_macro]` claims a command name that is
  already registered, so a same-name wrapper would break the printer at config
  load
- `TOOL` defaults to 0 rather than "whatever is picked" — right for a console
  command, wrong for a button, since clicking it should never load filament into
  a tool you did not mean

## Upgrading to 2026-09-08 from an earlier version

> ### No firmware rebuild is needed
>
> Unlike the 2026-08-16 release, everything here is Python and config. Your
> XLBuddy firmware is untouched.

### 1. Copy the Python modules

```bash
cp klippy/puppy_bootloader.py ~/klipper/klippy/extras/
cp klippy/tool_offsets.py     ~/klipper/klippy/extras/
```

### 2. Copy the new config file

```bash
cp config/filament_macros.cfg ~/printer_data/config/
```

and add the include to your `printer.cfg`, next to the others:

```ini
[include filament_macros.cfg]
```

### 3. Merge the macro changes into your own printer.cfg

> ⚠ **Do not copy `config/printer.cfg` over your own.** Yours holds your MCU
> serial, your measured input shaper and Z offsets, and the `SAVE_CONFIG` block
> with your bed mesh and per-tool values. Copying the shipped file over it
> destroys all of that.

Merge these three changes by hand:

- **`END_PRINT` and `CANCEL_PRINT`** — replace the five hardcoded
  `SET_DWARF_TEMP DWARF=1..5 TEMP=0` lines with a single `DISABLE_ALL_HOTENDS`.
  In `END_PRINT`, keep `CLEAR_BED_AREA` **before** `M140 S0`
- **`CANCEL_PRINT` melt retract** — add `_MELT_RETRACT_ON_CANCEL` as the first
  thing the macro does, above `M104 S0`, and delete the old `G1 E-1 F648`. Both
  details matter: the retract needs a hot nozzle, and leaving the 1 mm in place
  pulls more than gets recorded
- **`START_PRINT` / `END_PRINT` melt-zone recovery** — the `melt_retract_t<n>`
  save/restore blocks. These need `[save_variables]`, which this config has
  already shipped for some time
- **Single-tool Z datum** — the `[gcode_macro _KXL_Z_DATUM]` container, the
  `PROBE_TOOL` / `SINGLE_MODE` lines in `[homing_override]`, and the `TOOLS`
  parameter in `START_PRINT`

Diffing the shipped `config/printer.cfg` against yours is the quickest way to
see them in context.

### 4. Restart Klipper properly

```bash
sudo systemctl restart klipper
```

> ⚠ A `RESTART` or `FIRMWARE_RESTART` from the console **does not reload Python
> modules**. If you only do that, `DISABLE_ALL_HOTENDS` will come back as an
> unknown command and `END_PRINT` will fail. It has to be the service restart.

### 5. If your slicer should use the single-tool Z datum

Add `TOOLS=` to your start gcode so `START_PRINT` knows how many tools the job
uses. In OrcaSlicer:

```
TOOLS={(is_extruder_used[0]?1:0)+(is_extruder_used[1]?1:0)+(is_extruder_used[2]?1:0)+(is_extruder_used[3]?1:0)+(is_extruder_used[4]?1:0)}
```

Leaving it out is safe — `TOOLS` defaults to 0 and the original multi-tool
homing path runs, exactly as before.

---

## 2026-08-16

### FIXED: prime line massively over-extruded, causing skipped steps

- `START_PRINT` step 10 extruded a **hardcoded `E15` / `E30`** regardless of how long the prime line actually was, and derived that length from the *part's* bounding box. A small part therefore received the same amount of filament as a large one, crammed into a much shorter line
- Compounding it, the shipped OrcaSlicer profiles set `use_relative_e_distances = 1` (as does Prusa's own slicer), but those `E15` / `E30` values were written for **absolute** E, where they would have meant "15, then 15 more". In relative mode the second pass extruded the full **30**, not 15 — so a prime line got **45 mm of filament, not 30**
- Real example from a 20.2 mm tall part: 45 mm of filament (108 mm³) into a 20 mm line produced beads roughly **6 mm and 12 mm wide** from a 0.4 mm nozzle, spaced 0.4 mm apart. The second pass landed inside the first, the nozzle plowed through its own deposit, and **the extruder skipped steps**. Correct for that geometry is about 2.2 mm of filament
- Extrusion is now computed from the actual line length, the line is clamped to the **bed** rather than the part, spacing matches the bead width, and `M83` is stated explicitly so the block is correct regardless of what mode the slicer left set

### FIXED: acceleration silently capped at 1800

- `[printer] max_accel` was **1800**, so every slicer request above it was clamped with no warning — a profile asking for 5000 got 1800. This is most of why prints ran slower than stock
- Stock Prusa XL uses 2500–4000 for print moves and 5000 for travel, against a 7000 hardware ceiling
- Now **4000**, matching stock's highest print acceleration. `max_velocity: 400` and `square_corner_velocity: 8.0` already matched stock and are unchanged

### FIXED: a clean install could not start Klipper at all

`config/printer.cfg` shipped with two includes enabled that nothing installs:

```ini
[include led_effects.cfg]   # in the repo, but the install script never copied it
[include timelapse.cfg]     # not in the repo at all
```

- Klipper refuses to start on an include it cannot find, so a fresh install stopped
  dead on the first one — before ever reaching a working printer
- `led_effects.cfg` is in the repo but `INSTALL_ON_PI.sh` never deployed it, and it
  needs the third-party `klipper-led_effect` plugin on top of that
- `timelapse.cfg` is generated by `moonraker-timelapse`, a separate KIAUH component
  that the guide never mentioned installing
- The install script only *printed* "Comment out `[include led_effects.cfg]` if not
  using side LED strips" — advice that was backwards, since the file is absent either
  way, and it said nothing about timelapse
- Both includes now ship **commented out**, each with a pointer to the guide section
  that enables it. The script's note says the same thing
- New guide section **8.3** documents installing `moonraker-timelapse`; **8.1** now
  ends with the step that uncomments the LED include

Neither is auto-installed on purpose. Cloning someone else's repo during install is
the dependency this project deliberately avoids — the same reasoning that vendored
the Pressure Advance analyser instead of fetching it.

### FIXED: input shaping shipped disabled

- `[input_shaper]` was declared but every value inside it was **commented out**, so a fresh install had the section present and doing nothing. The stock Prusa XL defaults were sitting right there as comments
- They are now **active**: `mzv 35.8` on X, `mzv 35.4` on Y
- Verified against Prusa's own source rather than assumed — `lib/Marlin/Marlin/src/feature/input_shaper/input_shaper_config.hpp`, the `PRINTER_IS_PRUSA_XL()` branch of `axis_x_default` / `axis_y_default`. Buddy is a multi-printer firmware and the other models in that file differ (CoreOne 60.0/48.0, MINI 118.2/32.8), so these are the XL's own numbers and not a shared fallback. Prusa's `damping_ratio` of 0.1 is already Klipper's default
- **This is why `max_accel` can go to 4000.** Stock runs that acceleration with shaping on; raising one without the other would be copying half the machine. Run `CALIBRATE_INPUT_SHAPER` to measure your own — `SAVE_CONFIG` overrides these with your values
- New Installation Guide section **10.9** walks through `CALIBRATE_INPUT_SHAPER` using the Dwarf accelerometers — no extra hardware needed, since `[resonance_tester]` and the accelerometer module were already configured. Pressure Advance moved to 10.10

### FIXED: extruder over-extrusion guard was disabled

- `max_extrude_cross_section` was **50.0**, which never trips. That is why the prime-line bug above produced skipped steps instead of an error
- Now **4.0** — still permissive enough for legitimate wide first-layer and purge moves, but it will catch a real mistake

### FIXED: the install script pointed at the DFU recovery path, not the normal one

Found by walking the installer and the guide side by side. `scripts/INSTALL_ON_PI.sh` and `docs/INSTALLATION_GUIDE.md` disagreed about how to flash, and the script's version was the destructive one:

- The script told users to build with **"No bootloader"**, while the guide correctly specifies **`128KiB with STM32 bootloader (0x08020200)`**. That offset is what places Klipper *after* Prusa's bootloader so the bootloader survives
- The script then finished by telling users to `dfu-util ... -s 0x08000000` — the **bootloader region**. Following it overwrites Prusa's bootloader, so USB `.bbf` flashing stops working until the bootloader is restored (section 12.5, using the shipped `firmware/recovery/bootloader-xl-2.5.0.bin`). That is documented as an advanced recovery method, and the script was handing it out as the default
- The script also never produced a `.bbf`, so there was nothing to flash by the documented method even if you wanted to

Now the script builds at `0x08020200`, packages `klipper.bbf` with `pack_fw.py` automatically, and its closing instructions are the USB-drive flash from section 6. DFU is referenced only as the fallback, with its different build offset and its consequence stated.

- The README's Safety Notes also still claimed "The XLBuddy **requires** DFU mode (physical jumper) for firmware flashing", left over from before USB `.bbf` flashing became the documented method. Corrected to state the USB path and to frame DFU as recovery-only

### README now states the one irreversible prerequisite up front

- Installing KlipperXL requires **breaking the appendix / safety seal on the XLBuddy**, which is a permanent physical modification to the mainboard. The Installation Guide has always documented this in section 1 — but the README, which is what most people read first, never mentioned it anywhere
- Hardware Requirements now lists it (and the FAT32 USB drive), with a callout stating plainly that it cannot be undone, and that everything else about KlipperXL is reversible by flashing stock firmware back. That distinction is the part worth knowing before you start rather than halfway through

### FIXED: installer robustness and accuracy

- **`set -e` could abort the whole install partway.** `python3 -m venv` fails on Raspberry Pi OS Lite (no `python3-venv` by default), which killed the script *after* configs were deployed but *before* the Makefile patch and firmware build. PA setup is now non-fatal and prints a recovery path
- **The script told users to add config sections that it had already installed.** The deployed `printer.cfg` contains the PA includes and sections; following the on-screen instruction created duplicates, which Klipper rejects at startup
- `rm -rf ~/pa_analysis` now moves the old directory to `~/pa_analysis.old` instead of deleting it
- The guide's manual install omitted `strobe_test.py`, which the script copies — anyone installing by hand silently lost the LED stroboscope module. Both lists now match exactly

### FIXED: shipped printer.cfg no longer carries one machine's calibration

- The `SAVE_CONFIG` autosave block was being published with the config, which meant every install started with **another machine's `[input_shaper]` values** (`mzv 45.2` / `mzv 23.0`) already applied. The `[input_shaper]` section directly above it says "Will be calibrated with SHAPER_CALIBRATE" — the autosave block was silently overriding exactly that
- It also shipped `dock_pos_t0`–`t4`, which are not load-bearing (`_load_dock_positions()` falls back to the same built-in Prusa defaults) but do set `dock_positions_calibrated = True` — so a fresh printer claimed its docks were calibrated when they never had been
- The block is now stripped. Klipper recreates it on your first `SAVE_CONFIG`, containing your own values

### Automatic PER-TOOL Pressure Advance tuning

- **Every tool now measures, stores and uses its own Pressure Advance value.** This is not a single PA setting shared across the toolhead group — all five tools are calibrated independently and the correct value is applied automatically on each tool change
- New `PA_TUNE_T0` … `PA_TUNE_T4` macros run the whole job unattended for one tool: home → pick tool → heat → sweep K → analyse → apply. About 6 minutes per tool
- Uses the Dwarf's own loadcell as a back-pressure sensor. No test prints, no squinting at corner artefacts
- **Why per-tool matters:** measured on a 5-tool XL with the *same spool, same nozzle size, same temperature*, only the tool changed — T0 `0.0367`, T1 `0.0552`, T2 `0.0647`. A **77% spread** between lowest and highest. A single shared PA value is wrong for most of the tools on the machine. Extruder gear wear, PTFE liner friction and heatbreak tolerance all land on this one parameter, and on a five-tool machine they differ per tool
- Upstream PrusaPATuner states multi-tool machines are not handled; per-tool measurement, storage and recall is what KlipperXL adds

### MCU-side loadcell streaming (`src/modbus_stm32f4.c`)

- The XLBuddy now streams Dwarf loadcell records to the host as they arrive, instead of answering one host query at a time
- **Required, not an optimisation.** Host-side polling works at idle but fails under extrusion: Klipper's MCU query timeout fires whenever a Dwarf misses a read, and the capture stalls
- Measured on the same hardware over a 12-minute run — host polling @20 ms: **110 Hz with 5 stalls**; MCU streaming: **300 Hz with zero gaps**
- A firmware built from before this release cannot do automatic PA tuning

### New modules and configs

- `klippy/dwarf_pa_tuner.py` — loadcell recorder, sweep capture, `ANALYZE_PA`. Runs the analyser as a background subprocess so it can never block Klipper's reactor
- `klippy/tool_pa.py` — per-tool K storage and auto-recall. Wraps `T0`–`T4` via `register_command(name, None)` and calls the previous handler first, so it needs **zero edits to `puppy_bootloader.py`**
- `config/pa_tuner.cfg` — `CALIBRATE_PA` sweep, ported from a real PrusaPATuner run on a stock XL and verified line by line against that G-code. `CALIBRATE_PA_FINE` for a 0.002 grid
- `config/pa_menu.cfg` — the five per-tool buttons
- `pa_analysis/` — the analyser, in its own virtualenv (numpy + scipy). scipy is deliberately kept out of `klippy-env`, which drives the printer
- New commands: `GET_TOOL_PA`, `SET_TOOL_PA`, `SAVE_TOOL_PA`, `APPLY_TOOL_PA`, `CALIBRATE_PA`, `CALIBRATE_PA_FINE`, `ANALYZE_PA`, `START_PA_RECORDING`, `STOP_PA_RECORDING`

### Third-party code: CNC Kitchen's analyser, vendored

- `pa_analysis/prusa_pa_tuner/` is **Stefan Hermann's (CNC Kitchen)** [PrusaPATuner](https://github.com/CNCKitchen/PrusaPATuner) analysis library, included **unmodified** under **AGPL-3.0-or-later**. The mathematics that turns a pressure trace into an optimal K is his work; KlipperXL does not claim it
- Vendored rather than downloaded at install time so KlipperXL stays buildable if the upstream repository is ever renamed, made private or removed. A firmware project that cannot be rebuilt from its own sources is not reproducible
- Runs as a detached subprocess, never imported into the Klipper process — which also keeps the AGPL and GPL components cleanly separated
- Full attribution, provenance and licence explanation in `pa_analysis/prusa_pa_tuner/NOTICE`

### Behaviour notes

- **Nothing is persisted automatically.** The measured K goes live in memory immediately, so the tool is already using it and can be print-tested before you commit; the console then reminds you to run `SAVE_CONFIG`. Pass `SAVECONFIG=1` for a fully hands-off run
- A never-tuned tool reports `--`, not `0.0000`, and is left alone on a tool change. `0.0` is a legitimate PA value, so printing it for an unmeasured tool would be misleading
- `G28` runs **before** the tool pick. `[homing_override]` couples T0 for the Z probe, so picking a tool first would silently switch back to T0 and record T0's loadcell while saving the result as another tool's K
- The sweep homes and parks **cold**, then heats. A failure mid-macro must not leave a hot nozzle behind
- Default K range is `0.02`–`0.12`. The earlier `0.02`–`0.07` did not contain the optimum for PLA on these tools, and when the optimum falls outside the swept window the estimators extrapolate and disagree
- Each run writes a timestamped CSV (`/tmp/pa_recording_T<n>_<timestamp>.csv`) so previous runs stay re-analysable
- Documentation: [`docs/PRESSURE_ADVANCE.md`](docs/PRESSURE_ADVANCE.md)

## Upgrading to 2026-08-16 from an earlier version

> ### ⚠ THIS RELEASE REQUIRES A FIRMWARE REBUILD AND RE-FLASH
>
> Automatic Pressure Advance depends on **MCU-side loadcell streaming**, which is a
> change to `src/modbus_stm32f4.c`. Copying the Python modules alone will **not**
> work — `ANALYZE_PA` will capture nothing, because the firmware on your XLBuddy
> cannot stream loadcell records. This is the first release where updating means
> more than dropping in Python files.
>
> Everything else in this release works without reflashing. Only PA needs it.

### 1. Rebuild and re-flash the MCU firmware

```bash
cp src/modbus_stm32f4.c ~/klipper/src/
cd ~/klipper && make clean && make -j4
```

Then package and flash by the normal USB route — see
[Installation Guide section 6.5](docs/INSTALLATION_GUIDE.md#65-future-firmware-updates).
Confirm your build offset is still **`128KiB with STM32 bootloader (0x08020200)`**.

### 2. Copy the new Python modules

```bash
cp klippy/dwarf_pa_tuner.py ~/klipper/klippy/extras/
cp klippy/tool_pa.py        ~/klipper/klippy/extras/
```

### 3. Copy the new config files

```bash
cp config/pa_tuner.cfg       ~/printer_data/config/
cp config/pa_menu.cfg        ~/printer_data/config/
```

### 4. Set up the analyser and its virtualenv

```bash
cp -r pa_analysis ~/pa_analysis
python3 -m venv ~/pa_analysis_env          # sudo apt install python3-venv if this fails
~/pa_analysis_env/bin/pip install -r ~/pa_analysis/requirements.txt
```

### 5. Edit your existing `printer.cfg` — do NOT overwrite it

Replacing it wholesale loses your calibration. Add these by hand:

```ini
[include pa_tuner.cfg]
[include pa_menu.cfg]
```

and, **above** the `#*# <---- SAVE_CONFIG ---->` marker:

```ini
[dwarf_pa_tuner]
[tool_pa]
```

### 6. Replace your `START_PRINT` prime line

The prime line in earlier versions extrudes a fixed `E15` / `E30` regardless of
line length, and those values were written for absolute E while the slicer runs
relative — so it can push **45 mm of filament into a 20 mm line** and skip steps.
Copy the whole `# Step 10: Prime line` block from the new `config/printer.cfg`
over your existing one.

### 7. `max_extrude_cross_section: 50.0` → `4.0`

**This reverses the instruction in the previous release**, which told you to raise
it to 50.0 "to accommodate slicer-emitted purge moves ... would otherwise error
mid-print on the first prime line". That diagnosis was backwards. The moves
tripping the guard were not the slicer's — they were `START_PRINT`'s own prime
line extruding up to 45 mm of filament into a 20 mm move (step 6). The guard was
correctly reporting a real bug, and raising the limit silenced the messenger.

**Measured, not assumed.** Six real sliced files were scanned — about 414,000
extruding moves across Benchy, multi-tool panels and a tolerance test:

| File | Peak cross-section |
|---|---|
| 3DBenchy | **0.501 mm²** |
| Mini Bob | 0.252 mm² |
| magnetic panel (28 / 24) | 0.163 / 0.156 mm² |
| tolerance test | 0.124 mm² |

Klipper's own default for a 0.4 nozzle is **0.64 mm²**. Real slicer output never
even reached that, let alone needed 50.0. **4.0 leaves roughly 8× headroom over
the worst move any of these files contains** — ample for wide first layers, prime
towers and purge moves, while still catching a fault like the prime-line bug
instead of grinding through it.

**Do step 6 before step 7**, or your first print will error out instead of just
extruding badly.

### 8. `max_accel: 1800` → `4000` (optional, recommended)

1800 silently capped every slicer request above it — a profile asking for 5000 got
1800 with no warning. Stock Prusa XL runs 2500–4000 for print moves against a 7000
hardware ceiling.

**Do step 9 with it, not after it.** Stock runs 4000 with input shaping active, and
raising acceleration on an unshaped machine is how you get ringing.

### 9. Turn input shaping on (do this with step 8)

Check your `printer.cfg` for a `#*# [input_shaper]` block below the SAVE_CONFIG
marker. If it has `shaper_freq_x` / `shaper_freq_y` values, you have already
calibrated and there is nothing to do — those win over anything in this step.

If it is empty, or your `[input_shaper]` section is all commented out, put stock's
factory values in above the marker:

```ini
[input_shaper]
shaper_type_x: mzv
shaper_freq_x: 35.8
shaper_type_y: mzv
shaper_freq_y: 35.4
```

Then measure your own with `CALIBRATE_INPUT_SHAPER` followed by `SAVE_CONFIG`.

### 10. Restart and tune

```bash
sudo systemctl restart klipper
```

Then, per tool: `PA_TUNE_T0` … `PA_TUNE_T4`, followed by `SAVE_CONFIG`.
Each tool stores its own value — see [docs/PRESSURE_ADVANCE.md](docs/PRESSURE_ADVANCE.md).

---

## 2026-05-30

### Loadcell mesh-probe reliability fixes
- Fixed intermittent first-mesh-point probe failure ("Probe triggered prior to movement" with sample_count=0)
- Three host-side fixes in `puppy_bootloader.py`, all addressing asymmetry between Z-home and bed-mesh probe paths:
  - `run_probe`: added `toolhead.wait_moves()` before tare so the loadcell samples a settled toolhead instead of mid-deceleration
  - `multi_probe_begin`: post-coil-on settle 0.2s → 1.0s so the loadcell FIFO refills with steady-state samples before tare
  - `LoadcellEndstop.home_wait`: disarm via `probe_stop_cmd` instead of re-arming with `oid=0xff` — preserves MCU diagnostic state (count/load/min/max) for failure capture instead of wiping it
- Root cause confirmed by 3-agent code read: trsync framework and MCU loadcell module are both clean; bug was host-side Z-home/mesh path asymmetry
- Multiple clean bed-mesh runs validated post-deploy

### Loadcell probe DIAG instrumentation
- Added `DIAG probe-start` (X/Y/Z/threshold/multi-mode) and `DIAG probe-FAIL` (MCU count/load/min/max/errors + endpos) logging in `run_probe`
- Added `DIAG home_wait NO-HIT` log for non-trigger trsync results
- Provides actionable data in `klippy.log` when probe failures recur

### Anti-ooze: END_PRINT auto-retract + cancel-path retract
- `END_PRINT` macro now does a hot-guarded `G1 E-20 F1800` deep retract to empty the melt zone while the nozzle is still at print temp
- Hot path emits `RESPOND MSG="END_PRINT: deep retract E-20 (nozzle XXXC)"` so users see the action fire; skip path (nozzle below `min_extrude_temp`) emits a corresponding skip message
- New `_MELT_RETRACT_ON_CANCEL` macro mirrors the same retract on print cancel
- `_CLIENT_VARIABLE` macro registers `_MELT_RETRACT_ON_CANCEL` as Fluidd's `user_cancel_macro` hook so cancel-from-pause fires the retract
- Reduces dock-ooze residue and lowers refill-on-next-print burden

### Babystep capture via SAVE_TOOL_Z_OFFSET wrapper
- New `[gcode_macro SET_GCODE_OFFSET]` wraps Klipper's stock `SET_GCODE_OFFSET`
- On a babystep `Z_ADJUST`, calls `SAVE_TOOL_Z_OFFSET` to stage the active tool's `tN_z_offset` immediately
- Stage survives print complete / cancel / soft-error; lost only on hard klippy shutdown (then `SAVE_CONFIG` before risky prints)
- Toolchange and homing `Z_ADJUST` calls pass through untouched

### max_extrude_cross_section bumped
- `5.0 → 50.0` (in `[extruder]`) to accommodate slicer-emitted purge moves that exceed the default 4×nozzle² limit (would otherwise error mid-print on the first prime line)
- > ⚠️ **Later found to be a misdiagnosis and reverted to `4.0` on 2026-08-16.** The offending moves were `START_PRINT`'s own prime line, not the slicer's. Measured slicer output peaks at 0.501 mm², below Klipper's own 0.64 default

---

## Upgrading from an earlier version

The install **procedure** itself has not changed (USB BBF flash, MCU build, Python module deploy are all the same). However existing users updating to this version need to take these steps:

### 1. Update Python modules
Re-run the install script's Python deploy step, OR manually copy:
```bash
cp klippy/puppy_bootloader.py /home/pi/klipper/klippy/extras/puppy_bootloader.py
sudo systemctl restart klipper
```

### 2. Copy the new macros into your existing `printer.cfg`
(Skip this step if you're replacing your `printer.cfg` wholesale with the new template — but back up your `SAVE_CONFIG` block first, it contains your calibration!)

From the new `config/printer.cfg`, copy these blocks into your local printer.cfg:
- **`[gcode_macro SET_GCODE_OFFSET]`** — babystep capture (wraps Klipper's stock `SET_GCODE_OFFSET` and stages `tN_z_offset` on `Z_ADJUST`)
- **`[gcode_macro _CLIENT_VARIABLE]`** — Fluidd cancel-macro hook (registers the cancel-retract)
- **`[gcode_macro _MELT_RETRACT_ON_CANCEL]`** — cancel-path deep retract
- **`[gcode_macro END_PRINT]`** — REPLACE your existing END_PRINT block with the new version (adds hot-guarded deep retract + RESPOND messages)

### 3. Bump `max_extrude_cross_section`
> ⚠️ **SUPERSEDED by the 2026-08-16 release — do not follow this step.** Set
> `4.0`, not `50.0`. The reasoning below turned out to be wrong: the moves
> tripping the guard were `START_PRINT`'s own over-extruding prime line, not the
> slicer's. See [step 7 of the 2026-08-16 upgrade notes](#upgrading-to-2026-08-16-from-an-earlier-version).
> Kept here as a record of what this release said at the time.

In your `[extruder]` block, change `max_extrude_cross_section: 5.0` → `max_extrude_cross_section: 50.0`.
Required if your slicer emits purge moves wider than 4× nozzle area (most modern slicers do for prime lines). No reason not to update.

### 4. Restart klipper
```bash
sudo systemctl restart klipper
```
Or `RESTART` from the gcode console. Verify klippy comes back to "Ready" state with no config errors.

---

## Troubleshooting recipe: occasional hard X/Y home tap → missed tool pick

If you see your X or Y axis occasionally do a "hard tap" at homing (sometimes skipping a step), and that misalignment causes a downstream tool-pick failure ("Tool not detected as picked after wiggle"), the following values were validated on the developer's XL as a starting point you can try:

- `[tmc2130 stepper_x]` and `[tmc2130 stepper_y]`:
  - `homing_speed: 55` (default is 52)
  - `second_homing_speed: 55` (default is 52)
  - `driver_SGT: 0` (default is 1)
- `[homing_override]` macro: change `{% set HOME_CUR = 0.750 %}` → `{% set HOME_CUR = 0.70 %}`

**Rationale:** Higher `homing_speed` puts TMC2130 StallGuard back inside its velocity window where the trigger is more decisive. Lower `HOME_CUR` softens the impact at trip.

**These values are per-printer.** StallGuard sensitivity depends on mechanical variance between XLs (belt tension, frame stiffness, motor batch tolerances, even dock magnet strength). The above is a known-good starting point — your printer's sweet spot may differ. If `0.70` is too low and you start missing home detection entirely, step up by 0.02 at a time. If `55` mm/s feels too fast and causes overshoot, try 54. Do not adopt these values unless you're already experiencing the hard-tap problem.

---

# Changelog

All notable changes to KlipperXL are documented here.

---

## 2026-04-20

### LED Stroboscope for belt tension / VFA visualization
- New `[strobe_test]` module + MCU-side `neopixel_spi.c` — strobes the side LED strip white channel at configurable frequency using TIM13 autonomous timer (no host involvement during strobe)
- New commands:
  - `STROBE_START FREQ=75` — default 75 Hz, 14% duty
  - `STROBE_START FREQ=90 DUTY=35` — custom frequency (0-255 Hz) and duty (0-255)
  - `STROBE_STOP` — stop strobe
- Prusa reference range: 60-130 Hz, 13.7% duty (35/255) — matches stock XL belt tension tool
- Implementation detail: WS2812 data pin (PG14) is shared with SPI6 display. MCU atomically switches pin AF mode → GPIO output and back via PE9 mux, avoiding SPI race conditions
- Config:
  ```
  [strobe_test]
  data_pin: PG14
  mux_pin: PE9
  ```
- New files: `klippy/extras/strobe_test.py`, `src/neopixel_spi.c`; `src/Makefile` adds `neopixel_spi.c` alongside `neopixel.c`
- Requires MCU firmware rebuild + flash (bundled in this release)
- Added helper macros `STROBE_BELT` / `STROBE_OFF` in `print_macros.cfg` — button-friendly wrappers that stop running LED effects on strobe start and restore `effect_idle` on strobe stop
- MCU-side fix: `command_neopixel_spi_strobe` with freq=0 now restores PG14 to GPIO output mode (MODER=01) and sets the PE9 mux HIGH after the final OFF frame, so the standard Klipper neopixel driver can drive the LED strip immediately after strobe stops (previously required a FIRMWARE_RESTART to recover the LEDs)
- Install script (`INSTALL_ON_PI.sh`) now deploys both files and patches `src/Makefile` to build `neopixel_spi.c` automatically

---

## 2026-04-17

### Install guide start G-code fix
- Added `TOOL=[initial_extruder]` to the OrcaSlicer Machine Start G-code example (Section 9.2)
- Without this parameter, `START_PRINT` always defaulted to `TOOL=0` regardless of which tool the slicer assigned — caused prints to use T0 even when another tool was selected
- The `KlipperXL.json` and `KlipperXL_profile.ini` profiles already included `TOOL=[initial_extruder]` — only the manually-pasted install guide line was missing it

### Time Lapse G-code section added
- New "Time Lapse G-code" subsection under 9.3 with `TIMELAPSE_TAKE_FRAME` — pairs with moonraker-timelapse installs

### Between Objects G-code heading clarified
- Heading now notes that OrcaSlicer labels this field "Printing by object G-code" and it is only emitted when Print Sequence = By Object

---

## 2026-04-15

### PCA9557 commands renamed to IOEXP (fixes [#11](https://github.com/racoutlaw/KlipperXL/issues/11))
- Renamed `PCA9557_RELEASE_DWARFS`, `PCA9557_RESET_DWARFS`, `PCA9557_STATUS`, and `PCA9557_SET_PIN` to `IOEXP_*` counterparts
- Klipper's gcode parser strips digits from command names, so `PCA9557_STATUS` was being dispatched as unknown command `PCA9557` and was never callable from the console
- Added `ENCLOSURE_FAN_POWER_ON` / `ENCLOSURE_FAN_POWER_OFF` user-facing commands that toggle PCA9557 pin 6 (12V rail to the enclosure fan header) without requiring users to know pin numbers
- Replaced cached-output-state `_set_pin()` with read-modify-write on the hardware register — the PCA9557 is shared with modbus_stm32f4.c firmware and puppy_bootloader, so cached state could go stale and a naive write could clobber Dwarf reset pins

### Dead modbus I2C scaffolding removed (fixes [#16](https://github.com/racoutlaw/KlipperXL/issues/16))
- Removed 144 lines of unused bit-banged I2C code from `src/modbus_stm32f4.c` — had been triggering `-Wunused-function` warnings since the initial KlipperXL commit
- Functions removed: `i2c_delay`, `i2c_sda_high/low/read`, `i2c_scl_high/low`, `i2c_init`, `i2c_start`, `i2c_stop`, `i2c_write_byte`, `pca9557_write`, `pca9557_release_all`, `pca9557_reset_all`, `pca9557_raw_write`
- Replaced during initial development by hardware I2C (`hw_i2c_init`, `hw_i2c_write`) — old functions were never called
- Verified zero warnings after cleanup; resulting `klipper.bin` is functionally equivalent (removed code was never executed)

---

## 2026-04-14

### Force-flash button sequence documented
- Section 6.2 now explains how to trigger the Prusa bootloader's firmware-update menu: press the selector knob on power-up, or reset + double-press the knob to force the update menu
- Credit @Ro3Deee ([#13](https://github.com/racoutlaw/KlipperXL/issues/13)) for pointing out that some users couldn't get the bootloader to see `klipper.bbf` without the button sequence

### USB BBF is now the default flash method
- Restructured `INSTALLATION_GUIDE.md` to make USB drive flashing the primary path (Section 6)
- DFU flashing moved to Section 12 as an advanced / recovery-only method
- Rationale: USB BBF requires no opening the printer, no BOOT0 jumper, preserves the Prusa bootloader, and future updates are drag-and-drop to a USB stick
- Section 3.4 now instructs users to set bootloader offset to **128KiB with STM32 bootloader (0x08020200)** by default
- New Section 3.6: packaging the firmware as `.bbf` with `pack_fw.py`
- Section 1 (Prerequisites) now lists the FAT32 USB drive and broken-appendix requirement up front
- Troubleshooting updated with USB BBF-first error guidance
- Thanks to @Ro3Deee for raising the issue

---

## 2026-04-05

### Build & Install Fixes
- Fixed missing `build-essential` dependency that caused "cpp: No such file or directory" error during firmware build
- Install script now checks for `cpp` before building and tells you exactly what to install
- Fixed install script path bug that couldn't find source files when run from the scripts directory
- Guide now uses `git clone` instead of referencing a tarball package that was never published
- Guide uses `make menuconfig` instead of a pre-built .config file that was incompatible with current Klipper versions (caused "Multiple definitions for command 'identify'" error)
- All commands now run directly on the Pi — no separate build computer needed
- Added `tool_offsets.py` to the Python module deployment (was missing)
- Fixed section order: Python modules and config files deploy before MCU flash so you can prep the Pi without the printer connected
- Added note that KIAUH already clones Klipper (no duplicate clone step)
- Added section for fixing config includes (Mainsail vs Fluidd, LED effects)

### Side LED Strip Fix
- Corrected LED configuration from `chain_count: 1` / `color_order: RGBW` to `chain_count: 2` / `color_order: GRB`
- Prusa XL uses two daisy-chained WS2812 drivers per strip: Driver 1 handles RGB color, Driver 2 handles white on its green channel
- Confirmed from Prusa firmware source code

### USB Drive Firmware Flashing (New)
- Added advanced guide for flashing KlipperXL via USB drive instead of DFU
- Preserves the Prusa bootloader — no BOOT0 jumper needed for future updates
- Klipper compiled at 0x08020200 offset, packaged as .bbf using Prusa's pack_fw.py tool
- Bootloader progress bar stops at ~50% when Klipper takes over (normal — Klipper doesn't drive the display, just restart the Klipper service)
- Requires broken appendix (safety seal) to bypass signature verification
- Included pack_fw.py in the repo for packaging

### Recovery & Stock Restore
- Complete recovery instructions for both flash methods
- With bootloader preserved: just put stock Prusa .bbf on USB drive and power cycle
- Without bootloader: DFU flash bootloader first, then stock firmware via USB
- Included bootloader-xl-2.5.0.bin and verified stock XL_firmware_6.4.0.bbf in the recovery directory
- Emergency DFU noboot recovery option as last resort

### Dirty State Fix
- Added `tool_offsets.py` to the git exclude list so Klipper update manager stays clean

All changes tested with a complete fresh install from SD card flash through DWARF_STATUS confirmation.
