# pa_analysis — the Pressure Advance analyser

This directory holds the off-printer half of KlipperXL's automatic **per-tool**
Pressure Advance tuning. It runs as a **separate process in its own virtual
environment**, never inside Klipper.

```
pa_analysis/
  pa_analyze.py         KlipperXL's adapter        GPL-3.0   (Richard Crook)
  requirements.txt      numpy + scipy
  prusa_pa_tuner/       the analysis mathematics   AGPL-3.0  (Stefan Hermann, CNC Kitchen)
    LICENSE             full AGPL-3.0 text
    NOTICE              attribution, provenance, licence explanation  <-- read this
```

## Two licences, on purpose

`prusa_pa_tuner/` is **not** KlipperXL code. It is Stefan Hermann's
[PrusaPATuner](https://github.com/CNCKitchen/PrusaPATuner), vendored unmodified
under AGPL-3.0-or-later. It stays AGPL no matter where it is copied to.
`prusa_pa_tuner/NOTICE` explains the provenance and how the two licences fit
together.

It is **vendored rather than downloaded at install time** so KlipperXL stays
buildable if the upstream repository ever disappears.

## Why a separate venv, and not klippy-env

Two independent reasons, both deliberate:

1. **scipy must never be able to disturb the environment driving the printer.**
   The analyser wants numpy and scipy; `klippy-env` deliberately does not have
   scipy, and adding it there puts a heavy scientific stack inside the process
   that runs your motion queue.
2. **Licence separation.** The AGPL code is executed as a detached subprocess
   and is never imported into the Klipper process.

## Install

```bash
python3 -m venv ~/pa_analysis_env
~/pa_analysis_env/bin/pip install -r ~/KlipperXL/pa_analysis/requirements.txt
cp -r ~/KlipperXL/pa_analysis ~/pa_analysis
```

`[dwarf_pa_tuner]` finds both by default, relative to the Klipper user's home:

```ini
[dwarf_pa_tuner]
# analysis_python: ~/pa_analysis_env/bin/python
# analysis_script: ~/pa_analysis/pa_analyze.py
```

Override either if you put them elsewhere.

## Running it by hand

Every sweep leaves a timestamped CSV, so old runs stay analysable:

```bash
~/pa_analysis_env/bin/python ~/pa_analysis/pa_analyze.py /tmp/pa_recording_T1_20260816-121500.csv
```

It prints a summary and writes `/tmp/pa_result.json`.

The CSV header carries the sweep parameters as `# param NAME=VALUE`, so the
analysis plan is reconstructed from what the machine *actually ran* rather than
from assumptions — change the macro and the analyser follows automatically.
