# Probe Thresholds & Loadcell Reference

## Current Thresholds (Prusa-Matched)

| Threshold | Current Value | Prusa Reference |
|-----------|---------------|-----------------|
| Z Fast | **125g** | thresholdStatic = 125g |
| Z Slow | **125g** | thresholdStatic = 125g |
| Mesh | **125g** | Same as Z fast |
| XY Cal | **40g** | XY_PROBE_THRESHOLD = 40g |

**Commands:** `PROBE_THRESHOLDS`, `SET_Z_FAST_THRESHOLD`, `SET_Z_SLOW_THRESHOLD`, `SET_MESH_THRESHOLD`, `SET_XY_THRESHOLD`

**Mainsail Buttons:** `SHOW_THRESHOLDS`, `SET_Z_FAST`, `SET_Z_SLOW`, `SET_MESH`, `SET_XY` (require VALUE parameter)

## CRITICAL FIXES (2026-02-02)
1. **LOADCELL_PROBE command** was using hardcoded 60g/40g defaults instead of configurable thresholds
   - Fixed to use `self.loadcell_probe._z_fast_threshold` and `self.loadcell_probe._z_slow_threshold`
2. **LOADCELL_PROBE travel limit** was hardcoded to 0.0 instead of position_min (-10)
   - Fixed to use `z_min = self.loadcell_probe._get_z_min_position()`

## Tare Strategy (from Prusa `probe.cpp`)

Prusa's `loadcell_retare_for_analysis()`:
```cpp
safe_delay(Z_FIRST_PROBE_DELAY);  // 300ms - let vibrations settle
loadcell.WaitBarrier();            // Sync to fresh samples
float tare = loadcell.Tare();      // SINGLE tare (48 samples @ 320Hz = 150ms)
loadcell.analysis.Reset();         // Clear analysis window
```

**Context-dependent tare strategy (CRITICAL):**
- **Z Home (standalone probe):** Double tare for maximum accuracy
- **Bed Mesh (multi-probe):** Single tare per point - matches Prusa

**BUG FIXED:** Original code did double tare on EVERY mesh point:
- 3x3 mesh = 1 + (9 x 2) = 19 tare operations (MODBUS overload)
- "Timeout on wait for 'loadcell_tare_result' response" -> MCU shutdown
- Fixed: Now 1 + 9 = 10 tare operations

**Prusa deviation check (IMPLEMENTED):**
- If tare offset deviates >125g from reference baseline -> lift 20mm and re-tare
- `_reference_tare` stored at session start, compared at each mesh point
- Catches debris on nozzle that would cause systematic mesh errors

## Prusa Loadcell Constants

```cpp
// Thresholds (src/common/loadcell.hpp)
thresholdStatic = -125.f;      // Z probe trigger (grams)
thresholdContinuous = -40.f;   // Continuous filter mode
XY_PROBE_THRESHOLD = 40.f;     // XY calibration
hysteresis = 80.f;             // De-trigger: -125 + 80 = -45g

// Timing
Z_FIRST_PROBE_DELAY = 300;     // ms before tare (vibration settle)
STATIC_TARE_SAMPLE_CNT = 48;   // samples @ 320Hz = 150ms
TOUCHDOWN_DELAY_MS = 150;      // ms after probe triggers
MULTIPLE_PROBING = 40;         // max retries per mesh point

// Scale
scale = 0.0192f;               // Raw ADC -> grams conversion
```

## Prusa Loadcell Reference Files
| File | Purpose |
|------|---------|
| `lib/Marlin/Marlin/src/module/probe.cpp` | Probing logic, mesh tare strategy |
| `src/common/loadcell.cpp` | Core loadcell: Tare(), WaitBarrier(), thresholds |
| `src/common/loadcell.hpp` | Constants: thresholds, timing |
| `src/common/probe_analysis.hpp` | Analysis window: 310ms lookback, 430ms lookahead |
| `include/marlin/Configuration_XL.h` | Z_FIRST_PROBE_DELAY=300, Z_CLEARANCE=0.2 |
| `src/puppies/Dwarf.cpp` | Dwarf communication, loadcell enable/disable |
| `src/puppy/dwarf/loadcell.cpp` | Dwarf-side: 16-sample FIFO, spike filter, HX717 |
