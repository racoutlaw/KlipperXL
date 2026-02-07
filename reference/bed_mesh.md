# Bed Mesh Configuration - Prusa-Matched Reference

## Our Klipper Config
```ini
[bed_mesh]
probe_count: 12, 12    # Prusa uses 12x12 major points (was 5,5 - too sparse!)
algorithm: bicubic
fade_start: 1
fade_end: 10
```

**CRITICAL:** 5x5 is too sparse for 360mm bed. Must use 12x12 (144 points) for proper first layer.

## Prusa XL Firmware Bed Mesh Settings

**Source Files:**
- `Prusa-Firmware-Buddy/include/marlin/Configuration_XL.h` (Lines 1228-1235)
- `Prusa-Firmware-Buddy/lib/Marlin/Marlin/src/feature/bedlevel/ubl/ubl_G29.cpp`
- `Prusa-Firmware-Buddy/lib/Marlin/Marlin/src/feature/bedlevel/ubl/ubl.h`
- `Prusa-Firmware-Buddy/lib/Marlin/Marlin/src/feature/print_area.h`

### Grid Size Settings (Configuration_XL.h)
```c
#define GRID_BORDER 1              // Border never probed (only size 1 supported)
#define GRID_MAJOR_STEP 3          // Offset between major probe points
#define GRID_MAJOR_POINTS_X 12     // Number of major probes on X axis
#define GRID_MAJOR_POINTS_Y 12     // Number of major probes on Y axis
#define GRID_MAX_POINTS_X 36       // Total grid resolution X
#define GRID_MAX_POINTS_Y 36       // Total grid resolution Y

// Formula: GRID_MAX = GRID_BORDER*2 + GRID_MAJOR + (GRID_MAJOR-1)*(GRID_MAJOR_STEP-1)
//        = 1*2 + 12 + 11*2 = 2 + 12 + 22 = 36
```

### Key Numbers
| Parameter | Value | Notes |
|-----------|-------|-------|
| Grid Storage | **36 x 36 = 1,296 points** | Full mesh resolution |
| Actually Probed | **12 x 12 = 144 points** | Major points only |
| Grid Spacing | **~10.29mm** | 360mm / 35 divisions |
| Probe Spacing | **~30.86mm** | 10.29mm x GRID_MAJOR_STEP(3) |
| Bed Size | 360 x 360mm | Full XL bed |

### Probing Speed/Timing (Configuration_XL.h Lines 920-975)
```c
#define XY_PROBE_SPEED_INITIAL 8000    // 133 mm/s for first approach
#define XY_PROBE_SPEED 18000           // 300 mm/s between probes
#define Z_PROBE_SPEED_FAST 600         // 10 mm/s fast approach
#define Z_PROBE_SPEED_SLOW 70          // ~1.17 mm/s slow/accurate
#define Z_FIRST_PROBE_DELAY 300        // 300ms before tare (vibration settle)
#define Z_CLEARANCE_BETWEEN_PROBES 0.21 // 0.21mm between points
#define Z_CLEARANCE_MULTI_PROBE 0.2    // 0.2mm between retries
#define MULTIPLE_PROBING 40            // Max 40 retries per point
```

### Mesh Coordinate Calculation (ubl.h)
```cpp
#define MESH_X_DIST (float(MESH_MAX_X - MESH_MIN_X) / float(GRID_MAX_POINTS_X - 1))
#define MESH_Y_DIST (float(MESH_MAX_Y - MESH_MIN_Y) / float(GRID_MAX_POINTS_Y - 1))

static inline float mesh_index_to_xpos(const uint8_t i) {
  return MESH_MIN_X + i * (MESH_X_DIST);  // 0 + i * 10.29mm
}
```

## How Prusa Adaptive Mesh Works (ubl_G29.cpp Lines 697-828)

**Key Algorithm - probe_major_points():**
```cpp
auto for_each_grid_point = [&](auto&& func) {
  // Loop Y from top to bottom, stepping by GRID_MAJOR_STEP (every 3rd point)
  for (int y = GRID_MAX_POINTS_Y - GRID_BORDER - 1; y >= GRID_BORDER; y -= GRID_MAJOR_STEP) {
    int y_idx = (y - GRID_BORDER) / GRID_MAJOR_STEP;
    bool is_odd_y_position = y_idx % 2 == 1;

    // Snake pattern - alternate X direction each row
    int x0 = is_odd_y_position ? GRID_BORDER : GRID_MAX_POINTS_X - 1 - GRID_BORDER;
    int xStep = is_odd_y_position ? GRID_MAJOR_STEP : -GRID_MAJOR_STEP;

    for (int x = x0; GRID_BORDER <= x && x < GRID_MAX_POINTS_X - GRID_BORDER; x += xStep) {
      xy_pos_t pos = {mesh_index_to_xpos(x), mesh_index_to_ypos(y)};
      if (!position_is_reachable_by_probe(pos.x, pos.y)) continue;
      if (!probe_area.contains(pos)) continue;  // ADAPTIVE: Skip outside print area
      func(pos, x, y);
    }
  }
};
```

**Prusa "Adaptive" vs Klipper "Adaptive":**
- **Prusa:** Fixed ~31mm grid positions, SKIPS points outside print area
- **Klipper ADAPTIVE=1:** SCALES DOWN probe count (breaks our spacing!)

### Print Area Bounding (print_area.h, ubl_G29.cpp)
```cpp
// Expand print area by one major step (~31mm) for probing margin
auto probe_area = print_area.get_bounding_rect().inset(
  -MESH_X_DIST * GRID_MAJOR_STEP,  // Expand by ~31mm in X
  -MESH_Y_DIST * GRID_MAJOR_STEP   // Expand by ~31mm in Y
);
```

## Bicubic Interpolation - Filling 36x36 Grid (ubl_G29.cpp Lines 1204-1235)

After probing 144 major points, Prusa fills remaining 1,152 via bicubic interpolation.

## Segment Leveling (Configuration_XL.h)
```c
#define SEGMENT_LEVELED_MOVES
#define LEVELED_SEGMENT_LENGTH 5.0  // Each segment = 5mm
```

## Comparison Table

| Parameter | Prusa XL | Our Klipper | Notes |
|-----------|----------|-------------|-------|
| Grid Storage | 36x36 (1296) | 12x12 (144) | Prusa interpolates more |
| Actually Probed | 12x12 (144) | 12x12 (144) | Same probe count |
| Probe Spacing | ~31mm fixed | ~30mm target | Similar |
| Adaptive Method | Skip outside area | Scale count | **DIFFERENT!** |
| Interpolation | Bicubic to 36x36 | Bicubic in Klipper | Both use bicubic |
| Segment Leveling | 5mm segments | Per G-code line | Prusa is finer |

## START_PRINT Adaptive Mesh Macro
```jinja2
# Prusa-style probe spacing: ~30mm between probes, min 4 for bicubic
{% set PROBE_SPACING = 30 %}
{% set area_x = X1 - X0 %}
{% set area_y = Y1 - Y0 %}
{% set probe_x = ((area_x / PROBE_SPACING) | round(0, 'ceil') | int) + 1 %}
{% set probe_y = ((area_y / PROBE_SPACING) | round(0, 'ceil') | int) + 1 %}
{% set probe_x = [probe_x, 4] | max %}
{% set probe_y = [probe_y, 4] | max %}
{% set probe_x = [probe_x, 12] | min %}
{% set probe_y = [probe_y, 12] | min %}

# FIXED 2026-02-03: Removed ADAPTIVE=1 so Klipper uses our exact probe count
BED_MESH_CALIBRATE MESH_MIN={X0},{Y0} MESH_MAX={X1},{Y1} PROBE_COUNT={probe_x},{probe_y}
```
