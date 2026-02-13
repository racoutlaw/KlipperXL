# KlipperXL OrcaSlicer Setup

## Quick Setup (Recommended)

1. Open OrcaSlicer
2. Add the **Generic Tool Changer** printer profile (this gives you all layer heights, nozzle sizes, and print presets)
3. Go to **File > Import > Import Configs**
4. Select `KlipperXL.json` from this folder
5. Select **KlipperXL** from the printer dropdown

Done. All custom G-codes, speeds, bed size, and machine limits are pre-configured.

## What the KlipperXL Profile Sets

The import overrides these Generic Tool Changer settings to match the Prusa XL running Klipper:

| Setting | Value |
|---------|-------|
| Bed size | 360x360mm |
| Print height | 360mm |
| G-code flavor | Klipper |
| Nozzle type | Hardened steel |
| Z offset | 0 (tuned per-tool in Klipper, see main README) |
| Max speed X/Y | 400 mm/s |
| Max speed Z | 12 mm/s |
| Max accel X/Y | 1800 mm/s² |
| Retraction speed | 60 mm/s |
| Retraction length | 0.8mm |
| Z hop | 0.4mm |

## Custom G-codes (Pre-configured in Profile)

### Start G-code
```
START_PRINT EXTRUDER_TEMP=[nozzle_temperature_initial_layer] BED_TEMP=[bed_temperature_initial_layer_single] TOOL=[initial_extruder] TOTAL_LAYER_COUNT={total_layer_count} X0={first_layer_print_min[0]} Y0={first_layer_print_min[1]} X1={first_layer_print_max[0]} Y1={first_layer_print_max[1]}
```

The START_PRINT macro handles everything automatically:
- XY and Z homing
- Bed heating and waiting
- Tool pickup and heating to probe temp (150C)
- Z homing with loadcell
- Adaptive bed mesh (probes only the print area)
- Switching to the correct print tool
- Heating to print temperature

### End G-code
```
END_PRINT
```

### Pause G-code
```
PAUSE
```

### Before Layer Change G-code
```
;BEFORE_LAYER_CHANGE
;[layer_z]
G92 E0
_ON_LAYER_CHANGE LAYER={layer_num + 1}
```

### After Layer Change G-code
```
;AFTER_LAYER_CHANGE
;[layer_z]
```

### Between Objects G-code
```
;BETWEEN_OBJECTS
G92 E0
```

## Troubleshooting

### "Extrude below minimum temp" on print start
The start G-code still has the Generic Tool Changer purge sequences. Make sure it only contains the single `START_PRINT` line shown above.

### Bed mesh probes entire bed instead of print area
The start G-code is missing the `X0/Y0/X1/Y1` parameters. Copy the full START_PRINT line from above.

### Print profiles (layer heights) not showing
Make sure you added the Generic Tool Changer printer BEFORE importing KlipperXL.json. The profile inherits from Generic Tool Changer to get all print presets.

### Tower falls over
- Increase brim width to 5-6mm
- Decrease cone angle to 20 degrees

### Ooze/stringing between tools
- Set standby temperature delta to -110
- Enable ooze prevention
