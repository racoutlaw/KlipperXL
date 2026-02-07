# XlKlipper OrcaSlicer Setup

Use the built-in **Generic Tool Changer** profile in OrcaSlicer as your base.
It already has multi-tool settings (wipe tower, purge volumes, tool change
handling). You only need to make the changes listed below.

## Step 1: Select Generic Tool Changer Profile

In OrcaSlicer:
1. Go to **Printer** tab
2. Click **Add Printer** or select from dropdown
3. Choose **Generic Tool Changer** (5-tool variant)

## Step 2: Change These 4 Settings

### 1. G-code Flavor
Set to: **Klipper**

### 2. Start G-code (REPLACE ENTIRE BLOCK)
Delete everything in the start G-code box and replace with this single line:
```
START_PRINT EXTRUDER_TEMP=[nozzle_temperature_initial_layer] BED_TEMP=[bed_temperature_initial_layer_single] TOTAL_LAYER_COUNT={total_layer_count} X0={first_layer_print_min[0]} Y0={first_layer_print_min[1]} X1={first_layer_print_max[0]} Y1={first_layer_print_max[1]}
```

**IMPORTANT:** Replace the ENTIRE start gcode. The generic profile has per-tool
purge sequences that conflict with our START_PRINT macro. Our macro handles
homing, heating, probing, and adaptive bed mesh automatically.

**DO NOT** add `BED_MESH_PROFILE LOAD=default` - START_PRINT does adaptive mesh.

### 3. End G-code (REPLACE ENTIRE BLOCK)
```
END_PRINT
```

### 4. Acceleration Values (Prusa XL Matched)
The generic profile has high acceleration values (up to 10000) that are not
safe for Prusa XL hardware. Change to these Prusa-matched values:

| Setting | Value |
|---------|-------|
| Default acceleration | **1250** |
| Perimeter acceleration | **1000** |
| Infill acceleration | **2000** |
| Solid infill acceleration | **1500** |
| Top solid infill acceleration | **800** |
| Bridge acceleration | **1000** |
| Travel acceleration | **5000** |
| First layer acceleration | **800** |
| Machine max accel X/Y | **5000** |

Prusa XL hardware limit is 7000. Never exceed this.

## Step 3: Additional Settings

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

### Print Settings (Process Tab)
| Setting | Location | Value |
|---------|----------|-------|
| Label objects | Others | **Firmware** (enables cancel single object) |
| Arc fitting | Others | **Emit center** (enables G2/G3 arcs) |

### Wipe Tower Settings (verify or set)
| Setting | Value |
|---------|-------|
| Enable prime tower | **ON** |
| Prime tower width | **60** |
| Prime tower brim width | **3** |
| Wall type | **Cone** |
| Stabilization cone apex angle | **25** |
| Enable ooze prevention | **ON** |
| Standby temperature delta | **-110** |

## Step 4: Save as New Profile

Save as a new printer profile (e.g., "XlKlipper 5-Tool") so your changes
don't get overwritten by OrcaSlicer updates.

## That's It

Everything else (retraction, speeds, filament settings) from the generic
tool changer profile works fine. The START_PRINT macro handles:
- XY and Z homing
- Bed heating and waiting
- Tool pickup and heating to probe temp (150C)
- Z homing with loadcell
- Adaptive bed mesh (probes only the print area)
- Switching to the correct print tool
- Heating to print temperature

## Troubleshooting

### "Extrude below minimum temp" on print start
You didn't replace the entire start gcode. The generic profile's purge
sequences try to extrude before tools are heated. Replace the whole
start gcode block with our single START_PRINT line.

### Acceleration showing 10000+ in Mainsail
You didn't update the acceleration values. See Step 2 item 5 above.

### Bed mesh probes entire bed instead of print area
You forgot the X0/Y0/X1/Y1 parameters in the start gcode. Make sure
the full START_PRINT line includes all parameters.

### Tower falls over
- Increase brim width to 5-6mm
- Decrease cone angle to 20 degrees

### Ooze/stringing between tools
- Verify standby_temperature_delta = -110
- Check that ooze prevention is ON

## Alternative: Full Profile Import

If you prefer importing a pre-configured profile bundle:
1. Open OrcaSlicer
2. **File > Import > Import Config Bundle**
3. Select `XlKlipper_Profile.ini` from this folder
4. Profiles appear in your dropdowns
