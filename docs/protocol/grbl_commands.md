# GRBL Protocol Documentation

## Overview
The CNC controller uses standard GRBL commands over a serial connection. Key commands from the Node-RED implementation:

## Command Format
All movement commands use `$J=` prefix followed by G-code modifiers.

### Examples from Node-RED Implementation
```
$J=G90G21X535Y315Z-230F100 # Move to target origo absolute
$J=G91G21Z-14F100 # Relative Z move down
$J=G90G21X{x}Y{y}Z{z}F{feed} // General absolute move
```

## Movement Commands
- Absolute positioning: `$J=G90G21X{x}Y{y}Z{z}F{feed}`
- Relative positioning: `$J=G91G21X{x}Y{y}Z{z}F{feed}`

## Configuration Commands
- Home: `$H`
- Reset: Ctrl-X (0x18)
- Status query: `?`

## Key Parameters
- G90: Absolute positioning
- G91: Relative positioning  
- G21: Use millimeters
- F: Feed rate in mm/min

## Status Response Format
Example: `<Idle|WPos:100.000,0.000,0.000|FS:0,0>`
Fields:
- Machine state (Idle/Run/Hold/Alarm)
- Work position (WPos:X,Y,Z)
- Feed & speed (FS:current,rpm)

## Error Codes and Recovery
- 1. Error:15 (Travel exceeded):
  - $X // Unlock alarm $H // Re-home machine
- 2. Position lost:
  - Ctrl-X // Soft reset (0x18) $H // Re-home

## Implementation Notes
1. Always initialize with absolute positioning (G90) and metric units (G21)
1. Status polling recommended at 5Hz (200ms interval)
1. Feed rates typically 100-3500 mm/min
