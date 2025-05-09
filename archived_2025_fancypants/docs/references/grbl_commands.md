# GRBL Command Reference

This document provides a reference for common GRBL commands used with the OpenBuilds BlackBox controller in this project.

## Basic G-Codes

| Command | Description | Example |
|---------|-------------|---------|
| `G0`    | Rapid positioning (fastest possible speed) | `G0 X10 Y10` |
| `G1`    | Linear movement at specified feed rate | `G1 X10 Y10 F500` |
| `G20`   | Set units to inches | `G20` |
| `G21`   | Set units to millimeters | `G21` |
| `G28`   | Return to home position | `G28` |
| `G90`   | Absolute positioning mode | `G90` |
| `G91`   | Relative positioning mode | `G91` |
| `G92`   | Set position (without moving) | `G92 X0 Y0 Z0` |

## GRBL-Specific Commands

| Command | Description | Example |
|---------|-------------|---------|
| `$`     | Show GRBL settings | `$` |
| `$$`    | Show GRBL settings (detailed) | `$$` |
| `$H`    | Run homing cycle | `$H` |
| `$X`    | Kill alarm lock (unlock GRBL) | `$X` |
| `$G`    | Show parser state | `$G` |
| `?`     | Request status report | `?` |
| `!`     | Feed hold (pause) | `!` |
| `~`     | Cycle start / Resume | `~` |
| `Ctrl+X`| Soft reset (in terminal: `\x18`) | N/A |

## Common GRBL Settings

| Setting | Description | Example |
|---------|-------------|---------|
| `$0`    | Step pulse time (microseconds) | `$0=10` |
| `$1`    | Step idle delay (milliseconds) | `$1=25` |
| `$10`   | Status report options | `$10=1` |
| `$13`   | Report in inches (boolean) | `$13=0` |
| `$20`   | Soft limits enable (boolean) | `$20=1` |
| `$21`   | Hard limits enable (boolean) | `$21=1` |
| `$22`   | Homing cycle enable (boolean) | `$22=1` |
| `$23`   | Homing direction invert (mask) | `$23=0` |
| `$24`   | Homing feed rate (units/min) | `$24=25.0` |
| `$25`   | Homing seek rate (units/min) | `$25=500.0` |
| `$27`   | Homing switch pull-off distance | `$27=1.0` |
| `$110`  | X-axis maximum rate (units/min) | `$110=8000.0` |
| `$111`  | Y-axis maximum rate (units/min) | `$111=8000.0` |
| `$112`  | Z-axis maximum rate (units/min) | `$112=500.0` |
| `$120`  | X-axis acceleration (units/sec^2) | `$120=500.0` |
| `$121`  | Y-axis acceleration (units/sec^2) | `$121=500.0` |
| `$122`  | Z-axis acceleration (units/sec^2) | `$122=50.0` |

## Movement-Related G-Codes

| Command | Description | Example |
|---------|-------------|---------|
| `F`     | Set feed rate | `F1000` or within: `G1 X10 F1000` |
| `S`     | Set spindle speed | `S1000` |
| `M3`    | Start spindle clockwise | `M3 S1000` |
| `M4`    | Start spindle counter-clockwise | `M4 S1000` |
| `M5`    | Stop spindle | `M5` |
| `M8`    | Coolant on | `M8` |
| `M9`    | Coolant off | `M9` |

## Error Codes

Common GRBL error codes you might encounter:

| Code | Description |
|------|-------------|
| 1    | G-code words consist of a letter and a value. Letter was not found. |
| 2    | Numeric value format is not valid or missing an expected value. |
| 3    | GRBL '$' system command was not recognized or supported. |
| 5    | Homing cycle is not enabled via settings. |
| 9    | GRBL system is in alarm state. |
| 10   | Soft limit error - G-code command exceeds workspace. |
| 11   | Hard limit error - Physical limit switch triggered. |

## OpenBuilds BlackBox X4 Resources

For more detailed information about the OpenBuilds BlackBox X4 controller, please refer to the [official documentation](https://docs.openbuilds.com/doku.php?id=docs:blackbox-4x:start).

## Usage with ROS2 Interface

When using these commands with our ROS2 interface:

```bash
# Send G-code command
ros2 service call /cnc/send_gcode std_srvs/srv/SetBool "{data: 'G0 X10 Y10'}"

# Run homing cycle
ros2 service call /cnc/home std_srvs/srv/Trigger "{}"

# Reset the controller
ros2 service call /cnc/reset std_srvs/srv/Trigger "{}"
```
