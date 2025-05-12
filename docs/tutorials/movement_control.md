# CNC Movement Control

This tutorial explains how to control CNC movements using the CNC Controller Node, covering both absolute and relative positioning modes, feed rate control, and common movement patterns.

## Movement Modes

The CNC Controller supports two movement modes:

### Absolute Positioning (G90)

In absolute mode, coordinates are specified relative to the machine's origin (zero point):

- `X10` means "move to position X=10" regardless of current position
- This mode is useful for precise, repeatable movements to known locations

To set absolute mode:

```bash
ros2 service call /cnc/set_absolute_mode std_srvs/srv/Trigger "{}"
```

### Relative Positioning (G91)

In relative mode, coordinates are specified relative to the current position:

- `X10` means "move 10 units in the X direction" from current position
- This mode is useful for incremental movements or patterns

To set relative mode:

```bash
ros2 service call /cnc/set_relative_mode std_srvs/srv/Trigger "{}"
```

## Feed Rate Control

The feed rate determines the speed at which the CNC machine moves:

- Higher values result in faster movement
- Lower values provide more precise control

To set the feed rate (in mm/min):

```bash
ros2 service call /cnc/set_feed_rate std_srvs/srv/SetString "{data: '500'}"
```

## Basic Movement Commands

### Linear Movement ($J=)

All movement commands use the `$J=` format with G90/G91 and G21 (metric) modifiers:

```bash
# Absolute move with feed rate
ros2 service call /cnc/send_gcode std_srvs/srv/SetString "{data: '$J=G90G21X100Y50F300'}"

# Relative move with feed rate
ros2 service call /cnc/send_gcode std_srvs/srv/SetString "{data: '$J=G91G21X10Y5F300'}"
```

Examples:

```bash
# Rapid move (G0)
ros2 service call /cnc/send_gcode std_srvs/srv/SetString "{data: '$J=G90G21X100Y50'}"

# Controlled move with feed rate (G1)
ros2 service call /cnc/send_gcode std_srvs/srv/SetString "{data: '$J=G91G21X10Y5F300'}"
```

### Single-Axis Movement

To move along only one axis:

```bash
# Move only in X direction
ros2 service call /cnc/send_gcode std_srvs/srv/SetString "{data: '$J=G90G21X100'}"

# Move only in Y direction
ros2 service call /cnc/send_gcode std_srvs/srv/SetString "{data: '$J=G90G21Y50'}"
```

### Incremental Movements

Using relative mode for incremental movements:

```bash
# Set relative mode
ros2 service call /cnc/set_relative_mode std_srvs/srv/Trigger "{}"

# Move 10mm in X direction
ros2 service call /cnc/send_gcode std_srvs/srv/SetString "{data: '$J=G91G21X10'}"

# Move another 5mm in Y direction
ros2 service call /cnc/send_gcode std_srvs/srv/SetString "{data: '$J=G91G21Y5'}"
```

## Movement Patterns

### Grid Pattern

This example creates a 3x3 grid pattern with 10mm spacing:

```bash
# Set absolute mode
ros2 service call /cnc/set_absolute_mode std_srvs/srv/Trigger "{}"

# Set feed rate
ros2 service call /cnc/set_feed_rate std_srvs/srv/SetString "{data: '500'}"

# Go to starting position
ros2 service call /cnc/send_gcode std_srvs/srv/SetString "{data: '$J=G90G21X0Y0'}"

# Create grid pattern
for row in 0 1 2; do
  for col in 0 1 2; do
    x=$((col * 10))
    y=$((row * 10))
    ros2 service call /cnc/send_gcode std_srvs/srv/SetString "{data: '$J=G90G21X$xY$y'}"
    # Wait briefly at each point
    sleep 0.5
  done
done
```

### Circular Pattern

This example moves in a circular pattern (approximate):

```bash
# Set absolute mode
ros2 service call /cnc/set_absolute_mode std_srvs/srv/Trigger "{}"

# Go to center position
ros2 service call /cnc/send_gcode std_srvs/srv/SetString "{data: '$J=G90G21X50Y50'}"

# Create circular pattern
for angle in $(seq 0 30 360); do
  # Calculate position on circle with radius 20
  x=$(echo "50 + 20 * c($angle * 3.14159 / 180)" | bc -l)
  y=$(echo "50 + 20 * s($angle * 3.14159 / 180)" | bc -l)
  
  # Format to 2 decimal places
  x=$(printf "%.2f" $x)
  y=$(printf "%.2f" $y)
  
  # Move to position
  ros2 service call /cnc/send_gcode std_srvs/srv/SetString "{data: '$J=G90G21X$xY$y'}"
done
```

## Target Origo Movement

The CNC controller provides services for target origo operations:

```bash
# Move to position above target origo (at safe Z height)
ros2 service call /cnc/move_over_target_origo std_srvs/srv/Trigger "{}"

# Dock at target origo (if within XY tolerance)
ros2 service call /cnc/dock_at_target_origo std_srvs/srv/Trigger "{}"

# Move directly to target origo (with safety checks)
ros2 service call /cnc/move_to_target_origo std_srvs/srv/Trigger "{}"
```

## Jog Control

For incremental movements, use the jog service with JSON-formatted parameters:

```bash
# Jog 10mm in X direction
ros2 service call /cnc/jog_increment std_srvs/srv/SetString "{data: '{\"x\": 10.0, \"feed\": 500, \"relative\": true}'}"

# Jog -5mm in Z direction
ros2 service call /cnc/jog_increment std_srvs/srv/SetString "{data: '{\"z\": -5.0, \"feed\": 200, \"relative\": true}'}"

# Move to absolute position using jog
ros2 service call /cnc/jog_increment std_srvs/srv/SetString "{data: '{\"x\": 100.0, \"y\": 50.0, \"feed\": 500, \"relative\": false}'}"
```

## Monitoring Movement

### Position Tracking

Monitor the current position:

```bash
ros2 topic echo /cnc/position
```

### Status Monitoring

Monitor the machine status:

```bash
ros2 topic echo /cnc/status
```

### Response Monitoring

Monitor raw responses from the GRBL controller:

```bash
ros2 topic echo /cnc/response
```

## Safe Movement Practices

### Understanding Inertia and Movement Dynamics

CNC machines are subject to physical forces that affect their movement:

- **Inertia**: The machine's moving parts have mass and resist changes in motion
- **Acceleration/Deceleration**: Rapid changes in speed can cause vibrations and mechanical stress
- **Momentum**: At high speeds, the machine requires time and distance to stop smoothly

These physical properties can cause:
- Vibration when starting or stopping quickly
- Overshooting of target positions
- Mechanical stress on components
- Reduced precision at higher speeds

To minimize these effects:
- Start with slower feed rates and gradually increase
- Avoid frequent, small movements (these cause "jerky" motion)
- Plan for smooth, continuous paths rather than sharp direction changes
- Allow enough distance for acceleration and deceleration
- Consider using G1 (controlled movement) instead of G0 (rapid) for precision work

### Pre-Movement Checklist

Before executing movements:

1. Ensure the machine is homed
2. Verify the movement mode (absolute or relative)
3. Set appropriate feed rate for the operation
4. Check that the path is clear of obstacles

### Post-Movement Verification

After executing a movement:

1. Verify that the machine reached the intended position
2. Check for any error messages
3. Move to a safe position when done

### Error Recovery

If an error occurs:

1. Reset the controller:
   ```bash
   ros2 service call /cnc/reset std_srvs/srv/Trigger "{}"
   ```

2. Unlock if necessary:
   ```bash
   ros2 service call /cnc/send_gcode std_srvs/srv/SetString "{data: '$X'}"
   ```

3. Re-home if position is uncertain:
   ```bash
   ros2 service call /cnc/home std_srvs/srv/Trigger "{}"
   ```

## Advanced Movement Techniques

### Coordinated Multi-Axis Movement

Move multiple axes simultaneously for smooth paths:

```bash
ros2 service call /cnc/send_gcode std_srvs/srv/SetString "{data: '$J=G1X100Y50Z25F400'}"
```

### Sequential Movements

Execute a sequence of movements:

```bash
# Create a function to send G-code and wait for completion
send_gcode_and_wait() {
  ros2 service call /cnc/send_gcode std_srvs/srv/SetString "{data: '$1'}"
  sleep 1  # Adjust based on movement time
}

# Execute sequence
send_gcode_and_wait "$J=G90G21X0Y0"
send_gcode_and_wait "$J=G90G21X100Y0"
send_gcode_and_wait "$J=G90G21X100Y100"
send_gcode_and_wait "$J=G90G21X0Y100"
send_gcode_and_wait "$J=G90G21X0Y0"
```

### Creating a Movement Script

For complex movement patterns, create a separate script:

```bash
#!/bin/zsh

# Set absolute mode
ros2 service call /cnc/set_absolute_mode std_srvs/srv/Trigger "{}"

# Set feed rate
ros2 service call /cnc/set_feed_rate std_srvs/srv/SetString "{data: '500'}"

# Define movement function
function move_to {
  ros2 service call /cnc/send_gcode std_srvs/srv/SetString "{data: '$J=G90G21X$1Y$2'}"
  sleep $3
}

# Execute movement pattern
move_to 0 0 1
move_to 100 0 2
move_to 100 100 2
move_to 0 100 2
move_to 0 0 1

echo "Movement pattern completed"
```

Save as `movement_pattern.sh` and make executable:
```bash
chmod +x movement_pattern.sh
./movement_pattern.sh
```
