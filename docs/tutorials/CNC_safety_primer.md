# CNC Safety Primer

This document provides a basic introduction to CNC machine operation with GRBL controllers, focusing on safety considerations and best practices. This primer is intended to complement the technical documentation for the Magnetic Homing System, not to replace formal training or manufacturer guidance.

> **IMPORTANT DISCLAIMER**: This document is not a comprehensive safety manual. The user is ultimately responsible for their own safety, the safety of others, and the proper operation of equipment. Always refer to machine-specific manuals, receive proper training, and follow local safety regulations.

## Understanding CNC Basics

### What is a CNC Machine?
Computer Numerical Control (CNC) machines are automated manufacturing tools that operate based on pre-programmed instructions. The machine follows G-code commands to perform precise movements and operations.

### GRBL Controllers
GRBL is an open-source firmware used to control the movement of CNC machines. It interprets G-code commands and translates them into motor signals.

## Movement Modes: Absolute vs. Relative

Understanding the difference between absolute and relative movement is critical for safe CNC operation:

### Absolute Positioning (G90)
- Coordinates are measured from a fixed zero point (origin)
- Example: `G0 X10 Y10` will move to position 10,10 in the workspace
- Useful for precise, predictable positioning
- **Our system default**: The controller defaults to absolute mode on startup

### Relative Positioning (G91)
- Coordinates are measured from the current position
- Example: `G0 X10 Y10` will move 10 units right and 10 units forward from current position
- Useful for incremental movements
- **Use with caution**: Always be aware of your current position

## Safety Guidelines for CNC Operation

### Before Operation
1. **Workspace Preparation**
   - Ensure the machine area is clear of obstacles
   - Secure all materials properly
   - Check for proper tool installation

2. **System Check**
   - Verify all connections (power, data)
   - Ensure emergency stop systems are accessible
   - Test limit switches if available

3. **Software Preparation**
   - Verify your G-code before sending
   - Start with slow feed rates when testing new operations
   - Use visualization tools when available to simulate movements

### During Operation
1. **Monitoring**
   - Never leave a running CNC machine unattended
   - Stay within reach of emergency stop controls
   - Watch for unusual sounds or movements

2. **Feed Rates and Speed**
   - Start with slower feed rates than you think necessary
   - Increase speeds gradually after confirming proper operation

3. **Emergency Procedures**
   - Know how to quickly stop the machine (software and hardware methods)
   - Keep appropriate safety equipment nearby

### After Operation
1. **Shutdown Procedure**
   - Return to a safe position before powering down
   - Disable motors when not in use

## Special Considerations for Magnetic Homing System

### Working with Sensors
- Understand the capabilities and limitations of the magnetic sensors
- Be aware that sensor readings may be affected by nearby metal objects
- Test sensor responsiveness before relying on it for positioning

### Safe Movement Patterns
- When testing new homing procedures, use a "step and check" approach
- Implement movement limits in software to prevent collisions
- Consider implementing soft limits in GRBL (`$20=1`)
- **IMPORTANT**: The CNC rig is mechanically capable of crashing into itself at some of the extreme minimum and maximum positions. Always operate within safe travel limits and approach boundaries with caution.

### Troubleshooting
- If the machine behaves unexpectedly, use the emergency stop
- After an emergency stop, reset the controller before resuming
- Regularly check sensor calibration

## GRBL Commands for Safety

Learn these essential GRBL commands for safe operation:

- `$X` - Unlock GRBL after an alarm
- `!` - Feed hold (pause)
- `~` - Cycle start/resume
- `Ctrl+X` - Soft reset
- `?` - Status report
- `$` - View settings

## Understanding G-Code Basics

Basic G-code commands that affect safety:

- `G0` - Rapid move (fastest possible speed)
- `G1` - Linear move at specified feed rate
- `G90` - Set absolute positioning mode
- `G91` - Set relative positioning mode
- `F` - Set feed rate (e.g., `F500` for 500 mm/min)

## Using the ROS2 Interface Safely

When using the ROS2 interface for controlling the CNC machine:

1. **Start with a Reset and Home**
   ```bash
   ros2 service call /cnc/reset std_srvs/srv/Trigger "{}"
   ros2 service call /cnc/home std_srvs/srv/Trigger "{}"
   ```

2. **Always Monitor Status**
   ```bash
   ros2 topic echo /cnc/status
   ```

3. **Set a Safe Feed Rate**
   ```bash
   ros2 service call /cnc/set_feed_rate std_srvs/srv/setBool "{data: '100'}"
   ```

4. **Be Explicit About Movement Mode**
   ```bash
   # Always specify which mode you're using
   ros2 service call /cnc/set_absolute_mode std_srvs/srv/Trigger "{}"
   # or
   ros2 service call /cnc/set_relative_mode std_srvs/srv/Trigger "{}"
   ```

5. **Test Movements Incrementally**
   - Start with small movements to verify behavior
   - Increase distance or complexity only after confirming proper operation
   - Example movement command:
   ```bash
   ros2 service call /cnc/send_gcode std_srvs/srv/setBool "{data: 'G0 X10 Y10'}"
   ```

## Final Safety Reminders

1. **Understand Your Equipment**: Read manuals and specifications for all hardware components.

2. **Training First**: Get proper training before operating CNC equipment.

3. **Safety Equipment**: Use appropriate personal protection equipment.

4. **Machine Maintenance**: Regularly inspect and maintain all components.

5. **Be Present and Alert**: Never operate CNC equipment when tired or impaired.

Remember, safety is an ongoing process. Stay informed, stay cautious, and prioritize safety over speed or convenience.
