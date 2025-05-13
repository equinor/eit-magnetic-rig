# Quick Start Guide

This guide will help you quickly set up and start using the Magnetic Homing ROS2 package.

## Prerequisites

- ROS2 (Humble or Foxy recommended)
- Python 3.8 or higher
- `pyserial` package installed 
- A complete ROS2 development environment for message compilation

## Installation

1. **Clone the repository** into your ROS2 workspace:
   ```bash
   cd ~/ros2_ws/src
   git clone <repository-url> magnetic_homing
   ```

2. **Build the package** to compile the custom message type:
   ```bash
   cd ~/ros2_ws
   colcon build --packages-select magnetic_homing
   source install/setup.zsh
   ```

## Hardware Setup

1. **Magnetic Sensors**:
   - Connect the magnetic sensor array to your network
   - Note the IP address (default is `192.168.1.254`)
   - Ensure the device is accessible over TCP port 2468

2. **CNC Controller**:
   - Connect the GRBL-based CNC controller (e.g., OpenBuilds Blackbox) via USB
   - Note the serial port (e.g., `/dev/ttyUSB0` on Linux, `/dev/tty.usbserial-*` on macOS)
   - Ensure you have appropriate permissions to access the serial port

## Starting the System

### Using the Launch File (Recommended)

The easiest way to start both nodes with proper configuration is using the provided launch file:

```bash
ros2 launch magnetic_homing magnetic_homing.launch.py sensor_ip:=192.168.1.254 serial_port:=/dev/ttyUSB0
```

Customize the `sensor_ip` and `serial_port` parameters to match your hardware configuration.

### Starting Individual Nodes

Alternatively, you can start each node individually:

1. **Start the Magnetic Sensor Node**:
   ```bash
   ros2 run magnetic_homing magnetic_sensor_node.py --ros-args -p sensor_ip:=192.168.1.254
   ```

2. **Start the CNC Controller Node**:
   ```bash
   ros2 run magnetic_homing cnc_controller_node.py --ros-args -p serial_port:=/dev/ttyUSB0
   ```

## Verifying Operation

After starting the nodes, verify that they are operating correctly:

1. **Check published topics**:
   ```bash
   ros2 topic list
   ```
   You should see `/magnetic_sensor_data`, `/cnc/status`, `/cnc/position`, and `/cnc/response`.

2. **Monitor sensor data**:
   ```bash
   ros2 topic echo /magnetic_sensor_data
   ```
   You should see regular updates with readings from sensors A, B, C, and Z.

3. **Monitor CNC status**:
   ```bash
   ros2 topic echo /cnc/status
   ```
   You should see JSON-formatted status information including machine state and position.

## Basic Operations

### Homing the Machine

Before performing any movements, it's recommended to home the machine:

```bash
ros2 service call /cnc/home std_srvs/srv/Trigger "{}"
```

### Setting Movement Mode

Choose between absolute or relative positioning:

```bash
# For absolute positioning (move to specific coordinates)
ros2 service call /cnc/set_absolute_mode std_srvs/srv/Trigger "{}"

# For relative positioning (move relative to current position)
ros2 service call /cnc/set_relative_mode std_srvs/srv/Trigger "{}"
```

### Setting Feed Rate

Set the movement speed:

```bash
ros2 service call /cnc/set_feed_rate std_srvs/srv/SetBool "{data: '500'}"
```

### Moving the CNC Machine

Send G-code commands to move the machine:

```bash
# In absolute mode, move to coordinates X=100, Y=100
ros2 service call /cnc/send_gcode std_srvs/srv/SetBool "{data: 'G0 X100 Y100'}"

# In relative mode, move 10mm in X and -5mm in Y
ros2 service call /cnc/send_gcode std_srvs/srv/SetBool "{data: 'G0 X10 Y-5'}"
```

### Target Position Setup

1. Configure target origo position:
```bash
# Set target origo coordinates (example values)
ros2 param set /cnc_controller_node target_origo_x 524.0
ros2 param set /cnc_controller_node target_origo_y 318.0
ros2 param set /cnc_controller_node target_origo_z -235.0
```

2. Basic target movement commands:
```bash
# Move to position above target
ros2 service call /cnc/move_over_target_origo std_srvs/Trigger
# Dock at target (if within tolerance)
ros2 service call /cnc/dock_at_target_origo std_srvs/Trigger
```

### Emergency Procedures

1. **Immediate Stop:**
```bash
ros2 service call /cnc/emergency_stop std_srvs/Trigger
```

2. **After Emergency Stop:**
   - Wait for complete stop
   - Clear alarms: `ros2 service call /cnc/unlock_alarm std_srvs/Trigger`
   - Re-home if needed: Send `$H` via gcode service

### Key Configuration Parameters

The following parameters control critical movement behaviors:

- `target_origo_x/y/z`: Target docking position
- `safe_z_for_xy_traverse`: Safe Z height for XY movements
- `docking_xy_tolerance`: Maximum allowed XY offset for docking

For complete parameter documentation, see `docs/cnc_controller_node_api.md`.

### Common Issues and Solutions

1. **No Movement:**
   - Check if in alarm state
   - Verify within soft limits
   - Try unlocking with `$X`

2. **Position Lost:**
   - Use emergency stop
   - Re-home machine
   - Verify parameters

3. **Erratic Movement:**
   - Check feed rates
   - Verify Z safety height
   - Consider emergency stop

## Next Steps

- Read the [CNC Safety Primer](CNC_safety_primer.md) for important safety information
- Check the [Magnetic Sensor Calibration](sensor_calibration.md) tutorial to calibrate your sensors
- Explore the [CNC Movement Control](movement_control.md) guide for more advanced movement options
- Learn about implementing a [Homing Procedure](homing_implementation.md) using magnetic sensors

## See Also

For more comprehensive information about the Magnetic Homing ROS2 package, refer to these resources:

- [Main Documentation Index](../index.md) - Overview of all available documentation
- [Physical Setup](../physical_setup.md) - Description of the CNC rig, sensor arrangement, and charging system
- [API Documentation](../magnetic_sensor_node_api.md) - Detailed API reference for both nodes
- [GRBL Command Reference](../references/grbl_commands.md) - Reference for GRBL commands used with the CNC controller
- [Troubleshooting Guide](../references/troubleshooting.md) - Solutions for common issues
- [Example Implementations](../examples/custom_controller.md) - Example code for extending the system

These documents contain all the advanced guidance, integration examples, and troubleshooting information you might need when working with this package.
