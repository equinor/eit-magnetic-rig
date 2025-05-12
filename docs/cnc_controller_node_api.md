# CNC Controller Node API Documentation

This document provides comprehensive API documentation for the CNC Controller Node within the Magnetic Homing ROS2 package.

## Overview

The CNC Controller Node communicates with a GRBL-based CNC controller over a serial connection. It provides a ROS2 interface for sending commands, receiving status updates, and controlling the CNC rig.

## Node Details
- Name: `cnc_controller_node`
- Implementation: `magnetic_homing/cnc_controller_node.py`

## Parameters

### Connection Parameters

- **`serial_port`** (string, default: "/dev/ttyUSB0")
  - The serial port connected to the GRBL controller

- **`baud_rate`** (integer, default: 115200)
  - The baud rate for serial communication

- **`status_interval`** (double, default: 0.2)
  - The interval at which status messages are requested from GRBL, in seconds

### State Tracking

The node internally tracks several states:

- **Movement Mode:** "absolute" or "relative"
- **Feed Rate:** Current feed rate setting in mm/min
- **Machine Position:** Current coordinates in machine space
- **Work Position:** Current coordinates in work coordinate space
- **Machine State:** Current GRBL state (Idle, Run, Hold, Alarm, etc.)

### Target Origo Parameters

- **`target_origo_x`** (float, default: 524.0)
  - Target X coordinate for docking

- **`target_origo_y`** (float, default: 318.0)
  - Target Y coordinate for docking

- **`target_origo_z`** (float, default: -235.0)
  - Target Z coordinate for docking

- **`safe_z_for_xy_traverse`** (float, default: -185.0)
  - Safe Z height for XY movements

## Topics

### Published Topics

### `/cnc/status`

**Message Type:** `std_msgs/String`

**Description:**
Publishes the CNC controller status as a JSON-formatted string with machine state and position information.

**Contents:**
```json
{
  "state": "Idle",  // GRBL state (Idle, Run, Hold, Alarm, etc.)
  "mpos": {"x": 0.0, "y": 0.0, "z": 0.0},  // Machine position
  "wpos": {"x": 0.0, "y": 0.0, "z": 0.0},  // Work position
  "movement_mode": "absolute",  // Current movement mode (absolute or relative)
  "feed_rate": 100.0  // Current feed rate setting
}
```

**Frequency:** 
Publishes at the rate defined by the `status_interval` parameter (default: 0.2 Hz).

### `/cnc/position`

**Message Type:** `std_msgs/Float32MultiArray`

**Description:**
Publishes the current position of the CNC machine as an array of coordinates.

**Contents:**
- `data[0]` - X position
- `data[1]` - Y position
- `data[2]` - Z position

**Frequency:** 
Publishes whenever the position changes and at the rate defined by the `status_interval` parameter.

### `/cnc/response`

**Message Type:** `std_msgs/String`

**Description:**
Publishes raw response messages from the GRBL controller, including command acknowledgments, error messages, and status reports.

**Contents:**
Raw string responses from the GRBL controller.

**Frequency:**
Publishes whenever a message is received from the GRBL controller.

## Subscribed Topics

### `/magnetic_sensor_data`

**Message Type:** 
- `magnetic_homing/MagneticSensorData` (primary)
- `std_msgs/Float32MultiArray` (fallback)

**Description:**
Subscribes to magnetic sensor data from the Magnetic Sensor Node.

## Services

### Movement Services (All Using Standard Service Types)

#### `/cnc/move_to_target_origo` (std_srvs/Trigger)
Safely moves to the configured target origo position:
1. Lifts to safe Z height if below
2. Moves to target XY
3. Descends to target Z

#### `/cnc/move_over_target_origo` (std_srvs/Trigger)
Moves to position above target origo at safe Z height.

#### `/cnc/dock_at_target_origo` (std_srvs/Trigger)  
Docks at target origo if already within XY tolerance.

#### `/cnc/jog_increment` (std_srvs/SetString)
Executes a jog movement. Expects JSON-formatted string:
```json
{
  "x": float,       // X increment (optional)
  "y": float,       // Y increment (optional)
  "z": float,       // Z increment (optional)
  "feed": float,    // Feed rate (optional)
  "relative": bool  // Use relative coords (default: true)
}
```

### `/cnc/send_gcode`

**Service Type:** `std_srvs/SetString`

**Request:**
- `string data` - Contains the G-code command

**Response:**
- `bool success` - Indicates whether the command was sent successfully
- `string message` - Description of the result or error

### `/cnc/home`

**Service Type:** `std_srvs/Trigger`

**Description:**
Executes the homing procedure ($H) on the CNC machine.

**Request:** Empty

**Response:**
- `bool success` - Indicates whether the homing command was sent successfully
- `string message` - Description of the result or error

### `/cnc/reset`

**Service Type:** `std_srvs/Trigger`

**Description:**
Resets the GRBL controller with a soft reset command (Ctrl+X).

**Request:** Empty

**Response:**
- `bool success` - Indicates whether the reset was successful
- `string message` - Description of the result or error

### `/cnc/set_absolute_mode`

**Service Type:** `std_srvs/Trigger`

**Description:**
Sets the CNC controller to absolute positioning mode (G90).

**Request:** Empty

**Response:**
- `bool success` - Indicates whether the mode was set successfully
- `string message` - Description of the result or error

### `/cnc/set_relative_mode`

**Service Type:** `std_srvs/Trigger`

**Description:**
Sets the CNC controller to relative positioning mode (G91).

**Request:** Empty

**Response:**
- `bool success` - Indicates whether the mode was set successfully
- `string message` - Description of the result or error

### `/cnc/set_feed_rate`

**Service Type:** `std_srvs/SetString`

**Request:**
- `string data` - Contains the feed rate value

**Response:**
- `bool success` - Indicates whether the feed rate was set successfully
- `string message` - Description of the result or error

## Parameters

### Connection Parameters

- **`serial_port`** (string, default: "/dev/ttyUSB0")
  - The serial port connected to the GRBL controller

- **`baud_rate`** (integer, default: 115200)
  - The baud rate for serial communication

- **`status_interval`** (double, default: 0.2)
  - The interval at which status messages are requested from GRBL, in seconds

### State Tracking

The node internally tracks several states:

- **Movement Mode:** "absolute" or "relative"
- **Feed Rate:** Current feed rate setting in mm/min
- **Machine Position:** Current coordinates in machine space
- **Work Position:** Current coordinates in work coordinate space
- **Machine State:** Current GRBL state (Idle, Run, Hold, Alarm, etc.)

### Target Origo Parameters

- **`target_origo_x`** (float, default: 524.0)
  - Target X coordinate for docking

- **`target_origo_y`** (float, default: 318.0)
  - Target Y coordinate for docking

- **`target_origo_z`** (float, default: -235.0)
  - Target Z coordinate for docking

- **`safe_z_for_xy_traverse`** (float, default: -185.0)
  - Safe Z height for XY movements

## GRBL Communication

### Command Structure

The node communicates with GRBL using standard G-code commands and special GRBL commands:

- **G-code Commands:** Standard G-code (G0, G1, G90, G91, etc.)
- **GRBL Commands:**
  - `$X` - Unlock GRBL
  - `$H` - Start homing cycle
  - `?` - Status report query
  - Ctrl+X - Soft reset

### Response Handling

Responses from GRBL are handled in several ways:

1. **Status Responses:** Parsed to extract position and state information
2. **Command Responses:** Checked for success ("ok") or error messages
3. **Error Messages:** Parsed and logged with appropriate error codes

## Threaded Architecture

The node uses a threaded architecture to handle asynchronous communication:

1. **Main Thread:** Handles ROS2 services and message processing
2. **Serial Read Thread:** Continuously reads from the serial port
3. **Status Polling Thread:** Periodically requests status updates from GRBL

## Error Handling

The node implements several error handling mechanisms:

1. **Connection Errors:**
   - Detects serial connection failures
   - Logs connection errors
   - Gracefully handles disconnection and reconnection

2. **Command Errors:**
   - Validates commands before sending
   - Times out on non-responsive commands
   - Returns detailed error information

3. **State Recovery:**
   - Maintains state tracking even during errors
   - Provides reset functionality to recover from error states

## Usage Examples

### Launch Parameters

To start the node with custom parameters:

```bash
ros2 run magnetic_homing cnc_controller_node.py --ros-args -p serial_port:=/dev/ttyACM0 -p baud_rate:=115200
```

### Movement Commands

#### Absolute Mode (G90)

To move to specific coordinates:

```bash
# First, set absolute mode
ros2 service call /cnc/set_absolute_mode std_srvs/srv/Trigger "{}"

# Then move to X=100, Y=50
ros2 service call /cnc/send_gcode std_srvs/srv/SetBool "{data: 'G0 X100 Y50'}"
```

#### Relative Mode (G91)

To move relative to current position:

```bash
# First, set relative mode
ros2 service call /cnc/set_relative_mode std_srvs/srv/Trigger "{}"

# Then move 10mm in X and 5mm in Y from current position
ros2 service call /cnc/send_gcode std_srvs/srv/SetBool "{data: 'G0 X10 Y5'}"
```

### Feed Rate Control

To set the feed rate:

```bash
ros2 service call /cnc/set_feed_rate std_srvs/srv/SetBool "{data: '500'}"
```

### Homing Procedure

To initiate the homing procedure:

```bash
ros2 service call /cnc/home std_srvs/srv/Trigger "{}"
```

### Monitoring Status

To monitor the CNC controller status:

```bash
ros2 topic echo /cnc/status
```

### Resetting After Errors

If the machine is in an alarm state, you can reset it:

```bash
ros2 service call /cnc/reset std_srvs/srv/Trigger "{}"
```
