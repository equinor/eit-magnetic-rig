# Magnetic Sensor Node API Documentation

This document provides comprehensive API documentation for the Magnetic Sensor Node within the Magnetic Homing ROS2 package.

## Overview

The Magnetic Sensor Node (`magnetic_sensor_node.py`) communicates with magnetic sensors over TCP/IP to read magnetic field strength values. It implements a specific protocol for querying the sensors and publishes the data for other nodes to consume.

## Published Topics

### `/magnetic_sensor_data`

**Message Types:**
- `magnetic_homing/MagneticSensorData` (primary)
- `std_msgs/Float32MultiArray` (fallback)

**Description:**
Publishes the magnetic field strength readings from the four sensors (A, B, C, and Z). The node attempts to use the custom `MagneticSensorData` message type if available, with a fallback to `Float32MultiArray` if the custom message type cannot be found.

**Fields (MagneticSensorData):**
- `float32 sensor_a` - Reading from sensor A
- `float32 sensor_b` - Reading from sensor B
- `float32 sensor_c` - Reading from sensor C
- `float32 sensor_z` - Reading from sensor Z

**Fields (Float32MultiArray):**
- `data[0]` - Reading from sensor A
- `data[1]` - Reading from sensor B
- `data[2]` - Reading from sensor C
- `data[3]` - Reading from sensor Z

**Frequency:**
Publishes at the rate defined by the `publish_rate` parameter (default: 10 Hz).

## Services

### `/set_sensor_ip`

**Service Type:** `std_srvs/SetBool`

**Description:**
Dynamically reconfigures the IP address used to connect to the magnetic sensor device.

**Request:**
- `bool data` - Contains the new IP address as a string

**Response:**
- `bool success` - Indicates whether the IP change was successful
- `string message` - Description of the result or error

## Parameters

### Sensor Configuration

- **`sensor_ip`** (string, default: "192.168.1.100")
  - The IP address of the magnetic sensor device
  - Can be changed at runtime using the `/set_sensor_ip` service

- **`publish_rate`** (double, default: 10.0)
  - The frequency at which sensor data is published, in Hz

### Calibration Parameters

- **`calibration_factor`** (double, default: 1.0)
  - Scaling factor applied to raw sensor values
  - Used to convert raw values to meaningful units

- **`calibration_offset`** (double, default: 0.0)
  - Offset applied to sensor values after scaling
  - Formula: calibrated_value = raw_value * calibration_factor + calibration_offset

## Protocol Details

### Command Structure

The node implements a specific binary protocol for communicating with the magnetic sensors:

- **Sensor A Command:** `0x01, 0x01, 0x01, 0x01, 0x04, 0x10, 0x03, 0x10, 0x03, 0x7C, 0x9F`
- **Sensor B Command:** `0x01, 0x01, 0x01, 0x01, 0x04, 0x10, 0x03, 0x10, 0x04, 0xBE, 0xDE`
- **Sensor C Command:** `0x01, 0x01, 0x01, 0x01, 0x04, 0x10, 0x03, 0x10, 0x05, 0x7E, 0x1F`
- **Sensor Z Command:** `0x01, 0x01, 0x01, 0x01, 0x04, 0x10, 0x03, 0x10, 0x06, 0x7F, 0x5F`
- **All Sensors Command:** `0x01, 0x01, 0x01, 0x01, 0x10, 0x10, 0x03, 0x10, 0x03, 0x10, 0x03, 0x10, 0x04, 0x10, 0x03, 0x10, 0x05, 0x10, 0x03, 0x10, 0x06, 0x70, 0x75`

Commands are sent to TCP port 2468 on the sensor device IP address.

### Response Format

Responses follow a specific format with binary data containing sensor readings. The node handles parsing this data to extract the magnetic field strength values.

## Error Handling

The node implements several error handling mechanisms:

1. **Connection Errors:**
   - Detects TCP connection failures
   - Attempts to reconnect periodically
   - Logs connection status

2. **Protocol Errors:**
   - Validates response packet structure
   - Detects malformed or incomplete responses
   - Logs protocol errors for debugging

3. **Graceful Degradation:**
   - Falls back to previously known values when readings fail
   - Continues operation even when some sensors are unresponsive

## Usage Examples

### Launch Parameters

To start the node with custom parameters:

```bash
ros2 run magnetic_homing magnetic_sensor_node.py --ros-args -p sensor_ip:=192.168.1.150 -p publish_rate:=20.0
```

### Dynamic IP Reconfiguration

To change the sensor IP address at runtime:

```bash
ros2 service call /set_sensor_ip std_srvs/srv/SetBool "{data: '192.168.1.200'}"
```

### Monitoring Sensor Data

To monitor the published sensor data:

```bash
# For custom message type
ros2 topic echo /magnetic_sensor_data

# For array values
ros2 topic echo /magnetic_sensor_data.data
```

## Advanced Configuration

### Calibration Process

Calibration parameters can be adjusted to convert raw sensor values to meaningful units:

```bash
ros2 param set /magnetic_sensor_node calibration_factor 1.25
ros2 param set /magnetic_sensor_node calibration_offset -0.5
```

This would apply the formula: `calibrated_value = raw_value * 1.25 - 0.5`
