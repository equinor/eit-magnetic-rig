# Implementation Details: Magnetic Homing Sensors and CNC Controller

## Overview

This document details the implementation status of the ROS2 nodes for the Magnetic Homing Sensors and CNC Controller project. It outlines what has been implemented, how the implementation works, and what remains to be completed.

## Implemented Components

### Magnetic Sensor Node (`magnetic_sensor_node.py`)

#### Implemented Features

1. **Node Structure**
   - Initialized as a ROS2 node with proper lifecycle management
   - Parameter declaration and management for sensor configuration
   - Timer-based execution for periodic sensor readings

2. **Sensor Communication Protocol**
   - Implementation of the specific protocol for magnetic sensors
   - Command bytes defined for each sensor (A, B, C, and Z)
   - TCP socket communication with configurable IP address and port number
   - Proper data parsing from binary protocol format

3. **Data Publishing**
   - Supports both custom `MagneticSensorData` messages and legacy `Float32MultiArray` messages
   - Regular updates at configurable frequency (default: 10Hz)
   - Structured data format containing all four sensor values (A, B, C, Z)
   - Calibration support with configurable factor and offset parameters

4. **Service Interface**
   - Dynamic IP reconfiguration through ROS2 service
   - Implementation of the `set_sensor_ip` service using `std_srvs/SetBool`

5. **Error Handling**
   - Robust error detection and recovery mechanisms
   - Comprehensive logging of connection and parsing errors
   - Graceful degradation when sensor communication fails

#### Implementation Details

The magnetic sensor node establishes a TCP connection to a sensor device with the following architecture:

1. **Initialization Process**
   - The node initializes with default parameters for IP address and update rate
   - A timer is created to trigger periodic sensor readings
   - Publishes to the `magnetic_sensor_data` topic with sensor values

2. **Communication Protocol**
   - Uses specific byte sequences to query each sensor (A, B, C, Z)
   - Sends commands via TCP socket to port 2468
   - Parses binary response packets to extract sensor values
   - Each sensor is queried individually to ensure reliable readings

3. **Data Flow**
   - Sensor data is read via TCP socket
   - Raw binary data is parsed according to the protocol specification
   - Parsed values are stored in a Float32MultiArray
   - Data is published to the ROS2 topic for other nodes to consume

4. **Service Handler**
   - The `set_sensor_ip` service allows dynamic reconfiguration
   - IP address can be changed at runtime without restarting the node

### CNC Controller Node (`cnc_controller_node.py`)

#### Implemented Features

1. **Node Structure**
   - Initialized as a ROS2 node with proper lifecycle management
   - Parameter declaration and management for serial port configuration
   - Threaded architecture for non-blocking communication
   
2. **GRBL Communication**
   - Implemented serial connection with configurable port and baud rate
   - Support for core GRBL commands and response handling
   - Status polling with proper parsing of machine state and position
   - Command locking mechanism to prevent conflicts

3. **ROS2 Integration**
   - Publishers for machine status, position, and responses
   - Services for sending G-code commands, homing, and resetting
   - Subscription to magnetic sensor data
   - JSON-formatted status messages for easy integration

4. **Error Handling**
   - Comprehensive logging for connection and command errors
   - Graceful degradation when serial communication fails
   - Proper cleanup of resources when node is shut down

## What Remains to Be Done

### Magnetic Sensor Node

1. ✅ **Custom Message Type** - IMPLEMENTED
   - Custom `MagneticSensorData.msg` has been implemented
   - Node supports both custom message type and Float32MultiArray

2. ✅ **Unit Conversion and Calibration** - IMPLEMENTED
   - Added calibration parameters (factor and offset)
   - Implemented calibration in the sensor reading process

3. **Diagnostic Information**
   - Add more detailed diagnostic information about sensor status
   - Implement additional service for resetting/recalibrating sensors

### CNC Controller Node

The CNC Controller Node has been implemented as a focused GRBL controller interface with ROS2 integration:

1. ✅ **Basic Node Structure** - IMPLEMENTED
   - Successfully initialized ROS2 node with proper lifecycle management
   - Implemented serial communication with the GRBL controller
   - Added parameter handling for serial port configuration (baud rate, port name)

2. ✅ **GRBL Communication** - IMPLEMENTED
   - Implemented core G-code command sending and response parsing
   - Added GRBL status reporting (position, state, etc.)
   - Supported essential GRBL commands for initialization and configuration

3. ✅ **ROS2 Integration** - IMPLEMENTED
   - Created topics for broadcasting CNC status/telemetry to the ROS2 ecosystem
   - Implemented services for sending commands to the GRBL controller
   - Added parameter interface for controller configuration

4. ✅ **Essential Control Functions** - IMPLEMENTED
   - Implemented basic positioning and movement commands
   - Added support for homing and coordinate system management
   - Implemented movement mode control (absolute/relative positioning)
   - Added feed rate control capabilities
   - Provided error handling and recovery mechanisms

### Package Structure

The following package structure components have been implemented:

1. ✅ **ROS2 Package Structure** - IMPLEMENTED
   - Created proper package.xml with necessary dependencies
   - Added CMakeLists.txt for building the package
   - Organized code into ROS2-compliant directory structure

2. ✅ **Message Definitions** - IMPLEMENTED
   - Created custom MagneticSensorData.msg message type
   - Added message generation configuration in CMakeLists.txt
   - Implemented dual support for both custom messages and standard Float32MultiArray

   **Custom Message Format Rationale**:
   The implementation uses a custom message type (`MagneticSensorData`) with named fields rather than relying solely on standard array messages. This approach offers several benefits:
   
   - **Semantic Clarity**: Named fields (`sensor_a`, `sensor_b`, etc.) clearly indicate what each value represents
   - **Type Safety**: Reduces errors from incorrect array indexing
   - **Self-Documentation**: Message structure documents data organization
   - **Future Extensibility**: Allows adding new fields (timestamps, status flags) without breaking existing code
   
   However, the implementation also maintains compatibility with `Float32MultiArray` to provide:
   
   - **Backward Compatibility**: Works with existing systems expecting standard messages
   - **Graceful Degradation**: Functions even if custom message compilation fails
   - **Simplified Testing**: Allows testing without full message compilation setup

3. ✅ **Launch Files** - IMPLEMENTED
   - Created launch file for easy startup of both nodes
   - Added configurable parameters for hardware connections

### Additional Tasks (Pending)

1. **Documentation**
   - Add API documentation for both nodes
   - Create usage examples and tutorials

2. **Testing**
   - Develop unit tests and integration tests
   - Create test fixtures for simulating sensor data (planned but not yet implemented)
   - Implement simulated sensor data generation for testing without physical hardware
   - Create a sensor data simulator node that publishes realistic magnetic field data

3. **Safety Features**
   - Implement software limits to prevent the CNC from reaching extreme positions
   - Add emergency stop service that immediately halts all operations
   - Implement collision detection based on machine coordinates
   - Add fail-safe mechanisms for communication failures

4. **Configuration Management**
   - Implement loading/saving of GRBL configurations as profiles
   - Add support for different machine-specific configurations
   - Create configuration validation to prevent unsafe settings
   - Provide UI-friendly parameter descriptions

5. **Enhanced Visualization**
   - Implement RViz plugins for visualizing sensor data and magnetic field strength
   - Create real-time position visualization for the CNC machine
   - Add trajectory planning visualization
   - Create a dashboard for monitoring system status and sensor readings

## Next Steps

1. Test the system with real hardware
2. Implement simulated sensor data for testing without physical hardware
3. Add safety features such as software limits and emergency stop
4. Implement configuration management for GRBL settings
5. Add visualization and monitoring tools
6. Implement advanced control algorithms

## Testing Instructions

To test the system with the current implementation:

1. Build the ROS2 package:
   ```
   cd /Volumes/External/Dev/250424_magnetic_homing
   colcon build
   source install/setup.zsh
   ```

2. Launch the system with appropriate parameters:
   ```
   ros2 launch magnetic_homing magnetic_homing.launch.py sensor_ip:=<ACTUAL_IP> serial_port:=<ACTUAL_PORT>
   ```

3. Monitor the system:
   ```
   # In separate terminals:
   ros2 topic echo /magnetic_sensor_data
   ros2 topic echo /cnc/status
   ros2 topic echo /cnc/position
   ```

4. Send commands to the CNC controller:
   ```
   ros2 service call /cnc/send_gcode std_srvs/srv/SetBool "{data: 'G0 X10 Y10'}"
   ```

## Technical References

- For magnetic sensor protocol details, refer to `equinor-eit-magnetic-rig.txt` and `240120OYG01 Protocol specification Homing 0.2.pdf`
- For CNC control, refer to GRBL documentation and OpenBuilds Blackbox specifications
- ROS2 documentation provides information on creating services, topics, and message types