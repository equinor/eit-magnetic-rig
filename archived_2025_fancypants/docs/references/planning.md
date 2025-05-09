# Planning Document for ROS2 Magnetic Homing Package

## Overview
This document outlines the plan for creating a ROS2 package with two main nodes:
1. **Magnetic Sensor Node**: Interacts with magnetic homing sensors to read magnetic strength values dynamically.
2. **CNC Controller Node**: Communicates with a GRBL-based CNC controller (OpenBuilds Blackbox) to control a CNC rig.

---

## Magnetic Sensor Node

### Features
- **Initialization**: Configure the IP address and initialize the sensors.
- **Data Reading**: Continuously read magnetic strength values from sensors A, B, C, and Z.
- **ROS2 Integration**:
  - Publish sensor data to a topic (e.g., `/magnetic_sensor_data`).
  - Provide a service to reconfigure the IP address dynamically.

### Implementation Steps
1. **Create a ROS2 Package**:
   - Create the `magnetic_homing` package with both nodes.
   - Add dependencies for networking and message generation (e.g., `rclpy`, `std_msgs`).

2. **Implement Sensor Communication**:
   - Parse the protocol specification from the provided PDF (`240120OYG01 Protocol specification Homing 0.2.pdf`).
   - Use Python's `socket` library to communicate with the sensors over TCP/IP.
   - Add support for proper protocol command bytes for sensors A, B, C, and Z.

3. **Publish Sensor Data**:
   - Create a ROS2 publisher to publish sensor readings as a custom message type (`MagneticSensorData.msg`).
   - Include fallback to `Float32MultiArray` for backward compatibility.
   - Add calibration support with factor and offset parameters.

   **Message Type Strategy**:
   - Develop a custom message type (`MagneticSensorData.msg`) with named fields for sensor values for the following reasons:
     - Semantic clarity through named fields (sensor_a, sensor_b, etc.)
     - Type safety to reduce indexing errors
     - Self-documenting structure for better code readability
     - Future extensibility for adding metadata without breaking changes
   - Maintain fallback support for standard `Float32MultiArray` to ensure:
     - Backward compatibility with existing systems
     - Graceful degradation if custom message compilation isn't available
     - Simplified testing without message compilation setup

4. **Dynamic Reconfiguration**:
   - Implement a ROS2 service to allow users to change the IP address at runtime.

---

## CNC Controller Node

### Features
- **Initialization**: Configure the serial port and home the CNC rig.
- **Movement Control**: Support both absolute and relative positioning modes.
- **Parameter Control**: Configure feed rates and other GRBL settings.
- **Status Reporting**: Provide continuous status updates via ROS2 topics.
- **ROS2 Integration**:
  - Subscribe to sensor data from the Magnetic Sensor Node.
  - Provide services to send custom G-code commands and control movement modes.

### Implementation Steps
1. **Create a ROS2 Package**:
   - Include the CNC controller node in the `magnetic_homing` package.
   - Add dependencies for serial communication (e.g., `pyserial`).

2. **Implement GRBL Communication**:
   - Use Python's `serial` library to communicate with the GRBL controller over a serial port.
   - Implement homing and movement commands based on the GRBL protocol.
   - Create a threaded architecture for non-blocking serial communication.

3. **Subscribe to Sensor Data**:
   - Create a ROS2 subscriber to receive sensor data from the Magnetic Sensor Node.

4. **Movement Mode Control**:
   - Implement support for both absolute (G90) and relative (G91) positioning modes.
   - Provide services to switch between movement modes.
   - Track movement mode in node state.

5. **Feed Rate and Parameter Control**:
   - Implement feed rate control via dedicated service.
   - Add support for common GRBL settings and configuration.

6. **Status Reporting**:
   - Create publishers for machine status, position, and responses.
   - Use JSON-formatted status messages for easy integration.

7. **Custom G-code Service**:
   - Provide a ROS2 service to allow users to send custom G-code commands to the CNC controller.

---

## Integration and Testing

### Integration
- Ensure the Magnetic Sensor Node and CNC Controller Node can communicate via ROS2 topics and services.
- Test the end-to-end workflow: initialize sensors, read data, and control the CNC rig dynamically.
- Create launch files for easy startup of both nodes with configurable parameters.

### Testing
- Use simulated sensor data to test the CNC rig's movement logic.
- Create a dedicated sensor simulator node that publishes realistic magnetic field data.
- Verify that the nodes handle reconfiguration (e.g., changing IP address or serial port) without restarting.
- Test both absolute and relative movement modes with various feed rates.
- Develop integration tests to verify the complete system workflow.

### Safety Implementation
- Implement software-defined movement limits to prevent collisions with machine boundaries.
- Add emergency stop functionality via a dedicated ROS2 service.
- Create fail-safe mechanisms for handling communication failures.
- Implement collision detection based on machine coordinates.

### Configuration Management
- Develop a system for loading and saving GRBL configurations as profiles.
- Create configuration validation to prevent unsafe settings.
- Provide user-friendly parameter descriptions and configuration interfaces.
- Support machine-specific configurations for different CNC setups.

### Visualization and Monitoring
- Implement RViz plugins for visualizing sensor data and machine position.
- Create a position and trajectory visualization system for monitoring movements.
- Develop a dashboard for real-time monitoring of system status.
- Add magnetic field strength visualization tools for sensor calibration.

---

## Tools and Libraries
- **ROS2**: For creating nodes, topics, and services.
- **Python**: For implementing the nodes.
- **pyserial**: For serial communication with the GRBL controller.
- **Networking Libraries**: For communicating with the magnetic sensors.

---

## Reference Existing Code
- Use the Node.js code in `nodes/homing.js` and `nodes/test-pri.js` as a reference for sensor and GRBL communication.
- Adapt the logic to ROS2 and ensure compatibility with the required features.