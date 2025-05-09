# Troubleshooting Guide

This document provides solutions for common issues that may arise when using the Magnetic Homing ROS2 package.

## Magnetic Sensor Issues

### No Sensor Data

**Symptoms:**
- No messages on the `/magnetic_sensor_data` topic
- Error messages about connection failures

**Possible Causes and Solutions:**

1. **Incorrect IP Address**
   - Verify the IP address of your sensor device
   - Use `ping <sensor-ip>` to check connectivity
   - Update the IP address: `ros2 service call /set_sensor_ip std_srvs/srv/SetBool "{data: '192.168.1.xxx'}"`

2. **Network Connectivity Issues**
   - Ensure your computer and the sensor are on the same network
   - Check network cables and connections
   - Verify no firewall is blocking the connection (port 2468)

3. **Sensor Power Issues**
   - Ensure the sensor has power
   - Check indicator lights on the sensor device

### Inconsistent or Noisy Sensor Data

**Symptoms:**
- Wildly fluctuating sensor readings
- Inconsistent behavior during homing

**Possible Causes and Solutions:**

1. **Calibration Issues**
   - Recalibrate the sensors: `ros2 param set /magnetic_sensor_node calibration_factor <value>`
   - Adjust the offset: `ros2 param set /magnetic_sensor_node calibration_offset <value>`

2. **Interference**
   - Remove nearby metal objects or sources of electromagnetic interference
   - Shield the sensors if necessary

3. **Damaged Sensors**
   - Test each sensor individually
   - Replace damaged sensors if necessary

## CNC Controller Issues

### Connection Failures

**Symptoms:**
- Error messages about serial port
- No status updates on `/cnc/status` topic

**Possible Causes and Solutions:**

1. **Incorrect Serial Port**
   - List available ports: `ls /dev/tty*`
   - Update the serial port parameter: `ros2 param set /cnc_controller_node serial_port /dev/ttyXXX`

2. **Permission Issues**
   - Add your user to the dialout group: `sudo usermod -a -G dialout $USER`
   - Set permissions: `sudo chmod 666 /dev/ttyXXX`
   - Log out and log back in for group changes to take effect

3. **USB Connection Issues**
   - Try a different USB port
   - Use a different USB cable
   - Restart the CNC controller

### GRBL Alarm State

**Symptoms:**
- Status shows "Alarm" state
- Machine won't move
- Error messages when sending commands

**Possible Causes and Solutions:**

1. **Machine Not Homed**
   - Home the machine: `ros2 service call /cnc/home std_srvs/srv/Trigger "{}"`
   - Unlock GRBL: `ros2 service call /cnc/send_gcode std_srvs/srv/SetBool "{data: '$X'}"`

2. **Limit Switch Triggered**
   - Check if any limit switches are physically triggered
   - Manually move the machine away from the limit switch
   - Reset GRBL: `ros2 service call /cnc/reset std_srvs/srv/Trigger "{}"`

3. **Command Outside Machine Limits**
   - Ensure movement commands are within machine limits
   - Check soft limit settings (`$20=1`)

### Movement Issues

**Symptoms:**
- Machine moves in unexpected ways
- Movements are too fast or too slow
- Positioning is inaccurate

**Possible Causes and Solutions:**

1. **Incorrect Movement Mode**
   - Check current mode in status messages
   - Set to absolute mode: `ros2 service call /cnc/set_absolute_mode std_srvs/srv/Trigger "{}"`
   - Set to relative mode: `ros2 service call /cnc/set_relative_mode std_srvs/srv/Trigger "{}"`

2. **Feed Rate Issues**
   - Set an appropriate feed rate: `ros2 service call /cnc/set_feed_rate std_srvs/srv/SetBool "{data: '500'}"`
   - Check acceleration settings in GRBL

3. **Mechanical Issues**
   - Check for loose belts or couplings
   - Ensure smooth movement of all axes
   - Check for obstructions in the movement path

## Message Compilation Issues

**Symptoms:**
- Errors about missing message types
- Node falling back to Float32MultiArray

**Possible Causes and Solutions:**

1. **Package Not Built**
   - Rebuild the package: `colcon build --packages-select magnetic_homing`
   - Source the setup files: `source install/setup.zsh`

2. **Missing Dependencies**
   - Install required dependencies: `rosdep install --from-paths src --ignore-src -r -y`
   - Check for error messages during build

## Integration Issues

**Symptoms:**
- Custom nodes can't access sensor data
- Custom controllers not working properly

**Possible Causes and Solutions:**

1. **Message Type Mismatch**
   - Ensure you're using the correct message type (MagneticSensorData vs Float32MultiArray)
   - Check the import statements in your code

2. **Service Client Issues**
   - Wait for service to be available before calling
   - Handle service call failures gracefully

3. **ROS2 Node Graph Issues**
   - Check node connections: `ros2 node list` and `ros2 topic list`
   - Use `ros2 topic info <topic>` to verify publishers and subscribers
   - Use `ros2 service list` to verify available services

## Performance Issues

**Symptoms:**
- High CPU usage
- Delayed responses
- Dropped messages

**Possible Causes and Solutions:**

1. **Publishing Rate Too High**
   - Reduce sensor publishing rate: `ros2 param set /magnetic_sensor_node publish_rate 5.0`
   - Adjust status interval: `ros2 param set /cnc_controller_node status_interval 0.5`

2. **Resource Contention**
   - Close unnecessary applications
   - Use `top` or `htop` to identify resource-hungry processes
   - Consider running on a more powerful machine

## Debugging Tools

Use these commands to gather more information for troubleshooting:

1. **View Node Information**
   ```bash
   ros2 node list
   ros2 node info /magnetic_sensor_node
   ros2 node info /cnc_controller_node
   ```

2. **Check Topic Data**
   ```bash
   ros2 topic list
   ros2 topic echo /magnetic_sensor_data
   ros2 topic echo /cnc/status
   ros2 topic info /magnetic_sensor_data
   ```

3. **Check Parameter Values**
   ```bash
   ros2 param list
   ros2 param get /magnetic_sensor_node sensor_ip
   ros2 param get /cnc_controller_node serial_port
   ```

4. **Monitor Service Calls**
   ```bash
   ros2 service list
   ros2 service type /cnc/send_gcode
   ```

5. **ROS2 Doctor**
   ```bash
   ros2 doctor
   ```

If issues persist after trying these solutions, please file an issue in the repository with detailed information about your setup and the problem you're experiencing.
