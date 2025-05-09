# Implementing a Magnetic Homing Procedure

This tutorial guides you through implementing a magnetic homing procedure using the sensor data from the Magnetic Sensor Node to control the CNC machine through the CNC Controller Node.

## Concept Overview

Magnetic homing is the process of using magnetic field strength readings to precisely position a machine. The basic approach is:

1. Move the CNC machine in a search pattern
2. Monitor magnetic field strength from all sensors
3. Move in the direction of increasing field strength
4. Fine-tune position at the point of maximum field strength
5. Set this position as the reference point (home position)

## Prerequisites

- Both the Magnetic Sensor Node and CNC Controller Node running
- Basic understanding of ROS2 node programming
- CNC machine with proper movement controls
- Magnetic sensors properly connected and calibrated

## Creating a Magnetic Homing Node

Below is an example implementation of a magnetic homing node in Python:

```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger, SetBool
from std_msgs.msg import Float32MultiArray
import time
import math

try:
    from magnetic_homing.msg import MagneticSensorData
    USE_CUSTOM_MSG = True
except ImportError:
    USE_CUSTOM_MSG = False
    print("Custom message type not found, falling back to Float32MultiArray")

class MagneticHomingNode(Node):
    def __init__(self):
        super().__init__('magnetic_homing_node')
        
        # Initialize parameters
        self.declare_parameter('target_field_strength', 1000.0)
        self.declare_parameter('search_step_size', 1.0)  # mm
        self.declare_parameter('fine_step_size', 0.1)    # mm
        
        self.target_field_strength = self.get_parameter('target_field_strength').value
        self.search_step_size = self.get_parameter('search_step_size').value
        self.fine_step_size = self.get_parameter('fine_step_size').value
        
        # Initialize state variables
        self.is_homing = False
        self.sensor_values = [0.0, 0.0, 0.0, 0.0]  # A, B, C, Z
        self.last_max_value = 0.0
        self.current_phase = "idle"  # idle, coarse_search, fine_search, complete
        
        # Create subscription to magnetic sensor data
        if USE_CUSTOM_MSG:
            self.subscription = self.create_subscription(
                MagneticSensorData,
                'magnetic_sensor_data',
                self.sensor_data_callback_custom,
                10
            )
        else:
            self.subscription = self.create_subscription(
                Float32MultiArray,
                'magnetic_sensor_data',
                self.sensor_data_callback_array,
                10
            )
        
        # Create service clients for CNC control
        self.set_absolute_mode_client = self.create_client(Trigger, '/cnc/set_absolute_mode')
        self.set_relative_mode_client = self.create_client(Trigger, '/cnc/set_relative_mode')
        self.send_gcode_client = self.create_client(SetBool, '/cnc/send_gcode')
        self.set_feed_rate_client = self.create_client(SetBool, '/cnc/set_feed_rate')
        
        # Create service for starting the homing procedure
        self.start_homing_service = self.create_service(
            Trigger, 'start_magnetic_homing', self.start_homing_callback
        )
        
        # Create timer for homing procedure
        self.homing_timer = self.create_timer(0.2, self.homing_procedure)
        
        self.get_logger().info('Magnetic Homing Node initialized')
    
    def sensor_data_callback_custom(self, msg):
        self.sensor_values = [msg.sensor_a, msg.sensor_b, msg.sensor_c, msg.sensor_z]
    
    def sensor_data_callback_array(self, msg):
        if len(msg.data) >= 4:
            self.sensor_values = list(msg.data)[:4]
    
    def start_homing_callback(self, request, response):
        if self.is_homing:
            response.success = False
            response.message = "Homing procedure already in progress"
        else:
            self.is_homing = True
            self.current_phase = "coarse_search"
            self.last_max_value = 0.0
            
            # Initialize by setting to relative mode and slow feed rate
            self.call_service_sync(self.set_relative_mode_client, Trigger.Request())
            self.call_service_sync(self.set_feed_rate_client, SetBool.Request(data="100"))
            
            self.get_logger().info('Starting magnetic homing procedure')
            response.success = True
            response.message = "Homing procedure started"
        
        return response
    
    def homing_procedure(self):
        if not self.is_homing:
            return
        
        # Find the current maximum field strength
        max_value = max(self.sensor_values)
        max_index = self.sensor_values.index(max_value)
        
        self.get_logger().info(f'Current values: {self.sensor_values}, max: {max_value}, phase: {self.current_phase}')
        
        # Different phases of the homing procedure
        if self.current_phase == "coarse_search":
            if max_value >= self.target_field_strength:
                # Switch to fine search
                self.get_logger().info('Target field strength reached, switching to fine search')
                self.current_phase = "fine_search"
                self.last_max_value = max_value
                return
                
            # Determine movement direction based on sensor readings
            # This is a simplified approach - you may need a more sophisticated algorithm
            direction = self.determine_search_direction(max_index)
            step_size = self.search_step_size
            
            # Move in the determined direction
            self.move_in_direction(direction, step_size)
            
        elif self.current_phase == "fine_search":
            # Check if we've found the peak
            if max_value < self.last_max_value * 0.95:
                # If strength decreased significantly, we've passed the peak
                # Move back to the last position with higher reading
                self.get_logger().info('Passed peak field strength, moving back')
                self.move_back_to_peak()
                self.current_phase = "complete"
                return
                
            # Update the last max value if current reading is higher
            if max_value > self.last_max_value:
                self.last_max_value = max_value
                
            # Determine movement direction based on sensor readings
            direction = self.determine_search_direction(max_index)
            step_size = self.fine_step_size
            
            # Move in the determined direction with finer step size
            self.move_in_direction(direction, step_size)
            
        elif self.current_phase == "complete":
            # Set this position as the home position
            # This is application-specific and may require custom implementation
            self.get_logger().info('Homing procedure complete')
            self.is_homing = False
    
    def determine_search_direction(self, max_index):
        # This is a simplified algorithm - you might need a more complex one
        # based on your sensor arrangement
        if max_index == 0:  # Sensor A strongest
            return 'X+'  # Move in positive X direction
        elif max_index == 1:  # Sensor B strongest
            return 'Y+'  # Move in positive Y direction
        elif max_index == 2:  # Sensor C strongest
            return 'X-'  # Move in negative X direction
        elif max_index == 3:  # Sensor Z strongest
            return 'Y-'  # Move in negative Y direction
        return 'Z+'  # Default to Z if no clear direction
    
    def move_in_direction(self, direction, step_size):
        gcode_command = "G0 "
        
        if direction == 'X+':
            gcode_command += f"X{step_size}"
        elif direction == 'X-':
            gcode_command += f"X-{step_size}"
        elif direction == 'Y+':
            gcode_command += f"Y{step_size}"
        elif direction == 'Y-':
            gcode_command += f"Y-{step_size}"
        elif direction == 'Z+':
            gcode_command += f"Z{step_size}"
        elif direction == 'Z-':
            gcode_command += f"Z-{step_size}"
        
        # Send the command
        request = SetBool.Request()
        request.data = gcode_command
        self.get_logger().info(f'Sending command: {gcode_command}')
        self.call_service_sync(self.send_gcode_client, request)
        
    def move_back_to_peak(self):
        # This is a simplified approach - you might need a more complex one
        # Reverse the last movement to get back to peak position
        self.get_logger().info('Moving back to peak position')
        # Implementation would depend on how you're tracking movements
        
    def call_service_sync(self, client, request):
        # Helper function for synchronous service calls
        while not client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info(f'Waiting for service {client.srv_name} to become available...')
        
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        return future.result()

def main(args=None):
    rclpy.init(args=args)
    node = MagneticHomingNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Using the Magnetic Homing Node

1. **Save and Build**:
   Save the above code as `magnetic_homing_node.py` in the `scripts` directory and make it executable:
   ```bash
   chmod +x magnetic_homing_node.py
   ```

2. **Add to CMakeLists.txt**:
   Add the node to your `CMakeLists.txt`:
   ```cmake
   install(PROGRAMS
     scripts/magnetic_sensor_node.py
     scripts/cnc_controller_node.py
     scripts/magnetic_homing_node.py
     DESTINATION lib/${PROJECT_NAME}
   )
   ```

3. **Start the System**:
   ```bash
   # Start the magnetic sensor and CNC controller nodes
   ros2 launch magnetic_homing magnetic_homing.launch.py
   
   # Start the magnetic homing node
   ros2 run magnetic_homing magnetic_homing_node.py
   ```

4. **Initiate Homing**:
   ```bash
   ros2 service call /start_magnetic_homing std_srvs/srv/Trigger "{}"
   ```

## Customizing the Homing Algorithm

The provided implementation is a basic search algorithm. Depending on your specific setup, you might need to customize:

1. **Search Pattern**:
   - Grid search for initial coarse detection
   - Spiral search for broader area coverage
   - Directional search based on known field characteristics

2. **Field Strength Thresholds**:
   - Adjust `target_field_strength` based on your magnets and sensors
   - Set different thresholds for different phases of homing

3. **Sensor Arrangement**:
   - Update the `determine_search_direction()` function based on your sensor arrangement
   - Create a more sophisticated algorithm for multi-sensor integration

4. **Movement Strategy**:
   - Implement gradient descent for more efficient peak finding
   - Add hysteresis to prevent oscillation around the peak

## Advanced Implementation

For a more robust implementation, consider adding:

1. **Error Handling**:
   - Safety checks to prevent collisions
   - Timeout mechanisms for incomplete procedures
   - Recovery procedures for lost sensor readings

2. **State Machine**:
   - A proper state machine to manage more complex homing sequences
   - Support for different homing modes or strategies

3. **Calibration Integration**:
   - Auto-calibration of sensors before homing
   - Position verification after homing

4. **Data Logging**:
   - Record sensor values during homing for analysis
   - Generate homing quality metrics

## Troubleshooting

### Common Issues

1. **No sensor readings**:
   - Check sensor connections
   - Verify the magnetic sensor node is running correctly
   - Monitor the `/magnetic_sensor_data` topic

2. **CNC not moving**:
   - Check GRBL controller status
   - Verify the CNC controller node is responding to commands
   - Ensure proper movement mode is set (relative/absolute)

3. **Oscillation around peak**:
   - Reduce fine_step_size parameter
   - Implement hysteresis in the peak detection
   - Consider a different search algorithm

4. **Not finding the peak**:
   - Adjust target_field_strength parameter
   - Modify the search pattern
   - Check sensor calibration
