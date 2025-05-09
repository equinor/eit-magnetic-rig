# Custom Controller Example

This example demonstrates how to create a custom controller node that uses magnetic sensor data to control CNC movement. It implements a simple tracking algorithm that follows the strongest magnetic field.

## Implementation

Create a file named `magnetic_field_tracker.py` with the following content:

```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_srvs.srv import SetBool, Trigger
import math
import time
import numpy as np

# Import custom message if available, otherwise use Float32MultiArray
try:
    from magnetic_homing.msg import MagneticSensorData
    USE_CUSTOM_MSG = True
    print("Using custom MagneticSensorData message type")
except ImportError:
    from std_msgs.msg import Float32MultiArray
    USE_CUSTOM_MSG = False
    print("Custom message type not available, using Float32MultiArray")

class MagneticFieldTracker(Node):
    """
    A controller that tracks the strongest magnetic field by controlling 
    CNC movement based on magnetic sensor readings.
    """
    
    def __init__(self):
        super().__init__('magnetic_field_tracker')
        
        # Parameters
        self.declare_parameter('target_strength', 800.0)
        self.declare_parameter('step_size', 1.0)  # mm
        self.declare_parameter('fine_step_size', 0.2)  # mm
        self.declare_parameter('threshold_percentage', 0.90)  # 90% of target
        self.declare_parameter('settle_time', 0.5)  # seconds between movements
        
        self.target_strength = self.get_parameter('target_strength').value
        self.step_size = self.get_parameter('step_size').value
        self.fine_step_size = self.get_parameter('fine_step_size').value
        self.threshold_percentage = self.get_parameter('threshold_percentage').value
        self.settle_time = self.get_parameter('settle_time').value
        
        # State variables
        self.sensor_values = [0.0, 0.0, 0.0, 0.0]  # Initial values for sensors A, B, C, Z
        self.sensor_history = []  # Store last N readings for smoothing
        self.history_size = 5
        self.is_tracking = False
        self.current_phase = "coarse"  # coarse, fine, complete
        self.last_max_value = 0.0
        self.last_direction = None
        self.last_movement_time = 0.0
        
        # Create subscription to sensor data
        if USE_CUSTOM_MSG:
            self.subscription = self.create_subscription(
                MagneticSensorData,
                'magnetic_sensor_data',
                self.sensor_callback_custom,
                10
            )
        else:
            self.subscription = self.create_subscription(
                Float32MultiArray,
                'magnetic_sensor_data',
                self.sensor_callback_array,
                10
            )
        
        # Create service clients for CNC control
        self.send_gcode_client = self.create_client(SetBool, '/cnc/send_gcode')
        self.set_absolute_mode_client = self.create_client(Trigger, '/cnc/set_absolute_mode')
        self.set_relative_mode_client = self.create_client(Trigger, '/cnc/set_relative_mode')
        self.set_feed_rate_client = self.create_client(SetBool, '/cnc/set_feed_rate')
        
        # Create service for starting and stopping tracking
        self.start_service = self.create_service(
            Trigger, 'start_tracking', self.start_tracking_callback)
        self.stop_service = self.create_service(
            Trigger, 'stop_tracking', self.stop_tracking_callback)
        
        # Create timer for tracking loop
        self.tracking_timer = self.create_timer(0.1, self.tracking_loop)
        
        self.get_logger().info("Magnetic Field Tracker initialized")
        
    def sensor_callback_custom(self, msg):
        """Callback for custom MagneticSensorData messages"""
        self.sensor_values = [msg.sensor_a, msg.sensor_b, msg.sensor_c, msg.sensor_z]
        self.update_sensor_history()
        
    def sensor_callback_array(self, msg):
        """Callback for Float32MultiArray messages"""
        if len(msg.data) >= 4:
            self.sensor_values = list(msg.data)[:4]
            self.update_sensor_history()
    
    def update_sensor_history(self):
        """Update the history of sensor readings for smoothing"""
        self.sensor_history.append(self.sensor_values.copy())
        if len(self.sensor_history) > self.history_size:
            self.sensor_history.pop(0)
    
    def get_smoothed_values(self):
        """Get smoothed sensor values by averaging the history"""
        if not self.sensor_history:
            return self.sensor_values
        
        # Calculate mean of each sensor over history
        smoothed = [0.0, 0.0, 0.0, 0.0]
        for i in range(4):
            values = [reading[i] for reading in self.sensor_history]
            smoothed[i] = sum(values) / len(values)
        
        return smoothed
    
    def start_tracking_callback(self, request, response):
        """Service callback to start magnetic field tracking"""
        if self.is_tracking:
            response.success = False
            response.message = "Tracking already in progress"
            return response
        
        # Initialize tracking
        self.is_tracking = True
        self.current_phase = "coarse"
        self.last_max_value = 0.0
        self.sensor_history = []
        
        # Set up the CNC controller
        self.setup_cnc()
        
        response.success = True
        response.message = "Magnetic field tracking started"
        self.get_logger().info("Magnetic field tracking started")
        return response
    
    def stop_tracking_callback(self, request, response):
        """Service callback to stop magnetic field tracking"""
        if not self.is_tracking:
            response.success = False
            response.message = "Tracking not in progress"
            return response
        
        self.is_tracking = False
        response.success = True
        response.message = "Magnetic field tracking stopped"
        self.get_logger().info("Magnetic field tracking stopped")
        return response
    
    def setup_cnc(self):
        """Set up the CNC controller for tracking"""
        # Set relative positioning mode
        self.call_service(self.set_relative_mode_client, Trigger.Request())
        
        # Set feed rate
        feed_rate_req = SetBool.Request()
        feed_rate_req.data = "200"  # Medium-slow feed rate for control
        self.call_service(self.set_feed_rate_client, feed_rate_req)
        
        self.get_logger().info("CNC controller set up for tracking")
    
    def tracking_loop(self):
        """Main tracking logic, called periodically by the timer"""
        if not self.is_tracking:
            return
            
        # Get smoothed sensor values
        smoothed_values = self.get_smoothed_values()
        
        # Find maximum value and corresponding sensor
        max_value = max(smoothed_values)
        max_index = smoothed_values.index(max_value)
        
        # Wait for settle time between movements
        current_time = time.time()
        if current_time - self.last_movement_time < self.settle_time:
            return
            
        # Log current status
        self.get_logger().info(
            f"Phase: {self.current_phase}, "
            f"Values: {[round(v, 2) for v in smoothed_values]}, "
            f"Max: {round(max_value, 2)} at sensor {max_index}"
        )
        
        # Check if we've reached target strength
        if max_value >= self.target_strength:
            if self.current_phase == "coarse":
                self.get_logger().info("Switching to fine tracking")
                self.current_phase = "fine"
            elif self.current_phase == "fine":
                self.get_logger().info("Target field strength reached!")
                self.current_phase = "complete"
                self.is_tracking = False
                return
        
        # If field strength is decreasing, we might have passed the peak
        if self.last_max_value > 0 and max_value < self.last_max_value * self.threshold_percentage:
            self.get_logger().info("Field strength decreasing, possible peak overshoot")
            # If we've overshot, try reversing direction
            if self.last_direction:
                reverse_direction = self.get_opposite_direction(self.last_direction)
                self.move_in_direction(reverse_direction, self.fine_step_size)
                self.last_direction = reverse_direction
        
        # Determine and execute movement
        if self.current_phase != "complete":
            direction = self.determine_direction(max_index, smoothed_values)
            step = self.step_size if self.current_phase == "coarse" else self.fine_step_size
            self.move_in_direction(direction, step)
            self.last_direction = direction
            
        # Update last max value
        if max_value > self.last_max_value:
            self.last_max_value = max_value
            
    def determine_direction(self, max_sensor_index, values):
        """
        Determine movement direction based on sensor readings
        Customize this based on your sensor arrangement
        """
        # Simple algorithm that moves toward the strongest sensor
        if max_sensor_index == 0:  # Sensor A strongest (assume +X direction)
            return "X+"
        elif max_sensor_index == 1:  # Sensor B strongest (assume +Y direction)
            return "Y+"
        elif max_sensor_index == 2:  # Sensor C strongest (assume -X direction) 
            return "X-"
        elif max_sensor_index == 3:  # Sensor Z strongest (assume -Y direction)
            return "Y-"
            
        # If all sensors are equal (unlikely), move in +X direction
        return "X+"
        
    def get_opposite_direction(self, direction):
        """Get the opposite direction for reversing movement"""
        opposites = {
            "X+": "X-", "X-": "X+",
            "Y+": "Y-", "Y-": "Y+",
            "Z+": "Z-", "Z-": "Z+"
        }
        return opposites.get(direction, "X+")
        
    def move_in_direction(self, direction, distance):
        """Send G-code command to move in specified direction"""
        # Parse direction and create G-code
        axis = direction[0]
        sign = direction[1]  # + or -
        value = distance if sign == "+" else -distance
        
        g_code = f"G0 {axis}{value}"
        
        # Send the command
        self.get_logger().info(f"Moving: {g_code}")
        request = SetBool.Request()
        request.data = g_code
        self.call_service(self.send_gcode_client, request)
        
        # Update last movement time
        self.last_movement_time = time.time()
        
    def call_service(self, client, request):
        """Helper function for calling services with proper waiting"""
        # Wait for service to be available
        if not client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn(f"Service {client.srv_name} not available")
            return None
            
        # Call the service asynchronously
        future = client.call_async(request)
        # For simplicity, we're not waiting for the result in this example
        return future


def main(args=None):
    rclpy.init(args=args)
    node = MagneticFieldTracker()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

## Usage

1. **Build and source the workspace**:
   ```bash
   cd ~/ros2_ws
   colcon build --packages-select magnetic_homing
   source install/setup.zsh
   ```

2. **Start the base nodes**:
   ```bash
   ros2 launch magnetic_homing magnetic_homing.launch.py
   ```

3. **Run the custom controller**:
   ```bash
   # Save the Python script above as magnetic_field_tracker.py
   chmod +x magnetic_field_tracker.py
   ./magnetic_field_tracker.py
   ```

4. **Start magnetic field tracking**:
   ```bash
   ros2 service call /start_tracking std_srvs/srv/Trigger "{}"
   ```

5. **Stop tracking at any time**:
   ```bash
   ros2 service call /stop_tracking std_srvs/srv/Trigger "{}"
   ```

## Algorithm Explanation

This controller implements a magnetic field tracking algorithm with two phases:

1. **Coarse tracking**: Uses larger step sizes to quickly approach the target field strength.
2. **Fine tracking**: Uses smaller step sizes for precise positioning once close to target.

The controller determines movement direction based on which sensor detects the strongest field. It also includes:

- Sensor reading smoothing to reduce noise
- Overshoot detection to prevent oscillation
- Configurable parameters for different environments

## Customization Options

- Adjust `target_strength` based on your specific magnets and calibration
- Modify `step_size` and `fine_step_size` based on your machine's precision
- Change the `settle_time` to balance speed vs. stability
- Customize the sensor-to-direction mapping in `determine_direction()` to match your sensor arrangement

## Advanced Implementation Ideas

- Add a grid search pattern for initial field detection
- Implement gradient descent for more efficient field tracking
- Add visualization of sensor values and machine position
- Record tracking data for analysis and optimization

For more examples of custom controllers, see the Magnetic Homing ROS2 package documentation.
