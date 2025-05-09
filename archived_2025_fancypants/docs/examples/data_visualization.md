# Data Visualization

This example demonstrates how to visualize magnetic sensor data and CNC position information using RViz and matplotlib in Python.

## RViz Visualization

RViz is a powerful visualization tool for ROS2 that can be used to visualize both magnetic field data and CNC machine position.

### Installation

If you don't have RViz2 installed:

```bash
sudo apt install ros-humble-rviz2
```

### Creating a MarkerArray for Magnetic Field Visualization

Create a file named `magnetic_field_visualizer.py`:

```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from visualization_msgs.msg import MarkerArray, Marker
from std_msgs.msg import ColorRGBA
from geometry_msgs.msg import Point
import math

# Import custom message if available, otherwise use Float32MultiArray
try:
    from magnetic_homing.msg import MagneticSensorData
    USE_CUSTOM_MSG = True
except ImportError:
    from std_msgs.msg import Float32MultiArray
    USE_CUSTOM_MSG = False
    print("Custom message type not available, using Float32MultiArray")

class MagneticFieldVisualizer(Node):
    """
    Visualizes magnetic field data as arrows in RViz
    """
    
    def __init__(self):
        super().__init__('magnetic_field_visualizer')
        
        # Parameters
        self.declare_parameter('sensor_positions', [
            [0.0, 0.0, 0.0],    # Sensor A position (x, y, z)
            [100.0, 0.0, 0.0],  # Sensor B position
            [100.0, 100.0, 0.0], # Sensor C position
            [0.0, 100.0, 0.0]   # Sensor Z position
        ])
        self.declare_parameter('scale_factor', 0.01)
        self.declare_parameter('max_field_value', 1000.0)
        
        self.sensor_positions = self.get_parameter('sensor_positions').value
        self.scale_factor = self.get_parameter('scale_factor').value
        self.max_field_value = self.get_parameter('max_field_value').value
        
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
        
        # Create publisher for marker array
        self.marker_publisher = self.create_publisher(
            MarkerArray,
            'magnetic_field_markers',
            10
        )
        
        self.get_logger().info("Magnetic Field Visualizer started")
    
    def sensor_callback_custom(self, msg):
        """Callback for custom MagneticSensorData messages"""
        sensor_values = [msg.sensor_a, msg.sensor_b, msg.sensor_c, msg.sensor_z]
        self.publish_markers(sensor_values)
    
    def sensor_callback_array(self, msg):
        """Callback for Float32MultiArray messages"""
        if len(msg.data) >= 4:
            sensor_values = list(msg.data)[:4]
            self.publish_markers(sensor_values)
    
    def publish_markers(self, sensor_values):
        """Create and publish marker array for visualization"""
        marker_array = MarkerArray()
        
        # Create a marker for each sensor
        for i, (value, position) in enumerate(zip(sensor_values, self.sensor_positions)):
            # Create arrow marker
            arrow_marker = Marker()
            arrow_marker.header.frame_id = "map"
            arrow_marker.header.stamp = self.get_clock().now().to_msg()
            arrow_marker.ns = f"sensor_{i}"
            arrow_marker.id = i
            arrow_marker.type = Marker.ARROW
            arrow_marker.action = Marker.ADD
            
            # Set start point at sensor position
            start_point = Point()
            start_point.x = float(position[0])
            start_point.y = float(position[1])
            start_point.z = float(position[2])
            
            # Set end point with length proportional to field strength
            end_point = Point()
            # Simplified: just extend in z-direction for visualization
            end_point.x = float(position[0])
            end_point.y = float(position[1])
            end_point.z = float(position[2] + value * self.scale_factor)
            
            arrow_marker.points = [start_point, end_point]
            
            # Set size
            arrow_marker.scale.x = 5.0  # shaft diameter
            arrow_marker.scale.y = 10.0  # head diameter
            arrow_marker.scale.z = 10.0  # head length
            
            # Color based on field strength (green -> yellow -> red)
            normalized_value = min(1.0, value / self.max_field_value)
            color = ColorRGBA()
            color.r = normalized_value  # Red component increases with field strength
            color.g = 1.0 - normalized_value * 0.5  # Green decreases slightly
            color.b = 0.0
            color.a = 1.0  # Fully opaque
            arrow_marker.color = color
            
            # Add to marker array
            marker_array.markers.append(arrow_marker)
            
            # Also add text marker for value
            text_marker = Marker()
            text_marker.header.frame_id = "map"
            text_marker.header.stamp = self.get_clock().now().to_msg()
            text_marker.ns = f"sensor_text_{i}"
            text_marker.id = i + 100  # Different ID from arrow
            text_marker.type = Marker.TEXT_VIEW_FACING
            text_marker.action = Marker.ADD
            text_marker.pose.position = start_point
            text_marker.pose.position.z += 20.0  # Position text above arrow
            text_marker.scale.z = 20.0  # Text size
            text_marker.color.r = 1.0
            text_marker.color.g = 1.0
            text_marker.color.b = 1.0
            text_marker.color.a = 1.0
            text_marker.text = f"Sensor {['A', 'B', 'C', 'Z'][i]}: {value:.2f}"
            
            # Add to marker array
            marker_array.markers.append(text_marker)
        
        # Publish marker array
        self.marker_publisher.publish(marker_array)


def main(args=None):
    rclpy.init(args=args)
    node = MagneticFieldVisualizer()
    
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

### Visualizing CNC Position

Create a file named `cnc_position_visualizer.py`:

```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import Point
import json

class CNCPositionVisualizer(Node):
    """
    Visualizes CNC position as a marker in RViz
    """
    
    def __init__(self):
        super().__init__('cnc_position_visualizer')
        
        # Create subscription to CNC position
        self.position_sub = self.create_subscription(
            Float32MultiArray,
            '/cnc/position',
            self.position_callback,
            10
        )
        
        # Also subscribe to status to get work position
        self.status_sub = self.create_subscription(
            Float32MultiArray,
            '/cnc/status',
            self.status_callback,
            10
        )
        
        # Create publisher for marker
        self.marker_publisher = self.create_publisher(
            Marker,
            'cnc_position_marker',
            10
        )
        
        # Initialize position
        self.machine_position = [0.0, 0.0, 0.0]
        self.work_position = [0.0, 0.0, 0.0]
        
        # Create timer for publishing marker
        self.timer = self.create_timer(0.1, self.publish_marker)
        
        self.get_logger().info("CNC Position Visualizer started")
    
    def position_callback(self, msg):
        """Callback for CNC position updates"""
        if len(msg.data) >= 3:
            self.machine_position = list(msg.data)[:3]
    
    def status_callback(self, msg):
        """Callback for CNC status updates to get work position"""
        try:
            status_str = msg.data.decode('utf-8')
            status_json = json.loads(status_str)
            if 'wpos' in status_json:
                wpos = status_json['wpos']
                self.work_position = [wpos['x'], wpos['y'], wpos['z']]
        except Exception as e:
            self.get_logger().warn(f"Error parsing status: {e}")
    
    def publish_marker(self):
        """Create and publish marker for CNC position"""
        # Create marker for machine position
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "cnc_position"
        marker.id = 0
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        
        # Set position
        marker.pose.position.x = self.machine_position[0]
        marker.pose.position.y = self.machine_position[1]
        marker.pose.position.z = self.machine_position[2]
        marker.pose.orientation.w = 1.0
        
        # Set size
        marker.scale.x = 10.0
        marker.scale.y = 10.0
        marker.scale.z = 10.0
        
        # Set color (blue)
        marker.color.r = 0.0
        marker.color.g = 0.0
        marker.color.b = 1.0
        marker.color.a = 1.0
        
        # Publish marker
        self.marker_publisher.publish(marker)


def main(args=None):
    rclpy.init(args=args)
    node = CNCPositionVisualizer()
    
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

### RViz Configuration

Create an RViz configuration file named `magnetic_homing_viz.rviz`:

```
Panels:
  - Class: rviz_common/Displays
    Help Height: 78
    Name: Displays
    Property Tree Widget:
      Expanded:
        - /Global Options1
        - /Status1
        - /MarkerArray1
        - /Marker1
      Splitter Ratio: 0.5
    Tree Height: 719
  - Class: rviz_common/Selection
    Name: Selection
  - Class: rviz_common/Tool Properties
    Expanded:
      - /2D Pose Estimate1
      - /2D Nav Goal1
      - /Publish Point1
    Name: Tool Properties
    Splitter Ratio: 0.5886790156364441
  - Class: rviz_common/Views
    Expanded:
      - /Current View1
    Name: Views
    Splitter Ratio: 0.5
Visualization Manager:
  Class: ""
  Displays:
    - Alpha: 0.5
      Cell Size: 50
      Class: rviz_default_plugins/Grid
      Color: 160; 160; 164
      Enabled: true
      Line Style:
        Line Width: 0.029999999329447746
        Value: Lines
      Name: Grid
      Normal Cell Count: 0
      Offset:
        X: 0
        Y: 0
        Z: 0
      Plane: XY
      Plane Cell Count: 10
      Reference Frame: <Fixed Frame>
      Value: true
    - Class: rviz_default_plugins/MarkerArray
      Enabled: true
      Name: MarkerArray
      Namespaces:
        {}
      Topic:
        Depth: 5
        Durability Policy: Volatile
        History Policy: Keep Last
        Reliability Policy: Reliable
        Value: /magnetic_field_markers
      Value: true
    - Class: rviz_default_plugins/Marker
      Enabled: true
      Name: Marker
      Namespaces:
        {}
      Topic:
        Depth: 5
        Durability Policy: Volatile
        History Policy: Keep Last
        Reliability Policy: Reliable
        Value: /cnc_position_marker
      Value: true
  Enabled: true
  Global Options:
    Background Color: 48; 48; 48
    Fixed Frame: map
    Frame Rate: 30
  Name: root
  Tools:
    - Class: rviz_default_plugins/Interact
      Hide Inactive Objects: true
    - Class: rviz_default_plugins/MoveCamera
    - Class: rviz_default_plugins/Select
    - Class: rviz_default_plugins/FocusCamera
    - Class: rviz_default_plugins/Measure
      Line color: 128; 128; 0
    - Class: rviz_default_plugins/SetInitialPose
      Topic:
        Depth: 5
        Durability Policy: Volatile
        History Policy: Keep Last
        Reliability Policy: Reliable
        Value: /initialpose
    - Class: rviz_default_plugins/SetGoal
      Topic:
        Depth: 5
        Durability Policy: Volatile
        History Policy: Keep Last
        Reliability Policy: Reliable
        Value: /goal_pose
    - Class: rviz_default_plugins/PublishPoint
      Single click: true
      Topic:
        Depth: 5
        Durability Policy: Volatile
        History Policy: Keep Last
        Reliability Policy: Reliable
        Value: /clicked_point
  Transformation:
    Current:
      Class: rviz_default_plugins/TF
  Value: true
  Views:
    Current:
      Class: rviz_default_plugins/Orbit
      Distance: 500
      Enable Stereo Rendering:
        Stereo Eye Separation: 0.05999999865889549
        Stereo Focal Distance: 1
        Swap Stereo Eyes: false
        Value: false
      Focal Point:
        X: 50
        Y: 50
        Z: 0
      Focal Shape Fixed Size: true
      Focal Shape Size: 0.05000000074505806
      Invert Z Axis: false
      Name: Current View
      Near Clip Distance: 0.009999999776482582
      Pitch: 0.785398006439209
      Target Frame: <Fixed Frame>
      Value: Orbit (rviz)
      Yaw: 0.785398006439209
    Saved: ~
```

### Running the Visualization

1. **Start the Magnetic Homing system**:
   ```bash
   ros2 launch magnetic_homing magnetic_homing.launch.py
   ```

2. **Run the visualizer nodes**:
   ```bash
   # Terminal 1
   python3 magnetic_field_visualizer.py
   
   # Terminal 2
   python3 cnc_position_visualizer.py
   ```

3. **Start RViz with the configuration**:
   ```bash
   ros2 run rviz2 rviz2 -d magnetic_homing_viz.rviz
   ```

## Matplotlib Real-time Visualization

For more detailed data visualization and analysis, you can use matplotlib to create real-time plots.

Create a file named `magnetic_data_plotter.py`:

```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import numpy as np
import time
from threading import Lock

# Import custom message if available, otherwise use Float32MultiArray
try:
    from magnetic_homing.msg import MagneticSensorData
    USE_CUSTOM_MSG = True
except ImportError:
    from std_msgs.msg import Float32MultiArray
    USE_CUSTOM_MSG = False
    print("Custom message type not available, using Float32MultiArray")

class MagneticDataPlotter(Node):
    """
    Plots magnetic sensor data in real-time using matplotlib
    """
    
    def __init__(self):
        super().__init__('magnetic_data_plotter')
        
        # Initialize data storage
        self.data_lock = Lock()
        self.time_data = []
        self.sensor_data = [[], [], [], []]  # A, B, C, Z
        self.start_time = time.time()
        
        # Maximum points to display (window size)
        self.max_points = 100
        
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
        
        # Set up the plot
        self.fig, self.ax = plt.subplots(figsize=(10, 6))
        self.lines = [
            self.ax.plot([], [], label=f"Sensor {label}")[0]
            for label in ['A', 'B', 'C', 'Z']
        ]
        
        self.ax.set_title('Magnetic Field Strength Over Time')
        self.ax.set_xlabel('Time (s)')
        self.ax.set_ylabel('Field Strength')
        self.ax.set_ylim(0, 1000)  # Adjust based on your sensor calibration
        self.ax.legend()
        self.ax.grid(True)
        
        # Create animation
        self.ani = FuncAnimation(
            self.fig, self.update_plot, interval=100, blit=True)
        
        self.get_logger().info("Magnetic Data Plotter started")
    
    def sensor_callback_custom(self, msg):
        """Callback for custom MagneticSensorData messages"""
        sensor_values = [msg.sensor_a, msg.sensor_b, msg.sensor_c, msg.sensor_z]
        self.update_data(sensor_values)
    
    def sensor_callback_array(self, msg):
        """Callback for Float32MultiArray messages"""
        if len(msg.data) >= 4:
            sensor_values = list(msg.data)[:4]
            self.update_data(sensor_values)
    
    def update_data(self, values):
        """Update the data storage with new sensor values"""
        with self.data_lock:
            current_time = time.time() - self.start_time
            
            self.time_data.append(current_time)
            for i, val in enumerate(values):
                self.sensor_data[i].append(val)
            
            # Limit the amount of data stored
            if len(self.time_data) > self.max_points:
                self.time_data = self.time_data[-self.max_points:]
                for i in range(4):
                    self.sensor_data[i] = self.sensor_data[i][-self.max_points:]
    
    def update_plot(self, frame):
        """Update the plot with the latest data"""
        with self.data_lock:
            if not self.time_data:
                return self.lines
                
            # Update data for each line
            for i, line in enumerate(self.lines):
                line.set_data(self.time_data, self.sensor_data[i])
            
            # Adjust x-axis limits to show all data
            self.ax.set_xlim(
                self.time_data[0] if self.time_data else 0,
                self.time_data[-1] if self.time_data else 10
            )
            
            # Auto-adjust y-axis if needed
            all_data = [val for sensor_data in self.sensor_data for val in sensor_data]
            if all_data:
                max_val = max(all_data)
                min_val = min(all_data)
                range_val = max_val - min_val
                self.ax.set_ylim(
                    max(0, min_val - 0.1 * range_val),
                    max_val + 0.1 * range_val
                )
        
        return self.lines


def main(args=None):
    rclpy.init(args=args)
    node = MagneticDataPlotter()
    
    try:
        plt.show()  # This will block until the plot window is closed
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

### Running the Matplotlib Visualization

```bash
# Ensure you have matplotlib installed
pip install matplotlib

# Run the plotter
python3 magnetic_data_plotter.py
```

## Heatmap Visualization

For visualizing magnetic field strength across a plane, you can create a heatmap by collecting data while moving the CNC machine in a grid pattern.

Create a file named `magnetic_field_heatmap.py`:

```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_srvs.srv import SetBool, Trigger
import numpy as np
import matplotlib.pyplot as plt
import time

# Import custom message if available, otherwise use Float32MultiArray
try:
    from magnetic_homing.msg import MagneticSensorData
    USE_CUSTOM_MSG = True
except ImportError:
    from std_msgs.msg import Float32MultiArray
    USE_CUSTOM_MSG = False
    print("Custom message type not available, using Float32MultiArray")

class MagneticFieldHeatmap(Node):
    """
    Creates a heatmap of magnetic field strength by moving the CNC machine
    in a grid pattern and recording sensor values
    """
    
    def __init__(self):
        super().__init__('magnetic_field_heatmap')
        
        # Parameters
        self.declare_parameter('grid_size_x', 100)
        self.declare_parameter('grid_size_y', 100)
        self.declare_parameter('grid_steps_x', 10)
        self.declare_parameter('grid_steps_y', 10)
        self.declare_parameter('settle_time', 0.5)
        self.declare_parameter('sensor_index', 0)  # 0=A, 1=B, 2=C, 3=Z
        
        self.grid_size_x = self.get_parameter('grid_size_x').value
        self.grid_size_y = self.get_parameter('grid_size_y').value
        self.grid_steps_x = self.get_parameter('grid_steps_x').value
        self.grid_steps_y = self.get_parameter('grid_steps_y').value
        self.settle_time = self.get_parameter('settle_time').value
        self.sensor_index = self.get_parameter('sensor_index').value
        
        # Initialize data grid
        self.grid_data = np.zeros((self.grid_steps_y, self.grid_steps_x))
        self.current_sensor_values = [0.0, 0.0, 0.0, 0.0]
        self.is_scanning = False
        self.current_x = 0
        self.current_y = 0
        
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
        
        # Service clients for CNC control
        self.send_gcode_client = self.create_client(SetBool, '/cnc/send_gcode')
        self.set_absolute_mode_client = self.create_client(Trigger, '/cnc/set_absolute_mode')
        self.set_feed_rate_client = self.create_client(SetBool, '/cnc/set_feed_rate')
        
        # Service to start the scan
        self.start_service = self.create_service(
            Trigger, '/start_heatmap_scan', self.start_scan_callback)
        
        self.get_logger().info("Magnetic Field Heatmap generator initialized")
    
    def sensor_callback_custom(self, msg):
        """Callback for custom MagneticSensorData messages"""
        self.current_sensor_values = [msg.sensor_a, msg.sensor_b, msg.sensor_c, msg.sensor_z]
    
    def sensor_callback_array(self, msg):
        """Callback for Float32MultiArray messages"""
        if len(msg.data) >= 4:
            self.current_sensor_values = list(msg.data)[:4]
    
    def start_scan_callback(self, request, response):
        """Service callback to start the scanning process"""
        if self.is_scanning:
            response.success = False
            response.message = "Scan already in progress"
            return response
        
        self.is_scanning = True
        self.grid_data = np.zeros((self.grid_steps_y, self.grid_steps_x))
        
        # Ensure absolute mode and set feed rate
        self.call_service(self.set_absolute_mode_client, Trigger.Request())
        
        feed_req = SetBool.Request()
        feed_req.data = "500"  # Medium feed rate
        self.call_service(self.set_feed_rate_client, feed_req)
        
        # Start scanning in a separate thread
        self.get_logger().info("Starting heatmap scan")
        
        # Use a timer to start the scanning process
        self.scan_timer = self.create_timer(0.1, self.scan_step)
        
        response.success = True
        response.message = "Heatmap scan started"
        return response
    
    def scan_step(self):
        """Execute one step of the scanning process"""
        if not self.is_scanning:
            self.scan_timer.cancel()
            return
        
        # Calculate current position
        x_pos = (self.current_x * self.grid_size_x) / (self.grid_steps_x - 1)
        y_pos = (self.current_y * self.grid_size_y) / (self.grid_steps_y - 1)
        
        # Move to position
        self.get_logger().info(f"Moving to ({x_pos}, {y_pos})")
        move_req = SetBool.Request()
        move_req.data = f"G0 X{x_pos} Y{y_pos}"
        self.call_service(self.send_gcode_client, move_req)
        
        # Wait for movement to complete and readings to stabilize
        time.sleep(self.settle_time)
        
        # Record the sensor value
        value = self.current_sensor_values[self.sensor_index]
        self.grid_data[self.current_y, self.current_x] = value
        self.get_logger().info(f"Recorded value: {value}")
        
        # Move to next position
        self.current_x += 1
        if self.current_x >= self.grid_steps_x:
            self.current_x = 0
            self.current_y += 1
            
            if self.current_y >= self.grid_steps_y:
                # Scan complete
                self.is_scanning = False
                self.scan_timer.cancel()
                self.get_logger().info("Scan complete, generating heatmap")
                self.generate_heatmap()
                return
    
    def generate_heatmap(self):
        """Generate and save a heatmap visualization"""
        # Create figure
        plt.figure(figsize=(10, 8))
        
        # Create heatmap
        plt.imshow(
            self.grid_data, 
            interpolation='bilinear',
            origin='lower',
            extent=[0, self.grid_size_x, 0, self.grid_size_y]
        )
        
        # Add colorbar and labels
        plt.colorbar(label='Magnetic Field Strength')
        plt.xlabel('X Position (mm)')
        plt.ylabel('Y Position (mm)')
        plt.title(f'Magnetic Field Heatmap - Sensor {["A", "B", "C", "Z"][self.sensor_index]}')
        
        # Save figure
        timestamp = time.strftime("%Y%m%d-%H%M%S")
        filename = f"magnetic_heatmap_{timestamp}.png"
        plt.savefig(filename)
        self.get_logger().info(f"Heatmap saved to {filename}")
        
        # Also show the figure
        plt.show()
    
    def call_service(self, client, request):
        """Helper function for calling services with proper waiting"""
        if not client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn(f"Service {client.srv_name} not available")
            return None
            
        future = client.call_async(request)
        return future


def main(args=None):
    rclpy.init(args=args)
    node = MagneticFieldHeatmap()
    
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

### Running the Heatmap Generator

```bash
# Ensure you have matplotlib and numpy installed
pip install matplotlib numpy

# Start the heatmap generator
python3 magnetic_field_heatmap.py

# In another terminal, start the scan
ros2 service call /start_heatmap_scan std_srvs/srv/Trigger "{}"
```

The scan will move the CNC machine in a grid pattern, measuring the magnetic field strength at each point. After the scan is complete, it will generate a heatmap visualization showing the field strength distribution across the scanned area.

## Combining Visualizations

For a complete visualization solution, you can combine these approaches:
- Use RViz for real-time 3D visualization
- Use matplotlib for detailed data analysis and plotting
- Use heatmaps for spatial field strength mapping

This will provide a comprehensive understanding of the magnetic field environment and CNC positioning.
