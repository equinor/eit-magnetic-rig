import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from std_srvs.srv import SetBool
import socket
import struct
import time
import os
import sys

# Import the custom message type if available
try:
    from magnetic_homing.msg import MagneticSensorData
    USE_CUSTOM_MSG = True
except ImportError:
    USE_CUSTOM_MSG = False
    print("Warning: MagneticSensorData message type not found, using Float32MultiArray instead")

class MagneticSensorNode(Node):
    # Command bytes for different sensor requests based on protocol
    SENSOR_A_CMD = bytes([0x01, 0x01, 0x01, 0x01, 0x04, 0x10, 0x03, 0x10, 0x03, 0x7C, 0x9F])
    SENSOR_B_CMD = bytes([0x01, 0x01, 0x01, 0x01, 0x04, 0x10, 0x03, 0x10, 0x04, 0xBE, 0xDE])
    SENSOR_C_CMD = bytes([0x01, 0x01, 0x01, 0x01, 0x04, 0x10, 0x03, 0x10, 0x05, 0x7E, 0x1F])
    SENSOR_Z_CMD = bytes([0x01, 0x01, 0x01, 0x01, 0x04, 0x10, 0x03, 0x10, 0x06, 0x7F, 0x5F])
    ALL_SENSORS_CMD = bytes([0x01, 0x01, 0x01, 0x01, 0x10, 0x10, 0x03, 0x10, 0x03, 0x10, 0x03, 0x10, 0x04, 0x10, 0x03, 0x10, 0x05, 0x10, 0x03, 0x10, 0x06, 0x70, 0x75])
    
    # TCP port from the protocol documentation
    SENSOR_PORT = 2468
    
    def __init__(self):
        super().__init__('magnetic_sensor_node')
        
        # Create the appropriate publisher based on available message types
        if USE_CUSTOM_MSG:
            self.publisher_ = self.create_publisher(MagneticSensorData, 'magnetic_sensor_data', 10)
            self.get_logger().info('Using custom MagneticSensorData message type')
        else:
            self.publisher_ = self.create_publisher(Float32MultiArray, 'magnetic_sensor_data', 10)
            self.get_logger().info('Using Float32MultiArray message type')
            
        self.srv = self.create_service(SetBool, 'set_sensor_ip', self.set_sensor_ip_callback)
        
        # Parameters
        self.declare_parameter('sensor_ip', '192.168.1.100')
        self.declare_parameter('publish_rate', 10.0)  # Hz
        self.declare_parameter('calibration_factor', 1.0)  # Scaling factor for sensor values
        self.declare_parameter('calibration_offset', 0.0)  # Offset for sensor values
        
        # Get parameter values
        self.sensor_ip = self.get_parameter('sensor_ip').get_parameter_value().string_value
        publish_rate = self.get_parameter('publish_rate').get_parameter_value().double_value
        self.calibration_factor = self.get_parameter('calibration_factor').get_parameter_value().double_value
        self.calibration_offset = self.get_parameter('calibration_offset').get_parameter_value().double_value
        
        # Sensor data storage
        self.sensor_values = [0.0, 0.0, 0.0, 0.0]  # A, B, C, Z values
        
        # Create timer for periodic sensor readings
        timer_period = 1.0 / publish_rate  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        
        # Connection status
        self.connected = False
        
        self.get_logger().info(f'Magnetic Sensor Node has been started. Connecting to sensor at {self.sensor_ip}:{self.SENSOR_PORT}')

    def set_sensor_ip_callback(self, request, response):
        self.sensor_ip = request.data
        self.get_logger().info(f'Sensor IP updated to: {self.sensor_ip}')
        response.success = True
        response.message = 'Sensor IP updated successfully.'
        return response
    
    def parse_sensor_response(self, data):
        """Parse the response from the sensors according to the protocol specification."""
        try:
            if len(data) < 7:  # Minimum packet length (header + value + checksum)
                self.get_logger().warning(f"Received short packet: {data.hex(' ')}")
                return None
                
            # Check if it's a response to a sensor read command
            if data[0] == 0x01 and data[1] == 0x02 and data[2] == 0x01:
                # Get sensor ID from the response
                sensor_id = None
                sensor_index = None
                
                # Find the sensor ID in the response
                for i in range(len(data)):
                    if i+3 < len(data) and data[i] == 0x10 and data[i+1] == 0x03 and data[i+2] == 0x10:
                        sensor_id = data[i+3]
                        if sensor_id == 0x03:  # Sensor A
                            sensor_index = 0
                        elif sensor_id == 0x04:  # Sensor B
                            sensor_index = 1
                        elif sensor_id == 0x05:  # Sensor C
                            sensor_index = 2
                        elif sensor_id == 0x06:  # Sensor Z
                            sensor_index = 3
                        break
                
                if sensor_index is not None and i+5 < len(data):
                    # Extract the 2-byte sensor value (MSB format)
                    value_bytes = data[i+4:i+6]
                    value = (value_bytes[0] << 8) | value_bytes[1]
                    
                    # Convert to appropriate units (depends on sensor calibration)
                    # For now, we're just returning the raw value
                    return sensor_index, float(value)
            
            self.get_logger().debug(f"Received unrecognized packet: {data.hex(' ')}")
            return None
        except Exception as e:
            self.get_logger().error(f"Error parsing sensor response: {e}")
            return None

    def query_sensor(self, sensor_cmd):
        """Send a query command to a specific sensor and process the response."""
        try:
            with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
                s.settimeout(2.0)  # Set a timeout for socket operations
                s.connect((self.sensor_ip, self.SENSOR_PORT))
                s.sendall(sensor_cmd)
                data = s.recv(1024)
                return data
        except Exception as e:
            self.get_logger().error(f"Error connecting to sensor at {self.sensor_ip}:{self.SENSOR_PORT}: {e}")
            return None

    def query_all_sensors(self):
        """Query all sensors in sequence and update the sensor values."""
        sensors_updated = False
        
        try:
            # Query individual sensors for better reliability
            for cmd, idx in [(self.SENSOR_A_CMD, 0), 
                             (self.SENSOR_B_CMD, 1), 
                             (self.SENSOR_C_CMD, 2), 
                             (self.SENSOR_Z_CMD, 3)]:
                response = self.query_sensor(cmd)
                if response:
                    result = self.parse_sensor_response(response)
                    if result:
                        sensor_idx, value = result
                        if sensor_idx == idx:  # Verify we got the right sensor
                            self.sensor_values[idx] = value
                            sensors_updated = True
                        else:
                            self.get_logger().warning(f"Sensor index mismatch: expected {idx}, got {sensor_idx}")
                
                # Small delay between queries to avoid overwhelming the device
                time.sleep(0.05)
            
            return sensors_updated
            
        except Exception as e:
            self.get_logger().error(f"Error querying sensors: {e}")
            return False

    def timer_callback(self):
        """Timer callback to periodically query sensors and publish data."""
        try:
            # Try to query all sensors
            if self.query_all_sensors():
                # Apply calibration to sensor values
                calibrated_values = [
                    value * self.calibration_factor + self.calibration_offset
                    for value in self.sensor_values
                ]
                
                # Publish sensor values using the appropriate message type
                if USE_CUSTOM_MSG:
                    msg = MagneticSensorData()
                    msg.sensor_a = calibrated_values[0]
                    msg.sensor_b = calibrated_values[1]
                    msg.sensor_c = calibrated_values[2]
                    msg.sensor_z = calibrated_values[3]
                else:
                    msg = Float32MultiArray()
                    msg.data = calibrated_values
                
                self.publisher_.publish(msg)
                self.get_logger().info(f'Published sensor data: A={calibrated_values[0]:.2f}, ' +
                                      f'B={calibrated_values[1]:.2f}, ' +
                                      f'C={calibrated_values[2]:.2f}, ' +
                                      f'Z={calibrated_values[3]:.2f}')
            else:
                self.get_logger().warning('Failed to update sensor values')
                
        except Exception as e:
            self.get_logger().error(f'Error in timer callback: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = MagneticSensorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()