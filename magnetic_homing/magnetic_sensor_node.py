import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from std_srvs.srv import SetBool
import socket
import struct
import time
import os
import sys
from enum import Enum, auto

# Import the custom message type if available
try:
    from magnetic_homing.msg import MagneticSensorData
    USE_CUSTOM_MSG = True
except ImportError:
    USE_CUSTOM_MSG = False
    print("Warning: MagneticSensorData message type not found, using Float32MultiArray instead")

class ConnectionState(Enum):
    """Enumeration for tracking the sensor connection state"""
    DISCONNECTED = auto()
    CONNECTING = auto()
    CONNECTED = auto()
    INITIALIZED = auto()

class MagneticSensorNode(Node):
    # Protocol Commands from the Node.js implementation
    HOMING_ON_CMD = bytes([0x01, 0x01, 0x02, 0x01, 0x05, 0x10, 0x03, 0x10, 0x01, 0x01, 0x0C, 0xBD])
    HOMING_OFF_CMD = bytes([0x01, 0x01, 0x02, 0x01, 0x05, 0x10, 0x03, 0x10, 0x01, 0x00, 0xCC, 0x7C])
    ALL_SENSORS_CMD = bytes([0x01, 0x01, 0x01, 0x01, 0x10, 0x10, 0x03, 0x10, 0x03, 0x10, 0x03, 0x10, 
                           0x04, 0x10, 0x03, 0x10, 0x05, 0x10, 0x03, 0x10, 0x06, 0x70, 0x75])
    
    # Protocol constants
    SENSOR_PORT = 2468
    RESPONSE_TIMEOUT = 2.0  # seconds
    RECONNECT_DELAY = 5.0  # seconds
    
    def __init__(self):
        super().__init__('magnetic_sensor_node')
        
        # Socket and connection state management
        self._socket = None
        self._connection_state = ConnectionState.DISCONNECTED
        self._last_reconnect_attempt = 0.0
        
        # Parameters (with explicit defaults)
        self.declare_parameter('sensor_ip', '192.168.1.254')  # Fixed IP as per requirements
        self.declare_parameter('publish_rate', 10.0)
        self.declare_parameter('calibration_factor', 1.0)
        self.declare_parameter('calibration_offset', 0.0)
        self.declare_parameter('connection_retry_limit', 5)
        
        # Get parameters
        self.sensor_ip = self.get_parameter('sensor_ip').get_parameter_value().string_value
        publish_rate = self.get_parameter('publish_rate').get_parameter_value().double_value
        self.calibration_factor = self.get_parameter('calibration_factor').get_parameter_value().double_value
        self.calibration_offset = self.get_parameter('calibration_offset').get_parameter_value().double_value
        self._connection_retry_limit = self.get_parameter('connection_retry_limit').get_parameter_value().integer_value
        
        # Create the appropriate publisher based on available message types
        if USE_CUSTOM_MSG:
            self.publisher_ = self.create_publisher(MagneticSensorData, 'magnetic_sensor_data', 10)
            self.get_logger().info('Using custom MagneticSensorData message type')
        else:
            self.publisher_ = self.create_publisher(Float32MultiArray, 'magnetic_sensor_data', 10)
            self.get_logger().info('Using Float32MultiArray message type')
            
        self.srv = self.create_service(SetBool, 'set_sensor_ip', self.set_sensor_ip_callback)
        
        # Create timer for periodic sensor readings
        self.timer = self.create_timer(1.0 / publish_rate, self.timer_callback)
        
        # Attempt initial connection
        self._connect_and_initialize()

    def _connect_and_initialize(self):
        """Establish connection and initialize the sensor array"""
        if self._connection_state != ConnectionState.DISCONNECTED:
            return

        try:
            self._connection_state = ConnectionState.CONNECTING
            self.get_logger().info(f'Connecting to sensor at {self.sensor_ip}:{self.SENSOR_PORT}')
            
            # Create new socket
            self._socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self._socket.settimeout(self.RESPONSE_TIMEOUT)
            self._socket.connect((self.sensor_ip, self.SENSOR_PORT))
            
            # Send initialization (ON) command
            self._socket.sendall(self.HOMING_ON_CMD)
            response = self._socket.recv(1024)
            
            if response:  # Check if we got any response
                self._connection_state = ConnectionState.INITIALIZED
                self.get_logger().info('Sensor initialized successfully')
            else:
                raise Exception("No response to initialization command")
                
        except Exception as e:
            self.get_logger().error(f'Failed to connect/initialize sensor: {str(e)}')
            self._handle_connection_failure()

    def _handle_connection_failure(self):
        """Handle connection failures and cleanup"""
        if self._socket:
            try:
                self._socket.close()
            except:
                pass
            self._socket = None
        
        self._connection_state = ConnectionState.DISCONNECTED
        self._last_reconnect_attempt = time.time()

    def _ensure_connection(self):
        """Ensure we have an active connection, attempt reconnection if needed"""
        if self._connection_state == ConnectionState.INITIALIZED:
            return True
            
        # Don't retry too frequently
        if (time.time() - self._last_reconnect_attempt) < self.RECONNECT_DELAY:
            return False
            
        self._connect_and_initialize()
        return self._connection_state == ConnectionState.INITIALIZED

    def set_sensor_ip_callback(self, request, response):
        """Callback for setting sensor IP address"""
        self.sensor_ip = request.data  # Changed from request.data (bool) to request.data (string)
        self.get_logger().info(f'Sensor IP updated to: {self.sensor_ip}')
        response.success = True
        response.message = 'Sensor IP updated successfully.'
        return response
    
    def parse_all_sensors_response(self, data):
        """Parse the combined sensor response packet based on the Node.js implementation"""
        try:
            if len(data) != 31:  # Expected length for ALL_SENSORS response
                self.get_logger().warning(f'Unexpected response length: {len(data)}')
                return None
                
            # Extract sensor values based on known offsets (from Node.js implementation)
            sensor_a = struct.unpack('>H', data[9:11])[0]   # bytes 9-10
            sensor_b = struct.unpack('>H', data[15:17])[0]  # bytes 15-16
            sensor_c = struct.unpack('>H', data[21:23])[0]  # bytes 21-22
            sensor_z = struct.unpack('>H', data[27:29])[0]  # bytes 27-28
            
            return [float(x) for x in (sensor_a, sensor_b, sensor_c, sensor_z)]
            
        except Exception as e:
            self.get_logger().error(f'Error parsing sensor response: {e}')
            return None

    def query_all_sensors(self):
        """Query all sensors using the ALL_SENSORS command"""
        if not self._ensure_connection():
            return None
            
        try:
            self._socket.sendall(self.ALL_SENSORS_CMD)
            response = self._socket.recv(1024)
            return self.parse_all_sensors_response(response)
            
        except socket.timeout:
            self.get_logger().warning('Timeout while querying sensors')
            return None
        except Exception as e:
            self.get_logger().error(f'Error querying sensors: {e}')
            self._handle_connection_failure()
            return None

    def timer_callback(self):
        """Timer callback to periodically query sensors and publish data"""
        try:
            values = self.query_all_sensors()
            if values:
                # Apply calibration
                calibrated_values = [
                    value * self.calibration_factor + self.calibration_offset
                    for value in values
                ]
                
                # Publish using appropriate message type
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
                self.get_logger().debug(
                    f'Published sensor data: A={calibrated_values[0]:.2f}, '
                    f'B={calibrated_values[1]:.2f}, C={calibrated_values[2]:.2f}, '
                    f'Z={calibrated_values[3]:.2f}'
                )
                
        except Exception as e:
            self.get_logger().error(f'Error in timer callback: {e}')

    def destroy_node(self):
        """Clean up resources when the node is shut down"""
        if self._socket:
            try:
                # Send HOMING_OFF command before disconnecting
                self._socket.sendall(self.HOMING_OFF_CMD)
                self._socket.close()
            except:
                pass
        super().destroy_node()

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