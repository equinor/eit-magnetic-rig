import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, String, Float32
from std_srvs.srv import Trigger, SetBool
import serial
import time
import threading
import re
import json
import random

class CNCControllerNode(Node):
    """
    ROS2 Node for interfacing with a GRBL-based CNC controller.
    Provides functionality for sending G-code commands and receiving status updates.
    """
    
    # GRBL status codes
    GRBL_STATUS = {
        0: "OK",
        1: "Expected command letter",
        2: "Bad number format",
        3: "Invalid statement",
        4: "Negative value",
        5: "Setting disabled",
        6: "Setting value exceeds limits",
        7: "Command value exceeds limits",
        8: "Command unavailable",
        9: "G-code locked during alarm or jog"
    }
    
    def __init__(self):
        super().__init__('cnc_controller_node')
        
        # Declare parameters
        self.declare_parameter('serial_port', '/dev/ttyUSB0')
        self.declare_parameter('baud_rate', 115200)
        self.declare_parameter('status_interval', 0.2)  # Status poll interval in seconds
        
        # Add target and safety parameters
        self.declare_parameter('target_origo_x', 524.0)
        self.declare_parameter('target_origo_y', 318.0)
        self.declare_parameter('target_origo_z', -235.0)
        self.declare_parameter('safe_z_for_xy_traverse', -185.0)
        self.declare_parameter('docking_xy_tolerance', 1.0)
        self.declare_parameter('random_safe_min_x', 0.0)
        self.declare_parameter('random_safe_max_x', 1000.0)
        self.declare_parameter('random_safe_min_y', 0.0)
        self.declare_parameter('random_safe_max_y', 600.0)
        self.declare_parameter('random_safe_min_z', -185.0)
        self.declare_parameter('random_safe_max_z', -50.0)

        # Get parameter values
        self.serial_port = self.get_parameter('serial_port').get_parameter_value().string_value
        self.baud_rate = self.get_parameter('baud_rate').get_parameter_value().integer_value
        self.status_interval = self.get_parameter('status_interval').get_parameter_value().double_value
        
        # Initialize serial connection
        try:
            self.serial_connection = serial.Serial(
                port=self.serial_port,
                baudrate=self.baud_rate,
                timeout=1.0,
                bytesize=serial.EIGHTBITS,
                stopbits=serial.STOPBITS_ONE
            )
            self.serial_connected = True
        except serial.SerialException as e:
            self.get_logger().error(f'Failed to connect to serial port {self.serial_port}: {e}')
            self.serial_connected = False
            self.serial_connection = None
        
        # Initialize machine state
        self.machine_position = {'x': 0.0, 'y': 0.0, 'z': 0.0}
        self.work_position = {'x': 0.0, 'y': 0.0, 'z': 0.0}
        self.machine_state = "Unknown"
        self.movement_mode = "absolute"  # Default to absolute movement (G90)
        self.feed_rate = 100.0  # Default feed rate (mm/min)
        self.last_error = None
        self.last_alarm = None
        self.response_buffer = []
        self.command_lock = threading.Lock()
        
        # Create publishers
        self.status_publisher = self.create_publisher(String, 'cnc/status', 10)
        self.position_publisher = self.create_publisher(Float32MultiArray, 'cnc/position', 10)
        self.response_publisher = self.create_publisher(String, 'cnc/response', 10)
        
        # Create subscribers
        self.subscription = self.create_subscription(
            Float32MultiArray,
            'magnetic_sensor_data',
            self.sensor_data_callback,
            10
        )
        
        # Create services
        self.gcode_service = self.create_service(
            SetBool, 'cnc/send_gcode', self.send_gcode_callback
        )
        self.home_service = self.create_service(
            Trigger, 'cnc/home', self.home_callback
        )
        self.reset_service = self.create_service(
            Trigger, 'cnc/reset', self.reset_callback
        )
        self.set_absolute_mode_service = self.create_service(
            Trigger, 'cnc/set_absolute_mode', self.set_absolute_mode_callback
        )
        self.set_relative_mode_service = self.create_service(
            Trigger, 'cnc/set_relative_mode', self.set_relative_mode_callback
        )
        self.set_feed_rate_service = self.create_service(
            SetBool, 'cnc/set_feed_rate', self.set_feed_rate_callback
        )
        
        # Add new movement services
        self.move_to_target_service = self.create_service(
            Trigger, 'cnc/move_to_target_origo', self.move_to_target_origo_callback
        )
        self.move_over_target_service = self.create_service(
            Trigger, 'cnc/move_over_target_origo', self.move_over_target_origo_callback
        )
        self.dock_target_service = self.create_service(
            Trigger, 'cnc/dock_at_target_origo', self.dock_at_target_origo_callback
        )
        self.random_move_service = self.create_service(
            Trigger, 'cnc/move_to_random_safe_position', self.move_to_random_safe_callback
        )
        self.jog_service = self.create_service(
            SetBool, 'cnc/jog_increment', self.jog_increment_callback
        )
        self.estop_service = self.create_service(
            Trigger, 'cnc/emergency_stop', self.emergency_stop_callback
        )
        self.unlock_service = self.create_service(
            Trigger, 'cnc/unlock_alarm', self.unlock_alarm_callback
        )
        
        # Serial reading and status polling threads
        self.stop_thread = False
        if self.serial_connected:
            # Thread for reading from serial port
            self.serial_thread = threading.Thread(target=self.serial_read_loop)
            self.serial_thread.daemon = True
            self.serial_thread.start()
            
            # Thread for polling status
            self.status_thread = threading.Thread(target=self.status_polling)
            self.status_thread.daemon = True
            self.status_thread.start()
            
            # Initialize GRBL by sending a soft reset
            self.reset_grbl()
        
        self.get_logger().info(f'CNC Controller Node has been started on port {self.serial_port}')
    
    def send_command(self, command, wait_for_response=True):
        """Send a command to GRBL and optionally wait for a response."""
        if not self.serial_connected:
            self.get_logger().warning("Not connected to serial port")
            return False, "Not connected to serial port"
        
        try:
            with self.command_lock:
                # Clear response buffer for new command
                self.response_buffer = []
                
                # Add newline if not present
                if not command.endswith('\n'):
                    command += '\n'
                
                # Send the command
                self.get_logger().debug(f"Sending: {command.strip()}")
                self.serial_connection.write(command.encode())
                
                if wait_for_response:
                    # Wait for response (ok or error)
                    timeout = time.time() + 5.0  # 5 second timeout
                    while time.time() < timeout:
                        if any(resp.startswith('ok') or resp.startswith('error:') for resp in self.response_buffer):
                            # Get all responses
                            responses = self.response_buffer.copy()
                            self.response_buffer = []
                            
                            # Check for errors
                            for resp in responses:
                                if resp.startswith('error:'):
                                    return False, resp
                            
                            return True, '\n'.join(responses)
                        time.sleep(0.01)
                    
                    return False, "Command timeout"
                
                return True, "Command sent (async)"
                
        except Exception as e:
            self.get_logger().error(f"Error sending command: {e}")
            return False, str(e)
    
    def reset_grbl(self):
        """Send a soft reset command to GRBL."""
        if self.serial_connected:
            try:
                # Send GRBL soft reset command (Ctrl+X)
                self.serial_connection.write(b'\x18')
                time.sleep(1)  # Give GRBL time to reset and initialize
                
                # Flush any startup messages
                self.serial_connection.reset_input_buffer()
                
                # Send initial configuration
                self.send_command("$X")  # Unlock GRBL
                self.send_command("G21")  # Set to millimeters
                
                # Set movement mode (default to absolute)
                success, _ = self.send_command("G90")
                if success:
                    self.movement_mode = "absolute"
                
                # Set default feed rate
                success, _ = self.send_command(f"F{self.feed_rate}")
                
                self.get_logger().info(f"GRBL has been reset and configured (Mode: {self.movement_mode}, Feed rate: {self.feed_rate})")
                
                # Publish initial status
                self.request_status()
                
                return True
            except Exception as e:
                self.get_logger().error(f"Error resetting GRBL: {e}")
        return False
    
    def send_jog_command(self, x=None, y=None, z=None, feed_rate=None, relative=False):
        """Send a jog command matching the Node-RED implementation format"""
        try:
            cmd = "$J="
            cmd += "G91" if relative else "G90"
            cmd += "G21"  # Metric mode (always used in Node.js code)
            
            if x is not None:
                cmd += f"X{x}"
            if y is not None:
                cmd += f"Y{y}"
            if z is not None:
                cmd += f"Z{z}"
            if feed_rate is not None:
                cmd += f"F{feed_rate}"
            else:
                cmd += f"F{self.feed_rate}"
                
            return self.send_command(cmd)
            
        except Exception as e:
            self.get_logger().error(f"Error sending jog command: {e}")
            return False, str(e)

    def parse_status(self, status_string):
        """Parse GRBL status response based on Node-RED implementation format."""
        try:
            # Node-RED format: <Idle|WPos:100.000,0.000,0.000|FS:0,0>
            state_match = re.search(r'<([^|>]+)', status_string)
            if state_match:
                self.machine_state = state_match.group(1)
            
            # Parse work position (WPos) as used in Node-RED
            wpos_match = re.search(r'WPos:([^|>]+)', status_string)
            if wpos_match:
                pos = wpos_match.group(1).split(',')
                if len(pos) >= 3:
                    self.work_position = {
                        'x': float(pos[0]),
                        'y': float(pos[1]),
                        'z': float(pos[2])
                    }
                    # Node-RED mainly used work position, so sync machine position
                    self.machine_position = self.work_position.copy()
            
            # Publish the current position
            pos_msg = Float32MultiArray()
            pos_msg.data = [
                self.work_position['x'],
                self.work_position['y'],
                self.work_position['z']
            ]
            self.position_publisher.publish(pos_msg)
            
            # Publish the current status
            status_msg = String()
            status_msg.data = json.dumps({
                'state': self.machine_state,
                'mpos': self.machine_position,
                'wpos': self.work_position,
                'movement_mode': self.movement_mode,
                'feed_rate': self.feed_rate
            })
            self.status_publisher.publish(status_msg)
            
        except Exception as e:
            self.get_logger().error(f"Error parsing status: {e}")
    
    def request_status(self):
        """Send status request to GRBL."""
        if self.serial_connected:
            try:
                self.serial_connection.write(b'?')
                return True
            except Exception as e:
                self.get_logger().error(f"Error requesting status: {e}")
        return False
    
    def serial_read_loop(self):
        """Thread for continuously reading from the serial port."""
        if not self.serial_connected:
            return
        
        while self.serial_connected and not self.stop_thread:
            try:
                if self.serial_connection.in_waiting > 0:
                    line = self.serial_connection.readline().decode('utf-8').strip()
                    if line:
                        self.get_logger().debug(f"Received: {line}")
                        
                        # Process status responses
                        if line.startswith('<') and line.endswith('>'):
                            self.parse_status(line)
                        else:
                            # Store other responses in buffer
                            self.response_buffer.append(line)
                            
                            # Publish response for subscribers
                            response_msg = String()
                            response_msg.data = line
                            self.response_publisher.publish(response_msg)
                            
            except Exception as e:
                self.get_logger().error(f"Error reading from serial: {e}")
                time.sleep(0.1)
    
    def status_polling(self):
        """Thread for periodically requesting status from GRBL."""
        while self.serial_connected and not self.stop_thread:
            self.request_status()
            time.sleep(self.status_interval)
    
    def send_gcode_callback(self, request, response):
        """ROS2 service callback for sending G-code commands."""
        try:
            success, result = self.send_command(request.data)
            response.success = success
            response.message = result
        except Exception as e:
            self.get_logger().error(f"Error in send_gcode_callback: {e}")
            response.success = False
            response.message = str(e)
        return response
    
    def home_callback(self, request, response):
        """ROS2 service callback for homing the machine."""
        if not self.serial_connected:
            response.success = False
            response.message = "Not connected to serial port"
            return response
        
        try:
            # Send homing command
            success, result = self.send_command("$H")
            
            response.success = success
            response.message = result
        except Exception as e:
            self.get_logger().error(f"Error in home_callback: {e}")
            response.success = False
            response.message = str(e)
        
        return response
    
    def reset_callback(self, request, response):
        """ROS2 service callback for resetting the GRBL controller."""
        try:
            success = self.reset_grbl()
            response.success = success
            response.message = "GRBL reset successful" if success else "GRBL reset failed"
        except Exception as e:
            self.get_logger().error(f"Error in reset_callback: {e}")
            response.success = False
            response.message = str(e)
        
        return response
        
    def set_absolute_mode_callback(self, request, response):
        """ROS2 service callback for setting absolute movement mode (G90)."""
        try:
            success, result = self.send_command("G90")
            self.get_logger().info("Set to absolute movement mode (G90)")
            response.success = success
            response.message = "Set to absolute movement mode" if success else f"Failed: {result}"
        except Exception as e:
            self.get_logger().error(f"Error setting absolute movement mode: {e}")
            response.success = False
            response.message = str(e)
        
        return response
        
    def set_relative_mode_callback(self, request, response):
        """ROS2 service callback for setting relative movement mode (G91)."""
        try:
            success, result = self.send_command("G91")
            self.get_logger().info("Set to relative movement mode (G91)")
            response.success = success
            response.message = "Set to relative movement mode" if success else f"Failed: {result}"
        except Exception as e:
            self.get_logger().error(f"Error setting relative movement mode: {e}")
            response.success = False
            response.message = str(e)
        
        return response
        
    def set_feed_rate_callback(self, request, response):
        """ROS2 service callback for setting feed rate."""
        try:
            feed_rate = float(request.data)
            if feed_rate <= 0:
                raise ValueError("Feed rate must be positive")
            success, result = self.send_command(f"F{feed_rate}")
            self.feed_rate = feed_rate if success else self.feed_rate
            response.success = success
            response.message = f"Set feed rate to {feed_rate}" if success else result
        except ValueError as ve:
            response.success = False
            response.message = f"Invalid feed rate: {ve}"
        return response

    def sensor_data_callback(self, msg):
        """Callback for processing magnetic sensor data."""
        # This callback is a stub that could be customized based on your specific needs
        # Currently, it just logs the received sensor data
        self.get_logger().debug(f"Received sensor data: {msg.data}")
        
        # Example of what you might do with this data
        # Instead of automatically sending commands based on sensor data,
        # this would typically be handled by a higher-level control node
        # We're just passing the information through
    
    def destroy_node(self):
        """Cleanup when node is shut down."""
        self.get_logger().info("Shutting down CNC Controller Node")
        self.stop_thread = True
        
        # Close serial connection
        if self.serial_connected and self.serial_connection:
            try:
                self.serial_connection.close()
                self.get_logger().info("Serial connection closed")
            except Exception as e:
                self.get_logger().error(f"Error closing serial connection: {e}")
        
        super().destroy_node()

    # Helper methods for movement operations
    def _ensure_absolute_mode(self):
        """Ensure GRBL is in absolute mode (G90)"""
        success, resp = self.send_command("G90")
        return success

    def _get_current_position(self):
        """Get current machine position"""
        if not hasattr(self, 'machine_position'):
            return None
        return self.machine_position

    def _lift_z_if_needed(self, safe_z):
        """Lift Z to safe height if currently below it"""
        current_pos = self._get_current_position()
        if not current_pos:
            return False, "Cannot determine current position"
            
        if current_pos['z'] < safe_z:
            success, resp = self.send_command(f"G0 Z{safe_z}")
            return success, resp
        return True, "Z already at safe height"

    # New service callbacks
    def move_to_target_origo_callback(self, request, response):
        """Move to target origo position safely"""
        try:
            target_x = self.get_parameter('target_origo_x').value
            target_y = self.get_parameter('target_origo_y').value
            target_z = self.get_parameter('target_origo_z').value
            safe_z = self.get_parameter('safe_z_for_xy_traverse').value
            
            # First ensure we're in absolute mode
            success, _ = self.send_command("G90")
            if not success:
                response.success = False
                response.message = "Failed to set absolute mode"
                return response

            # Get current position
            current_pos = self._get_current_position()
            if not current_pos:
                response.success = False
                response.message = "Could not determine current position"
                return response

            # If below safe Z, move up first
            if current_pos['z'] < safe_z:
                success, msg = self.send_jog_command(z=safe_z, relative=False)
                if not success:
                    response.success = False
                    response.message = f"Failed to reach safe Z height: {msg}"
                    return response

            # Move to XY target
            success, msg = self.send_jog_command(x=target_x, y=target_y, relative=False)
            if not success:
                response.success = False
                response.message = f"Failed to reach target XY: {msg}"
                return response

            # Finally move to target Z
            success, msg = self.send_jog_command(z=target_z, relative=False)
            response.success = success
            response.message = "Successfully reached target origo" if success else f"Failed to reach target Z: {msg}"
            return response

        except Exception as e:
            response.success = False
            response.message = f"Error during movement: {str(e)}"
            return response

    def move_over_target_origo_callback(self, request, response):
        """Move to position above target origo at safe Z height"""
        try:
            target_x = self.get_parameter('target_origo_x').value
            target_y = self.get_parameter('target_origo_y').value
            safe_z = self.get_parameter('safe_z_for_xy_traverse').value
            
            # Get current position and lift Z if needed
            current_pos = self._get_current_position()
            if not current_pos:
                return self._failed_response("Could not determine current position")
            
            if current_pos['z'] < safe_z:
                success, msg = self.send_jog_command(z=safe_z, relative=False)
                if not success:
                    return self._failed_response(f"Failed to reach safe Z height: {msg}")
            
            # Move to target XY using jog command
            success, msg = self.send_jog_command(x=target_x, y=target_y, relative=False)
            
            response.success = success
            response.message = "Successfully moved over target" if success else f"Failed to reach target: {msg}"
            return response
            
        except Exception as e:
            return self._failed_response(f"Error: {str(e)}")

    def dock_at_target_origo_callback(self, request, response):
        """Dock at the target origo position"""
        try:
            if not self._ensure_absolute_mode():
                return self._failed_response("Could not ensure absolute mode")

            # Get target coordinates from parameters
            target_x = self.get_parameter('target_origo_x').value
            target_y = self.get_parameter('target_origo_y').value
            target_z = self.get_parameter('target_origo_z').value
            tolerance = self.get_parameter('docking_xy_tolerance').value

            # Move to target position with tolerance
            success, msg = self.send_command(f"G0 X{target_x} Y{target_y} Z{target_z} F{self.feed_rate}")
            if not success:
                return self._failed_response(f"Failed to dock at target: {msg}")

            # Fine adjustment: move in small increments to find the exact origo
            increments = [(-tolerance, -tolerance, 0), (tolerance, -tolerance, 0),
                          (-tolerance, tolerance, 0), (tolerance, tolerance, 0)]
            for inc in increments:
                success, msg = self.send_command(f"G0 X{inc[0]} Y{inc[1]} Z{inc[2]} F{self.feed_rate}")
                if not success:
                    return self._failed_response(f"Failed in fine adjustment: {msg}")

            response.success = True
            response.message = "Successfully docked at target origo"
            return response

        except Exception as e:
            return self._failed_response(f"Error: {str(e)}")

    def move_to_random_safe_callback(self, request, response):
        """Move to random position within safe boundaries"""
        try:
            # Get boundaries from parameters
            min_x = self.get_parameter('random_safe_min_x').value
            max_x = self.get_parameter('random_safe_max_x').value
            min_y = self.get_parameter('random_safe_min_y').value
            max_y = self.get_parameter('random_safe_max_y').value
            min_z = self.get_parameter('random_safe_min_z').value
            max_z = self.get_parameter('random_safe_max_z').value
            safe_z = self.get_parameter('safe_z_for_xy_traverse').value

            # Generate random target position
            import random
            target_x = random.uniform(min_x, max_x)
            target_y = random.uniform(min_y, max_y)
            target_z = random.uniform(min_z, max_z)

            # Safety sequence: safe Z, then XY, then target Z
            current_pos = self._get_current_position()
            if not current_pos:
                return self._failed_response("Could not determine current position")

            if current_pos['z'] < safe_z:
                success, msg = self.send_jog_command(z=safe_z, relative=False)
                if not success:
                    return self._failed_response(f"Failed to reach safe Z: {msg}")

            # Move XY
            success, msg = self.send_jog_command(x=target_x, y=target_y, relative=False)
            if not success:
                return self._failed_response(f"Failed to reach target XY: {msg}")

            # Move Z
            success, msg = self.send_jog_command(z=target_z, relative=False)
            
            response.success = success
            response.message = f"Moved to random position ({target_x:.1f}, {target_y:.1f}, {target_z:.1f})" if success else f"Failed: {msg}"
            return response

        except Exception as e:
            return self._failed_response(f"Error: {str(e)}")

    def jog_increment_callback(self, request, response):
        """Handle jog movement requests with JSON parameters"""
        try:
            # Parse JSON request
            params = json.loads(request.data)
            
            # Extract optional parameters with defaults
            x = params.get('x')
            y = params.get('y')
            z = params.get('z')
            feed = params.get('feed', self.feed_rate)
            relative = params.get('relative', True)
            
            success, msg = self.send_jog_command(
                x=x, y=y, z=z, 
                feed_rate=feed,
                relative=relative
            )
            
            response.success = success
            response.message = msg
            return response
            
        except json.JSONDecodeError:
            response.success = False
            response.message = "Invalid JSON format in request"
            return response
        except Exception as e:
            response.success = False
            response.message = str(e)
            return response

    def emergency_stop_callback(self, request, response):
        """Immediate halt of all motion"""
        try:
            if self.serial_connected:
                # Send feed hold and reset based on Node.js implementation
                self.serial_connection.write(b"!")  # Feed hold
                time.sleep(0.1)
                self.serial_connection.write(b"\x18")  # Soft reset
                response.success = True
                response.message = "Emergency stop executed"
            else:
                response.success = False
                response.message = "Not connected to GRBL"
        except Exception as e:
            response.success = False
            response.message = f"Emergency stop failed: {str(e)}"
        return response

    def unlock_alarm_callback(self, request, response):
        """Clear alarm lock using $X command"""
        try:
            success, msg = self.send_command("$X")
            response.success = success
            response.message = "Alarm cleared" if success else msg
            return response
        except Exception as e:
            response.success = False
            response.message = f"Failed to clear alarm: {str(e)}"
            return response

    def _get_current_position(self):
        """Get current machine position"""
        if not hasattr(self, 'machine_position'):
            return None
        return self.machine_position

    def _failed_response(self, message):
        """Helper to create failed service responses"""
        response = Trigger.Response()
        response.success = False
        response.message = message
        return response


def main(args=None):
    rclpy.init(args=args)
    node = CNCControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
