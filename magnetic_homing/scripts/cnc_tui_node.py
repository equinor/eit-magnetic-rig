#!/usr/bin/env python3
import curses
import json
import sys
import signal
import time

try:
    import rclpy
    from rclpy.node import Node
    from std_msgs.msg import String, Float32MultiArray
    from std_srvs.srv import Trigger, SetString
    HAS_ROS2 = True
except ImportError:
    HAS_ROS2 = False
    print("""
ERROR: ROS2 Python packages not found!
This program requires ROS2 and its Python packages to be installed.

Please ensure that:
1. ROS2 is installed on your system
2. You have sourced the ROS2 setup file:
   source /opt/ros/<distro>/setup.bash
   
For installation instructions, visit:
https://docs.ros.org/en/jazzy/Installation.html
""")
    sys.exit(1)

class CNCTuiNode(Node):
    def __init__(self, screen):
        super().__init__('cnc_tui_node')
        self.screen = screen
        self.setup_curses()
        self.setup_ros()
        self.setup_windows()
        
        # Data storage
        self.cnc_status = {}
        self.cnc_position = [0.0, 0.0, 0.0]
        self.sensor_data = [0.0, 0.0, 0.0, 0.0]
        self.last_response = ""
        self.input_buffer = ""
        self.input_mode = False
        
    def setup_curses(self):
        curses.start_color()
        curses.use_default_colors()
        curses.init_pair(1, curses.COLOR_RED, -1)
        curses.init_pair(2, curses.COLOR_GREEN, -1)
        curses.init_pair(3, curses.COLOR_YELLOW, -1)
        self.screen.nodelay(True)
        
    def setup_windows(self):
        height, width = self.screen.getmaxyx()
        # Status window (top left)
        self.status_win = curses.newwin(8, width//2, 0, 0)
        # Sensor window (top right)
        self.sensor_win = curses.newwin(8, width//2, 0, width//2)
        # Response window (middle)
        self.response_win = curses.newwin(6, width, 8, 0)
        # Menu window (bottom)
        self.menu_win = curses.newwin(10, width, 14, 0)
        
    def setup_ros(self):
        # Create subscribers
        self.create_subscription(String, '/cnc/status', 
                               self.status_callback, 10)
        self.create_subscription(Float32MultiArray, '/cnc/position',
                               self.position_callback, 10)
        self.create_subscription(Float32MultiArray, '/magnetic_sensor_data',
                               self.sensor_callback, 10)
        self.create_subscription(String, '/cnc/response',
                               self.response_callback, 10)
        
        # Create service clients
        self.service_clients = {
            'home': self.create_client(Trigger, '/cnc/home'),
            'reset': self.create_client(Trigger, '/cnc/reset'),
            'set_absolute': self.create_client(Trigger, '/cnc/set_absolute_mode'),
            'set_relative': self.create_client(Trigger, '/cnc/set_relative_mode'),
            'move_to_target': self.create_client(Trigger, '/cnc/move_to_target_origo'),
            'move_over_target': self.create_client(Trigger, '/cnc/move_over_target_origo'),
            'dock_target': self.create_client(Trigger, '/cnc/dock_at_target_origo'),
            'random_move': self.create_client(Trigger, '/cnc/move_to_random_safe_position'),
            'emergency_stop': self.create_client(Trigger, '/cnc/emergency_stop'),
            'unlock_alarm': self.create_client(Trigger, '/cnc/unlock_alarm'),
            'send_gcode': self.create_client(SetString, '/cnc/send_gcode'),
            'set_feed': self.create_client(SetString, '/cnc/set_feed_rate'),
            'jog': self.create_client(SetString, '/cnc/jog_increment'),
        }

    def update_display(self):
        try:
            self.update_status_window()
            self.update_sensor_window()
            self.update_response_window()
            self.update_menu_window()
            self.screen.refresh()
        except curses.error:
            pass

    def update_status_window(self):
        self.status_win.clear()
        self.status_win.box()
        self.status_win.addstr(0, 2, "CNC Status", curses.A_BOLD)
        
        if self.cnc_status:
            y = 1
            state = self.cnc_status.get('state', 'Unknown')
            state_color = curses.color_pair(2) if state == 'Idle' else curses.color_pair(3)
            self.status_win.addstr(y, 2, f"State: {state}", state_color)
            
            y += 1
            pos = self.cnc_status.get('wpos', {'x': 0, 'y': 0, 'z': 0})
            self.status_win.addstr(y, 2, f"Position: X:{pos['x']:.3f} Y:{pos['y']:.3f} Z:{pos['z']:.3f}")
            
            y += 1
            self.status_win.addstr(y, 2, f"Mode: {self.cnc_status.get('movement_mode', 'unknown')}")
            
            y += 1
            self.status_win.addstr(y, 2, f"Feed Rate: {self.cnc_status.get('feed_rate', 0)}")
        
        self.status_win.refresh()

    def sensor_data_callback(self, msg):
        """Handle magnetic sensor data updates"""
        self.sensor_data = msg.data
        self.update_sensor_window()

    def update_sensor_window(self):
        """Update sensor data display"""
        self.sensor_win.clear()
        self.sensor_win.box()
        self.sensor_win.addstr(0, 2, "Magnetic Sensors", curses.A_BOLD)
        
        if len(self.sensor_data) == 4:
            self.sensor_win.addstr(1, 2, f"Sensor A: {self.sensor_data[0]:.2f}")
            self.sensor_win.addstr(2, 2, f"Sensor B: {self.sensor_data[1]:.2f}")
            self.sensor_win.addstr(3, 2, f"Sensor C: {self.sensor_data[2]:.2f}")
            self.sensor_win.addstr(4, 2, f"Sensor Z: {self.sensor_data[3]:.2f}")
        
        self.sensor_win.refresh()

    def update_response_window(self):
        self.response_win.clear()
        self.response_win.box()
        self.response_win.addstr(0, 2, "Last Response", curses.A_BOLD)
        self.response_win.addstr(1, 2, self.last_response[:100])  # Truncate if too long
        self.response_win.refresh()

    def update_menu_window(self):
        self.menu_win.clear()
        self.menu_win.box()
        self.menu_win.addstr(0, 2, "Menu", curses.A_BOLD)
        
        menu_items = [
            "1. Home CNC",
            "2. Movement Controls >",
            "3. Send G-Code",
            "4. Settings >",
            "5. Recovery >",
            "9. EMERGENCY STOP",
            "0. QUIT"
        ]
        
        for i, item in enumerate(menu_items):
            color = curses.color_pair(1) if "EMERGENCY" in item or "QUIT" in item else curses.A_NORMAL
            self.menu_win.addstr(i + 1, 2, item, color)
        
        if self.input_mode:
            self.menu_win.addstr(len(menu_items) + 1, 2, f"Input: {self.input_buffer}")
        
        self.menu_win.refresh()

    def status_callback(self, msg):
        """Handle status updates from CNC controller"""
        try:
            self.cnc_status = json.loads(msg.data)
            self.update_display()
        except json.JSONDecodeError:
            self.last_response = "Error parsing status JSON"

    def position_callback(self, msg):
        """Handle position updates from CNC controller"""
        self.cnc_position = msg.data
        self.update_display()

    def sensor_callback(self, msg):
        """Handle magnetic sensor data updates"""
        self.sensor_data = msg.data
        self.update_display()

    def response_callback(self, msg):
        """Handle response messages from CNC controller"""
        self.last_response = msg.data
        self.update_display()

    def handle_input(self, key):
        """Process user input"""
        if self.input_mode:
            if key == 27:  # ESC
                self.input_mode = False
                self.input_buffer = ""
            elif key == 10:  # Enter
                self.process_input()
                self.input_mode = False
                self.input_buffer = ""
            elif key == 127:  # Backspace
                self.input_buffer = self.input_buffer[:-1]
            elif key >= 32:  # Printable characters
                self.input_buffer += chr(key)
        else:
            self.process_menu_selection(key)

    def process_menu_selection(self, key):
        """Handle menu selections"""
        ch = chr(key)
        if ch in '12345':
            if ch == '1':
                self.call_service('home')
            elif ch == '2':
                # Movement Controls submenu handling would go here
                pass
            elif ch == '3':
                self.input_mode = True
                self.input_buffer = ""
                self.last_response = "Enter G-code command:"
            elif ch == '4':
                # Settings submenu handling would go here
                pass
            elif ch == '5':
                # Recovery submenu handling would go here
                pass
        elif ch == '9':
            self.call_service('emergency_stop')
        elif ch == '0':
            self.call_service('emergency_stop')  # Ensure machine stops before quitting
            raise KeyboardInterrupt

    def process_input(self):
        """Process buffered input"""
        if not self.input_buffer:
            return

        if self.last_response == "Enter G-code command:":
            self.call_service('send_gcode', self.input_buffer)
        elif self.last_response == "Enter feed rate:":
            self.call_service('set_feed', self.input_buffer)

    def call_service(self, service_name, data=None):
        """Call a ROS2 service"""
        client = self.service_clients.get(service_name)
        if not client:
            self.last_response = f"Service {service_name} not found"
            return

        if not client.service_is_ready():
            self.last_response = f"Service {service_name} is not available"
            return

        try:
            if data is not None:
                request = SetString.Request()
                request.data = data
            else:
                request = Trigger.Request()

            future = client.call_async(request)
            # We're not waiting for the response - it will come via the response topic
            self.last_response = f"Called service {service_name}"
        except Exception as e:
            self.last_response = f"Error calling service {service_name}: {str(e)}"

def main(args=None):
    """Main function"""
    rclpy.init(args=args)
    
    def cleanup(screen):
        """Restore terminal settings"""
        curses.nocbreak()
        screen.keypad(False)
        curses.echo()
        curses.endwin()

    try:
        # Initialize curses
        screen = curses.initscr()
        node = CNCTuiNode(screen)

        # Handle Ctrl+C gracefully
        def signal_handler(sig, frame):
            cleanup(screen)
            node.destroy_node()
            rclpy.shutdown()
            sys.exit(0)
        signal.signal(signal.SIGINT, signal_handler)

        # Main loop
        while rclpy.ok():
            # Process ROS callbacks
            rclpy.spin_once(node, timeout_sec=0.1)
            
            # Get user input
            try:
                key = screen.getch()
                if key != -1:  # -1 means no input
                    node.handle_input(key)
            except curses.error:
                pass

            # Update display
            node.update_display()

    except KeyboardInterrupt:
        pass
    finally:
        cleanup(screen)
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
