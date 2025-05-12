#!/usr/bin/env python3
import os
os.environ['TF_SILENCE_DEPRECATION'] = '1'  # Silence IMKClient warning
os.environ['TK_SILENCE_DEPRECATION'] = '1'  # Silence deprecation warning

import tkinter as tk
from tkinter import ttk, scrolledtext, messagebox
import json
import threading
from queue import Queue
import sys
import argparse

# --- Start of Revised ROS Import Block ---

# Initialize defaults: HAS_ROS2 to False, and all ROS types to Mocks.
# These will be overwritten by real ROS types if imports succeed.
HAS_ROS2 = False

class _MockBase:  # Base class for mocks, useful for type checking if needed
    pass

class MockRclpy(_MockBase): # Mock for the rclpy module itself
    @staticmethod
    def init(args=None): pass # Added args=None to match rclpy.init
    @staticmethod
    def shutdown(): pass
    @staticmethod
    def ok(): return False
    @staticmethod
    def spin_once(node, timeout_sec=0): pass

class MockNode(_MockBase):
    def __init__(self, name, **kwargs): pass # Added **kwargs to match rclpy.node.Node
    def create_subscription(self, msg_type, topic, callback, qos_profile, **kwargs): pass # Added **kwargs
    def create_client(self, srv_type, srv_name, **kwargs): # Added **kwargs
        # Return a mock client that has a service_is_ready method and call_async
        class MockClient:
            def service_is_ready(self): return False
            def call_async(self, request):
                class MockFuture:
                    def add_done_callback(self, cb): pass
                return MockFuture()
        return MockClient()
    def destroy_node(self): pass

class MockStringMsg(_MockBase):
    def __init__(self, data=None): self.data = data

class MockFloat32MultiArrayMsg(_MockBase):
    def __init__(self, data=None): self.data = data

class MockServiceRequest(_MockBase): pass
class MockServiceResponse(_MockBase):
    def __init__(self, success=False, message=""):
        self.success = success
        self.message = message

class MockTrigger(_MockBase):
    Request = MockServiceRequest
    Response = MockServiceResponse

class MockSetString(_MockBase):
    class Request(MockServiceRequest):
        def __init__(self, data=None): self.data = data
    Response = MockServiceResponse


# Assign mocks as the default global names for ROS types/modules
rclpy = MockRclpy() # Mock the rclpy module itself
Node = MockNode
String = MockStringMsg
Float32MultiArray = MockFloat32MultiArrayMsg
Trigger = MockTrigger
SetString = MockSetString # This will be the name used for the service type


try:
    # Try importing core ROS2 packages
    import rclpy as rclpy_real
    from rclpy.node import Node as Node_real
    from std_msgs.msg import String as String_real, Float32MultiArray as Float32MultiArray_real
    from std_srvs.srv import Trigger as Trigger_real

    # If core imports succeed, update globals and set HAS_ROS2
    rclpy = rclpy_real
    Node = Node_real
    String = String_real
    Float32MultiArray = Float32MultiArray_real
    Trigger = Trigger_real
    HAS_ROS2 = True  # Crucially, set this after successful core imports

    # Now, specifically try to import SetString, as it might be in different packages
    _set_string_found = False
    try:
        from example_interfaces.srv import SetString as SetString_example
        SetString = SetString_example # Overwrite mock with real type
        _set_string_found = True
    except ImportError:
        pass # example_interfaces.srv.SetString not found, try next option

    if not _set_string_found:
        try:
            # std_srvs.srv.SetString is non-standard but check just in case
            from std_srvs.srv import SetString as SetString_std
            SetString = SetString_std # Overwrite mock with real type
            _set_string_found = True
            print("Warning: Using SetString from std_srvs.srv. "
                  "Please ensure this is the intended service definition for your ROS 2 setup.", file=sys.stderr)
        except ImportError:
            pass # std_srvs.srv.SetString also not found

    if not _set_string_found and HAS_ROS2: # If still using MockSetString despite core ROS being present
        print("Warning: Core ROS 2 packages found, but a 'SetString' service definition "
              "(e.g., from 'example_interfaces.srv') was not located. "
              "Services requiring 'SetString' (G-code, Feed Rate, Jog) will use a mock "
              "and are unlikely to function correctly in full ROS mode.", file=sys.stderr)
        # SetString remains MockSetString (its default initial value)

except ImportError as e:
    # This block is hit if core ROS2 packages (rclpy, std_msgs, etc.) are missing.
    # HAS_ROS2 remains False (its initial value).
    # All ROS type globals remain their mock versions assigned earlier.
    # We don't print here, as the main() function will handle showing a messagebox
    pass

# --- End of Revised ROS Import Block ---

class CNCGuiNode(tk.Tk):
    def __init__(self, preview_mode=False):
        super().__init__()
        
        self.preview_mode = preview_mode or not HAS_ROS2
        
        # Configure ttk styles
        self.setup_styles()
        
        # Initialize ROS only if not in preview mode
        if not self.preview_mode:
            rclpy.init(args=sys.argv if hasattr(sys, 'argv') else None)
            self.node = Node('cnc_gui_node')
            self.update_queue = Queue()
            
            self.spin_thread = threading.Thread(target=self.ros_spin, daemon=True)
            self.spin_thread.start()
        
        # Configure main window
        self.title("CNC Control GUI" + (" (Preview Mode)" if self.preview_mode else ""))
        self.geometry("800x600")
        self.configure(bg='#f0f0f0')
        
        # Create GUI components in correct order
        self.build_gui()

    def build_gui(self):
        """Create all GUI elements in the correct order"""
        # Main container
        self.main_frame = ttk.Frame(self, padding="5")
        self.main_frame.pack(fill=tk.BOTH, expand=True)
        
        # Warning banner if needed
        if self.preview_mode:
            self.create_preview_warning()
        
        # Create main sections
        self.create_status_section()
        self.create_control_section()
        self.create_response_section()
        
        # Setup ROS communications last
        if not self.preview_mode:
            self.setup_ros_communications()

    def create_preview_warning(self):
        warning_frame = ttk.Frame(self.main_frame)
        warning_frame.pack(fill=tk.X, padx=5, pady=5)
        
        if not HAS_ROS2:
            warning_text = ("PREVIEW MODE - ROS2 Not Available\n"
                          "This is a non-functional preview of the GUI.\n"
                          "Install ROS2 and run without --preview for full functionality.")
        else:
            warning_text = ("PREVIEW MODE - GUI is non-functional by choice (--preview flag used).\n"
                          "ROS2 is available on your system.")
        
        ttk.Label(warning_frame, text=warning_text,
                 style='Warning.TLabel',
                 justify=tk.CENTER).pack(fill=tk.X)

    def create_status_section(self):
        # Create status frame
        self.status_frame = ttk.LabelFrame(self.main_frame, text="Status")
        self.status_frame.pack(fill=tk.X, padx=5, pady=5)
        
        # Create status widgets
        self.create_status_widgets()

    def create_control_section(self):
        # Create control frame
        self.control_frame = ttk.LabelFrame(self.main_frame, text="Controls")
        self.control_frame.pack(fill=tk.BOTH, expand=True, padx=5, pady=5)
        
        # Create control widgets
        self.create_control_widgets()

    def create_response_section(self):
        # Create response frame with explicit colors
        self.response_frame = ttk.LabelFrame(
            self.main_frame,
            text="GRBL Response"
        )
        self.response_frame.pack(fill=tk.X, padx=5, pady=5)
        
        # Configure response text with explicit colors
        self.response_text = scrolledtext.ScrolledText(
            self.response_frame,
            height=5,
            wrap=tk.WORD,
            bg='white',  # Explicit background color
            fg='black',  # Explicit text color
            font=('Courier', 10)  # Monospace font for better readability
        )
        self.response_text.pack(fill=tk.BOTH, expand=True, padx=5, pady=5)

    def setup_styles(self):
        style = ttk.Style()
        
        # Configure colors and styles
        bg_color = '#f0f0f0'
        fg_color = '#000000'
        
        # Basic styles
        style.configure('Warning.TLabel',
                       foreground='red',
                       font=('Helvetica', 10, 'bold'),
                       background=bg_color)
        
        style.configure('TFrame',
                       background=bg_color)
        
        style.configure('TLabelframe',
                       background=bg_color,
                       padding=5)
        
        style.configure('TLabelframe.Label',
                       font=('Helvetica', 9, 'bold'),
                       background=bg_color,
                       foreground=fg_color)
        
        style.configure('TButton',
                       padding=5,
                       background=bg_color,
                       foreground=fg_color)
        
        style.configure('TEntry',
                       padding=2,
                       fieldbackground='white',
                       background=bg_color)

    def create_frames(self):
        self.status_frame = ttk.LabelFrame(master=self.main_frame, text="Status")
        self.status_frame.pack(fill=tk.X, padx=5, pady=5)

        self.control_frame = ttk.LabelFrame(master=self.main_frame, text="Controls")
        self.control_frame.pack(fill=tk.BOTH, expand=True, padx=5, pady=5)

        self.response_frame = ttk.LabelFrame(master=self.main_frame, text="GRBL Response")
        self.response_frame.pack(fill=tk.X, padx=5, pady=5)

    def create_status_widgets(self):
        self.cnc_status_var = tk.StringVar(value="State: Unknown")
        self.cnc_pos_var = tk.StringVar(value="Position: X:0.000 Y:0.000 Z:0.000")
        self.cnc_mode_var = tk.StringVar(value="Mode: Unknown")

        ttk.Label(self.status_frame, textvariable=self.cnc_status_var).pack(anchor=tk.W)
        ttk.Label(self.status_frame, textvariable=self.cnc_pos_var).pack(anchor=tk.W)
        ttk.Label(self.status_frame, textvariable=self.cnc_mode_var).pack(anchor=tk.W)

        self.sensor_frame = ttk.LabelFrame(self.status_frame, text="Sensor Data")
        self.sensor_frame.pack(fill=tk.X, padx=5, pady=5)
        self.sensor_vars = { k: tk.StringVar(value=f"{k}: 0.00") for k in ['A', 'B', 'C', 'Z']}
        for var in self.sensor_vars.values():
            ttk.Label(self.sensor_frame, textvariable=var).pack(side=tk.LEFT, padx=10)

    def create_control_widgets(self):
        print("Creating control widgets...")
        
        # Main container with visible border
        control_container = tk.Frame(master=self.control_frame, relief=tk.GROOVE, borderwidth=2)
        control_container.pack(fill=tk.BOTH, expand=True, padx=5, pady=5)
        
        # Emergency button - keep minimal styling
        tk.Button(
            master=control_container,
            text="EMERGENCY STOP",
            bg='red',
            height=2,
            command=self.emergency_stop
        ).pack(fill=tk.X, padx=5, pady=5)
        
        # Basic control buttons in frame
        button_frame = tk.Frame(master=control_container, relief=tk.GROOVE, borderwidth=1)
        button_frame.pack(fill=tk.X, padx=5, pady=5)
        
        for text, cmd in [("Home CNC", self.home_cnc),
                         ("Reset CNC", self.reset_cnc),
                         ("Unlock Alarm", self.unlock_alarm)]:
            tk.Button(master=button_frame, text=text, command=cmd).pack(fill=tk.X, padx=5, pady=2)
        
        # G-code section with separate label and frame
        tk.Label(master=control_container, text="G-code:").pack(anchor=tk.W, padx=5)
        
        gcode_frame = tk.Frame(master=control_container, relief=tk.GROOVE, borderwidth=1)
        gcode_frame.pack(fill=tk.X, padx=5, pady=2)
        
        self.gcode_entry = tk.Entry(master=gcode_frame, width=40)  # Explicit width
        self.gcode_entry.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=5, pady=5)
        
        tk.Button(
            master=gcode_frame,
            text="Send",
            command=self.send_gcode,
            width=10  # Explicit width
        ).pack(side=tk.RIGHT, padx=5, pady=5)
        
        print("Control widgets creation complete")

    def create_basic_controls(self, parent):
        basic_frame = ttk.Frame(master=parent)
        basic_frame.pack(fill=tk.BOTH, expand=True, padx=5, pady=5)

        # Remove emergency stop button from here since it's in the main control frame

        # Basic control buttons
        button_frame = ttk.Frame(master=basic_frame)
        button_frame.pack(fill=tk.X, padx=5, pady=5)
        commands = [
            ("Home CNC", self.home_cnc),
            ("Reset CNC", self.reset_cnc),
            ("Unlock Alarm", self.unlock_alarm),
            ("Set Absolute Mode", self.set_absolute_mode),
            ("Set Relative Mode", self.set_relative_mode)
        ]
        for text, cmd_func in commands:
            ttk.Button(
                master=button_frame,
                text=text,
                command=cmd_func
            ).pack(fill=tk.X, padx=5, pady=2)

        gcode_frame = ttk.LabelFrame(master=basic_frame, text="Send G-code")
        gcode_frame.pack(fill=tk.X, padx=5, pady=5)
        self.gcode_entry = ttk.Entry(master=gcode_frame)
        self.gcode_entry.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=5, pady=5)
        ttk.Button(master=gcode_frame, text="Send", command=self.send_gcode).pack(side=tk.RIGHT, padx=5, pady=5)

        feed_frame = ttk.LabelFrame(master=basic_frame, text="Set Feed Rate")
        feed_frame.pack(fill=tk.X, padx=5, pady=5)
        self.feed_entry = ttk.Entry(master=feed_frame)
        self.feed_entry.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=5, pady=5)
        ttk.Button(master=feed_frame, text="Set", command=self.set_feed_rate).pack(side=tk.RIGHT, padx=5, pady=5)

    def create_movement_controls(self, parent):
        movement_frame = ttk.Frame(master=parent)
        movement_frame.pack(fill=tk.BOTH, expand=True, padx=5, pady=5)
        commands = [
            ("Move to Target Origo", self.move_to_target),
            ("Move Over Target Origo", self.move_over_target),
            ("Dock at Target Origo", self.dock_target),
            ("Move to Random Safe Position", self.move_random)
        ]
        for text, cmd_func in commands:
            ttk.Button(master=movement_frame, text=text, command=cmd_func).pack(fill=tk.X, padx=5, pady=2)

    def create_jog_controls(self, parent):
        jog_container = ttk.Frame(master=parent)
        jog_container.pack(fill=tk.BOTH, expand=True, padx=5, pady=5)

        settings_frame = ttk.LabelFrame(master=jog_container, text="Jog Settings")
        settings_frame.pack(fill=tk.X, padx=5, pady=5)
        self.jog_entries = {}
        for axis in ['X', 'Y', 'Z']:
            frame = ttk.Frame(master=settings_frame)
            frame.pack(fill=tk.X, padx=5, pady=2)
            ttk.Label(master=frame, text=f"{axis}:").pack(side=tk.LEFT)
            entry = ttk.Entry(master=frame, width=10)
            entry.pack(side=tk.LEFT, padx=5)
            self.jog_entries[axis] = entry
            entry.insert(0, "1.0")

        frame = ttk.Frame(master=settings_frame)
        frame.pack(fill=tk.X, padx=5, pady=2)
        ttk.Label(master=frame, text="Feed:").pack(side=tk.LEFT)
        self.jog_feed_entry = ttk.Entry(master=frame, width=10)
        self.jog_feed_entry.pack(side=tk.LEFT, padx=5)
        self.jog_feed_entry.insert(0, "1000")

        self.jog_relative_var = tk.BooleanVar(value=True)
        ttk.Checkbutton(master=settings_frame, text="Relative Movement", variable=self.jog_relative_var).pack(padx=5, pady=5)

        jog_buttons_frame = ttk.LabelFrame(master=jog_container, text="Jog Control")
        jog_buttons_frame.pack(fill=tk.BOTH, expand=True, padx=5, pady=5)
        directions = [
            ('↖', -1, 1), ('↑', 0, 1), ('↗', 1, 1),
            ('←', -1, 0), ('·', 0, 0), ('→', 1, 0),
            ('↙', -1, -1), ('↓', 0, -1), ('↘', 1, -1)
        ]
        for i, (symbol, dx, dy) in enumerate(directions):
            row, col = i // 3, i % 3
            if symbol == '·': continue
            btn = ttk.Button(master=jog_buttons_frame, text=symbol, command=lambda x=dx, y=dy: self.jog_xy(x, y))
            btn.grid(row=row, column=col, padx=2, pady=2, sticky="nsew")
        
        ttk.Button(master=jog_buttons_frame, text="Z ↑", command=lambda: self.jog_z(1)).grid(row=0, column=3, padx=2, pady=2, sticky="nsew")
        ttk.Button(master=jog_buttons_frame, text="Z ↓", command=lambda: self.jog_z(-1)).grid(row=1, column=3, padx=2, pady=2, sticky="nsew")
        
        for i in range(4): jog_buttons_frame.grid_columnconfigure(i, weight=1)
        for i in range(3): jog_buttons_frame.grid_rowconfigure(i, weight=1)

    def setup_ros_communications(self):
        self.node.create_subscription(String, '/cnc/status', self.status_callback, 10)
        self.node.create_subscription(Float32MultiArray, '/magnetic_sensor_data', self.sensor_callback, 10)
        self.node.create_subscription(String, '/cnc/response', self.response_callback, 10)

        self.services = {
            'home': self.node.create_client(Trigger, '/cnc/home'),
            'reset': self.node.create_client(Trigger, '/cnc/reset'),
            'estop': self.node.create_client(Trigger, '/cnc/emergency_stop'),
            'unlock': self.node.create_client(Trigger, '/cnc/unlock_alarm'),
            'abs_mode': self.node.create_client(Trigger, '/cnc/set_absolute_mode'),
            'rel_mode': self.node.create_client(Trigger, '/cnc/set_relative_mode'),
            'target': self.node.create_client(Trigger, '/cnc/move_to_target_origo'),
            'over_target': self.node.create_client(Trigger, '/cnc/move_over_target_origo'),
            'dock': self.node.create_client(Trigger, '/cnc/dock_at_target_origo'),
            'random': self.node.create_client(Trigger, '/cnc/move_to_random_safe_position'),
            'gcode': self.node.create_client(SetString, '/cnc/send_gcode'),
            'feed': self.node.create_client(SetString, '/cnc/set_feed_rate'),
            'jog': self.node.create_client(SetString, '/cnc/jog_increment')
        }

    def _update_response_display(self, message):
        def update():
            # Ensure text is visible by setting tag colors
            self.response_text.tag_configure('message', foreground='black')
            self.response_text.insert(tk.END, message + "\n", 'message')
            self.response_text.see(tk.END)
            
            # Keep only last N lines to prevent unbounded growth
            lines = self.response_text.get('1.0', tk.END).splitlines()
            if len(lines) > 100:  # Keep last 100 lines
                self.response_text.delete('1.0', tk.END)
                self.response_text.insert(tk.END, '\n'.join(lines[-100:]))
        
        if hasattr(self, 'update_queue') and not self.preview_mode:
            self.update_queue.put(update)
        else:
            self.after(0, update) # Use self.after for main thread updates if not using queue


    def queue_gui_update(self, callable_func, *args):
        if hasattr(self, 'update_queue') and not self.preview_mode:
            self.update_queue.put(lambda: callable_func(*args))
        else: 
            self.after(0, lambda: callable_func(*args)) # Use self.after for main thread

    def call_trigger_service(self, service_name):
        if self.preview_mode:
            self._update_response_display(f"Preview: {service_name} (Trigger)")
            return
        
        client = self.services.get(service_name)
        if client and client.service_is_ready():
            future = client.call_async(Trigger.Request())
            future.add_done_callback(
                lambda f: self.queue_gui_update(self._handle_service_response, service_name, f.result()))
        elif client: self._update_response_display(f"Service '{service_name}' not ready.")
        else: self._update_response_display(f"Client for '{service_name}' not found.")

    def call_string_service(self, service_name, data):
        if self.preview_mode:
            self._update_response_display(f"Preview: {service_name} with '{data}' (String)")
            return
        
        client = self.services.get(service_name)
        if client and client.service_is_ready():
            request = SetString.Request()
            request.data = data
            future = client.call_async(request)
            future.add_done_callback(
                lambda f: self.queue_gui_update(self._handle_service_response, service_name, f.result()))
        elif client: self._update_response_display(f"Service '{service_name}' not ready.")
        else: self._update_response_display(f"Client for '{service_name}' not found.")

    def _handle_service_response(self, service_name, response):
        if response:
            status = "Success" if hasattr(response, 'success') and response.success else "Failed/Unknown"
            msg = response.message if hasattr(response, 'message') else "No message"
            self._update_response_display(f"{service_name}: {status} - {msg}")
        else:
            self._update_response_display(f"{service_name}: Call failed/timed out.")

    def status_callback(self, msg):
        try: data = json.loads(msg.data)
        except json.JSONDecodeError:
            self.queue_gui_update(self._update_response_display, f"Err JSON status: {msg.data}")
            return
        self.queue_gui_update(self._update_status_display, data)

    def _update_status_display(self, data):
        self.cnc_status_var.set(f"State: {data.get('state', 'N/A')}")
        pos = data.get('pos', {})
        self.cnc_pos_var.set(f"Pos: X:{pos.get('x', 0):.3f} Y:{pos.get('y', 0):.3f} Z:{pos.get('z', 0):.3f}")
        self.cnc_mode_var.set(f"Mode: {data.get('mode', 'N/A')}")

    def sensor_callback(self, msg):
        if len(msg.data) == 4:
            self.queue_gui_update(self._update_sensor_display, msg.data)

    def _update_sensor_display(self, data):
        for i, key in enumerate(['A', 'B', 'C', 'Z']):
            self.sensor_vars[key].set(f"{key}: {data[i]:.2f}")

    def response_callback(self, msg):
        self.queue_gui_update(self._update_response_display, f"GRBL: {msg.data}")

    def emergency_stop(self): self.call_trigger_service('estop')
    def home_cnc(self): self.call_trigger_service('home')
    def reset_cnc(self): self.call_trigger_service('reset')
    def unlock_alarm(self): self.call_trigger_service('unlock')
    def set_absolute_mode(self): self.call_trigger_service('abs_mode')
    def set_relative_mode(self): self.call_trigger_service('rel_mode')
    def move_to_target(self): self.call_trigger_service('target')
    def move_over_target(self): self.call_trigger_service('over_target')
    def dock_target(self): self.call_trigger_service('dock')
    def move_random(self): self.call_trigger_service('random')

    def send_gcode(self):
        gcode = self.gcode_entry.get().strip()
        if gcode:
            self.call_string_service('gcode', gcode)
            self.gcode_entry.delete(0, tk.END)

    def set_feed_rate(self):
        feed = self.feed_entry.get().strip()
        if feed:
            try:
                float(feed)
                self.call_string_service('feed', feed)
                self.feed_entry.delete(0, tk.END)
            except ValueError: self._update_response_display("Invalid feed: Must be number.")
    
    def jog_xy(self, dx, dy): self._send_jog_command({'x': dx, 'y': dy, 'z': 0.0})
    def jog_z(self, dz): self._send_jog_command({'x': 0.0, 'y': 0.0, 'z': dz})

    def _send_jog_command(self, axes_directions):
        try:
            command = {'relative': self.jog_relative_var.get(), 'feed': float(self.jog_feed_entry.get() or "100")}
            for axis_char, direction in axes_directions.items():
                increment = float(self.jog_entries[axis_char.upper()].get() or "0.0")
                command[axis_char] = direction * increment
            
            if self.preview_mode:
                self._update_response_display(f"Preview Jog: {json.dumps(command)}")
                return
            self.call_string_service('jog', json.dumps(command))
        except ValueError as e:
            self._update_response_display(f"Invalid jog params: {e}")

    def ros_spin(self):
        if self.preview_mode: return # Should not happen if thread isn't started
        try:
            while rclpy.ok():
                rclpy.spin_once(self.node, timeout_sec=0.05) # Reduced timeout
                while not self.update_queue.empty():
                    try: callback = self.update_queue.get_nowait()
                    except Exception: break # Queue empty between check and get
                    try: callback()
                    except Exception as e: print(f"GUI update error: {e}", file=sys.stderr)
        except Exception as e:
            if rclpy.ok(): print(f"ROS spin loop error: {e}", file=sys.stderr)

    def on_closing(self):
        if not self.preview_mode and rclpy.ok():
            # Potentially stop the spin_thread more gracefully if it's stuck
            # For now, just destroy node and shutdown rclpy
            if hasattr(self, 'node'): # Check if node was created
                 self.node.destroy_node()
            rclpy.shutdown()
        self.quit()
        self.destroy()

def main():
    parser = argparse.ArgumentParser(description='CNC Control GUI')
    parser.add_argument('--preview', action='store_true', help='Run in non-functional preview mode')
    args = parser.parse_args()

    # Show error dialog if ROS2 is missing AND not in preview mode
    if not HAS_ROS2 and not args.preview:
        root = tk.Tk()
        root.withdraw() # Hide the dummy root window
        messagebox.showerror(
            "ROS2 Not Found",
            "ROS2 Python packages (rclpy, etc.) were not found.\n\n"
            "This program requires ROS2 to be installed and its Python packages to be accessible.\n\n"
            "You can either:\n"
            "1. Install ROS2 and ensure your environment is sourced correctly, or\n"
            "2. Run this program with the '--preview' flag to see a non-functional GUI preview.\n\n"
            "For ROS2 installation instructions, please visit:\n"
            "https://docs.ros.org/en/rolling/Installation.html (or your specific ROS2 distro)"
        )
        sys.exit(1)

    app = CNCGuiNode(preview_mode=args.preview)
    app.protocol("WM_DELETE_WINDOW", app.on_closing)
    app.mainloop()

if __name__ == '__main__':
    main()