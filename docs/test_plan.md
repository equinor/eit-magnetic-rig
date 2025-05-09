# Test Plan: Magnetic Homing System

This document outlines the testing strategy for the `magnetic_homing` ROS2 package, covering unit tests for individual node logic and integration tests for ROS2 communication.

## I. Phase 1: Unit Tests (Hardware NOT Attached)

**Objective:** Verify the internal logic of each node in isolation, using mocks for hardware dependencies (TCP sockets, serial ports) and ROS2 components where necessary.

**Framework:** `unittest` or `pytest`

---

### A. `magnetic_sensor_node.py`

**1. `__init__(self)` (Constructor)**
    *   **Test:** Parameter declaration and retrieval.
        *   **Mock:** `rclpy.node.Node.declare_parameter`, `rclpy.node.Node.get_parameter` (to return mock parameter objects).
        *   **Assert:** Correct default values are used if parameters are not set. Correct values are assigned to `self.sensor_ip`, `self.calibration_factor`, etc.
    *   **Test:** Publisher creation.
        *   **Mock:** `rclpy.node.Node.create_publisher`.
        *   **Assert:** `create_publisher` is called with the correct message type (`MagneticSensorData` or `Float32MultiArray` based on `USE_CUSTOM_MSG`), topic name (`'magnetic_sensor_data'`), and QoS.
    *   **Test:** Service creation.
        *   **Mock:** `rclpy.node.Node.create_service`.
        *   **Assert:** `create_service` is called with the correct service type (`SetBool`), service name (`'set_sensor_ip'`), and callback.
    *   **Test:** Timer creation.
        *   **Mock:** `rclpy.node.Node.create_timer`.
        *   **Assert:** `create_timer` is called with the correct period (calculated from `publish_rate`) and callback.
    *   **Mock:** `self.get_logger()` to capture log messages.

**2. `set_sensor_ip_callback(self, request, response)`**
    *   **Test:** Successful IP update.
        *   **Input:** Mock `request` with `request.data = "192.168.1.200"`.
        *   **Assert:** `self.sensor_ip` is updated. `response.success` is `True`. `response.message` is as expected. Logger is called.
    *   **Mock:** `self.get_logger()`.

**3. `parse_sensor_response(self, data)`**
    *   **Test:** Valid Sensor A data.
        *   **Input:** `bytes([0x01, 0x02, 0x01, 0x00, 0x10, 0x03, 0x10, 0x03, 0x01, 0x02, 0xchecksum])` (assuming value 0x0102 = 258).
        *   **Assert:** Returns `(0, 258.0)`.
    *   **Test:** Valid Sensor B, C, Z data (similar to Sensor A with respective IDs and indices).
    *   **Test:** Short packet (e.g., `len(data) < 7`).
        *   **Input:** `bytes([0x01, 0x02, 0x01, 0x00, 0x10])`.
        *   **Assert:** Returns `None`. Warning logged.
    *   **Test:** Incorrect header (e.g., `data[0] != 0x01`).
        *   **Input:** `bytes([0x00, 0x02, 0x01, 0x00, 0x10, 0x03, 0x10, 0x03, 0x01, 0x02, 0xchecksum])`.
        *   **Assert:** Returns `None`. Debug message logged.
    *   **Test:** Valid header, but no recognizable sensor ID pattern.
        *   **Input:** `bytes([0x01, 0x02, 0x01, 0xAA, 0xBB, 0xCC, 0xDD, 0xEE])`.
        *   **Assert:** Returns `None`. Debug message logged.
    *   **Test:** Sensor ID pattern found, but data too short for value.
        *   **Input:** `bytes([0x01, 0x02, 0x01, 0x00, 0x10, 0x03, 0x10, 0x03])`.
        *   **Assert:** Returns `None`.
    *   **Test:** Empty data `bytes()`.
        *   **Assert:** Returns `None`. Warning logged.
    *   **Mock:** `self.get_logger()`.

**4. `query_sensor(self, sensor_cmd)`**
    *   **Mock:** `socket.socket` (and its methods `connect`, `sendall`, `recv`, `settimeout`, `close`).
    *   **Test:** Successful query.
        *   **Mock `recv`:** Return a valid sensor response byte string.
        *   **Assert:** `sendall` called with `sensor_cmd`. Returns the mocked response bytes.
    *   **Test:** Connection timeout.
        *   **Mock `connect`:** Raise `socket.timeout`.
        *   **Assert:** Returns `None`. Error logged.
    *   **Test:** Connection refused.
        *   **Mock `connect`:** Raise `ConnectionRefusedError`.
        *   **Assert:** Returns `None`. Error logged.
    *   **Test:** Send error.
        *   **Mock `sendall`:** Raise `socket.error`.
        *   **Assert:** Returns `None`. Error logged.
    *   **Test:** Receive error/timeout.
        *   **Mock `recv`:** Raise `socket.timeout` or `socket.error`.
        *   **Assert:** Returns `None`. Error logged.
    *   **Mock:** `self.get_logger()`.

**5. `query_all_sensors(self)`**
    *   **Mock:** `self.query_sensor`, `self.parse_sensor_response`, `time.sleep`.
    *   **Test:** All sensors respond successfully.
        *   **Mock `self.query_sensor`:** Return valid response bytes for each cmd.
        *   **Mock `self.parse_sensor_response`:** Return valid `(idx, value)` for each response.
        *   **Assert:** `self.sensor_values` correctly updated. Returns `True`.
    *   **Test:** One sensor fails to respond (query_sensor returns `None`).
        *   **Assert:** Other sensors updated. `self.sensor_values` reflects partial update. Returns `True`.
    *   **Test:** One sensor response is unparseable (parse_sensor_response returns `None`).
        *   **Assert:** Other sensors updated. Returns `True`.
    *   **Test:** Sensor index mismatch from `parse_sensor_response`.
        *   **Mock `self.parse_sensor_response`:** Return `(wrong_idx, value)`.
        *   **Assert:** Value for `idx` is not updated. Warning logged. Returns `True` (if other sensors updated).
    *   **Test:** All sensors fail.
        *   **Mock `self.query_sensor`:** Return `None` for all calls.
        *   **Assert:** `self.sensor_values` unchanged. Returns `False`.
    *   **Mock:** `self.get_logger()`.

**6. `timer_callback(self)`**
    *   **Mock:** `self.query_all_sensors`, `self.publisher_.publish`, `USE_CUSTOM_MSG`, `self.get_logger()`.
    *   **Test:** Successful query and publish (with `USE_CUSTOM_MSG = True`).
        *   **Mock `self.query_all_sensors`:** Return `True`. Set `self.sensor_values` to known values.
        *   **Set:** `self.calibration_factor = 2.0`, `self.calibration_offset = 1.0`.
        *   **Assert:** `self.publisher_.publish` called with `MagneticSensorData` message. Message fields contain calibrated values. Info log for published data.
    *   **Test:** Successful query and publish (with `USE_CUSTOM_MSG = False`).
        *   **Assert:** `self.publisher_.publish` called with `Float32MultiArray` message. `msg.data` contains calibrated values.
    *   **Test:** Failed sensor query.
        *   **Mock `self.query_all_sensors`:** Return `False`.
        *   **Assert:** `self.publisher_.publish` NOT called. Warning logged.
    *   **Test:** Exception during callback.
        *   **Mock `self.query_all_sensors`:** Raise an exception.
        *   **Assert:** Error logged. `self.publisher_.publish` NOT called.

---

### B. `cnc_controller_node.py`

*(Assuming structure from previous analysis)*

**1. `__init__(self)`**
    *   **Test:** Parameter handling, publisher/subscriber/service creation (similar to `MagneticSensorNode`).
    *   **Mock:** `serial.Serial`, `threading.Lock`, ROS2 `create_*` methods, `get_logger`.
    *   **Assert:** Serial connection attempted if parameters valid (mock `serial.Serial` constructor to check args).

**2. `parse_status(self, status_string)`**
    *   **Test:** Various valid GRBL status strings (Idle, Run, Alarm, Hold, MPos, WPos, FS combinations).
        *   **Assert:** `self.machine_state`, `self.machine_position`, `self.work_position`, `self.feed_rate` (if applicable from FS) are correctly updated.
    *   **Test:** Malformed status strings.
        *   **Assert:** Graceful handling, error/warning logged, state remains consistent or defaults.
    *   **Mock:** `self.get_logger()`, `self.status_publisher.publish`, `self.position_publisher.publish`.

**3. `send_command(self, command, wait_for_response=True)`**
    *   **Mock:** `self.serial_connection` (mock its `write`, `readline` methods), `self.command_lock`, `self.response_publisher.publish`.
    *   **Test:** Command sent, "ok" received.
        *   **Assert:** `serial_connection.write` called with `command + '\n'`. Returns `(True, "ok")`.
    *   **Test:** Command sent, "error:X" received.
        *   **Assert:** Returns `(False, "error:X")`.
    *   **Test:** Timeout waiting for response.
        *   **Mock `self.response_buffer` to stay empty or `readline` to simulate timeout.**
        *   **Assert:** Returns `(False, "Command timeout")`.
    *   **Test:** `wait_for_response=False`.
        *   **Assert:** Returns `(True, "Command sent (async)")` immediately after write.
    *   **Test:** Serial port not connected (`self.serial_connected = False`).
        *   **Assert:** Returns `(False, "Not connected to serial port")`.
    *   **Mock:** `self.get_logger()`.

**4. Service Callbacks (e.g., `home_callback`, `set_absolute_mode_callback`, `send_gcode_callback`)**
    *   **Mock:** `self.send_command`.
    *   **Test:** Callback logic for each service.
        *   **Input:** Mock service `request`.
        *   **Assert:** `self.send_command` called with the correct G-code. `response.success` and `response.message` set based on `self.send_command`'s return.
    *   **Test (`set_feed_rate_callback`):** Valid and invalid feed rate values in `request.data` (for `SetBool` service, assuming `request.data` is a float passed as a string or similar if `SetBool` is a placeholder).

**5. `reset_grbl(self)`**
    *   **Mock:** `self.serial_connection.write`, `self.send_command`, `time.sleep`.
    *   **Assert:** `\x18` (Ctrl-X) is written. Correct sequence of setup G-codes (`$X`, `G21`, `G90`, `F...`) are sent via `self.send_command`.

**6. `serial_read_loop(self)` (if testing its logic directly)**
    *   **Mock:** `self.serial_connection.readline`, `self.parse_status`, `self.response_publisher.publish`.
    *   **Test:** Reads lines, adds to `response_buffer`, calls `parse_status` for status messages.
    *   **Test:** Handles `self.stop_thread`.

---

## II. Phase 2: Standalone Hardware Interaction Tests (Hardware ATTACHED, No ROS2 Environment)

**Objective:** Verify basic communication with the actual hardware components without the full ROS2 stack. These are more like manual or scripted sanity checks.

**Method:** Simple Python scripts that instantiate the node classes and directly call their methods. Minimal `rclpy.init()` if needed for parameter loading, but no `rclpy.spin()`.

### A. `MagneticSensorNode`
1.  **Script 1: Query Single Sensor**
    *   Instantiate `MagneticSensorNode`.
    *   Call `node.query_sensor(node.SENSOR_A_CMD)` (and for B, C, Z).
    *   Print the raw byte response. Manually verify if it looks plausible.
    *   Call `node.parse_sensor_response()` on the raw bytes and print the result.
2.  **Script 2: Query All Sensors**
    *   Instantiate `MagneticSensorNode`.
    *   Call `node.query_all_sensors()`.
    *   Print `node.sensor_values`. Manually verify.
3.  **Script 3: Basic Timer Test (Conceptual)**
    *   Instantiate `MagneticSensorNode`.
    *   Manually call `node.timer_callback()` a few times in a loop with a small delay.
    *   Observe console logs for "Published sensor data" or errors.

### B. `CNCControllerNode`
1.  **Script 1: Send Basic G-Code**
    *   Instantiate `CNCControllerNode` (ensure correct serial port/baud).
    *   Call `node.send_command("?")` (status query). Print response.
    *   Call `node.send_command("G0 X1")` (small safe move, **USE WITH CAUTION**). Observe CNC rig. Print response.
2.  **Script 2: Test Homing (Conceptual, **USE WITH EXTREME CAUTION**)**
    *   Instantiate `CNCControllerNode`.
    *   Call `node.home_callback(None, mock_response_object)` or directly `node.send_command("$H")`.
    *   Observe CNC rig behavior.
3.  **Script 3: Monitor State**
    *   Instantiate `CNCControllerNode`.
    *   In a loop, call `node.request_status()` (or `node.send_command("?")`), then `node.parse_status()` on a simulated response if `serial_read_loop` isn't running, or check `node.machine_state` / `node.machine_position` if `serial_read_loop` is made to run.

---

## III. Phase 3: ROS2 Integration Tests (Hardware ATTACHED or MOCKED, ROS2 Environment Connected)

**Objective:** Verify that the nodes function correctly within the ROS2 ecosystem, including topic publishing/subscription and service interactions.

**Framework:** `launch_testing` with `pytest`.

### A. `magnetic_sensor_node`
1.  **Test: `magnetic_sensor_data` Topic Publication**
    *   Launch `magnetic_sensor_node.py`.
    *   Launch a test subscriber node that subscribes to `/magnetic_sensor_data`.
    *   **Assert:** Test subscriber receives messages. Messages have the correct type (`MagneticSensorData` or `Float32MultiArray`). Data values are plausible (e.g., within expected range, change over time if hardware is active).
2.  **Test: `set_sensor_ip` Service**
    *   Launch `magnetic_sensor_node.py`.
    *   Launch a test service client node.
    *   Call `/set_sensor_ip` service with a new IP address.
    *   **Assert:** Service call succeeds. Response is `success=True`. Node logs the IP change. (Difficult to verify effect without a mock server that can be reconfigured or by observing connection attempts).
3.  **Test: Parameter Functionality**
    *   Launch `magnetic_sensor_node.py` with different `publish_rate`, `calibration_factor`, `calibration_offset` via launch file arguments.
    *   **Assert:** Observe message frequency on `/magnetic_sensor_data` matches `publish_rate`. If raw sensor values can be somehow known/mocked, verify calibration is applied to published data.

### B. `cnc_controller_node`
1.  **Test: CNC Status and Position Topic Publications**
    *   Launch `cnc_controller_node.py`.
    *   Launch test subscriber nodes for `/cnc/status`, `/cnc/position`, `/cnc/response`.
    *   Use a test service client to call `/cnc/send_gcode` with "?" or other commands.
    *   **Assert:** Messages are published on all three topics. `/cnc/status` reflects GRBL state. `/cnc/position` reflects MPos/WPos. `/cnc/response` shows "ok" or errors.
2.  **Test: G-Code Service (`/cnc/send_gcode`)**
    *   Launch `cnc_controller_node.py`.
    *   Launch a test service client.
    *   Call `/cnc/send_gcode` with various G-code commands (e.g., "G0 X10", "$H").
    *   **Assert:** Service response indicates success/failure. Observe CNC rig for expected physical movement (if hardware connected and safe). Check `/cnc/response` topic.
3.  **Test: Control Services (`/cnc/home`, `/cnc/reset`, etc.)**
    *   Launch `cnc_controller_node.py`.
    *   Launch test service clients for each control service.
    *   Call each service.
    *   **Assert:** Service response is appropriate. Observe CNC rig behavior.
4.  **Test: `magnetic_sensor_data` Subscription (if `cnc_controller_node` uses it)**
    *   Launch `magnetic_sensor_node.py` and `cnc_controller_node.py`.
    *   (If `cnc_controller_node.sensor_data_callback` has testable logic/logging).
    *   **Assert:** `cnc_controller_node` logs receipt of sensor data or performs an action based on it.

---