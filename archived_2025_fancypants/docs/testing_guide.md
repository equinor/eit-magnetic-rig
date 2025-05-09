# Testing Guide: Magnetic Homing System

This document provides instructions on how to run the unit tests for the `magnetic_homing` Python nodes.

## Phase 1: Unit Tests (Hardware NOT Attached, No Live ROS2 Environment)

**Objective:** Verify the internal logic of individual Python node scripts in isolation. These tests use Python's `unittest` framework and `unittest.mock` to simulate hardware dependencies (like TCP sockets or serial ports) and ROS2-specific calls (like parameter getting, publisher creation).

**Prerequisites:**

1.  **Python 3:** Ensure Python 3 is installed on your system.
2.  **Required Python Packages:**
    *   `rclpy`: Even though we are not running a full ROS2 environment, the node classes inherit from `rclpy.node.Node`. The `rclpy` library needs to be importable by Python for the node classes to be defined and instantiated.
    *   `std_msgs`, `std_srvs`, `geometry_msgs`: These are standard ROS2 message and service type libraries that need to be importable.
    *   `pyserial`: The `cnc_controller_node.py` likely uses this for serial communication. Even if mocked, the import might be attempted.
    *   `magnetic_homing` (as a Python package): The tests will need to import the node scripts (e.g., `from magnetic_homing.scripts.magnetic_sensor_node import MagneticSensorNode`). For this to work without ROS2 build tools, your `PYTHONPATH` environment variable must be set up so that Python can find your `magnetic_homing` package.
        *   If your `magnetic_homing` package is in `/Volumes/External/Dev/250424_magnetic_homing`, and your scripts are in a `scripts` subdirectory, you might need to add `/Volumes/External/Dev/250424_magnetic_homing` to your `PYTHONPATH`.

**Directory Structure (Example):**

```
/Volumes/External/Dev/250424_magnetic_homing/
├── magnetic_homing/
│   ├── scripts/
│   │   ├── __init__.py  (can be empty, makes 'scripts' a package)
│   │   ├── magnetic_sensor_node.py
│   │   └── cnc_controller_node.py
│   ├── msg/
│   │   └── MagneticSensorData.msg (if used)
│   ├── srv/
│   ├── package.xml
│   └── CMakeLists.txt
├── test/
│   ├── __init__.py (can be empty, makes 'test' a package)
│   ├── test_magnetic_sensor_node.py
│   └── test_cnc_controller_node.py
└── docs/
    └── testing_guide.md
```

**Running Phase 1 Unit Tests:**

1.  **Navigate to the root directory** of your project in the terminal (e.g., `/Volumes/External/Dev/250424_magnetic_homing`).
2.  **Set `PYTHONPATH` (if needed):**
    If your project structure is like the example, and you are in `/Volumes/External/Dev/250424_magnetic_homing`, you might run:
    ```bash
    export PYTHONPATH=$(pwd):$PYTHONPATH
    ```
    This allows Python to find the `magnetic_homing.scripts` module. The test scripts use `from magnetic_homing.scripts... import ...`.

3.  **Run all tests using Python's `unittest` discovery:**
    This command will find and run all files named `test_*.py` in the `test` directory.
    ```bash
    python3 -m unittest discover -s test -p "test_*.py"
    ```
    Or, to run tests for a specific file:
    *   For `magnetic_sensor_node.py`:
        ```bash
        python3 -m unittest test.test_magnetic_sensor_node
        ```
    *   For `cnc_controller_node.py`:
        ```bash
        python3 -m unittest test.test_cnc_controller_node
        ```

**Interpreting Results:**

*   **Successful Tests:** For each test method, you will see an "OK" if it passes. A summary line will show the total number of tests run and "OK".
    ```
    ..................................................
    ----------------------------------------------------------------------
    Ran 50 tests in 0.250s

    OK
    ```
*   **Failed Tests:** If a test fails, you will see an "F" or "ERROR" and a traceback indicating the assertion that failed or the error that occurred. This will point to the specific test method and the line number in your test script.
    ```
    ..F...............................................
    ======================================================================
    FAIL: test_some_functionality (test.test_some_node.TestSomeNode)
    ----------------------------------------------------------------------
    Traceback (most recent call last):
      File "/path/to/your/test/test_some_node.py", line XX, in test_some_functionality
        self.assertEqual(result, expected_value)
    AssertionError: XXX != YYY
    ...
    ----------------------------------------------------------------------
    Ran 50 tests in 0.250s

    FAILED (failures=1)
    ```
*   **Skipped Tests:** Some tests might be skipped (e.g., if a dependency like a custom message type is not available or a feature is not implemented). These will be marked as "s".

**Phase 1 Test Coverage:**

The Phase 1 unit tests aim to cover the internal logic of each node:

**A. `test_magnetic_sensor_node.py` Coverage:**

*   **`__init__(self)` (Constructor):**
    *   Parameter declaration and retrieval.
    *   Publisher creation (mocked, for `MagneticSensorData` or `Float32MultiArray`).
    *   Service creation (mocked, for `set_sensor_ip`).
    *   Timer creation (mocked) with correct period.
*   **`set_sensor_ip_callback(self, request, response)`:**
    *   IP address update logic.
    *   Correct response formulation.
*   **`parse_sensor_response(self, data)`:**
    *   Parsing valid data for all sensor types (A, B, C, Z).
    *   Handling of short packets, incorrect headers, missing sensor ID patterns, and insufficient data for values.
    *   Handling of empty data and unknown sensor IDs.
*   **`query_sensor(self, sensor_cmd)`:** (Mocks `socket.socket`)
    *   Successful TCP query and response.
    *   Handling of connection timeouts, refusals, send errors, and receive errors.
*   **`query_all_sensors(self)`:** (Mocks `self.query_sensor`, `self.parse_sensor_response`)
    *   Successful query of all sensors.
    *   Handling of partial failures (one sensor query fails, one sensor parse fails).
    *   Handling of all sensors failing.
*   **`timer_callback(self)`:** (Mocks `self.query_all_sensors`, `self.publisher_.publish`)
    *   Successful data query, calibration, and publication (for both custom and default message types).
    *   Handling of failed sensor queries (no publish).
    *   Exception handling during the callback.

**B. `test_cnc_controller_node.py` Coverage:**

*   **`__init__(self)` (Constructor):**
    *   Parameter declaration and retrieval (serial port, baud rate, feed rate, timeouts).
    *   Creation of publishers (mocked, for status, position, raw responses).
    *   Creation of services (mocked, for homing, reset, G-code sending, etc.).
    *   Initialization of serial connection attempt (mocked `serial.Serial`).
    *   Initialization of threading for `serial_read_loop` (mocked `threading.Thread`).
*   **Serial Connection Methods (e.g., `connect_serial`, `disconnect_serial` - if public):**
    *   Successful connection and disconnection logic.
    *   Handling of connection failures (e.g., `serial.SerialException`).
*   **`parse_status(self, status_string)`:**
    *   Parsing various valid GRBL status strings (Idle, Run, Alarm, with MPos, WPos, FS).
    *   Correct updates to `machine_state`, `machine_position`, `work_position`.
    *   Publication of status and position messages.
    *   Handling of malformed status strings.
*   **`send_command(self, command, wait_for_response=True)`:** (Mocks `self.serial_connection.write`, `self.response_buffer`)
    *   Successful command sending and "ok" response.
    *   Handling of "error:X" responses from GRBL.
    *   Command timeout logic.
    *   Behavior when not connected to serial port.
    *   Handling of serial write exceptions.
    *   Correct usage of command lock.
*   **Service Callbacks (e.g., `home_callback`, `reset_grbl_callback`, `send_gcode_callback`, `set_feed_rate_callback`):**
    *   Correct G-code or control characters sent via `self.send_command`.
    *   Response success/message based on `send_command` outcome.
    *   Specific logic for each service (e.g., Ctrl-X for reset).
*   **`serial_read_loop(self)` (Conceptual test of its logic):**
    *   Reading lines from the (mocked) serial port.
    *   Calling `parse_status` for status messages.
    *   Adding responses to `response_buffer`.
    *   Publishing raw responses.
    *   Respecting `self.stop_thread` for loop termination.

*(This document will be updated with Phase 2 instructions once those tests are implemented.)*
---
