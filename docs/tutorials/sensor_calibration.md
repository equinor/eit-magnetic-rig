# Magnetic Sensor Calibration

This tutorial explains how to calibrate the magnetic sensors to obtain accurate and meaningful readings.

## Understanding Calibration Parameters

The Magnetic Sensor Node provides two calibration parameters:

1. **`calibration_factor`** - A multiplier applied to raw sensor values
2. **`calibration_offset`** - An offset added after multiplication

The calibration formula is:

```
calibrated_value = raw_value * calibration_factor + calibration_offset
```

These parameters allow you to convert raw sensor readings into meaningful units or normalize the values across different sensors.

## Determining Calibration Values

### Method 1: Known Reference Values

If you have a known magnetic field strength reference:

1. Place the reference at a fixed distance from the sensor
2. Note the raw sensor reading
3. Calculate calibration_factor = reference_value / raw_value
4. Set calibration_offset = 0 (or adjust if needed)

### Method 2: Normalization

To normalize readings across multiple sensors:

1. Place all sensors at the same distance from a magnetic source
2. Record raw readings from all sensors
3. Choose one sensor as the reference
4. For each other sensor, calculate:
   - calibration_factor = reference_reading / current_sensor_reading
   - calibration_offset = 0

### Method 3: Range Mapping

To map sensor values to a desired range:

1. Record minimum (min_raw) and maximum (max_raw) raw readings
2. Determine desired output range (min_desired to max_desired)
3. Calculate:
   - calibration_factor = (max_desired - min_desired) / (max_raw - min_raw)
   - calibration_offset = min_desired - (min_raw * calibration_factor)

## Applying Calibration Parameters

### Setting Parameters at Launch

Include calibration parameters in your launch file:

```python
sensor_node = Node(
    package='magnetic_homing',
    executable='magnetic_sensor_node.py',
    name='magnetic_sensor_node',
    parameters=[{
        'sensor_ip': '192.168.1.254',
        'calibration_factor': 1.25,
        'calibration_offset': -0.5
    }],
    output='screen'
)
```

### Setting Parameters at Runtime

Use the ROS2 parameter set command:

```bash
ros2 param set /magnetic_sensor_node calibration_factor 1.25
ros2 param set /magnetic_sensor_node calibration_offset -0.5
```

## Calibration Procedure Example

Here's a step-by-step example of calibrating your sensors:

1. **Gather Raw Data**:
   ```bash
   # Start the magnetic sensor node with default calibration
   ros2 run magnetic_homing magnetic_sensor_node.py
   
   # Record readings at various positions
   ros2 topic echo /magnetic_sensor_data > sensor_readings.txt
   ```

2. **Analyze the Data**:
   - Review the recorded data to identify minimum and maximum values
   - Determine the desired output range (e.g., 0-100 or 0-1)

3. **Calculate Calibration Parameters**:
   - Use the range mapping method to calculate calibration_factor and calibration_offset

4. **Apply the Calibration**:
   ```bash
   # Restart the node with calibration parameters
   ros2 run magnetic_homing magnetic_sensor_node.py --ros-args -p calibration_factor:=1.25 -p calibration_offset:=-0.5
   ```

5. **Verify the Calibration**:
   ```bash
   # Check that values are now in the expected range
   ros2 topic echo /magnetic_sensor_data
   ```

## Sensor-Specific Calibration

Each sensor may require different calibration parameters. You can create a custom launch file with individual calibration values for each sensor. Currently, the implementation applies the same calibration to all sensors, but you could enhance the node to support per-sensor calibration if needed.

## Calibration Maintenance

- Re-calibrate periodically to account for sensor drift
- Document your calibration values and procedure for future reference
- Consider environmental factors that might affect readings (temperature, nearby metal objects)

## Advanced Calibration Considerations

### Temperature Compensation

If sensor readings vary with temperature, you might need to implement temperature compensation:

1. Measure sensor readings at different temperatures
2. Determine the relationship between temperature and reading drift
3. Implement a compensation algorithm in your processing code

### Non-Linear Calibration

For sensors with non-linear response, the simple linear calibration might be insufficient. In such cases, you might need to:

1. Sample readings across the full range of expected values
2. Fit a higher-order polynomial to the data
3. Implement a custom calibration function in your processing code
