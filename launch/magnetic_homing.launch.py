from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    """Generate launch description for magnetic homing system."""
    
    # Launch arguments
    sensor_ip_arg = DeclareLaunchArgument(
        'sensor_ip',
        default_value='192.168.1.100',
        description='IP address of the magnetic sensor device'
    )
    
    serial_port_arg = DeclareLaunchArgument(
        'serial_port',
        default_value='/dev/ttyUSB0',
        description='Serial port for the CNC controller'
    )
    
    baud_rate_arg = DeclareLaunchArgument(
        'baud_rate',
        default_value='115200',
        description='Baud rate for the CNC controller'
    )
    
    # Create sensor node
    # We cannot call a python script directly, so we call the executable declared in entry_points/console_scripts in setup.py
    sensor_node = Node(
        package='magnetic_homing',
        executable='magnetic_sensor_node',
        name='magnetic_sensor_node',
        parameters=[{
            'sensor_ip': LaunchConfiguration('sensor_ip'),
            'publish_rate': 10.0
        }],
        output='screen'
    )
    
    # Create CNC controller node
    # We cannot call a python script directly, so we call the executable declared in entry_points/console_scripts in setup.py
    cnc_node = Node(
        package='magnetic_homing',
        executable='cnc_node',
        name='cnc_controller_node',
        parameters=[{
            'serial_port': LaunchConfiguration('serial_port'),
            'baud_rate': LaunchConfiguration('baud_rate')
        }],
        output='screen'
    )
    
    return LaunchDescription([
        sensor_ip_arg,
        serial_port_arg,
        baud_rate_arg,
        sensor_node,
        cnc_node
    ])
