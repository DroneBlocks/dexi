from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # Declare a launch argument for servo_pin with a default value of 13
    servo_pin_arg = DeclareLaunchArgument(
        'servo_pin',
        default_value='13',
        description='GPIO pin number connected to the servo'
    )
    
    # Servo Controller Node with the servo_pin parameter
    servo_node = Node(
        package='dexi_py',
        executable='servo_controller',
        name='servo_controller',
        parameters=[{
            'servo_pin': LaunchConfiguration('servo_pin')
        }]
    )
    
    return LaunchDescription([
        servo_pin_arg,
        servo_node
    ])
