from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch import LaunchDescription

def generate_launch_description():

    num_pixels_arg = DeclareLaunchArgument('num_pixels', default_value='45', description='Number of pixels')

    return LaunchDescription([
        num_pixels_arg,
        
        Node(
            package='dexi_py',
            namespace='dexi',
            executable='led_service',
            name='led_service',
            parameters=[{'num_pixels': LaunchConfiguration('num_pixels')}]
        ),

        Node(
            package='dexi_py',
            namespace='dexi',
            executable='flight_mode_status',
            name='flight_mode_status'
        ),
    ])
