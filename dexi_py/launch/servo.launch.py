from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='dexi_py',
            executable='servo_controller',
            name='servo_controller_13',
            parameters=[
                {'servo_pin': 13},
                {'initial_pwm': 1500}
            ],
            remappings=[
                ('/servo_pwm', '/servo_13_pwm')
            ]
        ),
        Node(
            package='dexi_py',
            executable='servo_controller',
            name='servo_controller_16',
            parameters=[
                {'servo_pin': 16},
                {'initial_pwm': 1500}
            ],
            remappings=[
                ('/servo_pwm', '/servo_16_pwm')
            ]
        ),
    ])
