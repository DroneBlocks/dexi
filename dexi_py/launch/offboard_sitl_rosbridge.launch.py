import os
import launch
from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource

def generate_launch_description():
    # Get the path to the rosbridge websocket launch file
    rosbridge_launch_file = os.path.join(
        get_package_share_directory('rosbridge_server'),
        'launch',
        'rosbridge_websocket_launch.xml'
    )

    return LaunchDescription([
        # Start Micro XRCE-DDS Agent
        ExecuteProcess(
            cmd=['MicroXRCEAgent', 'udp4', '--port', '8888'],
            output='screen'
        ),

        # Start px4_offboard_manager node
        Node(
            package='dexi_py',
            executable='px4_offboard_manager',
            name='px4_offboard_manager',
            namespace='dexi',
            output='screen'
        ),

        # Include rosbridge websocket launch file
        IncludeLaunchDescription(
            XMLLaunchDescriptionSource(rosbridge_launch_file),
            launch_arguments={
                'port': '9090'
            }.items()
        )
    ])
