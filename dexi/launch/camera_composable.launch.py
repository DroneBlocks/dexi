from launch.actions import DeclareLaunchArgument
from launch.launch_description import LaunchDescription
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    """
    Generate a launch description with for the camera node and a visualiser.

    Returns
    -------
        LaunchDescription: the launch description

    """
    # parameters
    camera_param_name = "camera"
    camera_param_default = str(0)
    camera_param = LaunchConfiguration(
        camera_param_name,
        default=camera_param_default,
    )
    camera_launch_arg = DeclareLaunchArgument(
        camera_param_name,
        default_value=camera_param_default,
        description="camera ID or name"
    )

    format_param_name = "format"
    format_param_default = str()
    format_param = LaunchConfiguration(
        format_param_name,
        default=format_param_default,
    )
    format_launch_arg = DeclareLaunchArgument(
        format_param_name,
        default_value=format_param_default,
        description="pixel format"
    )

    # composable nodes in single container
    container = ComposableNodeContainer(
        name='camera_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[

            # Camera node
            ComposableNode(
                package='camera_ros',
                plugin='camera::CameraNode',
                parameters=[{
                    "camera": camera_param,
                    "width": 320,
                    "height": 240,
                    "format": format_param,
                }],
                remappings=[('~/image_raw', 'image_rect'),
                            ('~/camera_info', 'camera_info')],
                extra_arguments=[{'use_intra_process_comms': True}],
            ),

            # April tag node
            ComposableNode(
                package='apriltag_ros',
                plugin='AprilTagNode',
                extra_arguments=[{'use_intra_process_comms': True}],
            ),

            # April tag throttling
            ComposableNode(
                package='topic_tools',
                plugin='topic_tools::ThrottleNode',
                extra_arguments=[{'use_intra_process_comms': True}],
                parameters=[
                    {'throttle_type': 'messages'},
                    {'input_topic': '/detections'},
                    {'output_topic': '/throttled/detections'},
                    {'msgs_per_sec': 1.0}
                ]
            ),
        ],
    )

    web_video_node = Node(
         package='web_video_server',
         executable='web_video_server',
    )

    return LaunchDescription([
        container,
        web_video_node,
        camera_launch_arg,
        format_launch_arg,
    ])