import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
   config = os.path.join(
      get_package_share_directory('dexi_py'),
      'config',
      'gpio.yaml'
      )

   return LaunchDescription([
      Node(
         package='dexi_py',
         namespace='dexi',
         executable='gpio_writer_service',
         name='gpio_writer_service',
         parameters=[config]
      )
   ])