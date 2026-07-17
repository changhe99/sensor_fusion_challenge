import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    default_params = os.path.join(
        get_package_share_directory('bmi088_driver'), 'config', 'bmi088.yaml')

    params_arg = DeclareLaunchArgument(
        'params_file', default_value=default_params,
        description='Path to the bmi088_node parameter file.')

    return LaunchDescription([
        params_arg,
        Node(
            package='bmi088_driver',
            executable='bmi088_node',
            name='bmi088_node',
            output='screen',
            parameters=[LaunchConfiguration('params_file')],
        ),
    ])
