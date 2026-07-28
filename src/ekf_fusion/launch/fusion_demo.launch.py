"""Convenience bringup: IMU driver + GNSS driver + EKF fusion, one launch."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description() -> LaunchDescription:
    def include(package: str, launch_file: str) -> IncludeLaunchDescription:
        path = os.path.join(
            get_package_share_directory(package), 'launch', launch_file)
        return IncludeLaunchDescription(PythonLaunchDescriptionSource(path))

    return LaunchDescription([
        include('bmi088_driver', 'bmi088.launch.py'),
        include('gnss_driver', 'gnss.launch.py'),
        include('ekf_fusion', 'ekf_fusion.launch.py'),
    ])
