from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare

from pathlib import Path


def generate_launch_description():

    pkg_share = FindPackageShare('open_3d_scanner')

    camera_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            pkg_share,
            '/launch/camera.launch.py'
        ])
    )

    lidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            pkg_share,
            '/launch/airy.launch.py'
        ])
    )

    colorizer_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            pkg_share,
            '/launch/colorizer.launch.py'
        ])
    )

    return LaunchDescription([
        camera_launch,
        lidar_launch,
        colorizer_launch,
    ])
