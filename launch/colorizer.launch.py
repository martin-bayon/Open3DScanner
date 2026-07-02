from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

import os
import json


def generate_launch_description():

    pkg_share = get_package_share_directory('open_3d_scanner')

    default_config = os.path.join(pkg_share, 'config', 'colorizer.yaml')
    default_calib = os.path.join(pkg_share, 'config', 'calib.json')

    config_file = LaunchConfiguration('config_file')
    calib_file = LaunchConfiguration('calib_file')
    publish_tf = LaunchConfiguration('publish_tf')

    colorizer_node = Node(
        package='open_3d_scanner',
        executable='colorize_pointcloud_node',
        name='colorize_pointcloud_node',
        output='screen',
        parameters=[
            config_file,
            {
                'calib_json_path': calib_file,
            }
        ],
        remappings=[
            ('/ouster/points', '/ouster/points'),
            ('/camera/camera/color/image_raw', '/camera/camera/color/image_raw'),
            ('/camera/camera/color/camera_info', '/camera/camera/color/camera_info'),
        ]
    )

    with open(default_calib, 'r') as f:
        calib = json.load(f)

    x, y, z, qx, qy, qz, qw = calib['T_lidar_camera']

    static_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='lidar_to_camera_static_tf',
        output='screen',
        arguments=[
            str(x), str(y), str(z),
            str(qx), str(qy), str(qz), str(qw),
            'rslidar',
            'camera_color_optical_frame'
        ],
        condition=None
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'config_file',
            default_value=default_config,
            description='Path to colorizer YAML config file'
        ),
        DeclareLaunchArgument(
            'calib_file',
            default_value=default_calib,
            description='Path to calibration JSON file'
        ),
        DeclareLaunchArgument(
            'publish_tf',
            default_value='true',
            description='Whether to publish static TF from calib.json'
        ),

        colorizer_node,
        static_tf_node,
    ])
