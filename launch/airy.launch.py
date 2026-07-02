from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    lidar_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=[
            '0.0', '0.0', '0.0',
            '0.482978', '0.234973', '0.758513', '0.369023',
            'base_link', 'rslidar'
        ]
    )

    robosense_airy = Node(
        namespace='rslidar_sdk',
        package='rslidar_sdk',
        executable='rslidar_sdk_node',
        output='screen',
    )


    return LaunchDescription([
        lidar_tf,
        robosense_airy,
    ])
