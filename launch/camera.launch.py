from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    return LaunchDescription([
        Node(
            package='realsense2_camera',
            executable='realsense2_camera_node',
            output='screen',
            parameters=[
                {
                    'device_type': 'd435i',
                    'initial_reset': False,

                    'enable_color': True,
                    'enable_depth': False,
                    'enable_infra1': False,
                    'enable_infra2': False,

                    'rgb_camera.color_profile': '640x480x15',
                    'depth_module.depth_profile': '640x480x15',

                    'align_depth.enable': True,
                    'enable_sync': True,

                    'pointcloud__neon_.enable': False,
                    'pointcloud__neon_.ordered_pc': False,
                    'pointcloud__neon_.allow_no_texture_points': True,
                    'pointcloud__neon_.pointcloud_qos': 'SENSOR_DATA',
                    'pointcloud__neon_.stream_filter': 2,
                    'pointcloud__neon_.stream_index_filter': 0,

                    'depth_module.laser_power': 360.0,
                    'depth_module.emitter_always_on': False,

                    'publish_tf': True,
                    'tf_publish_rate': 10.0,

                    'enable_gyro': True,
                    'enable_accel': True,
                    'unite_imu_method': 2,
                    'hold_back_imu_for_frames': False,
                    'gyro_fps': 200,
                    'accel_fps': 63,
                    'linear_accel_cov': 0.01,
                    'angular_velocity_cov': 0.01,
                }
            ]
        )
    ])
