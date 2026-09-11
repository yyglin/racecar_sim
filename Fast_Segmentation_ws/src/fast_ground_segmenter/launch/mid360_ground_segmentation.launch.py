import atexit
import ipaddress
import json
import os
import tempfile

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    package_share = FindPackageShare('fast_ground_segmenter')

    default_segmenter_params = PathJoinSubstitution(
        [package_share, 'config', 'mid360_ground_segmenter.yaml']
    )
    default_rviz_config = PathJoinSubstitution(
        [package_share, 'config', 'mid360_ground_segmentation.rviz']
    )

    segmenter_params = LaunchConfiguration('segmenter_params')
    host_ip = LaunchConfiguration('host_ip')
    lidar_ip = LaunchConfiguration('lidar_ip')
    frame_id = LaunchConfiguration('frame_id')
    publish_freq = LaunchConfiguration('publish_freq')
    foxglove = LaunchConfiguration('foxglove')
    foxglove_address = LaunchConfiguration('foxglove_address')
    foxglove_port = LaunchConfiguration('foxglove_port')
    rviz = LaunchConfiguration('rviz')

    def create_livox_driver(context):
        resolved_host_ip = ipaddress.ip_address(host_ip.perform(context))
        resolved_lidar_ip = ipaddress.ip_address(lidar_ip.perform(context))
        if resolved_host_ip.version != 4 or resolved_lidar_ip.version != 4:
            raise RuntimeError('Mid-360 host_ip and lidar_ip must be IPv4 addresses')
        if resolved_lidar_ip not in ipaddress.ip_network(
            f'{resolved_host_ip}/24', strict=False
        ):
            raise RuntimeError('Mid-360 host_ip and lidar_ip must be in the same /24 subnet')

        config = {
            'lidar_summary_info': {'lidar_type': 8},
            'MID360': {
                'lidar_net_info': {
                    'cmd_data_port': 56100,
                    'push_msg_port': 56200,
                    'point_data_port': 56300,
                    'imu_data_port': 56400,
                    'log_data_port': 56500,
                },
                'host_net_info': {
                    'cmd_data_ip': str(resolved_host_ip),
                    'cmd_data_port': 56101,
                    'push_msg_ip': str(resolved_host_ip),
                    'push_msg_port': 56201,
                    'point_data_ip': str(resolved_host_ip),
                    'point_data_port': 56301,
                    'imu_data_ip': str(resolved_host_ip),
                    'imu_data_port': 56401,
                    'log_data_ip': '',
                    'log_data_port': 56501,
                },
            },
            'lidar_configs': [
                {
                    'ip': str(resolved_lidar_ip),
                    'pcl_data_type': 1,
                    'pattern_mode': 0,
                    'extrinsic_parameter': {
                        'roll': 0.0,
                        'pitch': 0.0,
                        'yaw': 0.0,
                        'x': 0,
                        'y': 0,
                        'z': 0,
                    },
                }
            ],
        }

        config_file = tempfile.NamedTemporaryFile(
            mode='w', prefix='mid360_', suffix='.json', delete=False
        )
        with config_file:
            json.dump(config, config_file, indent=2)
        atexit.register(lambda: os.path.exists(config_file.name) and os.unlink(config_file.name))

        return [
            Node(
                package='livox_ros_driver2',
                executable='livox_ros_driver2_node',
                name='livox_lidar_publisher',
                output='screen',
                parameters=[
                    {
                        # ROS 2 supports Livox PointCloud2 (XYZ + intensity + tag,
                        # line and per-point timestamp). PCL safely selects XYZI.
                        'xfer_format': 0,
                        'multi_topic': 0,
                        'data_src': 0,
                        'publish_freq': ParameterValue(publish_freq, value_type=float),
                        'output_data_type': 0,
                        'frame_id': frame_id,
                        'lvx_file_path': '/tmp/livox_test.lvx',
                        'user_config_path': config_file.name,
                        'cmdline_input_bd_code': 'livox0000000001',
                    }
                ],
            )
        ]

    ground_segmenter = Node(
        package='fast_ground_segmenter',
        executable='ground_segmenter_node',
        name='ground_segmenter_node',
        output='screen',
        parameters=[segmenter_params, {'use_sim_time': False}],
    )

    foxglove_bridge = Node(
        package='foxglove_bridge',
        executable='foxglove_bridge',
        name='foxglove_bridge',
        output='screen',
        parameters=[
            {
                'address': foxglove_address,
                'port': ParameterValue(foxglove_port, value_type=int),
                'topic_whitelist': [
                    '^/livox/lidar$',
                    '^/livox/imu$',
                    '^/ground_points$',
                    '^/nonground_points$',
                    '^/debug/filtered_points$',
                    '^/debug/bin_min_points$',
                    '^/tf$',
                    '^/tf_static$',
                ],
            }
        ],
        condition=IfCondition(foxglove),
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='mid360_ground_segmentation_rviz',
        output='screen',
        arguments=['-d', default_rviz_config],
        condition=IfCondition(rviz),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                'segmenter_params',
                default_value=default_segmenter_params,
                description='Ground segmenter parameters for Mid-360',
            ),
            DeclareLaunchArgument(
                'host_ip',
                default_value='192.168.1.50',
                description='Static IPv4 address assigned to the computer NIC',
            ),
            DeclareLaunchArgument(
                'lidar_ip',
                default_value='192.168.1.133',
                description='Mid-360 IPv4 address (default: 192.168.1.1XX)',
            ),
            DeclareLaunchArgument(
                'frame_id',
                default_value='livox_frame',
                description='Frame ID attached to the Livox messages',
            ),
            DeclareLaunchArgument(
                'publish_freq',
                default_value='10.0',
                description='Point cloud aggregation/publish frequency in Hz',
            ),
            DeclareLaunchArgument(
                'foxglove',
                default_value='true',
                description='Start Foxglove Bridge for point-cloud visualization',
            ),
            DeclareLaunchArgument(
                'foxglove_address',
                default_value='127.0.0.1',
                description='Foxglove WebSocket bind address',
            ),
            DeclareLaunchArgument(
                'foxglove_port',
                default_value='8765',
                description='Foxglove WebSocket port',
            ),
            DeclareLaunchArgument(
                'rviz',
                default_value='true',
                description='Start RViz2 with the Mid-360 debug layout',
            ),
            OpaqueFunction(function=create_livox_driver),
            ground_segmenter,
            foxglove_bridge,
            rviz_node,
        ]
    )
