from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    default_params_file = PathJoinSubstitution(
        [
            FindPackageShare("fast_ground_segmenter"),
            "config",
            "ground_segmenter.yaml",
        ]
    )

    params_file = LaunchConfiguration("params_file")
    input_topic = LaunchConfiguration("input_topic")
    use_sim_time = LaunchConfiguration("use_sim_time")

    ground_segmenter = Node(
        package="fast_ground_segmenter",
        executable="ground_segmenter_node",
        name="ground_segmenter_node",
        output="screen",
        parameters=[
            params_file,
            {
                "input_topic": input_topic,
                "use_sim_time": use_sim_time,
            },
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "params_file",
                default_value=default_params_file,
                description="Ground segmenter parameter file",
            ),
            DeclareLaunchArgument(
                "input_topic",
                default_value="/sensing/lidar/top/rectified/pointcloud",
                description="Input PointCloud2 topic",
            ),
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="false",
                description="Use the simulation clock",
            ),
            ground_segmenter,
        ]
    )
