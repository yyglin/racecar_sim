from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
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
    bag_path = LaunchConfiguration("bag_path")
    input_topic = LaunchConfiguration("input_topic")
    play_rate = LaunchConfiguration("play_rate")

    declare_params_file =DeclareLaunchArgument("params_file", 
                                               default_value=default_params_file,
                                               description="Ground segmenter parameter file",)


    declare_bag_path = DeclareLaunchArgument("bag_path",
                                             default_value="/data/rosbag2_2022_04_14-trucks",
                                             description="Path to the ROS 2 bag",)

    declare_input_topic = DeclareLaunchArgument("input_topic",
                                                default_value="/sensing/lidar/top/rectified/pointcloud",
                                                description="Input PointCloud topic",)

    declare_play_rate = DeclareLaunchArgument("play_rate",
                                              default_value="1.0",
                                              description="ROS bag playback rate",)

    ground_segmenter_node = Node(package="fast_ground_segmenter",
                                 executable="ground_segmenter_node",
                                 name="ground_segmenter_node",
                                 output="screen",
                                 parameters=[params_file, {
                                     "input_topic": input_topic,
                                     "use_sim_time": True,
                                 },
                                ],
                            )

    bag_player = ExecuteProcess(
        cmd=[
            "ros2",
            "bag",
            "play",
            bag_path,
            "--loop",
            "--clock",
            "100.0",
            "--rate",
            play_rate,
        ],
        output="screen",
    )

    return LaunchDescription(
        [
            declare_params_file,
            declare_bag_path,
            declare_input_topic,
            declare_play_rate,
            ground_segmenter_node,
            bag_player,
        ]
    )