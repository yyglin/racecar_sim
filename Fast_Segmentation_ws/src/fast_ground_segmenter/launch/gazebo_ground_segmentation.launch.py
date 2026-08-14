import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    segmenter_share = get_package_share_directory("fast_ground_segmenter")
    gazebo_share = get_package_share_directory("racecar_gazebo")

    default_params_file = os.path.join(
        segmenter_share, "config", "gazebo_ground_segmenter.yaml"
    )
    default_rviz_config = os.path.join(
        segmenter_share, "config", "gazebo_ground_segmentation.rviz"
    )
    default_world = os.path.join(
        gazebo_share, "worlds", "skidpad.sdf"
    )
    simulation_launch = os.path.join(
        gazebo_share, "launch", "simulation.launch.py"
    )

    params_file = LaunchConfiguration("params_file")
    input_topic = LaunchConfiguration("input_topic")
    world = LaunchConfiguration("world")
    gui = LaunchConfiguration("gui")
    headless = LaunchConfiguration("headless")
    rviz = LaunchConfiguration("rviz")
    rviz_config = LaunchConfiguration("rviz_config")
    paused = LaunchConfiguration("paused")
    verbosity = LaunchConfiguration("verbosity")

    simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(simulation_launch),
        launch_arguments={
            "world": world,
            "gui": gui,
            "headless": headless,
            "rviz": rviz,
            "rviz_config": rviz_config,
            "paused": paused,
            "verbosity": verbosity,
        }.items(),
    )

    ground_segmenter = Node(
        package="fast_ground_segmenter",
        executable="ground_segmenter_node",
        name="ground_segmenter_node",
        output="screen",
        parameters=[
            params_file,
            {
                "input_topic": input_topic,
                "use_sim_time": True,
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
                default_value="/lidar/points",
                description="Gazebo PointCloud2 input topic",
            ),
            DeclareLaunchArgument(
                "world",
                default_value=default_world,
                description="Absolute path to the Gazebo SDF world",
            ),
            DeclareLaunchArgument(
                "gui",
                default_value="true",
                description="Start the Gazebo GUI",
            ),
            DeclareLaunchArgument(
                "headless",
                default_value="false",
                description="Run Gazebo with headless rendering",
            ),
            DeclareLaunchArgument(
                "rviz",
                default_value="true",
                description="Start RViz2 with the G7 segmentation view",
            ),
            DeclareLaunchArgument(
                "rviz_config",
                default_value=default_rviz_config,
                description="Absolute path to an RViz2 configuration file",
            ),
            DeclareLaunchArgument(
                "paused",
                default_value="false",
                description="Start Gazebo paused",
            ),
            DeclareLaunchArgument(
                "verbosity",
                default_value="3",
                description="Gazebo console verbosity from 0 to 4",
            ),
            simulation,
            TimerAction(period=3.0, actions=[ground_segmenter]),
        ]
    )
