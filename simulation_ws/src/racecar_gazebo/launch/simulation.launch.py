import os
import shlex

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    LogInfo,
    OpaqueFunction,
    SetEnvironmentVariable,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable, LaunchConfiguration
from launch_ros.actions import Node


def _parse_bool(value, argument_name):
    normalized = value.strip().lower()
    if normalized in {"1", "true", "yes", "on"}:
        return True
    if normalized in {"0", "false", "no", "off"}:
        return False
    raise RuntimeError(
        f"Launch argument '{argument_name}' must be a boolean, got '{value}'"
    )


def _launch_gazebo(context):
    world = LaunchConfiguration("world").perform(context)
    gui = _parse_bool(LaunchConfiguration("gui").perform(context), "gui")
    headless = _parse_bool(
        LaunchConfiguration("headless").perform(context), "headless"
    )
    paused = _parse_bool(
        LaunchConfiguration("paused").perform(context), "paused"
    )
    verbosity = LaunchConfiguration("verbosity").perform(context)

    gz_arguments = ["-v", verbosity]

    if headless or not gui:
        gz_arguments.extend(["-s", "--headless-rendering"])

    if not paused:
        gz_arguments.append("-r")

    gz_arguments.append(world)
    gz_args = " ".join(shlex.quote(argument) for argument in gz_arguments)

    ros_gz_sim_share = get_package_share_directory("ros_gz_sim")
    return [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(ros_gz_sim_share, "launch", "gz_sim.launch.py")
            ),
            launch_arguments={
                "gz_args": gz_args,
                "on_exit_shutdown": "true",
            }.items(),
        )
    ]


def generate_launch_description():
    racecar_description_share = get_package_share_directory(
        "racecar_description"
    )
    racecar_gazebo_share = get_package_share_directory("racecar_gazebo")

    default_world = os.path.join(
        racecar_gazebo_share, "worlds", "flat_cones.sdf"
    )
    bridge_config = os.path.join(
        racecar_gazebo_share, "config", "bridge.yaml"
    )
    model_path = os.path.join(racecar_description_share, "models")

    world = LaunchConfiguration("world")
    gui = LaunchConfiguration("gui")
    headless = LaunchConfiguration("headless")
    rviz = LaunchConfiguration("rviz")
    paused = LaunchConfiguration("paused")
    verbosity = LaunchConfiguration("verbosity")

    bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="ros_gz_bridge",
        output="screen",
        parameters=[
            {
                "config_file": bridge_config,
                "use_sim_time": True,
            }
        ],
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        parameters=[{"use_sim_time": True}],
        condition=IfCondition(rviz),
    )

    return LaunchDescription(
        [
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
                description="Run only the Gazebo server with headless rendering",
            ),
            DeclareLaunchArgument(
                "rviz",
                default_value="false",
                description="Start RViz2 without a project-specific config",
            ),
            DeclareLaunchArgument(
                "paused",
                default_value="false",
                description="Start Gazebo with simulation paused",
            ),
            DeclareLaunchArgument(
                "verbosity",
                default_value="3",
                description="Gazebo console verbosity from 0 to 4",
            ),
            SetEnvironmentVariable(
                name="GZ_SIM_RESOURCE_PATH",
                value=[
                    model_path,
                    os.pathsep,
                    EnvironmentVariable(
                        "GZ_SIM_RESOURCE_PATH", default_value=""
                    ),
                ],
            ),
            LogInfo(
                msg=[
                    "Starting racecar simulation: world=",
                    world,
                    ", gui=",
                    gui,
                    ", headless=",
                    headless,
                    ", paused=",
                    paused,
                    ", verbosity=",
                    verbosity,
                ]
            ),
            OpaqueFunction(function=_launch_gazebo),
            bridge,
            rviz_node,
        ]
    )
