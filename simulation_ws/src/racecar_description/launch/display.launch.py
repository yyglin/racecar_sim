from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package_share = Path(get_package_share_directory("racecar_description"))
    urdf_path = package_share / "urdf" / "racecar.urdf"
    rviz_config_path = package_share / "rviz" / "racecar.rviz"

    robot_description = urdf_path.read_text(encoding="utf-8")

    use_sim_time = LaunchConfiguration("use_sim_time")
    joint_state_gui = LaunchConfiguration("joint_state_gui")
    rviz = LaunchConfiguration("rviz")

    joint_state_publisher = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        name="joint_state_publisher",
        output="screen",
        parameters=[{"use_sim_time": use_sim_time}],
        condition=UnlessCondition(joint_state_gui),
    )

    joint_state_publisher_gui = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        name="joint_state_publisher_gui",
        output="screen",
        parameters=[{"use_sim_time": use_sim_time}],
        condition=IfCondition(joint_state_gui),
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[
            {
                "robot_description": robot_description,
                "use_sim_time": use_sim_time,
            }
        ],
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", str(rviz_config_path)],
        parameters=[{"use_sim_time": use_sim_time}],
        condition=IfCondition(rviz),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="false",
                description="Use the simulation clock when true",
            ),
            DeclareLaunchArgument(
                "joint_state_gui",
                default_value="true",
                description="Use the joint-state slider GUI when true",
            ),
            DeclareLaunchArgument(
                "rviz",
                default_value="true",
                description="Start RViz2 when true",
            ),
            joint_state_publisher,
            joint_state_publisher_gui,
            robot_state_publisher,
            rviz_node,
        ]
    )
