import os.path

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node


def generate_launch_description():
    node_list = []

    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time", default_value="true", description="Use simulation clock for bag replay"
    )
    node_list.append(use_sim_time_arg)

    config_dir = os.path.join(get_package_share_directory("mins"), "config", "ZSD-DP010")
    config_path = os.path.join(config_dir, "config.yaml")
    print(f"[MINS] config_path = {config_path}")

    mins_node = Node(
        package="mins",
        executable="subscribe",
        name="mins_subscribe",
        output="screen",
        parameters=[
            {"config_path": config_path},
            {"use_sim_time": LaunchConfiguration("use_sim_time")},
        ],
    )
    node_list.append(mins_node)

    return LaunchDescription(node_list)
