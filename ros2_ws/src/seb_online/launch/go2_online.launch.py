"""Online state estimation on the Unitree Go2 (monitoring only).

Runs exactly ONE estimator, selected with `estimator:=muse|iekf|invariant_smoother`.
The node subscribes to unitree_go/LowState and publishes seb_interfaces/EstimatorState;
it has no actuation interface.

Examples:
  ros2 launch seb_online go2_online.launch.py estimator:=muse
  ros2 launch seb_online go2_online.launch.py estimator:=invariant_smoother window_size:=5
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

ESTIMATORS = ("muse", "iekf", "invariant_smoother")


def _setup(context):
    estimator = LaunchConfiguration("estimator").perform(context)
    if estimator not in ESTIMATORS:
        raise RuntimeError(f"estimator must be one of {ESTIMATORS}, got '{estimator}'")

    overrides = {
        "estimator.type": estimator,
        "input.lowstate_topic": LaunchConfiguration("lowstate_topic").perform(context),
        "log.enabled": LaunchConfiguration("log").perform(context).lower() == "true",
        "log.directory": LaunchConfiguration("log_dir").perform(context),
        "log.run_name": LaunchConfiguration("run_name").perform(context),
    }
    window_size = LaunchConfiguration("window_size").perform(context)
    if window_size:
        overrides["smoother.window_size"] = int(window_size)

    return [
        Node(
            package="seb_online",
            executable="online_estimator_node",
            name="seb_online_estimator",
            output="screen",
            parameters=[LaunchConfiguration("config").perform(context), overrides],
            emulate_tty=True,
        )
    ]


def generate_launch_description():
    default_config = os.path.join(get_package_share_directory("seb_online"), "config", "go2_online.yaml")
    return LaunchDescription(
        [
            DeclareLaunchArgument("estimator", default_value="muse", description="muse | iekf | invariant_smoother"),
            DeclareLaunchArgument("config", default_value=default_config, description="Parameter file"),
            DeclareLaunchArgument("lowstate_topic", default_value="/lowstate"),
            DeclareLaunchArgument("window_size", default_value="", description="Smoother window size override"),
            DeclareLaunchArgument("log", default_value="true", description="Write per-sample CSV logs"),
            DeclareLaunchArgument("log_dir", default_value="~/.ros/seb_online"),
            DeclareLaunchArgument("run_name", default_value="", description="Log sub-directory (default: date_estimator)"),
            OpaqueFunction(function=_setup),
        ]
    )
