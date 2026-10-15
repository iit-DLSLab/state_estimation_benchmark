"""Safe offline replay of a recorded Go2 LowState bag.

Two modes (argument `mode`):

  deterministic (default)
      seb_replay reads the bag file directly and feeds the online pipeline with the
      RECORDED reception timestamps. Nothing is published on /lowstate, so the replay
      cannot reach a robot or a controller. `replay_mode:=lossless` processes every
      sample (use it to profile execution time, e.g. smoother window sizes);
      `replay_mode:=realtime` injects at the recorded pace (overload/backlog behave
      as on the robot).

  ros_play
      `ros2 bag play` republishes the LowState topic REMAPPED to /seb_replay/lowstate
      (never /lowstate) and the online node subscribes to it; stamps are the live
      reception times of the replay, i.e. the full ROS transport path is exercised.

Examples:
  ros2 launch seb_online go2_bag_replay.launch.py bag:=/path/to/bag estimator:=iekf
  ros2 launch seb_online go2_bag_replay.launch.py bag:=/path/to/bag estimator:=invariant_smoother window_size:=10
  ros2 launch seb_online go2_bag_replay.launch.py bag:=/path/to/bag mode:=ros_play estimator:=muse
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction, RegisterEventHandler, Shutdown
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

ESTIMATORS = ("muse", "iekf", "invariant_smoother")
REPLAY_TOPIC = "/seb_replay/lowstate"


def _setup(context):
    bag = os.path.expanduser(LaunchConfiguration("bag").perform(context))
    if not bag:
        raise RuntimeError("bag:=<path to rosbag2 directory> is required")
    estimator = LaunchConfiguration("estimator").perform(context)
    if estimator not in ESTIMATORS:
        raise RuntimeError(f"estimator must be one of {ESTIMATORS}, got '{estimator}'")
    mode = LaunchConfiguration("mode").perform(context)
    topic = LaunchConfiguration("topic").perform(context)
    config = LaunchConfiguration("config").perform(context)

    overrides = {
        "estimator.type": estimator,
        "log.enabled": LaunchConfiguration("log").perform(context).lower() == "true",
        "log.directory": LaunchConfiguration("log_dir").perform(context),
        "log.run_name": LaunchConfiguration("run_name").perform(context),
        "output.topic": "/seb_replay/state",
    }
    window_size = LaunchConfiguration("window_size").perform(context)
    if window_size:
        overrides["smoother.window_size"] = int(window_size)

    if mode == "deterministic":
        overrides.update(
            {
                "replay.bag_path": bag,
                "replay.topic": topic,
                "replay.mode": LaunchConfiguration("replay_mode").perform(context),
                "replay.rate": float(LaunchConfiguration("rate").perform(context)),
                "replay.publish": LaunchConfiguration("publish").perform(context).lower() == "true",
            }
        )
        replay = Node(
            package="seb_online",
            executable="seb_replay",
            name="seb_replay",
            output="screen",
            parameters=[config, overrides],
        )
        return [replay, RegisterEventHandler(OnProcessExit(target_action=replay, on_exit=[Shutdown()]))]

    if mode == "ros_play":
        overrides["input.lowstate_topic"] = REPLAY_TOPIC
        node = Node(
            package="seb_online",
            executable="online_estimator_node",
            name="seb_online_estimator",
            output="screen",
            parameters=[config, overrides],
        )
        play = ExecuteProcess(
            cmd=[
                "ros2", "bag", "play", bag,
                "--topics", topic,
                "--remap", f"{topic}:={REPLAY_TOPIC}",
                "--rate", LaunchConfiguration("rate").perform(context),
            ],
            output="screen",
        )
        return [node, play, RegisterEventHandler(OnProcessExit(target_action=play, on_exit=[Shutdown()]))]

    raise RuntimeError("mode must be deterministic or ros_play")


def generate_launch_description():
    default_config = os.path.join(get_package_share_directory("seb_online"), "config", "go2_online.yaml")
    return LaunchDescription(
        [
            DeclareLaunchArgument("bag", description="rosbag2 directory containing unitree_go/LowState"),
            DeclareLaunchArgument("estimator", default_value="muse", description="muse | iekf | invariant_smoother"),
            DeclareLaunchArgument("mode", default_value="deterministic", description="deterministic | ros_play"),
            DeclareLaunchArgument("replay_mode", default_value="lossless", description="lossless | realtime (deterministic mode)"),
            DeclareLaunchArgument("rate", default_value="1.0"),
            DeclareLaunchArgument("topic", default_value="/lowstate", description="LowState topic name inside the bag"),
            DeclareLaunchArgument("window_size", default_value="", description="Smoother window size override"),
            DeclareLaunchArgument("publish", default_value="false", description="Publish states (deterministic mode)"),
            DeclareLaunchArgument("config", default_value=default_config),
            DeclareLaunchArgument("log", default_value="true"),
            DeclareLaunchArgument("log_dir", default_value="~/.ros/seb_online"),
            DeclareLaunchArgument("run_name", default_value=""),
            OpaqueFunction(function=_setup),
        ]
    )
