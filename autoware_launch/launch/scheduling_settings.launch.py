import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import ExecuteProcess
from launch.actions import OpaqueFunction
from launch.substitutions import LaunchConfiguration


def launch_setup(context, *args, **kwargs):
    script_file = LaunchConfiguration("scheduling_settings_script_file").perform(context)
    yaml_file = LaunchConfiguration("scheduling_settings_autoware_yaml_file").perform(context)

    action = []
    if os.path.exists(script_file):
        action.append(
            ExecuteProcess(
                cmd=["python3", script_file, "-s", "-o", "-e", yaml_file],
                output="screen",
                name="scheduling_settings_for_autoware",
            )
        )
    return action


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("scheduling_settings_script_file"),
            DeclareLaunchArgument("scheduling_settings_autoware_yaml_file"),
            OpaqueFunction(function=launch_setup),
        ]
    )
