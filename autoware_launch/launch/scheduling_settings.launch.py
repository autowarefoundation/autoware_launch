# Copyright 2025 Tier IV, Inc. All rights reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

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
