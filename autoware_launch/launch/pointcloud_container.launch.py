# Copyright 2021 Tier IV, Inc. All rights reserved.
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
from launch.actions import GroupAction
from launch.actions import OpaqueFunction
from launch.actions import SetEnvironmentVariable
from launch.actions import SetLaunchConfiguration
from launch.conditions import IfCondition
from launch.conditions import UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def launch_setup(context, *args, **kwargs):
    agnocast_heaphook_path = LaunchConfiguration("agnocast_heaphook_path").perform(context)
    use_agnocast_str = LaunchConfiguration("use_agnocast").perform(context)
    use_agnocast = use_agnocast_str.lower() == "true"

    glog_component = ComposableNode(
        package="autoware_glog_component",
        plugin="autoware::glog_component::GlogComponent",
        name="glog_component",
        namespace="pointcloud_container",
    )

    container_package = "rclcpp_components"

    pointcloud_container = ComposableNodeContainer(
        name=LaunchConfiguration("container_name"),
        namespace="/",
        package=container_package,
        executable=LaunchConfiguration("container_executable"),
        composable_node_descriptions=[glog_component],
        output="both",
    )

    actions = (
        []
        if not use_agnocast
        else [
            SetEnvironmentVariable(
                name="LD_PRELOAD", value=f"{agnocast_heaphook_path}:{os.getenv('LD_PRELOAD', '')}"
            ),
            SetEnvironmentVariable(name="AGNOCAST_MEMPOOL_SIZE", value="8589934592"),  # 8GB
        ]
    )
    actions.append(pointcloud_container)

    return [GroupAction(actions=actions)]


def generate_launch_description():
    def add_launch_arg(name: str, default_value=None):
        return DeclareLaunchArgument(name, default_value=default_value)

    set_container_executable = SetLaunchConfiguration(
        "container_executable",
        "component_container",
        condition=UnlessCondition(LaunchConfiguration("use_multithread")),
    )

    set_container_mt_executable = SetLaunchConfiguration(
        "container_executable",
        "component_container_mt",
        condition=IfCondition(LaunchConfiguration("use_multithread")),
    )

    return LaunchDescription(
        [
            add_launch_arg("agnocast_heaphook_path"),
            add_launch_arg("use_agnocast", "false"),
            add_launch_arg("use_multithread", "false"),
            add_launch_arg("container_name", "pointcloud_container"),
            set_container_executable,
            set_container_mt_executable,
            OpaqueFunction(function=launch_setup),
        ]
    )
