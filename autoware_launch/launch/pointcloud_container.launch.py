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

use_agnocast = os.getenv("ENABLE_AGNOCAST") == "1"


def launch_setup(context, *args, **kwargs):
    agnocast_heaphook_path = LaunchConfiguration("agnocast_heaphook_path").perform(context)

    # Debug: Print current LD_PRELOAD value
    import sys
    current_ld_preload = os.getenv('LD_PRELOAD', 'NOT SET')
    print(f"[pointcloud_container] Current LD_PRELOAD: {current_ld_preload}", file=sys.stderr)
    print(f"[pointcloud_container] agnocast_heaphook_path: {agnocast_heaphook_path}", file=sys.stderr)
    print(f"[pointcloud_container] use_agnocast: {use_agnocast}", file=sys.stderr)

    glog_component = ComposableNode(
        package="autoware_glog_component",
        plugin="autoware::glog_component::GlogComponent",
        name="glog_component",
        namespace="pointcloud_container",
    )

    container_package = "agnocastlib" if use_agnocast else "rclcpp_components"

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

    container_exec = "agnocast_component_container" if use_agnocast else "component_container"
    container_exec_mt = (
        "agnocast_component_container_mt" if use_agnocast else "component_container_mt"
    )

    set_container_executable = SetLaunchConfiguration(
        "container_executable",
        container_exec,
        condition=UnlessCondition(LaunchConfiguration("use_multithread")),
    )

    set_container_mt_executable = SetLaunchConfiguration(
        "container_executable",
        container_exec_mt,
        condition=IfCondition(LaunchConfiguration("use_multithread")),
    )

    return LaunchDescription(
        [
            add_launch_arg("agnocast_heaphook_path"),
            add_launch_arg("use_multithread", "false"),
            add_launch_arg("container_name", "pointcloud_container"),
            set_container_executable,
            set_container_mt_executable,
            OpaqueFunction(function=launch_setup),
        ]
    )
