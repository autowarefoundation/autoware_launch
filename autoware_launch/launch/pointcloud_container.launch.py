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

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import GroupAction
from launch.actions import IncludeLaunchDescription
from launch.actions import OpaqueFunction
from launch.actions import SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer


def launch_setup(context, *args, **kwargs):
    ld_preload_value = context.launch_configurations.get("ld_preload_value", "")
    container_package = context.launch_configurations.get("container_package", "rclcpp_components")
    container_executable = context.launch_configurations.get(
        "container_executable", "component_container"
    )

    pointcloud_container = ComposableNodeContainer(
        name=LaunchConfiguration("container_name"),
        namespace="/",
        package=container_package,
        executable=container_executable,
        composable_node_descriptions=[],
        output="both",
    )

    actions = []
    if ld_preload_value:
        actions.append(SetEnvironmentVariable(name="LD_PRELOAD", value=ld_preload_value))
    actions.append(pointcloud_container)

    return [GroupAction(actions=actions)]


def generate_launch_description():
    def add_launch_arg(name: str, default_value=None):
        return DeclareLaunchArgument(name, default_value=default_value)

    agnocast_env_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            get_package_share_directory("autoware_agnocast_wrapper")
            + "/launch/agnocast_env.launch.py"
        ),
        launch_arguments={
            "use_multithread": LaunchConfiguration("use_multithread"),
        }.items(),
    )

    return LaunchDescription(
        [
            add_launch_arg("use_multithread", "false"),
            add_launch_arg("container_name", "pointcloud_container"),
            agnocast_env_launch,
            OpaqueFunction(function=launch_setup),
        ],
    )
