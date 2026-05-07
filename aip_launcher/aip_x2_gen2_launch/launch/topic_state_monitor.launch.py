# Copyright 2024 Tier IV, Inc. All rights reserved.
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

from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    # IMU topic monitor
    imu_topic_monitor = ComposableNode(
        package="autoware_topic_state_monitor",
        plugin="autoware::topic_state_monitor::TopicStateMonitorNode",
        name="topic_state_monitor_imu_data",
        parameters=[
            {
                "topic": "/sensing/imu/imu_data",
                "topic_type": "sensor_msgs/msg/Imu",
                "best_effort": True,
                "diag_name": "imu_topic_status",
                "warn_rate": 5.0,
                "error_rate": 1.0,
                "timeout": 5.0,
                "window_size": 10,
            }
        ],
        extra_arguments=[{"use_intra_process_comms": True}],
    )

    # ComposableNodeContainer to run enabled ComposableNodes
    container = ComposableNodeContainer(
        name="topic_state_monitor_container",
        namespace="topic_state_monitor",
        package="rclcpp_components",
        executable="component_container",
        composable_node_descriptions=[
            imu_topic_monitor,
        ],
        output="screen",
    )

    return LaunchDescription([container])
