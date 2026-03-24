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
    # GNSS topic monitor
    gnss_topic_monitor = ComposableNode(
        package="autoware_topic_state_monitor",
        plugin="autoware::topic_state_monitor::TopicStateMonitorNode",
        name="topic_state_monitor_gnss_pose",
        parameters=[
            {
                "topic": "/sensing/gnss/pose",
                "topic_type": "geometry_msgs/msg/PoseStamped",
                "best_effort": True,
                "diag_name": "gnss_topic_status",
                "warn_rate": 2.5,
                "error_rate": 0.5,
                "timeout": 5.0,
                "window_size": 10,
            }
        ],
        extra_arguments=[{"use_intra_process_comms": True}],
    )

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

    # Radar topic monitors
    radar_front_center_monitor = ComposableNode(
        package="autoware_topic_state_monitor",
        plugin="autoware::topic_state_monitor::TopicStateMonitorNode",
        name="topic_state_monitor_radar_front_center",
        parameters=[
            {
                "topic": "/sensing/radar/front_center/nebula_packets",
                "topic_type": "nebula_msgs/msg/NebulaPackets",
                "best_effort": True,
                "diag_name": "radar_front_center_topic_status",
                "warn_rate": 20.0,
                "error_rate": 5.0,
                "timeout": 5.0,
                "window_size": 10,
            }
        ],
        extra_arguments=[{"use_intra_process_comms": True}],
    )

    radar_front_left_monitor = ComposableNode(
        package="autoware_topic_state_monitor",
        plugin="autoware::topic_state_monitor::TopicStateMonitorNode",
        name="topic_state_monitor_radar_front_left",
        parameters=[
            {
                "topic": "/sensing/radar/front_left/nebula_packets",
                "topic_type": "nebula_msgs/msg/NebulaPackets",
                "best_effort": True,
                "diag_name": "radar_front_left_topic_status",
                "warn_rate": 20.0,
                "error_rate": 5.0,
                "timeout": 5.0,
                "window_size": 10,
            }
        ],
        extra_arguments=[{"use_intra_process_comms": True}],
    )

    radar_front_right_monitor = ComposableNode(
        package="autoware_topic_state_monitor",
        plugin="autoware::topic_state_monitor::TopicStateMonitorNode",
        name="topic_state_monitor_radar_front_right",
        parameters=[
            {
                "topic": "/sensing/radar/front_right/nebula_packets",
                "topic_type": "nebula_msgs/msg/NebulaPackets",
                "best_effort": True,
                "diag_name": "radar_front_right_topic_status",
                "warn_rate": 20.0,
                "error_rate": 5.0,
                "timeout": 5.0,
                "window_size": 10,
            }
        ],
        extra_arguments=[{"use_intra_process_comms": True}],
    )

    radar_rear_center_monitor = ComposableNode(
        package="autoware_topic_state_monitor",
        plugin="autoware::topic_state_monitor::TopicStateMonitorNode",
        name="topic_state_monitor_radar_rear_center",
        parameters=[
            {
                "topic": "/sensing/radar/rear_center/nebula_packets",
                "topic_type": "nebula_msgs/msg/NebulaPackets",
                "best_effort": True,
                "diag_name": "radar_rear_center_topic_status",
                "warn_rate": 20.0,
                "error_rate": 5.0,
                "timeout": 5.0,
                "window_size": 10,
            }
        ],
        extra_arguments=[{"use_intra_process_comms": True}],
    )

    radar_rear_left_monitor = ComposableNode(
        package="autoware_topic_state_monitor",
        plugin="autoware::topic_state_monitor::TopicStateMonitorNode",
        name="topic_state_monitor_radar_rear_left",
        parameters=[
            {
                "topic": "/sensing/radar/rear_left/nebula_packets",
                "topic_type": "nebula_msgs/msg/NebulaPackets",
                "best_effort": True,
                "diag_name": "radar_rear_left_topic_status",
                "warn_rate": 20.0,
                "error_rate": 5.0,
                "timeout": 5.0,
                "window_size": 10,
            }
        ],
        extra_arguments=[{"use_intra_process_comms": True}],
    )

    radar_rear_right_monitor = ComposableNode(
        package="autoware_topic_state_monitor",
        plugin="autoware::topic_state_monitor::TopicStateMonitorNode",
        name="topic_state_monitor_radar_rear_right",
        parameters=[
            {
                "topic": "/sensing/radar/rear_right/nebula_packets",
                "topic_type": "nebula_msgs/msg/NebulaPackets",
                "best_effort": True,
                "diag_name": "radar_rear_right_topic_status",
                "warn_rate": 20.0,
                "error_rate": 5.0,
                "timeout": 5.0,
                "window_size": 10,
            }
        ],
        extra_arguments=[{"use_intra_process_comms": True}],
    )

    # ComposableNodeContainer to run all ComposableNodes
    container = ComposableNodeContainer(
        name="topic_state_monitor_container",
        namespace="topic_state_monitor",
        package="rclcpp_components",
        executable="component_container",
        composable_node_descriptions=[
            gnss_topic_monitor,
            imu_topic_monitor,
            radar_front_center_monitor,
            radar_front_left_monitor,
            radar_front_right_monitor,
            radar_rear_center_monitor,
            radar_rear_left_monitor,
            radar_rear_right_monitor,
        ],
        output="screen",
    )

    return LaunchDescription([container])
