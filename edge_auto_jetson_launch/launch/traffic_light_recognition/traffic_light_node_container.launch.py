# Copyright 2020 Tier IV, Inc. All rights reserved.
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
from launch.actions import DeclareLaunchArgument
from launch.actions import SetLaunchConfiguration
from launch.conditions import IfCondition
from launch.conditions import UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PythonExpression
from launch_ros.actions import LoadComposableNodes
from launch_ros.descriptions import ComposableNode
from launch_ros.parameter_descriptions import ParameterFile


def generate_launch_description():
    launch_arguments = []

    def add_launch_arg(name: str, default_value=None, description=None):
        # a default_value of None is equivalent to not passing that kwarg at all
        launch_arguments.append(
            DeclareLaunchArgument(name, default_value=default_value, description=description)
        )

    add_launch_arg("use_high_accuracy_detection", "True")
    add_launch_arg("high_accuracy_detection_type", "whole_image_detection")
    add_launch_arg("use_image_transport", "False")
    add_launch_arg("input/image", "/sensing/camera/traffic_light/image_raw")
    add_launch_arg("input/camera_info", "/sensing/camera/traffic_light/camera_info")
    add_launch_arg("output/rois", "/perception/traffic_light_recognition/rois")
    add_launch_arg(
        "output/traffic_signals", "/perception/traffic_light_recognition/traffic_signals"
    )
    add_launch_arg(
        "output/car/traffic_signals", "/perception/traffic_light_recognition/car/traffic_signals"
    )
    add_launch_arg(
        "output/pedestrian/traffic_signals",
        "/perception/traffic_light_recognition/pedestrian/traffic_signals",
    )

    add_launch_arg("use_intra_process", "False")
    add_launch_arg("use_multithread", "False")
    add_launch_arg("container", "traffic_light_node_container")

    add_launch_arg("traffic_light_fine_detector_param_path")
    add_launch_arg("car_traffic_light_classifier_param_path")
    add_launch_arg("pedestrian_traffic_light_classifier_param_path")
    add_launch_arg("traffic_light_roi_visualizer_param_path")
    add_launch_arg("whole_image_detector_param_path")

    traffic_light_nodes = [
        ComposableNode(
            package="autoware_traffic_light_classifier",
            plugin="autoware::traffic_light::TrafficLightClassifierNodelet",
            name="car_traffic_light_classifier",
            namespace="classification",
            parameters=[
                ParameterFile(
                    param_file=LaunchConfiguration("car_traffic_light_classifier_param_path"),
                    allow_substs=True,
                ),
                {
                    "build_only": False,
                },
            ],
            remappings=[
                ("~/input/image", LaunchConfiguration("input/image")),
                ("~/input/rois", LaunchConfiguration("output/rois")),
                ("~/output/traffic_signals", "car/traffic_signals"),
            ],
            extra_arguments=[{"use_intra_process_comms": LaunchConfiguration("use_intra_process")}],
        ),
        ComposableNode(
            package="autoware_traffic_light_classifier",
            plugin="autoware::traffic_light::TrafficLightClassifierNodelet",
            name="pedestrian_traffic_light_classifier",
            namespace="classification",
            parameters=[
                ParameterFile(
                    param_file=LaunchConfiguration(
                        "pedestrian_traffic_light_classifier_param_path"
                    ),
                    allow_substs=True,
                ),
                {
                    "build_only": False,
                },
            ],
            remappings=[
                ("~/input/image", LaunchConfiguration("input/image")),
                ("~/input/rois", LaunchConfiguration("output/rois")),
                ("~/output/traffic_signals", "pedestrian/traffic_signals"),
            ],
            extra_arguments=[{"use_intra_process_comms": LaunchConfiguration("use_intra_process")}],
        ),
        ComposableNode(
            package="autoware_traffic_light_visualization",
            plugin="autoware::traffic_light::TrafficLightRoiVisualizerNode",
            name="traffic_light_roi_visualizer",
            parameters=[
                ParameterFile(
                    param_file=LaunchConfiguration("traffic_light_roi_visualizer_param_path"),
                    allow_substs=True,
                ),
                {
                    "use_high_accuracy_detection": LaunchConfiguration(
                        "use_high_accuracy_detection"
                    ),
                },
            ],
            remappings=[
                ("~/input/image", LaunchConfiguration("input/image")),
                ("~/input/rois", LaunchConfiguration("output/rois")),
                ("~/input/rough/rois", "detection/rough/rois"),
                ("~/input/traffic_signals", LaunchConfiguration("output/traffic_signals")),
                ("~/output/image", "debug/rois"),
                ("~/output/image/compressed", "debug/rois/compressed"),
                ("~/output/image/compressedDepth", "debug/rois/compressedDepth"),
                ("~/output/image/theora", "debug/rois/theora"),
            ],
            extra_arguments=[{"use_intra_process_comms": LaunchConfiguration("use_intra_process")}],
        ),
    ]

    traffic_light_node_loader = LoadComposableNodes(
        composable_node_descriptions=traffic_light_nodes,
        target_container=LaunchConfiguration("container"),
    )

    fine_detector_loader = LoadComposableNodes(
        composable_node_descriptions=[
            ComposableNode(
                package="autoware_traffic_light_fine_detector",
                plugin="autoware::traffic_light::TrafficLightFineDetectorNode",
                name="traffic_light_fine_detector",
                namespace="detection",
                parameters=[
                    ParameterFile(
                        param_file=LaunchConfiguration("traffic_light_fine_detector_param_path"),
                        allow_substs=True,
                    ),
                    {
                        "build_only": False,
                    },
                ],
                remappings=[
                    ("~/input/image", LaunchConfiguration("input/image")),
                    ("~/input/rois", "rough/rois"),
                    ("~/expect/rois", "expect/rois"),
                    ("~/output/rois", LaunchConfiguration("output/rois")),
                ],
                extra_arguments=[
                    {"use_intra_process_comms": LaunchConfiguration("use_intra_process")}
                ],
            ),
        ],
        target_container=LaunchConfiguration("container"),
        condition=IfCondition(
            PythonExpression(
                [
                    "'",
                    LaunchConfiguration("high_accuracy_detection_type"),
                    "'",
                    " == 'fine_detection'",
                ]
            )
        ),
    )

    internal_node_name = "traffic_light_whole_image_detector"
    whole_img_detector_loader = LoadComposableNodes(
        composable_node_descriptions=[
            ComposableNode(
                package="autoware_tensorrt_yolox",
                plugin="autoware::tensorrt_yolox::TrtYoloXNode",
                name=internal_node_name,
                namespace="detection",
                parameters=[
                    ParameterFile(
                        param_file=LaunchConfiguration("whole_image_detector_param_path"),
                        allow_substs=True,
                    ),
                    {
                        "build_only": False,
                    },
                ],
                remappings=[
                    ("~/in/image", LaunchConfiguration("input/image")),
                    ("~/out/objects", internal_node_name),
                    ("~/out/image", internal_node_name + "/debug/image"),
                    (
                        "~/out/image/compressed",
                        internal_node_name + "/debug/image/compressed",
                    ),
                    (
                        "~/out/image/compressedDepth",
                        internal_node_name + "/debug/image/compressedDepth",
                    ),
                    ("~/out/image/theora", internal_node_name + "/debug/image/theora"),
                ],
                extra_arguments=[
                    {"use_intra_process_comms": LaunchConfiguration("use_intra_process")}
                ],
            ),
            ComposableNode(
                package="autoware_traffic_light_selector",
                plugin="autoware::traffic_light::TrafficLightSelectorNode",
                name="traffic_light_selector",
                namespace="detection",
                parameters=[],
                remappings=[
                    ("input/detected_rois", internal_node_name),
                    ("input/rough_rois", "rough/rois"),
                    ("input/expect_rois", "expect/rois"),
                    ("input/camera_info", LaunchConfiguration("input/camera_info")),
                    ("output/traffic_rois", LaunchConfiguration("output/rois")),
                ],
                extra_arguments=[
                    {"use_intra_process_comms": LaunchConfiguration("use_intra_process")}
                ],
            ),
            ComposableNode(
                package="autoware_traffic_light_category_merger",
                plugin="autoware::traffic_light::TrafficLightCategoryMergerNode",
                name="traffic_light_category_merger",
                namespace="classification",
                parameters=[],
                remappings=[
                    ("input/car_signals", "car/traffic_signals"),
                    ("input/pedestrian_signals", "pedestrian/traffic_signals"),
                    ("output/traffic_signals", LaunchConfiguration("output/traffic_signals")),
                ],
                extra_arguments=[
                    {"use_intra_process_comms": LaunchConfiguration("use_intra_process")}
                ],
            ),
        ],
        target_container=LaunchConfiguration("container"),
        condition=IfCondition(
            PythonExpression(
                [
                    "'",
                    LaunchConfiguration("high_accuracy_detection_type"),
                    "'",
                    " == 'whole_image_detection' ",
                ]
            )
        ),
    )

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
            *launch_arguments,
            set_container_executable,
            set_container_mt_executable,
            traffic_light_node_loader,
            fine_detector_loader,
            whole_img_detector_loader,
        ]
    )
