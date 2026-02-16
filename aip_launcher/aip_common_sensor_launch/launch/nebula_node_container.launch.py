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

from ament_index_python.packages import get_package_share_directory
import launch
from launch.actions import DeclareLaunchArgument
from launch.actions import OpaqueFunction
from launch.actions import SetLaunchConfiguration
from launch.conditions import IfCondition
from launch.conditions import UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.actions import LoadComposableNodes
from launch_ros.descriptions import ComposableNode
from launch_ros.parameter_descriptions import ParameterFile
from launch_ros.substitutions import FindPackageShare
import yaml


def get_lidar_make(sensor_name):
    if sensor_name[:6].lower() == "pandar":
        return "Hesai", ".csv"
    elif sensor_name[:3].lower() in ["hdl", "vlp", "vls"]:
        return "Velodyne", ".yaml"
    elif sensor_name.lower() in ["helios", "bpearl"]:
        return "Robosense", None
    return "unrecognized_sensor_model"


def get_vehicle_info(context):
    # TODO(TIER IV): Use Parameter Substitution after we drop Galactic support
    # https://github.com/ros2/launch_ros/blob/master/launch_ros/launch_ros/substitutions/parameter.py
    gp = context.launch_configurations.get("ros_params", {})
    if not gp:
        gp = dict(context.launch_configurations.get("global_params", {}))
    p = {}
    p["vehicle_length"] = gp["front_overhang"] + gp["wheel_base"] + gp["rear_overhang"]
    p["vehicle_width"] = gp["wheel_tread"] + gp["left_overhang"] + gp["right_overhang"]
    p["min_longitudinal_offset"] = -gp["rear_overhang"]
    p["max_longitudinal_offset"] = gp["front_overhang"] + gp["wheel_base"]
    p["min_lateral_offset"] = -(gp["wheel_tread"] / 2.0 + gp["right_overhang"])
    p["max_lateral_offset"] = gp["wheel_tread"] / 2.0 + gp["left_overhang"]
    p["min_height_offset"] = 0.0
    p["max_height_offset"] = gp["vehicle_height"]
    return p


def load_composable_node_param(context, param_path):
    with open(LaunchConfiguration(param_path).perform(context), "r") as f:
        return yaml.safe_load(f)["/**"]["ros__parameters"]


def create_parameter_dict(*args):
    result = {}
    for x in args:
        result[x] = LaunchConfiguration(x)
    return result


def make_common_nodes(context):
    if UnlessCondition(LaunchConfiguration("use_shared_container")).evaluate(context):
        return [
            ComposableNode(
                package="autoware_glog_component",
                plugin="autoware::glog_component::GlogComponent",
                name="glog_component",
            )
        ]

    return []


def make_nebula_nodes(context):
    # Model and make
    sensor_model = LaunchConfiguration("sensor_model").perform(context)
    sensor_make, sensor_extension = get_lidar_make(sensor_model)
    nebula_decoders_share_dir = get_package_share_directory("nebula_decoders")

    # Calibration file
    if sensor_extension is not None:  # Velodyne and Hesai
        sensor_calib_fp = os.path.join(
            nebula_decoders_share_dir,
            "calibration",
            sensor_make.lower(),
            sensor_model + sensor_extension,
        )
        assert os.path.exists(
            sensor_calib_fp
        ), "Sensor calib file under calibration/ was not found: {}".format(sensor_calib_fp)
    else:  # Robosense
        sensor_calib_fp = ""

    return [
        ComposableNode(
            package="nebula_ros",
            plugin=sensor_make + "RosWrapper",
            name=sensor_make.lower() + "_ros_wrapper_node",
            parameters=[
                ParameterFile(
                    LaunchConfiguration("nebula_common_config_file").perform(context),
                    allow_substs=True,
                ),
                {
                    "calibration_file": sensor_calib_fp,
                    "sensor_model": sensor_model,
                    "launch_hw": LaunchConfiguration("launch_driver"),
                    "calibration_download_enabled": True,
                    **create_parameter_dict(
                        "host_ip",
                        "sensor_ip",
                        "multicast_ip",
                        "advanced_diagnostics",
                        "data_port",
                        "return_mode",
                        "min_range",
                        "max_range",
                        "cut_angle",
                        "sync_angle",
                        "frame_id",
                        "retry_hw",
                        "scan_phase",
                        "dual_return_distance_threshold",
                        "rotation_speed",
                        "cloud_min_angle",
                        "cloud_max_angle",
                        "gnss_port",
                        "packet_mtu_size",
                        "setup_sensor",
                        "udp_only",
                        "hires_mode",
                        "diag_span",
                        "diagnostics.packet_loss.error_threshold",
                        "udp_socket_receive_buffer_size_bytes",
                    ),
                },
            ],
            remappings=[
                # cSpell:ignore knzo25
                # TODO(knzo25): fix the remapping once nebula gets updated
                ("pandar_points", "pointcloud_raw_ex"),
                ("velodyne_points", "pointcloud_raw_ex"),
            ],
            extra_arguments=[{"use_intra_process_comms": LaunchConfiguration("use_intra_process")}],
        )
    ]


def make_cuda_preprocessor_nodes(context):
    # Vehicle parameters
    vehicle_info = get_vehicle_info(context)
    mirror_info = load_composable_node_param(context, "vehicle_mirror_param_file")

    # Pointcloud preprocessor parameters
    distortion_corrector_node_param = ParameterFile(
        param_file=LaunchConfiguration("distortion_correction_node_param_path").perform(context),
        allow_substs=True,
    )
    ring_outlier_filter_node_param = ParameterFile(
        param_file=LaunchConfiguration("ring_outlier_filter_node_param_path").perform(context),
        allow_substs=True,
    )

    preprocessor_parameters = {}
    preprocessor_parameters["crop_box.min_x"] = [
        vehicle_info["min_longitudinal_offset"],
        mirror_info["min_longitudinal_offset"],
    ]
    preprocessor_parameters["crop_box.max_x"] = [
        vehicle_info["max_longitudinal_offset"],
        mirror_info["max_longitudinal_offset"],
    ]
    preprocessor_parameters["crop_box.min_y"] = [
        vehicle_info["min_lateral_offset"],
        mirror_info["min_lateral_offset"],
    ]
    preprocessor_parameters["crop_box.max_y"] = [
        vehicle_info["max_lateral_offset"],
        mirror_info["max_lateral_offset"],
    ]
    preprocessor_parameters["crop_box.min_z"] = [
        vehicle_info["min_height_offset"],
        mirror_info["min_height_offset"],
    ]
    preprocessor_parameters["crop_box.max_z"] = [
        vehicle_info["max_height_offset"],
        mirror_info["max_height_offset"],
    ]

    return [
        ComposableNode(
            package="autoware_cuda_pointcloud_preprocessor",
            plugin="autoware::cuda_pointcloud_preprocessor::CudaPointcloudPreprocessorNode",
            name="cuda_pointcloud_preprocessor_node",
            parameters=[
                preprocessor_parameters,
                distortion_corrector_node_param,
                ring_outlier_filter_node_param,
            ],
            remappings=[
                ("~/input/pointcloud", "pointcloud_raw_ex"),
                (
                    "~/input/twist",
                    "/sensing/vehicle_velocity_converter/twist_with_covariance",
                ),
                ("~/input/imu", "/sensing/imu/imu_data"),
                ("~/output/pointcloud", "pointcloud_before_sync"),
                ("~/output/pointcloud/cuda", "pointcloud_before_sync/cuda"),
            ],
            # The whole node can not set use_intra_process due to type negotiation internal topics
            # extra_arguments=[{"use_intra_process_comms": LaunchConfiguration("use_intra_process")}],
        )
    ]


def make_preprocessor_nodes(context):
    # Vehicle parameters
    vehicle_info = get_vehicle_info(context)
    mirror_info = load_composable_node_param(context, "vehicle_mirror_param_file")

    # Pointcloud preprocessor parameters
    distortion_corrector_node_param = ParameterFile(
        param_file=LaunchConfiguration("distortion_correction_node_param_path").perform(context),
        allow_substs=True,
    )
    ring_outlier_filter_node_param = ParameterFile(
        param_file=LaunchConfiguration("ring_outlier_filter_node_param_path").perform(context),
        allow_substs=True,
    )

    cropbox_parameters = create_parameter_dict("input_frame", "output_frame")
    cropbox_parameters["negative"] = True
    cropbox_parameters["processing_time_threshold_sec"] = 0.01

    vehicle_info = get_vehicle_info(context)
    cropbox_parameters["min_x"] = vehicle_info["min_longitudinal_offset"]
    cropbox_parameters["max_x"] = vehicle_info["max_longitudinal_offset"]
    cropbox_parameters["min_y"] = vehicle_info["min_lateral_offset"]
    cropbox_parameters["max_y"] = vehicle_info["max_lateral_offset"]
    cropbox_parameters["min_z"] = vehicle_info["min_height_offset"]
    cropbox_parameters["max_z"] = vehicle_info["max_height_offset"]

    nodes = []

    nodes.append(
        ComposableNode(
            package="autoware_pointcloud_preprocessor",
            plugin="autoware::pointcloud_preprocessor::CropBoxFilterComponent",
            name="crop_box_filter_self",
            remappings=[
                ("input", "pointcloud_raw_ex"),
                ("output", "self_cropped/pointcloud_ex"),
            ],
            parameters=[cropbox_parameters],
            extra_arguments=[{"use_intra_process_comms": LaunchConfiguration("use_intra_process")}],
        )
    )

    cropbox_parameters["min_x"] = mirror_info["min_longitudinal_offset"]
    cropbox_parameters["max_x"] = mirror_info["max_longitudinal_offset"]
    cropbox_parameters["min_y"] = mirror_info["min_lateral_offset"]
    cropbox_parameters["max_y"] = mirror_info["max_lateral_offset"]
    cropbox_parameters["min_z"] = mirror_info["min_height_offset"]
    cropbox_parameters["max_z"] = mirror_info["max_height_offset"]

    nodes.append(
        ComposableNode(
            package="autoware_pointcloud_preprocessor",
            plugin="autoware::pointcloud_preprocessor::CropBoxFilterComponent",
            name="crop_box_filter_mirror",
            remappings=[
                ("input", "self_cropped/pointcloud_ex"),
                ("output", "mirror_cropped/pointcloud_ex"),
            ],
            parameters=[cropbox_parameters],
            extra_arguments=[{"use_intra_process_comms": LaunchConfiguration("use_intra_process")}],
        )
    )

    nodes.append(
        ComposableNode(
            package="autoware_pointcloud_preprocessor",
            plugin="autoware::pointcloud_preprocessor::DistortionCorrectorComponent",
            name="distortion_corrector_node",
            remappings=[
                ("~/input/twist", "/sensing/vehicle_velocity_converter/twist_with_covariance"),
                ("~/input/imu", "/sensing/imu/imu_data"),
                ("~/input/pointcloud", "mirror_cropped/pointcloud_ex"),
                ("~/output/pointcloud", "rectified/pointcloud_ex"),
            ],
            parameters=[distortion_corrector_node_param],
            extra_arguments=[{"use_intra_process_comms": LaunchConfiguration("use_intra_process")}],
        )
    )

    # Ring Outlier Filter is the last component in the pipeline, so control the output frame here
    if LaunchConfiguration("output_as_sensor_frame").perform(context).lower() == "true":
        ring_outlier_output_frame = {"output_frame": LaunchConfiguration("frame_id")}
    else:
        # keep the output frame as the input frame
        ring_outlier_output_frame = {"output_frame": ""}

    nodes.append(
        ComposableNode(
            package="autoware_pointcloud_preprocessor",
            plugin="autoware::pointcloud_preprocessor::RingOutlierFilterComponent",
            name="ring_outlier_filter",
            remappings=[
                ("input", "rectified/pointcloud_ex"),
                ("output", "pointcloud_before_sync"),
            ],
            parameters=[ring_outlier_filter_node_param, ring_outlier_output_frame],
            extra_arguments=[{"use_intra_process_comms": LaunchConfiguration("use_intra_process")}],
        )
    )

    return nodes


def make_blockage_diag_nodes(context):
    return [
        ComposableNode(
            package="autoware_pointcloud_preprocessor",
            plugin="autoware::pointcloud_preprocessor::BlockageDiagComponent",
            name="blockage_diag",
            remappings=[
                ("input", "pointcloud_raw_ex"),
                ("output", "blockage_diag/pointcloud"),
            ],
            parameters=[
                {
                    "angle_range": [
                        float(context.perform_substitution(LaunchConfiguration("cloud_min_angle"))),
                        float(context.perform_substitution(LaunchConfiguration("cloud_max_angle"))),
                    ],
                    "horizontal_ring_id": LaunchConfiguration("horizontal_ring_id"),
                    "vertical_bins": LaunchConfiguration("vertical_bins"),
                    "is_channel_order_top2down": LaunchConfiguration("is_channel_order_top2down"),
                    "max_distance_range": LaunchConfiguration("max_range"),
                    "horizontal_resolution": LaunchConfiguration("horizontal_resolution"),
                }
            ]
            + [load_composable_node_param(context, "blockage_diagnostics_param_file")],
            extra_arguments=[{"use_intra_process_comms": LaunchConfiguration("use_intra_process")}],
        )
    ]


def launch_setup(context, *args, **kwargs):
    nodes = []

    nodes.extend(make_common_nodes(context))
    nodes.extend(make_nebula_nodes(context))

    if IfCondition(LaunchConfiguration("use_cuda_preprocessor")).evaluate(context):
        nodes.extend(make_cuda_preprocessor_nodes(context))
    else:
        nodes.extend(make_preprocessor_nodes(context))

    if IfCondition(LaunchConfiguration("enable_blockage_diag")).evaluate(context):
        nodes.extend(make_blockage_diag_nodes(context))

    # set container to run all required components in the same process
    container = ComposableNodeContainer(
        name=LaunchConfiguration("container_name"),
        namespace="pointcloud_preprocessor",
        package="rclcpp_components",
        executable=LaunchConfiguration("container_executable"),
        composable_node_descriptions=nodes,
        output="both",
        condition=UnlessCondition(LaunchConfiguration("use_shared_container")),
    )

    load_composable_nodes = LoadComposableNodes(
        composable_node_descriptions=nodes,
        target_container=LaunchConfiguration("container_name"),
        condition=IfCondition(LaunchConfiguration("use_shared_container")),
    )

    return [container, load_composable_nodes]


def generate_launch_description():
    launch_arguments = []

    def add_launch_arg(name: str, default_value=None, description=None):
        # a default_value of None is equivalent to not passing that kwarg at all
        launch_arguments.append(
            DeclareLaunchArgument(name, default_value=default_value, description=description)
        )

    common_sensor_share_dir = get_package_share_directory("aip_common_sensor_launch")

    add_launch_arg("sensor_model", description="sensor model name")
    add_launch_arg(
        "nebula_common_config_file",
        [
            FindPackageShare("aip_common_sensor_launch"),
            "/config/nebula_hesai_common.param.yaml",
        ],
        description="file containing parameters common to all Nebula instances",
    )
    add_launch_arg("config_file", "", description="sensor configuration file")
    add_launch_arg("launch_driver", "True", "do launch driver")
    add_launch_arg("setup_sensor", "True", "configure sensor")
    add_launch_arg("udp_only", "False", "use UDP only")
    add_launch_arg(
        "udp_socket_receive_buffer_size_bytes", "5400000", "UDP socket receive buffer size in bytes"
    )
    add_launch_arg("retry_hw", "false", "retry hw")
    add_launch_arg("sensor_ip", "192.168.1.201", "device ip address")
    add_launch_arg(
        "multicast_ip",
        "",
        "the multicast group the sensor shall broadcast to. leave empty to disable multicast",
    )
    add_launch_arg("host_ip", "255.255.255.255", "host ip address")
    add_launch_arg("sync_angle", "0")
    add_launch_arg("cut_angle", "0.0")
    add_launch_arg("base_frame", "base_link", "base frame id")
    add_launch_arg("min_range", "0.3", "minimum view range for Velodyne sensors")
    add_launch_arg("max_range", "300.0", "maximum view range for Velodyne sensors")
    add_launch_arg("cloud_min_angle", "0", "minimum view angle setting on device")
    add_launch_arg("cloud_max_angle", "360", "maximum view angle setting on device")
    add_launch_arg("data_port", "2368", "device data port number")
    add_launch_arg("gnss_port", "2380", "device gnss port number")
    add_launch_arg("packet_mtu_size", "1500", "packet mtu size")
    add_launch_arg("rotation_speed", "600", "rotational frequency")
    add_launch_arg("dual_return_distance_threshold", "0.1", "dual return distance threshold")
    add_launch_arg("frame_id", "lidar", "frame id")
    add_launch_arg("input_frame", LaunchConfiguration("base_frame"), "use for cropbox")
    add_launch_arg("output_frame", LaunchConfiguration("base_frame"), "use for cropbox")
    add_launch_arg("diag_span", "1000")
    add_launch_arg("hires_mode", "true", "enable high resolution mode in OT128")
    add_launch_arg("advanced_diagnostics", "false")
    add_launch_arg("use_multithread", "False", "use multithread")
    add_launch_arg("use_intra_process", "False", "use ROS 2 component container communication")
    add_launch_arg(
        "diagnostics.packet_loss.error_threshold",
        "10",
        "packet_loss error_threshold for diagnostics",
    )

    add_launch_arg(
        "lidar_container_name",
        "nebula_node_container",
        "Name of the new container to be created when use_shared_container is false",
    )
    add_launch_arg(
        "use_shared_container",
        "False",
        "Whether to use a new container for this lidar or use an existing one",
    )
    add_launch_arg(
        "use_cuda_preprocessor",
        "False",
        "Use the cuda implementation of the pointcloud preprocessor. When using the CUDA implementations for both concatenation and the preprocessor, requires use_shared_container to be enabled",
    )
    add_launch_arg("output_as_sensor_frame", "True", "output final pointcloud in sensor frame")
    add_launch_arg("enable_blockage_diag", "true")
    add_launch_arg("horizontal_ring_id", "64")
    add_launch_arg("vertical_bins", "128")
    add_launch_arg("is_channel_order_top2down", "true")
    add_launch_arg("horizontal_resolution", "0.4")
    add_launch_arg(
        "blockage_diagnostics_param_file",
        os.path.join(
            common_sensor_share_dir,
            "config",
            "blockage_diagnostics.param.yaml",
        ),
        description="path to parameter file of blockage diagnostics node",
    )
    add_launch_arg(
        "vehicle_mirror_param_file",
        description="path to the file of vehicle mirror position yaml",
    )
    add_launch_arg(
        "distortion_correction_node_param_path",
        os.path.join(
            common_sensor_share_dir,
            "config",
            "distortion_corrector_node.param.yaml",
        ),
        description="path to parameter file of distortion correction node",
    )
    add_launch_arg(
        "ring_outlier_filter_node_param_path",
        os.path.join(
            common_sensor_share_dir,
            "config",
            "ring_outlier_filter_node.param.yaml",
        ),
        description="path to parameter file of ring outlier filter node",
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

    return launch.LaunchDescription(
        launch_arguments
        + [set_container_executable, set_container_mt_executable]
        + [OpaqueFunction(function=launch_setup)]
    )
