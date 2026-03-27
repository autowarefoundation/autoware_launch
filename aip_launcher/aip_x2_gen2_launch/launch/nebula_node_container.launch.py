# Copyright 2023 Tier IV, Inc. All rights reserved.
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

import logging
from math import atan2
from math import cos
from math import pi
from math import sin
from math import sqrt
import os

from ament_index_python.packages import get_package_share_directory
import launch
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.actions import OpaqueFunction

# from launch.conditions import LaunchConfigurationNotEquals
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.actions import LoadComposableNodes
from launch_ros.actions import Node
from launch_ros.descriptions import ComposableNode
from launch_ros.parameter_descriptions import ParameterFile
from launch_ros.substitutions import FindPackageShare
import yaml

logger = logging.getLogger(__name__)


def get_lidar_make(sensor_name):
    if sensor_name[:6].lower() == "pandar":
        return "Hesai", ".csv"
    elif sensor_name[:3].lower() in ["hdl", "vlp", "vls"]:
        return "Velodyne", ".yaml"
    return "unrecognized_sensor_model"


def get_max_extents_from_wheel_center(
    wheel_width_m: float, wheel_radius_m: float, max_steer_angle_rad: float
):
    """
    Calculate bounding box extents for wheels when at maximum steering angle.

    Given a wheel of a given width and radius, and a maximum steering angle, calculate the maximum
    extent from the wheel center in vehicle coordinates when the wheel is steered to its maximum
    angle.

    Result is a tuple containing the maximum lateral (outward from tire center) and longitudinal
    (forward/backward from tire center) extents.
    """
    assert wheel_width_m > 0
    assert wheel_radius_m > 0
    assert max_steer_angle_rad < 90 / 180 * pi

    w_half = wheel_width_m / 2
    d_center_to_corner = sqrt(w_half**2 + wheel_radius_m**2)
    angle_corner = atan2(w_half, wheel_radius_m)

    if angle_corner < max_steer_angle_rad:
        max_longitudinal_offset = d_center_to_corner
    else:
        max_longitudinal_offset = d_center_to_corner * cos(max_steer_angle_rad - angle_corner)

    max_lateral_offset = d_center_to_corner * sin(max_steer_angle_rad + angle_corner)

    assert max_lateral_offset > 0
    assert max_longitudinal_offset > 0

    return max_lateral_offset, max_longitudinal_offset


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

    wheel_width = gp["wheel_width"]
    wheel_radius = gp["wheel_radius"]
    max_tire_cut_angle = 0.838

    max_lat_offset, max_lon_offset = get_max_extents_from_wheel_center(
        wheel_width, wheel_radius, max_tire_cut_angle
    )

    p["wheels_min_longitudinal_offset"] = gp["wheel_base"] - max_lon_offset
    p["wheels_max_longitudinal_offset"] = gp["wheel_base"] + max_lon_offset
    p["wheels_min_lateral_offset"] = -(gp["wheel_tread"] / 2 + max_lat_offset)
    p["wheels_max_lateral_offset"] = gp["wheel_tread"] / 2 + max_lat_offset
    p["wheels_min_height_offset"] = 0.0

    # The wheel height is scaled to 110% here (radius * 2) * 1.1 to account for
    # possible suspension movement. There is no data at full steering on bumpy
    # ground, so this is hard to verify. Might not be needed at all, or might
    # need individual tuning for different vehicle platforms.
    # Leaving it in for now, as it doesn't cause anyone trouble and *might* be needed.
    p["wheels_max_height_offset"] = wheel_radius * 2.2

    return p


def load_composable_node_param(context, param_path):
    with open(LaunchConfiguration(param_path).perform(context), "r") as f:
        return yaml.safe_load(f)["/**"]["ros__parameters"]


def create_parameter_dict(*args):
    result = {}
    for x in args:
        result[x] = LaunchConfiguration(x)
    return result


def make_nebula_node(context, as_composable_node, env=None):
    # Model and make
    sensor_model = LaunchConfiguration("sensor_model").perform(context)
    sensor_make, _ = get_lidar_make(sensor_model)

    parameters = [
        ParameterFile(
            LaunchConfiguration("nebula_common_config_file").perform(context),
            allow_substs=True,
        ),
        ParameterFile(
            LaunchConfiguration("nebula_model_specific_config_file").perform(context),
            allow_substs=True,
        ),
        {
            "sensor_model": sensor_model,
            **create_parameter_dict(
                "host_ip",
                "sensor_ip",
                "multicast_ip",
                "data_port",
                "return_mode",
                "min_range",
                "max_range",
                "frame_id",
                "sync_angle",
                "cut_angle",
                "dual_return_distance_threshold",
                "rotation_speed",
                "cloud_min_angle",
                "cloud_max_angle",
                "gnss_port",
                "packet_mtu_size",
                "setup_sensor",
                "diag_span",
                "calibration_file",
                "calibration_download_enabled",
                "launch_hw",
                "udp_only",
                "point_filters.downsample_mask.path",
                "hires_mode",
                "diagnostics.packet_loss.error_threshold",
                "udp_socket_receive_buffer_size_bytes",
            ),
            "retry_hw": True,
        },
    ]

    node_name = sensor_make.lower() + "_ros_wrapper_node"
    remappings = [
        ("pandar_points", "pointcloud_raw_ex"),
    ]

    if as_composable_node:
        return ComposableNode(
            package="nebula_" + sensor_make.lower(),
            plugin=sensor_make + "RosWrapper",
            name=node_name,
            parameters=parameters,
            remappings=remappings,
            extra_arguments=[{"use_intra_process_comms": LaunchConfiguration("use_intra_process")}],
        )

    else:
        return Node(
            package="nebula_" + sensor_make.lower(),
            executable=node_name,
            name=node_name,
            parameters=parameters,
            remappings=remappings,
            additional_env=env,
        )


def make_preprocessor_nodes(context):
    vehicle_info = get_vehicle_info(context)

    cropbox_parameters_self = create_parameter_dict("input_frame", "output_frame")
    cropbox_parameters_self["negative"] = True
    cropbox_parameters_self["processing_time_threshold_sec"] = 0.01

    cropbox_parameters_self["min_x"] = vehicle_info["min_longitudinal_offset"]
    cropbox_parameters_self["max_x"] = vehicle_info["max_longitudinal_offset"]
    cropbox_parameters_self["min_y"] = vehicle_info["min_lateral_offset"]
    cropbox_parameters_self["max_y"] = vehicle_info["max_lateral_offset"]
    cropbox_parameters_self["min_z"] = vehicle_info["min_height_offset"]
    cropbox_parameters_self["max_z"] = vehicle_info["max_height_offset"]

    cropbox_parameters_wheels = create_parameter_dict("input_frame", "output_frame")
    cropbox_parameters_wheels["negative"] = True
    cropbox_parameters_wheels["processing_time_threshold_sec"] = 0.01

    cropbox_parameters_wheels["min_x"] = vehicle_info["wheels_min_longitudinal_offset"]
    cropbox_parameters_wheels["max_x"] = vehicle_info["wheels_max_longitudinal_offset"]
    cropbox_parameters_wheels["min_y"] = vehicle_info["wheels_min_lateral_offset"]
    cropbox_parameters_wheels["max_y"] = vehicle_info["wheels_max_lateral_offset"]
    cropbox_parameters_wheels["min_z"] = vehicle_info["wheels_min_height_offset"]
    cropbox_parameters_wheels["max_z"] = vehicle_info["wheels_max_height_offset"]

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
            parameters=[cropbox_parameters_self],
            extra_arguments=[{"use_intra_process_comms": LaunchConfiguration("use_intra_process")}],
        )
    )

    nodes.append(
        ComposableNode(
            package="autoware_pointcloud_preprocessor",
            plugin="autoware::pointcloud_preprocessor::CropBoxFilterComponent",
            name="crop_box_filter_wheels",
            remappings=[
                ("input", "self_cropped/pointcloud_ex"),
                ("output", "wheels_cropped/pointcloud_ex"),
            ],
            parameters=[cropbox_parameters_wheels],
            extra_arguments=[{"use_intra_process_comms": LaunchConfiguration("use_intra_process")}],
        )
    )

    nodes.append(
        ComposableNode(
            package="autoware_pointcloud_preprocessor",
            plugin="autoware::pointcloud_preprocessor::DistortionCorrectorComponent",
            name="distortion_corrector_node",
            remappings=[
                (
                    "~/input/twist",
                    "/sensing/vehicle_velocity_converter/twist_with_covariance",
                ),
                ("~/input/imu", "/sensing/imu/imu_data"),
                ("~/input/pointcloud", "wheels_cropped/pointcloud_ex"),
                ("~/output/pointcloud", "rectified/pointcloud_ex"),
            ],
            parameters=[
                load_composable_node_param(context, "distortion_corrector_node_param_file")
            ],
            extra_arguments=[{"use_intra_process_comms": LaunchConfiguration("use_intra_process")}],
        )
    )

    ring_outlier_filter_node_param = ParameterFile(
        param_file=LaunchConfiguration("ring_outlier_filter_node_param_file").perform(context),
        allow_substs=True,
    )

    # Ring Outlier Filter is the last component in the pipeline, so control the output frame here
    if LaunchConfiguration("output_as_sensor_frame").perform(context).lower() == "true":
        ring_outlier_output_frame = {"output_frame": LaunchConfiguration("frame_id")}
    else:
        # keep the output frame as the input frame
        ring_outlier_output_frame = {"output_frame": ""}

    use_dual_return_filter = IfCondition(LaunchConfiguration("use_dual_return_filter")).evaluate(
        context
    )

    if not use_dual_return_filter:
        nodes.append(
            ComposableNode(
                package="autoware_pointcloud_preprocessor",
                plugin="autoware::pointcloud_preprocessor::RingOutlierFilterComponent",
                name="ring_outlier_filter",
                remappings=[
                    ("input", "rectified/pointcloud_ex"),
                    ("output", "pointcloud_before_sync"),
                ],
                parameters=[
                    ring_outlier_filter_node_param,
                    ring_outlier_output_frame,
                    {"is_agnocast_publish_node": True},
                ],
                extra_arguments=[
                    {"use_intra_process_comms": LaunchConfiguration("use_intra_process")}
                ],
            )
        )
    else:
        nodes.append(
            ComposableNode(
                package="autoware_pointcloud_preprocessor",
                plugin="autoware::pointcloud_preprocessor::DualReturnOutlierFilterComponent",
                name="dual_return_filter",
                remappings=[
                    ("input", "rectified/pointcloud_ex"),
                    ("output", "pointcloud_before_sync"),
                ],
                parameters=[
                    {
                        "vertical_bins": LaunchConfiguration("vertical_bins"),
                        "min_azimuth_deg": LaunchConfiguration("min_azimuth_deg"),
                        "max_azimuth_deg": LaunchConfiguration("max_azimuth_deg"),
                        "is_agnocast_publish_node": True,
                    }
                ]
                + [load_composable_node_param(context, "dual_return_filter_param_file")],
                extra_arguments=[
                    {"use_intra_process_comms": LaunchConfiguration("use_intra_process")}
                ],
            )
        )

    return nodes


def make_cuda_preprocessor_nodes(context):
    # Vehicle parameters
    vehicle_info = get_vehicle_info(context)

    # Pointcloud preprocessor parameters
    distortion_corrector_node_param = ParameterFile(
        param_file=LaunchConfiguration("distortion_corrector_node_param_file").perform(context),
        allow_substs=True,
    )
    ring_outlier_filter_node_param = ParameterFile(
        param_file=LaunchConfiguration("ring_outlier_filter_node_param_file").perform(context),
        allow_substs=True,
    )

    preprocessor_parameters = {}
    preprocessor_parameters["crop_box.min_x"] = [
        vehicle_info["min_longitudinal_offset"],
        vehicle_info["wheels_min_longitudinal_offset"],
    ]
    preprocessor_parameters["crop_box.max_x"] = [
        vehicle_info["max_longitudinal_offset"],
        vehicle_info["wheels_max_longitudinal_offset"],
    ]
    preprocessor_parameters["crop_box.min_y"] = [
        vehicle_info["min_lateral_offset"],
        vehicle_info["wheels_min_lateral_offset"],
    ]
    preprocessor_parameters["crop_box.max_y"] = [
        vehicle_info["max_lateral_offset"],
        vehicle_info["wheels_max_lateral_offset"],
    ]
    preprocessor_parameters["crop_box.min_z"] = [
        vehicle_info["min_height_offset"],
        vehicle_info["wheels_min_height_offset"],
    ]
    preprocessor_parameters["crop_box.max_z"] = [
        vehicle_info["max_height_offset"],
        vehicle_info["wheels_max_height_offset"],
    ]
    preprocessor_parameters["crop_box.negative"] = [True, True]

    return [
        ComposableNode(
            package="autoware_cuda_pointcloud_preprocessor",
            plugin="autoware::cuda_pointcloud_preprocessor::CudaPointcloudPreprocessorNode",
            name="cuda_pointcloud_preprocessor_node",
            parameters=[
                preprocessor_parameters,
                distortion_corrector_node_param,
                ring_outlier_filter_node_param,
                {"enable_ring_outlier_filter": True},
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


def make_blockage_diag_nodes(context):
    return [
        ComposableNode(
            package="autoware_pointcloud_preprocessor",
            plugin="autoware::pointcloud_preprocessor::BlockageDiagComponent",
            name="blockage_return_diag",
            remappings=[
                ("input", "pointcloud_raw_ex"),
                ("output", "blockage_diag/pointcloud"),
            ],
            parameters=[
                {
                    "angle_range": LaunchConfiguration("blockage_range"),
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


def make_polar_voxel_outlier_filter_node(context):
    parameters = ParameterFile(
        LaunchConfiguration("polar_voxel_outlier_filter_node_param_file").perform(context),
        allow_substs=True,
    )

    mode = LaunchConfiguration("polar_voxel_visibility_estimation_mode").perform(context)
    node_name = (
        "polar_voxel_outlier_filter"  # node name should be consistent with metric agent config
    )

    match mode:
        case "cpu":
            return [
                ComposableNode(
                    package="autoware_pointcloud_preprocessor",
                    plugin="autoware::pointcloud_preprocessor::PolarVoxelOutlierFilterComponent",
                    name=node_name,
                    parameters=[
                        parameters,
                    ],
                    remappings=[
                        ("input", "pointcloud_raw_ex"),
                    ],
                    extra_arguments=[
                        {"use_intra_process_comms": LaunchConfiguration("use_intra_process")}
                    ],
                )
            ]
        case "cuda":
            return [
                ComposableNode(
                    package="autoware_cuda_pointcloud_preprocessor",
                    plugin="autoware::cuda_pointcloud_preprocessor::CudaPolarVoxelOutlierFilterNode",
                    name=node_name,
                    parameters=[
                        parameters,
                        {"hardware_id": node_name},
                    ],
                    remappings=[
                        ("~/input/pointcloud", "pointcloud_raw_ex"),
                        ("~/input/pointcloud/cuda", "pointcloud_raw_ex"),
                    ],
                )
            ]
        case _:
            return []


def launch_setup(context, *args, **kwargs):
    mode = LaunchConfiguration("pipeline_mode").perform(context)
    use_agnocast = os.getenv("ENABLE_AGNOCAST") == "1"

    use_blockage_diag = IfCondition(LaunchConfiguration("enable_blockage_diag")).evaluate(context)

    use_polar_voxel_outlier_filter = (
        LaunchConfiguration("polar_voxel_visibility_estimation_mode").perform(context) != "disable"
    )

    shared_container_nodes = []
    lidar_specific_container_nodes = []
    standalone_nodes = []

    match mode:
        case "cuda":
            if not use_agnocast:
                logger.warning("This pipeline mode may perform better when using Agnocast")

            shared_container_nodes.extend(make_cuda_preprocessor_nodes(context))

            if use_blockage_diag or use_polar_voxel_outlier_filter:
                lidar_specific_container_nodes.append(make_nebula_node(context, True))
            if use_polar_voxel_outlier_filter:
                lidar_specific_container_nodes.extend(make_polar_voxel_outlier_filter_node(context))
            if use_blockage_diag:
                lidar_specific_container_nodes.extend(make_blockage_diag_nodes(context))
        case "cuda-all-in-one":
            shared_container_nodes.extend(make_cuda_preprocessor_nodes(context))
            shared_container_nodes.append(make_nebula_node(context, True))

            if use_blockage_diag:
                shared_container_nodes.extend(make_blockage_diag_nodes(context))
            if use_polar_voxel_outlier_filter:
                shared_container_nodes.extend(make_polar_voxel_outlier_filter_node(context))
        case "cpu":
            lidar_specific_container_nodes.extend(make_preprocessor_nodes(context))
            lidar_specific_container_nodes.append(make_nebula_node(context, True))

            if use_blockage_diag:
                lidar_specific_container_nodes.extend(make_blockage_diag_nodes(context))
            if use_polar_voxel_outlier_filter:
                lidar_specific_container_nodes.extend(make_polar_voxel_outlier_filter_node(context))
        case "cuda-with-cpu-concat":
            lidar_specific_container_nodes.extend(make_cuda_preprocessor_nodes(context))
            lidar_specific_container_nodes.append(make_nebula_node(context, True))

            if use_blockage_diag:
                lidar_specific_container_nodes.extend(make_blockage_diag_nodes(context))
            if use_polar_voxel_outlier_filter:
                lidar_specific_container_nodes.extend(make_polar_voxel_outlier_filter_node(context))
        case _:
            raise ValueError(f"Unknown pipeline mode: {mode}")

    launch_targets = []

    if lidar_specific_container_nodes:
        lidar_specific_container = ComposableNodeContainer(
            name=LaunchConfiguration("container_name"),
            namespace="pointcloud_preprocessor",
            package=LaunchConfiguration("container_package"),
            executable=LaunchConfiguration("container_executable"),
            composable_node_descriptions=lidar_specific_container_nodes,
            output="both",
            additional_env=(
                {
                    "LD_PRELOAD": LaunchConfiguration("ld_preload_value"),
                    "AGNOCAST_MEMPOOL_SIZE": "1073741824",
                }
                if use_agnocast
                else {}
            ),
        )
        launch_targets.append(lidar_specific_container)

    if shared_container_nodes:
        load_shared_container_nodes = LoadComposableNodes(
            composable_node_descriptions=shared_container_nodes,
            target_container=LaunchConfiguration("container_name"),
        )
        launch_targets.append(load_shared_container_nodes)

    launch_targets.extend(standalone_nodes)

    return launch_targets


def generate_launch_description():
    launch_arguments = []

    def add_launch_arg(name: str, default_value=None, description=None, **kwargs):
        # a default_value of None is equivalent to not passing that kwarg at all
        launch_arguments.append(
            DeclareLaunchArgument(
                name, default_value=default_value, description=description, **kwargs
            )
        )

    # Agnocast parameters
    add_launch_arg(
        "agnocast_heaphook_path",
        "/opt/ros/humble/lib/libagnocast_heaphook.so",
        "Path to the agnocast heaphook library",
    )

    # Nebula parameters
    add_launch_arg("sensor_model", description="sensor model name")
    add_launch_arg(
        "nebula_common_config_file",
        [
            FindPackageShare("aip_x2_gen2_launch"),
            "/config/nebula_hesai_common.param.yaml",
        ],
        description="file containing parameters common to all Nebula instances",
    )
    add_launch_arg(
        "nebula_model_specific_config_file",
        [
            FindPackageShare("aip_x2_gen2_launch"),
            "/config/",
            LaunchConfiguration("sensor_model"),
            ".param.yaml",
        ],
        description="file containing parameters specific to the sensor model",
    )
    add_launch_arg("config_file", "", description="sensor configuration file")
    add_launch_arg("launch_hw", "true", "do launch driver")
    add_launch_arg("setup_sensor", "true", "configure sensor")
    add_launch_arg("sensor_ip", "192.168.1.201", "device ip address")
    add_launch_arg(
        "multicast_ip",
        "",
        "the multicast group the sensor shall broadcast to. leave empty to disable multicast",
    )
    add_launch_arg("host_ip", "255.255.255.255", "host ip address")
    add_launch_arg("sync_angle", "0")
    add_launch_arg("cut_angle", "0.0")
    add_launch_arg("udp_only", "false")
    add_launch_arg("base_frame", "base_link", "base frame id")
    add_launch_arg("min_range", "0.3", "minimum view range for Velodyne sensors")
    add_launch_arg("max_range", "300.0", "maximum view range for Velodyne sensors")
    add_launch_arg("cloud_min_angle", "0", "minimum view angle setting on device")
    add_launch_arg("cloud_max_angle", "360", "maximum view angle setting on device")
    add_launch_arg("data_port", "2368", "device data port number")
    add_launch_arg("gnss_port", "2380", "device gnss port number")
    add_launch_arg(
        "udp_socket_receive_buffer_size_bytes", "10800000", "UDP socket receive buffer size"
    )
    add_launch_arg("packet_mtu_size", "1500", "packet mtu size")
    add_launch_arg("rotation_speed", "600", "rotational frequency")
    add_launch_arg("dual_return_distance_threshold", "0.1", "dual return distance threshold")
    add_launch_arg("frame_id", "lidar", "frame id")
    add_launch_arg("diag_span", "1000")
    add_launch_arg("input_frame", LaunchConfiguration("base_frame"), "use for cropbox")
    add_launch_arg("output_frame", LaunchConfiguration("base_frame"), "use for cropbox")
    add_launch_arg("use_multithread", "true", "use multithread")
    add_launch_arg("use_intra_process", "true", "use intra-process communication in containers")
    add_launch_arg("container_name", "pointcloud_container")
    add_launch_arg(
        "pipeline_mode",
        "cuda",
        "Which pointcloud preprocessor pipeline mode to use",
        choices=["cuda", "cpu", "cuda-all-in-one", "cuda-with-cpu-concat"],
    )
    add_launch_arg(
        "polar_voxel_visibility_estimation_mode",
        "disable",
        "Which polar_voxel visibility estimation mode to use",
        choices=["cpu", "cuda", "disable"],
    )

    add_launch_arg(
        "dual_return_filter_param_file",
        [
            FindPackageShare("aip_x2_gen2_launch"),
            "/config/dual_return_filter.param.yaml",
        ],
    )
    add_launch_arg(
        "blockage_diagnostics_param_file",
        [
            FindPackageShare("aip_x2_gen2_launch"),
            "/config/blockage_diagnostics.param.yaml",
        ],
    )
    add_launch_arg(
        "ring_outlier_filter_node_param_file",
        [
            FindPackageShare("aip_x2_gen2_launch"),
            "/config/ring_outlier_filter_node.param.yaml",
        ],
    )
    add_launch_arg(
        "distortion_corrector_node_param_file",
        [
            FindPackageShare("aip_x2_gen2_launch"),
            "/config/distortion_corrector_node.param.yaml",
        ],
    )
    add_launch_arg(
        "polar_voxel_outlier_filter_node_param_file",
        [
            FindPackageShare("aip_x2_gen2_launch"),
            "/config/polar_voxel_outlier_filter_node.param.yaml",
        ],
    )
    add_launch_arg("vertical_bins", "128")
    add_launch_arg("horizontal_ring_id", "12")
    add_launch_arg("blockage_range", "[270.0, 90.0]")

    add_launch_arg("min_azimuth_deg", "135.0")
    add_launch_arg("max_azimuth_deg", "225.0")
    add_launch_arg("enable_blockage_diag", "true")

    add_launch_arg("calibration_file", "")
    add_launch_arg("calibration_download_enabled")
    add_launch_arg("output_as_sensor_frame", "true", "output final pointcloud in sensor frame")
    add_launch_arg("use_dual_return_filter", "false")
    add_launch_arg("point_filters.downsample_mask.path", "")
    add_launch_arg("hires_mode", "true")
    add_launch_arg("diagnostics.packet_loss.error_threshold")

    agnocast_env = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("autoware_agnocast_wrapper"),
                "launch",
                "agnocast_env.launch.py",
            )
        ),
        launch_arguments=[
            ("use_multithread", LaunchConfiguration("use_multithread")),
            ("agnocast_heaphook_path", LaunchConfiguration("agnocast_heaphook_path")),
        ],
    )

    return launch.LaunchDescription(
        launch_arguments + [agnocast_env, OpaqueFunction(function=launch_setup)]
    )
