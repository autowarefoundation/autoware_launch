# Copyright 2024 TIER IV, Inc.
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


from distutils.util import strtobool
import json
import os

import launch
from launch.actions import DeclareLaunchArgument
from launch.actions import GroupAction
from launch.actions import IncludeLaunchDescription
from launch.actions import LogInfo
from launch.actions import OpaqueFunction
from launch.actions import TimerAction
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def create_camera_container(camera_id, container_name, use_multithread):
    package = FindPackageShare("edge_auto_jetson_launch")
    include = PathJoinSubstitution([package, "launch/camera_common/camera_container.launch.py"])
    local_container_name = container_name + str(camera_id)
    arguments = [
        ("camera_id", str(camera_id)),
        ("container", local_container_name),
        ("use_multithread", use_multithread),
    ]
    return IncludeLaunchDescription(include, launch_arguments=arguments)


def create_object_recognition(
    camera_id, container_name, yolox_precision, process_index, build_engine_only
):
    package = FindPackageShare("edge_auto_jetson_launch")
    include = PathJoinSubstitution(
        [package, "launch/object_recognition/object_recognition.launch.xml"]
    )
    container_name += str(camera_id)
    arguments = [
        ("camera_id", str(camera_id)),
        ("roi_id", str(camera_id)),
        ("container_name", container_name),
        ("dla_core_id", str(-1)),  # 0 or 1, select -1 to run on GPU
        ("yolox_precision", yolox_precision),
        ("build_engine_only", build_engine_only),
    ]
    return IncludeLaunchDescription(include, launch_arguments=arguments)


def create_traffic_light_recognition(camera_id, container_name, build_engine_only):
    package = FindPackageShare("edge_auto_jetson_launch")
    include = PathJoinSubstitution(
        [
            package,
            "launch/traffic_light_recognition/traffic_light_recognition.launch.xml",
        ]
    )
    container_name += str(camera_id)
    camera_name = "camera" + str(camera_id)
    arguments = [
        ("camera_name", camera_name),
        ("container", container_name),
        ("build_engine_only", build_engine_only),
    ]
    return IncludeLaunchDescription(include, launch_arguments=arguments)


def create_camera_driver(camera_id, container_name):
    # get bsp version as suffix when bsp version >= 36
    cti_version_file = "/etc/cti/CTI-L4T.version"
    version_suffix = ""
    try:
        with open(cti_version_file) as f:
            version_line = f.read()
            version_str = version_line.split("ORIN-AGX-")[-1].split("-")[0]
            major_version = int(version_str.split(".")[0])
            if major_version >= 36:
                version_suffix = "_bsp_r36"
    except Exception:
        pass

    individual_param_package = FindPackageShare("individual_params")
    individual_param_dir = PathJoinSubstitution(
        [
            individual_param_package,
            "config",
            os.environ.get("VEHICLE_ID", "default"),
            os.environ.get("SENSOR_MODEL", "aip_x2_gen2"),
            "tier4-c2/",
        ]
    )
    readout_delay_param_path = PathJoinSubstitution(
        [individual_param_dir, f"readout_delay_{camera_id}{version_suffix}.param.yaml"]
    )

    package = FindPackageShare("edge_auto_jetson_launch")
    include = PathJoinSubstitution([package, "launch/camera_common/v4l2_camera.launch.xml"])
    container_name += str(camera_id)

    arguments = [
        ("camera_id", str(camera_id)),
        ("readout_delay_setter_param_path", readout_delay_param_path),
        ("container_name", container_name),
    ]
    return IncludeLaunchDescription(include, launch_arguments=arguments)


def wrap_with_delay(action, delay_sec, camera_id):
    description = f"v4l2 initialization for camera{camera_id}"
    if delay_sec <= 0.0:
        return GroupAction(
            actions=[
                LogInfo(
                    msg=(
                        "[v4l2_camera.launch] "
                        f"camera{camera_id}: scheduling {description} "
                        f"with startup_delay_sec={delay_sec:.1f}"
                    )
                ),
                LogInfo(
                    msg=(
                        "[v4l2_camera.launch] "
                        f"camera{camera_id}: starting {description} "
                        f"after startup_delay_sec={delay_sec:.1f}"
                    )
                ),
                action,
            ]
        )

    return GroupAction(
        actions=[
            LogInfo(
                msg=(
                    "[v4l2_camera.launch] "
                    f"camera{camera_id}: scheduling {description} "
                    f"with startup_delay_sec={delay_sec:.1f}"
                )
            ),
            TimerAction(
                period=delay_sec,
                actions=[
                    LogInfo(
                        msg=(
                            "[v4l2_camera.launch] "
                            f"camera{camera_id}: starting {description} "
                            f"after startup_delay_sec={delay_sec:.1f}"
                        )
                    ),
                    action,
                ],
            ),
        ]
    )


def create_image_decompressor(camera_id, container_name):
    package = FindPackageShare("edge_auto_jetson_launch")
    include = PathJoinSubstitution(
        [package, "launch/camera_common/image_transport_decompressor.launch.xml"]
    )
    container_name += str(camera_id)
    arguments = [("camera_id", str(camera_id)), ("container_name", container_name)]
    return IncludeLaunchDescription(include, launch_arguments=arguments)


def create_image_diagnostics(camera_id, container_name):
    package = FindPackageShare("edge_auto_jetson_launch")
    include = PathJoinSubstitution([package, "launch/camera_common/image_diagnostics.launch.xml"])
    container_name += str(camera_id)
    arguments = [("camera_id", str(camera_id)), ("container_name", container_name)]
    return IncludeLaunchDescription(include, launch_arguments=arguments)


def create_camera_sync_doctor(
    camera_id,
    container_name,
    use_sxpf,
    trigger_id,
    trigger_time_tolerance_ns,
    camera_time_tolerance_ns,
):
    package = FindPackageShare("edge_auto_jetson_launch")
    include = PathJoinSubstitution([package, "launch/camera_common/camera_sync_doctor.launch.xml"])
    container_name = "" if container_name == "" else container_name + str(camera_id)

    individual_param_package = FindPackageShare("individual_params")
    individual_param_dir = PathJoinSubstitution(
        [
            individual_param_package,
            "config",
            os.environ.get("VEHICLE_ID", "default"),
            os.environ.get("SENSOR_MODEL", "aip_x2_gen2"),
            "tier4-c2/",
        ]
    )
    if use_sxpf:
        camera_driver_param_path = PathJoinSubstitution(
            [individual_param_dir, f"camera{camera_id}_sxpf.param.yaml"]
        )
        # sxpf param file also includes readout delay information
        readout_delay_param_path = camera_driver_param_path
        trigger_topic = "/addon_trigger/trigger_time"
    else:
        # get bsp version as suffix when bsp version >= 36
        cti_version_file = "/etc/cti/CTI-L4T.version"
        version_suffix = ""
        try:
            with open(cti_version_file) as f:
                version_line = f.read()
                version_str = version_line.split("ORIN-AGX-")[-1].split("-")[0]
                major_version = int(version_str.split(".")[0])
                if major_version >= 36:
                    version_suffix = "_bsp_r36"
        except Exception:
            pass

        camera_driver_param_path = PathJoinSubstitution(
            [individual_param_dir, f"v4l2_{camera_id}.param.yaml"]
        )
        readout_delay_param_path = PathJoinSubstitution(
            [individual_param_dir, f"readout_delay_{camera_id}{version_suffix}.param.yaml"]
        )
        trigger_topic = f"/sensing/camera/camera{trigger_id}/trigger_time"

    arguments = [
        ("camera_id", str(camera_id)),
        ("container_name", container_name),
        ("trigger_id", trigger_id),
        ("trigger_topic", trigger_topic),
        ("camera_driver_param_path", camera_driver_param_path),
        ("readout_delay_param_path", readout_delay_param_path),
        ("trigger_time_tolerance_ns", trigger_time_tolerance_ns),
        ("camera_time_tolerance_ns", camera_time_tolerance_ns),
    ]
    return IncludeLaunchDescription(include, launch_arguments=arguments)


def launch_setup(context, *args, **kwargs):

    # Load all camera ids
    object_recognition_camera_ids = LaunchConfiguration("object_recognition_camera_ids").perform(
        context
    )
    traffic_light_camera_ids = LaunchConfiguration("traffic_light_camera_ids").perform(context)
    camera_driver_camera_ids = LaunchConfiguration("camera_driver_camera_ids").perform(context)
    image_diagnostics_camera_ids = LaunchConfiguration("image_diagnostics_camera_ids").perform(
        context
    )
    live_sensor = LaunchConfiguration("live_sensor").perform(context)
    container_name = LaunchConfiguration("container_name").perform(context)
    yolox_precision = LaunchConfiguration("yolox_precision").perform(context)
    use_multithread = LaunchConfiguration("use_multithread").perform(context)
    build_engine_only = LaunchConfiguration("build_engine_only").perform(context)
    trigger_id = LaunchConfiguration("trigger_id").perform(context)
    trigger_time_tolerance_ns = LaunchConfiguration("trigger_time_tolerance_ns").perform(context)
    camera_time_tolerance_ns = LaunchConfiguration("camera_time_tolerance_ns").perform(context)
    camera_start_interval_sec = float(
        LaunchConfiguration("camera_start_interval_sec").perform(context)
    )

    # Convert string to list safely
    object_recognition_camera_ids = json.loads(object_recognition_camera_ids)
    camera_driver_camera_ids = json.loads(camera_driver_camera_ids)
    traffic_light_camera_ids = json.loads(traffic_light_camera_ids)
    image_diagnostics_camera_ids = json.loads(image_diagnostics_camera_ids)

    # containers will be used for object recognition and traffic light recognition
    # so we need to merge them into one set to avoid duplication
    camera_containers = set(object_recognition_camera_ids + traffic_light_camera_ids)
    camera_containers = [
        create_camera_container(camera_id, container_name, use_multithread)
        for camera_id in camera_containers
    ]

    if build_engine_only.lower() == "true":
        camera_containers = []

    # object recognition
    object_recognitions = [
        create_object_recognition(
            camera_id, container_name, yolox_precision, process_index, build_engine_only
        )
        for process_index, camera_id in enumerate(object_recognition_camera_ids)
    ]

    traffic_light_recognitions = [
        create_traffic_light_recognition(camera_id, container_name, build_engine_only)
        for camera_id in traffic_light_camera_ids
    ]

    image_diagnostics = [
        create_image_diagnostics(camera_id, container_name)
        for camera_id in image_diagnostics_camera_ids
    ]

    # cspell: ignore decompressors
    # camera driver and image decompressor
    if bool(strtobool(live_sensor)):
        camera_drivers = [
            wrap_with_delay(
                create_camera_driver(camera_id, container_name),
                camera_start_interval_sec * process_index,
                camera_id,
            )
            for process_index, camera_id in enumerate(camera_driver_camera_ids)
        ]
        image_decompressors = []

        # Run camera sync doctor for all related cameras
        camera_sync_doctor_camera_ids = list(
            set(camera_driver_camera_ids)
            | set(object_recognition_camera_ids)
            | set(traffic_light_camera_ids)
        )
        # If IDs for camera driver are not specified at all,
        # that implies the camera drivers are executed outside of this launch package,
        # i.e., the SXPF case
        use_sxpf = len(camera_driver_camera_ids) == 0
        container_name_sync_doc = "" if use_sxpf else container_name
        camera_sync_doctors = [
            create_camera_sync_doctor(
                camera_id,
                container_name_sync_doc,
                use_sxpf,
                trigger_id,
                trigger_time_tolerance_ns,
                camera_time_tolerance_ns,
            )
            for camera_id in camera_sync_doctor_camera_ids
        ]
    if not bool(strtobool(live_sensor)):
        camera_drivers = []
        image_decompressors = [
            create_image_decompressor(camera_id, container_name)
            for camera_id in camera_driver_camera_ids
        ]
        camera_sync_doctors = []

    return (
        camera_containers
        + object_recognitions
        + traffic_light_recognitions
        + camera_drivers
        + image_decompressors
        + image_diagnostics
        + camera_sync_doctors
    )


def generate_launch_description():
    return launch.LaunchDescription(
        [
            DeclareLaunchArgument(
                "object_recognition_camera_ids",
                description="camera index list for object recognition",
            ),
            DeclareLaunchArgument(
                "traffic_light_camera_ids",
                description="camera index list for traffic light recognition",
            ),
            DeclareLaunchArgument(
                "camera_driver_camera_ids",
                description="camera index list for starting camera driver",
            ),
            DeclareLaunchArgument(
                "image_diagnostics_camera_ids",
                default_value="[]",
                description="camera index list for image blockage diagnostics",
            ),
            DeclareLaunchArgument("live_sensor", description="live camera driver or not"),
            DeclareLaunchArgument(
                "container_name", description="container name for object recognition"
            ),
            DeclareLaunchArgument(
                "use_multithread", description="use multithread container or not"
            ),
            DeclareLaunchArgument("build_engine_only", description="build engine only or not"),
            DeclareLaunchArgument(
                "trigger_id",
                default_value="0",
                description="camera id under which trigger_time is published ",
            ),
            DeclareLaunchArgument(
                "trigger_time_tolerance_ns",
                default_value="1000000",
                description="tolerance time in nanoseconds for trigger sync diagnostic",
            ),
            DeclareLaunchArgument(
                "camera_time_tolerance_ns",
                default_value="10000000",
                description="tolerance time in nanoseconds for camera sync diagnostic",
            ),
            DeclareLaunchArgument(
                "camera_start_interval_sec",
                default_value="0.0",
                description="interval seconds between each camera driver startup",
            ),
            OpaqueFunction(function=launch_setup),
        ]
    )
