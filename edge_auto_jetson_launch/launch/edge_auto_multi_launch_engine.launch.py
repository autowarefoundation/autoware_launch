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


import launch
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.actions import OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
import json
from distutils.util import strtobool


def create_camera_container(camera_id, container_name, use_multithread):
    package = FindPackageShare("edge_auto_jetson_launch")
    include = PathJoinSubstitution(
        [package, f"launch/camera_common/camera_container.launch.py"]
    )
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
        [package, f"launch/object_recognition/object_recognition.launch.xml"]
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
            f"launch/traffic_light_recognition/traffic_light_recognition.launch.xml",
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
    package = FindPackageShare("edge_auto_jetson_launch")
    include = PathJoinSubstitution(
        [package, f"launch/camera_common/v4l2_camera.launch.xml"]
    )
    container_name += str(camera_id)
    arguments = [("camera_id", str(camera_id)), ("container_name", container_name)]
    return IncludeLaunchDescription(include, launch_arguments=arguments)


def create_image_decompressor(camera_id, container_name):
    package = FindPackageShare("edge_auto_jetson_launch")
    include = PathJoinSubstitution(
        [package, f"launch/camera_common/image_transport_decompressor.launch.xml"]
    )
    container_name += str(camera_id)
    arguments = [("camera_id", str(camera_id)), ("container_name", container_name)]
    return IncludeLaunchDescription(include, launch_arguments=arguments)

def create_image_diagnostics(camera_id, container_name):
    package = FindPackageShare("edge_auto_jetson_launch")
    include = PathJoinSubstitution(
        [package, f"launch/camera_common/image_diagnostics.launch.xml"]
    )
    container_name += str(camera_id)
    arguments = [("camera_id", str(camera_id)), ("container_name", container_name)]
    return IncludeLaunchDescription(include, launch_arguments=arguments)

def launch_setup(context, *args, **kwargs):

    # Load all camera ids
    object_recognition_camera_ids = LaunchConfiguration(
        "object_recognition_camera_ids"
    ).perform(context)
    traffic_light_camera_ids = LaunchConfiguration("traffic_light_camera_ids").perform(
        context
    )
    camera_driver_camera_ids = LaunchConfiguration("camera_driver_camera_ids").perform(
        context
    )
    image_diagnostics_camera_ids = LaunchConfiguration("image_diagnostics_camera_ids").perform(
        context
    )
    live_sensor = LaunchConfiguration("live_sensor").perform(context)
    container_name = LaunchConfiguration("container_name").perform(context)
    yolox_precision = LaunchConfiguration("yolox_precision").perform(context)
    use_multithread = LaunchConfiguration("use_multithread").perform(context)
    build_engine_only = LaunchConfiguration("build_engine_only").perform(context)

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
            create_camera_driver(camera_id, container_name)
            for camera_id in camera_driver_camera_ids
        ]
        image_decompressors = []
    if not bool(strtobool(live_sensor)):
        camera_drivers = []
        image_decompressors = [
            create_image_decompressor(camera_id, container_name)
            for camera_id in camera_driver_camera_ids
        ]

    return (
        camera_containers
        + object_recognitions
        + traffic_light_recognitions
        + camera_drivers
        + image_decompressors
        + image_diagnostics
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
                description="camera index list for image blockage diagnostics",
            ),
            DeclareLaunchArgument(
                "live_sensor", description="live camera driver or not"
            ),
            DeclareLaunchArgument(
                "container_name", description="container name for object recognition"
            ),
            DeclareLaunchArgument(
                "use_multithread", description="use multithread container or not"
            ),
            DeclareLaunchArgument(
                "build_engine_only", description="build engine only or not"
            ),
            OpaqueFunction(function=launch_setup),
        ]
    )
