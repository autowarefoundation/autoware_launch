#!/usr/bin/env python3
# Copyright 2026 TIER IV, Inc.
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

"""
Crop box parameters of the vehicle body and the tire steering envelope, from a vehicle_info.param.yaml.

Generator of autoware_system_designer_generate_param_files: generate(args) returns the parameters
of <output_dir>/crop_box_self.param.yaml and <output_dir>/crop_box_wheels.param.yaml.

Args:
    <vehicle_info.param.yaml> <output_dir> [--max-tire-cut-angle RAD]
"""

import argparse
from math import atan2
from math import cos
from math import pi
from math import sin
from math import sqrt
from pathlib import Path

import yaml

# Crop box bound keyed by the extent it spans.
CROP_BOX_EXTENTS = {
    "min_x": "min_longitudinal_offset",
    "max_x": "max_longitudinal_offset",
    "min_y": "min_lateral_offset",
    "max_y": "max_lateral_offset",
    "min_z": "min_height_offset",
    "max_z": "max_height_offset",
}


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


def vehicle_extents(gp):
    return {
        "min_longitudinal_offset": -gp["rear_overhang"],
        "max_longitudinal_offset": gp["front_overhang"] + gp["wheel_base"],
        "min_lateral_offset": -(gp["wheel_tread"] / 2.0 + gp["right_overhang"]),
        "max_lateral_offset": gp["wheel_tread"] / 2.0 + gp["left_overhang"],
        "min_height_offset": 0.0,
        "max_height_offset": gp["vehicle_height"],
    }


def wheels_extents(gp, max_tire_cut_angle):
    wheel_radius = gp["wheel_radius"]
    max_lat_offset, max_lon_offset = get_max_extents_from_wheel_center(
        gp["wheel_width"], wheel_radius, max_tire_cut_angle
    )

    return {
        "min_longitudinal_offset": gp["wheel_base"] - max_lon_offset,
        "max_longitudinal_offset": gp["wheel_base"] + max_lon_offset,
        "min_lateral_offset": -(gp["wheel_tread"] / 2 + max_lat_offset),
        "max_lateral_offset": gp["wheel_tread"] / 2 + max_lat_offset,
        "min_height_offset": 0.0,
        # The wheel height is scaled to 110% here (radius * 2) * 1.1 to account for
        # possible suspension movement.
        "max_height_offset": wheel_radius * 2.2,
    }


def generate(args):
    parser = argparse.ArgumentParser(prog="generate_crop_box_params.py")
    parser.add_argument("vehicle_info", type=Path, help="path to vehicle_info.param.yaml")
    parser.add_argument(
        "output_dir", help="directory of the output files, relative to the package share"
    )
    parser.add_argument(
        "--max-tire-cut-angle",
        type=float,
        default=0.838,
        help="steering envelope of the tire in radians, cropped out after the vehicle body",
    )
    args = parser.parse_args(args)

    with open(args.vehicle_info) as f:
        gp = yaml.safe_load(f)["/**"]["ros__parameters"]

    boxes = {
        "self": vehicle_extents(gp),
        "wheels": wheels_extents(gp, args.max_tire_cut_angle),
    }

    # Files carry the crop box node's own parameter names.
    files = {}
    for name, extents in boxes.items():
        params = {bound: extents[key] for bound, key in CROP_BOX_EXTENTS.items()}
        params["negative"] = True
        files[f"{args.output_dir}/crop_box_{name}.param.yaml"] = params
    return files
