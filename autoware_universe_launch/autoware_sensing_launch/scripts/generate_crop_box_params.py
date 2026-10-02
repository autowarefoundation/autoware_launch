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

"""Generate crop box filter parameter files from vehicle description parameters.

The self crop box is derived from vehicle_info.param.yaml; the mirror crop box
is taken from mirror.param.yaml. Sensor kits commit the generated files under
their own config/ and pass them via the
crop_box_filter_{self,secondary}_param_path launch arguments.
"""

import argparse
from pathlib import Path

import yaml


def load_params(path):
    with open(path) as f:
        return yaml.safe_load(f)["/**"]["ros__parameters"]


def self_crop_box(vehicle_info):
    v = vehicle_info
    return {
        "min_x": -v["rear_overhang"],
        "max_x": v["front_overhang"] + v["wheel_base"],
        "min_y": -(v["wheel_tread"] / 2.0 + v["right_overhang"]),
        "max_y": v["wheel_tread"] / 2.0 + v["left_overhang"],
        "min_z": 0.0,
        "max_z": v["vehicle_height"],
    }


def mirror_crop_box(mirror):
    return {
        "min_x": mirror["min_longitudinal_offset"],
        "max_x": mirror["max_longitudinal_offset"],
        "min_y": mirror["min_lateral_offset"],
        "max_y": mirror["max_lateral_offset"],
        "min_z": mirror["min_height_offset"],
        "max_z": mirror["max_height_offset"],
    }


def write_param_file(path, params):
    with open(path, "w") as f:
        f.write("/**:\n  ros__parameters:\n")
        for key, value in params.items():
            f.write(f"    {key}: {float(value)}\n")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("vehicle_info", type=Path, help="path to vehicle_info.param.yaml")
    parser.add_argument("mirror", type=Path, help="path to mirror.param.yaml")
    parser.add_argument("output_dir", type=Path, help="directory for the generated param files")
    args = parser.parse_args()

    write_param_file(
        args.output_dir / "crop_box_filter_self.param.yaml",
        self_crop_box(load_params(args.vehicle_info)),
    )
    write_param_file(
        args.output_dir / "crop_box_filter_mirror.param.yaml",
        mirror_crop_box(load_params(args.mirror)),
    )


if __name__ == "__main__":
    main()
