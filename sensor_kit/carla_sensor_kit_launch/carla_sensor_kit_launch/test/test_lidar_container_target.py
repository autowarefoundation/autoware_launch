# Copyright 2026 The Autoware Contributors
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

"""Guard the CARLA lidar preprocessing against a relative container target.

lidar.launch.xml loads its crop boxes and relay into ``pointcloud_container``,
which autoware_launch creates in the root namespace (pointcloud_container.launch.py
uses ``namespace="/"``). These loads, however, run inside the pushed
``/sensing/lidar`` namespace. A *relative* target is resolved against that
namespace, so the load waits on a container that never appears and the whole
lidar preprocessing silently never runs (no concatenated pointcloud, no NDT
input, no kinematic_state). The target must therefore be an absolute name.

This test parses the launch file, resolves the one-level ``$(var ...)``
indirection through its ``<let>``/``<arg>`` defaults, and asserts every
composable-node load target is absolute.
"""

import os
import re
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory


def _build_var_map(root: ET.Element) -> dict:
    """Collect literal values of <arg default=...> and <let value=...>."""
    values = {}
    for tag in ("arg", "let"):
        for el in root.iter(tag):
            name = el.get("name")
            value = el.get("default") if tag == "arg" else el.get("value")
            if name is not None and value is not None:
                values[name] = value
    return values


def _resolve_once(value: str, values: dict) -> str:
    """Resolve a single level of $(var NAME) using the collected values."""
    match = re.fullmatch(r"\$\(var\s+([^)]+)\)", value.strip())
    if match:
        return values.get(match.group(1).strip(), value)
    return value


def _collect_load_targets(root: ET.Element) -> list:
    """Every container target: load_composable_node@target and target_container args."""
    targets = []
    for el in root.iter("load_composable_node"):
        if el.get("target") is not None:
            targets.append(el.get("target"))
    for el in root.iter("arg"):
        if el.get("name") == "target_container" and el.get("value") is not None:
            targets.append(el.get("value"))
    return targets


def test_lidar_load_targets_are_absolute():
    launch_file = os.path.join(
        get_package_share_directory("carla_sensor_kit_launch"),
        "launch",
        "lidar.launch.xml",
    )
    root = ET.parse(launch_file).getroot()
    values = _build_var_map(root)

    targets = _collect_load_targets(root)
    assert targets, "no composable-node load targets found in lidar.launch.xml"

    relative = []
    for raw in targets:
        resolved = _resolve_once(raw, values)
        # A leftover $(var ...) we cannot resolve is treated as unknown; the ones
        # in this file resolve to the target_container let, which must be absolute.
        if not resolved.startswith("/"):
            relative.append((raw, resolved))

    assert not relative, (
        "lidar preprocessing loads into the root-namespace pointcloud_container "
        "but uses a relative target (resolves under /sensing/lidar and never "
        "loads): " + ", ".join(f"{raw!r}->{res!r}" for raw, res in relative)
    )
