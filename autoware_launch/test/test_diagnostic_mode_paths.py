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

"""Guard the CARLA diagnostic graph against missing driving-mode units.

autoware_diagnostic_graph_aggregator maps each driving/command mode to a unit
path (config/default.param.yaml). On start-up it looks every mapped path up in
the loaded graph and logs ``Mode path not found: <mode> <path>`` for any that is
absent; that mode is then never published as available, so the vehicle cannot be
engaged. autowarefoundation/autoware_universe#13252 split
``/autoware/modes/autonomous/available`` out of the continuable flag, and the
CARLA graph did not define it -- silently breaking engagement.

REQUIRED_MODE_PATHS mirrors the mode->path entries of the aggregator's
config/default.param.yaml (command_mode_mappings + driving_mode_available +
driving_mode_continuable). It is kept here as data rather than read from the
aggregator package so the test runs in the differential CI, whose underlay does
not install autoware_diagnostic_graph_aggregator. If the aggregator adds or
renames a mode, update this set to match.
"""

import os

from ament_index_python.packages import get_package_share_directory
import yaml

# Union of the paths referenced by every mode->path list in
# autoware_diagnostic_graph_aggregator/config/default.param.yaml.
REQUIRED_MODE_PATHS = {
    "/autoware/modes/stop",
    "/autoware/modes/autonomous",
    "/autoware/modes/autonomous/available",  # split out in autoware_universe#13252
    "/autoware/modes/local",
    "/autoware/modes/remote",
    "/autoware/modes/emergency_stop",
    "/autoware/modes/comfortable_stop",
    "/autoware/modes/pull_over",
}


def _resolve_include(raw: str, current_dir: str) -> str:
    """Resolve the $(dirname) / $(find-pkg-share pkg) prefixes a graph may use."""
    resolved = raw.replace("$(dirname)", current_dir)
    if "$(find-pkg-share" in resolved:
        start = resolved.index("$(find-pkg-share") + len("$(find-pkg-share")
        end = resolved.index(")", start)
        pkg = resolved[start:end].strip()
        resolved = (
            resolved[: resolved.index("$(find-pkg-share")]
            + get_package_share_directory(pkg)
            + resolved[end + 1 :]
        )
    return resolved


def _collect_unit_paths(graph_file: str, seen=None) -> set:
    """Read a graph and, following ``files:`` includes, collect every unit path."""
    if seen is None:
        seen = set()
    graph_file = os.path.abspath(graph_file)
    if graph_file in seen:
        return set()
    seen.add(graph_file)

    with open(graph_file) as f:
        doc = yaml.safe_load(f) or {}

    current_dir = os.path.dirname(graph_file)
    paths = {u["path"] for u in (doc.get("units") or []) if isinstance(u, dict) and "path" in u}

    for inc in doc.get("files") or []:
        raw = inc["path"] if isinstance(inc, dict) else inc
        resolved = _resolve_include(raw, current_dir)
        if "$(" not in resolved and os.path.exists(resolved):
            paths |= _collect_unit_paths(resolved, seen)
    return paths


def test_carla_graph_defines_every_required_mode_path():
    graph = os.path.join(
        get_package_share_directory("autoware_launch"),
        "config",
        "system",
        "diagnostics",
        "autoware-carla.yaml",
    )
    unit_paths = _collect_unit_paths(graph)
    missing = sorted(REQUIRED_MODE_PATHS - unit_paths)

    assert not missing, (
        "autoware-carla.yaml is missing driving-mode units required by "
        "autoware_diagnostic_graph_aggregator (it would log 'Mode path not "
        "found' and never report those modes available): " + ", ".join(missing)
    )
