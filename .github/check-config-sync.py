#!/usr/bin/env python3
"""Check that parameter changes are reflected across all config directories.

`autoware_launch` keeps several config directories side by side (e.g.
`config_lv2`, `config_lv4`, or other use-case specific names) that are switched
depending on the autonomous driving level / use case. A parameter file usually
has a counterpart with the same relative path in each of these directories.

When a change (typically cherry-picked from autowarefoundation/autoware_launch)
touches a parameter file in only one of these directories, the counterpart in
the other directories is easy to forget. This script detects such one-sided
changes and exits with a non-zero status so the PR status check fails,
prompting the author to confirm whether the other config directories should be
updated as well.

The config directories are discovered automatically: any immediate
subdirectory of `autoware_launch/` that contains a `*.param.yaml` file is
treated as a config directory, so new directories (whatever their name) are
picked up without any configuration. Add the `ignore-config-sync` label to the
PR to bypass the check when the divergence is intentional.

The list of changed/added files is read from the `CHANGED_FILES` /
`ADDED_FILES` environment variables (whitespace separated, as produced by
`tj-actions/changed-files`). For local testing the same values can be passed
as positional arguments / `--added-file` options instead.
"""

from __future__ import annotations

import argparse
import os
from pathlib import Path
import sys
from typing import List
from typing import Optional

# Package directory whose immediate subdirectories are scanned for config dirs.
DEFAULT_PACKAGE_DIR = Path("autoware_launch")
# A subdirectory is treated as a config directory if it contains such a file.
PARAM_GLOB = "*.param.yaml"


def discover_config_roots(package_dir: Path) -> List[Path]:
    if not package_dir.is_dir():
        return []
    roots = [
        child
        for child in package_dir.iterdir()
        if child.is_dir() and next(child.rglob(PARAM_GLOB), None) is not None
    ]
    return sorted(roots)


def relative_within(path: Path, root: Path) -> Optional[Path]:
    try:
        return path.relative_to(root)
    except ValueError:
        return None


def split_env(name: str) -> List[str]:
    return os.environ.get(name, "").split()


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--package-dir",
        type=Path,
        default=DEFAULT_PACKAGE_DIR,
        help=f"package dir to scan for config dirs (default: {DEFAULT_PACKAGE_DIR})",
    )
    parser.add_argument(
        "--root",
        action="append",
        default=[],
        help="config dir to compare, disables auto-discovery (may be repeated)",
    )
    parser.add_argument(
        "--added-file",
        action="append",
        default=[],
        help="a file added in the PR (may be repeated); defaults to $ADDED_FILES",
    )
    parser.add_argument(
        "changed_files",
        nargs="*",
        help="files changed in the PR; defaults to $CHANGED_FILES",
    )
    args = parser.parse_args()

    changed_files = args.changed_files or split_env("CHANGED_FILES")
    added_files = args.added_file or split_env("ADDED_FILES")

    if args.root:
        config_roots = sorted(Path(r) for r in args.root)
    else:
        config_roots = discover_config_roots(args.package_dir)

    if len(config_roots) < 2:
        print(
            f"Found fewer than 2 config directories under '{args.package_dir}'. "
            "Nothing to check."
        )
        return 0

    print("Config directories to keep in sync:")
    for root in config_roots:
        print(f"  - {root}")
    print("")

    changed = {Path(f) for f in changed_files}
    added = {Path(f) for f in added_files}

    problems: List[str] = []
    for changed_file in sorted(changed):
        for root in config_roots:
            rel = relative_within(changed_file, root)
            if rel is None:
                continue
            # changed_file belongs to exactly one config root.
            for other in config_roots:
                if other == root:
                    continue
                counterpart = other / rel
                if counterpart in changed:
                    continue  # already updated together
                if counterpart.exists():
                    problems.append(
                        f"{changed_file} was changed, but its counterpart "
                        f"{counterpart} was NOT."
                    )
                elif changed_file in added:
                    problems.append(
                        f"{changed_file} was added, but the counterpart "
                        f"{counterpart} does not exist."
                    )
            break

    if not problems:
        print("All config directories are in sync. ✅")
        return 0

    print("::error::Parameter changes are not reflected across all config directories.")
    print("")
    print("The following changes were made to only one config directory:")
    for problem in problems:
        print(f"  - {problem}")
    print("")
    print(
        "Please apply the same change to the counterpart file(s), or add the "
        "'ignore-config-sync' label to this PR if the divergence is intentional."
    )
    return 1


if __name__ == "__main__":
    sys.exit(main())
