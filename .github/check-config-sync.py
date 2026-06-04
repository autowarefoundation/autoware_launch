#!/usr/bin/env python3
import argparse
import os
from pathlib import Path
import sys
from typing import List
from typing import Optional

"""
This module checks that a parameter change is reflected across all config
directories of `autoware_launch` (e.g. `config`, `config_lv2`, ...), which are
auto-discovered as any subdirectory holding a `*.param.yaml` file. When a file
is changed in only one of them, it exits non-zero so the PR status check fails.
Add the `ignore-config-sync` label to bypass an intentional divergence.
"""

PACKAGE_DIR = Path("autoware_launch")
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


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--package-dir", type=Path, default=PACKAGE_DIR)
    parser.add_argument("--added-file", action="append", default=[])
    parser.add_argument("--deleted-file", action="append", default=[])
    parser.add_argument("changed_files", nargs="*")
    args = parser.parse_args()

    changed = {Path(f) for f in (args.changed_files or split_env("CHANGED_FILES"))}
    added = {Path(f) for f in (args.added_file or split_env("ADDED_FILES"))}
    deleted = {Path(f) for f in (args.deleted_file or split_env("DELETED_FILES"))}
    touched = changed | deleted

    config_roots = discover_config_roots(args.package_dir)
    if len(config_roots) < 2:
        print(f"Less than 2 config directories under '{args.package_dir}'. Skipped.")
        return 0

    # A file touched under one config root must also be touched in the others.
    problems = []
    for touched_file in sorted(touched):
        for root in config_roots:
            rel = relative_within(touched_file, root)
            if rel is None:
                continue
            for other in config_roots:
                counterpart = other / rel
                if other == root or counterpart in touched:
                    continue
                if counterpart.exists():
                    problems.append(f"{touched_file} -> {counterpart} (counterpart not updated)")
                elif touched_file in added:
                    problems.append(f"{touched_file} -> {counterpart} (does not exist)")
            break

    if not problems:
        print("All config directories are in sync.")
        return 0

    print("::error::Parameter changes are not reflected across all config directories.")
    for problem in problems:
        print(f"  {problem}")
    print("Update the counterpart file(s), or add the 'ignore-config-sync' label.")
    return 1


if __name__ == "__main__":
    sys.exit(main())
