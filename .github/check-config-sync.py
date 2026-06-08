#!/usr/bin/env python3
import argparse
import dataclasses
import os
from pathlib import Path
import shutil
import sys
from typing import List
from typing import Optional
from typing import Set

"""
This module checks that a parameter change is reflected across the config
directories of `autoware_launch` listed in CONFIG_ROOTS. When a file is added,
modified, or deleted in only one of them, the comment body is written to the
`comment` step output, `status` is set to `mismatch`, and the script exits
non-zero so the PR status check also fails.

With `--apply` the one-sided changes are instead mirrored to the corresponding
directory (run when the `apply-config-sync` label is added) and the summary is
written to the `comment` step output.

If fewer than two config directories exist (e.g. only `config` is present), the
check is skipped. Add the `ignore-config-sync` label to bypass an intentional
divergence.
"""

# Config directories to keep in sync. Edit this list when a directory is added.
CONFIG_ROOTS = [
    Path("autoware_launch/config"),
    Path("autoware_launch/config_lv2"),
]

IGNORE_LABEL = "ignore-config-sync"
APPLY_LABEL = "apply-config-sync"

OP_LABELS = {
    "added": "✨ Added",
    "deleted": "🗑️ Deleted",
    "modified": "✏️ Modified",
}


@dataclasses.dataclass
class Change:
    operation: str  # "added", "modified" or "deleted"
    source: Path  # the file changed in this PR
    counterpart: Path  # the file that should receive the same change


def relative_within(path: Path, root: Path) -> Optional[Path]:
    try:
        return path.relative_to(root)
    except ValueError:
        return None


def config_root_of(path: Path) -> Path:
    for root in CONFIG_ROOTS:
        if root == path or root in path.parents:
            return root
    return path.parent


def split_env(name: str) -> Set[Path]:
    return {Path(f) for f in os.environ.get(name, "").split()}


def set_output(key: str, value: str) -> None:
    github_output = os.environ.get("GITHUB_OUTPUT")
    if not github_output:
        return
    with open(github_output, "a") as f:
        if "\n" in value:
            delimiter = "CONFIG_SYNC_EOF"
            f.write(f"{key}<<{delimiter}\n{value}\n{delimiter}\n")
        else:
            f.write(f"{key}={value}\n")


def op_label(operation: str) -> str:
    # Non-breaking space keeps the emoji and label on one line in the table.
    return OP_LABELS.get(operation, operation).replace(" ", "\N{NO-BREAK SPACE}")


def find_unsynced_changes(
    changed: Set[Path], added: Set[Path], deleted: Set[Path], config_roots: List[Path]
) -> List[Change]:
    touched = changed | deleted
    changes = []
    for path in sorted(touched):
        if path in deleted:
            operation = "deleted"
        elif path in added:
            operation = "added"
        else:
            operation = "modified"
        for root in config_roots:
            rel = relative_within(path, root)
            if rel is None:
                continue
            for other in config_roots:
                counterpart = other / rel
                if other == root or counterpart in touched:
                    continue
                if counterpart.exists() or path in added:
                    changes.append(Change(operation, path, counterpart))
            break
    return changes


def build_check_comment(changes: List[Change]) -> str:
    author = os.environ.get("PR_AUTHOR", "").strip()
    mention = f"@{author} " if author else ""
    lines = [
        f"{mention}thank you for the update! 🙏",
        "",
        "This PR changes parameter files in only one config directory. "
        "`autoware_launch` keeps its config directories in sync, so please make "
        "the same change to the same relative path in the corresponding directory:",
        "",
        "| Operation | File | Corresponding directory |",
        "| --- | --- | --- |",
    ]
    for change in changes:
        lines.append(
            f"| {op_label(change.operation)} | `{change.source}` "
            f"| `{config_root_of(change.counterpart)}` |"
        )
    lines += [
        "",
        f"To apply these changes to the corresponding directory automatically, add "
        f"the `{APPLY_LABEL}` label to this PR. If the divergence is intentional, "
        f"add the `{IGNORE_LABEL}` label to silence this check.",
    ]
    return "\n".join(lines) + "\n"


def build_apply_comment(changes: List[Change]) -> str:
    lines = [
        "🤖 Mirrored the change(s) to the corresponding config directory and "
        "pushed a commit. Please review it:",
        "",
        "| Operation | Updated file |",
        "| --- | --- |",
    ]
    for change in changes:
        lines.append(f"| {op_label(change.operation)} | `{change.counterpart}` |")
    return "\n".join(lines) + "\n"


def apply_changes(changes: List[Change]) -> List[Change]:
    applied = []
    for change in changes:
        if change.operation == "deleted":
            if change.counterpart.exists():
                change.counterpart.unlink()
                applied.append(change)
        else:  # added or modified
            change.counterpart.parent.mkdir(parents=True, exist_ok=True)
            shutil.copy2(change.source, change.counterpart)
            applied.append(change)
    return applied


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--apply",
        action="store_true",
        help="mirror one-sided changes to the corresponding directory",
    )
    args = parser.parse_args()

    changed = split_env("CHANGED_FILES")
    added = split_env("ADDED_FILES")
    deleted = split_env("DELETED_FILES")

    config_roots = [root for root in CONFIG_ROOTS if root.is_dir()]
    if len(config_roots) < 2:
        print("Fewer than two config directories present; skipping.")
        set_output("status", "ok")
        set_output("applied", "false")
        return 0

    changes = find_unsynced_changes(changed, added, deleted, config_roots)

    if args.apply:
        applied = apply_changes(changes)
        set_output("applied", "true" if applied else "false")
        if not applied:
            print("Nothing to apply.")
            return 0
        set_output("comment", build_apply_comment(applied))
        for change in applied:
            print(f"  [{change.operation}] {change.source} -> {change.counterpart}")
        return 0

    if not changes:
        print("All config directories are in sync.")
        set_output("status", "ok")
        return 0

    set_output("status", "mismatch")
    set_output("comment", build_check_comment(changes))
    print("::error::Parameter changes are not reflected across all config directories.")
    for change in changes:
        print(f"  [{change.operation}] {change.source} -> {change.counterpart}")
    print(f"Update the counterpart file(s), or add the '{IGNORE_LABEL}' label.")
    return 1


if __name__ == "__main__":
    sys.exit(main())
