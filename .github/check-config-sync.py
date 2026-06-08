#!/usr/bin/env python3
import argparse
import os
from pathlib import Path
import shutil
import sys
from typing import List
from typing import Optional

"""
This module checks that a parameter change is reflected across the config
directories of `autoware_launch` listed in CONFIG_ROOTS. When a file is added,
changed, or deleted in only one of them, it writes a PR comment body
(config-sync-comment.md), sets the `status` step output to `mismatch`, and exits
non-zero so the PR status check also fails.

With `--apply` it instead mirrors each one-sided change to the corresponding
directory (run when the `apply-config-sync` label is added) and writes the
`config-sync-applied.md` comment body. Add the `ignore-config-sync` label to
bypass an intentional divergence.
"""

# Config directories to keep in sync. Edit this list when a directory is added.
CONFIG_ROOTS = [
    Path("autoware_launch/config"),
    Path("autoware_launch/config_lv2"),
]

IGNORE_LABEL = "ignore-config-sync"
APPLY_LABEL = "apply-config-sync"

COMMENT_PATH = Path("config-sync-comment.md")
APPLIED_COMMENT_PATH = Path("config-sync-applied.md")

OP_LABELS = {
    "added": "✨ Added",
    "deleted": "🗑️ Deleted",
    "modified": "✏️ Modified",
}


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


def split_env(name: str) -> List[str]:
    return os.environ.get(name, "").split()


def set_output(key: str, value: str) -> None:
    github_output = os.environ.get("GITHUB_OUTPUT")
    if github_output:
        with open(github_output, "a") as f:
            f.write(f"{key}={value}\n")


def op_label(op: str) -> str:
    # Non-breaking space keeps the emoji and label on one line in the table.
    return OP_LABELS.get(op, op).replace(" ", "\N{NO-BREAK SPACE}")


def collect_problems(touched, added, deleted, config_roots):
    problems = []
    for touched_file in sorted(touched):
        if touched_file in deleted:
            op = "deleted"
        elif touched_file in added:
            op = "added"
        else:
            op = "modified"
        for root in config_roots:
            rel = relative_within(touched_file, root)
            if rel is None:
                continue
            for other in config_roots:
                counterpart = other / rel
                if other == root or counterpart in touched:
                    continue
                if counterpart.exists() or touched_file in added:
                    problems.append((op, touched_file, counterpart))
            break
    return problems


def build_comment(problems) -> str:
    author = os.environ.get("PR_AUTHOR", "").strip()
    mention = f"@{author} " if author else ""
    body = [
        f"{mention}thank you for the update! 🙏",
        "",
        "This PR changes parameter files in only one config directory. "
        "`autoware_launch` keeps its config directories in sync, so please make "
        "the same change to the same relative path in the corresponding directory:",
        "",
        "| Operation | File | Corresponding directory |",
        "| --- | --- | --- |",
    ]
    for op, src, counterpart in problems:
        body.append(f"| {op_label(op)} | `{src}` | `{config_root_of(counterpart)}` |")
    body += [
        "",
        f"To apply the same changes to the corresponding directory automatically, "
        f"add the `{APPLY_LABEL}` label to this PR. If the divergence is "
        f"intentional, add the `{IGNORE_LABEL}` label to silence this check.",
    ]
    return "\n".join(body) + "\n"


def build_applied_comment(applied) -> str:
    body = [
        "🤖 Automatically mirrored the change(s) to the corresponding config "
        "directory and pushed a commit. Please review it:",
        "",
        "| Operation | Updated file |",
        "| --- | --- |",
    ]
    for op, _src, counterpart in applied:
        body.append(f"| {op_label(op)} | `{counterpart}` |")
    return "\n".join(body) + "\n"


def apply_sync(problems):
    applied = []
    for op, src, counterpart in problems:
        if op == "deleted":
            if counterpart.exists():
                counterpart.unlink()
                applied.append((op, src, counterpart))
        else:  # added or modified
            counterpart.parent.mkdir(parents=True, exist_ok=True)
            shutil.copy2(src, counterpart)
            applied.append((op, src, counterpart))
    return applied


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--apply",
        action="store_true",
        help="mirror one-sided changes to the corresponding directory",
    )
    parser.add_argument("--added-file", action="append", default=[])
    parser.add_argument("--deleted-file", action="append", default=[])
    parser.add_argument("changed_files", nargs="*")
    args = parser.parse_args()

    changed = {Path(f) for f in (args.changed_files or split_env("CHANGED_FILES"))}
    added = {Path(f) for f in (args.added_file or split_env("ADDED_FILES"))}
    deleted = {Path(f) for f in (args.deleted_file or split_env("DELETED_FILES"))}
    touched = changed | deleted

    config_roots = [root for root in CONFIG_ROOTS if root.is_dir()]
    if len(config_roots) < 2:
        print("Less than 2 config directories present. Skipped.")
        set_output("status", "ok")
        set_output("applied", "false")
        return 0

    problems = collect_problems(touched, added, deleted, config_roots)

    if args.apply:
        applied = apply_sync(problems)
        set_output("applied", "true" if applied else "false")
        if not applied:
            print("Nothing to apply.")
            return 0
        APPLIED_COMMENT_PATH.write_text(build_applied_comment(applied))
        for op, src, counterpart in applied:
            print(f"  [{op}] {src} -> {counterpart}")
        return 0

    if not problems:
        print("All config directories are in sync.")
        set_output("status", "ok")
        return 0

    COMMENT_PATH.write_text(build_comment(problems))
    set_output("status", "mismatch")
    print("::error::Parameter changes are not reflected across all config directories.")
    for op, src, counterpart in problems:
        print(f"  [{op}] {src} -> {counterpart}")
    print(f"Update the counterpart file(s), or add the '{IGNORE_LABEL}' label.")
    return 1


if __name__ == "__main__":
    sys.exit(main())
