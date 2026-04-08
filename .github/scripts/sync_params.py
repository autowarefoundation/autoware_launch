#!/usr/bin/env python3

from __future__ import annotations

import argparse
import copy
from dataclasses import dataclass
import hashlib
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile
from typing import Any
from typing import Iterable

from ruamel.yaml import YAML
from ruamel.yaml.comments import CommentedMap
from ruamel.yaml.comments import CommentedSeq
import yaml

OVERRIDE_MARKER = "*OVERRIDE*"


class SyncError(RuntimeError):
    """Raised when sync configuration or processing is invalid."""


@dataclass(frozen=True)
class VariantEntry:
    path: str


@dataclass(frozen=True)
class FileEntry:
    source: str
    dest: str
    variants: tuple[VariantEntry, ...]


@dataclass(frozen=True)
class RepositoryEntry:
    repository: str
    ref: str
    files: tuple[FileEntry, ...]


def _is_scalar(value: Any) -> bool:
    return value is None or isinstance(value, (bool, int, float, str))


def _is_scalar_or_scalar_array(value: Any) -> bool:
    if _is_scalar(value):
        return True
    if isinstance(value, list):
        return all(_is_scalar(v) for v in value)
    return False


def _format_path(path: Iterable[str]) -> str:
    return "".join(f"[{p}]" for p in path)


def _strip_quotes(value: str) -> str:
    if len(value) >= 2 and value[0] == value[-1] and value[0] in {"'", '"'}:
        return value[1:-1]
    return value


_YAML_RT = YAML(typ="rt")
_YAML_RT.preserve_quotes = True
_YAML_RT.indent(mapping=2, sequence=4, offset=2)


def _comment_has_override_marker(comment_obj: Any) -> bool:
    if comment_obj is None:
        return False
    comment_value = getattr(comment_obj, "value", str(comment_obj))
    return OVERRIDE_MARKER in comment_value


def _extract_comment_text(comment_obj: Any) -> str:
    comment_value = getattr(comment_obj, "value", str(comment_obj))
    normalized = comment_value.strip()
    if normalized.startswith("#"):
        normalized = normalized[1:].strip()
    return normalized


@dataclass(frozen=True)
class KeyLineInfo:
    line_idx: int
    indent: int
    has_inline_value: bool


def _build_key_line_index(text: str) -> dict[tuple[str, ...], KeyLineInfo]:
    lines = text.splitlines()
    index: dict[tuple[str, ...], KeyLineInfo] = {}
    stack: list[tuple[int, str]] = []

    for idx, line in enumerate(lines):
        before_comment = line.split("#", 1)[0]
        stripped = before_comment.lstrip()
        if not stripped or stripped.startswith("- ") or ":" not in stripped:
            continue
        indent = len(before_comment) - len(stripped)
        key_part, value_part = stripped.split(":", 1)
        key = _strip_quotes(key_part.strip())
        if not key:
            continue
        while stack and indent <= stack[-1][0]:
            stack.pop()
        path = tuple(p for _, p in stack) + (key,)
        has_inline_value = value_part.strip() != ""
        index[path] = KeyLineInfo(line_idx=idx, indent=indent, has_inline_value=has_inline_value)
        if not has_inline_value:
            stack.append((indent, key))
    return index


def _render_inline_yaml_value(value: Any) -> str:
    dumped = yaml.safe_dump(
        value,
        default_flow_style=True,
        width=10**9,
        allow_unicode=False,
        sort_keys=False,
    ).strip()
    return dumped.splitlines()[0] if dumped else "null"


def _find_first_value_line(lines: list[str], key_line_idx: int, key_indent: int) -> int | None:
    idx = key_line_idx + 1
    while idx < len(lines):
        line = lines[idx]
        stripped = line.strip()
        if stripped == "":
            idx += 1
            continue
        indent = len(line) - len(line.lstrip(" "))
        if indent <= key_indent:
            return None
        if line.lstrip().startswith("#"):
            idx += 1
            continue
        return idx
    return None


def _replace_inline_value(line: str, value: Any) -> str:
    before_comment, sep, after_comment = line.partition("#")
    key_prefix = before_comment.split(":", 1)[0]
    if sep:
        trailing_spaces = len(before_comment) - len(before_comment.rstrip(" "))
        new_before_comment = f"{key_prefix}: {_render_inline_yaml_value(value)}" + (
            " " * trailing_spaces
        )
        return f"{new_before_comment}#{after_comment}"
    new_before_comment = f"{key_prefix}: {_render_inline_yaml_value(value)}"
    return new_before_comment


def _replace_block_value(lines: list[str], key_line_idx: int, key_indent: int, value: Any) -> None:
    value_line_idx = _find_first_value_line(lines, key_line_idx, key_indent)
    if value_line_idx is None:
        raise SyncError(f"Override target key on line {key_line_idx + 1} has no value line.")
    value_line = lines[value_line_idx]
    value_indent = len(value_line) - len(value_line.lstrip(" "))
    value_stripped = value_line.lstrip()

    if value_stripped.startswith("["):
        lines[value_line_idx] = (" " * value_indent) + _render_inline_yaml_value(value)
        return

    if value_stripped.startswith("-"):
        if not isinstance(value, list):
            raise SyncError("Cannot apply scalar override to block sequence value.")
        end_idx = value_line_idx + 1
        while end_idx < len(lines):
            line = lines[end_idx]
            stripped = line.strip()
            if stripped == "":
                end_idx += 1
                continue
            indent = len(line) - len(line.lstrip(" "))
            if indent <= key_indent:
                break
            end_idx += 1
        replacement = [(" " * value_indent) + f"- {_render_inline_yaml_value(v)}" for v in value]
        lines[value_line_idx:end_idx] = replacement
        return

    lines[value_line_idx] = (" " * value_indent) + _render_inline_yaml_value(value)


def apply_overrides_to_source_text(source_text: str, overrides: dict[tuple[str, ...], Any]) -> str:
    lines = source_text.splitlines()
    key_index = _build_key_line_index(source_text)
    for path, value in overrides.items():
        info = key_index.get(path)
        if info is None:
            raise SyncError(f"Override path {_format_path(path)} not found in source text.")
        if info.has_inline_value:
            lines[info.line_idx] = _replace_inline_value(lines[info.line_idx], value)
        else:
            _replace_block_value(lines, info.line_idx, info.indent, value)
    return "\n".join(lines) + ("\n" if source_text.endswith("\n") or lines else "")


def ensure_override_markers_in_text(
    text: str, override_comments: dict[tuple[str, ...], str]
) -> str:
    lines = text.splitlines()
    key_index = _build_key_line_index(text)
    for path, comment_text in override_comments.items():
        info = key_index.get(path)
        if info is None:
            raise SyncError(f"Override path {_format_path(path)} not found while adding marker.")
        line = lines[info.line_idx]
        before_comment, sep, existing_comment = line.partition("#")
        if sep:
            # Preserve the source inline comment text and spacing exactly,
            # prefixing only the override marker token.
            comment_tail = existing_comment
            stripped = comment_tail.lstrip(" ")
            leading_spaces = comment_tail[: len(comment_tail) - len(stripped)]
            if stripped.startswith(OVERRIDE_MARKER):
                stripped = stripped[len(OVERRIDE_MARKER) :]
                stripped = stripped.lstrip(" ")
            lines[info.line_idx] = f"{before_comment}# {OVERRIDE_MARKER}{leading_spaces}{stripped}"
            continue
        suffix = comment_text if comment_text else OVERRIDE_MARKER
        lines[info.line_idx] = f"{line} # {suffix}"
    return "\n".join(lines) + ("\n" if text.endswith("\n") or lines else "")


def dedupe_nearby_identical_comment_lines(text: str) -> str:
    lines = text.splitlines()
    out: list[str] = []
    i = 0
    while i < len(lines):
        line = lines[i]
        out.append(line)
        stripped = line.lstrip(" ")
        if not stripped.startswith("#"):
            i += 1
            continue
        indent = len(line) - len(stripped)
        comment_text = stripped
        j = i + 1
        while j < len(lines):
            candidate = lines[j]
            candidate_stripped = candidate.strip()
            if candidate_stripped == "":
                j += 1
                continue
            c = lines[j].lstrip(" ")
            c_indent = len(lines[j]) - len(c)
            if c.startswith("#") and c_indent == indent and c == comment_text:
                i = j
            break
        i += 1
    return "\n".join(out) + ("\n" if text.endswith("\n") or out else "")


def parse_override_comments_from_variant_text(text: str) -> dict[tuple[str, ...], str]:
    """
    Parse inline override markers from YAML comments using ruamel.

    Supports both:
    - `some_list: [1,2,3] # *OVERRIDE* ...`
    - `other_list: # *OVERRIDE* ...` followed by block list/mapping value
    """
    try:
        yaml_with_comments = _YAML_RT.load(text)
    except Exception as exc:  # pragma: no cover - narrowed by extract_override_values
        raise SyncError(f"Invalid YAML while parsing override markers: {exc}") from exc

    if yaml_with_comments is None:
        return []

    override_comments: dict[tuple[str, ...], str] = {}

    def walk(node: Any, path_prefix: tuple[str, ...]) -> None:
        if isinstance(node, CommentedMap):
            for key, value in node.items():
                key_path = path_prefix + (str(key),)
                key_comments = node.ca.items.get(key)
                eol_comment = key_comments[2] if key_comments and len(key_comments) > 2 else None
                if _comment_has_override_marker(eol_comment):
                    override_comments[key_path] = _extract_comment_text(eol_comment)
                walk(value, key_path)
        elif isinstance(node, CommentedSeq):
            for item in node:
                walk(item, path_prefix)

    walk(yaml_with_comments, ())
    return override_comments


def parse_override_paths_from_variant_text(text: str) -> list[tuple[str, ...]]:
    return list(parse_override_comments_from_variant_text(text).keys())


def _expect_mapping(value: Any, context: str) -> dict[str, Any]:
    if not isinstance(value, dict):
        raise SyncError(f"{context} must be a mapping.")
    return value


def load_config(config_path: Path) -> list[RepositoryEntry]:
    try:
        config_data = yaml.safe_load(config_path.read_text(encoding="utf-8"))
    except yaml.YAMLError as exc:
        raise SyncError(f"Invalid YAML in config '{config_path}': {exc}") from exc

    if not isinstance(config_data, list):
        raise SyncError(f"Top-level config must be a list in '{config_path}'.")

    repositories: list[RepositoryEntry] = []
    for repo_index, repo_item in enumerate(config_data):
        repo_map = _expect_mapping(repo_item, f"repository entry #{repo_index}")
        repository = repo_map.get("repository")
        ref = repo_map.get("ref")
        files_data = repo_map.get("files", [])
        if not isinstance(repository, str) or not repository:
            raise SyncError(f"repository entry #{repo_index} has invalid 'repository'.")
        if not isinstance(ref, str) or not ref:
            raise SyncError(f"repository entry #{repo_index} has invalid 'ref'.")
        if not isinstance(files_data, list):
            raise SyncError(f"repository entry #{repo_index} has invalid 'files'.")

        files: list[FileEntry] = []
        for file_index, file_item in enumerate(files_data):
            file_map = _expect_mapping(file_item, f"file entry #{file_index} in {repository}")
            source = file_map.get("source")
            dest = file_map.get("dest")
            variants_data = file_map.get("variants", [])
            if not isinstance(source, str) or not source:
                raise SyncError(f"Invalid source in {repository} file entry #{file_index}.")
            if not isinstance(dest, str) or not dest:
                raise SyncError(f"Invalid dest in {repository} file entry #{file_index}.")
            if not isinstance(variants_data, list):
                raise SyncError(f"Invalid variants list for {source} in {repository}.")

            variants: list[VariantEntry] = []
            for variant_index, variant_item in enumerate(variants_data):
                if not isinstance(variant_item, dict):
                    raise SyncError(
                        f"Variant entry #{variant_index} for {source} in {repository} "
                        "must be an object with 'path'."
                    )
                unknown_keys = set(variant_item.keys()) - {"path"}
                if unknown_keys:
                    raise SyncError(
                        f"Variant entry #{variant_index} for {source} in {repository} "
                        f"has unsupported keys: {sorted(unknown_keys)}."
                    )
                variant_path = variant_item.get("path")
                if not isinstance(variant_path, str) or not variant_path:
                    raise SyncError(
                        f"Variant entry #{variant_index} for {source} in {repository} "
                        "must define non-empty string 'path'."
                    )
                variants.append(VariantEntry(path=variant_path))

            files.append(FileEntry(source=source, dest=dest, variants=tuple(variants)))

        repositories.append(RepositoryEntry(repository=repository, ref=ref, files=tuple(files)))

    return repositories


def run_git(args: list[str], cwd: Path | None = None) -> str:
    process = subprocess.run(
        ["git", *args],
        cwd=str(cwd) if cwd else None,
        check=False,
        capture_output=True,
        text=True,
    )
    if process.returncode != 0:
        raise SyncError(
            f"git {' '.join(args)} failed"
            + (f" (cwd={cwd})" if cwd else "")
            + f": {process.stderr.strip()}"
        )
    return process.stdout.strip()


def clone_repository(repo: str, ref: str, temp_root: Path) -> Path:
    repo_name = repo.replace("/", "__")
    repo_dir = temp_root / f"{repo_name}_{hashlib.sha1(f'{repo}@{ref}'.encode()).hexdigest()[:8]}"
    if repo_dir.exists():
        shutil.rmtree(repo_dir)
    # Full history is required to compute per-file last-modified SHA accurately.
    run_git(["clone", "--branch", ref, f"https://github.com/{repo}.git", str(repo_dir)])
    run_git(["checkout", ref], cwd=repo_dir)
    return repo_dir


def last_modified_sha(repo_dir: Path, source_path: str) -> str:
    sha = run_git(["log", "-n", "1", "--format=%H", "--", source_path], cwd=repo_dir)
    if not sha:
        raise SyncError(f"No commit found for source file '{source_path}'.")
    return sha


def load_yaml_file(path: Path) -> Any:
    try:
        return yaml.safe_load(path.read_text(encoding="utf-8"))
    except yaml.YAMLError as exc:
        raise SyncError(f"Invalid YAML file '{path}': {exc}") from exc


def dump_yaml(data: Any) -> str:
    return yaml.safe_dump(data, sort_keys=False, default_flow_style=False, allow_unicode=False)


def source_blob_url(source_repo: str, source_sha: str, source_path: str) -> str:
    return f"https://github.com/{source_repo}/blob/{source_sha}/{source_path}"


def build_dest_header(source_url: str, variant_paths: Iterable[str]) -> str:
    lines = [
        "# This file is generated by workflow. Do not modify directly.",
        f"# Source: {source_url}",
        "# Variant files:",
    ]
    variant_list = list(variant_paths)
    if variant_list:
        lines.extend([f"# - {path}" for path in variant_list])
    else:
        lines.append("# - (none)")
    return "\n".join(lines) + "\n\n"


def build_variant_header(source_url: str, dest_path: str) -> str:
    lines = [
        "# This file is generated by workflow. Do not modify directly.",
        f"# Source: {source_url}",
        f"# Destination sync file in this repository: {dest_path}",
        f"# Keys marked with '{OVERRIDE_MARKER}' are persisted as variant overrides.",
    ]
    return "\n".join(lines) + "\n\n"


def get_value_at_path(data: Any, path: tuple[str, ...]) -> Any:
    cursor = data
    for segment in path:
        if not isinstance(cursor, dict) or segment not in cursor:
            raise KeyError(segment)
        cursor = cursor[segment]
    return cursor


def set_value_at_path(data: Any, path: tuple[str, ...], value: Any) -> None:
    cursor = data
    for segment in path[:-1]:
        if not isinstance(cursor, dict) or segment not in cursor:
            raise KeyError(segment)
        cursor = cursor[segment]
    if not isinstance(cursor, dict) or path[-1] not in cursor:
        raise KeyError(path[-1])
    cursor[path[-1]] = copy.deepcopy(value)


def extract_override_values(
    variant_path: Path,
) -> tuple[dict[tuple[str, ...], Any], dict[tuple[str, ...], str]]:
    if not variant_path.exists():
        return {}, {}

    text = variant_path.read_text(encoding="utf-8")
    override_comments = parse_override_comments_from_variant_text(text)
    if not override_comments:
        return {}, {}

    yaml_data = load_yaml_file(variant_path)
    if yaml_data is None:
        yaml_data = {}

    overrides: dict[tuple[str, ...], Any] = {}
    for path in override_comments:
        try:
            value = get_value_at_path(yaml_data, path)
        except KeyError as exc:
            raise SyncError(
                f"Override path {_format_path(path)} marked in '{variant_path}' "
                "does not exist in current variant content."
            ) from exc
        if not _is_scalar_or_scalar_array(value):
            raise SyncError(
                f"Override path {_format_path(path)} in '{variant_path}' must be a leaf "
                "scalar or array of scalars."
            )
        overrides[path] = copy.deepcopy(value)
    return overrides, override_comments


def write_or_check(path: Path, new_content: str, check: bool) -> bool:
    old_content = path.read_text(encoding="utf-8") if path.exists() else None
    if old_content == new_content:
        return False
    if check:
        return True
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(new_content, encoding="utf-8")
    return True


def sync_file_entry(
    workspace_root: Path,
    repository: RepositoryEntry,
    file_entry: FileEntry,
    source_repo_dir: Path,
    check: bool,
) -> tuple[int, int]:
    source_abs = source_repo_dir / file_entry.source
    if not source_abs.exists():
        raise SyncError(
            f"Source file does not exist: repository={repository.repository} "
            f"ref={repository.ref} source={file_entry.source}"
        )

    source_text = source_abs.read_text(encoding="utf-8")
    source_yaml = load_yaml_file(source_abs)
    if source_yaml is None:
        source_yaml = {}
    source_rt_yaml = _YAML_RT.load(source_text)
    if source_rt_yaml is None:
        source_rt_yaml = CommentedMap()
    source_sha = last_modified_sha(source_repo_dir, file_entry.source)
    source_url = source_blob_url(repository.repository, source_sha, file_entry.source)

    dest_abs = workspace_root / file_entry.dest
    source_body = source_text if source_text.endswith("\n") else f"{source_text}\n"
    dest_content = (
        build_dest_header(source_url, [variant.path for variant in file_entry.variants])
        + source_body
    )
    dest_changed = write_or_check(dest_abs, dest_content, check)

    variants_changed = 0
    for variant in file_entry.variants:
        variant_abs = workspace_root / variant.path
        overrides, override_comments = extract_override_values(variant_abs)
        if not overrides:
            variant_content = build_variant_header(source_url, file_entry.dest) + source_body
            if write_or_check(variant_abs, variant_content, check):
                variants_changed += 1
            continue
        variant_yaml = copy.deepcopy(source_yaml)

        for override_path, override_value in overrides.items():
            try:
                get_value_at_path(source_yaml, override_path)
            except KeyError as exc:
                raise SyncError(
                    f"Override path {_format_path(override_path)} in '{variant.path}' "
                    f"is missing from source '{file_entry.source}'."
                ) from exc
            set_value_at_path(variant_yaml, override_path, override_value)

        variant_body = apply_overrides_to_source_text(source_body, overrides)
        variant_body = ensure_override_markers_in_text(variant_body, override_comments)
        variant_body = dedupe_nearby_identical_comment_lines(variant_body)
        variant_content = build_variant_header(source_url, file_entry.dest) + variant_body
        if write_or_check(variant_abs, variant_content, check):
            variants_changed += 1

    return int(dest_changed), variants_changed


def run_sync(config_path: Path, check: bool, verbose: bool = True) -> int:
    workspace_root = Path.cwd()
    repositories = load_config(config_path)

    total_repos = 0
    total_files = 0
    changed_dest = 0
    changed_variants = 0
    drift_paths: list[str] = []

    with tempfile.TemporaryDirectory(prefix="sync-params-") as tmp_dir_name:
        temp_root = Path(tmp_dir_name)
        for repo_entry in repositories:
            total_repos += 1
            if verbose:
                print(f"[sync] Cloning {repo_entry.repository}@{repo_entry.ref}")
            repo_dir = clone_repository(repo_entry.repository, repo_entry.ref, temp_root)

            for file_entry in repo_entry.files:
                total_files += 1
                if verbose:
                    print(
                        "[sync] Processing "
                        f"{repo_entry.repository}:{file_entry.source} -> {file_entry.dest}"
                    )
                before_dest = changed_dest
                before_variants = changed_variants
                changed_dest_delta, changed_variants_delta = sync_file_entry(
                    workspace_root=workspace_root,
                    repository=repo_entry,
                    file_entry=file_entry,
                    source_repo_dir=repo_dir,
                    check=check,
                )
                changed_dest += changed_dest_delta
                changed_variants += changed_variants_delta
                if check and (changed_dest > before_dest or changed_variants > before_variants):
                    drift_paths.append(file_entry.dest)
                    drift_paths.extend(variant.path for variant in file_entry.variants)

    print(
        f"[sync] repositories={total_repos} files={total_files} "
        f"dest_changed={changed_dest} variants_changed={changed_variants}"
    )
    if check and (changed_dest > 0 or changed_variants > 0):
        unique_drift = sorted(set(drift_paths))
        print("[sync] Drift detected in:")
        for path in unique_drift:
            print(f"  - {path}")
        return 1
    return 0


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Sync parameter files from external repositories.")
    parser.add_argument(
        "--config",
        type=Path,
        default=Path(".github/sync-params.yaml"),
        help="Path to sync config YAML.",
    )
    parser.add_argument(
        "--check",
        action="store_true",
        help="Check for drift without writing files.",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    config_path = args.config if args.config.is_absolute() else (Path.cwd() / args.config)
    if not config_path.exists():
        raise SyncError(f"Config file does not exist: {config_path}")
    return run_sync(config_path=config_path, check=args.check, verbose=True)


if __name__ == "__main__":
    try:
        sys.exit(main())
    except SyncError as exc:
        print(f"[sync] ERROR: {exc}", file=sys.stderr)
        sys.exit(2)
