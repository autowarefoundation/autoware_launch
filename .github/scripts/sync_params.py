#!/usr/bin/env python3

from __future__ import annotations

import argparse
import copy
from dataclasses import dataclass
import difflib
import hashlib
import io
from pathlib import Path
import re
import shutil
import subprocess
import sys
import tempfile
from typing import Any
from typing import Iterable

from ruamel.yaml import YAML
from ruamel.yaml.comments import CommentedMap
from ruamel.yaml.comments import CommentedSeq

OVERRIDE_MARKER = "{OVERRIDE}"
_OVERRIDE_MARKER_PATTERN = re.compile(r"\{OVERRIDE(?:\s*:\s*([^{}]*?))?\}")
ORIGINAL_PIVOT = "###### ORIGINAL (DO NOT EDIT) ######"


class SyncError(RuntimeError):
    """Raised when sync configuration or processing is invalid."""


@dataclass(frozen=True)
class VariantEntry:
    path: str


@dataclass(frozen=True)
class FileEntry:
    source: str
    variants: tuple[VariantEntry, ...]


@dataclass(frozen=True)
class RepositoryEntry:
    repository: str
    ref: str | None
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
    return _extract_override_marker(comment_value) is not None


def _normalize_override_marker(reason: str | None) -> str:
    if reason is None or not reason.strip():
        return OVERRIDE_MARKER
    return f"{{OVERRIDE: {reason.strip()}}}"


def _extract_override_marker(text: str) -> str | None:
    match = _OVERRIDE_MARKER_PATTERN.search(text)
    if match is None:
        return None
    return _normalize_override_marker(match.group(1))


def _strip_leading_override_marker(text: str) -> str:
    match = _OVERRIDE_MARKER_PATTERN.match(text)
    if match is None:
        return text
    return text[match.end() :].lstrip(" ")


def _extract_comment_text(comment_obj: Any) -> str:
    comment_value = getattr(comment_obj, "value", str(comment_obj))
    normalized = comment_value.splitlines()[0].strip()
    if normalized.startswith("#"):
        normalized = normalized[1:].strip()
    marker = _extract_override_marker(normalized)
    return marker if marker is not None else normalized


def _extract_override_comment_from_line(line: str) -> str | None:
    _before_comment, sep, comment = line.partition("#")
    if not sep:
        return None
    normalized = comment.strip()
    return _extract_override_marker(normalized)


@dataclass(frozen=True)
class KeyLineInfo:
    line_idx: int
    indent: int
    has_inline_value: bool


def _find_flow_sequence_end_idx(lines: list[str], start_idx: int) -> int:
    depth = 0
    opened = False
    idx = start_idx
    while idx < len(lines):
        current_before_comment = lines[idx].split("#", 1)[0]
        for char in current_before_comment:
            if char == "[":
                depth += 1
                opened = True
            elif char == "]":
                depth -= 1
        if opened and depth <= 0:
            return idx + 1
        idx += 1
    raise SyncError("Unterminated flow sequence while applying override.")


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
    previous_flow_style = getattr(_YAML_RT, "default_flow_style", None)
    previous_width = getattr(_YAML_RT, "width", None)
    try:
        _YAML_RT.default_flow_style = True
        _YAML_RT.width = 10**9
        stream = io.StringIO()
        _YAML_RT.dump(value, stream)
        dumped = stream.getvalue().strip()
    finally:
        _YAML_RT.default_flow_style = previous_flow_style
        _YAML_RT.width = previous_width
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
        new_value_text = f"{key_prefix}: {_render_inline_yaml_value(value)}"
        original_comment_col = len(before_comment)
        spaces_to_preserve_col = max(1, original_comment_col - len(new_value_text))
        new_before_comment = f"{new_value_text}{' ' * spaces_to_preserve_col}"
        return f"{new_before_comment}#{after_comment}"
    new_before_comment = f"{key_prefix}: {_render_inline_yaml_value(value)}"
    return new_before_comment


def _replace_block_value(
    lines: list[str],
    key_line_idx: int,
    key_indent: int,
    value: Any,
    raw_flow_block_lines: list[str] | None = None,
) -> None:
    value_line_idx = _find_first_value_line(lines, key_line_idx, key_indent)
    if value_line_idx is None:
        raise SyncError(f"Override target key on line {key_line_idx + 1} has no value line.")
    value_line = lines[value_line_idx]
    value_indent = len(value_line) - len(value_line.lstrip(" "))
    value_stripped = value_line.lstrip()

    if value_stripped.startswith("["):
        end_idx = _find_flow_sequence_end_idx(lines, value_line_idx)
        if raw_flow_block_lines:
            raw_base_indent = len(raw_flow_block_lines[0]) - len(
                raw_flow_block_lines[0].lstrip(" ")
            )
            replacement: list[str] = []
            for raw_line in raw_flow_block_lines:
                stripped = raw_line.lstrip(" ")
                if stripped == "":
                    replacement.append("")
                    continue
                raw_indent = len(raw_line) - len(stripped)
                relative_indent = max(0, raw_indent - raw_base_indent)
                replacement.append((" " * (value_indent + relative_indent)) + stripped)
            lines[value_line_idx:end_idx] = replacement
            return
        lines[value_line_idx:end_idx] = [(" " * value_indent) + _render_inline_yaml_value(value)]
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


def apply_overrides_to_source_text(
    source_text: str,
    overrides: dict[tuple[str, ...], Any],
    override_flow_blocks: dict[tuple[str, ...], list[str]] | None = None,
) -> str:
    lines = source_text.splitlines()
    key_index = _build_key_line_index(source_text)
    for path, value in overrides.items():
        info = key_index.get(path)
        if info is None:
            raise SyncError(f"Override path {_format_path(path)} not found in source text.")
        if info.has_inline_value:
            lines[info.line_idx] = _replace_inline_value(lines[info.line_idx], value)
        else:
            _replace_block_value(
                lines,
                info.line_idx,
                info.indent,
                value,
                (override_flow_blocks or {}).get(path),
            )
    return "\n".join(lines) + ("\n" if source_text.endswith("\n") or lines else "")


def extract_flow_sequence_override_blocks_from_text(
    text: str, paths: Iterable[tuple[str, ...]]
) -> dict[tuple[str, ...], list[str]]:
    if not text:
        return {}
    lines = text.splitlines()
    key_index = _build_key_line_index(text)
    blocks: dict[tuple[str, ...], list[str]] = {}
    for path in paths:
        info = key_index.get(path)
        if info is None or info.has_inline_value:
            continue
        value_line_idx = _find_first_value_line(lines, info.line_idx, info.indent)
        if value_line_idx is None:
            continue
        if not lines[value_line_idx].lstrip().startswith("["):
            continue
        end_idx = _find_flow_sequence_end_idx(lines, value_line_idx)
        blocks[path] = lines[value_line_idx:end_idx]
    return blocks


def ensure_override_markers_in_text(
    text: str,
    override_comments: dict[tuple[str, ...], str],
    override_comment_columns: dict[tuple[str, ...], int] | None = None,
) -> str:
    lines = text.splitlines()
    key_index = _build_key_line_index(text)
    for path, comment_text in override_comments.items():
        info = key_index.get(path)
        if info is None:
            raise SyncError(f"Override path {_format_path(path)} not found while adding marker.")
        line = lines[info.line_idx]
        desired_col = (override_comment_columns or {}).get(path)
        before_comment, sep, existing_comment = line.partition("#")
        if sep:
            # Preserve the source inline comment text and spacing exactly,
            # prefixing only the override marker token.
            comment_tail = existing_comment
            stripped = comment_tail.lstrip(" ")
            leading_spaces = comment_tail[: len(comment_tail) - len(stripped)]
            existing_marker = _extract_override_marker(stripped)
            stripped = _strip_leading_override_marker(stripped)
            stripped = stripped.rstrip(" ")
            comment_tail_after_marker = f"{leading_spaces}{stripped}".rstrip(" ")

            next_non_empty_idx = info.line_idx + 1
            while next_non_empty_idx < len(lines) and lines[next_non_empty_idx].strip() == "":
                next_non_empty_idx += 1
            if next_non_empty_idx < len(lines):
                next_line = lines[next_non_empty_idx].lstrip(" ")
                if next_line.startswith("#"):
                    next_comment_text = next_line[1:].strip()
                    if next_comment_text == stripped.strip():
                        comment_tail_after_marker = ""

            marker_from_comment = _extract_override_marker(comment_text)
            marker_token = (
                existing_marker
                if existing_marker is not None
                else (marker_from_comment if marker_from_comment is not None else OVERRIDE_MARKER)
            )
            comment_payload = f"# {marker_token}"
            if comment_tail_after_marker:
                comment_payload += comment_tail_after_marker
            if desired_col is not None:
                base = before_comment.rstrip(" ")
                spaces = max(1, desired_col - len(base))
                lines[info.line_idx] = f"{base}{' ' * spaces}{comment_payload}"
            else:
                lines[info.line_idx] = f"{before_comment}{comment_payload}"
            continue
        marker = _extract_override_marker(comment_text)
        suffix = marker if marker is not None else _normalize_override_marker(comment_text)
        if desired_col is not None:
            spaces = max(1, desired_col - len(line))
            lines[info.line_idx] = f"{line}{' ' * spaces}# {suffix}"
        else:
            lines[info.line_idx] = f"{line} # {suffix}"
    return "\n".join(lines) + ("\n" if text.endswith("\n") or lines else "")


def parse_override_comment_columns_from_variant_text(text: str) -> dict[tuple[str, ...], int]:
    lines = text.splitlines()
    key_index = _build_key_line_index(text)
    columns: dict[tuple[str, ...], int] = {}
    for path, info in key_index.items():
        line = lines[info.line_idx]
        comment = _extract_override_comment_from_line(line)
        if comment is None:
            continue
        columns[path] = line.index("#")
    return columns


def parse_override_comments_from_variant_text(text: str) -> dict[tuple[str, ...], str]:
    """
    Parse inline override markers from YAML comments using ruamel.

    Supports both:
    - `some_list: [1,2,3] # {OVERRIDE} ...`
    - `other_list: # {OVERRIDE} ...` followed by block list/mapping value
    """
    try:
        yaml_with_comments = _YAML_RT.load(text)
    except Exception as exc:  # pragma: no cover - narrowed by extract_override_values
        raise SyncError(f"Invalid YAML while parsing override markers: {exc}") from exc

    if yaml_with_comments is None:
        return {}

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

    # Ruamel can drop key-line inline comments when the value is a multi-line
    # flow sequence (e.g., `key: # {OVERRIDE}` followed by `[ ... ]`).
    # Fall back to line-based extraction for those cases.
    lines = text.splitlines()
    key_index = _build_key_line_index(text)
    for path, info in key_index.items():
        if path in override_comments:
            continue
        inline_override_comment = _extract_override_comment_from_line(lines[info.line_idx])
        if inline_override_comment is not None:
            override_comments[path] = inline_override_comment

    return override_comments


def parse_override_paths_from_variant_text(text: str) -> list[tuple[str, ...]]:
    return list(parse_override_comments_from_variant_text(text).keys())


def _expect_mapping(value: Any, context: str) -> dict[str, Any]:
    if not isinstance(value, dict):
        raise SyncError(f"{context} must be a mapping.")
    return value


def _parse_repository_entries(entries_data: Any, context: str) -> list[RepositoryEntry]:
    if not isinstance(entries_data, list):
        raise SyncError(f"{context} must be a list.")

    repositories: list[RepositoryEntry] = []
    for repo_index, repo_item in enumerate(entries_data):
        repo_map = _expect_mapping(repo_item, f"repository entry #{repo_index} in {context}")
        repository = repo_map.get("repository")
        ref = repo_map.get("ref")
        files_data = repo_map.get("files", [])
        if not isinstance(repository, str) or not repository:
            raise SyncError(
                f"repository entry #{repo_index} in {context} has invalid 'repository'."
            )
        if ref is not None and (not isinstance(ref, str) or not ref):
            raise SyncError(f"repository entry #{repo_index} in {context} has invalid 'ref'.")
        if not isinstance(files_data, list):
            raise SyncError(f"repository entry #{repo_index} in {context} has invalid 'files'.")

        files: list[FileEntry] = []
        for file_index, file_item in enumerate(files_data):
            file_map = _expect_mapping(file_item, f"file entry #{file_index} in {repository}")
            unknown_file_keys = set(file_map.keys()) - {"source", "variants"}
            if unknown_file_keys:
                raise SyncError(
                    f"File entry #{file_index} in {repository} has unsupported keys: "
                    f"{sorted(unknown_file_keys)}."
                )
            source = file_map.get("source")
            variants_data = file_map.get("variants", [])
            if not isinstance(source, str) or not source:
                raise SyncError(f"Invalid source in {repository} file entry #{file_index}.")
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

            files.append(FileEntry(source=source, variants=tuple(variants)))

        repositories.append(RepositoryEntry(repository=repository, ref=ref, files=tuple(files)))

    return repositories


def load_config(config_path: Path, category: str) -> list[RepositoryEntry]:
    try:
        config_data = _YAML_RT.load(config_path.read_text(encoding="utf-8"))
    except Exception as exc:
        raise SyncError(f"Invalid YAML in config '{config_path}': {exc}") from exc

    if not isinstance(config_data, dict):
        raise SyncError(f"Top-level config must be a mapping in '{config_path}'.")

    if category not in config_data:
        raise SyncError(
            f"Category '{category}' is not defined in '{config_path}'. "
            f"Available categories: {sorted(config_data.keys())}."
        )

    return _parse_repository_entries(config_data[category], f"category '{category}'")


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


def load_source_body_at_sha(repo_dir: Path, sha: str, source_path: str) -> str:
    process = subprocess.run(
        ["git", "show", f"{sha}:{source_path}"],
        cwd=str(repo_dir),
        check=False,
        capture_output=True,
        text=True,
    )
    if process.returncode != 0:
        raise SyncError(
            f"git show failed for {sha}:{source_path} (cwd={repo_dir}): "
            f"{process.stderr.strip()}"
        )
    text = process.stdout
    return text if text.endswith("\n") else f"{text}\n"


def resolve_default_branch(repo: str) -> str:
    output = run_git(["ls-remote", "--symref", f"https://github.com/{repo}.git", "HEAD"])
    for line in output.splitlines():
        if line.startswith("ref: ") and line.endswith("\tHEAD"):
            ref_name = line.split()[1]
            if ref_name.startswith("refs/heads/"):
                return ref_name.removeprefix("refs/heads/")
    raise SyncError(f"Failed to resolve default branch for repository '{repo}'.")


def resolve_ref(repo: str, ref: str | None) -> str:
    if ref:
        return ref
    return resolve_default_branch(repo)


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
        return _YAML_RT.load(path.read_text(encoding="utf-8"))
    except Exception as exc:
        raise SyncError(f"Invalid YAML file '{path}': {exc}") from exc


def source_blob_url(source_repo: str, source_sha: str, source_path: str) -> str:
    return f"https://github.com/{source_repo}/blob/{source_sha}/{source_path}"


def build_variant_header(source_url: str) -> str:
    lines = [
        "# This file is managed by workflow.",
        f"# Keys marked with '{OVERRIDE_MARKER}' or '# {{OVERRIDE: reason}}' are persisted as variant overrides.",
        "# 'reason' can be any single-line text without braces.",
        f"# To keep local changes, annotate fields with '# {OVERRIDE_MARKER}' or '# {{OVERRIDE: reason}}'.",
        "#",
        f"# Source: {source_url}",
    ]
    return "\n".join(lines) + "\n\n"


def build_embedded_original_section(source_body: str) -> str:
    lines = source_body.splitlines()
    commented_lines = [f"# {line}" if line else "#" for line in lines]
    section = ["", f"# {ORIGINAL_PIVOT}", *commented_lines]
    return "\n".join(section) + "\n"


def extract_pinned_source_sha_and_path(variant_text: str) -> tuple[str, str] | None:
    for line in variant_text.splitlines():
        if not line.startswith("# Source: "):
            continue
        source_url = line.removeprefix("# Source: ").strip()
        if "/blob/" not in source_url:
            return None
        sha_and_path = source_url.split("/blob/", 1)[1]
        sha, sep, path = sha_and_path.partition("/")
        if not sep or not sha or not path:
            return None
        return sha, path
    return None


def extract_embedded_original_from_variant_text(text: str) -> str | None:
    lines = text.splitlines()
    pivot_line = f"# {ORIGINAL_PIVOT}"
    pivot_idx = next((idx for idx, line in enumerate(lines) if line.strip() == pivot_line), None)
    if pivot_idx is None:
        return None

    reconstructed: list[str] = []
    for line in lines[pivot_idx + 1 :]:
        if line == "#":
            reconstructed.append("")
            continue
        if line.startswith("# "):
            reconstructed.append(line[2:])
            continue
        if line.strip() == "":
            reconstructed.append("")
            continue
        raise SyncError(
            "Embedded original section is malformed; expected only commented lines "
            f"after '{ORIGINAL_PIVOT}'."
        )

    if not reconstructed:
        return ""
    return "\n".join(reconstructed) + "\n"


def embedded_original_matches_source(variant_text: str, source_body: str) -> bool:
    extracted = extract_embedded_original_from_variant_text(variant_text)
    return extracted is not None and extracted == source_body


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


def _build_unified_diff(path: Path, old_content: str | None, new_content: str) -> str:
    old_lines = old_content.splitlines(keepends=True) if old_content is not None else []
    new_lines = new_content.splitlines(keepends=True)
    cwd = Path.cwd()
    display_path = str(path)
    try:
        display_path = str(path.relative_to(cwd))
    except ValueError:
        pass

    diff_lines = list(
        difflib.unified_diff(
            old_lines,
            new_lines,
            fromfile=f"a/{display_path}",
            tofile=f"b/{display_path}",
            lineterm="",
            n=2,
        )
    )
    # Keep output concise in check mode.
    return "\n".join(diff_lines[:80])


def sync_file_entry(
    workspace_root: Path,
    repository: RepositoryEntry,
    file_entry: FileEntry,
    source_repo_dir: Path,
    check: bool,
) -> int:
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
    source_sha = last_modified_sha(source_repo_dir, file_entry.source)
    source_url = source_blob_url(repository.repository, source_sha, file_entry.source)

    source_body = source_text if source_text.endswith("\n") else f"{source_text}\n"
    embedded_original = build_embedded_original_section(source_body)

    variants_changed = 0
    for variant in file_entry.variants:
        variant_abs = workspace_root / variant.path
        variant_text = variant_abs.read_text(encoding="utf-8") if variant_abs.exists() else ""
        source_body_for_embedded_check = source_body
        pinned_sha_and_path = (
            extract_pinned_source_sha_and_path(variant_text) if variant_text else None
        )
        if pinned_sha_and_path:
            pinned_sha, pinned_source_path = pinned_sha_and_path
            if pinned_source_path != file_entry.source:
                raise SyncError(
                    f"Source path mismatch in '{variant.path}': header points to "
                    f"'{pinned_source_path}' but config source is '{file_entry.source}'."
                )
            source_body_for_embedded_check = load_source_body_at_sha(
                source_repo_dir, pinned_sha, pinned_source_path
            )

        if variant_text and not embedded_original_matches_source(
            variant_text, source_body_for_embedded_check
        ):
            if check:
                print(
                    "[sync] Embedded original drift detected in "
                    f"'{variant.path}' for source '{file_entry.source}'."
                )

        overrides, override_comments = extract_override_values(variant_abs)
        override_comment_columns = parse_override_comment_columns_from_variant_text(variant_text)
        if not overrides:
            variant_content = build_variant_header(source_url) + source_body
            variant_content += embedded_original
            changed = write_or_check(variant_abs, variant_content, check)
            if changed:
                variants_changed += 1
                if check:
                    old_content = (
                        variant_abs.read_text(encoding="utf-8") if variant_abs.exists() else None
                    )
                    diff_text = _build_unified_diff(variant_abs, old_content, variant_content)
                    print(f"[sync] Drift detail: {variant.path}")
                    if diff_text:
                        print(diff_text)
            continue

        active_overrides: dict[tuple[str, ...], Any] = {}
        stale_paths: list[tuple[str, ...]] = []
        for override_path, override_value in overrides.items():
            try:
                get_value_at_path(source_yaml, override_path)
                active_overrides[override_path] = override_value
            except KeyError:
                stale_paths.append(override_path)

        if stale_paths:
            for p in stale_paths:
                print(
                    f"[sync] Stale override {_format_path(p)} in '{variant.path}' "
                    f"no longer exists in source '{file_entry.source}'."
                )
            if check:
                variants_changed += 1
                continue

        override_flow_blocks = extract_flow_sequence_override_blocks_from_text(
            variant_text, active_overrides.keys()
        )

        active_override_comments = {
            p: c for p, c in override_comments.items() if p in active_overrides
        }
        active_override_comment_columns = {
            p: c for p, c in override_comment_columns.items() if p in active_overrides
        }

        variant_body = apply_overrides_to_source_text(
            source_body, active_overrides, override_flow_blocks
        )
        variant_body = ensure_override_markers_in_text(
            variant_body, active_override_comments, active_override_comment_columns
        )
        variant_content = build_variant_header(source_url) + variant_body
        variant_content += embedded_original
        changed = write_or_check(variant_abs, variant_content, check)
        if changed:
            variants_changed += 1
            if check:
                old_content = (
                    variant_abs.read_text(encoding="utf-8") if variant_abs.exists() else None
                )
                diff_text = _build_unified_diff(variant_abs, old_content, variant_content)
                print(f"[sync] Drift detail: {variant.path}")
                if diff_text:
                    print(diff_text)

    return variants_changed


def run_sync(
    config_path: Path,
    category: str,
    check: bool,
    verbose: bool = True,
) -> int:
    workspace_root = Path.cwd()
    repositories = load_config(config_path, category)

    total_repos = 0
    total_files = 0
    changed_variants = 0

    with tempfile.TemporaryDirectory(prefix="sync-params-") as tmp_dir_name:
        temp_root = Path(tmp_dir_name)
        for repo_entry in repositories:
            total_repos += 1
            resolved_ref = resolve_ref(repo_entry.repository, repo_entry.ref)
            if verbose:
                print(f"[sync] Cloning {repo_entry.repository}@{resolved_ref}")
            repo_dir = clone_repository(repo_entry.repository, resolved_ref, temp_root)

            for file_entry in repo_entry.files:
                total_files += 1
                if verbose:
                    print(f"[sync] Processing {repo_entry.repository}:{file_entry.source}")
                before_variants = changed_variants
                changed_variants_delta = sync_file_entry(
                    workspace_root=workspace_root,
                    repository=RepositoryEntry(
                        repository=repo_entry.repository, ref=resolved_ref, files=repo_entry.files
                    ),
                    file_entry=file_entry,
                    source_repo_dir=repo_dir,
                    check=check,
                )
                changed_variants += changed_variants_delta
                if not check and changed_variants > before_variants:
                    print(f"[sync] Updated variant(s) for source: {file_entry.source}")

    print(
        f"[sync] repositories={total_repos} files={total_files} variants_changed={changed_variants}"
    )
    if check and changed_variants > 0:
        print("[sync] Drift detected (details shown above).")
        return 1
    return 0


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Sync parameter files from external repositories.")
    parser.add_argument(
        "category",
        type=str,
        help="Config category to sync (for example: perception).",
    )
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
    return run_sync(
        config_path=config_path,
        category=args.category,
        check=args.check,
        verbose=True,
    )


if __name__ == "__main__":
    try:
        sys.exit(main())
    except SyncError as exc:
        print(f"[sync] ERROR: {exc}", file=sys.stderr)
        sys.exit(2)
