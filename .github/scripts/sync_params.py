#!/usr/bin/env python3
"""Sync parameter files from upstream repositories into this repository's variant files.

This script is the backend for the ``update-params`` and ``check-params`` GitHub Actions
workflows.  Given a *category* name (e.g. ``perception``), it reads the corresponding
section of ``.github/sync-params.yaml``, clones each listed upstream repository, and
for every ``source → variants`` mapping it either **updates** the variant files on disk
or **checks** that they are still in sync with the pinned upstream revision.

Overview
--------
Each variant file is a copy of an upstream ``*.param.yaml`` with three additions:

1. **Header** — workflow-managed comment block that records the source GitHub URL
   (pinned to the last-modified commit SHA of that file).
2. **Override markers** — leaf fields annotated with ``# {OVERRIDE}`` or
   ``# {OVERRIDE: reason}`` retain their local values across syncs.  All other
   fields are overwritten from the upstream source.
3. **Embedded original** — the full upstream content is appended as a comment
   block (``# ###### ORIGINAL (DO NOT EDIT) ######``) so that any upstream change
   to an overridden field is visible in the diff of the generated sync PR.

Usage
-----
Update variant files for the ``perception`` category::

    python .github/scripts/sync_params.py perception

Check for drift without writing anything (exits non-zero if drift is detected)::

    python .github/scripts/sync_params.py --check perception

Use a custom config file::

    python .github/scripts/sync_params.py --config path/to/sync-params.yaml perception

Config format (``.github/sync-params.yaml``)
--------------------------------------------
::

    perception:               # category name (passed as the positional CLI argument)
      - repository: autowarefoundation/autoware_universe
        ref: main             # branch/tag/SHA; omit to use the default branch
        files:
          - source: perception/autoware_image_object_locator/config/bbox_object_locator.param.yaml
            variants:
              - path: autoware_launch/config/perception/.../near_range_camera_vru_detector.param.yaml

Override markers
----------------
Add ``# {OVERRIDE}`` (or ``# {OVERRIDE: reason}``) as an inline comment on any
**leaf** field (scalar value or flat list of scalars) in a variant file to preserve
that value across syncs::

    some_param: 42        # {OVERRIDE}
    other_param: [1, 2]   # {OVERRIDE: tuned for this vehicle}

After a sync the marker is re-emitted verbatim in the output, so the annotation
survives repeated workflow runs.

Check mode vs. update mode
--------------------------
- **Update mode** (default): clones upstream, applies overrides on top of the
  current HEAD of each source file, and writes the result.
- **Check mode** (``--check``): uses the SHA pinned in each variant's header
  (``# Source: …/blob/<sha>/…``) instead of the current HEAD.  This means only
  *manual edits* to variant files are reported as drift; upstream changes between
  the pinned SHA and HEAD are intentionally ignored (those are handled by a
  dedicated update run).
"""

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
    """Return True if *value* is a YAML leaf scalar (None, bool, int, float, or str)."""
    return value is None or isinstance(value, (bool, int, float, str))


def _is_scalar_or_scalar_array(value: Any) -> bool:
    """Return True if *value* is a leaf scalar or a flat list of scalars.

    Only these types may be used as override values; nested mappings or lists of
    mappings are rejected so that overrides remain unambiguous.
    """
    if _is_scalar(value):
        return True
    if isinstance(value, list):
        return all(_is_scalar(v) for v in value)
    return False


def _format_path(path: Iterable[str]) -> str:
    """Format a key path tuple as a human-readable bracket string, e.g. ``[a][b][c]``."""
    return "".join(f"[{p}]" for p in path)


def _strip_quotes(value: str) -> str:
    """Remove matching surrounding single or double quotes from *value*, if present."""
    if len(value) >= 2 and value[0] == value[-1] and value[0] in {"'", '"'}:
        return value[1:-1]
    return value


_YAML_RT = YAML(typ="rt")
_YAML_RT.preserve_quotes = True
_YAML_RT.indent(mapping=2, sequence=4, offset=2)


def _comment_has_override_marker(comment_obj: Any) -> bool:
    """Return True if the ruamel comment object contains an ``{OVERRIDE}`` marker."""
    if comment_obj is None:
        return False
    comment_value = getattr(comment_obj, "value", str(comment_obj))
    return _extract_override_marker(comment_value) is not None


def _normalize_override_marker(reason: str | None) -> str:
    """Return the canonical marker string for *reason*.

    Returns ``{OVERRIDE}`` when *reason* is absent or blank, and
    ``{OVERRIDE: <reason>}`` otherwise.
    """
    if reason is None or not reason.strip():
        return OVERRIDE_MARKER
    return f"{{OVERRIDE: {reason.strip()}}}"


def _extract_override_marker(text: str) -> str | None:
    """Search *text* for an ``{OVERRIDE}`` or ``{OVERRIDE: reason}`` token.

    Returns the normalised marker string if found, or ``None``.
    """
    match = _OVERRIDE_MARKER_PATTERN.search(text)
    if match is None:
        return None
    return _normalize_override_marker(match.group(1))


def _strip_leading_override_marker(text: str) -> str:
    """Remove a leading ``{OVERRIDE …}`` token from *text* and return the remainder."""
    match = _OVERRIDE_MARKER_PATTERN.match(text)
    if match is None:
        return text
    return text[match.end() :].lstrip(" ")


def _extract_comment_text(comment_obj: Any) -> str:
    """Extract the normalised comment body from a ruamel comment object.

    If the comment contains an override marker the marker is returned as-is;
    otherwise the plain text (stripped of the leading ``#``) is returned.
    """
    comment_value = getattr(comment_obj, "value", str(comment_obj))
    normalized = comment_value.splitlines()[0].strip()
    if normalized.startswith("#"):
        normalized = normalized[1:].strip()
    marker = _extract_override_marker(normalized)
    return marker if marker is not None else normalized


def _extract_override_comment_from_line(line: str) -> str | None:
    """Return the normalised override marker from the inline comment of a YAML line.

    Returns ``None`` if the line has no comment or the comment contains no marker.
    """
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
    """Return the index *after* the closing ``]`` of a flow sequence that starts at *start_idx*.

    Handles multi-line flow sequences by tracking ``[`` / ``]`` depth, ignoring any
    ``#``-prefixed comment text on each line.

    Raises :class:`SyncError` if the sequence is never closed.
    """
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
    """Build a mapping from YAML key path to line metadata for *text*.

    Parses *text* line-by-line (without a full YAML parse) to record, for every
    mapping key, its line index, indentation level, and whether the value is on
    the same line (inline) or on a following line (block).  The result is used to
    locate lines that must be patched when applying override values.
    """
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
    """Render *value* as a single-line YAML flow string (e.g. ``[1, 2, 3]`` or ``true``).

    Temporarily overrides the global ruamel instance's flow-style and width settings
    and restores them on exit.
    """
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
    """Return the index of the first non-comment, non-blank value line after a block key.

    Scans forward from *key_line_idx + 1*, skipping blank lines and comment-only
    lines.  Returns ``None`` if the next content line is at an indent level ``<=``
    *key_indent* (i.e. there is no child value in the block).
    """
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
    """Replace the value portion of *line* (a ``key: value [# comment]`` YAML line).

    Preserves the original column position of any trailing inline comment so that
    alignment is maintained after the value changes length.
    """
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
    """Replace a block-style value in *lines* in-place.

    Handles three block value shapes:

    - **Flow sequence** (``[…]``, possibly multi-line): replaced with the rendered
      flow form of *value*, or with *raw_flow_block_lines* verbatim if supplied
      (preserving the original multi-line layout and inline comments).
    - **Block sequence** (``- item`` list): replaced line-by-line with the new
      list items rendered as flow scalars.
    - **Scalar on its own line**: the value line is replaced in-place.
    """
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
    """Return *source_text* with override values substituted at the specified key paths.

    Overrides are applied **in descending line order** so that patching a
    lower block (which may change the line count) does not invalidate the stored
    ``line_idx`` for overrides higher up in the file.

    Args:
        source_text: The upstream YAML content to patch.
        overrides: Mapping from key path tuple to the new value to inject.
        override_flow_blocks: Optional mapping from key path to raw source lines for
            multi-line flow sequences.  When provided, the exact original block
            layout (including inline comments) is re-used instead of re-rendering
            through :func:`_render_inline_yaml_value`.

    Raises:
        :class:`SyncError`: If any override path cannot be found in *source_text*.
    """
    lines = source_text.splitlines()
    key_index = _build_key_line_index(source_text)
    # Apply in descending line order so mutations to lower lines never shift the
    # stored line_idx values for overrides that haven't been applied yet.
    ordered = sorted(
        overrides.items(),
        key=lambda item: key_index[item[0]].line_idx if item[0] in key_index else -1,
        reverse=True,
    )
    for path, value in ordered:
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
    """Extract the raw source lines of multi-line flow sequences for the given paths.

    For each path in *paths* that resolves to a block-style flow sequence
    (i.e. the value starts on the *next* line as ``[…]``), the raw lines of
    that sequence (from the opening ``[`` to the closing ``]`` inclusive) are
    collected.  These raw blocks are later passed to
    :func:`apply_overrides_to_source_text` so the original multi-line layout and
    row-annotation comments are preserved in the output.
    """
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
    """Re-attach ``{OVERRIDE}`` markers to the patched source text.

    After :func:`apply_overrides_to_source_text` replaces values it strips any
    existing inline comments.  This function walks over every overridden key and
    re-inserts the marker comment, obeying the following rules:

    - If the patched line already has an inline comment, the marker is prepended
      to the existing comment text (avoiding duplication of the comment body when
      it matches the marker reason).
    - If *override_comment_columns* is given, the ``#`` character is placed at
      the same column as it appeared in the variant file, preserving alignment.
    - An existing reasoned marker (``{OVERRIDE: …}``) is kept as-is; only a plain
      ``{OVERRIDE}`` is replaced if a reasoned marker is available.

    Args:
        text: The YAML text after values have been patched.
        override_comments: Mapping from key path to the marker string to emit.
        override_comment_columns: Optional column indices (0-based) for the ``#``
            character, taken from the previous variant file to preserve alignment.
    """
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
    """Return the column position of each override marker comment in *text*.

    The column (0-based character offset of ``#``) is recorded so that
    :func:`ensure_override_markers_in_text` can re-align the comment in the
    regenerated output at the same position used in the previous variant file.
    """
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
    """Return the ordered list of key paths that carry override markers in *text*."""
    return list(parse_override_comments_from_variant_text(text).keys())


def _expect_mapping(value: Any, context: str) -> dict[str, Any]:
    """Assert that *value* is a dict mapping and return it, raising :class:`SyncError` otherwise."""
    if not isinstance(value, dict):
        raise SyncError(f"{context} must be a mapping.")
    return value


def _parse_repository_entries(entries_data: Any, context: str) -> list[RepositoryEntry]:
    """Parse and validate the raw YAML list *entries_data* into a list of :class:`RepositoryEntry`.

    Each entry must be a mapping with ``repository`` (required), optional ``ref``,
    and ``files`` (list of source/variants pairs).  Unknown keys raise
    :class:`SyncError` immediately so configuration mistakes are caught early.
    """
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
    """Load and validate the sync-params config file, returning entries for *category*.

    Args:
        config_path: Path to the YAML config file (e.g. ``.github/sync-params.yaml``).
        category: Top-level key to read (e.g. ``perception``).

    Raises:
        :class:`SyncError`: If the file is invalid YAML, the top level is not a
            mapping, or *category* is not defined.
    """
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
    """Run a ``git`` sub-command and return its stdout, raising :class:`SyncError` on failure."""
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
    """Read the content of *source_path* at commit *sha* from a local git repository.

    Used in check mode to compare the variant against the pinned upstream revision
    rather than the current HEAD, so that upstream-only changes do not trigger
    false drift reports.
    """
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
    """Resolve and return the default branch name (e.g. ``main``) of a GitHub repository.

    Uses ``git ls-remote --symref`` to avoid a full clone.
    """
    # cSpell:ignore symref
    output = run_git(["ls-remote", "--symref", f"https://github.com/{repo}.git", "HEAD"])
    for line in output.splitlines():
        if line.startswith("ref: ") and line.endswith("\tHEAD"):
            ref_name = line.split()[1]
            if ref_name.startswith("refs/heads/"):
                return ref_name.removeprefix("refs/heads/")
    raise SyncError(f"Failed to resolve default branch for repository '{repo}'.")


def resolve_ref(repo: str, ref: str | None) -> str:
    """Return *ref* if specified, otherwise look up the default branch of *repo*."""
    if ref:
        return ref
    return resolve_default_branch(repo)


def clone_repository(repo: str, ref: str, temp_root: Path) -> Path:
    """Clone *repo* at the given *ref* into a deterministic sub-directory of *temp_root*.

    The destination directory name is derived from the repo and ref so that a
    second call with identical arguments is idempotent (the existing directory is
    removed and re-cloned).  Full history is fetched because
    :func:`last_modified_sha` requires ``git log`` over the complete ancestry.
    """
    repo_name = repo.replace("/", "__")
    repo_dir = temp_root / f"{repo_name}_{hashlib.sha1(f'{repo}@{ref}'.encode()).hexdigest()[:8]}"
    if repo_dir.exists():
        shutil.rmtree(repo_dir)
    # Full history is required to compute per-file last-modified SHA accurately.
    run_git(["clone", "--branch", ref, f"https://github.com/{repo}.git", str(repo_dir)])
    run_git(["checkout", ref], cwd=repo_dir)
    return repo_dir


def last_modified_sha(repo_dir: Path, source_path: str) -> str:
    """Return the full SHA of the most recent commit that modified *source_path*.

    This SHA is embedded in the variant header and used as the pinned reference
    point for future check-mode runs.
    """
    sha = run_git(["log", "-n", "1", "--format=%H", "--", source_path], cwd=repo_dir)
    if not sha:
        raise SyncError(f"No commit found for source file '{source_path}'.")
    return sha


def load_yaml_file(path: Path) -> Any:
    """Load a YAML file with ruamel (round-trip mode), raising :class:`SyncError` on parse failure."""
    try:
        return _YAML_RT.load(path.read_text(encoding="utf-8"))
    except Exception as exc:
        raise SyncError(f"Invalid YAML file '{path}': {exc}") from exc


def source_blob_url(source_repo: str, source_sha: str, source_path: str) -> str:
    """Build the GitHub blob permalink for a source file at a specific commit SHA."""
    return f"https://github.com/{source_repo}/blob/{source_sha}/{source_path}"


def build_variant_header(source_url: str) -> str:
    """Build the workflow-managed comment header that is prepended to every variant file.

    The header instructs maintainers how to mark overrides and embeds the
    ``# Source: <url>`` line whose SHA is used by check mode as the pinned revision.
    """
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
    """Return the full upstream source content encoded as a comment block.

    The block starts with ``# ###### ORIGINAL (DO NOT EDIT) ######`` and is
    appended at the end of each variant file.  Its purpose is to make upstream
    changes to overridden fields explicitly visible in the diff of the sync PR,
    prompting reviewers to consider whether the override value should also change.
    """
    lines = source_body.splitlines()
    commented_lines = [f"# {line}" if line else "#" for line in lines]
    section = ["", f"# {ORIGINAL_PIVOT}", *commented_lines]
    return "\n".join(section) + "\n"


def extract_pinned_source_sha_and_path(variant_text: str) -> tuple[str, str] | None:
    """Parse the ``# Source:`` header line and return ``(sha, path)`` or ``None``.

    The SHA and path are extracted from the GitHub blob URL embedded in the
    variant header (e.g. ``https://github.com/org/repo/blob/<sha>/path/to/file.yaml``).
    Returns ``None`` when no valid ``# Source:`` line is found.
    """
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
    """Reconstruct the upstream source body from the embedded comment block in *text*.

    Strips the leading ``# `` prefix from each commented line to recover the
    original content.  Returns ``None`` if the pivot header is absent, or an
    empty string if the block is present but contains no content lines.

    Raises :class:`SyncError` if any line in the block does not conform to the
    expected ``# <content>`` or ``#`` (blank) format.
    """
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
    """Return True if the embedded original block in *variant_text* equals *source_body* exactly."""
    extracted = extract_embedded_original_from_variant_text(variant_text)
    return extracted is not None and extracted == source_body


def get_value_at_path(data: Any, path: tuple[str, ...]) -> Any:
    """Traverse nested dicts following *path* and return the value at the leaf.

    Raises :exc:`KeyError` with the missing segment if any step of *path* cannot
    be resolved.  Used to verify that override paths still exist in the upstream
    source and to read the current value held in a variant.
    """
    cursor = data
    for segment in path:
        if not isinstance(cursor, dict) or segment not in cursor:
            raise KeyError(segment)
        cursor = cursor[segment]
    return cursor


def set_value_at_path(data: Any, path: tuple[str, ...], value: Any) -> None:
    """Set the leaf value at *path* inside *data* to a deep copy of *value*.

    Raises :exc:`KeyError` if any intermediate key or the final key is missing.
    """
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
    """Read override values and their comment strings from an existing variant file.

    Returns a tuple of two dicts:

    - **overrides**: maps each override key path to its current value (deep-copied
      from the variant's parsed YAML).
    - **override_comments**: maps each override key path to the normalised marker
      string (e.g. ``{OVERRIDE}`` or ``{OVERRIDE: reason}``).  This is re-emitted
      verbatim when re-generating the variant.

    Returns ``({}, {})`` when *variant_path* does not exist or contains no markers.

    Raises :class:`SyncError` if a marked path is not present in the variant YAML
    or if its value is not a leaf scalar or flat list of scalars.
    """
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
    """Write *new_content* to *path* (update mode) or just report whether it differs (check mode).

    Returns ``True`` if a change was detected (or made), ``False`` if the file
    already matches *new_content*.
    """
    old_content = path.read_text(encoding="utf-8") if path.exists() else None
    if old_content == new_content:
        return False
    if not check:
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(new_content, encoding="utf-8")
    return True


def _build_unified_diff(path: Path, old_content: str | None, new_content: str) -> str:
    """Build a concise unified diff string for display in check-mode output.

    At most 80 diff lines are emitted to keep CI logs readable.  When
    *old_content* is ``None`` the diff is shown as a pure addition.
    """
    # cSpell:ignore keepends tofile lineterm
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


def _load_yaml_from_text(text: str, label: str) -> Any:
    """Parse *text* as YAML (round-trip), raising :class:`SyncError` with *label* on failure."""
    try:
        return _YAML_RT.load(text)
    except Exception as exc:
        raise SyncError(f"Invalid YAML in {label}: {exc}") from exc


def sync_file_entry(
    workspace_root: Path,
    repository: RepositoryEntry,
    file_entry: FileEntry,
    source_repo_dir: Path,
    check: bool,
) -> int:
    """Sync (or check) all variant files for a single source file entry.

    For each variant listed under *file_entry*:

    1. Read the existing variant (if any) and extract pinned SHA and overrides.
    2. Choose the *effective* source body:
       - **Update mode**: current HEAD content of the source file.
       - **Check mode**: content at the SHA pinned in the variant's header,
         so that upstream changes between the pin and HEAD are ignored.
    3. Apply active overrides on top of the effective source body.
    4. Re-attach override markers and prepend the header.
    5. Append the embedded original comment block.
    6. Write the result (update mode) or compare and report drift (check mode).

    Overrides whose key paths have been removed from the upstream source are
    treated as ``stale``:  they are silently dropped in update mode (the deletion
    is visible in the sync PR diff) and counted as drift in check mode.

    Returns the number of variant files that were changed (update mode) or that
    differ from the expected state (check mode).
    """
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

    variants_changed = 0
    for variant in file_entry.variants:
        variant_abs = workspace_root / variant.path
        variant_text = variant_abs.read_text(encoding="utf-8") if variant_abs.exists() else ""

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
            pinned_body = load_source_body_at_sha(source_repo_dir, pinned_sha, pinned_source_path)
        else:
            pinned_sha = None
            pinned_body = None

        # In update mode: use current HEAD source.
        # In check mode: use the pinned SHA source so that we only detect manual edits
        # to the variant file, not upstream changes (those are handled by update-params).
        if check and pinned_body is not None:
            effective_source_body = pinned_body
            effective_source_yaml = _load_yaml_from_text(pinned_body, f"pinned {pinned_sha}")
            if effective_source_yaml is None:
                effective_source_yaml = {}
            effective_source_url = source_blob_url(
                repository.repository, pinned_sha, file_entry.source
            )
        else:
            effective_source_body = source_body
            effective_source_yaml = source_yaml
            effective_source_url = source_url

        effective_embedded_original = build_embedded_original_section(effective_source_body)

        # In update mode, warn if the embedded original drifted from the pinned source.
        if not check and pinned_body is not None:
            if variant_text and not embedded_original_matches_source(variant_text, pinned_body):
                print(
                    "[sync] Embedded original drift detected in "
                    f"'{variant.path}' for source '{file_entry.source}'."
                )

        overrides, override_comments = extract_override_values(variant_abs)
        override_comment_columns = parse_override_comment_columns_from_variant_text(variant_text)
        if not overrides:
            variant_content = build_variant_header(effective_source_url) + effective_source_body
            variant_content += effective_embedded_original
            changed = write_or_check(variant_abs, variant_content, check)
            if changed:
                variants_changed += 1
                if check:
                    old_content = variant_text if variant_text else None
                    diff_text = _build_unified_diff(variant_abs, old_content, variant_content)
                    print(f"[sync] Drift detail: {variant.path}")
                    if diff_text:
                        print(diff_text)
            continue

        active_overrides: dict[tuple[str, ...], Any] = {}
        stale_paths: list[tuple[str, ...]] = []
        for override_path, override_value in overrides.items():
            try:
                get_value_at_path(effective_source_yaml, override_path)
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
            effective_source_body, active_overrides, override_flow_blocks
        )
        variant_body = ensure_override_markers_in_text(
            variant_body, active_override_comments, active_override_comment_columns
        )
        variant_content = build_variant_header(effective_source_url) + variant_body
        variant_content += effective_embedded_original
        changed = write_or_check(variant_abs, variant_content, check)
        if changed:
            variants_changed += 1
            if check:
                old_content = variant_text if variant_text else None
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
    """Run the full sync pipeline for *category* and return an exit code.

    Clones each upstream repository into a temporary directory (cleaned up on
    exit), then calls :func:`sync_file_entry` for every configured source file.
    Prints a summary line at the end.  In check mode returns ``1`` if any variant
    drifted, otherwise returns ``0``.
    """
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


def main() -> int:
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
    args = parser.parse_args()
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
