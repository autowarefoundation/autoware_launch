"""Unit tests for sync_params.py.

This test script validates override parsing/reapplication, marker placement,
embedded original handling, and category-based config parsing for the parameter
sync workflow.

Run:
    python .github/scripts/test_sync_params.py
"""

# cSpell:disable

import copy
from pathlib import Path
import tempfile
from textwrap import dedent
import unittest
from unittest.mock import patch

from ruamel.yaml import YAML
from sync_params import FileEntry
from sync_params import RepositoryEntry
from sync_params import SyncError
from sync_params import VariantEntry
from sync_params import apply_overrides_to_source_text
from sync_params import build_embedded_original_section
from sync_params import build_variant_header
from sync_params import embedded_original_matches_source
from sync_params import ensure_override_markers_in_text
from sync_params import extract_embedded_original_from_variant_text
from sync_params import extract_flow_sequence_override_blocks_from_text
from sync_params import extract_override_values
from sync_params import extract_pinned_source_sha_and_path
from sync_params import get_value_at_path
from sync_params import load_config
from sync_params import parse_override_comment_columns_from_variant_text
from sync_params import parse_override_comments_from_variant_text
from sync_params import parse_override_paths_from_variant_text
from sync_params import set_value_at_path
from sync_params import source_blob_url
from sync_params import sync_file_entry

_YAML_SAFE = YAML(typ="safe")


def _parse_yaml(text: str):
    return _YAML_SAFE.load(text)


class SyncParamsTest(unittest.TestCase):
    def test_parse_override_paths(self) -> None:
        text = """
/**:
  ros__parameters:
    tracker_state_parameter:
      max_dt: 0.9 # {OVERRIDE}
      names: [a, b] # {OVERRIDE}
      tagged_block_list: # {OVERRIDE: keep as block style}
        - x
        - y
    plain: 1.0
"""
        paths = parse_override_paths_from_variant_text(text)
        self.assertEqual(
            paths,
            [
                ("/**", "ros__parameters", "tracker_state_parameter", "max_dt"),
                ("/**", "ros__parameters", "tracker_state_parameter", "names"),
                ("/**", "ros__parameters", "tracker_state_parameter", "tagged_block_list"),
            ],
        )

    def test_parse_override_comments_from_multiline_flow_value_key_comment(self) -> None:
        text = """
/**:
  ros__parameters:
    tracker_state_parameter:
      can_assign_matrix: # {OVERRIDE}
        [1, 0, # row0
        0, 1] # row1
"""
        comments = parse_override_comments_from_variant_text(text)
        self.assertEqual(
            comments,
            {
                (
                    "/**",
                    "ros__parameters",
                    "tracker_state_parameter",
                    "can_assign_matrix",
                ): "{OVERRIDE}"
            },
        )

    def test_parse_override_comments_ignores_following_standalone_comments(self) -> None:
        text = """
/**:
  ros__parameters:
    logging_file_path: /tmp/foo # {OVERRIDE: diagnostics}

    # diagnostics
"""
        comments = parse_override_comments_from_variant_text(text)
        self.assertEqual(
            comments,
            {("/**", "ros__parameters", "logging_file_path"): "{OVERRIDE: diagnostics}"},
        )

    def test_config_rejects_string_variant(self) -> None:
        config = """
perception:
  - repository: a/b
    ref: main
    files:
      - source: s.yaml
        variants:
          - bad/string/path.yaml
"""
        with tempfile.TemporaryDirectory() as tmpdir:
            config_path = Path(tmpdir) / "cfg.yaml"
            config_path.write_text(config, encoding="utf-8")
            with self.assertRaises(SyncError):
                load_config(config_path, "perception")

    def test_config_allows_missing_ref(self) -> None:
        config = """
perception:
  - repository: a/b
    files:
      - source: s.yaml
        variants: []
"""
        with tempfile.TemporaryDirectory() as tmpdir:
            config_path = Path(tmpdir) / "cfg.yaml"
            config_path.write_text(config, encoding="utf-8")
            repositories = load_config(config_path, "perception")
            self.assertEqual(len(repositories), 1)
            self.assertIsNone(repositories[0].ref)

    def test_config_missing_category_raises(self) -> None:
        config = """
perception: []
"""
        with tempfile.TemporaryDirectory() as tmpdir:
            config_path = Path(tmpdir) / "cfg.yaml"
            config_path.write_text(config, encoding="utf-8")
            with self.assertRaises(SyncError):
                load_config(config_path, "localization")

    def test_extract_override_values_requires_leaf_scalar_or_scalar_array(self) -> None:
        variant = """
/**:
  ros__parameters:
    tracker_state_parameter: # {OVERRIDE}
      max_dt: 0.8
"""
        with tempfile.TemporaryDirectory() as tmpdir:
            path = Path(tmpdir) / "variant.yaml"
            path.write_text(variant, encoding="utf-8")
            with self.assertRaises(SyncError):
                extract_override_values(path)

    def test_override_reapplication_tracker_max_dt(self) -> None:
        source_yaml = _parse_yaml(
            """
/**:
  ros__parameters:
    tracker_state_parameter:
      max_dt: 1.0
      decay_rate: 0.1
"""
        )
        variant_text = """
/**:
  ros__parameters:
    tracker_state_parameter:
      max_dt: 0.5 # {OVERRIDE}
      decay_rate: 0.1
"""
        with tempfile.TemporaryDirectory() as tmpdir:
            variant_path = Path(tmpdir) / "variant.yaml"
            variant_path.write_text(variant_text, encoding="utf-8")
            overrides, override_comments = extract_override_values(variant_path)

        merged = copy.deepcopy(source_yaml)
        for path, value in overrides.items():
            _ = get_value_at_path(source_yaml, path)
            set_value_at_path(merged, path, value)

        self.assertEqual(merged["/**"]["ros__parameters"]["tracker_state_parameter"]["max_dt"], 0.5)
        self.assertEqual(
            merged["/**"]["ros__parameters"]["tracker_state_parameter"]["decay_rate"], 0.1
        )
        self.assertEqual(
            override_comments,
            {
                ("/**", "ros__parameters", "tracker_state_parameter", "max_dt"): "{OVERRIDE}",
            },
        )

    def test_fail_fast_when_override_path_removed_in_source(self) -> None:
        source_yaml = _parse_yaml(
            """
/**:
  ros__parameters:
    tracker_state_parameter:
      decay_rate: 0.1
"""
        )
        variant_text = """
/**:
  ros__parameters:
    tracker_state_parameter:
      max_dt: 0.5 # {OVERRIDE}
"""
        with tempfile.TemporaryDirectory() as tmpdir:
            variant_path = Path(tmpdir) / "variant.yaml"
            variant_path.write_text(variant_text, encoding="utf-8")
            overrides, _ = extract_override_values(variant_path)

        for path in overrides:
            with self.assertRaises(KeyError):
                get_value_at_path(source_yaml, path)

    def test_stale_override_is_identified_separately_from_active(self) -> None:
        """Overrides whose paths were removed from upstream can be separated from active ones."""
        source_yaml = _parse_yaml(
            """
/**:
  ros__parameters:
    tracker_state_parameter:
      decay_rate: 0.1
      still_present: 5.0
"""
        )
        variant_text = """
/**:
  ros__parameters:
    tracker_state_parameter:
      max_dt: 0.5      # {OVERRIDE}
      still_present: 9.0 # {OVERRIDE}
"""
        with tempfile.TemporaryDirectory() as tmpdir:
            variant_path = Path(tmpdir) / "variant.yaml"
            variant_path.write_text(variant_text, encoding="utf-8")
            overrides, _ = extract_override_values(variant_path)

        active = {}
        stale = []
        for path, value in overrides.items():
            try:
                get_value_at_path(source_yaml, path)
                active[path] = value
            except KeyError:
                stale.append(path)

        self.assertEqual(stale, [("/**", "ros__parameters", "tracker_state_parameter", "max_dt")])
        self.assertEqual(
            list(active.keys()),
            [("/**", "ros__parameters", "tracker_state_parameter", "still_present")],
        )

    def test_active_overrides_apply_cleanly_when_sibling_is_stale(self) -> None:
        """Passing stale override to apply_overrides_to_source_text raises SyncError, but filtering to active-only succeeds."""
        source_text = """
/**:
  ros__parameters:
    tracker_state_parameter:
      decay_rate: 0.1
      still_present: 5.0
"""
        stale_path = ("/**", "ros__parameters", "tracker_state_parameter", "max_dt")
        active_path = ("/**", "ros__parameters", "tracker_state_parameter", "still_present")
        all_overrides = {stale_path: 0.5, active_path: 9.0}

        # Applying all overrides (stale included) would fail.
        with self.assertRaises(SyncError):
            apply_overrides_to_source_text(source_text, all_overrides)

        # Applying only the active override succeeds.
        patched = apply_overrides_to_source_text(source_text, {active_path: 9.0})
        reparsed = _parse_yaml(patched)
        self.assertEqual(
            reparsed["/**"]["ros__parameters"]["tracker_state_parameter"]["still_present"], 9.0
        )

    def test_variant_header_mentions_source(self) -> None:
        header = build_variant_header(
            source_url="https://github.com/org/repo/blob/abc123/path/to/file.yaml",
        )
        self.assertIn("managed by workflow", header)
        self.assertIn("https://github.com/org/repo/blob/abc123/path/to/file.yaml", header)

    def test_build_embedded_original_section(self) -> None:
        source_body = """/**:\n  ros__parameters:\n    foo: bar\n"""
        section = build_embedded_original_section(source_body)
        self.assertIn("# ###### ORIGINAL (DO NOT EDIT) ######", section)
        self.assertIn("# /**:", section)
        self.assertIn("#   ros__parameters:", section)
        self.assertIn("#     foo: bar", section)

    def test_extract_embedded_original_from_variant_text_roundtrip(self) -> None:
        source_body = """/**:\n  ros__parameters:\n    foo: bar\n"""
        variant_text = """# header\n""" + build_embedded_original_section(source_body)
        extracted = extract_embedded_original_from_variant_text(variant_text)
        self.assertEqual(extracted, source_body)

    def test_embedded_original_matches_source(self) -> None:
        source_body = """/**:\n  ros__parameters:\n    foo: bar\n"""
        variant_text = """/**:\n  ros__parameters:\n    foo: local # {OVERRIDE}\n"""
        variant_text += build_embedded_original_section(source_body)
        self.assertTrue(embedded_original_matches_source(variant_text, source_body))

    def test_embedded_original_mismatch(self) -> None:
        source_body = """/**:\n  ros__parameters:\n    foo: baz\n"""
        variant_text = """/**:\n  ros__parameters:\n    foo: local # {OVERRIDE}\n\n# ###### ORIGINAL (DO NOT EDIT) ######\n# /**:\n#   ros__parameters:\n#     foo: bar\n"""
        self.assertFalse(embedded_original_matches_source(variant_text, source_body))

    def test_extract_pinned_source_sha_and_path(self) -> None:
        variant_text = """# This file is managed by workflow.\n# Source: https://github.com/org/repo/blob/deadbeef/path/to/file.yaml\n"""
        self.assertEqual(
            extract_pinned_source_sha_and_path(variant_text),
            ("deadbeef", "path/to/file.yaml"),
        )

    def test_extract_pinned_source_sha_and_path_absent(self) -> None:
        variant_text = """# This file is managed by workflow.\n# no source line\n"""
        self.assertIsNone(extract_pinned_source_sha_and_path(variant_text))

    def test_source_blob_url_uses_sha(self) -> None:
        url = source_blob_url("org/repo", "deadbeef", "params/file.yaml")
        self.assertEqual(url, "https://github.com/org/repo/blob/deadbeef/params/file.yaml")

    def test_text_patching_preserves_comments_and_valid_yaml(self) -> None:
        source_text = """
/**:
  ros__parameters:
    min_points_num:
    #UNKNOWN, CAR, TRUCK
      [10, 10, 10]
    using_2d_validator: false
"""
        patched = apply_overrides_to_source_text(
            source_text,
            {("/**", "ros__parameters", "using_2d_validator"): True},
        )
        self.assertIn("#UNKNOWN, CAR, TRUCK", patched)
        self.assertIn("using_2d_validator: true", patched)
        reparsed = _parse_yaml(patched)
        self.assertTrue(reparsed["/**"]["ros__parameters"]["using_2d_validator"])

    def test_text_marker_reinsertion(self) -> None:
        source_text = """
/**:
  ros__parameters:
    max_dt: 1.0
"""
        with_markers = ensure_override_markers_in_text(
            source_text,
            {("/**", "ros__parameters", "max_dt"): "{OVERRIDE: keep this comment}"},
        )
        self.assertIn("max_dt: 1.0 # {OVERRIDE: keep this comment}", with_markers)

    def test_text_marker_reinsertion_preserves_column_from_variant(self) -> None:
        source_text = """
/**:
  ros__parameters:
    rois_timestamp_offsets: [0.059, 0.010, 0.026, 0.042, 0.076, 0.093]
"""
        variant_text = """
/**:
  ros__parameters:
    rois_timestamp_offsets: [0.098, 0.147, 0.078, 0.062, 0.115, 0.132]                 # {OVERRIDE}
"""
        path = ("/**", "ros__parameters", "rois_timestamp_offsets")
        columns = parse_override_comment_columns_from_variant_text(variant_text)
        with_markers = ensure_override_markers_in_text(
            source_text,
            {path: "{OVERRIDE}"},
            columns,
        )
        source_col = next(
            line for line in variant_text.splitlines() if "rois_timestamp_offsets" in line
        ).index("#")
        out_line = next(
            line for line in with_markers.splitlines() if "rois_timestamp_offsets" in line
        )
        self.assertEqual(out_line.index("#"), source_col)

    def test_text_marker_reinsertion_preserves_column_from_variant_when_source_has_comment(
        self,
    ) -> None:
        source_text = """
/**:
  ros__parameters:
    rois_timestamp_offsets: [0.059, 0.010, 0.026, 0.042, 0.076, 0.093]                             # source comment
"""
        variant_text = """
/**:
  ros__parameters:
    rois_timestamp_offsets: [0.098, 0.147, 0.078, 0.062, 0.115, 0.132] # {OVERRIDE: source comment}
"""
        path = ("/**", "ros__parameters", "rois_timestamp_offsets")
        columns = parse_override_comment_columns_from_variant_text(variant_text)
        with_markers = ensure_override_markers_in_text(
            source_text,
            {path: "{OVERRIDE: source comment}"},
            columns,
        )
        source_col = next(
            line for line in variant_text.splitlines() if "rois_timestamp_offsets" in line
        ).index("#")
        out_line = next(
            line for line in with_markers.splitlines() if "rois_timestamp_offsets" in line
        )
        self.assertEqual(out_line.index("#"), source_col)

    def test_text_marker_reinsertion_avoids_duplicate_comment_text(self) -> None:
        source_text = """
/**:
  ros__parameters:
    min_width: 0.1 #  Minimum width [m] (shoulder width)
"""
        with_markers = ensure_override_markers_in_text(
            source_text,
            {
                (
                    "/**",
                    "ros__parameters",
                    "min_width",
                ): "{OVERRIDE: Minimum width [m] (shoulder width)}"
            },
        )
        self.assertIn(
            "min_width: 0.1 # {OVERRIDE: Minimum width [m] (shoulder width)}  Minimum width [m] (shoulder width)",
            with_markers,
        )

    def test_text_marker_reinsertion_preserves_reasoned_marker_on_inline_comment_source(
        self,
    ) -> None:
        source_text = """
/**:
  ros__parameters:
    max_velocity: 1.0 #[m/s]
"""
        with_markers = ensure_override_markers_in_text(
            source_text,
            {
                ("/**", "ros__parameters", "max_velocity"): "{OVERRIDE: tuned for simulator}",
            },
        )
        self.assertIn(
            "max_velocity: 1.0 # {OVERRIDE: tuned for simulator}[m/s]",
            with_markers,
        )

    def test_text_marker_reinsertion_preserves_existing_reasoned_marker(self) -> None:
        source_text = """
/**:
  ros__parameters:
    min_width: 0.1 # {OVERRIDE: old text duplicated}
"""
        with_markers = ensure_override_markers_in_text(
            source_text,
            {
                ("/**", "ros__parameters", "min_width"): "{OVERRIDE: anything}",
            },
        )
        self.assertIn(
            "min_width: 0.1 # {OVERRIDE: old text duplicated}",
            with_markers,
        )

    def test_text_marker_reinsertion_preserves_existing_plain_marker(self) -> None:
        source_text = """
/**:
  ros__parameters:
    min_width: 0.1 # {OVERRIDE} old text duplicated
"""
        with_markers = ensure_override_markers_in_text(
            source_text,
            {
                ("/**", "ros__parameters", "min_width"): "{OVERRIDE: anything}",
            },
        )
        self.assertIn(
            "min_width: 0.1 # {OVERRIDE} old text duplicated",
            with_markers,
        )

    def test_multiline_flow_sequence_override_replacement_keeps_valid_yaml(self) -> None:
        source_text = """
/**:
  ros__parameters:
    tracker_state_parameter:
      can_assign_matrix:
        [0, 0, # row0
        0, 0] # row1
"""
        patched = apply_overrides_to_source_text(
            source_text,
            {
                (
                    "/**",
                    "ros__parameters",
                    "tracker_state_parameter",
                    "can_assign_matrix",
                ): [1, 1, 1, 1]
            },
        )
        reparsed = _parse_yaml(patched)
        self.assertEqual(
            reparsed["/**"]["ros__parameters"]["tracker_state_parameter"]["can_assign_matrix"],
            [1, 1, 1, 1],
        )

    def test_extract_flow_sequence_override_blocks_from_text(self) -> None:
        variant_text = """
/**:
  ros__parameters:
    tracker_state_parameter:
      can_assign_matrix: # {OVERRIDE}
        [1, 0, # row0
        0, 1] # row1
"""
        blocks = extract_flow_sequence_override_blocks_from_text(
            variant_text,
            [
                (
                    "/**",
                    "ros__parameters",
                    "tracker_state_parameter",
                    "can_assign_matrix",
                )
            ],
        )
        self.assertIn(
            (
                "/**",
                "ros__parameters",
                "tracker_state_parameter",
                "can_assign_matrix",
            ),
            blocks,
        )
        block_lines = blocks[
            (
                "/**",
                "ros__parameters",
                "tracker_state_parameter",
                "can_assign_matrix",
            )
        ]
        self.assertEqual(block_lines[0].strip(), "[1, 0, # row0")
        self.assertEqual(block_lines[1].strip(), "0, 1] # row1")

    def test_multiline_flow_sequence_override_replacement_can_preserve_block_style(self) -> None:
        source_text = """
/**:
  ros__parameters:
    tracker_state_parameter:
      can_assign_matrix:
        [0, 0, # src_row0
        0, 0] # src_row1
"""
        override_path = (
            "/**",
            "ros__parameters",
            "tracker_state_parameter",
            "can_assign_matrix",
        )
        raw_block_lines = [
            "        [1, 1, # row0",
            "        1, 1] # row1",
        ]
        patched = apply_overrides_to_source_text(
            source_text,
            {override_path: [1, 1, 1, 1]},
            {override_path: raw_block_lines},
        )
        self.assertIn("[1, 1, # row0", patched)
        self.assertIn("1, 1] # row1", patched)
        reparsed = _parse_yaml(patched)
        self.assertEqual(
            reparsed["/**"]["ros__parameters"]["tracker_state_parameter"]["can_assign_matrix"],
            [1, 1, 1, 1],
        )

    def test_inline_value_replacement_preserves_spacing_before_comment(self) -> None:
        source_text = """
/**:
  ros__parameters:
    min_width: 0.3                  # Minimum width [m]
"""
        patched = apply_overrides_to_source_text(
            source_text,
            {("/**", "ros__parameters", "min_width"): 0.1},
        )
        self.assertIn("min_width: 0.1                  # Minimum width [m]", patched)

    def test_inline_list_override_preserves_comment_column(self) -> None:
        source_text = """
/**:
  ros__parameters:
    rois_timestamp_offsets: [0.059, 0.010, 0.026, 0.042, 0.076, 0.093]                 # old
"""
        patched = apply_overrides_to_source_text(
            source_text,
            {
                ("/**", "ros__parameters", "rois_timestamp_offsets"): [
                    0.098,
                    0.147,
                    0.078,
                    0.062,
                    0.115,
                    0.132,
                ]
            },
        )
        source_line = next(
            line for line in source_text.splitlines() if "rois_timestamp_offsets" in line
        )
        patched_line = next(
            line for line in patched.splitlines() if "rois_timestamp_offsets" in line
        )
        self.assertEqual(source_line.index("#"), patched_line.index("#"))

    def test_text_marker_reinsertion_does_not_duplicate_next_comment_line(self) -> None:
        source_text = """
/**:
  ros__parameters:
    min_width: 0.1 #  Minimum width [m] (shoulder width)

    # Minimum width [m] (shoulder width)
"""
        with_markers = ensure_override_markers_in_text(
            source_text,
            {
                (
                    "/**",
                    "ros__parameters",
                    "min_width",
                ): "{OVERRIDE: Minimum width [m] (shoulder width)}"
            },
        )
        out_line = next(line for line in with_markers.splitlines() if "min_width:" in line)
        self.assertIn("# {OVERRIDE", out_line)
        self.assertEqual(with_markers.count("# Minimum width [m] (shoulder width)"), 1)

    def test_text_marker_reinsertion_with_parsed_comment_does_not_duplicate_following_comment(
        self,
    ) -> None:
        source_text = """
/**:
  ros__parameters:
    logging_file_path: /tmp/foo

    # diagnostics
"""
        variant_text = """
/**:
  ros__parameters:
    logging_file_path: /tmp/foo # {OVERRIDE: diagnostics}

    # diagnostics
"""
        override_comments = parse_override_comments_from_variant_text(variant_text)
        with_markers = ensure_override_markers_in_text(source_text, override_comments)
        self.assertIn("logging_file_path: /tmp/foo # {OVERRIDE: diagnostics}", with_markers)
        self.assertEqual(with_markers.count("# diagnostics"), 1)


class StaleOverrideIntegrationTest(unittest.TestCase):
    """Integration tests for stale override behavior in sync_file_entry.

    'Stale' means the override marker still exists in the variant file but the
    field has since been removed from the upstream source.

    Expected contract:
    - update mode (check=False): succeeds, removes the stale field from output.
    - check mode  (check=True):  reports the discrepancy (returns changed > 0),
                                  so the CI gate fails.
    """

    def _make_fixtures(self, tmpdir: str):
        workspace_root = Path(tmpdir) / "workspace"
        workspace_root.mkdir()
        source_dir = Path(tmpdir) / "source_repo"
        source_dir.mkdir()

        # Source no longer has 'removed_field'; only 'kept_field' remains.
        source_text = dedent(
            """\
            /**:
              ros__parameters:
                kept_field: 1.0
            """
        )
        source_file = source_dir / "params.yaml"
        source_file.write_text(source_text, encoding="utf-8")

        # Variant still has 'removed_field' with an override marker.
        variant_text = dedent(
            """\
            /**:
              ros__parameters:
                removed_field: 99.0 # {OVERRIDE}
                kept_field: 2.0     # {OVERRIDE}
            """
        )
        variant_rel = "config/params.yaml"
        variant_abs = workspace_root / variant_rel
        variant_abs.parent.mkdir(parents=True, exist_ok=True)
        variant_abs.write_text(variant_text, encoding="utf-8")

        repository = RepositoryEntry(
            repository="org/repo",
            ref="main",
            files=(
                FileEntry(
                    source="params.yaml",
                    variants=(VariantEntry(path=variant_rel),),
                ),
            ),
        )
        return workspace_root, source_dir, repository, repository.files[0], variant_abs

    def test_update_mode_succeeds_and_drops_stale_field(self) -> None:
        with tempfile.TemporaryDirectory() as tmpdir:
            workspace_root, source_dir, repo, file_entry, variant_abs = self._make_fixtures(tmpdir)
            with patch("sync_params.last_modified_sha", return_value="deadbeef"):
                changed = sync_file_entry(workspace_root, repo, file_entry, source_dir, check=False)

            self.assertEqual(changed, 1)
            result = variant_abs.read_text(encoding="utf-8")
            # Stale field is dropped — visible as a deletion in the PR diff for review.
            self.assertNotIn("removed_field", result)
            self.assertIn("kept_field: 2.0", result)
            self.assertIn("{OVERRIDE}", result)

    def test_check_mode_reports_stale_field_as_drift(self) -> None:
        with tempfile.TemporaryDirectory() as tmpdir:
            workspace_root, source_dir, repo, file_entry, variant_abs = self._make_fixtures(tmpdir)
            original_variant_text = variant_abs.read_text(encoding="utf-8")
            with patch("sync_params.last_modified_sha", return_value="deadbeef"):
                changed = sync_file_entry(workspace_root, repo, file_entry, source_dir, check=True)

            self.assertGreater(changed, 0)
            # Variant file must not be modified in check mode.
            self.assertEqual(variant_abs.read_text(encoding="utf-8"), original_variant_text)

    def test_full_sync_scenario_from_pr_description(self) -> None:
        """
        Covers all four field-change cases in a single sync cycle.

          Original upstream:          Variant before sync:
            foo: 42                     foo: 42
            bar: hoge                   bar: hogehoge  # {OVERRIDE}
            baz: 1                      baz: 2         # {OVERRIDE}

          Upstream changes:
            foo: 42   →  foo: 40 # updated   (non-overridden, value changed)
            bar: hoge →  bar: fuga            (overridden, upstream changed — override wins)
            baz: 1   →   (removed)            (overridden, removed upstream — dropped with warning)
                          qux: piyo # added   (new field, no override — synced verbatim)

          Expected variant after sync:
            foo: 40 # updated          ← synced from upstream
            bar: hogehoge # {OVERRIDE} ← override preserved (upstream change ignored)
                                         baz dropped (stale, visible as deletion in PR diff)
            qux: piyo # added          ← new field from upstream

          check mode must report drift so CI fails.
        """
        with tempfile.TemporaryDirectory() as tmpdir:
            workspace_root = Path(tmpdir) / "workspace"
            workspace_root.mkdir()
            source_dir = Path(tmpdir) / "source_repo"
            source_dir.mkdir()

            source_text = dedent(
                """\
                foo: 40 # updated
                bar: fuga
                qux: piyo # added
                """
            )
            (source_dir / "params.yaml").write_text(source_text, encoding="utf-8")

            variant_text = dedent(
                """\
                foo: 42
                bar: hogehoge # {OVERRIDE}
                baz: 2 # {OVERRIDE}
                """
            )
            variant_rel = "config/params.yaml"
            variant_abs = workspace_root / variant_rel
            variant_abs.parent.mkdir(parents=True, exist_ok=True)
            variant_abs.write_text(variant_text, encoding="utf-8")

            repo = RepositoryEntry(
                repository="org/repo",
                ref="main",
                files=(
                    FileEntry(
                        source="params.yaml",
                        variants=(VariantEntry(path=variant_rel),),
                    ),
                ),
            )

            # --- update mode ---
            with patch("sync_params.last_modified_sha", return_value="deadbeef"):
                changed = sync_file_entry(
                    workspace_root, repo, repo.files[0], source_dir, check=False
                )

            self.assertEqual(changed, 1)
            result = variant_abs.read_text(encoding="utf-8")

            # Non-overridden field updated upstream → synced.
            self.assertIn("foo: 40 # updated", result)
            # Overridden field changed upstream → override value preserved.
            self.assertIn("bar: hogehoge", result)
            # The upstream value must not appear as live YAML (may appear in the embedded original).
            body_lines = [ln for ln in result.splitlines() if not ln.startswith("#")]
            self.assertNotIn("bar: fuga", "\n".join(body_lines))
            # Stale overridden field (removed upstream) → dropped; visible in PR diff.
            self.assertNotIn("baz", result)
            # New field added upstream → present.
            self.assertIn("qux: piyo # added", result)

            # --- check mode (restore variant to pre-sync state) ---
            variant_abs.write_text(variant_text, encoding="utf-8")
            with patch("sync_params.last_modified_sha", return_value="deadbeef"):
                changed_check = sync_file_entry(
                    workspace_root, repo, repo.files[0], source_dir, check=True
                )

            self.assertGreater(changed_check, 0)
            # Variant file must not be modified in check mode.
            self.assertEqual(variant_abs.read_text(encoding="utf-8"), variant_text)


class CheckModePinnedShaIntegrationTest(unittest.TestCase):
    """Integration tests verifying that check mode uses the pinned SHA, not HEAD.

    Check mode should only detect manual edits to variant files (drift from the last
    synced state). Upstream changes between the pinned SHA and the current HEAD must
    not cause check mode to report drift — those are handled by update-params.
    """

    def _run(
        self,
        *,
        pinned_source_body: str,
        current_head_body: str,
        variant_text: str,
        check: bool,
    ) -> tuple[int, str]:
        """Set up fixtures, run sync_file_entry, return (changed, result_text)."""
        with tempfile.TemporaryDirectory() as tmpdir:
            workspace_root = Path(tmpdir) / "workspace"
            workspace_root.mkdir()
            source_dir = Path(tmpdir) / "source_repo"
            source_dir.mkdir()

            # Current HEAD source on disk.
            (source_dir / "params.yaml").write_text(current_head_body, encoding="utf-8")

            variant_rel = "config/params.yaml"
            variant_abs = workspace_root / variant_rel
            variant_abs.parent.mkdir(parents=True, exist_ok=True)
            variant_abs.write_text(variant_text, encoding="utf-8")

            repo = RepositoryEntry(
                repository="org/repo",
                ref="main",
                files=(
                    FileEntry(
                        source="params.yaml",
                        variants=(VariantEntry(path=variant_rel),),
                    ),
                ),
            )

            with (
                patch("sync_params.last_modified_sha", return_value="newsha"),
                patch("sync_params.load_source_body_at_sha", return_value=pinned_source_body),
            ):
                changed = sync_file_entry(
                    workspace_root, repo, repo.files[0], source_dir, check=check
                )

            return changed, variant_abs.read_text(encoding="utf-8")

    def _make_up_to_date_variant(self, source_body: str) -> str:
        """Build a variant that is exactly what sync would produce (no overrides)."""
        from sync_params import build_embedded_original_section
        from sync_params import build_variant_header

        url = "https://github.com/org/repo/blob/pinnedsha/params.yaml"
        return (
            build_variant_header(url) + source_body + build_embedded_original_section(source_body)
        )

    def test_check_passes_when_variant_matches_pinned_sha(self) -> None:
        """No drift reported when variant matches the pinned SHA, even if HEAD differs."""
        pinned_source = "foo: 42\nbar: hoge\n"
        head_source = "foo: 99\nbar: fuga\n"  # HEAD has changed

        variant_text = self._make_up_to_date_variant(pinned_source)

        changed, result = self._run(
            pinned_source_body=pinned_source,
            current_head_body=head_source,
            variant_text=variant_text,
            check=True,
        )

        self.assertEqual(changed, 0)
        # File must be untouched.
        self.assertEqual(result, variant_text)

    def test_check_uses_pinned_sha_not_head(self) -> None:
        """If check used HEAD the variant would appear drifted, but since it uses the pinned SHA it must pass."""
        pinned_source = "foo: 42\nbar: hoge\n"
        head_source = "foo: 99\nbar: fuga\n"  # HEAD differs from pinned

        variant_text = self._make_up_to_date_variant(pinned_source)

        # Sanity-check: if head were used, the reconstructed content would differ from
        # the variant (which was built from the pinned source), so changed would be > 0.
        # We verify this by running with head_source as the pinned source too.
        changed_if_head_used, _ = self._run(
            pinned_source_body=head_source,  # pretend pinned == head
            current_head_body=head_source,
            variant_text=variant_text,
            check=True,
        )
        self.assertGreater(
            changed_if_head_used,
            0,
            "Scenario is trivial: variant matches HEAD too, so this test proves nothing.",
        )

        # Now run with the correct pinned source — check must pass.
        changed_with_pinned, result = self._run(
            pinned_source_body=pinned_source,
            current_head_body=head_source,
            variant_text=variant_text,
            check=True,
        )
        self.assertEqual(changed_with_pinned, 0)
        self.assertEqual(result, variant_text)

    def test_check_fails_when_variant_manually_edited(self) -> None:
        """Drift reported when variant was manually edited without an {OVERRIDE} marker."""
        pinned_source = "foo: 42\nbar: hoge\n"
        head_source = pinned_source  # HEAD unchanged — only manual edit matters

        correct_variant = self._make_up_to_date_variant(pinned_source)
        # Simulate a manual edit: change foo's value without adding {OVERRIDE}.
        tampered_variant = correct_variant.replace("foo: 42", "foo: 99")

        changed, result = self._run(
            pinned_source_body=pinned_source,
            current_head_body=head_source,
            variant_text=tampered_variant,
            check=True,
        )

        self.assertGreater(changed, 0)
        # File must not be modified in check mode.
        self.assertEqual(result, tampered_variant)

    def test_freshly_synced_variant_always_passes_check(self) -> None:
        """A variant produced by update mode must pass check mode unchanged.

        In other words, check mode detects only manual edits — never the result of
        the sync workflow itself.  We verify this by running update then check on the
        same source and asserting that check reports zero drift.
        """
        head_source = "foo: 42\nbar: hoge\nbaz: 3.14\n"
        # Start from a stale variant (built from an older source body).
        stale_source = "foo: 1\nbar: old\n"
        stale_variant = self._make_up_to_date_variant(stale_source)

        # Step 1: update — sync the variant to HEAD.
        changed_update, synced_variant = self._run(
            pinned_source_body=stale_source,
            current_head_body=head_source,
            variant_text=stale_variant,
            check=False,
        )
        self.assertEqual(changed_update, 1, "Update must have produced a new variant.")

        # Step 2: check — the freshly synced variant must pass immediately.
        # The pinned SHA in the new header now points to the HEAD content we just wrote,
        # so we pass head_source as both the pinned body and the current HEAD.
        changed_check, _ = self._run(
            pinned_source_body=head_source,
            current_head_body=head_source,
            variant_text=synced_variant,
            check=True,
        )
        self.assertEqual(changed_check, 0, "Freshly synced variant must pass check with no drift.")

    def test_update_uses_head_not_pinned_sha(self) -> None:
        """Update mode ignores the pinned SHA and rebuilds from current HEAD."""
        pinned_source = "foo: 42\nbar: hoge\n"
        head_source = "foo: 99\nbar: fuga\n"  # upstream has changed

        variant_text = self._make_up_to_date_variant(pinned_source)

        changed, result = self._run(
            pinned_source_body=pinned_source,
            current_head_body=head_source,
            variant_text=variant_text,
            check=False,
        )

        self.assertEqual(changed, 1)
        # The new variant must reflect HEAD values, not the pinned ones.
        body_lines = [ln for ln in result.splitlines() if not ln.startswith("#")]
        self.assertIn("foo: 99", "\n".join(body_lines))
        self.assertIn("bar: fuga", "\n".join(body_lines))


if __name__ == "__main__":
    unittest.main()
