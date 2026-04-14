import copy
from pathlib import Path
import tempfile
import unittest

from sync_params import SyncError
from sync_params import apply_overrides_to_source_text
from sync_params import build_dest_header
from sync_params import dedupe_nearby_identical_comment_lines
from sync_params import ensure_override_markers_in_text
from sync_params import extract_flow_sequence_override_blocks_from_text
from sync_params import extract_override_values
from sync_params import get_value_at_path
from sync_params import load_config
from sync_params import parse_override_comments_from_variant_text
from sync_params import parse_override_paths_from_variant_text
from sync_params import set_value_at_path
from sync_params import source_blob_url
import yaml


class SyncParamsTest(unittest.TestCase):
    def test_parse_override_paths(self) -> None:
        text = """
/**:
  ros__parameters:
    tracker_state_parameter:
      max_dt: 0.9 # *OVERRIDE*
      names: [a, b] # *OVERRIDE*
      tagged_block_list: # *OVERRIDE* keep as block style
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
      can_assign_matrix: # *OVERRIDE*
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
                ): "*OVERRIDE*"
            },
        )

    def test_config_rejects_string_variant(self) -> None:
        config = """
perception:
  - repository: a/b
    ref: main
    files:
      - source: s.yaml
        dest: d.yaml
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
        dest: d.yaml
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
    tracker_state_parameter: # *OVERRIDE*
      max_dt: 0.8
"""
        with tempfile.TemporaryDirectory() as tmpdir:
            path = Path(tmpdir) / "variant.yaml"
            path.write_text(variant, encoding="utf-8")
            with self.assertRaises(SyncError):
                extract_override_values(path)

    def test_override_reapplication_tracker_max_dt(self) -> None:
        source_yaml = yaml.safe_load(
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
      max_dt: 0.5 # *OVERRIDE*
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
                ("/**", "ros__parameters", "tracker_state_parameter", "max_dt"): "*OVERRIDE*",
            },
        )

    def test_fail_fast_when_override_path_removed_in_source(self) -> None:
        source_yaml = yaml.safe_load(
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
      max_dt: 0.5 # *OVERRIDE*
"""
        with tempfile.TemporaryDirectory() as tmpdir:
            variant_path = Path(tmpdir) / "variant.yaml"
            variant_path.write_text(variant_text, encoding="utf-8")
            overrides, _ = extract_override_values(variant_path)

        for path in overrides:
            with self.assertRaises(KeyError):
                get_value_at_path(source_yaml, path)

    def test_dest_header_includes_source_and_variants(self) -> None:
        header = build_dest_header(
            source_url="https://github.com/org/repo/blob/abc123/path/to/file.yaml",
            variant_paths=["a.yaml", "b.yaml"],
        )
        self.assertIn("generated by workflow", header)
        self.assertIn("https://github.com/org/repo/blob/abc123/path/to/file.yaml", header)
        self.assertIn("a.yaml", header)
        self.assertIn("b.yaml", header)

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
        reparsed = yaml.safe_load(patched)
        self.assertTrue(reparsed["/**"]["ros__parameters"]["using_2d_validator"])

    def test_text_marker_reinsertion(self) -> None:
        source_text = """
/**:
  ros__parameters:
    max_dt: 1.0
"""
        with_markers = ensure_override_markers_in_text(
            source_text,
            {("/**", "ros__parameters", "max_dt"): "*OVERRIDE* keep this comment"},
        )
        self.assertIn("max_dt: 1.0 # *OVERRIDE* keep this comment", with_markers)

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
                ): "*OVERRIDE* Minimum width [m] (shoulder width)"
            },
        )
        self.assertIn(
            "min_width: 0.1 # *OVERRIDE*  Minimum width [m] (shoulder width)",
            with_markers,
        )

    def test_text_marker_reinsertion_rewrites_existing_marker_comment(self) -> None:
        source_text = """
/**:
  ros__parameters:
    min_width: 0.1 # *OVERRIDE* old text duplicated
"""
        with_markers = ensure_override_markers_in_text(
            source_text,
            {
                ("/**", "ros__parameters", "min_width"): "*OVERRIDE* anything",
            },
        )
        self.assertIn(
            "min_width: 0.1 # *OVERRIDE* old text duplicated",
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
        reparsed = yaml.safe_load(patched)
        self.assertEqual(
            reparsed["/**"]["ros__parameters"]["tracker_state_parameter"]["can_assign_matrix"],
            [1, 1, 1, 1],
        )

    def test_extract_flow_sequence_override_blocks_from_text(self) -> None:
        variant_text = """
/**:
  ros__parameters:
    tracker_state_parameter:
      can_assign_matrix: # *OVERRIDE*
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
        reparsed = yaml.safe_load(patched)
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

    def test_dedupe_nearby_identical_comment_lines(self) -> None:
        text = """
/**:
  ros__parameters:
    # Pedestrian size validation parameters

    # Pedestrian size validation parameters
    pedestrian_size_validation:
      enable: true
"""
        deduped = dedupe_nearby_identical_comment_lines(text)
        self.assertEqual(deduped.count("# Pedestrian size validation parameters"), 1)


if __name__ == "__main__":
    unittest.main()
