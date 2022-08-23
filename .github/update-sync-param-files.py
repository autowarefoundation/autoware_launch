import argparse
from pathlib import Path

import yaml


def src_name_to_dst_name(src: Path) -> Path:
    module_launch_pkg_name = src.parents[-3].stem  # e.g. "tier4_control_launch"
    file_path_under_config = src.relative_to(Path("launch") / module_launch_pkg_name / "config")
    dst = Path("autoware_launch/config") / module_launch_pkg_name / file_path_under_config

    return dst


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("input_universe_dir", type=str, help="path to the target autoware.universe")
    parser.add_argument("output_file_path", type=str, help="output path of sync-param-files.yaml")
    args = parser.parse_args()

    # iterate over the param files to create sync-param-files.yaml
    sync_files_config = []
    autoware_launch_dir = Path(args.input_universe_dir) / "launch"
    for config_path in autoware_launch_dir.glob("**/*.param.yaml"):
        src = config_path.relative_to(args.input_universe_dir)
        dst = src_name_to_dst_name(src)

        sync_files_config.append({"source": str(src), "dest": str(dst)})
    sync_param_file = [
        {"repository": "autowarefoundation/autoware.universe", "files": sync_files_config}
    ]

    # save sync-param-files.yaml
    with open(args.output_file_path, "w") as f:
        yaml.dump(sync_param_file, f, sort_keys=False)


if __name__ == "__main__":
    main()
