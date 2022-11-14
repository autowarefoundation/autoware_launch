import argparse
from pathlib import Path

import git

"""
This module updates `sync-param-files.yaml` based on the launch parameter files in `autoware.universe`.
"""

REPO_NAME = "autowarefoundation/autoware.universe"
REPO_URL = f"https://github.com/{REPO_NAME}.git"
CLONE_PATH = Path("/tmp/autoware.universe")

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("sync_param_files_path", type=Path, help="path to sync-param-files.yaml")
    args = parser.parse_args()

    # Clone Autoware
    if not CLONE_PATH.exists():
        git.Repo.clone_from(REPO_URL, CLONE_PATH)

    # Create sync config for all existing ROS param files
    ros_param_paths = list(CLONE_PATH.glob('**/*.param.yaml'))
    ros_param_sync_map = {
        str(p.relative_to(CLONE_PATH)): str('config' / p.relative_to(CLONE_PATH)).replace('/config', '')
            for p in ros_param_paths
    }

    # Create sync-param-files.yaml
    with open(args.sync_param_files_path, "w") as f:
        f.write(f"- repository: {REPO_NAME}\n")
        f.write("  files:\n")
        indent = 4 * " "
        for src, dest in ros_param_sync_map.items():
            f.write(f"{indent}- source: {src}\n")
            f.write(f"{indent}  dest: {dest}\n")

if __name__ == "__main__":
    main()
