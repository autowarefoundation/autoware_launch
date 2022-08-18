import argparse
import glob
import yaml
import os


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("output", type=str, help="output path of sync-param-files.yaml")
    args = parser.parse_args()

    # create a list of .param.yaml files within autoware.universe/launch directory
    tier4_launch_path = "/tmp/autoware.universe/launch/"
    files = glob.glob(tier4_launch_path + "**/*.param.yaml", recursive=True)

    # iterate over the param files to create sync-param-files.yaml
    src2dst = []
    for file in files:
        src = os.path.join(*(file.split('/')[3:]))
        dst = os.path.join("autoware_launch/config", *(file.replace("/config/", "/").split("/")[3:]))
        src2dst.append({"source": src, "dest": dst})
    sync_param_file = [{"repository": "autowarefoundation/autoware.universe", "files": src2dst}]

    # save sync-param-files.yaml
    with open(args.output, "w") as f:
        yaml.dump(sync_param_file, f, sort_keys=False)

if __name__ == "__main__":
    main()
