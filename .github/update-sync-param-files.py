import argparse
import glob

import yaml


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("output", type=str, help="output path of sync-param-files.yaml")
    args = parser.parse_args()

    # load current sync-param-files.yaml
    with open(args.output) as f:
        sync_param_file_curr = yaml.safe_load(f)
    src2dst_curr = {}
    for src_and_dst in sync_param_file_curr[0]["files"]:
        src, dst = src_and_dst["source"], src_and_dst["dest"]
        if src not in src2dst_curr.keys():
            src2dst_curr[src] = []
        src2dst_curr[src].append(dst)

    # create a list of .param.yaml files within autoware.universe/launch directory
    autoware_universe_path = "/tmp/autoware.universe/"
    files = glob.glob(autoware_universe_path + "launch/" + "**/*.param.yaml", recursive=True)
    files.sort()

    # iterate over the param files to create sync-param-files.yaml
    src_and_dst_list = []
    for file in files:
        src = file.replace(autoware_universe_path, "")

        # do not update if the key exists
        if src in src2dst_curr.keys():
            for dst in src2dst_curr[src]:
                src_and_dst_list.append({"source": src, "dest": dst})

        # do update if the key does not exist
        else:
            dst = "autoware_launch/config/" + src.replace("/config", "")
            src_and_dst_list.append({"source": src, "dest": dst})
    sync_param_file = [
        {"repository": "autowarefoundation/autoware.universe", "files": src_and_dst_list}
    ]

    # save sync-param-files.yaml
    with open(args.output, "w") as f:
        yaml.dump(sync_param_file, f, sort_keys=False)


if __name__ == "__main__":
    main()
