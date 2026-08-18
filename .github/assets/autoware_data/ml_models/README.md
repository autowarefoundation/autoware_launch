# Autoware ML Param Files

Vendored `.param.yaml` files from the
[Autoware ML artifact set](https://github.com/autowarefoundation/autoware/blob/811a7e75ba6d6752fd9d1caf4a9a26f09294d474/ansible/roles/artifacts/tasks/main.yaml), for [roscope](https://github.com/paulsohn/roscope) CI purpose.

Refer to the ansible role for the current URLs and versions:
<https://github.com/autowarefoundation/autoware/blob/main/ansible/roles/artifacts/tasks/main.yaml>

## What belongs here

`roscope resolve` runs with `--inline-params`, so `roscope` opens every file that a launch file names in `<param from=...>`. Only those files belong here, and only for the resolve path that the default arguments select. A file that a node opens by itself at run time does not belong here.

The folder structure here must be the same as the structure that the ansible role installs under `~/autoware_data/ml_models`. When a bundle changes its layout, this tree needs the same change by hand. No test compares this tree against the bundle.

## How to refresh a bundle

Download the param files of the revision that the ansible role pins:

```bash
hf download AutowareFoundation/<bundle> --revision <tag> --include "*.param.yaml" --local-dir <a temporary directory>
```

Then copy only the files that a launch file names into this tree. Remove the `.cache` directory that the download creates.

## lidar_centerpoint

Tag `v4.0` holds one folder per variant (`base`, `tiny`, `sigma`, `short_range`). Each folder holds a manifest with the fixed name `ml_package.param.yaml`.

The default arguments select `tiny`, so only `tiny/ml_package.param.yaml` is here. A resolve run with `use_short_range_detection:=true` also needs `short_range/ml_package.param.yaml`.

`detection_class_remapper.param.yaml` is not here, because no launch file names it. The node gets the path of that file from the manifest, then opens the file at run time.
