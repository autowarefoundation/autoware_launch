# autoware_perception_launch

Shared perception launch library. The perception entry pages live in the product layer (`autoware_launch/launch/components/perception/perception_<variant>.launch.xml`, one fixed pipeline per `perception_mode`); this package hosts the reusable perception units the pages build on:

```bash
launch/
├── common/                    # shared preprocess units (pointcloud downsample)
├── object_recognition/
│   ├── detection/             # detectors, filters, mergers
│   ├── tracking/
│   └── prediction/
├── obstacle_segmentation/
├── occupancy_grid_map/
└── traffic_light_recognition/
```

## Package Dependencies

Please see `<exec_depend>` in `package.xml`.

## Usage

Include a unit and provide its parameter paths as `PACKAGE_param_path` args:

```xml
  <include file="$(find-pkg-share autoware_perception_launch)/launch/object_recognition/detection/detection.launch.xml">
    <!-- Parameter files -->
    <arg name="FOO_param_path" value="..."/>
    <arg name="BAR_param_path" value="..."/>
    ...
  </include>
```
