# autoware_localization_config

Parameter set for `autoware_localization_launch`. This package ships only data: every file under `config/` is a ROS 2 parameter file.

```bash
config/
├── ekf_localizer.param.yaml
├── stop_filter.param.yaml
├── twist2accel.param.yaml
├── pose_instability_detector.param.yaml
├── pose_initializer.param.yaml
├── localization_error_monitor.param.yaml
├── ndt_scan_matcher/                     # matcher params and pointcloud_preprocessor/ downsampling params
├── yabloc/
├── lidar_marker_localizer/               # localizer params and pointcloud_preprocessor/ filter params
├── ar_tag_based_localizer.param.yaml
└── eagleye_config.param.yaml
```

## Usage

`autoware_localization_launch` resolves the tree through its `localization_config_pkg` argument, which defaults to this package:

```xml
<include file="$(find-pkg-share autoware_localization_launch)/launch/localization.launch.xml">
  <arg name="localization_config_pkg" value="my_localization_config"/>
</include>
```

A product that sets `localization_config_pkg` to its own package must ship the same `config/` layout, since the launch package addresses individual files by relative path. Products that differ in only a few files instead override the corresponding `*_param_path` arguments.
