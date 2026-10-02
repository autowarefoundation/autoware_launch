# autoware_map_config

Parameter set for `autoware_map_launch`. This package ships only data: every file under `config/` is a ROS 2 parameter file.

```bash
config/
├── pointcloud_map_loader.param.yaml
├── lanelet2_map_loader.param.yaml
├── map_projection_loader.param.yaml
└── map_tf_generator.param.yaml
```

The loader parameter files address the map files through `$(var lanelet2_map_path)`-style substitutions, so the launchers load them with `allow_substs="true"` and the system supplies the paths.

## Usage

`autoware_map_launch` resolves the tree through its `map_config_pkg` argument, which defaults to this package:

```xml
<include file="$(find-pkg-share autoware_map_launch)/launch/map_loader/pointcloud_map_loader.launch.xml">
  <arg name="map_config_pkg" value="my_map_config"/>
</include>
```

A product that sets `map_config_pkg` to its own package must ship the same `config/` layout, since the launch package addresses individual files by relative path. Products that differ in only a few files instead override the corresponding `*_param_path` arguments.
