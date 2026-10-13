# autoware_map_launch

Unit launchers for the map component. Each launcher brings up one map feature and is self-contained: it declares its own `*_param_path` arguments and resolves them through `map_config_pkg` (default `autoware_map_config`).

```bash
launch/
├── map_loader/
│   ├── pointcloud_map_loader.launch.xml    # PCD map publication and the partial/differential/selected map services
│   └── lanelet2_map_loader.launch.xml      # Lanelet2 map loading and its marker visualization
├── map_projection_loader/
│   └── map_projection_loader.launch.xml    # map projector info
├── map_tf_generator/
│   └── vector_map_tf_generator.launch.xml  # map -> viewer transform
└── map_hash_generator/
    └── map_hash_generator.launch.xml       # map hash and the lanelet2 XML service
```

`design/module/Map.module.yaml` describes the same composition for the system designer.

## Composition

There is no all-in-one entry launcher: the system composes the units it needs, pushes the `map` namespace and supplies the map file paths. `autoware_launch/launch/components/component_map.launch.xml` is the reference composition.

```xml
<group>
  <push-ros-namespace namespace="map"/>
  <include file="$(find-pkg-share autoware_map_launch)/launch/map_loader/lanelet2_map_loader.launch.xml">
    <arg name="lanelet2_map_path" value="$(var map_path)/$(var lanelet2_map_file)"/>
    <arg name="lanelet2_map_metadata_path" value="$(var map_path)/lanelet2_map_metadata.yaml"/>
  </include>
</group>
```

## Parameters

Point `map_config_pkg` at another package to relocate the whole parameter tree, or override a single `*_param_path` argument to replace one file:

```xml
<include file="$(find-pkg-share autoware_map_launch)/launch/map_loader/pointcloud_map_loader.launch.xml">
  <arg name="map_config_pkg" value="my_map_config"/>
</include>
```

## Package Dependencies

Please see `<exec_depend>` in `package.xml`.
