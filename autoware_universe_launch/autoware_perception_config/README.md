# autoware_perception_config

Parameter tree for the perception feature launchers in `autoware_perception_launch`. This package ships data only; it declares no nodes and no runtime dependencies.

```bash
config/
├── object_recognition/
│   ├── detection/
│   ├── tracking/
│   └── prediction/
├── obstacle_segmentation/
├── occupancy_grid_map/
└── traffic_light_recognition/
```

The feature launchers in `autoware_perception_launch` resolve this tree through their `perception_config_pkg` argument, which defaults to this package, and expose each file as a `*_param_path` argument. A product that ships a few different parameters overrides those arguments; a product that ships the whole tree points `perception_config_pkg` at its own package, which must then carry the same `config/` layout.

The `config/` tree is the interface downstream distributions adopt as-is; only `package.xml`, `CMakeLists.txt`, `README.md`, and changelogs stay package-local. Keep changes to the tree folder-merge compatible.

Parameter files that shadow a node's own defaults are kept in sync with upstream by `.github/scripts/sync_params.py`; see `.github/sync-params.yaml` for the source-to-variant mapping.

## Package Dependencies

Please see `<exec_depend>` in `package.xml`.
