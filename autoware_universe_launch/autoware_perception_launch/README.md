# autoware_perception_launch

Shared perception launch library. The product entry points live in the product layer (`autoware_launch/launch/components/component_perception.launch.xml` for the lidar pipeline and `component_traffic_light.launch.xml` for traffic light recognition); this package hosts the reusable perception units and the object-recognition orchestrator they build on:

```bash
launch/
├── common/                        # shared preprocess units (pointcloud downsample)
├── object_recognition/
│   ├── object_recognition.launch.xml  # detection + tracking + prediction, selected by `mode`
│   ├── detection/
│   │   ├── detection_<mode>.launch.xml  # one straight-line composition per sensor mode
│   │   ├── detector/              # detectors
│   │   ├── filter/                # filters
│   │   └── merger/                # mergers
│   ├── tracking/
│   └── prediction/
├── obstacle_segmentation/
├── occupancy_grid_map/
└── traffic_light_recognition/
```

## Launch files

- `object_recognition/object_recognition.launch.xml` — bundles detection, tracking, and prediction and pushes their module namespaces; `mode` selects the detection composition (`camera`, `camera_lidar_fusion`, `camera_lidar_radar_fusion`, `lidar_radar_fusion`, `lidar`).
- `object_recognition/detection/detection_<mode>.launch.xml` — straight-line detection compositions; each declares only the arguments its own units consume. Camera and radar input topics default to their absolute sensing paths (`/sensing/camera/cameraN/...`, `/sensing/radar/detected_objects`), so callers override only what differs.
- `object_recognition/detection/{detector,filter,merger}/` — single-purpose units wired together by the compositions.
- `obstacle_segmentation/`, `occupancy_grid_map/`, `common/`, `traffic_light_recognition/` — units included directly by the product components.

## Namespace and container addressing

Callers own the namespaces: the product component pushes `perception` and `object_recognition`; `object_recognition.launch.xml` pushes `detection`/`tracking`/`prediction`. The detection compositions address their internal topics through the fixed `/perception/object_recognition/detection` namespace. Composable nodes load into the shared pointcloud container, whose name is passed through unchanged as `pointcloud_container_name` (`node/pointcloud_container` inside the detection layer).

## Config

Each unit resolves its parameter files from this package's `config/` tree by default and exposes them as `*_param_path` arguments; products with different parameters pass their own file paths.

## Package Dependencies

Please see `<exec_depend>` in `package.xml`.
