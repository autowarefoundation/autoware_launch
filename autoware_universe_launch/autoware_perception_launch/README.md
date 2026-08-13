# autoware_perception_launch

Shared perception launch library. The system component launchers live in the product layer (`autoware_launch/launch/components/component_perception.launch.xml` for the lidar pipeline and `component_traffic_light.launch.xml` for traffic light recognition); this package hosts the perception feature launchers they combine:

```bash
launch/
├── common/                        # shared preprocess feature launchers (pointcloud downsample)
├── object_recognition/
│   ├── object_recognition_<mode>.launch.xml  # detection + tracking + prediction per sensor mode
│   ├── detection/
│   │   ├── detector/              # detectors
│   │   ├── filter/                # filters
│   │   └── merger/                # detection-side serial merger (used by the camera_lidar_radar_serial launcher)
│   └── prediction/
├── obstacle_segmentation/
├── occupancy_grid_map/
└── traffic_light_recognition/
```

## Launch files

- `object_recognition/object_recognition_<mode>.launch.xml` — one feature launcher per sensor mode (`camera`, `lidar`, `lidar_radar`, `camera_lidar_radar`, `camera_lidar_radar_serial`); hosts the detection stage inline together with the tracker and prediction, and pushes their module namespaces. Camera and radar input topics default to their absolute sensing paths (`/sensing/camera/cameraN/...`, `/sensing/radar/detected_objects`), so callers override only what differs. Most modes wire the multi-channel tracker to the detector outputs; `camera_lidar_radar_serial` instead merges the detectors in the detection stage (optionally regenerating detections through detection-by-tracker), tracks the single merged `detected_objects` channel, and joins radar tracked objects at the decorative tracker merger. `lidar_radar` is fixed to the PTv3 semantic-segmentation clustering pipeline; `camera` runs camera-only detectors (BEV and camera VRU).
- `object_recognition/detection/{detector,filter}/` — single-purpose launchers wired together by the object-recognition launchers. `merger/camera_lidar_merger.launch.xml` chains the detection-side object mergers for the `camera_lidar_radar_serial` launcher.
- `obstacle_segmentation/ground_segmentation_<backend>.launch.xml` — ground removal backends (`rule_based`, `rule_based_cuda`, `ml`); backend selection is file selection. `ground_segmentation_outlier_filter_{single_frame,time_series}.launch.xml` chain after the ground filter; `ground_segmentation_{additional_lidars,ransac}.launch.xml` fold extra lidars into the obstacle stream for kits that carry them.
- `occupancy_grid_map/`, `common/`, `traffic_light_recognition/` — feature launchers included directly by the system components.

## Namespace and container addressing

Callers own the namespaces: the system component pushes `perception` and `object_recognition`; `object_recognition_<mode>.launch.xml` pushes `detection`/`tracking`/`prediction`. The object-recognition launchers address their internal topics through the fixed `/perception/object_recognition` namespace. Composable nodes load into the shared pointcloud container, whose name is exposed as `pointcloud_container_name` and aliased to the `node/pointcloud_container` argument of the detector launchers.

## Config

Each feature launcher resolves its parameter files from a `config/` tree and exposes them as `*_param_path` arguments; products with different parameters pass their own file paths. The tree is addressed through the `perception_config_pkg` argument (default: this package), so a product can relocate the whole tree by setting `perception_config_pkg` once at its entry point — the launch configuration propagates through every include. A product that overrides it must ship the same `config/` layout; launching a single page standalone without that override resolves defaults against this package.

The `launch/` and `config/` trees are the interface downstream distributions adopt as-is; only `package.xml`, `CMakeLists.txt`, `README.md`, and changelogs stay package-local. Keep changes to these trees folder-merge compatible.

## Package Dependencies

Please see `<exec_depend>` in `package.xml`.
