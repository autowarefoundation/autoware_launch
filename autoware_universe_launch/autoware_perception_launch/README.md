# autoware_perception_launch

Shared perception launch library. The product entry points live in the product layer (`autoware_launch/launch/components/component_perception.launch.xml` for the lidar pipeline and `component_traffic_light.launch.xml` for traffic light recognition); this package hosts the reusable perception units and the object-recognition orchestrator they build on:

```bash
launch/
├── common/                        # shared preprocess units (pointcloud downsample)
├── object_recognition/
│   ├── object_recognition_<mode>.launch.xml  # detection + tracking + prediction per sensor mode
│   ├── detection/
│   │   ├── detector/              # detectors
│   │   ├── filter/                # filters
│   │   └── merger/                # detection-side serial merger (used by the camera_lidar_radar_serial orchestrator)
│   └── prediction/
├── obstacle_segmentation/
├── occupancy_grid_map/
└── traffic_light_recognition/
```

## Launch files

- `object_recognition/object_recognition_<mode>.launch.xml` — one orchestrator per sensor mode (`camera`, `lidar`, `lidar_radar`, `camera_lidar_radar`, `camera_lidar_radar_serial`); hosts the detection composition inline together with the tracker and prediction, and pushes their module namespaces. Camera and radar input topics default to their absolute sensing paths (`/sensing/camera/cameraN/...`, `/sensing/radar/detected_objects`), so callers override only what differs. Most modes wire the multi-channel tracker to the detector outputs; `camera_lidar_radar_serial` instead merges the detectors in the detection stage and tracks the single merged `detected_objects` channel. `lidar_radar` is fixed to the PTv3 semantic-segmentation clustering pipeline; `camera` runs camera-only detectors (BEV and camera VRU).
- `object_recognition/detection/{detector,filter}/` — single-purpose units wired together by the orchestrators. `merger/camera_lidar_radar_merger.launch.xml` chains the detection-side object mergers for the `camera_lidar_radar_serial` orchestrator.
- `obstacle_segmentation/`, `occupancy_grid_map/`, `common/`, `traffic_light_recognition/` — units included directly by the product components.

## Namespace and container addressing

Callers own the namespaces: the product component pushes `perception` and `object_recognition`; `object_recognition_<mode>.launch.xml` pushes `detection`/`tracking`/`prediction`. The orchestrators address their internal topics through the fixed `/perception/object_recognition` namespace. Composable nodes load into the shared pointcloud container, whose name is exposed as `pointcloud_container_name` and aliased to the `node/pointcloud_container` argument of the detector units.

## Config

Each unit resolves its parameter files from this package's `config/` tree by default and exposes them as `*_param_path` arguments; products with different parameters pass their own file paths.

## Package Dependencies

Please see `<exec_depend>` in `package.xml`.
