# autoware_sensing_launch

Shared sensing launch library. Sensor kits own the sensing entry point (`<sensor_model>_launch/launch/sensing.launch.xml`, included by `component_sensing.launch.xml`); this package hosts the reusable pre/post-processing units the kits build on. Driver launch files live in the driver layer package `autoware_sensor_driver_launch`.

## Container ownership and addressing

Sensor kits create the containers; units only load nodes into them. `pointcloud_container.launch.xml` is the one unit that creates a container, and it loads nothing into it.

Units that load composable nodes take the target container as an absolute path (`target_container`) supplied by the caller; units never derive it from their own namespace. Callers construct the path from their literal namespace knowledge and must keep it in step with their namespace pushes. The one exception is the concatenation units, whose `pointcloud_container_name` keeps its historical name and receives the top-level pointcloud container path passed through unchanged.

## Launch files

- `pointcloud_container.launch.xml` — component container for one sensor unit, created at `container_namespace/container_name`; `use_multithread` selects `component_container_mt`.
- `pointcloud_crop_boxes.launch.xml` — vehicle body crop box followed by a secondary crop box, loaded into an existing container. Used on its own where the remaining preprocessing stages do not apply.
- `pointcloud_preprocessor_cpu.launch.xml` — per-lidar preprocessing chain: the crop box unit, then distortion correction and ring outlier filter, loaded into an existing container. The secondary crop box is named by `secondary_crop_box` (e.g. `mirror`, `wheels`).
- `pointcloud_preprocessor_cuda.launch.xml` — the same chain folded into one CUDA node; crop box stages come as folded lists in `crop_box_param_path`.
- `pointcloud_concatenation_cpu.launch.xml`, `pointcloud_concatenation_cuda.launch.xml` — multi-lidar concatenation with time synchronization; the kit passes its `concatenate_and_time_sync_node_param_path` (input topic list is kit-specific).
- `blockage_diagnostics.launch.xml` — blockage diagnostics on the raw pointcloud; `angle_range` defaults to the configured device angles (float-formatted).
- `gnss_postprocessor.launch.xml` — septentrio heading converter plus NavSatFix-to-MGRS pose (`autoware_gnss_poser`).
- `imu_corrector.launch.xml` — imu corrector plus gyro bias estimator (`autoware_imu_corrector`).
- `radar_tracks_postprocessor.launch.xml` — radar tracks noise filter plus message converter.

## Config

- `distortion_corrector_node.param.yaml`, `ring_outlier_filter_node.param.yaml` — shared preprocessing defaults; kits with different sensors or thresholds pass their own file via the `*_param_path` arguments (the outlier shape parameters such as `vertical_bins` only take effect when `publish_outlier_pointcloud` is enabled).
- `blockage_diagnostics.param.yaml` — blockage diagnostics defaults; runtime shape parameters (angle range, bins) are set per sensor via launch arguments.
- `radar_tracks_msgs_converter.param.yaml`, `radar_tracks_noise_filter.param.yaml` — radar postprocessing defaults.

Crop box param files are owned by the sensor kits (they encode vehicle geometry). Kits generate them offline with:

```bash
scripts/generate_crop_box_params.py <vehicle_info.param.yaml> <mirror.param.yaml> <output_dir>
```

and pass the outputs via the `crop_box_filter_{self,secondary}_param_path` arguments.

## Scripts

- `septentrio_heading_converter.py` — runtime node converting the septentrio AttEuler heading to a GnssInsOrientationStamped (installed).
- `generate_crop_box_params.py` — offline generator for kit crop box files (not installed).

## Package Dependencies

Please see `<exec_depend>` in `package.xml`.
