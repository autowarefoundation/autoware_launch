# autoware_sensor_driver_launch

Per-sensor-model driver launch files shared by sensor kits. Pre/post-processing units live in `autoware_sensing_launch`; this package hosts the driver layer that feeds them.

## Launch files

- `velodyne_VLP16.launch.xml`, `velodyne_VLS128.launch.xml`, `robosense_Bpearl.launch.xml`, `robosense_Helios.launch.xml`, `hesai_XT32.launch.xml`, `hesai_OT128.launch.xml` — per-model LiDAR wrappers. Filenames and argument names are a stable interface for sensor kits inside and outside this repository.
- `nebula_velodyne.launch.xml`, `nebula_robosense.launch.xml`, `nebula_hesai.launch.xml` — per-make driver chains: the nebula driver node, configured by the model param file shipped in the nebula driver package.

## Container addressing

Every file takes `target_container`, the absolute path of an existing container (e.g. `/sensing/lidar/top/pointcloud_preprocessor/pointcloud_container`), and loads the driver node into it. Containers are created by the sensor kit, which also loads whatever preprocessing follows; this package never creates one.

## Interface notes

- `launch_driver` selects the nebula operating mode: online (hardware driver feeding the decoder) or offline (decoder on recorded packets). It does not control whether the node is launched — a kit whose sensor data arrives as pointclouds simply does not include a driver launch.
- The driver publishes `pointcloud_raw_ex`, which is the input the following preprocessing reads.

## Package Dependencies

Please see `<exec_depend>` in `package.xml`.
