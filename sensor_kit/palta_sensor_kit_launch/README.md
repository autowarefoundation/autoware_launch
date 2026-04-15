# palta_sensor_kit_launch

This sensor kit is intended for a setup with two Hesai PandarXT32 LiDARs and one Tamagawa IMU.

## sensor_model Name

Use `sensor_model:=palta_sensor_kit` when launching Autoware.

## Required Configuration

Before using this on a real vehicle, confirm at least the following:

- LiDAR 1 IP address
- LiDAR 2 IP address
- Host NIC IP address for each LiDAR
- UDP data and GNSS ports for each LiDAR
- IMU serial port
- `palta_sensor_kit_description/config/sensors_calibration.yaml`
- `palta_sensor_kit_description/config/sensor_kit_calibration.yaml`

## Standalone Launch Example

This example launches only the sensing side for the two LiDARs and the IMU.

```bash
ros2 launch palta_sensor_kit_launch sensing.launch.xml
```

If needed, `vehicle_mirror_param_file` can be overridden explicitly.

```bash
ros2 launch palta_sensor_kit_launch sensing.launch.xml \
  vehicle_mirror_param_file:=/path/to/vehicle_mirror.param.yaml
```

## Autoware Launch Example

To use this sensor kit from the full Autoware launch, set `sensor_model` to `palta_sensor_kit`.

```bash
ros2 launch autoware_launch autoware.launch.xml \
  vehicle_model:=sample_vehicle \
  sensor_model:=palta_sensor_kit \
  map_path:=/path/to/map
```
