# autoware_launcher

## Getting Started

### Using real vehicle

```bash
ros2 launch autoware_launch autoware.launch.xml map_path:=/path/to/map_folder vehicle_model:=[vehicle_name] sensor_model:=[sensor_name]
```

### Using planning simulator

```bash
ros2 launch autoware_launch planning_simulator.launch.xml map_path:=/path/to/map_folder vehicle_model:=[vehicle_name] sensor_model:=[sensor_name]
```

## Directory Structure

- [autoware_launch](./autoware_launch)
- [control_launch](./control_launch)
- [integration_launch](./integration_launch)
- [localization_launch](./localization_launch)
- [perception_launch](./perception_launch)
- [planning_launch](./planning_launch)
- [sensing_launch](./sensing_launch)
- [system_launch](./system_launch)
- [vehicle_launch](./vehicle_launch)
