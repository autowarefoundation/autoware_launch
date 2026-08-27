# autoware_control_config

Parameter set for `autoware_control_launch`. This package ships only data: every file under `config/` is a ROS 2 parameter file, or a module preset that selects which control modules a launch run brings up.

```bash
config/
├── preset/                                  # module presets (which control modules launch)
├── common/                                  # nearest search params shared by the control nodes
├── trajectory_follower/                     # controller node, lateral/ and longitudinal/ controller params
├── vehicle_cmd_gate/
├── control_command_gate/
├── operation_mode_transition_manager/
├── shift_decider/
├── external_cmd_selector/
├── control_validator/
├── lane_departure_checker/
├── autoware_autonomous_emergency_braking/
├── autoware_collision_detector/
├── obstacle_collision_checker/
└── predicted_path_checker/
```

## Usage

`autoware_control_launch` resolves the tree through its `control_config_pkg` argument, which defaults to this package:

```xml
<include file="$(find-pkg-share autoware_control_launch)/launch/control.launch.xml">
  <arg name="control_config_pkg" value="my_control_config"/>
</include>
```

A product that sets `control_config_pkg` to its own package must ship the same `config/` layout, since the launch package addresses individual files by relative path. Products that differ in only a few files instead override the corresponding `*_param_path` arguments.

Per-vehicle controller tuning is a product-side concern: the caller overrides `lat_controller_param_path` and `lon_controller_param_path` (`autoware_launch` points them at a `trajectory_follower/<vehicle_id>/` subtree under `use_individual_control_param`).
