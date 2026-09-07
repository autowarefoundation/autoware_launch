# autoware_planning_config

Parameter set for `autoware_planning_launch`. This package ships only data: every file under `config/` is a ROS 2 parameter file, or a module preset that selects which planning modules a launch run brings up.

```bash
config/
├── preset/                        # module presets (which planning modules launch)
├── mission_planning/
├── neural_net_planner/            # diffusion planner and its trajectory processor plugins
└── scenario_planning/
    ├── common/                    # shared planning params, velocity smoother, planning validator
    ├── lane_driving/
    │   ├── behavior_planning/     # behavior path planner, behavior velocity planner, path generator
    │   └── motion_planning/       # path smoother, path optimizer, motion velocity planner
    └── parking/
```

## Usage

`autoware_planning_launch` resolves the tree through its `planning_config_pkg` argument, which defaults to this package:

```xml
<include file="$(find-pkg-share autoware_planning_launch)/launch/planning.launch.xml">
  <arg name="planning_config_pkg" value="my_planning_config"/>
</include>
```

A product that sets `planning_config_pkg` to its own package must ship the same `config/` layout, since the launch package addresses individual files by relative path. Products that differ in only a few files instead override the corresponding `*_param_path` arguments.
