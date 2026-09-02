# autoware_control_launch

Shared control launch library. The system component launcher lives in the product layer (`autoware_launch/launch/components/component_control.launch.xml`); this package hosts the control launcher it wraps:

```bash
launch/
└── control.launch.xml              # control entry: command gate, trajectory follower, control checkers and evaluator
design/module/                      # Autoware System Designer modules
├── Control.module.yaml             # control component: same node set as control.launch.xml
└── TrajectoryFollower.module.yaml  # controller + lane departure checker
```

## Structure

![autoware_control_launch](./control_launch.drawio.svg)

## Launch files

- `control.launch.xml` — the entry point. It includes the module preset, resolves every parameter file, and pushes the `control` namespace. It hosts the `control_container` (vehicle cmd gate, operation mode transition manager, trajectory follower, external command selector and converter) and the `control_check_container` with the optional checkers (lane departure checker, control validator, autonomous emergency braking, collision detector, obstacle collision checker, predicted path checker), followed by the control evaluator.

## Config

Parameter files are resolved from a `config/` tree addressed through the `control_config_pkg` argument, which defaults to `autoware_control_config`. A product can relocate the whole tree by setting `control_config_pkg` once at its entry point — the launch configuration propagates through every include — but it must then ship the same `config/` layout. Products that differ in only a few files instead override the corresponding `*_param_path` arguments, which are all declared at the top of `control.launch.xml`:

```xml
<include file="$(find-pkg-share autoware_control_launch)/launch/control.launch.xml">
  <arg name="control_config_pkg" value="my_control_config"/>
  <arg name="FOO_NODE_param_path" value="..."/>
</include>
```

Which modules launch is a separate axis, selected by `module_preset`; the presets live in the config package under `config/preset/`. The controller algorithms are selected by `lateral_controller_mode` (`mpc`, `pure_pursuit`) and `longitudinal_controller_mode` (`pid`), which also pick the controller parameter files.

## Notes

For reducing processing load, we use the [Component](https://docs.ros.org/en/humble/Concepts/About-Composition.html) feature in ROS 2 (similar to Nodelet in ROS 1 )

## Package Dependencies

Please see `<exec_depend>` in `package.xml`.
