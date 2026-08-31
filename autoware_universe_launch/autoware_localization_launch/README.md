# autoware_localization_launch

Shared localization launch library. This package ships one self-contained launcher per localization unit; the system component launcher in the product layer (`autoware_launch/launch/components/component_localization.launch.xml`) is the composition root that selects which units launch and wires the topics between them, the same way `Localization.module.yaml` composes its instances on the design side.

```bash
launch/
├── pose_twist_estimator/                 # one launcher per estimator
│   ├── ndt_scan_matcher.launch.xml
│   ├── gyro_odometer.launch.xml
│   ├── yabloc.launch.xml
│   ├── eagleye/eagleye_rt.launch.xml
│   ├── ar_tag_based_localizer.launch.xml
│   └── lidar_marker_localizer.launch.xml
├── pose_twist_fusion_filter/             # ekf localizer, stop filter, twist2accel, pose instability detector
├── localization_error_monitor/
└── util/                                 # pose initializer and the ndt input pointcloud downsampling
design/module/                            # Autoware System Designer modules
├── Localization.module.yaml              # composition of the instances a system runs
├── pose_twist_estimator/                 # ndt pose estimator and gyro odometer twist estimator
├── pose_twist_fusion_filter/
└── util/                                 # LocalizationUtil: same node set as launch/util
```

## Launch files

There is no single entry launcher: each unit is included on its own, so a system launches only the estimators it uses and never loads the parameters of the others. Each unit declares its own `*_param_path` arguments and resolves them from the config package; the caller selects units, pushes the namespaces (`pose_estimator`, `twist_estimator`, `util`, `pose_twist_fusion_filter`) and wires the boundary topics (the arbiter relay inputs, eagleye standing in for the gnss pose).

- `pose_twist_estimator/*.launch.xml` — one launcher per pose/twist estimator. Running several pose estimators additionally takes the pose estimator arbiter, which relays the estimator inputs and arbitrates the outputs; the composition root launches it from `autoware_pose_estimator_arbiter`.
- `util/util.launch.xml` — pose initializer, automatic pose initializer, and the downsampling chain feeding NDT, loaded into the shared pointcloud container.
- `pose_twist_fusion_filter/pose_twist_fusion_filter.launch.xml` — EKF localizer, stop filter, twist2accel, pose instability detector.
- `localization_error_monitor/localization_error_monitor.launch.xml`

## Config

Each unit resolves its parameter files from a `config/` tree addressed through the `localization_config_pkg` argument, which defaults to `autoware_localization_config`. A product can relocate the whole tree by setting `localization_config_pkg` once at its entry point — the launch configuration propagates through every include — but it must then ship the same `config/` layout. Products that differ in only a few files instead override the corresponding `*_param_path` arguments declared at the top of each unit launcher:

```xml
<include file="$(find-pkg-share autoware_localization_launch)/launch/pose_twist_estimator/ndt_scan_matcher.launch.xml">
  <arg name="ndt_scan_matcher/ndt_scan_matcher_param_path" value="..."/>
</include>
```

## Package Dependencies

Please see `<exec_depend>` in `package.xml`.
