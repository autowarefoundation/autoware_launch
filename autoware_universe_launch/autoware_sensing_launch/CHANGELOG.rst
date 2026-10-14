^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package autoware_sensing_launch
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.53.0 (2026-10-13)
-------------------
* Merge remote-tracking branch 'origin/main' into tmp/bot/bump_version_base
* feat(autoware_sensing_launch): run blockage_diag standalone when agnocast is enabled (`#1987 <https://github.com/autowarefoundation/autoware_launch/issues/1987>`_)
  * feat(autoware_sensing_launch): run blockage_diag standalone when agnocast is enabled
  autoware_universe`#13074 <https://github.com/autowarefoundation/autoware_launch/issues/13074>`_ moves BlockageDiagComponent to agnocast_wrapper::Node,
  which is not designed to run inside a component container. Start the node as a
  standalone process when ENABLE_AGNOCAST=1 and keep loading it into
  target_container otherwise, as component_state_monitor.launch.py does for
  topic_state_monitor. The node name, namespace, remaps and parameters are
  identical in both branches.
  * refactor(autoware_sensing_launch): launch blockage_diag through the autoware_node action
  ---------
* feat(autoware_launch): run crop_box_filter standalone when agnocast is enabled (`#1990 <https://github.com/autowarefoundation/autoware_launch/issues/1990>`_)
  * feat(autoware_launch): run crop_box_filter standalone when agnocast is enabled
  autoware_universe`#13384 <https://github.com/autowarefoundation/autoware_launch/issues/13384>`_ moves CropBoxFilterComponent to
  agnocast_wrapper::Node on an agnocast-only executor, which the component
  containers do not set up. Start each of the eight crop_box_filter nodes as a
  standalone process when ENABLE_AGNOCAST=1 and keep loading them into their
  container otherwise. The node names, namespaces, remaps and parameters are
  the same in both branches.
  * refactor(autoware_launch): launch crop_box_filter through the autoware_node action
  ---------
* refactor(sensing)!: separate sensor driver and pre-processing layers from sensor kit configuration (`#1938 <https://github.com/autowarefoundation/autoware_launch/issues/1938>`_)
  * refactor(launch)!: rename tier4_sensing_launch to autoware_sensing_launch
  tier4_sensing_launch held one 19-line pass-through page that pushed the
  sensing namespace and included the sensor kit entry. The component page
  now includes that entry directly and pushes the namespace itself, so the
  layer is gone and the package is left as the home of the shared sensing
  units added next.
  - move the package to autoware_universe_launch/autoware_sensing_launch
  - rename tier4_sensing_component.launch.xml to
  components/component_sensing.launch.xml
  - drop the pass-through page and its diagram
  Co-Authored-By: Claude Opus 5 (1M context) <noreply@anthropic.com>
  * feat(autoware_sensing_launch): add the shared sensing unit library
  Declarative XML units for the sensor data pre/post-processing stages,
  each addressed by an absolute container path so a caller can place it in
  any container it owns.
  - pointcloud_container, the only unit that creates a container
  - pointcloud_crop_boxes, pointcloud_preprocessor\_{cpu,cuda},
  pointcloud_concatenation\_{cpu,cuda}
  - blockage_diagnostics, gnss_postprocessor, imu_corrector,
  radar_tracks_postprocessor
  - shared preprocessing, blockage and radar defaults under config/
  - generate_crop_box_params.py, which turns vehicle info and mirror
  geometry into the crop box files the kits commit and pass in;
  septentrio_heading_converter.py is installed as a runtime node
  The parameter files and the heading converter are ported from
  tier4/aip_launcher (aip_common_sensor_launch), Apache-2.0 under the same
  TIER IV copyright. Nothing includes these units yet.
  Co-Authored-By: Claude Opus 5 (1M context) <noreply@anthropic.com>
  * feat(autoware_sensor_driver_launch): add the sensor driver layer
  The hardware interface layer, kept apart from the pre-processing units so
  that a sensor make and model can be brought into the ROS graph on its
  own.
  - nebula\_{velodyne,robosense,hesai}: per-make driver chains, configured
  by the model param file shipped in the nebula driver package
  - velodyne\_{VLP16,VLS128}, robosense\_{Bpearl,Helios},
  hesai\_{XT32,OT128}: per-model wrappers
  - every file loads its node into target_container, an existing container
  owned by the caller; this package never creates one
  The velodyne and robosense wrappers carry the content of the sample kit's
  common_sensor_launch, which the following commit removes. The hesai
  wrappers are ported from tier4/aip_launcher (aip_common_sensor_launch)
  onto the same chain pattern, with sync_angle and cut_angle in place of
  scan_phase and nebula's own host_ip default.
  Co-Authored-By: Claude Opus 5 (1M context) <noreply@anthropic.com>
  * refactor(sensing)!: adopt the shared sensing units in the sample and awsim kits
  - sample kit: create one container per lidar, then load the per-model
  driver wrapper and the CPU preprocessing chain into it; concatenation
  comes from the shared unit
  - awsim kit: preprocessing only, since AWSIM publishes pointcloud_raw_ex
  directly
  - commit the crop box geometry each kit owns, generated from the vehicle
  description by generate_crop_box_params.py, in place of the runtime
  vehicle_info and mirror lookups
  - remove common_sensor_launch, whose nebula_node_container.launch.py the
  driver chains replace; its changelog continues as
  autoware_sensor_driver_launch/CHANGELOG.rst
  The awsim_labs fork read its preprocessing defaults out of
  common_sensor_launch; it points at autoware_sensing_launch until its own
  commit retires it.
  The awsim kit no longer launches a nebula driver node or the velodyne
  monitor.
  Co-Authored-By: Claude Opus 5 (1M context) <noreply@anthropic.com>
  * refactor(sensing)!: adopt the shared sensing units in the single_lidar kit
  single_lidar_common_launch was a copy of common_sensor_launch that had
  drifted apart from it, having lost the container namespace handling,
  udp_only support and the pre-processing diagnostic thresholds. The kit
  includes the shared driver wrapper and preprocessing unit instead, so it
  gains all three, and the fork is removed.
  Co-Authored-By: Claude Opus 5 (1M context) <noreply@anthropic.com>
  * refactor(sensing)!: adopt the shared sensing units in the awsim_labs kit
  AWSIM Labs publishes pointcloud_raw_ex directly, so the kit runs
  preprocessing only: one container per lidar, the CPU preprocessing chain,
  and the shared concatenation unit. common_awsim_labs_sensor_launch and
  its velodyne_node_container.launch.py are removed.
  The per-lidar container moves from
  /sensing/lidar/<position>/pointcloud_container to
  /sensing/lidar/<position>/pointcloud_preprocessor/pointcloud_container,
  matching the other kits.
  Co-Authored-By: Claude Opus 5 (1M context) <noreply@anthropic.com>
  * refactor(sensing)!: adopt the shared sensing units in the carla kit
  CARLA publishes an undistorted pointcloud without ring information, so
  only the crop boxes apply: the kit includes pointcloud_crop_boxes and
  keeps its relay. pointcloud_preprocessor.launch.py is removed and
  mirror.param.yaml gives way to the committed crop box files.
  Co-Authored-By: Claude Opus 5 (1M context) <noreply@anthropic.com>
  * chore: add atteuler cspell exception
  ---------
  Co-authored-by: Claude Opus 5 (1M context) <noreply@anthropic.com>
* Contributors: Koichi Imai, Taekjin LEE, github-actions

0.52.0 (2026-06-30)
-------------------

0.51.0 (2026-05-01)
-------------------

0.50.0 (2026-02-13)
-------------------
* Merge remote-tracking branch 'origin/main' into tmp/bot/bump_version_base
* chore: import tier4 launchers from universe (`#1740 <https://github.com/autowarefoundation/autoware_launch/issues/1740>`_)
* Contributors: Taeseung Sohn, github-actions

0.49.0 (2025-12-30)
-------------------

0.48.0 (2025-11-18)
-------------------

0.47.1 (2025-08-14)
-------------------

0.47.0 (2025-08-11)
-------------------

0.46.0 (2025-06-20)
-------------------

0.45.0 (2025-05-22)
-------------------

0.44.2 (2025-06-10)
-------------------

0.44.1 (2025-05-01)
-------------------

0.44.0 (2025-04-18)
-------------------

0.43.0 (2025-03-21)
-------------------
* Merge remote-tracking branch 'origin/main' into chore/bump-version-0.43
* chore: rename from `autoware.universe` to `autoware_universe` (`#10306 <https://github.com/autowarefoundation/autoware_universe/issues/10306>`_)
* Contributors: Hayato Mizushima, Yutaka Kondo

0.42.0 (2025-03-03)
-------------------

0.41.2 (2025-02-19)
-------------------
* chore: bump version to 0.41.1 (`#10088 <https://github.com/autowarefoundation/autoware_universe/issues/10088>`_)
* Contributors: Ryohsuke Mitsudome

0.41.1 (2025-02-10)
-------------------

0.41.0 (2025-01-29)
-------------------

0.40.0 (2024-12-12)
-------------------
* Revert "chore(package.xml): bump version to 0.39.0 (`#9587 <https://github.com/autowarefoundation/autoware_universe/issues/9587>`_)"
  This reverts commit c9f0f2688c57b0f657f5c1f28f036a970682e7f5.
* fix: fix ticket links in CHANGELOG.rst (`#9588 <https://github.com/autowarefoundation/autoware_universe/issues/9588>`_)
* chore(package.xml): bump version to 0.39.0 (`#9587 <https://github.com/autowarefoundation/autoware_universe/issues/9587>`_)
  * chore(package.xml): bump version to 0.39.0
  * fix: fix ticket links in CHANGELOG.rst
  * fix: remove unnecessary diff
  ---------
  Co-authored-by: Yutaka Kondo <yutaka.kondo@youtalk.jp>
* fix: fix ticket links in CHANGELOG.rst (`#9588 <https://github.com/autowarefoundation/autoware_universe/issues/9588>`_)
* 0.39.0
* update changelog
* fix: fix ticket links to point to https://github.com/autowarefoundation/autoware_universe (`#9304 <https://github.com/autowarefoundation/autoware_universe/issues/9304>`_)
* fix: fix ticket links to point to https://github.com/autowarefoundation/autoware_universe (`#9304 <https://github.com/autowarefoundation/autoware_universe/issues/9304>`_)
* chore(package.xml): bump version to 0.38.0 (`#9266 <https://github.com/autowarefoundation/autoware_universe/issues/9266>`_) (`#9284 <https://github.com/autowarefoundation/autoware_universe/issues/9284>`_)
  * unify package.xml version to 0.37.0
  * remove system_monitor/CHANGELOG.rst
  * add changelog
  * 0.38.0
  ---------
* Contributors: Esteve Fernandez, Fumiya Watanabe, Ryohsuke Mitsudome, Yutaka Kondo

0.39.0 (2024-11-25)
-------------------
* fix: fix ticket links to point to https://github.com/autowarefoundation/autoware_universe (`#9304 <https://github.com/autowarefoundation/autoware_universe/issues/9304>`_)
* fix: fix ticket links to point to https://github.com/autowarefoundation/autoware_universe (`#9304 <https://github.com/autowarefoundation/autoware_universe/issues/9304>`_)
* chore(package.xml): bump version to 0.38.0 (`#9266 <https://github.com/autowarefoundation/autoware_universe/issues/9266>`_) (`#9284 <https://github.com/autowarefoundation/autoware_universe/issues/9284>`_)
  * unify package.xml version to 0.37.0
  * remove system_monitor/CHANGELOG.rst
  * add changelog
  * 0.38.0
  ---------
* Contributors: Esteve Fernandez, Yutaka Kondo

0.38.0 (2024-11-08)
-------------------
* unify package.xml version to 0.37.0
* refactor(vehicle_info_utils)!: prefix package and namespace with autoware (`#7353 <https://github.com/autowarefoundation/autoware_universe/issues/7353>`_)
  * chore(autoware_vehicle_info_utils): rename header
  * chore(bpp-common): vehicle info
  * chore(path_optimizer): vehicle info
  * chore(velocity_smoother): vehicle info
  * chore(bvp-common): vehicle info
  * chore(static_centerline_generator): vehicle info
  * chore(obstacle_cruise_planner): vehicle info
  * chore(obstacle_velocity_limiter): vehicle info
  * chore(mission_planner): vehicle info
  * chore(obstacle_stop_planner): vehicle info
  * chore(planning_validator): vehicle info
  * chore(surround_obstacle_checker): vehicle info
  * chore(goal_planner): vehicle info
  * chore(start_planner): vehicle info
  * chore(control_performance_analysis): vehicle info
  * chore(lane_departure_checker): vehicle info
  * chore(predicted_path_checker): vehicle info
  * chore(vehicle_cmd_gate): vehicle info
  * chore(obstacle_collision_checker): vehicle info
  * chore(operation_mode_transition_manager): vehicle info
  * chore(mpc): vehicle info
  * chore(control): vehicle info
  * chore(common): vehicle info
  * chore(perception): vehicle info
  * chore(evaluator): vehicle info
  * chore(freespace): vehicle info
  * chore(planning): vehicle info
  * chore(vehicle): vehicle info
  * chore(simulator): vehicle info
  * chore(launch): vehicle info
  * chore(system): vehicle info
  * chore(sensing): vehicle info
  * fix(autoware_joy_controller): remove unused deps
  ---------
* Contributors: Satoshi OTA, Yutaka Kondo

0.26.0 (2024-04-03)
-------------------
* feat: remove use_pointcloud_container (`#6115 <https://github.com/autowarefoundation/autoware_universe/issues/6115>`_)
  * feat!: remove use_pointcloud_container
  * fix pre-commit
  * fix: completely remove use_pointcloud_container after merge main
  * fix: set use_pointcloud_container = true
  * revert: revert change in probabilistic_occupancy_grid_map
  * revert change in launcher of ogm
  ---------
* build: mark autoware_cmake as <buildtool_depend> (`#3616 <https://github.com/autowarefoundation/autoware_universe/issues/3616>`_)
  * build: mark autoware_cmake as <buildtool_depend>
  with <build_depend>, autoware_cmake is automatically exported with ament_target_dependencies() (unecessary)
  * style(pre-commit): autofix
  * chore: fix pre-commit errors
  ---------
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
  Co-authored-by: Kenji Miyake <kenji.miyake@tier4.jp>
* feat(tier4_perception_launch): sync perception launch to autoware_launch (`#2168 <https://github.com/autowarefoundation/autoware_universe/issues/2168>`_)
  * sync launch file from tier4 autoware launch
  * sync tlr launcher
  * ci(pre-commit): autofix
  * sync launch file from tier4 autoware launch
  * sync tlr launcher
  * ci(pre-commit): autofix
  * fix exec_depend in package.xml
  * Sync traffic light node
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
* ci(pre-commit): format SVG files (`#2172 <https://github.com/autowarefoundation/autoware_universe/issues/2172>`_)
  * ci(pre-commit): format SVG files
  * ci(pre-commit): autofix
  * apply pre-commit
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
* chore: upgrade cmake_minimum_required to 3.14 (`#856 <https://github.com/autowarefoundation/autoware_universe/issues/856>`_)
* refactor: use autoware cmake (`#849 <https://github.com/autowarefoundation/autoware_universe/issues/849>`_)
  * remove autoware_auto_cmake
  * add build_depend of autoware_cmake
  * use autoware_cmake in CMakeLists.txt
  * fix bugs
  * fix cmake lint errors
* chore: remove bad chars (`#845 <https://github.com/autowarefoundation/autoware_universe/issues/845>`_)
* style: fix format of package.xml (`#844 <https://github.com/autowarefoundation/autoware_universe/issues/844>`_)
* ci(pre-commit): update pre-commit-hooks-ros (`#625 <https://github.com/autowarefoundation/autoware_universe/issues/625>`_)
  * ci(pre-commit): update pre-commit-hooks-ros
  * ci(pre-commit): autofix
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
* fix: delete common_sensor_launch from exec_depend (`#626 <https://github.com/autowarefoundation/autoware_universe/issues/626>`_)
* feat(tier4_autoware_launch)!: move package to autoware_launch (`#420 <https://github.com/autowarefoundation/autoware_universe/issues/420>`_)
  * feat(tier4_autoware_launch)!: move package to autoware_launch
  * remove unnecessary depends
* feat: change launch package name (`#186 <https://github.com/autowarefoundation/autoware_universe/issues/186>`_)
  * rename launch folder
  * autoware_launch -> tier4_autoware_launch
  * integration_launch -> tier4_integration_launch
  * map_launch -> tier4_map_launch
  * fix
  * planning_launch -> tier4_planning_launch
  * simulator_launch -> tier4_simulator_launch
  * control_launch -> tier4_control_launch
  * localization_launch -> tier4_localization_launch
  * perception_launch -> tier4_perception_launch
  * sensing_launch -> tier4_sensing_launch
  * system_launch -> tier4_system_launch
  * ci(pre-commit): autofix
  * vehicle_launch -> tier4_vehicle_launch
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
  Co-authored-by: tanaka3 <ttatcoder@outlook.jp>
  Co-authored-by: taikitanaka3 <65527974+taikitanaka3@users.noreply.github.com>
* Contributors: Hiroki OTA, Kenji Miyake, Shunsuke Miura, Tomoya Kimura, Vincent Richard, kminoda
