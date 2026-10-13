^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package awsim_sensor_kit_launch
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.53.0 (2026-10-13)
-------------------
* Merge remote-tracking branch 'origin/main' into tmp/bot/bump_version_base
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
* Contributors: Taekjin LEE, github-actions

0.52.0 (2026-06-30)
-------------------
* Merge remote-tracking branch 'origin/main' into tmp/bot/bump_version_base
* chore: sync files (`#1338 <https://github.com/autowarefoundation/autoware_launch/issues/1338>`_)
  * chore: sync files
  * style(pre-commit): autofix
  ---------
  Co-authored-by: github-actions <github-actions@github.com>
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
* Contributors: awf-autoware-bot[bot], github-actions

0.51.0 (2026-05-01)
-------------------

0.50.0 (2026-02-13)
-------------------
* Merge remote-tracking branch 'origin/main' into tmp/bot/bump_version_base
* feat(awsim_sensor_kit_launch): remove side lidar references (`#1733 <https://github.com/autowarefoundation/autoware_launch/issues/1733>`_)
* Contributors: Mete Fatih Cırıt, github-actions

0.49.0 (2025-12-30)
-------------------

0.48.0 (2025-11-16)
-------------------
* Merge remote-tracking branch 'origin/main' into release-humble-0.48.0
* feat: change AWSIM GNSS format to NavSatFix (`#1640 <https://github.com/autowarefoundation/autoware_launch/issues/1640>`_)
* chore: necessary mod to run with latest gyro scale estimation changes (`#1633 <https://github.com/autowarefoundation/autoware_launch/issues/1633>`_)
* Contributors: SergioReyesSan, mitsudome-r, zzhhaa10000

0.47.0 (2025-08-11)
-------------------
* feat(autoware_pointcloud_preprocessor): add remappings (`#1563 <https://github.com/autowarefoundation/autoware_launch/issues/1563>`_)
* Contributors: Amadeusz Szymko

0.46.0 (2025-06-20)
-------------------

0.45.3 (2025-07-17)
-------------------

0.45.2 (2025-06-28)
-------------------

0.45.1 (2025-06-27)
-------------------

0.45.0 (2025-05-22)
-------------------

0.44.3 (2025-06-10)
-------------------

0.44.2 (2025-05-30)
-------------------

0.44.1 (2025-05-12)
-------------------

0.44.0 (2025-05-01)
-------------------
* Merge commit 'bdbc8e8' into bump-up-version-to-0.44.0
* chore: bump version to 0.44.0 (`#1411 <https://github.com/autowarefoundation/autoware_launch/issues/1411>`_)
* feat: remove individual_params references (`#1403 <https://github.com/autowarefoundation/autoware_launch/issues/1403>`_)
* chore(sensor_kit_launch): remove unused has_static_tf_only param (`#1393 <https://github.com/autowarefoundation/autoware_launch/issues/1393>`_)
* chore: set all versions to 0.43.0 (`#1384 <https://github.com/autowarefoundation/autoware_launch/issues/1384>`_)
* feat(*_launch): move here (`#1369 <https://github.com/autowarefoundation/autoware_launch/issues/1369>`_)
* Contributors: Amadeusz Szymko, Mete Fatih Cırıt, Ryohsuke Mitsudome, Yutaka Kondo

0.43.1 (2025-04-01)
-------------------
* chore: set all versions to 0.43.0 (`#1384 <https://github.com/autowarefoundation/autoware_launch/issues/1384>`_)
* feat(*_launch): move here (`#1369 <https://github.com/autowarefoundation/autoware_launch/issues/1369>`_)
* Contributors: Mete Fatih Cırıt

0.43.0 (2025-03-28)
-------------------

0.42.0 (2025-03-28)
-------------------
* refactor(awsim_sensor_kit_launch): remove reference to tamagawa_imu_driver (`#32 <https://github.com/tier4/awsim_sensor_kit_launch/issues/32>`_)
* Contributors: Esteve Fernandez, M. Fatih Cırıt

0.41.0 (2025-02-12)
-------------------
* Merge remote-tracking branch 'origin/main' into humble
* Merge pull request `#26 <https://github.com/tier4/awsim_sensor_kit_launch/issues/26>`_ from SakodaShintaro/fix/add_udp_only
  fix: add `udp_only`
* Merge pull request `#25 <https://github.com/tier4/awsim_sensor_kit_launch/issues/25>`_ from vividf/feature/tier4_awsim_load_concatenate
  feat(awsim_sensor_kit_launch): awsim load concatenate node parameters
* chore: increase window size
* chore: add mathcing strategy
* chore: add use_naive_approach
* fix(awsim_sensor_kit_launch): add autoware prefix to vehicle_velocity_converter (`#27 <https://github.com/tier4/awsim_sensor_kit_launch/issues/27>`_)
  * add autoware prefix
  * add dependency autoware_vehicle_velocity_converter
  ---------
* chore: update params
* Added `udp_only`
* chore: fix timeout sec
* chore: add space
* feat: awsim load comcatenate parameter
* Contributors: Masaki Baba, Ryohsuke Mitsudome, SakodaShintaro, Shintaro Sakoda, vividf

0.39.0 (2024-12-09)
-------------------
* Merge pull request `#24 <https://github.com/tier4/awsim_sensor_kit_launch/issues/24>`_ from SakodaShintaro/fix/restore_timeout_sec
  fix: restore timeout_sec
* Restore timeout_sec
* fix: fix to use sample sensor kit (`#22 <https://github.com/tier4/awsim_sensor_kit_launch/issues/22>`_)
  * Fixed to use sample_sensor_kit
  * Removed common_awsim_sensor_launch
  * Added a trailing break line
  ---------
* feat: fix to be able to use nebula (`#19 <https://github.com/tier4/awsim_sensor_kit_launch/issues/19>`_)
  * Merge https://github.com/knzo25/awsim_sensor_kit_launch/tree/test/awsim_nebula_integration
  * Fixed some code
  * Fixed order
  * FIxed order
  ---------
* Added autoware\_ prefix to gnss_poser (`#17 <https://github.com/tier4/awsim_sensor_kit_launch/issues/17>`_)
* Merge pull request `#16 <https://github.com/tier4/awsim_sensor_kit_launch/issues/16>`_ from knzo25/chore/imu_corrector_refactor
  chore: changed the package name imu_corrector to autoware_imu_corrector
* chore: changed the package name imu_corrector to autoware_imu_corrector
* refactor: pointcloud_preprocessor prefix package and namespace with autoware (`#15 <https://github.com/tier4/awsim_sensor_kit_launch/issues/15>`_)
* fix: fix container name handling (`#14 <https://github.com/tier4/awsim_sensor_kit_launch/issues/14>`_)
* feat: always separate lidar preprocessing from pointcloud_container (`#13 <https://github.com/tier4/awsim_sensor_kit_launch/issues/13>`_)
  * feat!: replace use_pointcloud_container
  * revert: revert rename of use_pointcloud_container
  * revert: fix comment
  ---------
* rename lidar topic (`#12 <https://github.com/tier4/awsim_sensor_kit_launch/issues/12>`_)
  Co-authored-by: yamato-ando <Yamato ANDO>
* Fixed gyro_bias_estimator input (`#11 <https://github.com/tier4/awsim_sensor_kit_launch/issues/11>`_)
* Added gyro_bias_estimator (`#10 <https://github.com/tier4/awsim_sensor_kit_launch/issues/10>`_)
* fix(pointcloud_preprocessor): organize input twist topic (`#8 <https://github.com/tier4/awsim_sensor_kit_launch/issues/8>`_)
  * fix(pointcloud_preprocessor): organize input twist topic
  * fix
  ---------
* Merge pull request `#5 <https://github.com/tier4/awsim_sensor_kit_launch/issues/5>`_ from RobotecAI/feat/shorter_timeout
  feat: shorten timeout for subscribing dummy lidar
* feat: shorten timeout for subscribing dummy lidar
* Merge pull request `#2 <https://github.com/tier4/awsim_sensor_kit_launch/issues/2>`_ from RobotecAI/chore/distortion_filter_false
  chore(awsim_sensor_kit_launch): set use_distortion_corrector false
* chore(awsim_sensor_kit_launch): set use_distortion_corrector false
* Launchfiles cleanup
* AWSIM sensor kit
* Contributors: Amadeusz Szymko, Kenzo Lobos-Tsunekawa, Lukasz Chojnacki, Piotr Jaroszek, SakodaShintaro, Shintaro Sakoda, Shumpei Wakabayashi, TaikiYamada4, Yamato Ando, kminoda
