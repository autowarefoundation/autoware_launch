^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package awsim_sensor_kit_launch
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

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
