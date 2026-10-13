^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package autoware_perception_config
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.53.0 (2026-10-13)
-------------------
* Merge remote-tracking branch 'origin/main' into tmp/bot/bump_version_base
* fix(autoware_perception_config): add missing min_cluster_size_m to label_based_euclidean_cluster (`#1969 <https://github.com/autowarefoundation/autoware_launch/issues/1969>`_)
  * fix(autoware_perception_config): add missing min_cluster_size_m to label_based_euclidean_cluster
  autoware_universe`#13248 <https://github.com/autowarefoundation/autoware_launch/issues/13248>`_ (2026-08-27) added `min_cluster_size_m` to
  LabelBasedEuclideanClusterNode as a required parameter -- it is read with
  get_or_declare_parameter<double>() with no fallback, and the package schema
  lists it under "required". The package's own reference config was updated in
  that PR, but this deployed copy in autoware_perception_config was not, so the
  node aborted at construction:
  terminate called after throwing an instance of 'rclcpp::ParameterTypeException'
  what():  expected [double] got [not set]
  The node is only launched on the semantic segmentation path, so the breakage is
  invisible unless use_semantic_segmentation_ptv3:=true, which is how it went
  unnoticed.
  Values mirror the reference config shipped with the package in the same PR: a
  1.5 m global minimum XY bounding-circle diameter, disabled (0.0) for the
  small-object labels that have override blocks here. Only the labels present in
  this file are touched; the remaining drift from the package config (missing
  motorcycle/animal blocks, truck_trailer vs truck_trailer_car group) is left
  alone as it changes detection behavior beyond this crash fix.
  Co-Authored-By: Claude Opus 5 (1M context) <noreply@anthropic.com>
  * sync missing param
  ---------
  Co-authored-by: Claude Opus 5 (1M context) <noreply@anthropic.com>
* refactor(perception)!: extract the perception parameter tree into autoware_perception_config and modularize the simulated perception pipeline (`#1955 <https://github.com/autowarefoundation/autoware_launch/issues/1955>`_)
  * Refactor perception launch files to use autoware_perception_config package
  - Removed the perception_config_pkg argument from multiple launch files.
  - Updated parameter paths in various launch files to reference autoware_perception_config instead of perception_config_pkg.
  - Ensured consistency across object recognition, obstacle segmentation, occupancy grid map, and traffic light recognition launch files.
  - Added autoware_perception_config as a dependency in package.xml.
  * Refactor perception launch files to use dynamic configuration paths
  - Added a new argument `perception_config_pkg` to specify the perception configuration package.
  - Introduced `perception_config_dir` variable to dynamically set the path for configuration files across multiple launch files.
  - Updated parameter paths in various launch files to utilize the new dynamic configuration directory, enhancing maintainability and flexibility.
  - Modified launch files related to object recognition, obstacle segmentation, occupancy grid mapping, and traffic light recognition to reflect these changes.
  * chore(CODEOWNERS): remove obsolete perception config path
  * refactor: update perception configuration for simulator launch files
  * refactor: remove traffic light parameters from perception launch files
  * refactor: integrate perception simulation and update tier4 simulator component
  * chore(CODEOWNERS): restore auto-generated file
  * refactor: add elevation map support to perception simulation and update README
  * ci: distribute lint-autoware-system-design-format via sync-files
  ---------
* Contributors: Taekjin LEE, badai nguyen, github-actions
