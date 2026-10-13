^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package autoware_localization_config
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.53.0 (2026-10-13)
-------------------
* Merge remote-tracking branch 'origin/main' into tmp/bot/bump_version_base
* refactor(localization)!: migrate tier4_localization_launch to autoware_localization_launch with a separate config package (`#1971 <https://github.com/autowarefoundation/autoware_launch/issues/1971>`_)
  * refactor(localization)!: migrate tier4_localization_launch to autoware_localization_launch with a separate config package
  Move the localization launch stack out of the product layer and split its
  parameter tree into autoware_localization_config. localization.launch.xml
  resolves every parameter file through localization_config_pkg; the launch
  graph is unchanged.
  * refactor(localization): move the localization design modules alongside the localization launch package
  The localization modules move into autoware_localization_launch/design/module,
  matching the planning and control layout; the parameter set and the wrapper
  resolve parameter files from autoware_localization_config.
  * refactor(localization): flatten pose/twist source selection into per-estimator junctions
  pose_source/twist_source string parsing moves into the product component;
  localization.launch.xml and pose_twist_estimator.launch.xml take one flag
  per estimator plus the arbiter's pose_sources list, so a fixed system can
  pin the junctions directly. The launch graph is unchanged.
  * refactor(localization): restore the module-shaped launch composition
  localization.launch.xml composes the same instances as Localization.module:
  the util module (pose initializer and the ndt input downsampling) becomes a
  sibling group with launch/util/util.launch.xml as the LocalizationUtil
  counterpart, and cross-module wiring (the arbiter relay input, eagleye
  standing in for the gnss pose) is resolved at the composition root. The
  launch graph is unchanged.
  * refactor(localization)!: compose localization from self-contained unit launchers
  The all-mode entry launchers (localization.launch.xml and
  pose_twist_estimator.launch.xml) are removed: the launch package ships one
  launcher per unit, each resolving only its own parameters from
  localization_config_pkg, and component_localization.launch.xml is the
  composition root that includes only the units the selected sources need —
  the perception layout. The unreferenced Tier4LocalizationWrapper design is
  dropped with its launch entry. The launch graph is unchanged.
  * style(pre-commit): autofix
  * fix(localization): connect the automatic pose initializer in LocalizationUtil
  Wires the pose initializer state and initialize service to
  automatic_pose_initializer, whose node design now declares the fixed
  API names as remap targets.
  * chore(localization): drop the exec_depends the composition root owns
  autoware_pose_estimator_arbiter and autoware_pose_covariance_modifier are
  included from component_localization.launch.xml, which autoware_launch
  declares.
  * docs(launch): drop the stale launch diagrams and retarget the config usage example
  Both package structure diagrams draw the localization.launch.xml composition
  this series removes, and the autoware_launch one also draws the simulator
  wrappers as if the root launcher called the sub launchers. The config package
  documents the composition root as the place localization_config_pkg is set.
  * fix(localization): resolve stop_check_enabled for every system run mode
  The stop check belongs to a real vehicle, so any run mode other than
  online disables it; the value is defined unconditionally so a product
  run mode outside the in-tree set still resolves.
  * chore(localization): carry the changelog over unedited
  The recorded entries reference upstream pull requests by their original
  package name, matching the control, planning and perception migrations.
  ---------
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
* Contributors: Taekjin LEE, github-actions
