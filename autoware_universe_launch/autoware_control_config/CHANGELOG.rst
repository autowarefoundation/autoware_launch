^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package autoware_control_config
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.53.0 (2026-10-13)
-------------------
* Merge remote-tracking branch 'origin/main' into tmp/bot/bump_version_base
* refactor(control)!: migrate tier4_control_launch to autoware_control_launch with a separate config package (`#1962 <https://github.com/autowarefoundation/autoware_launch/issues/1962>`_)
  * refactor(control)!: migrate tier4_control_launch to autoware_control_launch with a separate config package
  Move the control launch stack out of the product layer and split its
  parameter tree into autoware_control_config. control.launch.xml resolves
  every parameter file through control_config_pkg and includes the module
  preset itself; the launch graph is unchanged.
  * refactor(control): move the control design modules alongside the control launch package
  Control.module moves to the sample design module root and
  TrajectoryFollower.module into autoware_control_launch/design/module,
  matching the perception layout.
  * refactor(control): enhance control parameter handling and update README for clarity
  * refactor(control): streamline control configuration handling and update related documentation
  ---------
* Contributors: Taekjin LEE, github-actions
