^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package sample_vehicle_launch
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.53.0 (2026-10-13)
-------------------
* Merge remote-tracking branch 'origin/main' into tmp/bot/bump_version_base
* feat(autoware_sample_designs): compose the AD API, system and vehicle components from design modules (`#1976 <https://github.com/autowarefoundation/autoware_launch/issues/1976>`_)
  * fix(control, planning): connect the operation mode state and velocity limit inputs
  Wires collision_detector's operation_mode_state and
  remaining_distance_time_calculator's planning_velocity through the
  module designs; their node designs now declare the fixed topic names
  as remap targets.
  * fix(planning): connect the internal velocity limit path to the selector
  Forwards motion_velocity_planner's velocity_limit and clear_velocity_limit
  up to ScenarioPlanning and feeds them into external_velocity_limit_selector,
  whose internal inputs were unconnected.
  * fix(control, planning): connect the remaining module inputs to their sources
  Wires the operation mode transition manager's gate and selector inputs,
  the stop mode operator's vehicle status, the manual lane change handler's
  route availability and preferred primitive service, the parking costmap,
  and the route marker. Exposes the vehicle control mode service, the
  preferred lane service and the system hazard lights command as component
  interfaces.
  * fix(planning): follow the hazard lights selector input rename
  * fix(planning): add steering_status subscriber to various planning modules
  * feat(vehicle): add vehicle wrapper and launch file for vehicle interface
  * style(pre-commit): autofix
  * fix(planning): add planning factors publishers and connections for various modules
  * refactor(autoware_sample_designs): compose the AD API from design modules
  * Refactor ADAPI module structure and enhance planning factors
  - Updated AdapiRvizHelpers.module.yaml to include helper prefixes for instances and connections.
  - Modified AutowareSample.system.yaml to reorganize ADAPI components, introducing default_adapi for better structure.
  - Expanded planning factors in Planning.module.yaml, LaneDriving.module.yaml, ScenarioPlanning.module.yaml, and associated modules to include additional factors for improved behavior and motion planning.
  - Enhanced connections in behavior planning and motion planning modules to accommodate new planning factors.
  - Ensured consistency across scenario planning modules by aligning publisher and subscriber connections for new planning factors.
  * refactor: reorganize clients and servers in module configurations
  * fix(control): drop the collision detector operation mode link bound globally by the node design
  * implement system modules
  * vehicle modules and nodes, need validations
  * fix api topic remapped
  * reorganize to adapi and api
  * fix: update perception obstacle segmentation filters for clarity and functionality
  * refactor: update trigger conditions and outcomes in topic tools configuration
  ---------
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
* Contributors: Taekjin LEE, github-actions

0.52.0 (2026-06-30)
-------------------

0.51.0 (2026-05-01)
-------------------

0.50.0 (2026-02-13)
-------------------

0.49.0 (2025-12-30)
-------------------

0.48.0 (2025-11-16)
-------------------

0.47.0 (2025-08-11)
-------------------

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
* chore: set all versions to 0.43.0 (`#1384 <https://github.com/autowarefoundation/autoware_launch/issues/1384>`_)
* feat(*_launch): move here (`#1369 <https://github.com/autowarefoundation/autoware_launch/issues/1369>`_)
* Contributors: Mete Fatih Cırıt, Ryohsuke Mitsudome, Yutaka Kondo

0.43.1 (2025-04-01)
-------------------
* chore: set all versions to 0.43.0 (`#1384 <https://github.com/autowarefoundation/autoware_launch/issues/1384>`_)
* feat(*_launch): move here (`#1369 <https://github.com/autowarefoundation/autoware_launch/issues/1369>`_)
* Contributors: Mete Fatih Cırıt

0.43.0 (2025-03-24)
-------------------

0.42.0 (2025-03-21)
-------------------

0.41.0 (2025-02-07)
-------------------

0.40.0 (2025-01-09)
-------------------

0.39.0 (2024-11-26)
-------------------

0.38.0 (2024-11-14)
-------------------
