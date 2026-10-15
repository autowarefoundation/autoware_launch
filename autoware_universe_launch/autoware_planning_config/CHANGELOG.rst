^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package autoware_planning_config
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.53.0 (2026-10-13)
-------------------
* Merge remote-tracking branch 'origin/main' into tmp/bot/bump_version_base
* chore(autoware_trajectory_processor): rename to autoware_trajectory_modifier (`#1986 <https://github.com/autowarefoundation/autoware_launch/issues/1986>`_)
  rename processor -> modifier
* chore(sync-params): update planning params (`#1978 <https://github.com/autowarefoundation/autoware_launch/issues/1978>`_)
  Co-authored-by: SakodaShintaro <28504069+SakodaShintaro@users.noreply.github.com>
* feat(trajectory_selectory): add missing config files (`#1979 <https://github.com/autowarefoundation/autoware_launch/issues/1979>`_)
  add missing config files
  - add validator.param.yaml config file
  - add ranker.param.yaml config file
  - add selector.param.yaml config file
  - update sync-params.yaml
* chore(sync-params): update planning params (`#1977 <https://github.com/autowarefoundation/autoware_launch/issues/1977>`_)
  Co-authored-by: SakodaShintaro <28504069+SakodaShintaro@users.noreply.github.com>
* fix(sync-params): correct the trajectory_processor source header and sync planning params (`#1959 <https://github.com/autowarefoundation/autoware_launch/issues/1959>`_)
  The sync-params header of trajectory_processor.param.yaml pointed to
  trajectory_modifier.param.yaml while the workflow config maps it to
  trajectory_processor.param.yaml, making sync_params.py fail with
  'Source path mismatch' on both the scheduled sync-params job and the
  check-params PR job for the planning category.
  Fix the header source path and run 'sync_params.py planning' to bring
  the planning params back in sync with the upstream repositories
  (new diffusion_planner ego_snap_to_prev_trajectory params, refreshed
  source commit hashes).
  Co-authored-by: Claude Fable 5 <noreply@anthropic.com>
* refactor(planning)!: migrate tier4_planning_launch to autoware_planning_launch with a separate config package (`#1954 <https://github.com/autowarefoundation/autoware_launch/issues/1954>`_)
  * Add autoware_planning_launch package and related files
  - Created package.xml for autoware_planning_launch with dependencies and maintainers.
  - Added planning_launch.drawio.svg for visual representation of the planning launch structure.
  - Removed obsolete README.md and planning.launch.xml from tier4_planning_launch.
  * feat(planning): make the diffusion planner tracked objects topic configurable
  Expose the tracked objects topic as `input_tracked_objects_topic_name`
  alongside the objects and pointcloud topics, plumbed from the planning
  component down to the diffusion planner unit.
  * refactor(planning): drop the redundant group around the planning component include
  `<group>` is scoped by default, so the conditional group already bounds the
  component's launch configurations. The extra group is only needed where sibling
  component includes share a scope.
  * Update .github/CODEOWNERS
  Co-authored-by: Shumpei Wakabayashi <42209144+shmpwk@users.noreply.github.com>
  * refactor: update CODEOWNERS and package maintainers for planning components
  * chore: restore auto-generated CODEOWNERS
  ---------
  Co-authored-by: Shumpei Wakabayashi <42209144+shmpwk@users.noreply.github.com>
* Contributors: Masaya Kataoka, Maxime CLEMENT, Taekjin LEE, awf-autoware-bot[bot], github-actions, mkquda
