^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package autoware_map_config
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.53.0 (2026-10-13)
-------------------
* Merge remote-tracking branch 'origin/main' into tmp/bot/bump_version_base
* refactor(map)!: migrate tier4_map_launch to autoware_map_launch with a separate config package (`#1980 <https://github.com/autowarefoundation/autoware_launch/issues/1980>`_)
  Part of `#1939 <https://github.com/autowarefoundation/autoware_launch/issues/1939>`_ (map item). Splits the map launch stack into unit launchers under
  autoware_universe_launch/autoware_map_launch and moves its parameter tree into the
  data-only autoware_map_config, with component_map.launch.xml as the composition root.
  The resolved launch graph is unchanged.
  Co-authored-by: Claude Opus 5 <noreply@anthropic.com>
* Contributors: Shumpei Wakabayashi, github-actions
