^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package autoware_launch
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.47.0 (2025-08-11)
-------------------
* feat: change planning output topic name to /planning/trajectory (`#1594 <https://github.com/autowarefoundation/autoware_launch/issues/1594>`_)
  * change planning output topic name to /planning/trajectory
  * style(pre-commit): autofix
  ---------
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
* fix(multi_object_tracker): remove confident_count_threshold parameter (`#1592 <https://github.com/autowarefoundation/autoware_launch/issues/1592>`_)
* feat(motion_velocity_planner): update pointcloud preprocess design (`#1588 <https://github.com/autowarefoundation/autoware_launch/issues/1588>`_)
  * add new params for pcl preprocess
  ---------
* feat(perception_online_evaluator): change launch setting to enable node (`#1583 <https://github.com/autowarefoundation/autoware_launch/issues/1583>`_)
  * feat: enable perception online evaluator
  * chore: add a new argument to enable autoware_perception_analytics_publisher_node instead of previous autoware_perception_online_evaluator_node
  ---------
  Co-authored-by: Jian Kang <jian.kang@tier4.jp>
* fix(multi_object_tracker): add param for irregular_object (`#1587 <https://github.com/autowarefoundation/autoware_launch/issues/1587>`_)
  * fix(multi_object_tracker): add param for irregular_object
  * fix: update param
  ---------
* fix(tracker): update pedestrian max_area_matrix for large pedestrian (`#1589 <https://github.com/autowarefoundation/autoware_launch/issues/1589>`_)
  fix(data_association_matrix): update pedestrian max_area_matrix for large pedestrian
* feat(boundary_departure): configurable departure points and type based on time (`#1579 <https://github.com/autowarefoundation/autoware_launch/issues/1579>`_)
  * feat(boundary_departure): configurable departure points and type based on time
  * set predicted path and near boundary to same value
  * increase time_buffer value
  ---------
* feat(boundary_departure): rename parameter (`#1586 <https://github.com/autowarefoundation/autoware_launch/issues/1586>`_)
* feat(blind_spot): new re-designed blind_spot module (`#1582 <https://github.com/autowarefoundation/autoware_launch/issues/1582>`_)
* refactor(planning_validator, trajectory_checker): update config (`#1581 <https://github.com/autowarefoundation/autoware_launch/issues/1581>`_)
  * update parameter files
  * rename param
  ---------
* feat(rear_collision_checker): support selecting safety metric from TTC or RSS (`#1584 <https://github.com/autowarefoundation/autoware_launch/issues/1584>`_)
  feat: selectable metric
* feat(road_user_stop): add road_user_stop module config (`#1566 <https://github.com/autowarefoundation/autoware_launch/issues/1566>`_)
  * add road_user_stop module config
  * update parameters
  * update parameter
  * update parameter
  * fix based on PR review
  * fix copy misstake of parameter
  * change parameter structure for opposing traffic
  * add virtual wall
  * format param.yaml
  ---------
* fix(path_generator): merge waypoint groups with shared overlap interval (`#1576 <https://github.com/autowarefoundation/autoware_launch/issues/1576>`_)
  replace parameters with new one
* feat(run_out): add parameters to select which debug markers to publish (`#1580 <https://github.com/autowarefoundation/autoware_launch/issues/1580>`_)
* refactor(rear_collision_checker): update parameter structure (`#1578 <https://github.com/autowarefoundation/autoware_launch/issues/1578>`_)
* feat(autoware_pipeline_latency_monitor): add autoware_pipeline_latency_monitor package (`#1569 <https://github.com/autowarefoundation/autoware_launch/issues/1569>`_)
  * feat(sensor_to_control_latency_checker): add latency checker parameters and update launch configuration
  * rename and set initial params
  * fix(pipeline_latency_monitor): update control latency value to 15.0
  * fix(pipeline_latency_monitor): update timestamp meaning to "end" for processing steps
  * fix(tier4_system_component): correct pipeline latency monitor parameter path
  ---------
  Co-authored-by: Maxime CLEMENT <maxime.clement@tier4.jp>
* feat(intersection_collision_checker): add parameter max_history_time (`#1575 <https://github.com/autowarefoundation/autoware_launch/issues/1575>`_)
  add parameter max_history_time
* feat(autoware_vehicle_cmd_gate): steer rate limit with lateral jerk constraint (`#1574 <https://github.com/autowarefoundation/autoware_launch/issues/1574>`_)
  feat(vehicle_cmd_gate): add lat_jerk_lim_for_steer_rate parameter for improved steering control
* refactor(vehicle_cmd_gate): rename variable naming consistency in vehicle command filter (`#1570 <https://github.com/autowarefoundation/autoware_launch/issues/1570>`_)
  * fix(vehicle_cmd_gate): standardize steering parameter naming for consistency
  ---------
* feat(start_planner): add clothoid path parameters (`#1573 <https://github.com/autowarefoundation/autoware_launch/issues/1573>`_)
  * feat(start_planner): add clothoid path parameters
  * fix(start_planner): correct parameter name for maximum steer angles in clothoid pull out
  ---------
* feat(launch): add flag to launch remaining distance calculator (`#1572 <https://github.com/autowarefoundation/autoware_launch/issues/1572>`_)
* feat(map_based_prediction): prevent predicted path from chattering under noisy pose and velocity estimation (`#1571 <https://github.com/autowarefoundation/autoware_launch/issues/1571>`_)
* fix(object_merger): update launch and add param file (`#1568 <https://github.com/autowarefoundation/autoware_launch/issues/1568>`_)
  * fix: update object_merger param
  * fix(object_merger): update launch and add param file
  ---------
* feat(multi_object_tracker): add generalized iou thresholds and overlap distance thresholds (`#1564 <https://github.com/autowarefoundation/autoware_launch/issues/1564>`_)
  * feat(multi_object_tracker): add generalized IoU thresholds and overlap distance thresholds
  * style(pre-commit): autofix
  * fix(multi_object_tracker): correct formatting of IoU and overlap distance thresholds
  * feat(multi_object_tracker): add GIoU threshold for unknown-unknown association
  * fix(multi_object_tracker): rename generalized IoU and distance thresholds to pruning parameters
  ---------
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
* feat(crosswalk): improve robustness to prevent stop decision from being canceled by transient noise (`#1565 <https://github.com/autowarefoundation/autoware_launch/issues/1565>`_)
* feat(intersection_collision_checker): update intersection collision checker params (`#1562 <https://github.com/autowarefoundation/autoware_launch/issues/1562>`_)
  * update intersection collision checker params
  * add parameters
  ---------
* feat(rear_collision_checker): added a parameter to configure how many seconds ahead to predict collisions (`#1552 <https://github.com/autowarefoundation/autoware_launch/issues/1552>`_)
* feat(obstacle_slow_down): rework type specific params, split left/right (`#1553 <https://github.com/autowarefoundation/autoware_launch/issues/1553>`_)
* fix(diagnostics): remove edits from AWSIM diagnostic file (`#1558 <https://github.com/autowarefoundation/autoware_launch/issues/1558>`_)
* feat(out_of_lane): add objects.extra_width parameter (`#1537 <https://github.com/autowarefoundation/autoware_launch/issues/1537>`_)
* feat(path_generator): improve goal connection for goal on the side (`#1533 <https://github.com/autowarefoundation/autoware_launch/issues/1533>`_)
  * rename parameter
  * change parameter name
  ---------
  Co-authored-by: Kosuke Takeuchi <kosuke.tnp@gmail.com>
* fix(autoware_launch): update trajectory relay to use parameters inste… (`#1555 <https://github.com/autowarefoundation/autoware_launch/issues/1555>`_)
  fix(autoware_launch): update trajectory relay to use parameters instead of remaps
* feat: add launch_pointcloud_container to tier4_localization_component.launch.xml (`#1529 <https://github.com/autowarefoundation/autoware_launch/issues/1529>`_)
* feat: add global parameters to tier4_localization_component.launch.xml (`#1528 <https://github.com/autowarefoundation/autoware_launch/issues/1528>`_)
* fix(obstacle_stop): fix for failing scenario (`#1540 <https://github.com/autowarefoundation/autoware_launch/issues/1540>`_)
  fix for failing scenari
* feat(tracking): add lidar_centerpoint_short_range configuration for multi-object tracker (`#1546 <https://github.com/autowarefoundation/autoware_launch/issues/1546>`_)
  * feat(tracking): add lidar_centerpoint_short_range configuration for multi-object tracker
  * fix(tracking): correct name for lidar_centerpoint_short_range parameter
  ---------
* feat(autoware_pose_instability_detector): make pose_instability_detector configurable in autoware_launch (`#1547 <https://github.com/autowarefoundation/autoware_launch/issues/1547>`_)
  make pose_instability_detector configurable from autoware_launch
* feat: update diag settings for control_command_gate (`#1332 <https://github.com/autowarefoundation/autoware_launch/issues/1332>`_)
  * update diag settings
  * fix command gate diag option
  * fix diagnostics setting
  * fix diagnostics setting
  * add param path
  * merge config
  * remove copied config
  * add control command gate
  ---------
* feat(planning): migrate for changing planning topic name (`#1543 <https://github.com/autowarefoundation/autoware_launch/issues/1543>`_)
  * feat(planning): add temporary relay node for trajectory topic migration
  * style(pre-commit): autofix
  ---------
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
* fix(detection_area): change max_acceleration of detection_area module (`#1544 <https://github.com/autowarefoundation/autoware_launch/issues/1544>`_)
  * fix(detection_area): change max_acceleration to pass evaluator
  * Update autoware_launch/config/planning/scenario_planning/lane_driving/behavior_planning/behavior_velocity_planner/detection_area.param.yaml
  ---------
  Co-authored-by: Kosuke Takeuchi <kosuke.tnp@gmail.com>
* feat(intersection_occlusion): request approval when occluded without traffic light (`#1542 <https://github.com/autowarefoundation/autoware_launch/issues/1542>`_)
* feat(lane_change): update lc frenet parameters (`#1545 <https://github.com/autowarefoundation/autoware_launch/issues/1545>`_)
  add flag use_entire_remaining_distance to lc frenet params
* feat(obstacle_stop_module)!: add leading vehicle following by rss stop position determination (`#1515 <https://github.com/autowarefoundation/autoware_launch/issues/1515>`_)
  * add RSS obstacle_stop feature, enable obstacle stop for all velocity region
  ---------
* feat(perception): update radar object tracker parameter paths for planning simulator (`#1541 <https://github.com/autowarefoundation/autoware_launch/issues/1541>`_)
  * feat(perception): update radar object tracker parameter paths for planning simulator
  * feat(perception): remove redundant radar object tracking parameters from simulator launch file
  ---------
* feat(perception): remove radar tracker and set tracked object lanelet filter (`#1531 <https://github.com/autowarefoundation/autoware_launch/issues/1531>`_)
  * fix: remove radar tracker and set tracked object lanelet filter
  * feat(tracked_object_sorter): add new parameter configuration for radar object tracking
  * fix(tracker_settings): update probability thresholds for object tracking
  ---------
* fix(lane_change): add curvature threshold for frenet planner (`#1536 <https://github.com/autowarefoundation/autoware_launch/issues/1536>`_)
  * fix(lane_change): add curvature threshold for frenet planner
  * update initial param
  * rename max→average
  ---------
* feat(intersection_collision_checker): add velocity estimation parameters (`#1538 <https://github.com/autowarefoundation/autoware_launch/issues/1538>`_)
  add max velocity and reset accel thresholds
* feat(detection_area): add max_acceleration (`#1535 <https://github.com/autowarefoundation/autoware_launch/issues/1535>`_)
* feat(out_of_lane): validate predicted paths on lanelets (`#1516 <https://github.com/autowarefoundation/autoware_launch/issues/1516>`_)
* feat(object_filter): enable lanelet object elevation filter (`#1499 <https://github.com/autowarefoundation/autoware_launch/issues/1499>`_)
  enable lanelet_object_elavation_filter
  Co-authored-by: Taekjin LEE <technolojin@gmail.com>
* feat(run_out): enable new module and disable the previous one (`#1532 <https://github.com/autowarefoundation/autoware_launch/issues/1532>`_)
* feat(velocity_smoother): implemting dynamic lateral acceleration and steering angle rate limit (`#1495 <https://github.com/autowarefoundation/autoware_launch/issues/1495>`_)
* feat(run_out): add option for strict cutting of predicted paths (`#1518 <https://github.com/autowarefoundation/autoware_launch/issues/1518>`_)
* feat(intersection_collision_checker): tune on time buffer param (`#1530 <https://github.com/autowarefoundation/autoware_launch/issues/1530>`_)
  * tune on time buffer param
  * add more configuration params
  * fix format
  ---------
* fix: enable use_dynamic_map_loading (`#1462 <https://github.com/autowarefoundation/autoware_launch/issues/1462>`_)
* feat(obstacle_stop_module): add parameters for "outside" region obstacles (`#1480 <https://github.com/autowarefoundation/autoware_launch/issues/1480>`_)
  * obs stop: add replace outside stop by cut in stop
  * add params
  * rename params
  ---------
* fix(perception): add `pointcloud_container_name` parameter to give (`#1527 <https://github.com/autowarefoundation/autoware_launch/issues/1527>`_)
  add arg parameter to give
* chore(obstacle_cruise_planner, obstacle_stop_planner): remove obstacle planner parameters (`#1526 <https://github.com/autowarefoundation/autoware_launch/issues/1526>`_)
  remove obstacle cruise parameters
* fix(tier4_simulator_component): fix argument name in tier4_simulator_component.launch.xml (`#1522 <https://github.com/autowarefoundation/autoware_launch/issues/1522>`_)
  fix use_point_cloud_container to use_pointcloud_container in tier4_simulator_component.launch.xml
* fix:  `pointcloud_contaner` is launched on `autoware.launch.xml` by default (`#1523 <https://github.com/autowarefoundation/autoware_launch/issues/1523>`_)
* feat(autoware_lidar_centerpoint): update score_threshold to score_thresholds for class-wise thresholds (`#1517 <https://github.com/autowarefoundation/autoware_launch/issues/1517>`_)
  Update score_threshold to score_thresholds for class-wise thresholds
* chore: remove default values of `pointcloud_container_name` (`#1521 <https://github.com/autowarefoundation/autoware_launch/issues/1521>`_)
  remove default value
* feat(launch/components): launch `pointcloud_container` in a component-wise manner (`#1513 <https://github.com/autowarefoundation/autoware_launch/issues/1513>`_)
  * component-wise pointcloud_container
  * remove comments
  * style(pre-commit): autofix
  * Update autoware_launch/launch/components/tier4_perception_component.launch.xml
  Co-authored-by: Yukihiro Saito <yukky.saito@gmail.com>
  ---------
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
  Co-authored-by: Yukihiro Saito <yukky.saito@gmail.com>
* feat: visualize all static_obstacle_avoidance info marker (`#1467 <https://github.com/autowarefoundation/autoware_launch/issues/1467>`_)
* feat(launch/components): launch `autoware_global_parameter_loader` in a component-wise manner (`#1512 <https://github.com/autowarefoundation/autoware_launch/issues/1512>`_)
  add global parameters
* fix(multi_object_tracker): disable debug markers in configuration for improved performance (`#1514 <https://github.com/autowarefoundation/autoware_launch/issues/1514>`_)
* feat(control_validator): 2 thresholds for the yaw deviation (warn/error) (`#1508 <https://github.com/autowarefoundation/autoware_launch/issues/1508>`_)
* feat(intersection_occlusion): adjustable occlusion wall position for intersection without traffic_light (`#1510 <https://github.com/autowarefoundation/autoware_launch/issues/1510>`_)
* feat(static_obstacle_avoidance): support separate lateral jerk constraints for avoidance and return maneuvers (`#1506 <https://github.com/autowarefoundation/autoware_launch/issues/1506>`_)
* feat(autoware_multi_object_tracker): unknown as static object (`#1481 <https://github.com/autowarefoundation/autoware_launch/issues/1481>`_)
  * feat(multi_object_tracker): add parameter to enable unknown object velocity estimation
  * feat(multi_object_tracker): enable unknown object velocity estimation by default
  * fix(multi_object_tracker): disable unknown object velocity estimation by default
  * fix(multi_object_tracker): enable unknown object velocity estimation and disable extrapolation
  Updated the multi_object_tracker_node parameters to enable unknown object velocity estimation and disable unknown object extrapolation for improved tracking behavior.
  * fix(multi_object_tracker): update parameter for unknown object motion output
  Changed the parameter from enable_unknown_object_extrapolation to enable_unknown_object_motion_output to better reflect its purpose in the multi_object_tracker_node configuration.
  ---------
* feat(intersection_collision_checker): update config (`#1505 <https://github.com/autowarefoundation/autoware_launch/issues/1505>`_)
  * update intersection collision checker parameters
  * update intersection_collision_checker params
  * add parameter on_time_buffer
  ---------
* feat(control_validator): add yaw_deviation (`#1507 <https://github.com/autowarefoundation/autoware_launch/issues/1507>`_)
* feat(path_generator): publish processing time (`#1504 <https://github.com/autowarefoundation/autoware_launch/issues/1504>`_)
* Contributors: Arjun Jagdish Ram, Kang, Kento Yabuuchi, Kok Seang Tan, Kosuke Takeuchi, Kotakku, Kyoichi Sugahara, Mamoru Sobue, Masato Saeki, Maxime CLEMENT, Mitsuhiro Sakamoto, Ryohsuke Mitsudome, Satoshi OTA, Taekjin LEE, Taiki Yamada, Takagi, Isamu, Takayuki Murooka, Yuki TAKAGI, Yukihiro Saito, Yukinari Hisaki, Yutaka Kondo, Yuxuan Liu, Zulfaqar Azmi, badai nguyen, kotaro-hihara, mkquda

0.46.0 (2025-06-20)
-------------------
* Merge remote-tracking branch 'origin/main' into tmp/bot/bump_version_base
* feat: boundary departure prevention module (`#1489 <https://github.com/autowarefoundation/autoware_launch/issues/1489>`_)
  * feat: boundary departure prevention module
  * include diagnostic settings
  * add goal dist param
  * removed unused param
  ---------
* chore(static_obstacle_avoidance): modify rviz visualization (`#1496 <https://github.com/autowarefoundation/autoware_launch/issues/1496>`_)
* feat(intersection): more conservative stop for NOT_PRIORITIZED (`#1501 <https://github.com/autowarefoundation/autoware_launch/issues/1501>`_)
* feat(planning_validator): add condition to check the yaw deviation (`#1494 <https://github.com/autowarefoundation/autoware_launch/issues/1494>`_)
* feat(autoware_behavior_velocity_traffic_light_module): add v2i  (`#1482 <https://github.com/autowarefoundation/autoware_launch/issues/1482>`_)
  * add v2i parameter
  * reflect review
  * reflect review
  ---------
* fix(roi_cluster_fusion): rename param (`#1500 <https://github.com/autowarefoundation/autoware_launch/issues/1500>`_)
* chore: clean up unused-parameters along with refactoring in motion_velocity_planner (`#1498 <https://github.com/autowarefoundation/autoware_launch/issues/1498>`_)
* feat(launch/components): launch in a component-wise manner (`#1497 <https://github.com/autowarefoundation/autoware_launch/issues/1497>`_)
  * launch component-wise
  * style(pre-commit): autofix
  * remove component_wise_launch
  * insert args to autoware.launch.xml
  ---------
  Co-authored-by: Yutaka Kondo <yutaka.kondo@youtalk.jp>
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
* feat(planning_validator): add new validation feature (`#1493 <https://github.com/autowarefoundation/autoware_launch/issues/1493>`_)
* fix(autoware_static_obstacle_avoidance): fix classification method of unstable object (`#1492 <https://github.com/autowarefoundation/autoware_launch/issues/1492>`_)
  * add param
  * Update autoware_launch/config/planning/scenario_planning/lane_driving/behavior_planning/behavior_path_planner/autoware_behavior_path_static_obstacle_avoidance_module/static_obstacle_avoidance.param.yaml
  Co-authored-by: Satoshi OTA <44889564+satoshi-ota@users.noreply.github.com>
  ---------
  Co-authored-by: Satoshi OTA <44889564+satoshi-ota@users.noreply.github.com>
* fix(crosswalk_module): rename parameters in crosswalk_module (`#1491 <https://github.com/autowarefoundation/autoware_launch/issues/1491>`_)
  * fix(crosswalk_module): rename parameter
  * fix some parameter name
* feat(detection_area): improve log message (`#1490 <https://github.com/autowarefoundation/autoware_launch/issues/1490>`_)
* feat(planning_validator): implement collision prevention feature when ego makes a turn (`#1485 <https://github.com/autowarefoundation/autoware_launch/issues/1485>`_)
  * add collision checker param file, and update planning component launch file
  * add configuration parameters
  * rename module
  * use proper name for module param
  ---------
* feat: add pointcloud container arg for simulator component (`#1479 <https://github.com/autowarefoundation/autoware_launch/issues/1479>`_)
  feat(autoware_launch): add pointcloud container arg for simulator component
* feat(planning_factor): add console output option (`#1474 <https://github.com/autowarefoundation/autoware_launch/issues/1474>`_)
* feat: add paramter for diag msg (`#1429 <https://github.com/autowarefoundation/autoware_launch/issues/1429>`_)
  * feat: add paramter for diag msg
  * chore: update mismatch threshold
  * chore: udpate variable name
  * chore: fix parameter naming
  * chore: add cropbox parameter
  ---------
* chore(perception): remove unused radar filer configs (`#1449 <https://github.com/autowarefoundation/autoware_launch/issues/1449>`_)
  * feat(autoware_launch): remove deprecated radar object clustering and noise filter parameters
  Deleted unused parameter files for radar object clustering and radar crossing objects noise filter, and updated the launch configuration accordingly.
  * feat(autoware_launch): remove unused object validation filter parameters
  Deleted parameter files for object lanelet filter and object position filter to clean up the configuration. These parameters are no longer needed in the current implementation.
  ---------
* feat(goal_planner): add param of path_decision_state_controller.check_collision_duration (`#1478 <https://github.com/autowarefoundation/autoware_launch/issues/1478>`_)
* fix(path_generator): ensure refined path connects start and goal (`#1484 <https://github.com/autowarefoundation/autoware_launch/issues/1484>`_)
  fix/path-generator-smooth-goal-connection
* feat(velocity_smoother): adjusting jerk weight (`#1475 <https://github.com/autowarefoundation/autoware_launch/issues/1475>`_)
  update JerkFiltered.param.yaml
* feat(run_out): option to preserve parts of ignored predicted paths (`#1476 <https://github.com/autowarefoundation/autoware_launch/issues/1476>`_)
* feat(perception): add parameters to use pedestrian traffic signal result estimated in perception pipeline (`#1477 <https://github.com/autowarefoundation/autoware_launch/issues/1477>`_)
  * add params
  * change param name
  ---------
* chore(multi_object_tracker): add multi-channel mot option, change deprecated channel configuration (`#1425 <https://github.com/autowarefoundation/autoware_launch/issues/1425>`_)
  feat(multi_object_tracker): clean up input_channels parameters by removing unused topics
* feat(goal_planner, start_planner): ignore_object_velocity_threshold: 0.25 (`#1471 <https://github.com/autowarefoundation/autoware_launch/issues/1471>`_)
  feat(goal_planenr,start_planner): ignore_object_velocity_threshold: 0.25
* feat(rviz): add ControlModeDisplay panel to RViz configuration (`#1473 <https://github.com/autowarefoundation/autoware_launch/issues/1473>`_)
* feat(crosswalk_module): add parked vehicles stop feature (`#1452 <https://github.com/autowarefoundation/autoware_launch/issues/1452>`_)
* feat(intersection): conservative stop in merging (`#1456 <https://github.com/autowarefoundation/autoware_launch/issues/1456>`_)
* feat(autoware_detected_object_validation): add lanelet object elevation filter option (`#1446 <https://github.com/autowarefoundation/autoware_launch/issues/1446>`_)
  * add 3d lanelet filter option
  * add 3d lanelet filter option
  * refactor
  ---------
* feat(autoware_euclidean_cluster): enhance VoxelGridBasedEuclideanCluster with Large Cluster Filtering Parameters (`#1457 <https://github.com/autowarefoundation/autoware_launch/issues/1457>`_)
  * feat(autoware_euclidean_cluster): enhance VoxelGridBasedEuclideanCluster with Large Cluster Filtering
  * fix(voxel_grid_based_euclidean_cluster): update max cluster size and output parameters
  Adjusted max_cluster_size to 3000 and renamed max_num_points_per_cluster to max_voxel_cluster_for_output with a new value of 800 for improved clarity and functionality in clustering operations.
  * fix(voxel_grid_based_euclidean_cluster): reduce min voxel cluster size for filtering from 150 to 65 to enhance clustering performance
  ---------
  Co-authored-by: badai nguyen <94814556+badai-nguyen@users.noreply.github.com>
* feat(rviz): add a script to sync rviz (`#1465 <https://github.com/autowarefoundation/autoware_launch/issues/1465>`_)
  * feat(rviz): add a script to sync rviz
  * update planing rviz
  * fix
  * fix
  * update
  * update
  ---------
* feat(rviz): add AccelerationMeter display configuration to RViz (`#1468 <https://github.com/autowarefoundation/autoware_launch/issues/1468>`_)
* feat: add logging.launch.xml of diagnostic_graph_utils (`#1466 <https://github.com/autowarefoundation/autoware_launch/issues/1466>`_)
  * feat: add logging.launch.xml
  * update rviz
  * disable by default
  ---------
* fix(ground_segmentation): extend mode switch radius (`#1464 <https://github.com/autowarefoundation/autoware_launch/issues/1464>`_)
* refactor(planning_validator): implement plugin structure for planning validator node (`#1441 <https://github.com/autowarefoundation/autoware_launch/issues/1441>`_)
  * add plugins param files and update planning component launch file
  * rename param files
  * separate trajectory checker parameters
  * add preset args for validator modules
  ---------
* feat(hazard_lights_selector): add a hazard lights selector package (`#1459 <https://github.com/autowarefoundation/autoware_launch/issues/1459>`_)
* fix(rviz): modify unused topic names (`#1461 <https://github.com/autowarefoundation/autoware_launch/issues/1461>`_)
  fix topic names
* fix(irregular_object_detector): add param for roi_pointcloud (`#1450 <https://github.com/autowarefoundation/autoware_launch/issues/1450>`_)
  * fix(irregular_object_detector): add param for roi_pointcloud
  * fix: add max_object size threshold
  ---------
* refactor(rviz): organize planning info/debug topics (`#1419 <https://github.com/autowarefoundation/autoware_launch/issues/1419>`_)
  update rviz
* feat(atuoware_behavior_path_bidirectional_traffic_module): add a functionality for bidirectional traffic (`#1423 <https://github.com/autowarefoundation/autoware_launch/issues/1423>`_)
  * add bidirectional traffic module
  * add parameter file
  * update
  * update bidirectional_traffic
  * update param
  * update
  ---------
* feat!: remove obstacle_stop_planner and obstacle_cruise_planner (`#1460 <https://github.com/autowarefoundation/autoware_launch/issues/1460>`_)
  feat: remove obstacle_stop_planner and obstacle_cruise_planner
* feat(obstacle_stop_module): maintain larger stop distance for opposing traffic (`#1437 <https://github.com/autowarefoundation/autoware_launch/issues/1437>`_)
  * launch changes for opposing traffic
  * fix
  ---------
* feat(ground_segmentation): add new parameter (`#1455 <https://github.com/autowarefoundation/autoware_launch/issues/1455>`_)
  * feat(ground_segmentation): add new parameter
  * fix: rename param
  ---------
* Contributors: Arjun Jagdish Ram, Fumiya Watanabe, Kosuke Takeuchi, Kotakku, Kyoichi Sugahara, Makoto Kurihara, Mamoru Sobue, Masaki Baba, Masato Saeki, Maxime CLEMENT, Ryohsuke Mitsudome, Satoshi OTA, Taekjin LEE, Takayuki Murooka, Yi-Hsiang Fang (Vivid), Yuki TAKAGI, Yukinari Hisaki, Yuxuan Liu, Zulfaqar Azmi, badai nguyen, github-actions, lei.gu, mkquda

0.45.3 (2025-07-17)
-------------------
* chore: back port 1527, 1528, 1529 to humble (`#1559 <https://github.com/autowarefoundation/autoware_launch/issues/1559>`_)
* feat: add launch_pointcloud_container to tier4_localization_component.launch.xml (`#1529 <https://github.com/autowarefoundation/autoware_launch/issues/1529>`_)
* feat: add global parameters to tier4_localization_component.launch.xml (`#1528 <https://github.com/autowarefoundation/autoware_launch/issues/1528>`_)
* fix(perception): add `pointcloud_container_name` parameter to give (`#1527 <https://github.com/autowarefoundation/autoware_launch/issues/1527>`_)
  add arg parameter to give
* Contributors: Masato Saeki, Ryohsuke Mitsudome

0.45.2 (2025-06-28)
-------------------
* fix:  `pointcloud_contaner` is launched on `autoware.launch.xml` by default (`#1523 <https://github.com/autowarefoundation/autoware_launch/issues/1523>`_) (`#1524 <https://github.com/autowarefoundation/autoware_launch/issues/1524>`_)
* fix:  `pointcloud_contaner` is launched on `autoware.launch.xml` by default (`#1523 <https://github.com/autowarefoundation/autoware_launch/issues/1523>`_)
* Contributors: Ryohsuke Mitsudome, Yutaka Kondo

0.45.1 (2025-06-27)
-------------------
* feat(launch/components): launch `autoware_global_parameter_loader` in a component-wise manner (`#1512 <https://github.com/autowarefoundation/autoware_launch/issues/1512>`_)
  add global parameters
* Contributors: Yutaka Kondo

0.45.0 (2025-05-22)
-------------------
* Merge remote-tracking branch 'origin/main' into tmp/bot/bump_version_base
* chore(traffic_light_recognition): add comment in config of autoware_traffic_light_map_based_detector (`#1448 <https://github.com/autowarefoundation/autoware_launch/issues/1448>`_)
  add comment
* feat(crosswalk): update stop position logic (`#1409 <https://github.com/autowarefoundation/autoware_launch/issues/1409>`_)
  * change crosswalk param list
  ---------
* feat(perception_component_launch): add common param for pointpainting_fusion (`#1401 <https://github.com/autowarefoundation/autoware_launch/issues/1401>`_)
  add common param
* fix(collision_detector): rename parameters to prevent launch issues (`#1447 <https://github.com/autowarefoundation/autoware_launch/issues/1447>`_)
  rename params
* feat(control_validator): add lateral jerk validation (`#1440 <https://github.com/autowarefoundation/autoware_launch/issues/1440>`_)
  fix(control_validator): add lateral jerk threshold to parameters
* feat(rviz): replace MRM Summary overlay with Error Diag Graph in debug rviz (`#1414 <https://github.com/autowarefoundation/autoware_launch/issues/1414>`_)
  * feat(rviz): replace MRM Summary overlay with Error Diag Graph
  * style(pre-commit): autofix
  ---------
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
* feat(collision_detector): time buffer and ignore behind rear axle (`#1443 <https://github.com/autowarefoundation/autoware_launch/issues/1443>`_)
* feat(dynamic_drivable_area_expansion): set extra offsets to 0 (`#1445 <https://github.com/autowarefoundation/autoware_launch/issues/1445>`_)
* feat(autoware_launch): remove trajectory_deviation from diagnostic graph (`#1442 <https://github.com/autowarefoundation/autoware_launch/issues/1442>`_)
* feat(autoware_launch): update adapi diags (`#1444 <https://github.com/autowarefoundation/autoware_launch/issues/1444>`_)
* fix(tier4_perception_launch): add camera ids param for segmentation_pointcloud_fusion (`#1439 <https://github.com/autowarefoundation/autoware_launch/issues/1439>`_)
  fix: add camera ids for segmentation_pointcloud_fusion
* fix(probabilistic_occupancy_grid_map): add missing parameter for multi-lidar  (`#1426 <https://github.com/autowarefoundation/autoware_launch/issues/1426>`_)
  add new parameter for multi-lidar
* feat(static_obstacle_avoidance): add flag to wait approval when the ego uses opposite lane (`#1431 <https://github.com/autowarefoundation/autoware_launch/issues/1431>`_)
* feat(behavior_velocity_planner)!: remove unused function parameter for extendLine (`#1422 <https://github.com/autowarefoundation/autoware_launch/issues/1422>`_)
  remove unused parameter
* feat(duplicated_node_checker): ignore duplicate get parameter (`#1413 <https://github.com/autowarefoundation/autoware_launch/issues/1413>`_)
* feat(autoware_bevfusion): added bevfusion config files (`#1416 <https://github.com/autowarefoundation/autoware_launch/issues/1416>`_)
  feat: added bevfusion files
* chore(simulator.launch): remove params of traffic_light_selector (`#1415 <https://github.com/autowarefoundation/autoware_launch/issues/1415>`_)
  remove param
* feat: add input_timeout in operation_mode_transition_manager (`#1407 <https://github.com/autowarefoundation/autoware_launch/issues/1407>`_)
* feat(traffic_light): remove unnecessary config file (`#1396 <https://github.com/autowarefoundation/autoware_launch/issues/1396>`_)
  remove unnecessary file
* feat(autoware_motion_velocity_planner): point-cloud clustering optimization (`#1410 <https://github.com/autowarefoundation/autoware_launch/issues/1410>`_)
  * fix
  * fix
  * fix
  ---------
* feat(mvp_run_out): update for new motion_velocity_planner run_out (`#1388 <https://github.com/autowarefoundation/autoware_launch/issues/1388>`_)
* feat(planning_validator): detect sudden trajectory shift (`#1380 <https://github.com/autowarefoundation/autoware_launch/issues/1380>`_)
  * update planning validator config
  * add new planning diagnostic path
  * visualize planning validator virtual wall by default
  * move planning validator display under planning/diagnostic namespace
  * add jerk limit parameter
  ---------
* feat(planning_validator): add lateral jerk validation parameters (`#1406 <https://github.com/autowarefoundation/autoware_launch/issues/1406>`_)
* Contributors: Arjun Jagdish Ram, Kento Yabuuchi, Kenzo Lobos Tsunekawa, Kyoichi Sugahara, Masaki Baba, Masato Saeki, Maxime CLEMENT, Mitsuhiro Sakamoto, Ryohsuke Mitsudome, Satoshi OTA, Sho Iwasawa, Takagi, Isamu, Takayuki Murooka, Yuki TAKAGI, badai nguyen, github-actions, mkquda

0.44.3 (2025-06-10)
-------------------
* feat: add pointcloud container arg for simulator component (`#1479 <https://github.com/autowarefoundation/autoware_launch/issues/1479>`_) (`#1487 <https://github.com/autowarefoundation/autoware_launch/issues/1487>`_)
* feat: add pointcloud container arg for simulator component (`#1479 <https://github.com/autowarefoundation/autoware_launch/issues/1479>`_)
  feat(autoware_launch): add pointcloud container arg for simulator component
* Contributors: Ryohsuke Mitsudome

0.44.2 (2025-05-30)
-------------------
* feat(launch/components): launch in a component-wise manner (`#1469 <https://github.com/autowarefoundation/autoware_launch/issues/1469>`_)
* insert args to autoware.launch.xml
* remove component_wise_launch
* style(pre-commit): autofix
* launch component-wise
* Contributors: Ryohsuke Mitsudome, Yutaka Kondo, pre-commit-ci[bot]

0.44.1 (2025-05-12)
-------------------
* feat(duplicated_node_checker): ignore duplicate get parameter (`#1413 <https://github.com/autowarefoundation/autoware_launch/issues/1413>`_)
* Contributors: Ryohsuke Mitsudome

0.44.0 (2025-05-01)
-------------------
* Merge commit 'bdbc8e8' into bump-up-version-to-0.44.0
* chore: bump version to 0.44.0 (`#1411 <https://github.com/autowarefoundation/autoware_launch/issues/1411>`_)
* feat(out_of_lane): improve out of lane stability (`#1404 <https://github.com/autowarefoundation/autoware_launch/issues/1404>`_)
  * update out_of_lane parameters
  * change parameter value
  * remove unused parameter
  * fix spelling
  ---------
* feat(control_validator): add over run estimation feature (`#1394 <https://github.com/autowarefoundation/autoware_launch/issues/1394>`_)
* feat(autoware_launch): add ColoredPoseWithCovarianceHistory to Localization NDT (`#1382 <https://github.com/autowarefoundation/autoware_launch/issues/1382>`_)
  * feat: update autoware.rviz
  - add Localization/NDT ColoredPoseWithCovarianceHistory
  * add: /localization/pose_estimator/nearest_voxel_transformation_likelihood
  * change color and alpha
  * fix: buffer size (15 seconds)
  * fix: typo
  * fix: add space
  ---------
* feat(traffic_light): add traffic light restart suppression parameters (`#1405 <https://github.com/autowarefoundation/autoware_launch/issues/1405>`_)
  * feat: add traffic light restart_suppression param
  * style(pre-commit): autofix
  ---------
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
* feat: remove individual_params references (`#1403 <https://github.com/autowarefoundation/autoware_launch/issues/1403>`_)
* refactor(planning_validator): separate validation check for steering and steering rate (`#1402 <https://github.com/autowarefoundation/autoware_launch/issues/1402>`_)
  feat(planning_validator): separate steering rate parameter for improved configuration
* feat(lidar_transfusion): add transfusion common param file (`#1400 <https://github.com/autowarefoundation/autoware_launch/issues/1400>`_)
  add transfusion_common param
* feat(multi_object_tracker): add detailed processing time publishing option (`#1398 <https://github.com/autowarefoundation/autoware_launch/issues/1398>`_)
  * feat(multi_object_tracker): add detailed processing time publishing option
  * fix(multi_object_tracker): disable detailed processing time publishing option
  ---------
* feat(radar_object_tracker): add diagnostics param (`#1399 <https://github.com/autowarefoundation/autoware_launch/issues/1399>`_)
  * add diagnostics param
  * fix param name
  * change callback param name
  ---------
* feat(multi_object_tracker): vehicle's ego frame as a parameter (`#1397 <https://github.com/autowarefoundation/autoware_launch/issues/1397>`_)
* revert(lane_departure_checker): "fix(lane_departure_checker): replace curbstone to road_border… (`#1395 <https://github.com/autowarefoundation/autoware_launch/issues/1395>`_)
  Revert "fix(lane_departure_checker): replace curbstone to road_border (RT1-9845) (`#1391 <https://github.com/autowarefoundation/autoware_launch/issues/1391>`_)"
  This reverts commit b26f3375d419f5e2e2621d8b36b7d6e8a0ded209.
* feat(probabilistic_occupancy_grid_map): add diagnostics warning when … (`#1387 <https://github.com/autowarefoundation/autoware_launch/issues/1387>`_)
  * feat(probabilistic_occupancy_grid_map): add diagnostics warning when latency is too long
  * style(pre-commit): autofix
  * feat(probabilistic_occupancy_grid_map):add tolerance duration
  * feat(probabilistic_occupancy_grid_map):  fix variable names to processing time
  ---------
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
* fix(lane_departure_checker): replace curbstone to road_border (RT1-9845) (`#1391 <https://github.com/autowarefoundation/autoware_launch/issues/1391>`_)
  fix(lane_departure_checker): replace curbstone to road_border
* feat(lidar_centerpoint): add common param file (`#1377 <https://github.com/autowarefoundation/autoware_launch/issues/1377>`_)
  * add common param
  * fix comment and change value to float
  ---------
* feat(autoware_launch): remove exec_depend on autoware_launch from tier4_simulator_launch (`#1392 <https://github.com/autowarefoundation/autoware_launch/issues/1392>`_)
* refactor(planning_validator): restructure planning validator configuration (`#1389 <https://github.com/autowarefoundation/autoware_launch/issues/1389>`_)
  update planning validator config
* feat: add parameter for irregular object pipeline (`#1381 <https://github.com/autowarefoundation/autoware_launch/issues/1381>`_)
  * feat: add parameter for small unknown object pipeline
  * chore: rename param
  * refactor
  * chore: spelling
  * refactor: file renaming
  * fix: param rename
  * refactor: param path update
  ---------
* feat(multi_object_tracker): add diagnostics warning when extrapolation time exceeds limit with latency guarantee enabled (`#1378 <https://github.com/autowarefoundation/autoware_launch/issues/1378>`_)
  * feat(multi_object_tracker): add diagnostics warning when extrapolation time exceeds limit with latency guarantee enabled
  * style(pre-commit): autofix
  ---------
  Co-authored-by: lei.gu <lei.gu@tier4.jp>
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
* fix(detection_area): integrate RTC feature (`#1383 <https://github.com/autowarefoundation/autoware_launch/issues/1383>`_)
* feat(start/goal_planner): update max steer angle parameters to use margin scale (`#1368 <https://github.com/autowarefoundation/autoware_launch/issues/1368>`_)
* feat: add parameter for diagnostics (`#1362 <https://github.com/autowarefoundation/autoware_launch/issues/1362>`_)
* feat(perception): add parameter for diag (`#1357 <https://github.com/autowarefoundation/autoware_launch/issues/1357>`_)
  * add parameter for diag
  * change param name
  * add unit
  ---------
* fix(roi_pointcloud_fusion): add roi scale factor param (`#1376 <https://github.com/autowarefoundation/autoware_launch/issues/1376>`_)
* feat(control_validator)!: add acceleration check (`#1375 <https://github.com/autowarefoundation/autoware_launch/issues/1375>`_)
* feat(crosswalk_module): add param to consider objects on crosswalk when pedestrian traffic light is red (`#1374 <https://github.com/autowarefoundation/autoware_launch/issues/1374>`_)
* feat(crosswalk): fix stop position calclaton params (`#1370 <https://github.com/autowarefoundation/autoware_launch/issues/1370>`_)
* feat(multi_object_tracker): add input channel flags for selective update per channel (`#1364 <https://github.com/autowarefoundation/autoware_launch/issues/1364>`_)
  feat(multi_object_tracker): update input channel flags for improved tracking parameters
* feat(goal_planner): expand outer collision check margin (`#1365 <https://github.com/autowarefoundation/autoware_launch/issues/1365>`_)
* Contributors: Amadeusz Szymko, Kazu, Kosuke Takeuchi, Kotaro Uetake, Kyoichi Sugahara, Masaki Baba, Masato Saeki, Mehmet Dogru, Mete Fatih Cırıt, Ryohsuke Mitsudome, Satoshi OTA, Taekjin LEE, Takagi, Isamu, Yuki TAKAGI, Yutaka Kondo, Zulfaqar Azmi, badai nguyen, eiki, lei.gu, mkquda

0.43.1 (2025-04-01)
-------------------

0.43.0 (2025-03-21)
-------------------
* Merge remote-tracking branch 'origin/main' into tmp/bot/bump_version_base
* feat(autoware_launch): rename to max_lat_margin_against_predicted_object_unknown (`#1360 <https://github.com/autowarefoundation/autoware_launch/issues/1360>`_)
* feat(pid_long): disable overshoot_emergency (`#1359 <https://github.com/autowarefoundation/autoware_launch/issues/1359>`_)
* feat(out_of_lane): add 'use_map_stop_lines' parameter (`#1259 <https://github.com/autowarefoundation/autoware_launch/issues/1259>`_)
* feat(planning_validator): add parameter for yaw deviation metric (`#1356 <https://github.com/autowarefoundation/autoware_launch/issues/1356>`_)
* feat(behavior_path_planner_common): update drivable area expansion parameters (`#1353 <https://github.com/autowarefoundation/autoware_launch/issues/1353>`_)
  * update drivable area expansion parameters
  * Update autoware_launch/config/planning/scenario_planning/lane_driving/behavior_planning/behavior_path_planner/drivable_area_expansion.param.yaml
  Co-authored-by: Maxime CLEMENT <78338830+maxime-clem@users.noreply.github.com>
  * Update autoware_launch/config/planning/scenario_planning/lane_driving/behavior_planning/behavior_path_planner/drivable_area_expansion.param.yaml
  Co-authored-by: Satoshi OTA <44889564+satoshi-ota@users.noreply.github.com>
  ---------
  Co-authored-by: Maxime CLEMENT <78338830+maxime-clem@users.noreply.github.com>
  Co-authored-by: Satoshi OTA <44889564+satoshi-ota@users.noreply.github.com>
* feat: disable merge_from_private (`#1350 <https://github.com/autowarefoundation/autoware_launch/issues/1350>`_)
* feat(control_validator)!: add overrun_stop_point_dist parameter (`#1346 <https://github.com/autowarefoundation/autoware_launch/issues/1346>`_)
* refactor(perception): refactor launch file and add parameter file (`#1336 <https://github.com/autowarefoundation/autoware_launch/issues/1336>`_)
  * fundamental change
  * style(pre-commit): autofix
  * change params name
  * remove param
  * integrate model and label path
  * for awsim
  * add comment
  * fix typo
  * change param name
  * chore
  ---------
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
  Co-authored-by: Kenzo Lobos Tsunekawa <kenzo.lobos@tier4.jp>
* feat(path_generator): add parameters (`#1343 <https://github.com/autowarefoundation/autoware_launch/issues/1343>`_)
  * feat(path_generator): add parameters (see below)
  * This fix is for the following PR:
  https://github.com/autowarefoundation/autoware.core/pull/227
  * Update autoware_launch/config/planning/scenario_planning/lane_driving/behavior_planning/path_generator/path_generator.param.yaml
  Co-authored-by: Kosuke Takeuchi <kosuke.tnp@gmail.com>
  ---------
  Co-authored-by: Kosuke Takeuchi <kosuke.tnp@gmail.com>
* fix(obstacle_cruise_planner): ignore invalid stopping objects (`#1354 <https://github.com/autowarefoundation/autoware_launch/issues/1354>`_)
  * add parameter
  * rename ahead_stopped -> side_stopped
  ---------
* feat(planning_simulator): disable tlr in psim (`#1352 <https://github.com/autowarefoundation/autoware_launch/issues/1352>`_)
  disable tlr in psim
* feat(planning_validator): add diag to check planning component latency (`#1347 <https://github.com/autowarefoundation/autoware_launch/issues/1347>`_)
* feat(control_validator): add diag to check control component latency (`#1349 <https://github.com/autowarefoundation/autoware_launch/issues/1349>`_)
* fix(planning): param update for sudden stop (`#1345 <https://github.com/autowarefoundation/autoware_launch/issues/1345>`_)
  fix for x1
* feat(dummy_infrastructur): auto approval when ego stops at stop line (`#1344 <https://github.com/autowarefoundation/autoware_launch/issues/1344>`_)
* feat(image_based_projection_fusion): update parameters for new image based projection fusion node (`#1339 <https://github.com/autowarefoundation/autoware_launch/issues/1339>`_)
  * feat: new fusion parameters
  * chore: remove some comment
  ---------
* feat(behavior_planning): add behavior_path_planner_type to launch path_generator (`#1342 <https://github.com/autowarefoundation/autoware_launch/issues/1342>`_)
* Contributors: Arjun Jagdish Ram, Junya Sasaki, Kento Yabuuchi, Kosuke Takeuchi, Masato Saeki, Maxime CLEMENT, Satoshi OTA, Takayuki Murooka, Yi-Hsiang Fang (Vivid), Yuki TAKAGI, github-actions, mkquda

0.42.0 (2025-03-03)
-------------------
* Merge remote-tracking branch 'origin/main' into tmp/bot/bump_version_base
* chore(ekf_localizer): increase z_filter_proc_dev for large gradient road (`#1337 <https://github.com/autowarefoundation/autoware_launch/issues/1337>`_)
  increase z_filter_proc_dev
* feat(autoware_motion_velocity_obstacle_slow_down_module): params for obstacle stop and slow down modules (`#1330 <https://github.com/autowarefoundation/autoware_launch/issues/1330>`_)
  * fix
  * style(pre-commit): autofix
  * fix
  ---------
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
* feat(goal_planner): align vehicle center to be parallel to lane boundary (`#1335 <https://github.com/autowarefoundation/autoware_launch/issues/1335>`_)
* chore(autoware_map_based_prediction): delete unused function and parameter (`#1326 <https://github.com/autowarefoundation/autoware_launch/issues/1326>`_)
* chore(traffic_light): rename enable_fine_detection (`#1310 <https://github.com/autowarefoundation/autoware_launch/issues/1310>`_)
  * chore: rename enable_fine_detection
  * feat: add new tlr param
  * change back to fine_detector
  * fix: typo
  * add args
  * style(pre-commit): autofix
  * change default param to fine detector
  * style(pre-commit): autofix
  ---------
  Co-authored-by: Masato Saeki <78376491+MasatoSaeki@users.noreply.github.com>
  Co-authored-by: MasatoSaeki <masato.saeki@tier4.jp>
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
* chore(traffic_light_multi_camera_fusion): read parameters from yaml file (`#1331 <https://github.com/autowarefoundation/autoware_launch/issues/1331>`_)
  * chore(traffic_light_multi_camera_fusion): read parameters from yaml file
  * style(pre-commit): autofix
  * remove camera namespace parameter from config file
  ---------
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
* feat(tier4_perception_component): change tlr pedestrian classifier model (`#1329 <https://github.com/autowarefoundation/autoware_launch/issues/1329>`_)
  change model name
* fix(ground_segmentation): bring junction parameter from param file to launch argument (`#1327 <https://github.com/autowarefoundation/autoware_launch/issues/1327>`_)
  * refactor(ground_segmentation): remove single frame filter and keep time series filter disabled
  * feat(tier4_perception): add single frame and time series filters for obstacle segmentation
  ---------
* feat(autoware_probabilistic_occupancy_grid_map): disabled the subsample filters for the ogm (`#1319 <https://github.com/autowarefoundation/autoware_launch/issues/1319>`_)
  feat: disabled the subsample filters for the ogm since its creation is faster now
* feat: use motion_velocity_obstacle\_<stop/slow_down/cruise>_module (`#1315 <https://github.com/autowarefoundation/autoware_launch/issues/1315>`_)
  Revert "enable obstacle_cruise_planner"
  This reverts commit cbd6873e7786bf139796b20a30ada5d90bd8407b.
* feat(autoware_behavior_velocity_traffic_light_module): adjust velocity threshold for ensure stop at yellow light (`#1322 <https://github.com/autowarefoundation/autoware_launch/issues/1322>`_)
* refactor(goal_planner): remove use_object_recognition because it is default (`#1318 <https://github.com/autowarefoundation/autoware_launch/issues/1318>`_)
* feat(rviz): update autoware.rviz for motion_velocity_obstacle\_<stop/slow_down/cruise>_module (`#1314 <https://github.com/autowarefoundation/autoware_launch/issues/1314>`_)
  * feat: add motion_velocity_obstacle_stop/slow_down/cruise_module
  * update autoware.rviz
  * update rviz
  * disable obstacle_cruise_planner
  * update motion velocity planner params
  * add module.param.yaml
  * enable obstacle_cruise_planner
  ---------
* Contributors: Arjun Jagdish Ram, Kento Yabuuchi, Kenzo Lobos Tsunekawa, Mamoru Sobue, Masato Saeki, Taekjin LEE, Takayuki Murooka, Tomohito ANDO, Tomoya Kimura, badai nguyen, github-actions

0.41.0 (2025-01-29)
-------------------
* Merge remote-tracking branch 'origin/main' into tmp/bot/bump_version_base
* feat(goal_planner): introduce bezier based pullover for bus stop area (`#1308 <https://github.com/autowarefoundation/autoware_launch/issues/1308>`_)
* feat: apply autoware prefix for state monitor (`#1313 <https://github.com/autowarefoundation/autoware_launch/issues/1313>`_)
  Co-authored-by: Takagi, Isamu <isamu.takagi@tier4.jp>
  Co-authored-by: Takagi, Isamu <43976882+isamu-takagi@users.noreply.github.com>
* revert: revert "feat: apply autoware prefix for state monitor" (`#1312 <https://github.com/autowarefoundation/autoware_launch/issues/1312>`_)
  Revert "feat: apply autoware prefix for state monitor (`#1311 <https://github.com/autowarefoundation/autoware_launch/issues/1311>`_)"
* feat: apply autoware prefix for state monitor (`#1311 <https://github.com/autowarefoundation/autoware_launch/issues/1311>`_)
  Co-authored-by: Ryohsuke Mitsudome <43976834+mitsudome-r@users.noreply.github.com>
* feat: apply autoware prefix for adapi helpers (`#1309 <https://github.com/autowarefoundation/autoware_launch/issues/1309>`_)
* feat(autoware_detected_object_validation): add height filter in lanelet filtering (`#1307 <https://github.com/autowarefoundation/autoware_launch/issues/1307>`_)
  * chore(package.xml): bump version to 0.38.0 (`#1226 <https://github.com/autowarefoundation/autoware_launch/issues/1226>`_)
  * add changelog
  * unify package.xml version to 0.37.0
  * 0.38.0
  * fix organization
  ---------
  * youtalk username
  * update changelog
  * 0.39.0
  * Update autoware_launch/CHANGELOG.rst
  Co-authored-by: Ryohsuke Mitsudome <43976834+mitsudome-r@users.noreply.github.com>
  * change username
  * chore(package.xml): bump version to 0.39.0 (`#1248 <https://github.com/autowarefoundation/autoware_launch/issues/1248>`_)
  Co-authored-by: Yuki TAKAGI <141538661+yuki-takagi-66@users.noreply.github.com>
  Co-authored-by: Go Sakayori <go-sakayori@users.noreply.github.com>
  Co-authored-by: Kosuke Takeuchi <kosuke.tnp@gmail.com>
  Co-authored-by: Taekjin LEE <technolojin@gmail.com>
  Co-authored-by: SakodaShintaro <rgbygscrsedppbwg@gmail.com>
  Co-authored-by: Zulfaqar Azmi <93502286+zulfaqar-azmi-t4@users.noreply.github.com>
  Co-authored-by: Satoshi OTA <44889564+satoshi-ota@users.noreply.github.com>
  Co-authored-by: iwatake <take.iwiw2222@gmail.com>
  Co-authored-by: ito-san <57388357+ito-san@users.noreply.github.com>
  Co-authored-by: Takayuki Murooka <takayuki5168@gmail.com>
  Co-authored-by: danielsanchezaran <daniel.sanchez@tier4.jp>
  Co-authored-by: Maxime CLEMENT <78338830+maxime-clem@users.noreply.github.com>
  Co-authored-by: Ryohsuke Mitsudome <43976834+mitsudome-r@users.noreply.github.com>
  Co-authored-by: xtk8532704 <1041084556@qq.com>
  * feat: add height filter option for lanelet filter
  * chore: add description in parameter
  ---------
  Co-authored-by: Yutaka Kondo <yutaka.kondo@youtalk.jp>
  Co-authored-by: Ryohsuke Mitsudome <43976834+mitsudome-r@users.noreply.github.com>
  Co-authored-by: Yuki TAKAGI <141538661+yuki-takagi-66@users.noreply.github.com>
  Co-authored-by: Go Sakayori <go-sakayori@users.noreply.github.com>
  Co-authored-by: Kosuke Takeuchi <kosuke.tnp@gmail.com>
  Co-authored-by: Taekjin LEE <technolojin@gmail.com>
  Co-authored-by: SakodaShintaro <rgbygscrsedppbwg@gmail.com>
  Co-authored-by: Zulfaqar Azmi <93502286+zulfaqar-azmi-t4@users.noreply.github.com>
  Co-authored-by: Satoshi OTA <44889564+satoshi-ota@users.noreply.github.com>
  Co-authored-by: iwatake <take.iwiw2222@gmail.com>
  Co-authored-by: ito-san <57388357+ito-san@users.noreply.github.com>
  Co-authored-by: Takayuki Murooka <takayuki5168@gmail.com>
  Co-authored-by: danielsanchezaran <daniel.sanchez@tier4.jp>
  Co-authored-by: Maxime CLEMENT <78338830+maxime-clem@users.noreply.github.com>
  Co-authored-by: xtk8532704 <1041084556@qq.com>
  Co-authored-by: Junya Sasaki <j2sasaki1990@gmail.com>
  Co-authored-by: Ryohsuke Mitsudome <ryohsuke.mitsudome@tier4.jp>
* chore(system_monitor): add parameters for UDP buf errors (`#1303 <https://github.com/autowarefoundation/autoware_launch/issues/1303>`_)
* refactor(lane_change): add missing safety check parameter  (`#1300 <https://github.com/autowarefoundation/autoware_launch/issues/1300>`_)
  * refactor(lane_change): parameterize incoming object angle for filter
  * add missing param
  ---------
* fix: remove unnecesary parameters (`#1301 <https://github.com/autowarefoundation/autoware_launch/issues/1301>`_)
* feat(lane_change): add time limit param (`#1298 <https://github.com/autowarefoundation/autoware_launch/issues/1298>`_)
  add time limit param
* chore(autoware_test_utils): add test_utils rviz config (`#1299 <https://github.com/autowarefoundation/autoware_launch/issues/1299>`_)
* chore: fix typo in drivable_are_expansion.param.yaml (`#1297 <https://github.com/autowarefoundation/autoware_launch/issues/1297>`_)
  fix typo in drivable_are_expansion.param.yaml
* feat(lane_change): using frenet planner to generate lane change path when ego near terminal (`#1290 <https://github.com/autowarefoundation/autoware_launch/issues/1290>`_)
  * add parameter for enabling frenet
  * parameterized th_yaw_diff
  * prepare segment curvature threshold
  * add curvature smoothing
  ---------
* feat(lane_change): add lane change parameter (`#1263 <https://github.com/autowarefoundation/autoware_launch/issues/1263>`_)
  * add flag to enable/disable terminal path feature
  * add parameter to cofigure stop point placement
  * add flag to disable feature near goal
  * set default flag value to false
  ---------
* feat: remove enable_rtc from detection_area (`#1292 <https://github.com/autowarefoundation/autoware_launch/issues/1292>`_)
* feat(lane_change): add feature flag param (`#1291 <https://github.com/autowarefoundation/autoware_launch/issues/1291>`_)
  add parameter to enable/disable keeping distance from front stopped object
* feat(autoware_traffic_light_arbiter): add current time validation (`#1289 <https://github.com/autowarefoundation/autoware_launch/issues/1289>`_)
  * add config
  * change ros parameter name
  ---------
* feat(image_projection_based_fusion): add cache options (`#1275 <https://github.com/autowarefoundation/autoware_launch/issues/1275>`_)
  * add timekeeper option
  * add cache option and mod unrectified_image option
  * fix parameter names
  ---------
* feat(pid_longitudinal_controller): change slope compensation mode to trajectory_goal_adaptive (`#1288 <https://github.com/autowarefoundation/autoware_launch/issues/1288>`_)
* feat: remove admissible\_{position/yaw}_error from trajectory_follower (`#1284 <https://github.com/autowarefoundation/autoware_launch/issues/1284>`_)
* feat: add velocity control virtual wall (`#1285 <https://github.com/autowarefoundation/autoware_launch/issues/1285>`_)
* feat: remove emergency_state_traj\_{trans/rot}_dev from trajectory_follower (`#1283 <https://github.com/autowarefoundation/autoware_launch/issues/1283>`_)
  * feat: remove emergency_state_traj\_{trans/rot}_dev from trajectory_follower
  * update codeowner
  ---------
* feat(detected_object_validation): add validation for maximum distance in obstacle_pointcloud_based_validator (`#1277 <https://github.com/autowarefoundation/autoware_launch/issues/1277>`_)
  feat: add validation for maximum distance in obstacle_pointcloud_based_validator
* chore: sync files (`#1280 <https://github.com/autowarefoundation/autoware_launch/issues/1280>`_)
  * chore: sync files
  * style(pre-commit): autofix
  ---------
  Co-authored-by: github-actions <github-actions@github.com>
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
* feat(pid_longitudinal_controller): change default slope compesation source to trajectory_adaptive (`#1276 <https://github.com/autowarefoundation/autoware_launch/issues/1276>`_)
* feat(image_projection_based_fusion): add timekeeper option (`#1274 <https://github.com/autowarefoundation/autoware_launch/issues/1274>`_)
  add timekeeper option
* refactor(autoware_multi_object_tracker): extract tracker parameters (`#1273 <https://github.com/autowarefoundation/autoware_launch/issues/1273>`_)
* feat(MRM_handler, MRM_emergency_stop_operator): revert mrm_stop parameter, enable mrm_comfortable_stop (`#1265 <https://github.com/autowarefoundation/autoware_launch/issues/1265>`_)
* Contributors: Autumn60, Mamoru Sobue, Masaki Baba, Masato Saeki, Ryohsuke Mitsudome, Taekjin LEE, Takagi, Isamu, Takayuki Murooka, Yoshi Ri, Yuki TAKAGI, Zulfaqar Azmi, awf-autoware-bot[bot], github-actions, iwatake, jakor97, mkquda

0.40.0 (2024-12-12)
-------------------
* Merge remote-tracking branch 'origin/main' into release-0.40.0
* refactor(obstacle_cruise_planner)!: refactor rviz and terminal info (`#1264 <https://github.com/autowarefoundation/autoware_launch/issues/1264>`_)
* fix(pointcloud_container.launch.py): autoware_glog_component (`#1260 <https://github.com/autowarefoundation/autoware_launch/issues/1260>`_)
  Fixed autoware_glog_component
* chore(package.xml): bump version to 0.39.0 (`#1248 <https://github.com/autowarefoundation/autoware_launch/issues/1248>`_) (`#1261 <https://github.com/autowarefoundation/autoware_launch/issues/1261>`_)
  Co-authored-by: Yutaka Kondo <yutaka.kondo@youtalk.jp>
* feat(lane_change): add new lane change parameter (`#1224 <https://github.com/autowarefoundation/autoware_launch/issues/1224>`_)
  * add minimum prepare duration parameter
  * increase min_prepare_duration to 1.0 s
  * increase min prepare duration value, add new parameter
  ---------
* refactor(global_parameter_loader): prefix package and namespace with autoware (`#1246 <https://github.com/autowarefoundation/autoware_launch/issues/1246>`_)
* refactor(glog_component): prefix package and namespace with autoware (`#1245 <https://github.com/autowarefoundation/autoware_launch/issues/1245>`_)
* fix(rviz): fix a bug about visualizing ego model (`#1257 <https://github.com/autowarefoundation/autoware_launch/issues/1257>`_)
  fix a visulization bug.
* feat(object_lanelet_filter): add configurable margin for object lanel… (`#1210 <https://github.com/autowarefoundation/autoware_launch/issues/1210>`_)
  feat(object_lanelet_filter): add configurable margin for object lanelet filter
  Co-authored-by: Sebastian Zęderowski <szederowski@autonomous-systems.pl>
* refactor(system_diagnostic_monitor, dummy_diag_publisher, diagnostic_graph_aggregator): combine diag list setting directories (`#1253 <https://github.com/autowarefoundation/autoware_launch/issues/1253>`_)
* feat(autonomous_emergency_braking): add parameter to limit IMU path length and rename longitudinal offset (`#1251 <https://github.com/autowarefoundation/autoware_launch/issues/1251>`_)
* feat(lane_change): add delay lane change parameters (`#1256 <https://github.com/autowarefoundation/autoware_launch/issues/1256>`_)
  add delay lane change parameters
* refactor(autoware_behavior_velocity_planner_common,autoware_behavior_velocity_planner): separate param files (`#1254 <https://github.com/autowarefoundation/autoware_launch/issues/1254>`_)
* fix(dynamic_obstacle_avoidance): improve avoidance for moving NPCs (`#1170 <https://github.com/autowarefoundation/autoware_launch/issues/1170>`_)
* fix(static_obstacle_avoidance): improve avoidance for parked NPCs (`#1129 <https://github.com/autowarefoundation/autoware_launch/issues/1129>`_)
* refactor(lane_change): refactor lane change parameters (`#1247 <https://github.com/autowarefoundation/autoware_launch/issues/1247>`_)
  refactor lane change params
* change username
* feat(scan_ground_filter): update grid size for ground segmentation (`#1223 <https://github.com/autowarefoundation/autoware_launch/issues/1223>`_)
  feat: update grid size for ground segmentation
  The grid size for ground segmentation has been updated from 0.1 to 0.5. This change improves the performance with the new grid data structure.
* Update autoware_launch/CHANGELOG.rst
  Co-authored-by: Ryohsuke Mitsudome <43976834+mitsudome-r@users.noreply.github.com>
* feat(autonomous_emergency_braking) add params for limiting imu path with lat deviation (`#1244 <https://github.com/autowarefoundation/autoware_launch/issues/1244>`_)
  add params
* 0.39.0
* update changelog
* youtalk username
* Merge commit '9d0e7055a' into release-0.39.0
* feat(processing_time_checker): update processing time list (`#1236 <https://github.com/autowarefoundation/autoware_launch/issues/1236>`_)
* fix: default value for control_module_preset (`#1243 <https://github.com/autowarefoundation/autoware_launch/issues/1243>`_)
* fix: default value for control_module_preset (`#1242 <https://github.com/autowarefoundation/autoware_launch/issues/1242>`_)
* feat: add an option of odometry uncertainty consideration in multi_object_tracker_node (`#1196 <https://github.com/autowarefoundation/autoware_launch/issues/1196>`_)
  feat: add an option of odometry uncertainty consideration in multi_object_tracker_node.param.yaml
* feat(control): use preset.yaml to control which modules to launch for control modules (`#1237 <https://github.com/autowarefoundation/autoware_launch/issues/1237>`_)
  * add control_module_preset
  * fix typo
  ---------
* chore(system_diagnostic_monitor): sort paths (`#1230 <https://github.com/autowarefoundation/autoware_launch/issues/1230>`_)
* feat(freespace_planner): lower safety distance margin from 0.5 to 0.4m (`#1234 <https://github.com/autowarefoundation/autoware_launch/issues/1234>`_)
* feat(rviz): show velocity/steering factors (`#1235 <https://github.com/autowarefoundation/autoware_launch/issues/1235>`_)
* chore(crosswalk)!: delete wide crosswalk corresponding function (`#1233 <https://github.com/autowarefoundation/autoware_launch/issues/1233>`_)
* feat(goal_planner): loosen safety check to prevent unnecessary stop (`#1231 <https://github.com/autowarefoundation/autoware_launch/issues/1231>`_)
* feat(crosswalk): disable slowdowns when the crosswalk is occluded (`#1232 <https://github.com/autowarefoundation/autoware_launch/issues/1232>`_)
* chore(package.xml): bump version to 0.38.0 (`#1226 <https://github.com/autowarefoundation/autoware_launch/issues/1226>`_) (`#1229 <https://github.com/autowarefoundation/autoware_launch/issues/1229>`_)
  * add changelog
  * unify package.xml version to 0.37.0
  * 0.38.0
  * fix organization
  ---------
* feat(psim, dummy_diag, diagnostic_graph_aggregator)!: launch dummy_diag_publisher (`#1220 <https://github.com/autowarefoundation/autoware_launch/issues/1220>`_)
* feat: change the old diagnostic_convertor to scenario_simulator_v2_adapter (`#1227 <https://github.com/autowarefoundation/autoware_launch/issues/1227>`_)
  Co-authored-by: xtk8532704 <1041084556@qq.com>
* feat(costmap_generator): change lidar height thresholds to vehicle frame (`#1225 <https://github.com/autowarefoundation/autoware_launch/issues/1225>`_)
* revert(obstacle_cruise): disable ouside stop feature (`#1222 <https://github.com/autowarefoundation/autoware_launch/issues/1222>`_)
* feat(aeb): set global param to override autoware state check (`#1218 <https://github.com/autowarefoundation/autoware_launch/issues/1218>`_)
  * set global param to override autoware state check
  * change variable for a more generic name
  * set var to false by default
  * move param to control component launch
  * change param name to be more straightforward
  ---------
* fix(pid_longitudinal_controller): revive hysteresis of state transition (`#1219 <https://github.com/autowarefoundation/autoware_launch/issues/1219>`_)
* feat(detection_area)!: add retruction feature (`#1216 <https://github.com/autowarefoundation/autoware_launch/issues/1216>`_)
* feat(system_monitor): add on/off config for network traffic monitor (`#1186 <https://github.com/autowarefoundation/autoware_launch/issues/1186>`_)
  feat(system_monitor): add config for network traffic monitor
  Co-authored-by: ito-san <57388357+ito-san@users.noreply.github.com>
* feat(goal_planner): set lane departure check margin 0.20 (`#1214 <https://github.com/autowarefoundation/autoware_launch/issues/1214>`_)
* fix(autoware_ekf_localizer): removed `publish_tf` (`#1212 <https://github.com/autowarefoundation/autoware_launch/issues/1212>`_)
  Removed `publish_tf`
* feat(rviz): add rviz config for debugging (`#1213 <https://github.com/autowarefoundation/autoware_launch/issues/1213>`_)
  * feat(rviz): add rviz config for debugging
  * feat(launch): select rviz config name
  ---------
* feat(lane_change): enable cancel when ego in turn direction lane main (RT0-33893) (`#1209 <https://github.com/autowarefoundation/autoware_launch/issues/1209>`_)
  RT0-33893 add dist from prev intersection
* fix: changed `loc_config_path` declaration from let to arg (`#1204 <https://github.com/autowarefoundation/autoware_launch/issues/1204>`_)
  Changed loc_config_path declaration from let to arg
* chore: update fusion_common.param.yaml with new image projection sett… (`#1207 <https://github.com/autowarefoundation/autoware_launch/issues/1207>`_)
  chore: update fusion_common.param.yaml with new image projection settings
* feat(goal_planner): set lane departure check margin 0.3 (`#1199 <https://github.com/autowarefoundation/autoware_launch/issues/1199>`_)
* feat(collision detector): add collision detector to launch/config (`#1205 <https://github.com/autowarefoundation/autoware_launch/issues/1205>`_)
  * add collision_detector
  * change collision detector default to false
  ---------
* chore(diagnostic_graph_aggregator, system_diagnostic_monitor)!: change the config file directories from universe to autoware_launch (`#1201 <https://github.com/autowarefoundation/autoware_launch/issues/1201>`_)
  * prepare dir
  * copy files from universe
* Contributors: Ahmed Ebrahim, Esteve Fernandez, Fumiya Watanabe, Go Sakayori, Kazunori-Nakajima, Kem (TiankuiXian), Kosuke Takeuchi, Kyoichi Sugahara, Maxime CLEMENT, Ryohsuke Mitsudome, SakodaShintaro, Satoshi OTA, Sebastian Zęderowski, Taekjin LEE, Takayuki Murooka, Yuki TAKAGI, Yukinari Hisaki, Yutaka Kondo, Zulfaqar Azmi, beyzanurkaya, danielsanchezaran, iwatake, mkquda

0.39.0 (2024-11-25)
-------------------
* autowarefoundation username
* Merge commit '9d0e7055a' into release-0.39.0
* feat: change the old diagnostic_convertor to scenario_simulator_v2_adapter (`#1227 <https://github.com/autowarefoundation/autoware_launch/issues/1227>`_)
  Co-authored-by: xtk8532704 <1041084556@qq.com>
* feat(costmap_generator): change lidar height thresholds to vehicle frame (`#1225 <https://github.com/autowarefoundation/autoware_launch/issues/1225>`_)
* revert(obstacle_cruise): disable ouside stop feature (`#1222 <https://github.com/autowarefoundation/autoware_launch/issues/1222>`_)
* feat(aeb): set global param to override autoware state check (`#1218 <https://github.com/autowarefoundation/autoware_launch/issues/1218>`_)
  * set global param to override autoware state check
  * change variable for a more generic name
  * set var to false by default
  * move param to control component launch
  * change param name to be more straightforward
  ---------
* fix(pid_longitudinal_controller): revive hysteresis of state transition (`#1219 <https://github.com/autowarefoundation/autoware_launch/issues/1219>`_)
* feat(detection_area)!: add retruction feature (`#1216 <https://github.com/autowarefoundation/autoware_launch/issues/1216>`_)
* feat(system_monitor): add on/off config for network traffic monitor (`#1186 <https://github.com/autowarefoundation/autoware_launch/issues/1186>`_)
  feat(system_monitor): add config for network traffic monitor
  Co-authored-by: ito-san <57388357+ito-san@users.noreply.github.com>
* feat(goal_planner): set lane departure check margin 0.20 (`#1214 <https://github.com/autowarefoundation/autoware_launch/issues/1214>`_)
* fix(autoware_ekf_localizer): removed `publish_tf` (`#1212 <https://github.com/autowarefoundation/autoware_launch/issues/1212>`_)
  Removed `publish_tf`
* feat(rviz): add rviz config for debugging (`#1213 <https://github.com/autowarefoundation/autoware_launch/issues/1213>`_)
  * feat(rviz): add rviz config for debugging
  * feat(launch): select rviz config name
  ---------
* feat(lane_change): enable cancel when ego in turn direction lane main (RT0-33893) (`#1209 <https://github.com/autowarefoundation/autoware_launch/issues/1209>`_)
  RT0-33893 add dist from prev intersection
* fix: changed `loc_config_path` declaration from let to arg (`#1204 <https://github.com/autowarefoundation/autoware_launch/issues/1204>`_)
  Changed loc_config_path declaration from let to arg
* chore: update fusion_common.param.yaml with new image projection sett… (`#1207 <https://github.com/autowarefoundation/autoware_launch/issues/1207>`_)
  chore: update fusion_common.param.yaml with new image projection settings
* feat(goal_planner): set lane departure check margin 0.3 (`#1199 <https://github.com/autowarefoundation/autoware_launch/issues/1199>`_)
* feat(collision detector): add collision detector to launch/config (`#1205 <https://github.com/autowarefoundation/autoware_launch/issues/1205>`_)
  * add collision_detector
  * change collision detector default to false
  ---------
* chore(diagnostic_graph_aggregator, system_diagnostic_monitor)!: change the config file directories from universe to autoware_launch (`#1201 <https://github.com/autowarefoundation/autoware_launch/issues/1201>`_)
  * prepare dir
  * copy files from universe
* Contributors: Go Sakayori, Kosuke Takeuchi, Maxime CLEMENT, Ryohsuke Mitsudome, SakodaShintaro, Satoshi OTA, Taekjin LEE, Takayuki Murooka, Yuki TAKAGI, Yutaka Kondo, Zulfaqar Azmi, danielsanchezaran, iwatake

0.38.0 (2024-11-13)
-------------------
* feat(start/goal_planner): increse max dry steering angle (`#1200 <https://github.com/autowarefoundation/autoware_launch/issues/1200>`_)
* fix(start_planner): set  ignore_distance_from_lane_end param to 0.0 since it is not needed (`#1198 <https://github.com/autowarefoundation/autoware_launch/issues/1198>`_)
  set param to 0.0 since it is not needed
* chore(tier4_perception_launch): enable to use argument `centerpoint_model_name` (`#1182 <https://github.com/autowarefoundation/autoware_launch/issues/1182>`_)
  * add arguments
  * adopt transfusion
  * add lidar_detection_model_type
  * integrate all in lidar_detection_model
  * adopt universe
  * fix typo
  * change description
  * change description
  * for pre-commit
  ---------
* feat(processing_time_checker): add five module. (`#1192 <https://github.com/autowarefoundation/autoware_launch/issues/1192>`_)
* feat(autonomous_emergency_braking): change params to cater to urban scenario (`#1197 <https://github.com/autowarefoundation/autoware_launch/issues/1197>`_)
  update scenarios
* feat(control_validator): add hold and lpf (`#1193 <https://github.com/autowarefoundation/autoware_launch/issues/1193>`_)
* chore(simple_planning_simulator): add stop_filter_param_path (`#1195 <https://github.com/autowarefoundation/autoware_launch/issues/1195>`_)
* feat(crosswalk_module): set the velocity of occluded objects to 2.0m/s (`#1194 <https://github.com/autowarefoundation/autoware_launch/issues/1194>`_)
* fix(pointcloud_map_filter): add threshold for split map grid size  (`#1184 <https://github.com/autowarefoundation/autoware_launch/issues/1184>`_)
  * fix(pointcloud_map_filter): add param
  * fix: disable dynamic map loader for default unsplit-map
  ---------
* refactor(rviz): add VirtualWall display for Autonomous Emergency Braking (`#1187 <https://github.com/autowarefoundation/autoware_launch/issues/1187>`_)
  feat(rviz): add VirtualWall display for Autonomous Emergency Braking
* revert(obstacle_cruisse): revert "fix(obstacle_cruise_planner): guarantee the stop margin (`#1076 <https://github.com/autowarefoundation/autoware_launch/issues/1076>`_)" (`#1185 <https://github.com/autowarefoundation/autoware_launch/issues/1185>`_)
* feat(obstacle_cruise_planner): improve stop and cruise behavior for cut-in & out (`#1142 <https://github.com/autowarefoundation/autoware_launch/issues/1142>`_)
* chore(crop_box_filter): add missing default parameter (`#1155 <https://github.com/autowarefoundation/autoware_launch/issues/1155>`_)
  fix: add missing parameter after crop_box_filter rework
* feat(autonomous_emergency_braking): set max imu path length (`#1183 <https://github.com/autowarefoundation/autoware_launch/issues/1183>`_)
  * set param for max imu path distance
  * change param
  ---------
* fix(obstacle_cruise_planner): tune obstacle_cruise_planner for cruising front NPCs in dense urban ODD scenarios (`#1166 <https://github.com/autowarefoundation/autoware_launch/issues/1166>`_)
  fix(obstacle_cruise_planner): tune obstacle_cruise_planner for cruising front NPCs in dense urban ODD scenarios
* feat(pose_initializer): add new parameter for check error between initial pose and GNSS pose (`#1180 <https://github.com/autowarefoundation/autoware_launch/issues/1180>`_)
  * add pose_error_check_enabled parameter
  * change default value
  ---------
* feat(autonomous_emergency_braking): initiate speed_calculation_expansion_margin parameter (`#1168 <https://github.com/autowarefoundation/autoware_launch/issues/1168>`_)
  initiate speed_calculation_expansion_margin parameter
* feat(system_error_monitor): delete system error monitor (`#1178 <https://github.com/autowarefoundation/autoware_launch/issues/1178>`_)
  feat: delete system error monitor
* revert: feat: change visualization of localization results from PoseHistory to PoseWithCovarianceHistory (`#1164 <https://github.com/autowarefoundation/autoware_launch/issues/1164>`_) (`#1179 <https://github.com/autowarefoundation/autoware_launch/issues/1179>`_)
  Revert "feat: change visualization of localization results from PoseHistory to PoseWithCovarianceHistory (`#1164 <https://github.com/autowarefoundation/autoware_launch/issues/1164>`_)"
  This reverts commit 593ad1f6c2ad967d8d04b349d7970deeed3f47a1.
* fix(perception): adopt awsim (tlr) camera topic (`#1177 <https://github.com/autowarefoundation/autoware_launch/issues/1177>`_)
* feat(lane_change): add lane change parameter (`#1157 <https://github.com/autowarefoundation/autoware_launch/issues/1157>`_)
  add parameter to enable/disable bound check
* fix(avoidance_by_lane_change): remove unused parameter (`#1176 <https://github.com/autowarefoundation/autoware_launch/issues/1176>`_)
  remove unused parameter
* feat(emergency_handler): delete package (`#1173 <https://github.com/autowarefoundation/autoware_launch/issues/1173>`_)
  * feat(emergency_handler): delete package
* refactor(system_monitor/net_monitor): remove-missing-patameters (`#1175 <https://github.com/autowarefoundation/autoware_launch/issues/1175>`_)
  refactor: remove-missing-patameters
* refactor(system_monitor/ntp_monitor): add-missing-parameters (`#1174 <https://github.com/autowarefoundation/autoware_launch/issues/1174>`_)
  refactor: add-missing-parameters
* refactor(behavior_path_planner): remove unnecessary parameters (`#1172 <https://github.com/autowarefoundation/autoware_launch/issues/1172>`_)
* feat(tier4_perception_launch): enable to use multi camera on traffic light recognition (`#1144 <https://github.com/autowarefoundation/autoware_launch/issues/1144>`_)
  change the way to declare camera num
* style(rviz-config): use colors consistent with new theme (`#1169 <https://github.com/autowarefoundation/autoware_launch/issues/1169>`_)
* feat: change visualization of localization results from PoseHistory to PoseWithCovarianceHistory (`#1164 <https://github.com/autowarefoundation/autoware_launch/issues/1164>`_)
  * PoseHistory to PoseWithCovarianceHistory
  * style(pre-commit): autofix
  * fix param of alpha related to PoseWithCovarianceHistory
  ---------
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
* feat(crosswalk)!: update stop position caluculation (`#1162 <https://github.com/autowarefoundation/autoware_launch/issues/1162>`_)
* feat: add an env variable to enable the new rviz2 theme (`#1017 <https://github.com/autowarefoundation/autoware_launch/issues/1017>`_)
* feat(start_planner): add option to skip rear vehicle check (`#1165 <https://github.com/autowarefoundation/autoware_launch/issues/1165>`_)
* feat(run_out): speed up run out response (`#1163 <https://github.com/autowarefoundation/autoware_launch/issues/1163>`_)
  speed up run out response
* feat(mission_planner): add option to prevent rerouting in autonomous driving mode (`#1153 <https://github.com/autowarefoundation/autoware_launch/issues/1153>`_)
* feat: add parameters for restart suppression in crosswalk (`#1160 <https://github.com/autowarefoundation/autoware_launch/issues/1160>`_)
  * feat: add parameters for restart suppression in crosswalk
  * update parameter
  ---------
* feat(goal_planner): dense goal candidate sampling in BusStopArea (`#1156 <https://github.com/autowarefoundation/autoware_launch/issues/1156>`_)
* chore(tier4_pereption_component): add image_segmentation_based_filter option param (`#1158 <https://github.com/autowarefoundation/autoware_launch/issues/1158>`_)
* feat(occupancy_grid_map): add option for time keeper (`#1138 <https://github.com/autowarefoundation/autoware_launch/issues/1138>`_)
  * add option for time keeper
  * set default to false
  ---------
  Co-authored-by: Taekjin LEE <technolojin@gmail.com>
* feat(ground_segmentation): add option for time keeper (`#1134 <https://github.com/autowarefoundation/autoware_launch/issues/1134>`_)
  add option for time keeper
  Co-authored-by: Taekjin LEE <technolojin@gmail.com>
* feat(occupancy_grid_map_outlier_filter): add option for time keeper (`#1147 <https://github.com/autowarefoundation/autoware_launch/issues/1147>`_)
  add timekeeper option
  Co-authored-by: Taekjin LEE <technolojin@gmail.com>
* feat(autoware_mpc_lateral_controller): add resampled reference trajectory for debug purpose (`#1114 <https://github.com/autowarefoundation/autoware_launch/issues/1114>`_)
  * chore: add debug_publish_resampled_reference_trajectory to parameter
  * feat: add use_delayed_initial_state flag to lateral MPC configuration
  ---------
* feat(autoware_launch): add expansion params (`#1133 <https://github.com/autowarefoundation/autoware_launch/issues/1133>`_)
  make expansion optional
* feat: add simulator rviz config (`#1150 <https://github.com/autowarefoundation/autoware_launch/issues/1150>`_)
* feat(autoware_lidar_transfusion): add transfusion config (`#1093 <https://github.com/autowarefoundation/autoware_launch/issues/1093>`_)
* fix(static_obstacle_avoidance): increase prepare time (`#1148 <https://github.com/autowarefoundation/autoware_launch/issues/1148>`_)
* fix(static_obstacle_avoidance): tune parameters (`#1143 <https://github.com/autowarefoundation/autoware_launch/issues/1143>`_)
* fix(min-velocity-map-based-prediction): reduce min_velocity_for_map_based_prediction (`#994 <https://github.com/autowarefoundation/autoware_launch/issues/994>`_)
  fix(min-velocity-map-based-prediction): reduce min_velocity_for_map_based_prediction to let intersection module run with low speed npc
* chore(stop_filter): extract stop_filter.param.yaml to autoware_launch (`#1145 <https://github.com/autowarefoundation/autoware_launch/issues/1145>`_)
  Extract stop_filter.param.yaml to autoware_launch
* feat: fix parameter type error in occupancy_grid_map_outlier_filter.param.yaml (`#1146 <https://github.com/autowarefoundation/autoware_launch/issues/1146>`_)
  * feat: fix parameter type
  * chore: change param name
  ---------
* feat(detected_object_validation): copy parameter files update from universe (`#1126 <https://github.com/autowarefoundation/autoware_launch/issues/1126>`_)
  feat: copy params from universe
* feat(pid_longitudinal_controller)!: add acceleration feedback block (`#1139 <https://github.com/autowarefoundation/autoware_launch/issues/1139>`_)
  * add params
  ---------
* feat(occupancy_grid_based_outlier_fillter): add config file to autoware_launch (`#1137 <https://github.com/autowarefoundation/autoware_launch/issues/1137>`_)
  * feat: add config file
  * style(pre-commit): autofix
  ---------
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
* feat(obstacle_pointcloud_based_validator): add enable_debugger parameter (`#1123 <https://github.com/autowarefoundation/autoware_launch/issues/1123>`_)
  * feat: add enable debugger parameter
  * style(pre-commit): autofix
  ---------
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
* fix(ekf_localizer): change roll, pitch proc dev (`#1140 <https://github.com/autowarefoundation/autoware_launch/issues/1140>`_)
  change roll, pitch proc dev
* feat(out_of_lane): redesign to improve accuracy and performance (`#1117 <https://github.com/autowarefoundation/autoware_launch/issues/1117>`_)
* feat(localization): add lidar_marker_localizer (`#861 <https://github.com/autowarefoundation/autoware_launch/issues/861>`_)
  * add config files
  * style(pre-commit): autofix
  * add param marker_height_from_ground
  * save log param
  * apply PointXYZIRC
  * to pass spell-check
  * refactor
  * change flag
  * fix typo
  ---------
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
  Co-authored-by: SakodaShintaro <rgbygscrsedppbwg@gmail.com>
* feat(raw_vehicle_cmd_converter): disable actuation to steering (`#1132 <https://github.com/autowarefoundation/autoware_launch/issues/1132>`_)
* chore(e2e_launch): add launch_sensing_driver arg (`#1095 <https://github.com/autowarefoundation/autoware_launch/issues/1095>`_)
* feat(raw_vehicle_cmd_converter): add steer command conversion with VGR (`#1131 <https://github.com/autowarefoundation/autoware_launch/issues/1131>`_)
* feat(lane_change): consider deceleration in safety check for cancel (`#1068 <https://github.com/autowarefoundation/autoware_launch/issues/1068>`_)
* refactor(lane_change): rename prepare_segment_ignore_object_velocity_thresh (`#1125 <https://github.com/autowarefoundation/autoware_launch/issues/1125>`_)
  change parameter name to a more expressive one
* feat(static_obstacle_avoidance): add parameter for envelope polygon creation (`#1130 <https://github.com/autowarefoundation/autoware_launch/issues/1130>`_)
  * add threshold for eclipse long radius
  * change parameter
  ---------
* perf(goal_planner): faster path sorting and selection (`#1119 <https://github.com/autowarefoundation/autoware_launch/issues/1119>`_)
* chore(vehicle_cmd_gate): delete deprecated parameters (`#1127 <https://github.com/autowarefoundation/autoware_launch/issues/1127>`_)
  delete deprecated params in vehicle_cmd_gate.param.yaml
* feat(freespace_planning_algorithms): add new parameters for astar planning algorithm (`#1120 <https://github.com/autowarefoundation/autoware_launch/issues/1120>`_)
  * add new astar planner parameters
  * add flag for obstacle confidence check
  * reduce freespace planner th_arrived_distance_m param value
  * reduce object polygon expand size in costmap generator
  * reduce vehicle shape margin in freespace planner
  * replace flag param by time threshold param
  ---------
* feat(tier4_perception_launch): add transfusion option for lidar_detection_model (`#1124 <https://github.com/autowarefoundation/autoware_launch/issues/1124>`_)
* fix(lidar_model): add centerpoint_sigma param file (`#1086 <https://github.com/autowarefoundation/autoware_launch/issues/1086>`_)
  fix: add centerpoint_sigma param file
* chore(autoware_multi_object_tracker): fix typo in input_channels (`#1121 <https://github.com/autowarefoundation/autoware_launch/issues/1121>`_)
  chore: fix typo of lidar_pointpainitng channel
* feat(psim)!: preapre settings to launch localization modules on psim (`#1094 <https://github.com/autowarefoundation/autoware_launch/issues/1094>`_)
* fix(lane_change): parameter update (`#1115 <https://github.com/autowarefoundation/autoware_launch/issues/1115>`_)
* feat(autoware_map_based_prediction): add debug parameters for map-based prediction (`#1118 <https://github.com/autowarefoundation/autoware_launch/issues/1118>`_)
  * feat: add debug parameters for map-based prediction
  * style(pre-commit): autofix
  ---------
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
* feat(psim)!: change a setting parameter type from bool to string (`#1106 <https://github.com/autowarefoundation/autoware_launch/issues/1106>`_)
  * change a param type, bool to string
  ---------
* fix(goal_planner): fix lane departure check not working correctly due to uninitialized variable (`#1116 <https://github.com/autowarefoundation/autoware_launch/issues/1116>`_)
* feat(static_obstacle_avoidance): change policy for ambiguous avoidance situation (`#1113 <https://github.com/autowarefoundation/autoware_launch/issues/1113>`_)
  * feat(static_obstacle_avoidance): change policy for ambiguous avoidance situation
  * fix(static_obstacle_avoidance): tune ambiguous vehicle ignore area
  ---------
* fix(lane_change): skip generating path if longitudinal distance difference is less than threshold (`#1108 <https://github.com/autowarefoundation/autoware_launch/issues/1108>`_)
  add skip process lon dist diff threshold
* feat(tracking_object_merger): add merge frame (`#1112 <https://github.com/autowarefoundation/autoware_launch/issues/1112>`_)
* fix(mpc_lateral_controller): publish predicted trajectory in Frenet coordinate and visualize it on Rviz (`#1111 <https://github.com/autowarefoundation/autoware_launch/issues/1111>`_)
* feat: increase the number of processes monitored by process_monitor (`#1110 <https://github.com/autowarefoundation/autoware_launch/issues/1110>`_)
* feat(lane_change): use different rss param to deal with parked vehicle (`#1104 <https://github.com/autowarefoundation/autoware_launch/issues/1104>`_)
  use separate rss for parked vehicle
* feat(lane_change): add param for lateral angle  deviation (`#1087 <https://github.com/autowarefoundation/autoware_launch/issues/1087>`_)
  * RT1-6514 adding lateral angle deviation param
  * decrease angle deviation threshold to fix rtc issue
  ---------
* feat(autonomous_emergency_braking): add info marker to aeb and state check override (`#1103 <https://github.com/autowarefoundation/autoware_launch/issues/1103>`_)
  * add info marker and override for state
  * make stop wall viz default
  ---------
* feat(behavior_path _planner): divide planner manager modules into dependent slots (`#1091 <https://github.com/autowarefoundation/autoware_launch/issues/1091>`_)
* feat(autonomous_emergency_braking): enable AEB stop in vehicle_cmd_gate and diag_graph_agg (`#1099 <https://github.com/autowarefoundation/autoware_launch/issues/1099>`_)
  * enable emergency handling for AEB stop
  * update AEB params to work better at 30 kmph
  ---------
* feat(static_obstacle_avoidance): add force deactivation duration time (`#1101 <https://github.com/autowarefoundation/autoware_launch/issues/1101>`_)
  add force cancel duration time
* perf(freespace_planning_algorithms): tune freespace planner parameters (`#1097 <https://github.com/autowarefoundation/autoware_launch/issues/1097>`_)
  * reduce longitudinal goal range
  * tune parameters
  ---------
* feat(dynamic_obstacle_avoidance): shorter predicted path for pedestrians (`#1084 <https://github.com/autowarefoundation/autoware_launch/issues/1084>`_)
* feat(crosswalk): more conservative when the ego pass first (`#1085 <https://github.com/autowarefoundation/autoware_launch/issues/1085>`_)
  * feat: use obstacle_cruise_planner and change safe_distance_margin
  * feat: set max_vel to 40km/h
  * feat: enable surround_obstacle_checker
  * feat: enable surround_obstacle_checker
  * feat: enable dynamic_avoidance and disable outside_drivable_area_stop
  * feat: disable AEB and set the maximum velocity to 40km/h
  * enable intersection_occlusion detection
  * chore(planning_launch): update motion module name (`#1014 <https://github.com/autowarefoundation/autoware_launch/issues/1014>`_)
  * disable AEB diag check
  * feat(diagnostic_graph_utils): launch logging node for diagnostic_graph
  * feat(api): set launch_deprecated_api true (`#496 <https://github.com/autowarefoundation/autoware_launch/issues/496>`_)
  feat(api): launch_deprecated_api=true
  * fix(api): disable rosbridge to fix duplicated node (`#497 <https://github.com/autowarefoundation/autoware_launch/issues/497>`_)
  * feat(crosswalk): more conservative when the ego pass first
  ---------
  Co-authored-by: tier4-autoware-public-bot[bot] <98652886+tier4-autoware-public-bot[bot]@users.noreply.github.com>
  Co-authored-by: Mamoru Sobue <mamoru.sobue@tier4.jp>
  Co-authored-by: Mamoru Sobue <hilo.soblin@gmail.com>
  Co-authored-by: badai nguyen <94814556+badai-nguyen@users.noreply.github.com>
  Co-authored-by: Satoshi OTA <44889564+satoshi-ota@users.noreply.github.com>
  Co-authored-by: Daniel Sanchez <danielsanchezaran@gmail.com>
  Co-authored-by: danielsanchezaran <daniel.sanchez@tier4.jp>
  Co-authored-by: Takamasa Horibe <horibe.takamasa@gmail.com>
  Co-authored-by: Kosuke Takeuchi <kosuke.tnp@gmail.com>
* perf(ndt_scan_matcher): change the temperature of multi_ndt_score to 0.05 (`#1096 <https://github.com/autowarefoundation/autoware_launch/issues/1096>`_)
  Changed the temperature of multi_ndt_score
* feat(out_of_lane): add lateral buffer between the lane and stop pose (`#1098 <https://github.com/autowarefoundation/autoware_launch/issues/1098>`_)
* feat(freespace_planning_algorithm): update freespace planner params (`#1080 <https://github.com/autowarefoundation/autoware_launch/issues/1080>`_)
  * update freespace planner params
  * update goal planner params
  * update start planner params
  * rename parameter
  * change parameter value
  ---------
* feat(dynamic_drivable_area_expansion): min_bound_interval parameter (`#1092 <https://github.com/autowarefoundation/autoware_launch/issues/1092>`_)
* feat(pid_longitudinal_controller): re-organize diff limit structure (`#1052 <https://github.com/autowarefoundation/autoware_launch/issues/1052>`_)
  * rearange params
* feat(start_planner): set end_pose_curvature_threshold 0.1 (`#1088 <https://github.com/autowarefoundation/autoware_launch/issues/1088>`_)
* feat(out_of_lane): add parameter to ignore objects behind ego (`#1062 <https://github.com/autowarefoundation/autoware_launch/issues/1062>`_)
* feat(start_planner): add end_pose_curvature_threshold (`#1059 <https://github.com/autowarefoundation/autoware_launch/issues/1059>`_)
* feat(vehicle_cmd_gate): change param to relax pedal rate limit when the vehicle velocity is slow enough (`#1077 <https://github.com/autowarefoundation/autoware_launch/issues/1077>`_)
  * change param
* feat(ndt_scan_matcher): add scale_factor to covariance_estimation (`#1081 <https://github.com/autowarefoundation/autoware_launch/issues/1081>`_)
  Added scale_factor to ndt_scan_matcher.covariance_estimation
* feat(simple_planning_simulator): add actuation command simulator (`#1078 <https://github.com/autowarefoundation/autoware_launch/issues/1078>`_)
* feat(e2e_simulator.launch): renamed carla interface package in e2e_launch (`#1075 <https://github.com/autowarefoundation/autoware_launch/issues/1075>`_)
  renamed carla package to autoware_carla_interface
* feat(control_validator)!: add velocity check (`#1050 <https://github.com/autowarefoundation/autoware_launch/issues/1050>`_)
  add param
* chore: add ml detectors' buffer size (`#1067 <https://github.com/autowarefoundation/autoware_launch/issues/1067>`_)
* fix(obstacle_cruise_planner): guarantee the stop margin (`#1076 <https://github.com/autowarefoundation/autoware_launch/issues/1076>`_)
* fix(static_obstacle_avoidance): check stopped time in freespace (`#1074 <https://github.com/autowarefoundation/autoware_launch/issues/1074>`_)
* feat(autoware_behavior_path_planner): remove max_iteration_num parameter (`#1064 <https://github.com/autowarefoundation/autoware_launch/issues/1064>`_)
  Update the behavior_path_planner.param.yaml file to remove the max_iteration_num parameter
* feat: add config for processing_time_checker (`#1072 <https://github.com/autowarefoundation/autoware_launch/issues/1072>`_)
* feat(duplicated_node_checker): add duplicate nodes to ignore (`#1070 <https://github.com/autowarefoundation/autoware_launch/issues/1070>`_)
  * feat(duplicated_node_checker): add duplicate nodes to ignore
  * pre-commit
  ---------
  Co-authored-by: Dmitrii Koldaev <dmitrii.koldaev@tier4.jp>
  Co-authored-by: Tomoya Kimura <tomoya.kimura@tier4.jp>
* feat(tier4_perception_component): refactored launch options (`#1060 <https://github.com/autowarefoundation/autoware_launch/issues/1060>`_)
  * chore: refactored launch options
  * modify launcher
  * fix args
  ---------
  Co-authored-by: kminoda <44218668+kminoda@users.noreply.github.com>
* feat(static_obstacle_avoidance): add new option to change policy (`#1065 <https://github.com/autowarefoundation/autoware_launch/issues/1065>`_)
* feat(map_loader, route_handler)!: add format_version validation (`#993 <https://github.com/autowarefoundation/autoware_launch/issues/993>`_)
  feat(map_loader): add format_version validation
* feat(autonomous_emergency_braking): add param for oublishing debug markers (`#1063 <https://github.com/autowarefoundation/autoware_launch/issues/1063>`_)
  add param for oublishing debug markers
* feat(ndt_scan_matcher): add params (`#1038 <https://github.com/autowarefoundation/autoware_launch/issues/1038>`_)
  * add params (ndt_scan_matcher)
  * fix param
  * rviz
  * rviz
  * rviz
  * style(pre-commit): autofix
  * true2false
  * Add temperature to parameters in autoware_launch
  ---------
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
* feat(goal_planner): prioritize pull over path by curvature (`#1048 <https://github.com/autowarefoundation/autoware_launch/issues/1048>`_)
* refactor(tier4_control_launch): replace python launch with xml (`#1047 <https://github.com/autowarefoundation/autoware_launch/issues/1047>`_)
  migrate to control.launch.xml
* feat(obstacle_cruise_planner): support pointcloud-based obstacles (`#980 <https://github.com/autowarefoundation/autoware_launch/issues/980>`_)
  * feat: use obstacle_cruise_planner and change safe_distance_margin
  * feat: set max_vel to 40km/h
  * feat: enable surround_obstacle_checker
  * feat: enable surround_obstacle_checker
  * feat: enable dynamic_avoidance and disable outside_drivable_area_stop
  * feat: disable AEB and set the maximum velocity to 40km/h
  * enable intersection_occlusion detection
  * add parameters for obstacle_cruise_planner
  * add parameters for pointcloud filtering
  * chore(planning_launch): update motion module name (`#1014 <https://github.com/autowarefoundation/autoware_launch/issues/1014>`_)
  * move use_pointcloud to common parameter
  * disable using pointcloud by default
  * disable AEB diag check
  * remove use_pointcloud parameter
  * feat(diagnostic_graph_utils): launch logging node for diagnostic_graph
  * reset to autowarefoundation:main
  ---------
  Co-authored-by: Takayuki Murooka <takayuki5168@gmail.com>
  Co-authored-by: tier4-autoware-public-bot[bot] <98652886+tier4-autoware-public-bot[bot]@users.noreply.github.com>
  Co-authored-by: Mamoru Sobue <mamoru.sobue@tier4.jp>
  Co-authored-by: Mamoru Sobue <hilo.soblin@gmail.com>
  Co-authored-by: badai nguyen <94814556+badai-nguyen@users.noreply.github.com>
  Co-authored-by: Satoshi OTA <44889564+satoshi-ota@users.noreply.github.com>
  Co-authored-by: Daniel Sanchez <danielsanchezaran@gmail.com>
  Co-authored-by: danielsanchezaran <daniel.sanchez@tier4.jp>
  Co-authored-by: Takamasa Horibe <horibe.takamasa@gmail.com>
* chore(eagleye): add septentrio msg option in eagleye_config (`#1049 <https://github.com/autowarefoundation/autoware_launch/issues/1049>`_)
  Added septentrio option for velocity_source in eagleye_config.param.yaml
* feat(behavior_path_planner): remove max_module_size param (`#1045 <https://github.com/autowarefoundation/autoware_launch/issues/1045>`_)
  The max_module_size param has been removed from the behavior_path_planner scene_module_manager.param.yaml file. This param was unnecessary and has been removed to simplify the configuration.
* feat(ekf_localizer): add covariance ellipse diagnostics (`#1041 <https://github.com/autowarefoundation/autoware_launch/issues/1041>`_)
  * Added ellipse diagnostics to ekf
  * Fixed to ellipse_scale
  ---------
* feat(autoware_launch): use mrm handler by default (`#1043 <https://github.com/autowarefoundation/autoware_launch/issues/1043>`_)
* refactor(static_obstacle_avoidance): organize params for drivable lane (`#1042 <https://github.com/autowarefoundation/autoware_launch/issues/1042>`_)
* feat(behavior_path_planner): add yaw threshold param (`#1040 <https://github.com/autowarefoundation/autoware_launch/issues/1040>`_)
  add yaw threshold param
* feat(autonomous_emergency_braking): add and tune params (`#1037 <https://github.com/autowarefoundation/autoware_launch/issues/1037>`_)
  * add and tune params
  * set back voxel grid z
  * fix grid to what it is in OSS launch
  ---------
* feat(static_obstacle_avoidance)!: add param to select path generation method (`#1036 <https://github.com/autowarefoundation/autoware_launch/issues/1036>`_)
  feat(static_obstacle_avoidance): add param to select path generation method
* fix(object_lanelet_filter): radar object lanelet filter parameter update (`#1032 <https://github.com/autowarefoundation/autoware_launch/issues/1032>`_)
  fix: radar object lanelet filter parameter update
  fix
* feat(autonomous_emergency_braking): add params to enable or disable PC and predicted objects (`#1031 <https://github.com/autowarefoundation/autoware_launch/issues/1031>`_)
  * add params to enable or disable PC and predicted objects
  * set predicted object usage to false
  ---------
* feat: add use_waypoints parameter in map_loader (`#1028 <https://github.com/autowarefoundation/autoware_launch/issues/1028>`_)
* feat(autonomous_emergency_braking): add param to toggle on or off object speed calc for aeb (`#1029 <https://github.com/autowarefoundation/autoware_launch/issues/1029>`_)
  add param to toggle on or off object speed calc for aeb
* refactor(ndt scan matcher): update parameter (`#1018 <https://github.com/autowarefoundation/autoware_launch/issues/1018>`_)
  * rename to sensor_points.timeout_sec
  * parameterize skipping_publish_num
  * parameterize initial_to_result_distance_tolerance_m
  * add new line
  ---------
* refactor(dynamic_obstacle_stop): move to motion_velocity_planner (`#1025 <https://github.com/autowarefoundation/autoware_launch/issues/1025>`_)
* fix(start_planner): redefine the necessary parameters (`#1027 <https://github.com/autowarefoundation/autoware_launch/issues/1027>`_)
  restore necessary param
* refactor(start_planner): remove unused parameters in start planner module (`#1022 <https://github.com/autowarefoundation/autoware_launch/issues/1022>`_)
  refactor: remove unused parameters in start planner module
* feat(obstacle_velocity_limiter): move to motion_velocity_planner (`#1023 <https://github.com/autowarefoundation/autoware_launch/issues/1023>`_)
* refactor(raw_vehicle_cmd_converter)!: prefix package and namespace with autoware (`#1021 <https://github.com/autowarefoundation/autoware_launch/issues/1021>`_)
  fix
* refactor(out_of_lane): remove from behavior_velocity (`#1020 <https://github.com/autowarefoundation/autoware_launch/issues/1020>`_)
* feat(autonomous_emergency_braking): add autoware prefix to AEB (`#1019 <https://github.com/autowarefoundation/autoware_launch/issues/1019>`_)
  * rename AEB param folder
  * change param path and add commented out emergency stop enabling
  ---------
* feat(obstacle_cruise)!: type specified stop deccel limit and enabling abandon to stop (`#1003 <https://github.com/autowarefoundation/autoware_launch/issues/1003>`_)
  abandon_to_stop
* feat(obstacle_curise): revert lateral stop margin for unknown objects (`#1015 <https://github.com/autowarefoundation/autoware_launch/issues/1015>`_)
* feat!: change from autoware_auto_msgs to autoware_msgs (`#1012 <https://github.com/autowarefoundation/autoware_launch/issues/1012>`_)
  * feat(autoware_launch): replace autoware_auto_mapping_msg with autoware_map_msg (`#688 <https://github.com/autowarefoundation/autoware_launch/issues/688>`_)
  feat(autoware_launch): remove autoware auto mapping msg
  * fix: planning_msg (`#717 <https://github.com/autowarefoundation/autoware_launch/issues/717>`_)
  fix:planning_msg
  * feat(autoware_launch): replace autoware_control_msg with autoware_con… (`#725 <https://github.com/autowarefoundation/autoware_launch/issues/725>`_)
  feat(autoware_launch): replace autoware_control_msg with autoware_control_msg
  * feat(autoware_launch): replace autoware_auto_vehicle_msgs with autoware_vehicle_msgs
  * fix(topics.yaml): fix AUTO button bug
  * feat(autoware_launch): rename autoware_auto_perception_rviz_plugin to autoware_perception_rviz_plugin
  * feat: rename TrafficSignal messages to TrafficLightGroup
  ---------
  Co-authored-by: cyn-liu <104069308+cyn-liu@users.noreply.github.com>
  Co-authored-by: shulanbushangshu <102840938+shulanbushangshu@users.noreply.github.com>
  Co-authored-by: NorahXiong <103234047+NorahXiong@users.noreply.github.com>
  Co-authored-by: liu cui <cynthia.liu@autocore.ai>
  Co-authored-by: Ryohsuke Mitsudome <ryohsuke.mitsudome@tier4.jp>
* chore(planning_launch): update motion module name (`#1014 <https://github.com/autowarefoundation/autoware_launch/issues/1014>`_)
* feat: rename autoware_auto_perception_rviz_plugin to autoware_perception_rviz_plugin (`#1013 <https://github.com/autowarefoundation/autoware_launch/issues/1013>`_)
* feat: update rviz layout (`#1004 <https://github.com/autowarefoundation/autoware_launch/issues/1004>`_)
* feat(lane_departure_checker): add params for lane departure margin (`#1011 <https://github.com/autowarefoundation/autoware_launch/issues/1011>`_)
  * add params
  * add param for start planner lane departure expansion margin
  ---------
* refactor(image_projection_based_fusion): rework params (`#845 <https://github.com/autowarefoundation/autoware_launch/issues/845>`_)
* feat(obstacle_cruise_planner)!: ignore to garze against unknwon objects (`#1009 <https://github.com/autowarefoundation/autoware_launch/issues/1009>`_)
* chore(planning_launch): update module name (`#1008 <https://github.com/autowarefoundation/autoware_launch/issues/1008>`_)
  * chore(planning_launch): update module name
  * chore(rviz): update rviz config
  * chore(avoidance): update module name
  ---------
* feat(motion_velocity_planner): add new motion velocity planning (`#992 <https://github.com/autowarefoundation/autoware_launch/issues/992>`_)
* feat(map_based_prediction): use different time horizon (`#1005 <https://github.com/autowarefoundation/autoware_launch/issues/1005>`_)
* feat(behavior_path_planner_common,turn_signal_decider): add turn_signal_remaining_shift_length_threshold (`#1007 <https://github.com/autowarefoundation/autoware_launch/issues/1007>`_)
  add turn_signal_remaining_shift_length_threshold
* revert(map_based_prediction): use different time horizon (`#967 <https://github.com/autowarefoundation/autoware_launch/issues/967>`_) (`#1006 <https://github.com/autowarefoundation/autoware_launch/issues/1006>`_)
* feat(map_based_prediction): use different time horizon (`#967 <https://github.com/autowarefoundation/autoware_launch/issues/967>`_)
* feat(blind_spot): consider time to collision (`#1002 <https://github.com/autowarefoundation/autoware_launch/issues/1002>`_)
* feat(object_lanelet_filter): update object_lanelet_filter parameter yaml (`#998 <https://github.com/autowarefoundation/autoware_launch/issues/998>`_)
  feat: update object_lanelet_filter parameter
* feat(autoware_launch): add diagnostic graph config for awsim (`#1000 <https://github.com/autowarefoundation/autoware_launch/issues/1000>`_)
* fix(rviz): remove StringStampedOverlayDisplay reference (`#1001 <https://github.com/autowarefoundation/autoware_launch/issues/1001>`_)
* feat(e2e_simulator.launch): add argument for running the CARLA interface (`#924 <https://github.com/autowarefoundation/autoware_launch/issues/924>`_)
* feat: add diagnostic graph settings (`#991 <https://github.com/autowarefoundation/autoware_launch/issues/991>`_)
* feat(multi_object_tracker): add multi object input config file (`#989 <https://github.com/autowarefoundation/autoware_launch/issues/989>`_)
  * feat: add multi-input channel config
  * fix: component config
  * fix: remove expected interval, add spawn
  * fix: missing config, default value
  ---------
* feat!(avoidance): make it selectable output debug marker from yaml (`#996 <https://github.com/autowarefoundation/autoware_launch/issues/996>`_)
  feat(avoidance): make it selectable output debug marker from yaml
* fix(avoidance): change lateral jerk param (`#995 <https://github.com/autowarefoundation/autoware_launch/issues/995>`_)
* fix(ndt_scan_matchere): improved tpe (`#985 <https://github.com/autowarefoundation/autoware_launch/issues/985>`_)
  Improved tpe
* feat(out_of_lane): add option to ignore overlaps in lane changes (`#986 <https://github.com/autowarefoundation/autoware_launch/issues/986>`_)
* feat(map_based_prediction): incorporate crosswalk user history (`#987 <https://github.com/autowarefoundation/autoware_launch/issues/987>`_)
* feat(remaining_dist_eta): add MissionDetailsDisplay plugin rviz configuration (`#963 <https://github.com/autowarefoundation/autoware_launch/issues/963>`_)
* fix: update widget size and position (`#982 <https://github.com/autowarefoundation/autoware_launch/issues/982>`_)
* feat(path_planner): params to adjust hard constraints and path reuse (`#983 <https://github.com/autowarefoundation/autoware_launch/issues/983>`_)
* fix(componet_state_monitor): remove ndt node alive monitoring (`#984 <https://github.com/autowarefoundation/autoware_launch/issues/984>`_)
  remove ndt node alive monitoring
* feat(autonomous_emergency_braking): add obstacle velocity estimation for aeb (`#978 <https://github.com/autowarefoundation/autoware_launch/issues/978>`_)
  * rebase to awf main
  * set debug PC as false
  * dictionary
  * eliminate duplicate parameter
  * eliminate duplicate parameter
  ---------
* feat(crosswalk)!: change a hard coding number and set as param (`#977 <https://github.com/autowarefoundation/autoware_launch/issues/977>`_)
  * change param
* fix: update traffic topic in autoware.rviz  (`#981 <https://github.com/autowarefoundation/autoware_launch/issues/981>`_)
* chore(component_state_monitor): relax pose_estimator_pose timeout (`#979 <https://github.com/autowarefoundation/autoware_launch/issues/979>`_)
* feat(system diags): rename diag of ndt scan matcher (`#973 <https://github.com/autowarefoundation/autoware_launch/issues/973>`_)
  rename ndt diag
* fix(avoidance): add target filtering threshold for merging/deviating vehicle (`#974 <https://github.com/autowarefoundation/autoware_launch/issues/974>`_)
* fix(ekf_localizer): updated ekf gate_dist params (`#965 <https://github.com/autowarefoundation/autoware_launch/issues/965>`_)
  Updated ekf gate_dist
* fix(lidar_centerpoint): add param file for centerpoint_tiny (`#976 <https://github.com/autowarefoundation/autoware_launch/issues/976>`_)
  fix(lidar_centerpoint): add param file
* feat(probabilistic_occupancy_grid_map): add downsample filter option to ogm creation  (`#962 <https://github.com/autowarefoundation/autoware_launch/issues/962>`_)
  * feat(probabilistic_occupancy_grid_map): add downsample filter option to ogm creation
  * chore: do not use pointcloud filter when downsample is true
  * Update autoware_launch/config/perception/occupancy_grid_map/multi_lidar_pointcloud_based_occupancy_grid_map.param.yaml
  Co-authored-by: Yukihiro Saito <yukky.saito@gmail.com>
  ---------
  Co-authored-by: Yukihiro Saito <yukky.saito@gmail.com>
* refactor(centerpoint, pointpainting): rearrange ML model and package params (`#915 <https://github.com/autowarefoundation/autoware_launch/issues/915>`_)
  * chore: separate param files
  * chore: fix launch
  * chore: rearrange param
  * style(pre-commit): autofix
  * refactor: rearrange param file
  * chore: move densification_params
  * style(pre-commit): autofix
  * fix(centerpoint): align param namespace with pointpainting
  * fix: param
  * fix: remove build_only from yaml
  ---------
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
* feat(autoware_launch): add centerpoint_sigma param to pointpainting.param.yaml (`#955 <https://github.com/autowarefoundation/autoware_launch/issues/955>`_)
  fix: add has_variance to pointpainting.param.yaml
* feat(autonomous_emergency_braking): add params for aeb (`#966 <https://github.com/autowarefoundation/autoware_launch/issues/966>`_)
  * add params for aeb
  * set collision keep time to be more conservative
  ---------
* fix(roi_pointcloud_fusion): add param (`#956 <https://github.com/autowarefoundation/autoware_launch/issues/956>`_)
* refactor(bpp): remove unused params (`#961 <https://github.com/autowarefoundation/autoware_launch/issues/961>`_)
* feat(api): add launch option (`#960 <https://github.com/autowarefoundation/autoware_launch/issues/960>`_)
* feat(dynamic_avoidance): avoid pedestrians (`#958 <https://github.com/autowarefoundation/autoware_launch/issues/958>`_)
  new feature
* chore(intersection_occlusion): more increase possible_object_bbox size to ignore small occlusion and ghost stop (`#959 <https://github.com/autowarefoundation/autoware_launch/issues/959>`_)
* feat(obstacle_cruise): change stop lateral margin (`#948 <https://github.com/autowarefoundation/autoware_launch/issues/948>`_)
* refactor(avoidance): unify redundant parameters (`#953 <https://github.com/autowarefoundation/autoware_launch/issues/953>`_)
  refactor(avoidance): remove unused parameters
* refactor(avoidance, AbLC): rebuild parameter structure (`#951 <https://github.com/autowarefoundation/autoware_launch/issues/951>`_)
  * refactor(avoidance): update yaml
  * refactor(AbLC): update yaml
  ---------
* chore(intersection_occlusion): increase possible_object_bbox size to ignore small occlusion and ghost stop (`#950 <https://github.com/autowarefoundation/autoware_launch/issues/950>`_)
* fix(tier4_control_component_launch): fix duplicate declaration of controller parameter paths (`#940 <https://github.com/autowarefoundation/autoware_launch/issues/940>`_)
* fix(trajectory_follower): accommodate the parameters of the controllers to the dynamics in the simulator. (`#941 <https://github.com/autowarefoundation/autoware_launch/issues/941>`_)
  correct the parameters of the controller. The parameters of the dynamics and the controller are identical after this commit
* feat(avoidance): limit acceleration during avoidance maneuver (`#947 <https://github.com/autowarefoundation/autoware_launch/issues/947>`_)
  * feat(avoidance): limit acceleration during avoidance maneuver
  * fix(avoidance): tune longitudinal max acceleration
  ---------
* chore(ground_segmentation): add tuning param (`#946 <https://github.com/autowarefoundation/autoware_launch/issues/946>`_)
* feat(run_out): maintain stop wall for some seconds (`#944 <https://github.com/autowarefoundation/autoware_launch/issues/944>`_)
  update stop wall maintain time to 1 sec
* feat(lane_change): check prepare phase in turn direction lanes (`#943 <https://github.com/autowarefoundation/autoware_launch/issues/943>`_)
* feat(autoware_launch): add centerpoint_sigma param (`#945 <https://github.com/autowarefoundation/autoware_launch/issues/945>`_)
  add: centerpoint_sigma.param
* fix(lane_change): collision check for prepare in intersection (`#930 <https://github.com/autowarefoundation/autoware_launch/issues/930>`_)
* feat(start_planner): add path validation check (`#942 <https://github.com/autowarefoundation/autoware_launch/issues/942>`_)
  add param
* feat(pose_initilizer): set intial pose directly (`#937 <https://github.com/autowarefoundation/autoware_launch/issues/937>`_)
  * feat(pose_initilizer): set intial pose directly
  * rename params
  ---------
* feat(run_out): add params to exclude obstacles already on the ego's path (`#939 <https://github.com/autowarefoundation/autoware_launch/issues/939>`_)
  * add params
  * add extra param
  ---------
* feat(crosswalk): rename parameter to ignore traffic light (`#919 <https://github.com/autowarefoundation/autoware_launch/issues/919>`_)
* feat(dynamic_obstacle_stop): split the duration buffer parameter in 2 (add/remove) (`#933 <https://github.com/autowarefoundation/autoware_launch/issues/933>`_)
* chore: add option to select graph path depending on running mode (`#938 <https://github.com/autowarefoundation/autoware_launch/issues/938>`_)
  chore: add option of using graph path for simulation
* feat: add option to launch mrm handler (`#929 <https://github.com/autowarefoundation/autoware_launch/issues/929>`_)
* feat(run_out): add obstacle types to run out (`#936 <https://github.com/autowarefoundation/autoware_launch/issues/936>`_)
  add obstacle types to run out
* feat(run_out_module): new params for run out, add ego cut lane (`#935 <https://github.com/autowarefoundation/autoware_launch/issues/935>`_)
  * new params for run out
  * rename param
  * update description
  ---------
* feat: add dummy doors for planning simulator (`#921 <https://github.com/autowarefoundation/autoware_launch/issues/921>`_)
* feat(AEB): add detection range params (`#934 <https://github.com/autowarefoundation/autoware_launch/issues/934>`_)
  * feat(AEB): add new params for detection_range
  * fix(AEB): fix mistake
  ---------
* feat(run_out): adjust parameter (`#931 <https://github.com/autowarefoundation/autoware_launch/issues/931>`_)
  chore(run_out): adjust parameter (`#777 <https://github.com/autowarefoundation/autoware_launch/issues/777>`_)
  Co-authored-by: Shumpei Wakabayashi <42209144+shmpwk@users.noreply.github.com>
* refactor(avoidance): update parameter namespace (`#928 <https://github.com/autowarefoundation/autoware_launch/issues/928>`_)
* feat: add a param file of a mrm handler node (`#927 <https://github.com/autowarefoundation/autoware_launch/issues/927>`_)
* feat(dynamic_obstacle_stop): add parameter to ignore unavoidable collisions (`#916 <https://github.com/autowarefoundation/autoware_launch/issues/916>`_)
* fix(avoidance): wait and see objects (`#925 <https://github.com/autowarefoundation/autoware_launch/issues/925>`_)
* refactor(obstacle_cruise_planner): move slow down params to a clear location (`#926 <https://github.com/autowarefoundation/autoware_launch/issues/926>`_)
  move slow down params to a clear location
* refactor(avoidance): rename param (`#923 <https://github.com/autowarefoundation/autoware_launch/issues/923>`_)
* feat(crosswalk): increase minimum occlusion size that causes slowdown to 1m (`#909 <https://github.com/autowarefoundation/autoware_launch/issues/909>`_)
* feat: add marker for control's stop reason, false by default (`#912 <https://github.com/autowarefoundation/autoware_launch/issues/912>`_)
* chore(duplicated_node_checker): print duplication name (`#888 <https://github.com/autowarefoundation/autoware_launch/issues/888>`_)
* feat(pointcloud_preprocessor, probabilistic_occupancy_grid_map): enable multi lidar occupancy grid map creation pipeline (`#740 <https://github.com/autowarefoundation/autoware_launch/issues/740>`_)
  * add multi lidar pointcloud based ogm creation
  * enable sensing launch to control concatenate node
  * style(pre-commit): autofix
  * refactor : change concatenate node parameter name
  * chore: set single lidar ogm to be default
  * feat: update multi_lidar_ogm param file
  * chore: remove sensing launch changes because it does not needed
  * chore: fix multi lidar settings for sample sensor kit
  ---------
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
* chore: change default of low_height_crop filter use (`#918 <https://github.com/autowarefoundation/autoware_launch/issues/918>`_)
* feat(ndt_scan_matcher): added a parameter of sensor points (`#908 <https://github.com/autowarefoundation/autoware_launch/issues/908>`_)
  * Added parameters of sensor points
  * Added unit
  ---------
* feat(obstacle_cruise_planner): enable obstacle cruise's yield function by default (`#917 <https://github.com/autowarefoundation/autoware_launch/issues/917>`_)
  enable obstacle cruise's yield function by default
* fix(avoidance): tune safety check params (`#914 <https://github.com/autowarefoundation/autoware_launch/issues/914>`_)
* fix(avoidance): tune lateral margin params (`#913 <https://github.com/autowarefoundation/autoware_launch/issues/913>`_)
* fix(component_state_monitor): change pose_estimator_pose rate (`#910 <https://github.com/autowarefoundation/autoware_launch/issues/910>`_)
* feat(out_of_lane): add cut_beyond_red_traffic_lights parameter (`#885 <https://github.com/autowarefoundation/autoware_launch/issues/885>`_)
* feat(planning_simulator): default use_sim_time arg to scenario_simulation (`#903 <https://github.com/autowarefoundation/autoware_launch/issues/903>`_)
* fix(raw_vehicle_cmd_converter): csv paths are resolved in param.yaml (`#884 <https://github.com/autowarefoundation/autoware_launch/issues/884>`_)
* feat(start_planner): prevent hindering rear vehicles (`#905 <https://github.com/autowarefoundation/autoware_launch/issues/905>`_)
  Add params to add extra margin to rear vehicle width
* feat(avoidance): change lateral margin based on if it's parked vehicle (`#894 <https://github.com/autowarefoundation/autoware_launch/issues/894>`_)
  * feat(avoidance): change lateral margin based on if it's parked vehicle
  * fix(AbLC): update values
  ---------
* chore: change max_z of cropbox filter to vehicle_height (`#906 <https://github.com/autowarefoundation/autoware_launch/issues/906>`_)
  chore: change max_z of cropbox filter to vehicle_heigh
* fix: the parameter name of max_vel (`#907 <https://github.com/autowarefoundation/autoware_launch/issues/907>`_)
* feat: switch to obstacle_cruise_planner (`#765 <https://github.com/autowarefoundation/autoware_launch/issues/765>`_)
* feat: enable autonomous emergency braking (`#764 <https://github.com/autowarefoundation/autoware_launch/issues/764>`_)
* feat: set the max velocity to 15km/h (`#763 <https://github.com/autowarefoundation/autoware_launch/issues/763>`_)
* feat(tier4_localization_component_launch): change the default input pointcloud of localization into the concatenated pointcloud (`#899 <https://github.com/autowarefoundation/autoware_launch/issues/899>`_)
  * Make concat pointcloud default
  * style(pre-commit): autofix
  ---------
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
* feat(start_planner): add object_types_to_check_for_path_generation (`#902 <https://github.com/autowarefoundation/autoware_launch/issues/902>`_)
  add object_types_to_check_for_path_generation
* chore: update package maintainers for autoware_launch package (`#897 <https://github.com/autowarefoundation/autoware_launch/issues/897>`_)
* revert: feat(autoware_launch): set use_sim_time parameter equal to true when (`#746 <https://github.com/autowarefoundation/autoware_launch/issues/746>`_) (`#901 <https://github.com/autowarefoundation/autoware_launch/issues/901>`_)
* feat(autoware_launch): add argument to enable/disable simulation time (`#886 <https://github.com/autowarefoundation/autoware_launch/issues/886>`_)
* refactor(behavior_path_planner): remove unused drivable area parameters (`#883 <https://github.com/autowarefoundation/autoware_launch/issues/883>`_)
* feat(start_planner): allow lane departure check override (`#893 <https://github.com/autowarefoundation/autoware_launch/issues/893>`_)
  new param added
* feat: add is_simulation variable in autoware.launch.xml (`#889 <https://github.com/autowarefoundation/autoware_launch/issues/889>`_)
* feat(avoidance): wait next shift approval until the ego reaches shift length threshold (`#891 <https://github.com/autowarefoundation/autoware_launch/issues/891>`_)
  * feat(avoidance): wait next shift approval until the ego reaches shift length threshold
  * fix(avoidance): param description
  ---------
* feat(rviz): make rviz2 background lighter, lower the contrast (`#887 <https://github.com/autowarefoundation/autoware_launch/issues/887>`_)
* feat(crosswalk): add parameters for occlusion slowdown feature (`#807 <https://github.com/autowarefoundation/autoware_launch/issues/807>`_)
* feat(lane_change): cancel hysteresis (`#844 <https://github.com/autowarefoundation/autoware_launch/issues/844>`_)
  * feat(lane_change): cancel hysteresis
  * reduce the hysteresis value
  ---------
* feat(autoware_launch): set use_sim_time parameter equal to true when … (`#746 <https://github.com/autowarefoundation/autoware_launch/issues/746>`_)
* fix: recovery default parameter (`#882 <https://github.com/autowarefoundation/autoware_launch/issues/882>`_)
* feat(goal_planner): change pull over path candidate priority with soft and hard margins (`#874 <https://github.com/autowarefoundation/autoware_launch/issues/874>`_)
* feat(traffic_light_arbiter): add parameter of signal match validator (`#879 <https://github.com/autowarefoundation/autoware_launch/issues/879>`_)
* feat(strat_planner): add a prepare time for blinker before taking action for approval (`#881 <https://github.com/autowarefoundation/autoware_launch/issues/881>`_)
* feat(avoidance): use free steer policy for safety check (`#865 <https://github.com/autowarefoundation/autoware_launch/issues/865>`_)
* fix(system_error_monitor): changed settings of /autoware/localization/performance_monitoring (`#877 <https://github.com/autowarefoundation/autoware_launch/issues/877>`_)
  Fixed settings of /autoware/localization/performance_monitoring
* fix(start_planner): fix safety_check_time_horizon (`#875 <https://github.com/autowarefoundation/autoware_launch/issues/875>`_)
* chore(start_planner): remove unused parameter (`#878 <https://github.com/autowarefoundation/autoware_launch/issues/878>`_)
* fix(planning_validator): add missing params (`#876 <https://github.com/autowarefoundation/autoware_launch/issues/876>`_)
* feat(tier4_control_launch): disable the trajectory extension (`#866 <https://github.com/autowarefoundation/autoware_launch/issues/866>`_)
  disable the trajectory extending for terminal yaw control
* refactor(blind_spot): find first_conflicting_lane just as intersection module (`#873 <https://github.com/autowarefoundation/autoware_launch/issues/873>`_)
  temp
* feat: define common max_vel (`#870 <https://github.com/autowarefoundation/autoware_launch/issues/870>`_)
* feat(motion_velocity_smoother): increase engage_acceleration (`#736 <https://github.com/autowarefoundation/autoware_launch/issues/736>`_)
  * feat(motion_velocity_smoother): increase engage_acceleration
  * Update autoware_launch/config/planning/scenario_planning/common/motion_velocity_smoother/motion_velocity_smoother.param.yaml
* fix(localization): add ar tag based localizer param (`#871 <https://github.com/autowarefoundation/autoware_launch/issues/871>`_)
  Added ar_tag_based_localizer.param.yaml
* chore(crosswalk): change LATER param (`#868 <https://github.com/autowarefoundation/autoware_launch/issues/868>`_)
  crosswalk/change-LATER-param
* feat(planning_simulator): use fit_target=vector_map in planning_simulator (`#859 <https://github.com/autowarefoundation/autoware_launch/issues/859>`_)
  * Added fit_target
  * Fixed arg name
  ---------
* feat(goal_planne): check objects within the area between ego edge and boudary of pull_over_lanes (`#867 <https://github.com/autowarefoundation/autoware_launch/issues/867>`_)
* fix(log-messages): reduce excessive log messages (`#760 <https://github.com/autowarefoundation/autoware_launch/issues/760>`_)
* fix(avoidance): tuning shiftable ratio & deviation param (`#869 <https://github.com/autowarefoundation/autoware_launch/issues/869>`_)
* chore(radar_object_tracker): move radar object tracker param to yaml (`#838 <https://github.com/autowarefoundation/autoware_launch/issues/838>`_)
  chore: move radar object tracker param to yaml
* feat(pid_longitudinal_controller): adjust slope compensation parameters (`#585 <https://github.com/autowarefoundation/autoware_launch/issues/585>`_)
* feat(map based prediction, crosswalk)!: transplantation of pedestrians' behavior prediction against green signal (`#860 <https://github.com/autowarefoundation/autoware_launch/issues/860>`_)
  pedestrians' intention estimation feature against the green signal
* fix(autoware_launch): remove use_pointcloud_container flag completely (`#864 <https://github.com/autowarefoundation/autoware_launch/issues/864>`_)
* chore(intersection): target type param (`#851 <https://github.com/autowarefoundation/autoware_launch/issues/851>`_)
* feat: remove use_pointcloud_container (`#806 <https://github.com/autowarefoundation/autoware_launch/issues/806>`_)
  * feat!: remove use_pointcloud_container
  * style(pre-commit): autofix
  * remove unnecessary files
  * revert: revert change in declaration of sample vehicle and sensor_kit
  ---------
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
* feat(start/goal_planner): remove unused param and update time horizon for goal planner's safety check (`#863 <https://github.com/autowarefoundation/autoware_launch/issues/863>`_)
  * remove unused param
  * update safety check time horizon
  ---------
* chore(ndt_scan_matcher): rename config path (`#854 <https://github.com/autowarefoundation/autoware_launch/issues/854>`_)
  * chore(ndt_scan_matcher): rename config path
  * rename path
  * style(pre-commit): autofix
  ---------
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
* refactor(rviz): update the class name and turn signal color (`#855 <https://github.com/autowarefoundation/autoware_launch/issues/855>`_)
* feat(intersection): use different expected deceleration for bike/car (`#852 <https://github.com/autowarefoundation/autoware_launch/issues/852>`_)
* chore(planning/control/vehicle): declare ROS params in yaml files (`#833 <https://github.com/autowarefoundation/autoware_launch/issues/833>`_)
  * update yaml
* chore(map): rework parameters of map  (`#843 <https://github.com/autowarefoundation/autoware_launch/issues/843>`_)
  * Added reference to launch parameters to yaml files of map/
  * style(pre-commit): autofix
  ---------
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
* fix(lidar_centerpoint): remove build_only param from param.yaml (`#856 <https://github.com/autowarefoundation/autoware_launch/issues/856>`_)
* refactor(pose_initializer): rework parameters (`#853 <https://github.com/autowarefoundation/autoware_launch/issues/853>`_)
* feat(traffic_light_recognition): add tlr args in tier4_perception_component.launch.xml (`#840 <https://github.com/autowarefoundation/autoware_launch/issues/840>`_)
  * feat(traffic_light_recognition): add tlr args in tier4_perception_component.launch.xml
  * fix dfault value of fusion_only to false
  * fix arg passing way
  ---------
* feat(behavior_path_sampling_planner): add sampling based planner to behavior path planner (`#810 <https://github.com/autowarefoundation/autoware_launch/issues/810>`_)
  * Add sampling based planner params
  * update keep_last param
  * change priority of sampling based planner
  * Set parameters for frenet planner
  * changes for testing
  * change curvature weight for testing
  * tuning params
  * tuning
  * for integ w/ other modules
  * add support for soft constraints weight reconfig
  * rebase
  * temp
  * update default params
  * Tune params
  * Set defaults back to normal
  * fix name of ablc
  * formatting fix
  * set verbose to false
  ---------
* refactor(map_tf_generator): rework parameters (`#835 <https://github.com/autowarefoundation/autoware_launch/issues/835>`_)
* fix(pointpainting): update parameter (`#850 <https://github.com/autowarefoundation/autoware_launch/issues/850>`_)
* chore(lidar_centerpoint): rework parameters (`#822 <https://github.com/autowarefoundation/autoware_launch/issues/822>`_)
  * chore(lidar_centerpoint): use config
  * fix: remove build_only param
  ---------
  Co-authored-by: Kenzo Lobos Tsunekawa <kenzo.lobos@tier4.jp>
* refactor(ekf_localizer): rework parameters (`#847 <https://github.com/autowarefoundation/autoware_launch/issues/847>`_)
  refactor: Add the classification names to yaml file
* feat(obstacle_cruise_planner): yield function for ocp (`#837 <https://github.com/autowarefoundation/autoware_launch/issues/837>`_)
  * add params for yield
  * param name change
  * add params
  * refactoring
  * fix typo, tuning
  * update parameters
  * delete unused param
  * set cruise planner as default for testing
  * add param for stopped obj speed threshold
  * change back param
  * set default false
  ---------
* fix(planning_launch): align parameters to real vehicle (`#848 <https://github.com/autowarefoundation/autoware_launch/issues/848>`_)
  update param
* feat(map_based_prediction): consider crosswalks signals (`#849 <https://github.com/autowarefoundation/autoware_launch/issues/849>`_)
  add param
* chore(image_projection_based_fusion): rework parameters (`#824 <https://github.com/autowarefoundation/autoware_launch/issues/824>`_)
  chore(image_projection_based_fusion): use config
* feat: update rviz splash and vehicle UI display (`#836 <https://github.com/autowarefoundation/autoware_launch/issues/836>`_)
* feat(detection): add container option (`#834 <https://github.com/autowarefoundation/autoware_launch/issues/834>`_)
  feat: use pointcloud_container
* chore(twist2accel): rework parameters (`#842 <https://github.com/autowarefoundation/autoware_launch/issues/842>`_)
  Added twist2accel.param.yaml
* refactor(ndt_scan_matcher): hierarchize parameters (`#830 <https://github.com/autowarefoundation/autoware_launch/issues/830>`_)
  * refactor(ndt_scan_matcher): hierarchize parameters
  * add new lines
  ---------
* fix(autoware_launch): add config file (`#829 <https://github.com/autowarefoundation/autoware_launch/issues/829>`_)
  * fix(autoware_launch): add config file
  * style(pre-commit): autofix
  ---------
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
* refactor(map_projection_loader): rework parameters (`#839 <https://github.com/autowarefoundation/autoware_launch/issues/839>`_)
  * Added launch argument map_projection_loader_param_path to tier4_map_component.launch.xml
  Copied map_projection_loader.launch.xml from universe
  * style(pre-commit): autofix
  ---------
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
* chore(object_velocity_splitter): rework parameters (`#820 <https://github.com/autowarefoundation/autoware_launch/issues/820>`_)
  chore(object_velocity_splitter): add config
* feat(autoware_launch): set default vehicle/sensor models to sample ones (`#768 <https://github.com/autowarefoundation/autoware_launch/issues/768>`_)
* chore(ground_segmentation): add default params (`#831 <https://github.com/autowarefoundation/autoware_launch/issues/831>`_)
  Co-authored-by: kminoda <44218668+kminoda@users.noreply.github.com>
* feat(start_planner): add collision check distances for shift and geometric pull out (`#832 <https://github.com/autowarefoundation/autoware_launch/issues/832>`_)
  * Add collision check distances for shift and geometric pull out
  ---------
* refactor(tier4_map_lcomponent): use map.launch.xml instead of map.launch.py (`#826 <https://github.com/autowarefoundation/autoware_launch/issues/826>`_)
* fix(tracking_object_merger): fix bug and rework parameters (`#823 <https://github.com/autowarefoundation/autoware_launch/issues/823>`_)
  fix(tracking_object_merger): fix bug and use param file
* refactor(ndt_scan_matcher): rename de-grounded (`#827 <https://github.com/autowarefoundation/autoware_launch/issues/827>`_)
  * refactor(ndt_scan_matcher): rename de-grounded
  * fix value
  ---------
* chore(object_range_splitter): rework parameters (`#821 <https://github.com/autowarefoundation/autoware_launch/issues/821>`_)
  * chore(object_range_splitter): add config
  * revert change
  ---------
* feat(intersection): publish and visualize the reason for dangerous situation to blame past detection fault retrospectively (`#828 <https://github.com/autowarefoundation/autoware_launch/issues/828>`_)
* fix(avoidance): change return dead line param (`#814 <https://github.com/autowarefoundation/autoware_launch/issues/814>`_)
* feat(avoidance): add new flag to use freespace in avoidance module (`#818 <https://github.com/autowarefoundation/autoware_launch/issues/818>`_)
* refactor(system_error_monitor): rename localization_accuracy (`#605 <https://github.com/autowarefoundation/autoware_launch/issues/605>`_)
  refactor: Rename localization_accuracy
  to localization_error_ellipse
* fix(tracking_object_merger): fix unknown is not associated problem (`#825 <https://github.com/autowarefoundation/autoware_launch/issues/825>`_)
  fix: unknown is not associated problem
* feat(crosswalk)!: improve stuck prevention on crosswalk (`#816 <https://github.com/autowarefoundation/autoware_launch/issues/816>`_)
  * change a param definition
* feat(start_planner): change collision_check_distance_from_end to shorten back distance (`#757 <https://github.com/autowarefoundation/autoware_launch/issues/757>`_)" (`#813 <https://github.com/autowarefoundation/autoware_launch/issues/813>`_)
  Revert "feat(start_planner): revert change collision_check_distance_from_end to shorten back distance (`#757 <https://github.com/autowarefoundation/autoware_launch/issues/757>`_)"
  This reverts commit 96f2f18d23ba829804415135b241065ecf53b13d.
* fix(ndt_scan_matcher): fix type of critical_upper_bound_exe_time_ms (`#819 <https://github.com/autowarefoundation/autoware_launch/issues/819>`_)
  * fix type
  * fix order
  ---------
* fix(avoidance): decrease velocity threshold for avoidance target objects (`#817 <https://github.com/autowarefoundation/autoware_launch/issues/817>`_)
* fix(vehicle_launch): add raw_vehicle_cmd_converter parameter file (`#812 <https://github.com/autowarefoundation/autoware_launch/issues/812>`_)
* chore(detection_by_tracker): organize parameter structure (`#811 <https://github.com/autowarefoundation/autoware_launch/issues/811>`_)
* refactor(run_out): reorganize the parameter (`#784 <https://github.com/autowarefoundation/autoware_launch/issues/784>`_)
  * chore(run_out): reorganize the parameter
  * style(pre-commit): autofix
  ---------
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
* chore(intersection): align param to robotaxi (`#809 <https://github.com/autowarefoundation/autoware_launch/issues/809>`_)
* feat(goal_planner): expand pull over lanes for detection area of path generation collision check (`#808 <https://github.com/autowarefoundation/autoware_launch/issues/808>`_)
* chore(pointcloud_container): move glog_component to autoware_launch (`#805 <https://github.com/autowarefoundation/autoware_launch/issues/805>`_)
* feat(planning): add enable_all_modules_auto_mode argument to launch files for planning modules (`#798 <https://github.com/autowarefoundation/autoware_launch/issues/798>`_)
  * Add auto mode setting for all modules
* chore(planning): change params to vehicle tested values (`#797 <https://github.com/autowarefoundation/autoware_launch/issues/797>`_)
  change params to vehicle tested values
* feat(map_based_prediction): use acc for map prediction (`#788 <https://github.com/autowarefoundation/autoware_launch/issues/788>`_)
  * add param to toggle on and off acc consideration
  * add params
  * set default to true for evaluator testing
  * set back to false default
  ---------
* feat: always separate lidar preprocessing from pointcloud_container (`#796 <https://github.com/autowarefoundation/autoware_launch/issues/796>`_)
  * feat!: replace use_pointcloud_container
  * change default value
  * remove from planning
  * revert: revert change in planning.launch
  * revert: revert rename of use_pointcloud_container
  * revert: revert pointcloud_container launch
  * style(pre-commit): autofix
  * feat: move glog to pointcloud_container.launch.py
  * revert: revert unnecessary change
  * revert: revert glog porting
  * fix: fix comment in localization launch
  * style(pre-commit): autofix
  * remove pointcloud_container_name from localization launcher
  ---------
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
* fix(surround_obstacle_checker): use xx1 params (`#800 <https://github.com/autowarefoundation/autoware_launch/issues/800>`_)
* chore(pointcloud_container): fix output log from screen to both (`#804 <https://github.com/autowarefoundation/autoware_launch/issues/804>`_)
* feat(start_planner): enable shift path lane departure check (`#803 <https://github.com/autowarefoundation/autoware_launch/issues/803>`_)
  enable shift path lane departure check in start planner
* feat(intersection): consider 1st/2nd pass judge line (`#792 <https://github.com/autowarefoundation/autoware_launch/issues/792>`_)
* chore: update roi_cluster_fusion default param (`#802 <https://github.com/autowarefoundation/autoware_launch/issues/802>`_)
* feat(rviz): add marker to show bpp internal state (`#801 <https://github.com/autowarefoundation/autoware_launch/issues/801>`_)
* fix(AbLC): fix module name inconsistency (`#795 <https://github.com/autowarefoundation/autoware_launch/issues/795>`_)
* feat(avoidance/goal_planner): execute avoidance and pull over simultaneously (`#782 <https://github.com/autowarefoundation/autoware_launch/issues/782>`_)
* fix: change the way to disable surround_obstacle_checker (`#794 <https://github.com/autowarefoundation/autoware_launch/issues/794>`_)
* fix(image_projection_based_fusion): add image_porojection_based_fusion params (`#789 <https://github.com/autowarefoundation/autoware_launch/issues/789>`_)
  add image_porojection_based_fusion params
* feat(mpc): add parameter for debug trajectory publisher (`#790 <https://github.com/autowarefoundation/autoware_launch/issues/790>`_)
* refactor(ekf_localizer): add Simple1DFilter params to parameter file (`#710 <https://github.com/autowarefoundation/autoware_launch/issues/710>`_)
  * feat(ekf_localizer): Add Simple1DFilter params to parameter file
  * Update autoware_launch/config/localization/ekf_localizer.param.yaml
  ---------
  Co-authored-by: Kento Yabuuchi <moc.liamg.8y8@gmail.com>
* feat(start_planner): shorten max backward distance  (`#734 <https://github.com/autowarefoundation/autoware_launch/issues/734>`_)
  Update start_planner.param.yaml
* feat(multi_object_tracker): fix typo in param name and change default value (`#785 <https://github.com/autowarefoundation/autoware_launch/issues/785>`_)
  * fix(multi_object_tracker): fix typo in param name
  * feat: update default param
  ---------
* chore(crosswalk): change params (`#780 <https://github.com/autowarefoundation/autoware_launch/issues/780>`_)
  * change params
* fix(intersection): fix bugs (`#781 <https://github.com/autowarefoundation/autoware_launch/issues/781>`_)
* feat(start_planner): define collision check margin as list (`#770 <https://github.com/autowarefoundation/autoware_launch/issues/770>`_)
  * Update collision check margins in start planner configuration
  ---------
* feat(ekf_localizer): add publish_tf arg (`#772 <https://github.com/autowarefoundation/autoware_launch/issues/772>`_)
* feat(start_planner): keep distance against front objects (`#766 <https://github.com/autowarefoundation/autoware_launch/issues/766>`_)
  Add collision check margin from front object
* feat: tune parameters for optimization path planning (`#774 <https://github.com/autowarefoundation/autoware_launch/issues/774>`_)
  * feat: tune parameters for optimization path planning
  * disable warm start
  * Update autoware_launch/config/planning/scenario_planning/lane_driving/motion_planning/obstacle_avoidance_planner/obstacle_avoidance_planner.param.yaml
  ---------
* feat(surround_obstacle_checker): disable the surround obstacle checker (`#685 <https://github.com/autowarefoundation/autoware_launch/issues/685>`_)
* fix(rviz): hide traffic light regulatory element id (`#777 <https://github.com/autowarefoundation/autoware_launch/issues/777>`_)
* feat(behavior_velocity_planner): add new 'dynamic_obstacle_stop' module (`#730 <https://github.com/autowarefoundation/autoware_launch/issues/730>`_)
* fix(pointpainting): update parameter structure (`#778 <https://github.com/autowarefoundation/autoware_launch/issues/778>`_)
  * fix(pointpainting): update parameter structure
  * update roi_sync.param.yaml
  * style(pre-commit): autofix
  ---------
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
* fix(lane_change): set lane change parameters to real vehicle environment (`#761 <https://github.com/autowarefoundation/autoware_launch/issues/761>`_)
* feat: tune dynamic avoidance parameters with the real vehicle (`#775 <https://github.com/autowarefoundation/autoware_launch/issues/775>`_)
* feat: add behavior_output_path_interval in behavior_velocity_planner (`#773 <https://github.com/autowarefoundation/autoware_launch/issues/773>`_)
* refactor(ndt_scan_matcher, map_loader): remove unused parameters (`#769 <https://github.com/autowarefoundation/autoware_launch/issues/769>`_)
  Removed unused parameters
* feat: add parameters to avoid sudden steering in dynamic avoidance (`#756 <https://github.com/autowarefoundation/autoware_launch/issues/756>`_)
* feat(autoware_launch): update traffic light recognition models (`#752 <https://github.com/autowarefoundation/autoware_launch/issues/752>`_)
  * fix: update model names
  * fix: argument name
  * Update autoware_launch/launch/components/tier4_perception_component.launch.xml
  * fix: model name
  * fix: add model path
  * Update autoware_launch/launch/components/tier4_perception_component.launch.xml
  ---------
  Co-authored-by: Yusuke Muramatsu <yukke42@users.noreply.github.com>
  Co-authored-by: Shunsuke Miura <37187849+miursh@users.noreply.github.com>
* feat: make crosswalk decision more aggressive towards the real world's driving (`#762 <https://github.com/autowarefoundation/autoware_launch/issues/762>`_)
* feat(map_based_prediction): map prediction with acc constraints (`#759 <https://github.com/autowarefoundation/autoware_launch/issues/759>`_)
  * Add params for acceleration constraints for map_based_prediction
  * add new param
  * tune params
  * add parameter to switch on and off constraints check
  * improve comment
  ---------
* feat(obstacle_stop_planner): change stop distance after goal (`#758 <https://github.com/autowarefoundation/autoware_launch/issues/758>`_)
  * feat(obstacle_stop/cruise): change stop distance after goal
  * Update autoware_launch/config/planning/scenario_planning/lane_driving/motion_planning/obstacle_cruise_planner/obstacle_cruise_planner.param.yaml
  ---------
* fix(avoidance): apply params used in xx1 vehicle (`#751 <https://github.com/autowarefoundation/autoware_launch/issues/751>`_)
  * fix(avoidance): use xx1 params
  * fix(avoidance): expand safety check polygon lateral margin
  ---------
* refactor(behavior_path_planner): rename parameter "extra_arc_length" to "arc_length_range" (`#755 <https://github.com/autowarefoundation/autoware_launch/issues/755>`_)
* feat(start_planner): revert change collision_check_distance_from_end to shorten back distance (`#757 <https://github.com/autowarefoundation/autoware_launch/issues/757>`_)
  Revert "feat(start_planner): change collision_check_distance_from_end to shorten back distance"
  This reverts commit 680fb05e9bebdff6cf2c9734631cb4e949d7c499.
* feat(start_planner): change collision_check_distance_from_end to shorten back distance  ## Description (`#754 <https://github.com/autowarefoundation/autoware_launch/issues/754>`_)
  feat(start_planner): change collision_check_distance_from_end to shorten back distance
* feat: add stopped_object.max_object_vel in dynamic_avoidance (`#753 <https://github.com/autowarefoundation/autoware_launch/issues/753>`_)
* revert: "fix(avoidance): shorten the parameter (`#745 <https://github.com/autowarefoundation/autoware_launch/issues/745>`_)" (`#750 <https://github.com/autowarefoundation/autoware_launch/issues/750>`_)
  revert "fix(avoidance): shorten the parameter (`#745 <https://github.com/autowarefoundation/autoware_launch/issues/745>`_)"
  This reverts commit 024254c82f2687deddfadba716afe0f2b8a3a03c.
* feat: run_out does not plan to stop when there is enough time for stopping (`#749 <https://github.com/autowarefoundation/autoware_launch/issues/749>`_)
* feat(avoidance): enable avoidance for objects that stop longer time than thresh (`#743 <https://github.com/autowarefoundation/autoware_launch/issues/743>`_)
* feat(avoidance): enable avoidance for objects that stop longer time than thresh (`#747 <https://github.com/autowarefoundation/autoware_launch/issues/747>`_)
* feat(intersection): disable stuck detection against private lane (`#744 <https://github.com/autowarefoundation/autoware_launch/issues/744>`_)
* fix(avoidance): shorten the parameter (`#745 <https://github.com/autowarefoundation/autoware_launch/issues/745>`_)
* feat(blind_spot): consider opposite adjacent lane for wrong vehicles (`#695 <https://github.com/autowarefoundation/autoware_launch/issues/695>`_)
* feat(run_out)!: ignore the collision points on crosswalk (`#737 <https://github.com/autowarefoundation/autoware_launch/issues/737>`_)
  suppress on crosswalk
* fix(intersection): generate yield stuck detect area from multiple lanes (`#742 <https://github.com/autowarefoundation/autoware_launch/issues/742>`_)
* refactor(autoware_launch): remove use_experimental_lane_change_function (`#741 <https://github.com/autowarefoundation/autoware_launch/issues/741>`_)
* chore(image_projection_based_fusion): add param (`#739 <https://github.com/autowarefoundation/autoware_launch/issues/739>`_)
  * chore(image_projection_based_fusion): add param
  * style(pre-commit): autofix
  ---------
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
* feat(crosswalk): ignore predicted path going across the crosswalk (`#733 <https://github.com/autowarefoundation/autoware_launch/issues/733>`_)
* feat(rviz_config): add objects of interest marker (`#738 <https://github.com/autowarefoundation/autoware_launch/issues/738>`_)
* refactor(localization_component_launch): rename lidar topic (`#722 <https://github.com/autowarefoundation/autoware_launch/issues/722>`_)
  rename lidar topic
  Co-authored-by: yamato-ando <Yamato ANDO>
* feat(multi_object_tracker): update tracker parameter yaml  (`#732 <https://github.com/autowarefoundation/autoware_launch/issues/732>`_)
  * add multi_object_tracker node param
  * add additional node parameters for future update
  * style(pre-commit): autofix
  * fix default value
  * update simulator component launch
  * feat: update multi_object_tracker node param
  ---------
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
* fix(crosswalk): fix inappropriate sync (`#731 <https://github.com/autowarefoundation/autoware_launch/issues/731>`_)
  fix in-appropriate sync
* chore(crosswalk): sync a config file to the univese one (`#729 <https://github.com/autowarefoundation/autoware_launch/issues/729>`_)
  update comment, by sync to the univese one
* feat(obstacle_cruise_planner): add slow down acc and jerk params (`#726 <https://github.com/autowarefoundation/autoware_launch/issues/726>`_)
  Add slow down acc and jerk params
* fix(traffic_light): stop if the traffic light signal timed out (`#727 <https://github.com/autowarefoundation/autoware_launch/issues/727>`_)
* fix(multi_object_tracker): fix psim launcher related to tracking launch changes (`#724 <https://github.com/autowarefoundation/autoware_launch/issues/724>`_)
  * add multi_object_tracker node param
  * add additional node parameters for future update
  * style(pre-commit): autofix
  * fix default value
  * update simulator component launch
  ---------
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
* feat(start_planner): add surround moving obstacle check (`#723 <https://github.com/autowarefoundation/autoware_launch/issues/723>`_)
  update start_planner.param.yaml
* feat: add polygon_generation_method in dynamic_avoidance (`#715 <https://github.com/autowarefoundation/autoware_launch/issues/715>`_)
* feat(rviz): fix perception debug topics in Rviz (`#721 <https://github.com/autowarefoundation/autoware_launch/issues/721>`_)
  fix perception debug topics in Rviz
* feat(component_state_monitor): monitor traffic light recognition output (`#720 <https://github.com/autowarefoundation/autoware_launch/issues/720>`_)
* refactor(start_planner): refactor debug and safety check logic (`#719 <https://github.com/autowarefoundation/autoware_launch/issues/719>`_)
  refactor(start_planner): refactor debug parameters
  This commit removes the `verbose` parameter under `start_planner` and introduces a new `debug` section. The newly added `debug` section includes a `print_debug_info` parameter, set to false by default. This change provides a more structured way to handle debugging configurations for the start planner.
* refactor(multi_object_tracker): add multi_object_tracker node param (`#718 <https://github.com/autowarefoundation/autoware_launch/issues/718>`_)
  * add multi_object_tracker node param
  * add additional node parameters for future update
  * style(pre-commit): autofix
  * fix default value
  ---------
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
* chore(obstacle_cruise)!: remove unused params (`#716 <https://github.com/autowarefoundation/autoware_launch/issues/716>`_)
  chore!: remove unused params
* refactor(intersection): rename param, update doc (`#708 <https://github.com/autowarefoundation/autoware_launch/issues/708>`_)
* feat(avoidance): keep stopping until all shift lines are registered (`#699 <https://github.com/autowarefoundation/autoware_launch/issues/699>`_)
* fix(crosswalk): don't stop in front of the crosswalk if vehicle stuck in intersection (`#714 <https://github.com/autowarefoundation/autoware_launch/issues/714>`_)
* feat: add several min_object_vel in dynamic_avoidance (`#707 <https://github.com/autowarefoundation/autoware_launch/issues/707>`_)
* feat: disable obstacle avoidance debug marker for optimization (`#711 <https://github.com/autowarefoundation/autoware_launch/issues/711>`_)
  feat: disable obstacle avoidance debug marker
* feat(avoidance): configurable object type for safety check (`#709 <https://github.com/autowarefoundation/autoware_launch/issues/709>`_)
* feat: add parameters for the front object decision in dynamic_avoidance module (`#706 <https://github.com/autowarefoundation/autoware_launch/issues/706>`_)
* feat(pid_longitudinal_controller): error integration on vehicle takeoff (`#698 <https://github.com/autowarefoundation/autoware_launch/issues/698>`_)
  * add parameter for PID integration time threshold
  * add param to enable or disable low speed error integration
  ---------
* feat(run_out): add parameter to decide whether to use the object's velocity (`#704 <https://github.com/autowarefoundation/autoware_launch/issues/704>`_)
* feat(goal_planenr): enable safety check (`#705 <https://github.com/autowarefoundation/autoware_launch/issues/705>`_)
* feat(goal_planner): safer safety checker (`#701 <https://github.com/autowarefoundation/autoware_launch/issues/701>`_)
  * feat(goal_planner): safer safety checker
  fix
  fix
  fix
  fix
  * disable safety check
  ---------
* feat(map_based_prediction): consider only routable neighbours for lane change (`#703 <https://github.com/autowarefoundation/autoware_launch/issues/703>`_)
* feat(avoidance): add new parameter for target object filtering (`#668 <https://github.com/autowarefoundation/autoware_launch/issues/668>`_)
* feat(start_planner): enable safety check for start planner (`#702 <https://github.com/autowarefoundation/autoware_launch/issues/702>`_)
  Enable safety check feature for start planner
* feat(goal_planner): add time hysteresis to keep unsafe (`#700 <https://github.com/autowarefoundation/autoware_launch/issues/700>`_)
  feat(goal_planner): add tiem hysteresis to keep unsafe
* fix(start_planner): disbale verbose flag to false in start_planner.param.yaml (`#696 <https://github.com/autowarefoundation/autoware_launch/issues/696>`_)
  Change verbose flag to false in start_planner.param.yaml
* refactor(start_planner): add verbose parameter for debug print (`#693 <https://github.com/autowarefoundation/autoware_launch/issues/693>`_)
  Add verbose option to start planner parameters
* feat(component_state_monitor): monitor pose_estimator output (`#692 <https://github.com/autowarefoundation/autoware_launch/issues/692>`_)
* fix(lane_change): regulate at the traffic light (`#673 <https://github.com/autowarefoundation/autoware_launch/issues/673>`_)
* feat: enable and tune drivable area expansion (`#689 <https://github.com/autowarefoundation/autoware_launch/issues/689>`_)
  enable drivable area expansion
* feat: lane_departure_checker with curbstones (`#687 <https://github.com/autowarefoundation/autoware_launch/issues/687>`_)
* feat(out_of_lane): more stable decisions (`#612 <https://github.com/autowarefoundation/autoware_launch/issues/612>`_)
* fix(avoidance): prevent sudden steering at yield maneuver (`#690 <https://github.com/autowarefoundation/autoware_launch/issues/690>`_)
* feat(radar_object_clustering): move radar object clustering params to autoware_launch (`#672 <https://github.com/autowarefoundation/autoware_launch/issues/672>`_)
  * add radar object clustering param path
  * style(pre-commit): autofix
  ---------
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
* fix(detected_object_validation): add param (`#669 <https://github.com/autowarefoundation/autoware_launch/issues/669>`_)
  * fix(detected_object_validation): add param
  * fix: change to 2d validator use
  ---------
* feat: add motion_velocity_smoother's virtual wall in rviz (`#684 <https://github.com/autowarefoundation/autoware_launch/issues/684>`_)
* feat(duplicated_node_checker): enable duplicated_node_checker in simulation (`#686 <https://github.com/autowarefoundation/autoware_launch/issues/686>`_)
  Enable duplicated node checker in planning
  simulation
* feat(obstacle_cruise_planner): use obstacle velocity based obstacle parameters (`#681 <https://github.com/autowarefoundation/autoware_launch/issues/681>`_)
  * add moving parameters for testing
  * param tuning for tests
  * wip params for velocity-based obscruise planner
  * add different values for debugging
  * set hysteresis-based obstacle moving classification
  * set params to match previous values
  * eliminate pedestrian mention
  ---------
* fix(perception): add detection_by_tracker param file (`#676 <https://github.com/autowarefoundation/autoware_launch/issues/676>`_)
* feat: enable the run_out module (`#683 <https://github.com/autowarefoundation/autoware_launch/issues/683>`_)
  feat: enable run_out
* refactor(launch): add new option to select planning preset (`#680 <https://github.com/autowarefoundation/autoware_launch/issues/680>`_)
  * chore(config): remove behavior launch modules
  * refactor(config): add preset yaml file
  * refactor(launch): add new option to select planning preset
  * refactor(config): remove unused params
  ---------
* feat(intersection): rectify initial accel/velocity profile in ego velocity profile (`#677 <https://github.com/autowarefoundation/autoware_launch/issues/677>`_)
  feat(intersection): rectify smoothed velocity
* chore(tier4_planning_launch): add costmap generator config (`#679 <https://github.com/autowarefoundation/autoware_launch/issues/679>`_)
* feat(ndt_scan_matcher): add parameters of real-time covariance estimation (`#643 <https://github.com/autowarefoundation/autoware_launch/issues/643>`_)
  * add covariance_estimation
  * fix
  * fix
  * fix: parameter names and explanations
  * fix: A parameter that I forgot to add
  * fix: remove white space
  * fix: remove white spaces
  ---------
* feat(ekf_localizer, system_error_monitor): system_error_monitor handles ekf diags (`#674 <https://github.com/autowarefoundation/autoware_launch/issues/674>`_)
  * fix(ekf_localizer): change default parameter for no update count
  * update system_error_monitor
  ---------
* chore(goal_planner): fix typo (`#670 <https://github.com/autowarefoundation/autoware_launch/issues/670>`_)
* refactor(planning): update args name (`#675 <https://github.com/autowarefoundation/autoware_launch/issues/675>`_)
* refactor(planning): update args name (`#671 <https://github.com/autowarefoundation/autoware_launch/issues/671>`_)
* feat(vehicle_cmd_gate): improve debug marker activation (`#659 <https://github.com/autowarefoundation/autoware_launch/issues/659>`_)
  * feat(vehicle_cmd_gate): add filter activated threshold
  * feat: update parameter
  * feat: add condition for filtering marker
  ---------
* feat(intersection): add ttc debug plotter (`#666 <https://github.com/autowarefoundation/autoware_launch/issues/666>`_)
* feat(avoidance): return original lane by red traffic light (`#663 <https://github.com/autowarefoundation/autoware_launch/issues/663>`_)
* refactor(avoidance): cleanup force avoidance params (`#667 <https://github.com/autowarefoundation/autoware_launch/issues/667>`_)
* feat(radar_object_tracker): update and add parameter about radar_object_tracker for far away perecption (`#658 <https://github.com/autowarefoundation/autoware_launch/issues/658>`_)
  update and add parameter about radar_object_tracker for far away detection
* feat(behavior_path_planner): add traffic light recognition timeout threshold (`#662 <https://github.com/autowarefoundation/autoware_launch/issues/662>`_)
* fix(lane_change): separate backward buffer for blocking object (`#661 <https://github.com/autowarefoundation/autoware_launch/issues/661>`_)
* fix(rviz2): update traffic_light/debug/rois topic name (`#642 <https://github.com/autowarefoundation/autoware_launch/issues/642>`_)
* feat(AEB): implement parameterized prediction time horizon and interval (`#657 <https://github.com/autowarefoundation/autoware_launch/issues/657>`_)
* chore(rviz): hide interseciton area polygon as default (`#655 <https://github.com/autowarefoundation/autoware_launch/issues/655>`_)
* feat: add use_conservative_buffer_longitudinal in avoidance (`#656 <https://github.com/autowarefoundation/autoware_launch/issues/656>`_)
* feat(intersection): check path margin for overshoot vehicles on red light (`#654 <https://github.com/autowarefoundation/autoware_launch/issues/654>`_)
* feat(rviz): add sensing/perception debug topics (`#653 <https://github.com/autowarefoundation/autoware_launch/issues/653>`_)
  * add perception debug topics
  * add sensing debug topics
  * change color of dbt to orange
  ---------
* perf(elastic_band_smoother): increase lateral replan threshold (`#652 <https://github.com/autowarefoundation/autoware_launch/issues/652>`_)
* feat(behavior_velocity_run_out): ignore momentary detection caused by false positive (`#647 <https://github.com/autowarefoundation/autoware_launch/issues/647>`_)
  * feat(behavior_velocity_run_out): ignore momentary detection caused by false positive
  * style(pre-commit): autofix
  ---------
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
* feat(duplicated_node_checker): add duplicated node names to msg (`#651 <https://github.com/autowarefoundation/autoware_launch/issues/651>`_)
  add duplicated node names to msg
* feat(intersection): use own max acc/jerk param (`#650 <https://github.com/autowarefoundation/autoware_launch/issues/650>`_)
* feat(duplicated_node_checker): disable duplicated_node_checker (`#649 <https://github.com/autowarefoundation/autoware_launch/issues/649>`_)
  * disable duplicated_node_checker
  * enable duplicated_node_checker
  ---------
* feat(intersection): timeout static occlusion with traffic light (`#646 <https://github.com/autowarefoundation/autoware_launch/issues/646>`_)
* feat(map_based_prediction): enable to control lateral path convergence time (`#637 <https://github.com/autowarefoundation/autoware_launch/issues/637>`_)
  enable to control lateral path convergence time
* feat(planner_manager): limit iteration number by parameter (`#645 <https://github.com/autowarefoundation/autoware_launch/issues/645>`_)
* feat(avoidance): add paramenters for dynamic detection area (`#634 <https://github.com/autowarefoundation/autoware_launch/issues/634>`_)
* fix(intersection): lower state_transit_margi_time to 0 (`#638 <https://github.com/autowarefoundation/autoware_launch/issues/638>`_)
* fix(drivable_area_expansion): disable by default (`#639 <https://github.com/autowarefoundation/autoware_launch/issues/639>`_)
* fix(tier4_simulator_component): add lacked param path (`#640 <https://github.com/autowarefoundation/autoware_launch/issues/640>`_)
* feat(lane_change): change stuck velocity to 0.5 (`#636 <https://github.com/autowarefoundation/autoware_launch/issues/636>`_)
* feat(behavior_path_planner): curvature based drivable area expansion (`#632 <https://github.com/autowarefoundation/autoware_launch/issues/632>`_)
  * Modify parameters for curvature based dynamic drivable area expansion
  * Add parameter to enable/disable printing the runtime
  * Add smoothing.extra_arc_length param
  ---------
* add tracking object merger for long range radar sensor (`#627 <https://github.com/autowarefoundation/autoware_launch/issues/627>`_)
  * add tracking object merger paramters
  * fix typo
  ---------
* feat(lane_change): add rss paramas for stuck (`#633 <https://github.com/autowarefoundation/autoware_launch/issues/633>`_)
* feat(intersection): ignore decelerating vehicle on amber traffic light (`#635 <https://github.com/autowarefoundation/autoware_launch/issues/635>`_)
  * feat(intersection): ignore decelerating vehicle on amber traffic light
  * tuning
  ---------
* feat(duplicated_node_checker): add duplicated_node_checker (`#631 <https://github.com/autowarefoundation/autoware_launch/issues/631>`_)
  * add duplicated_node_checker
  * add arguments for duplicated node checker, required by new PR on the universe
  * fix type
  * add config inside launch
  * style(pre-commit): autofix
  * the default should be set to 10
  ---------
  Co-authored-by: Owen-Liuyuxuan <uken.ryu@tier4.jp>
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
* feat(autoware_launch): add yield_stuck.distance_thr in intersection (`#628 <https://github.com/autowarefoundation/autoware_launch/issues/628>`_)
  * feat(autoware_launch): add yield_stuck.distance_thr in intersection
  * use turn_direction
  * update param
  ---------
* feat(ndt_scan_matcher): added a new parameter "n_startup_trials" (`#602 <https://github.com/autowarefoundation/autoware_launch/issues/602>`_)
  * Added a new parameter "n_startup_trials"
  * Changed default `n_startup_trials` to 20
  ---------
* chore(intersection): parameterize stuck vehicle detection turn_direction (`#630 <https://github.com/autowarefoundation/autoware_launch/issues/630>`_)
* feat(avoidance): check if the avoidance path is in drivable area (`#584 <https://github.com/autowarefoundation/autoware_launch/issues/584>`_)
  * feat(avoidance): check if the avoidance path is in drivable area
  * refactor(avoidance): remove unused param
  ---------
* feat(rtc_auto_mode_manager): eliminate rtc auto mode manager (`#625 <https://github.com/autowarefoundation/autoware_launch/issues/625>`_)
  * disable RTC
  * remove rtc auto mode manager
  * fix file name
  ---------
* feat(intersection): yield initially on green light (`#623 <https://github.com/autowarefoundation/autoware_launch/issues/623>`_)
* feat(lane_change): separate execution and cancel safety check param (`#626 <https://github.com/autowarefoundation/autoware_launch/issues/626>`_)
* feat(obstacle_cruise_planner): obstacle type dependent slow down for obstacle cruise planner param change (`#621 <https://github.com/autowarefoundation/autoware_launch/issues/621>`_)
  * set obstacle type dependant params
  * Set obstacle cruise planner as default to test changes
  * Change back testing parameters to default
  ---------
* feat(intersection)!: disable the exception behavior in the private areas (`#622 <https://github.com/autowarefoundation/autoware_launch/issues/622>`_)
  feat: add enabling param for the private areas
* refactor(avoidance): use safety check parameter struct (`#617 <https://github.com/autowarefoundation/autoware_launch/issues/617>`_)
* fix: add param file for obstacle pointcloud based validator (`#606 <https://github.com/autowarefoundation/autoware_launch/issues/606>`_)
  * fix: add param file for obstacle pointcloud based validator
  * fix: tier4_perception launch
  ---------
* feat(intersection): ignore occlusion beyond high curvature point (`#619 <https://github.com/autowarefoundation/autoware_launch/issues/619>`_)
* perf(ndt_scan_matcher): changed default `initial_estimate_particles_num` to 200 (`#618 <https://github.com/autowarefoundation/autoware_launch/issues/618>`_)
  Changed initial_estimate_particles_num to 200
* feat(intersection): aggressively peek into attention area if traffic light does not exist (`#611 <https://github.com/autowarefoundation/autoware_launch/issues/611>`_)
* feat(autoware_launch): dynamic timeout for no intention to walk decision in crosswalk (`#610 <https://github.com/autowarefoundation/autoware_launch/issues/610>`_)
  * feat(autoware_launch): dynamic timeout for no intention to walk decision in crosswalk
  * update config
  * revert a parg of config
  ---------
* fix(autoware_launch): improve stop decision in out_of_lane (`#615 <https://github.com/autowarefoundation/autoware_launch/issues/615>`_)
* feat(localization_error_monitor): update parameter (`#614 <https://github.com/autowarefoundation/autoware_launch/issues/614>`_)
* feat(behavior_path_planner): update rss param (`#604 <https://github.com/autowarefoundation/autoware_launch/issues/604>`_)
  update param
* feat(lane_change):  expand target lanes for object filtering (`#601 <https://github.com/autowarefoundation/autoware_launch/issues/601>`_)
* feat(autoware_launch): add predicted_path_checker package (`#385 <https://github.com/autowarefoundation/autoware_launch/issues/385>`_)
* refactor(ndt_scan_matcher): modified ndt_scan_matcher.param.yaml to match with the one in universe (`#596 <https://github.com/autowarefoundation/autoware_launch/issues/596>`_)
  Modified ndt_scan_matcher.param.yaml to match with the one in universe
* feat(intersection): use planned velocity from upstream modules (`#597 <https://github.com/autowarefoundation/autoware_launch/issues/597>`_)
* feat(goal_planner): prioritize goals before objects to avoid (`#594 <https://github.com/autowarefoundation/autoware_launch/issues/594>`_)
  * feat(goal_planner): extend goal search are
  * feat(goal_planner): prioritize goals before objects to avoid
  ---------
* feat(start_planner): change th_distance_to_middle_of_the_road 0.5 (`#599 <https://github.com/autowarefoundation/autoware_launch/issues/599>`_)
* feat(start_planner): enable divide_pull_out_path (`#600 <https://github.com/autowarefoundation/autoware_launch/issues/600>`_)
* feat(goal_planner): change minimum_request_length 0.0 (`#598 <https://github.com/autowarefoundation/autoware_launch/issues/598>`_)
* feat(goal_planner): extend goal search area (`#592 <https://github.com/autowarefoundation/autoware_launch/issues/592>`_)
  feat(goal_planner): extend goal search are
* feat(autoware_launch): add max_obstacle_vel in dynamic_avoidance (`#595 <https://github.com/autowarefoundation/autoware_launch/issues/595>`_)
* feat: add system monitor param file for awsim (`#568 <https://github.com/autowarefoundation/autoware_launch/issues/568>`_)
  * feat: add system monitor param file for awsim
  * feat: use system_error_monitor.awsim.param in e2e_simulator.launch
  ---------
* feat(autoware_launch): move dynamic_avoidance last (`#593 <https://github.com/autowarefoundation/autoware_launch/issues/593>`_)
* feat(ndt_scan_matcher): adding exe time parameter (`#559 <https://github.com/autowarefoundation/autoware_launch/issues/559>`_)
  add critical_upper_bound_exe_time_ms for ndt
* feat(lane_change): enable lane change in crosswalk/intersection if ego vehicle gets stuck (`#590 <https://github.com/autowarefoundation/autoware_launch/issues/590>`_)
* feat(goal_planner): sort goal candidates priority by weighted distance (`#591 <https://github.com/autowarefoundation/autoware_launch/issues/591>`_)
* feat(intersection): ensure-temporal-stop-before-upcoming-lane (`#578 <https://github.com/autowarefoundation/autoware_launch/issues/578>`_)
* feat(obstacle_cruise_planner): add parameters for a new feature (`#581 <https://github.com/autowarefoundation/autoware_launch/issues/581>`_)
  * feat: add parameters for the feature "cosider-current-ego-pose"
  * set the params to be merged.
  use stop planner as cruise planner type (conventional setting)
  polygon expansion in obstacle_cruise_planner is true
  ---------
* feat(autoware_launch): add traffic protected level for amber color in intersection (`#588 <https://github.com/autowarefoundation/autoware_launch/issues/588>`_)
  * feat(autoware_launch): add traffic protected level for amber color in intersection
  * update
  * update
  ---------
* feat(autoware_launch): add stop_distance_threshold in merge_from_private (`#587 <https://github.com/autowarefoundation/autoware_launch/issues/587>`_)
* feat(autoware_launch): add check_footprint_inside_lanes in mission_planner (`#589 <https://github.com/autowarefoundation/autoware_launch/issues/589>`_)
* chore(motion_velocity_smoother): add enable curve filtering param (`#580 <https://github.com/autowarefoundation/autoware_launch/issues/580>`_)
* fix(start/goal_planner): resample path and make params (`#586 <https://github.com/autowarefoundation/autoware_launch/issues/586>`_)
* fix(motion_velocity_smoother): change curvature calculation distance parameter (`#556 <https://github.com/autowarefoundation/autoware_launch/issues/556>`_)
* feat(planning_launch): add config for regulate lane change (`#582 <https://github.com/autowarefoundation/autoware_launch/issues/582>`_)
* refactor(ndt_scan_matcher): match ndt_scan_matcher.param.yaml (`#583 <https://github.com/autowarefoundation/autoware_launch/issues/583>`_)
  * Added ndt_base_link parameter in ndt_scan_matcher.param.yaml
  Deleted neighborhood_search_mathod paramter in ndt_scan_matcher.param.yaml
  * Copy-pasted the ndt_scan_matcher.param.yaml from universe
  * Correct spelling
  ---------
* feat: prevent start planner execution in the middle of the road (`#579 <https://github.com/autowarefoundation/autoware_launch/issues/579>`_)
  * start planner:new param: dist th to middle of road
  * refactor param order
  ---------
* feat(vehicle_cmd_gate): add steering angle and rate filter (`#576 <https://github.com/autowarefoundation/autoware_launch/issues/576>`_)
* feat(perception): add data_path argument to launch file (`#577 <https://github.com/autowarefoundation/autoware_launch/issues/577>`_)
  * feat(perception): add data_path argument to launch file
  * style(pre-commit): autofix
  ---------
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
* feat(ekf_localizer): ignore dead band of velocity sensor (`#574 <https://github.com/autowarefoundation/autoware_launch/issues/574>`_)
  * feat(ekf_localizer): ignore dead band of velocity sensor
  * style(pre-commit): autofix
  ---------
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
* feat(ekf_lolicazer): add diagnostics parameters (`#554 <https://github.com/autowarefoundation/autoware_launch/issues/554>`_)
  * feat(ekf_lolicazer): add diagnostics parameters
  * remote param
  ---------
  Co-authored-by: yamato-ando <Yamato ANDO>
* fix(autoware_launch): add radar lanelet filter parameter (`#566 <https://github.com/autowarefoundation/autoware_launch/issues/566>`_)
* refactor(perception): rearrange clustering pipeline parameters (`#567 <https://github.com/autowarefoundation/autoware_launch/issues/567>`_)
  * fix: use downsample before compare map
  * fix: remove downsample after compare map
  * fix: add low range crop filter param
  * chore: refactor
  * chore: typo
  ---------
* feat(behavior_path_planner): set param ignore_object_velocity_threshold (`#573 <https://github.com/autowarefoundation/autoware_launch/issues/573>`_)
  set param ignore_object_velocity_threshold
* fix(behavior_path_planner): change safety check default disable (`#572 <https://github.com/autowarefoundation/autoware_launch/issues/572>`_)
  * change safety check default disable
  * add warning message
  ---------
* feat(behavior_path_planner): update start_goal_planner's parameter (`#571 <https://github.com/autowarefoundation/autoware_launch/issues/571>`_)
  update start_goal_planner's parameter
* fix(behavior_path_planner): define hysteresis_factor_expand_rate (`#569 <https://github.com/autowarefoundation/autoware_launch/issues/569>`_)
  * hysteresis_factor_expand_rate
  * style(pre-commit): autofix
  * add hysteresis_factor_expand_rate in SafetyCheckParams
  * delete setting files
  * revert unnecessary change
  ---------
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
* feat(goal_planner): add options of occupancy grid map to use only for goal search (`#563 <https://github.com/autowarefoundation/autoware_launch/issues/563>`_)
* feat(tier4_system_rviz_plugin): add package (`#564 <https://github.com/autowarefoundation/autoware_launch/issues/564>`_)
  * feat(tier4_system_rviz_plugin): add package
  * fix
  ---------
* chore(localization_error_monitor): update default parameter (`#565 <https://github.com/autowarefoundation/autoware_launch/issues/565>`_)
* feat(goal_planner): use only static objects in pull over lanes to path generation (`#562 <https://github.com/autowarefoundation/autoware_launch/issues/562>`_)
* feat(autoware_launch): add approaching stop on curve in obstacle cruise planner (`#560 <https://github.com/autowarefoundation/autoware_launch/issues/560>`_)
  * feat(autoware_launch): add approaching stop on curve in obstacle cruise planner
  * update config
  ---------
* fix(autoware.rviz): remove initial_pose_button_panel (`#561 <https://github.com/autowarefoundation/autoware_launch/issues/561>`_)
  Co-authored-by: yamato-ando <Yamato ANDO>
* feat(autoware_launch): additional margin parameters in surround obstacle checker (`#557 <https://github.com/autowarefoundation/autoware_launch/issues/557>`_)
  feat(autoware_launch): additional margin parameters in surround ostacle checker
* feat(ndt_scan_matcher): add param lidar_topic_timeout_sec (`#540 <https://github.com/autowarefoundation/autoware_launch/issues/540>`_)
  Co-authored-by: yamato-ando <Yamato ANDO>
* feat(lane_departure_checker): add border types to check (`#549 <https://github.com/autowarefoundation/autoware_launch/issues/549>`_)
  update lane_departure_checker.param.yaml
* feat: add traffic light recogition namespace to e2e sim launch (`#555 <https://github.com/autowarefoundation/autoware_launch/issues/555>`_)
* feat(operation_transition_mannager): add param enable_engage_on_driving (`#553 <https://github.com/autowarefoundation/autoware_launch/issues/553>`_)
* feat(goal_planner): do not use minimum_request_length for fixed goal … (`#546 <https://github.com/autowarefoundation/autoware_launch/issues/546>`_)
  feat(goal_planner): do not use minimum_request_length for fixed goal planner
* feat(goal_planner): set ignore_distance_from_lane_start 0.0 (`#552 <https://github.com/autowarefoundation/autoware_launch/issues/552>`_)
* feat(behavior_path_planner): add safety check against dynamic objects for start/goal planner (`#550 <https://github.com/autowarefoundation/autoware_launch/issues/550>`_)
  add params for safety check
* feat(out_of_lane): add min_assumed_velocity parameter (`#548 <https://github.com/autowarefoundation/autoware_launch/issues/548>`_)
* feat(behavior_path_planner): add path resampling interval param (`#522 <https://github.com/autowarefoundation/autoware_launch/issues/522>`_)
* feat(interface): add new option `keep_last` (`#543 <https://github.com/autowarefoundation/autoware_launch/issues/543>`_)
  feat(planner_manager): keep last module
* chore(rviz_config): add localization debug config (`#544 <https://github.com/autowarefoundation/autoware_launch/issues/544>`_)
* fix(control_validator): default false for publishing diag and display terminal (`#545 <https://github.com/autowarefoundation/autoware_launch/issues/545>`_)
  default false for publishing diag and display terminal
* feat(autoware_launch): enable emergency handling when resource monitoring state becomes error (`#542 <https://github.com/autowarefoundation/autoware_launch/issues/542>`_)
* chore(rviz_config): add debug marker group (`#541 <https://github.com/autowarefoundation/autoware_launch/issues/541>`_)
* feat(autoware_launch): remove polygon_generation_method from dynamic_avoidance (`#539 <https://github.com/autowarefoundation/autoware_launch/issues/539>`_)
* feat(intersection): strict definition of stuck vehicle detection area (`#532 <https://github.com/autowarefoundation/autoware_launch/issues/532>`_)
* feat(intersection): suppress intersection occlusion chattering (`#533 <https://github.com/autowarefoundation/autoware_launch/issues/533>`_)
* feat(autoware_launch): add no stop decision parameters in crosswalk (`#537 <https://github.com/autowarefoundation/autoware_launch/issues/537>`_)
* chore: add default args for TLR models (`#538 <https://github.com/autowarefoundation/autoware_launch/issues/538>`_)
* feat(autoware_launch): add max_crosswalk_user_delta_yaw_threshold_for_lanelet in map_based_prediction (`#536 <https://github.com/autowarefoundation/autoware_launch/issues/536>`_)
* feat(autoware_launch): add min_longitudinal_polygon_margin and use object_path_base in dynamic_avoidance (`#534 <https://github.com/autowarefoundation/autoware_launch/issues/534>`_)
* feat(autoware_launch): set larger max_area for pedestrian with umbrella (`#535 <https://github.com/autowarefoundation/autoware_launch/issues/535>`_)
  set larger max_area for pedestrian with umbrella
* feat(avoidance): flexible avoidance safety check param (`#529 <https://github.com/autowarefoundation/autoware_launch/issues/529>`_)
* feat(avoidance): add time series hysteresis (`#530 <https://github.com/autowarefoundation/autoware_launch/issues/530>`_)
* refactor(map_based_prediction): update prediction yaml file (`#531 <https://github.com/autowarefoundation/autoware_launch/issues/531>`_)
  update prediction yaml file
* feat(autoware_launch): add suppress_sudden_obstacle_stop in obstacle_cruise_planner (`#525 <https://github.com/autowarefoundation/autoware_launch/issues/525>`_)
  * feat(autoware_launch): add suppress_sudden_obstacle_stop in obstacle_cruise_planner
  * update
  ---------
* fix(smoother): fix smoother jerk weight params (`#528 <https://github.com/autowarefoundation/autoware_launch/issues/528>`_)
* fix(avoidance): avoidance shift line processing bug (`#527 <https://github.com/autowarefoundation/autoware_launch/issues/527>`_)
  fix(avoidance): safety check chattering
* feat(autoware_launch): add hold stop threshold in obstacle_cruise_planner (`#524 <https://github.com/autowarefoundation/autoware_launch/issues/524>`_)
  * feat(autoware_launch): add hold stop threshold in obstacle_cruise_planner
  * update
  ---------
* refactor(safety_check): use safety check common param struct (`#526 <https://github.com/autowarefoundation/autoware_launch/issues/526>`_)
* fix(freespace_planner): fixed by adding parameters of RRTstar algorithm (`#517 <https://github.com/autowarefoundation/autoware_launch/issues/517>`_)
  * fix(freespace_planner): add parameters of RRTstar algorithm
  * style(pre-commit): autofix
  ---------
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
  Co-authored-by: Kyoichi Sugahara <kyoichi.sugahara@tier4.jp>
* feat(vehicle_cmd_gate): adaptive filter limit (`#510 <https://github.com/autowarefoundation/autoware_launch/issues/510>`_)
  * feat(vehicle_cmd_gate): adaptive filter limit
  * update
  ---------
* fix(autoware_launch): correct prediction_time_horizon default value (`#523 <https://github.com/autowarefoundation/autoware_launch/issues/523>`_)
  correct prediction_time_horizon value
* feat(goal_planner): add extra front margin for collision check considering stopping distance (`#520 <https://github.com/autowarefoundation/autoware_launch/issues/520>`_)
  * feat(goal_planner): add extra front margin for collision check considering stopping distance
  * object_recognition_collision_check_margin: 0.6
  * rename args and params
  * add comments
  ---------
* feat(autoware_launch): add use_raw_remote_control_command_input argument (`#460 <https://github.com/autowarefoundation/autoware_launch/issues/460>`_)
  * update external_cmd_converter
  * update default value
  * update argument name
  * move enable_cmd_limit_filter argument to param file
  * update enable_cmd_filter default value
  ---------
* feat(rviz): respawn rviz (`#518 <https://github.com/autowarefoundation/autoware_launch/issues/518>`_)
* feat(merge_from_private): use separate param (`#521 <https://github.com/autowarefoundation/autoware_launch/issues/521>`_)
* feat(start_planner): support freespace pull out (`#514 <https://github.com/autowarefoundation/autoware_launch/issues/514>`_)
* fix(ekf_localizer): fix parameter first capital letter (`#519 <https://github.com/autowarefoundation/autoware_launch/issues/519>`_)
* chore(tier4_simulator_component): add traffic light arbiter param path (`#502 <https://github.com/autowarefoundation/autoware_launch/issues/502>`_)
* feat(control_validator): measure predicted path deviation from trajectory (`#509 <https://github.com/autowarefoundation/autoware_launch/issues/509>`_)
  * update launcher
  * add config and modify launch file
  * style(pre-commit): autofix
  * feat(lane_departure_checker): add road_border departure checker (`#511 <https://github.com/autowarefoundation/autoware_launch/issues/511>`_)
  add param
  * feat(system_error_monitor): check lateral deviation in sim (`#516 <https://github.com/autowarefoundation/autoware_launch/issues/516>`_)
  * restore rviz config change
  ---------
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
  Co-authored-by: Satoshi OTA <44889564+satoshi-ota@users.noreply.github.com>
* feat(avoidance): reduce road shoulder margin if lateral distance is not enough to avoid (`#513 <https://github.com/autowarefoundation/autoware_launch/issues/513>`_)
* feat(autoware_launch): add enable_pub_extra_debug_marker in obstacle_avoidance_planner (`#512 <https://github.com/autowarefoundation/autoware_launch/issues/512>`_)
* feat(system_error_monitor): check lateral deviation in sim (`#516 <https://github.com/autowarefoundation/autoware_launch/issues/516>`_)
* feat(lane_departure_checker): add road_border departure checker (`#511 <https://github.com/autowarefoundation/autoware_launch/issues/511>`_)
  add param
* feat(autoware_launch): add polygon_generation_method in dynamic_avoidance (`#508 <https://github.com/autowarefoundation/autoware_launch/issues/508>`_)
* feat(avoidance): make it selectable avoidance policy (`#505 <https://github.com/autowarefoundation/autoware_launch/issues/505>`_)
* feat(map_projection_loader): add map_projection_loader (`#483 <https://github.com/autowarefoundation/autoware_launch/issues/483>`_)
  feat(map_loader): add map_projection_loader
* feat(tier4_perception_launch): update pointpainting param (`#506 <https://github.com/autowarefoundation/autoware_launch/issues/506>`_)
* feat(autoware_launch): use hatched road markings in dynamic avoidance (`#504 <https://github.com/autowarefoundation/autoware_launch/issues/504>`_)
  * feat(autoware_launch): add use_hatched_road_markings in dynamic_avoidance
  * add parameters
  * update
  ---------
* feat(avoidance): enable avoidance cancel (`#476 <https://github.com/autowarefoundation/autoware_launch/issues/476>`_)
* feat(autoware_launch): update dynamic_avoidance parameters (`#503 <https://github.com/autowarefoundation/autoware_launch/issues/503>`_)
* feat(routing_no_drivable_lane_when_module_enabled): add solution for routing no_drivable_lane only when module enabled (`#457 <https://github.com/autowarefoundation/autoware_launch/issues/457>`_)
  * feat(routing_no_drivable_lane_when_module_enabled): add proposed solution
  * style(pre-commit): autofix
  * feat(routing_no_drivable_lane_when_module_enabled): improving comments regarding new parameter
  ---------
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
* refactor(autoware_launch): add an option for filtering and validation (`#479 <https://github.com/autowarefoundation/autoware_launch/issues/479>`_)
  * init commit
  * style(pre-commit): autofix
  ---------
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
* feat(autoware_launch): add cut out parameters for dynamic avoidance (`#500 <https://github.com/autowarefoundation/autoware_launch/issues/500>`_)
* feat(lane_change): remove an unused parameter (`#501 <https://github.com/autowarefoundation/autoware_launch/issues/501>`_)
* feat(autoware_launch): add successive_num_to_exit_dynamic_avoidance_condition in dynamic_avoidance (`#484 <https://github.com/autowarefoundation/autoware_launch/issues/484>`_)
* feat(rviz): add acceleration meter for debugging which is disabled by default (`#499 <https://github.com/autowarefoundation/autoware_launch/issues/499>`_)
  * feat: add acceleration meter for debugging, disabled by default; https://github.com/autowarefoundation/autoware.universe/pull/4506
  * not directly setting pixel numbers in rviz for display on screen with various resolutions
  ---------
  Co-authored-by: Owen-Liuyuxuan <uken.ryu@tier4.jp>
* feat(start_planner): use stop objects in pull out lanes for collision check (`#498 <https://github.com/autowarefoundation/autoware_launch/issues/498>`_)
* fix(object_merger): separate GIoU (`#497 <https://github.com/autowarefoundation/autoware_launch/issues/497>`_)
* feat(autoware_launch): add gradable pass margin in crosswalk (`#496 <https://github.com/autowarefoundation/autoware_launch/issues/496>`_)
* feat(autoware_launch): add pass juge line parameter in crosswalk (`#495 <https://github.com/autowarefoundation/autoware_launch/issues/495>`_)
* feat(autoware_launch): add chattering suppression margin in crosswalk (`#494 <https://github.com/autowarefoundation/autoware_launch/issues/494>`_)
* refactor(traffic_light_arbiter): add traffic_light_arbiter param file (`#489 <https://github.com/autowarefoundation/autoware_launch/issues/489>`_)
  Co-authored-by: Kenzo Lobos Tsunekawa <kenzo.lobos@tier4.jp>
* feat(out_of_lane): add param for the min confidence of a predicted path (`#440 <https://github.com/autowarefoundation/autoware_launch/issues/440>`_)
  Co-authored-by: Takayuki Murooka <takayuki5168@gmail.com>
* feat(autoware_launch): add option of disable_yield_for_new_stopped_object in crosswalk (`#491 <https://github.com/autowarefoundation/autoware_launch/issues/491>`_)
* feat(intersection_occlusion): ignore occlusion behind parked vehicles on the attention lane (`#492 <https://github.com/autowarefoundation/autoware_launch/issues/492>`_)
* feat(autoware_launch): add acc/jerk parameters for stuck vehicle detection in crosswalk (`#487 <https://github.com/autowarefoundation/autoware_launch/issues/487>`_)
* feat(avoidance): add parameter to configurate avoidance return point (`#493 <https://github.com/autowarefoundation/autoware_launch/issues/493>`_)
* feat(intersection): extract occlusion contour as polygon (`#485 <https://github.com/autowarefoundation/autoware_launch/issues/485>`_)
* refactor(tier4_localization_component): input_pointcloud param added (`#480 <https://github.com/autowarefoundation/autoware_launch/issues/480>`_)
  * refactor(tier4_localization_component): input_pointcloud param added
  * style(pre-commit): autofix
  ---------
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
* feat(autoware_launch): add cut_in_object.min_lon_offset_ego_to_object in dynamic avoidance (`#481 <https://github.com/autowarefoundation/autoware_launch/issues/481>`_)
  * feat(autoware_launch): add cut_in_object.min_lon_offset_ego_to_object in dynamic avoidance
  * update
  ---------
* feat(autoware_launch): add enable_debug_info for dynamic_avoidance (`#478 <https://github.com/autowarefoundation/autoware_launch/issues/478>`_)
* feat(path_smoother): add parameters for the replan checker (`#482 <https://github.com/autowarefoundation/autoware_launch/issues/482>`_)
* refactor(avoidance): use common safety checker (`#477 <https://github.com/autowarefoundation/autoware_launch/issues/477>`_)
* refactor(planning_launch): clean stop line parameters (`#475 <https://github.com/autowarefoundation/autoware_launch/issues/475>`_)
* feat(autoware_launch): add dynamic avoidance parameters (`#474 <https://github.com/autowarefoundation/autoware_launch/issues/474>`_)
* perf(path_sampler): tune lateral_deviation_weight for more stable planning (`#455 <https://github.com/autowarefoundation/autoware_launch/issues/455>`_)
  Set lateral_deviation_weight 0.1 -> 1.0
* fix(compare_map_segmentation): add param for skip lower neighbor points comparision option (`#447 <https://github.com/autowarefoundation/autoware_launch/issues/447>`_)
  * fix(compare_map_segmentation): add param for check lower neighbor points option
  * fix: update param for reduce z distance threshold
  * fix: change param type
  ---------
* feat(autoware_launch): update pointpainting param (`#473 <https://github.com/autowarefoundation/autoware_launch/issues/473>`_)
* fix(autoware_launch): rename pull over to goal planner (`#472 <https://github.com/autowarefoundation/autoware_launch/issues/472>`_)
* add radar tracker parameter files and settings (`#470 <https://github.com/autowarefoundation/autoware_launch/issues/470>`_)
  * add radar tracker parameter files and settings
  * style(pre-commit): autofix
  ---------
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
* refactor(autoware_launch): add lidar models params (`#450 <https://github.com/autowarefoundation/autoware_launch/issues/450>`_)
  * init commit
  * add centerpoint params
  * remove dupplicated commits
  ---------
* feat(avoidance_by_lc): make it possible to configurate flexibly (`#469 <https://github.com/autowarefoundation/autoware_launch/issues/469>`_)
  feat(avodiance_by_lc): make it possible to configurate flexibly
* feat(behavior_velocity_planner): add flag to enable auto mode without rtc_auto_mode_manager (`#435 <https://github.com/autowarefoundation/autoware_launch/issues/435>`_)
  * add enable_rtc param
  * fix typo
  * revert change of rviz config
  * revert change of rviz config
  ---------
* refactor(avoidance): parameterize magic number (`#426 <https://github.com/autowarefoundation/autoware_launch/issues/426>`_)
* feat(avoidance): flexible avoidance path generation (`#454 <https://github.com/autowarefoundation/autoware_launch/issues/454>`_)
* refactor(autoware_launch): add object_merger param files (`#464 <https://github.com/autowarefoundation/autoware_launch/issues/464>`_)
  init commit
* chore(autoware_launch): zero margin for outside the drivable area (`#468 <https://github.com/autowarefoundation/autoware_launch/issues/468>`_)
* fix(avoidance): update config to prevent unconfortable deceleration (`#466 <https://github.com/autowarefoundation/autoware_launch/issues/466>`_)
* refactor(autoware_launch): rename crosswalk/walkway parameters (`#459 <https://github.com/autowarefoundation/autoware_launch/issues/459>`_)
* refactor(autoware_launch): add walkway param yaml (`#458 <https://github.com/autowarefoundation/autoware_launch/issues/458>`_)
* fix(tier4_simulator_component): add missing argument (`#465 <https://github.com/autowarefoundation/autoware_launch/issues/465>`_)
  update simulator launch component
* fix(obstacle_avoidance_planner): adding missing functionality for stop margin due to out of drivable area (`#438 <https://github.com/autowarefoundation/autoware_launch/issues/438>`_)
* refactor(autoware_launch): add map_based_prediction param file (`#463 <https://github.com/autowarefoundation/autoware_launch/issues/463>`_)
  init commit
* feat(planning_launch): add safety check flags for lane change (`#462 <https://github.com/autowarefoundation/autoware_launch/issues/462>`_)
* feat: use `pose_source` and `twist_source` for selecting localization methods (`#442 <https://github.com/autowarefoundation/autoware_launch/issues/442>`_)
  * feat: add pose and twist sources args for localization
  * removed unnecessary params
  * allow only one source
  * Move configs to tier4_localization_component.launch.xml
  * Remove unnecessary line
  * fix comment
  ---------
* feat(intersection): add behavior for arrow signal (`#456 <https://github.com/autowarefoundation/autoware_launch/issues/456>`_)
* fix(occlusion_spot): add lacking param (`#452 <https://github.com/autowarefoundation/autoware_launch/issues/452>`_)
* refactor(avoidance): update parameter names (`#453 <https://github.com/autowarefoundation/autoware_launch/issues/453>`_)
* refactor(autoware_launch): add euclidean_clustering params (`#445 <https://github.com/autowarefoundation/autoware_launch/issues/445>`_)
  * add euclidean_clustering param
  * style(pre-commit): autofix
  * update and rearrange comments
  * style(pre-commit): autofix
  ---------
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
* feat(autoware_launch): update lane change rviz configuration (`#451 <https://github.com/autowarefoundation/autoware_launch/issues/451>`_)
  * feat(autoware_launch): update autoware rviz
  * update
  ---------
* feat(autoware_launch): add disable_stop_for_yield_cancel in crosswalk (`#449 <https://github.com/autowarefoundation/autoware_launch/issues/449>`_)
  * feat(autoware_launch): add disable_stop_for_yield_cancel in crosswalk
  * update
  ---------
* feat(start_planner): add curvature limit for path generation (`#446 <https://github.com/autowarefoundation/autoware_launch/issues/446>`_)
  * add param for curvature shift start
  * update
  * change param
  * maximum_curvature: 0.07
  ---------
  Co-authored-by: kosuke55 <kosuke.tnp@gmail.com>
* feat(avoidance): enable zebra zone avoidance (`#448 <https://github.com/autowarefoundation/autoware_launch/issues/448>`_)
* feat(avoidance): enable to use intersection area (`#443 <https://github.com/autowarefoundation/autoware_launch/issues/443>`_)
* feat(avoidance): consider acceleration during avoidance maneuver (`#436 <https://github.com/autowarefoundation/autoware_launch/issues/436>`_)
  feat(avoidance): use improved path shifting logic
* refactor(behavior_path_planner): remove unused config files (`#441 <https://github.com/autowarefoundation/autoware_launch/issues/441>`_)
  * refactor(behavior_path_planner): remove unused config files
  * refactor(behavior_path_planner): remove unnecessary code
  ---------
* feat(avoidance): use intersection areas (`#439 <https://github.com/autowarefoundation/autoware_launch/issues/439>`_)
* refactor(obstacle_avoidance_planner): move the elastic band smoothing to a new package (`#420 <https://github.com/autowarefoundation/autoware_launch/issues/420>`_)
* feat(probabilistic_occupancy_grid_map): add projective raytracing option from scan_origin (`#434 <https://github.com/autowarefoundation/autoware_launch/issues/434>`_)
* feat(behavior_path_planner): shorten the wating time of force avoidance (`#437 <https://github.com/autowarefoundation/autoware_launch/issues/437>`_)
* feat(autoware_launch): add min_obj_lat_offset_to_ego_path in dynamic_avoidance (`#427 <https://github.com/autowarefoundation/autoware_launch/issues/427>`_)
* feat(autoware_launch): add intersection param for wrong direction vehicles (`#394 <https://github.com/autowarefoundation/autoware_launch/issues/394>`_)
  add consider_wrong_direction_vehicle param
  Co-authored-by: beyza <bnk@leodrive.ai>
* feat(avoidance): insert slow down speed (`#429 <https://github.com/autowarefoundation/autoware_launch/issues/429>`_)
* fix(avoidance): don't output new candidate path if there is huge offset between the ego and previous output path (`#431 <https://github.com/autowarefoundation/autoware_launch/issues/431>`_)
* feat(avoidance): extend object ignore section (`#433 <https://github.com/autowarefoundation/autoware_launch/issues/433>`_)
  feat(avoidance): increase object ignore section
* feat(start_planner): add option for lane departure (`#432 <https://github.com/autowarefoundation/autoware_launch/issues/432>`_)
* fix(autoware_launch): add missing pose_initializer param (`#430 <https://github.com/autowarefoundation/autoware_launch/issues/430>`_)
* feat(autoware_launch): no slow down against unknown object (`#428 <https://github.com/autowarefoundation/autoware_launch/issues/428>`_)
* feat(avoidance): update avoidance params (`#424 <https://github.com/autowarefoundation/autoware_launch/issues/424>`_)
* fix(behavior_velocity_intersection_module): fix condition of use_stuck_stopline (`#425 <https://github.com/autowarefoundation/autoware_launch/issues/425>`_)
* feat(lane_change): add param for a lateral distance margin where the abort can be performed (`#421 <https://github.com/autowarefoundation/autoware_launch/issues/421>`_)
* refactor(lane_change): add namespace for lane-change-cancel (`#423 <https://github.com/autowarefoundation/autoware_launch/issues/423>`_)
  * refactor(lane_change): add namespace for lane-change-cancel
  * update
  ---------
* refactor(avoidance): rename ununderstandable params (`#422 <https://github.com/autowarefoundation/autoware_launch/issues/422>`_)
* fix(autoware_launch): use experimental lane change function (`#418 <https://github.com/autowarefoundation/autoware_launch/issues/418>`_)
* feat: use vehicle_stop_checker for judging vehicle stop vehicle_cmd_gate (`#417 <https://github.com/autowarefoundation/autoware_launch/issues/417>`_)
* fix(occupancy grid): fix launcher (`#419 <https://github.com/autowarefoundation/autoware_launch/issues/419>`_)
  updated yaml
* feat(planning_launch): add parameters for delaying lane change (`#401 <https://github.com/autowarefoundation/autoware_launch/issues/401>`_)
* feat(avoidance): set additional buffer margin independently (`#412 <https://github.com/autowarefoundation/autoware_launch/issues/412>`_)
* revert: "feat(behavior_path_planner): relax longitudinal_velocity_delta_time" (`#415 <https://github.com/autowarefoundation/autoware_launch/issues/415>`_)
  Revert "feat(behavior_path_planner): relax longitudinal_velocity_delta_time (`#410 <https://github.com/autowarefoundation/autoware_launch/issues/410>`_)"
  This reverts commit e100e566ddae26173e4bc0d1e8aea40022bad658.
* feat(avoidance): enable avoidance for unknown object (`#416 <https://github.com/autowarefoundation/autoware_launch/issues/416>`_)
* feat(autoware_launch): update dynamic avoidance param (`#413 <https://github.com/autowarefoundation/autoware_launch/issues/413>`_)
* chore(probabilistic_occupancy_grid_map): revert map size (`#414 <https://github.com/autowarefoundation/autoware_launch/issues/414>`_)
* refactor(probabilistic_occupancy_grid_map): move param to yaml (`#409 <https://github.com/autowarefoundation/autoware_launch/issues/409>`_)
* feat(avoidance): avoid non car-like object (pedestrian, bicycle, motorcycle) (`#408 <https://github.com/autowarefoundation/autoware_launch/issues/408>`_)
  feat(avoidance): avoid non vehicle object
* feat(behavior_path_planner): relax longitudinal_velocity_delta_time (`#410 <https://github.com/autowarefoundation/autoware_launch/issues/410>`_)
* feat(yabloc): add camera and vector map localization (`#393 <https://github.com/autowarefoundation/autoware_launch/issues/393>`_)
  * add yabloc_localization_component.launch.xml
  * add pose_initializer.logging_simulator.yabloc.param.yaml
  * add yabloc params
  * add graph_segment.param.yaml
  * integrate yabloc_loc_launch into tier_loc_launch
  * fix some config files
  * add conditional branch for yabloc
  * add sample config for yabloc
  * style(pre-commit): autofix
  * removed some obsolete parameters
  * style(pre-commit): autofix
  * removed is_swap_mode from yabloc config
  * rename AbstParaticleFilter in config files
  * fixed typo
  * removed optional param files
  * refactored tier4_loc_comp.launch.xml
  * changed localization_mode option names
  ---------
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
* feat(intersection): timeout stuck vehicle stop in private area (`#406 <https://github.com/autowarefoundation/autoware_launch/issues/406>`_)
* feat(start_planner): add length_ratio_for_turn_signal_deactivation_near_intersection (`#407 <https://github.com/autowarefoundation/autoware_launch/issues/407>`_)
* fix(mpc): relax steering rate limit (`#405 <https://github.com/autowarefoundation/autoware_launch/issues/405>`_)
* feat(start_planner): change lateral acceleration sampling num (`#404 <https://github.com/autowarefoundation/autoware_launch/issues/404>`_)
* feat(start_planner): start with acceleration (`#402 <https://github.com/autowarefoundation/autoware_launch/issues/402>`_)
* feat(avoidance): can set stop/move judge threshold for each object class (`#399 <https://github.com/autowarefoundation/autoware_launch/issues/399>`_)
* feat(planning_launch): add a turn signal deactivation parameter for lane change (`#398 <https://github.com/autowarefoundation/autoware_launch/issues/398>`_)
* feat(avoidance): change object_check_backward_distance from 100m to 10m (`#388 <https://github.com/autowarefoundation/autoware_launch/issues/388>`_)
* feat(avoidance): additional buffer for perception noise (`#373 <https://github.com/autowarefoundation/autoware_launch/issues/373>`_)
  feat(avoidance): additional offset for perception noise
* feat(velocity_smoother): plan from ego velocity on manual mode (`#396 <https://github.com/autowarefoundation/autoware_launch/issues/396>`_)
* feat(rviz): hide crosswalk areas (`#395 <https://github.com/autowarefoundation/autoware_launch/issues/395>`_)
* feat(planning_launch): add a parameter for turn signal activation (`#397 <https://github.com/autowarefoundation/autoware_launch/issues/397>`_)
* refactor: intersection module (`#391 <https://github.com/autowarefoundation/autoware_launch/issues/391>`_)
* fix(autoware_launch): add missing parameter in autonmous emergency braking (`#392 <https://github.com/autowarefoundation/autoware_launch/issues/392>`_)
* feat(behavior_path_planner): add flag to enable auto mode without rtc_auto_mode_manager (`#387 <https://github.com/autowarefoundation/autoware_launch/issues/387>`_)
  * add param
  * enable rtc to false for default auto mode module
  * update
  * set avoidance module enable_rtc as false
  * set all module enable_rtc param as true
  ---------
* refactor(behavior_velocity_planner): load all module parameters (`#389 <https://github.com/autowarefoundation/autoware_launch/issues/389>`_)
  load all module parameters
* refactor(behavior_velocity_planner): update launch and parameter files for plugin (`#369 <https://github.com/autowarefoundation/autoware_launch/issues/369>`_)
  * feat: update parameter files
  * feat: update param name
  * feat: add disabled module as comment
  * feat: use behavior_velocity_config_path
  ---------
* refactor(start_planner): rename pull out to start planner (`#386 <https://github.com/autowarefoundation/autoware_launch/issues/386>`_)
* fix(planning_launch): parameterize scale for lc safety check (`#384 <https://github.com/autowarefoundation/autoware_launch/issues/384>`_)
* feat(avoidance): add option for yield during shifting (`#383 <https://github.com/autowarefoundation/autoware_launch/issues/383>`_)
* feat(autoware_launch): suppress flickering to entry slow down (`#382 <https://github.com/autowarefoundation/autoware_launch/issues/382>`_)
* feat(avoidance): add new parameter (`#374 <https://github.com/autowarefoundation/autoware_launch/issues/374>`_)
* feat(autoware_launch): dynamic steer rate limit in mpc (`#381 <https://github.com/autowarefoundation/autoware_launch/issues/381>`_)
  * feat(autoware_launch): dynamic steer rate limit in mpc
  * update
  * update
  ---------
* fix(trajectory_follower_nodes): mpc_follower does not send proper converged data under low steering rate limit (`#378 <https://github.com/autowarefoundation/autoware_launch/issues/378>`_)
* feat(rtc_auto_mode_manager): delete external_request_lane_change from rtc auto mode manager config (`#379 <https://github.com/autowarefoundation/autoware_launch/issues/379>`_)
* feat(pull_out): support pull out normal lane (`#377 <https://github.com/autowarefoundation/autoware_launch/issues/377>`_)
  * add th_blinker_on_lateral_offset: 1.0
  * minimum_shift_pull_out_distance: 0.0
  * minimum_lateral_jerk: 0.1
  ---------
* fix(tier4_perception_component_launch): add sync_param_path (`#350 <https://github.com/autowarefoundation/autoware_launch/issues/350>`_)
  * fix(tier4_perception_component_launch): add sync_param_path
  * style(pre-commit): autofix
  ---------
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
* feat(lane_change): check lateral offset at lc finish judgement (`#375 <https://github.com/autowarefoundation/autoware_launch/issues/375>`_)
* feat: first draft proposal implementation for handling invalid lanelets (`#235 <https://github.com/autowarefoundation/autoware_launch/issues/235>`_)
  * feat: first draft proposal implementation for handling invalid lanelets
  * style(pre-commit): autofix
  * feat: adding invalid lanelet for rviz visualization
  * feat: fixing review comment
  * feat: changing module name from invalid_lanelet to no_drivable_lane
  * feat: fixing merge conflict mistakes
  * feat: fixing merge conflict mistakes
  * feat: fixing merge conflicts
  * feat: removing rtc handing
  * feat(no_drivable_lane): reflecting refactoring changes to no_drivable_lane
  ---------
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
* refactor(autoware_launch): remove duplicated path in planning launch (`#365 <https://github.com/autowarefoundation/autoware_launch/issues/365>`_)
* feat(behavior_path_planner): relax margin_from_boundary in goal_planner param (`#371 <https://github.com/autowarefoundation/autoware_launch/issues/371>`_)
* fix(intersection): add the flag for intersection_occlusion grid publication (`#372 <https://github.com/autowarefoundation/autoware_launch/issues/372>`_)
* feat(planning_launch): add minimum reroute length (`#370 <https://github.com/autowarefoundation/autoware_launch/issues/370>`_)
* fix(behavior_path_planner): fix lateral distance max threshold (`#368 <https://github.com/autowarefoundation/autoware_launch/issues/368>`_)
* feat(planning_launch): add maximum and minimum longitudinal acceleration for the lane change (`#363 <https://github.com/autowarefoundation/autoware_launch/issues/363>`_)
* feat(vehicle_cmd_gate): add moderate_stop_interface parameter (`#367 <https://github.com/autowarefoundation/autoware_launch/issues/367>`_)
* feat(intersection): add the option to use intersection_area (`#366 <https://github.com/autowarefoundation/autoware_launch/issues/366>`_)
  add option to use intersection_area
* feat(autoware_launch): update obstacle_cruise_planner param (`#364 <https://github.com/autowarefoundation/autoware_launch/issues/364>`_)
  * feat(autoware_launch): update obstacle_cruise_planner param
  * update
  ---------
* feat(autoware_launch): remove min_acc in obstacle_cruise_planner for slow down (`#356 <https://github.com/autowarefoundation/autoware_launch/issues/356>`_)
* feat(autoware_launch): add time margins for dynamic avoidance (`#360 <https://github.com/autowarefoundation/autoware_launch/issues/360>`_)
* feat(avoidance): don't avoid objects around crosswalks (`#362 <https://github.com/autowarefoundation/autoware_launch/issues/362>`_)
* feat(autoware_launch): time margin for slow down (`#359 <https://github.com/autowarefoundation/autoware_launch/issues/359>`_)
* feat(autoware_launch): add hysteresis for slow down decision (`#354 <https://github.com/autowarefoundation/autoware_launch/issues/354>`_)
* feat(intersection): denoise occlusion by morphology open process (`#361 <https://github.com/autowarefoundation/autoware_launch/issues/361>`_)
  parameterize kernel size
* feat(planning_launch): change lane change sampling number (`#358 <https://github.com/autowarefoundation/autoware_launch/issues/358>`_)
* feat(behavior): increase dynamic avoidance module priority (`#355 <https://github.com/autowarefoundation/autoware_launch/issues/355>`_)
* feat(planning_launch): add lateral acceleration map (`#352 <https://github.com/autowarefoundation/autoware_launch/issues/352>`_)
  * feat(planning_launch): add lateral acceleration map
  * udpate
  ---------
* feat(autoware_launch): add lane change abort param (`#351 <https://github.com/autowarefoundation/autoware_launch/issues/351>`_)
  * feat(autoware_launch): add lane change abort param
  * fix(autoware_launch): restore some parameters
  ---------
* feat(path_sampler): add parameter file for the `path_sampler` node (`#322 <https://github.com/autowarefoundation/autoware_launch/issues/322>`_)
* fix(planning_launch): add backward_path_length_parameter (`#346 <https://github.com/autowarefoundation/autoware_launch/issues/346>`_)
* feat(planning_launch): side shift debug marker (`#298 <https://github.com/autowarefoundation/autoware_launch/issues/298>`_)
  * feat(planning_launch): side shift debug marker
  * delete blank line
  * style(pre-commit): autofix
  ---------
  Co-authored-by: kyoichi-sugahara <kyoichi.sugahara@tier4.jp>
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
* refactor(avoidance): remove redundant parameter (`#340 <https://github.com/autowarefoundation/autoware_launch/issues/340>`_)
  * refactor(avoidance): rename parameter
  * fix(avoidance): remove redundant parameter
  ---------
* feat(obstacle_stop_planner): add filtering feature for predicted objects (`#330 <https://github.com/autowarefoundation/autoware_launch/issues/330>`_)
* feat(autoware_launch): add marker of dynamic avoidance (`#345 <https://github.com/autowarefoundation/autoware_launch/issues/345>`_)
  * feat(autoware_launch): add marker of dynamic avoidance
  * update
  ---------
* feat(autoware_launch): add a flag to use hatched road markings for av… (`#337 <https://github.com/autowarefoundation/autoware_launch/issues/337>`_)
  feat(autoware_launch): add a flag to use hatched road markings for avoidance
* chore(autoware_launch): update rviz for hatched road markings (`#343 <https://github.com/autowarefoundation/autoware_launch/issues/343>`_)
  * chore(autoware_launch): update rviz for hatched road markings
  * fix typo
  ---------
* refactor(avoidance): remove hard code params in shift line triming process (`#341 <https://github.com/autowarefoundation/autoware_launch/issues/341>`_)
* fix(behavior): don't show behaivor module's marker (`#342 <https://github.com/autowarefoundation/autoware_launch/issues/342>`_)
  fix(rviz): hide all info markers
* feat(avoidance): hide detail information (`#339 <https://github.com/autowarefoundation/autoware_launch/issues/339>`_)
* refactor(behavior_path_planner): refactoring goal_planner and pull_out params (`#338 <https://github.com/autowarefoundation/autoware_launch/issues/338>`_)
* feat(behavior): add interface in order to publish marker that is always shown in rviz (`#333 <https://github.com/autowarefoundation/autoware_launch/issues/333>`_)
  feat(behavior): output module info markers
* feat(autoware_launch): add min_drivable_width (`#334 <https://github.com/autowarefoundation/autoware_launch/issues/334>`_)
  * feat(autoware_launch): add min_drivable_width
  * update
  ---------
* feat(autoware_launch): avoid oncoming vehicles (`#335 <https://github.com/autowarefoundation/autoware_launch/issues/335>`_)
* perf(mpc_lateral_controller): mpc works more stably (`#336 <https://github.com/autowarefoundation/autoware_launch/issues/336>`_)
* feat(autoware_launch): dynamic obstacle avoidance (`#299 <https://github.com/autowarefoundation/autoware_launch/issues/299>`_)
  * add param
  * update launch
  * update for dynamic avoidance
  * update
  * update
  * disable dynamic avoidance module
  * pre-commit
  ---------
* feat(behavior_velocity_planner::intersection): add intersection occlusion gridmap marker (`#332 <https://github.com/autowarefoundation/autoware_launch/issues/332>`_)
* feat(autoware.rviz): disable right and left bound (`#329 <https://github.com/autowarefoundation/autoware_launch/issues/329>`_)
  * feat(autoware.rviz): disable right and left bound
  * update
  ---------
* feat: add gnss/imu localizer  (`#200 <https://github.com/autowarefoundation/autoware_launch/issues/200>`_)
  * Add gnss_imu_localizar
  * Eagleye parameter update fot sample data
  * Restore perception/planning/control parameters in lsim
  * Change use_gnss_mode
  * Fix spell
  * Fix spell
  * Delete unnecessary white spaces
  * Remove unnecessary trailing spaces
  * Prettier format
  * prettier format
  * clang-format
  * Revert "clang-format"
  This reverts commit 46cd907089e6551a975bcff2f3971679598da24c.
  * Rename GNSS/Lidar localization switching parameters
  * Remove conditional branching by pose_estimatar_mode in system_error_monitor
  * Change launch directory structure
  * Delete unnecessary parameters and files
  * Integrate map4_localization_component1,2
  * Fix comment out in localization launch
  ---------
* refactor(planning_launch): remove duplicated lane change parameter (`#328 <https://github.com/autowarefoundation/autoware_launch/issues/328>`_)
* fix(lanelet2_map_loader): update comment for available projector type (`#327 <https://github.com/autowarefoundation/autoware_launch/issues/327>`_)
* refactor(rviz): fix typo (`#326 <https://github.com/autowarefoundation/autoware_launch/issues/326>`_)
* feat(rviz): add behavior path virtual wall (`#325 <https://github.com/autowarefoundation/autoware_launch/issues/325>`_)
* refactor(behavior_path_planner): rename pull_over to goal_planner (`#313 <https://github.com/autowarefoundation/autoware_launch/issues/313>`_)
  refactor(behavior_path_planenr): renaem pull_over to goal_planenr
* feat(autoware_launch): expand ogm size from 100m to 150m (`#324 <https://github.com/autowarefoundation/autoware_launch/issues/324>`_)
  expand ogm size from 100m to 150m
* chore(autoware_launch): tune intersection parameters (`#323 <https://github.com/autowarefoundation/autoware_launch/issues/323>`_)
* refactor(planning_launch): remove minimum prepare length (`#319 <https://github.com/autowarefoundation/autoware_launch/issues/319>`_)
* feat(behavior_path_planner): run avoidance and pull out simultaneously (`#321 <https://github.com/autowarefoundation/autoware_launch/issues/321>`_)
* feat(behavior_velocity_planner::intersection): add parameter for occlusion peeking offset (`#320 <https://github.com/autowarefoundation/autoware_launch/issues/320>`_)
* refactor(planning_launch): use common params for lane change (`#317 <https://github.com/autowarefoundation/autoware_launch/issues/317>`_)
* fix(autoware_launch): old architecture lane change path in rviz (`#316 <https://github.com/autowarefoundation/autoware_launch/issues/316>`_)
* feat(intersection): add flag to enable creep towards intersection occlusion (`#315 <https://github.com/autowarefoundation/autoware_launch/issues/315>`_)
* feat(behavior_velocity_planner::intersection): add occlusion detection feature (`#305 <https://github.com/autowarefoundation/autoware_launch/issues/305>`_)
  * migrated
  * fixed param
  * remove some params
  * organized param
  * disable occlusion feature off by default
  ---------
* feat(autoware_launch): make drivable area expansion parameters common (`#310 <https://github.com/autowarefoundation/autoware_launch/issues/310>`_)
* chore(autoware_launch): make backward detection length for avoidance longer (`#312 <https://github.com/autowarefoundation/autoware_launch/issues/312>`_)
  * chore(autoware_launch): make backward detection length for avoidance longer
  * make longer
  ---------
* feat(obstacle_avoidance_planner): replan when forward path shape changes (`#309 <https://github.com/autowarefoundation/autoware_launch/issues/309>`_)
  * feat(obstacle_avoidance_planner): replan when forward path shape changes
  * update
  ---------
* feat(autoware_launch): add time to fix reference points's boundary width (`#311 <https://github.com/autowarefoundation/autoware_launch/issues/311>`_)
* chore(autoware_launch): visualize thin predicted trajectory on rviz (`#314 <https://github.com/autowarefoundation/autoware_launch/issues/314>`_)
* refactor(planning_launch): remove minimum lane changing length (`#308 <https://github.com/autowarefoundation/autoware_launch/issues/308>`_)
  * refactor(planning_launch): remove minimum lane changing length
  * fix
  ---------
* feat(tier4_simulator_launch): add use_baselink_z option for dummy_perception_publisher (`#304 <https://github.com/autowarefoundation/autoware_launch/issues/304>`_)
* fix(planning_launch): change minimum prepare length (`#307 <https://github.com/autowarefoundation/autoware_launch/issues/307>`_)
* feat(planning_launch): add reroute safety check parameter (`#306 <https://github.com/autowarefoundation/autoware_launch/issues/306>`_)
* refactor(occupancy_grid_map): add occupancy_grid_map method/param var to launcher (`#294 <https://github.com/autowarefoundation/autoware_launch/issues/294>`_)
  add occcupancy_grid_map method/param var to launcher and use those ones in autoware_launch by default
* feat(autoware_launch): visualization for slow down (`#303 <https://github.com/autowarefoundation/autoware_launch/issues/303>`_)
* feat(behavior_path_planner): move lane_following_params to behavior path params (`#302 <https://github.com/autowarefoundation/autoware_launch/issues/302>`_)
* fix: compare map filter param (`#291 <https://github.com/autowarefoundation/autoware_launch/issues/291>`_)
* feat(behavior_path_planner): pull over support road_lane and right_hand_traffic (`#300 <https://github.com/autowarefoundation/autoware_launch/issues/300>`_)
* feat(obstacle_cruise_planner): implement slow down planner (`#288 <https://github.com/autowarefoundation/autoware_launch/issues/288>`_)
  * feat(obstacle_cruise_planner): add param for slow down
  * update
  ---------
* refactor(behavior_velocity_planner::intersection): organize param intersection (`#297 <https://github.com/autowarefoundation/autoware_launch/issues/297>`_)
  reorganize intersection param for readability
* refactor(behavior_velocity_planner): removed external input from behavior_velocity (`#296 <https://github.com/autowarefoundation/autoware_launch/issues/296>`_)
  removed external input from behavior_velocity
* feat(rviz): add rough goal (`#295 <https://github.com/autowarefoundation/autoware_launch/issues/295>`_)
* feat(avoidance): margin can be set independently for each class (`#286 <https://github.com/autowarefoundation/autoware_launch/issues/286>`_)
* feat(map_loader): add param for selected_map_loader (`#285 <https://github.com/autowarefoundation/autoware_launch/issues/285>`_)
  feat(map_loader): add param for selected_nap_loader
* refactor(obstacle_cruise_planner): clean up a part of the code (`#287 <https://github.com/autowarefoundation/autoware_launch/issues/287>`_)
  * modify parameters
  * use cruise planner temporarily
  * update
  * use obstacle_stop_planner by default
  ---------
* fix(behavior_path_planner): pull over deceleration (`#273 <https://github.com/autowarefoundation/autoware_launch/issues/273>`_)
* fix(rviz): fix debug marker topic name (`#289 <https://github.com/autowarefoundation/autoware_launch/issues/289>`_)
* feat(behavior_velocity_planner): add out of lane module (`#269 <https://github.com/autowarefoundation/autoware_launch/issues/269>`_)
  * Add initial param file for new out_of_lane module
  * Add params for extending the ego footprint
  * Add more parameters
  * add a few more params
  * Add/rename parameters for new version with 3 methods (thr, inter, ttc)
  * Add parameters for "skip_if\_*", "strict", and "use_predicted_path"
  * Update default parameters
  * style(pre-commit): autofix
  * Fix typo
  * Change param ego.extra_front_offset 1.0 -> 0.0
  * Update rviz config with "out_of_lane" virtual wall and debug markers
  ---------
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
* feat(behavior_path_planner): add new lateral acceleration (`#283 <https://github.com/autowarefoundation/autoware_launch/issues/283>`_)
* feat: add option of enable_cog_on_centerline (`#278 <https://github.com/autowarefoundation/autoware_launch/issues/278>`_)
* feat(behavior_path_planner): multiple candidate modules can run simultaneously (`#266 <https://github.com/autowarefoundation/autoware_launch/issues/266>`_)
* refactor(behavior_path_planner): rename lane chagne parameters (`#284 <https://github.com/autowarefoundation/autoware_launch/issues/284>`_)
* refactor(behavior_path_planner): rename lane change parameters (`#282 <https://github.com/autowarefoundation/autoware_launch/issues/282>`_)
  update
* fix(behavior_path_planner): remove unnecessary lane change parameter (`#280 <https://github.com/autowarefoundation/autoware_launch/issues/280>`_)
* feat(behavior_path_planner): enable LC+Avoidacne simultaneous execution (`#271 <https://github.com/autowarefoundation/autoware_launch/issues/271>`_)
* refactor(behavior_path_planner): remove lane change planner parameters (`#281 <https://github.com/autowarefoundation/autoware_launch/issues/281>`_)
* refactor(behavior_velocity_planner): add default values (`#272 <https://github.com/autowarefoundation/autoware_launch/issues/272>`_)
  * fix(behavior): add missing params
  * fix(behavior): add missing params
  * fix(behavior): add missing params
  * fix(behavior): add missing params
  * fix(behavior): add missing params
  * fix typo
  ---------
  Co-authored-by: satoshi-ota <satoshi.ota928@gmail.com>
  Co-authored-by: Satoshi OTA <44889564+satoshi-ota@users.noreply.github.com>
* feat(avoidance_by_lc): add new module to avoid obstacle by lane change (`#261 <https://github.com/autowarefoundation/autoware_launch/issues/261>`_)
  * feat(avoidance_by_lc): add config
  * feat(launch): add avoidance by lc param path
  * feat(rviz): add marker for avoidance by lc
  ---------
* feat(behavior_velocity_planner::blind_spot): consider adjacent lane (`#267 <https://github.com/autowarefoundation/autoware_launch/issues/267>`_)
* fix(autoware_launch): fix external lane change name (`#276 <https://github.com/autowarefoundation/autoware_launch/issues/276>`_)
  * fix(autoware_launch): fix external lane change name
  * update rviz
  ---------
* feat(autoware_launch): ext_lane_change -> external_lane_change in rtc (`#274 <https://github.com/autowarefoundation/autoware_launch/issues/274>`_)
  * fix
  * empty commit
  ---------
* refactor(behavior_path_planner): separate config file (`#270 <https://github.com/autowarefoundation/autoware_launch/issues/270>`_)
  feat(behavior_path_planner): add new config for manager
* feat(avoidance): update avoidance param (`#263 <https://github.com/autowarefoundation/autoware_launch/issues/263>`_)
  * feat(avoidance): enable safety check and yield
  * feat(avoidance): update lateral margin
  ---------
* feat(lane_following): consider lane ego angle diff (`#250 <https://github.com/autowarefoundation/autoware_launch/issues/250>`_)
  feat(lane_following): consider lane-ego angle diff
* feat(compare_map_segmentation): add param for dynamic map loading (`#257 <https://github.com/autowarefoundation/autoware_launch/issues/257>`_)
* feat(autoware_launch): consider behavior's drivable area violation (`#254 <https://github.com/autowarefoundation/autoware_launch/issues/254>`_)
* fix(behavior_velocity_planner): fix detection area being ignored when the ego vehicle stops over the stop line (`#260 <https://github.com/autowarefoundation/autoware_launch/issues/260>`_)
  * chore(behavior_velocity_planner): follow the latest implementation
  (hold_stop_margin_distance)
  * feat(behavior_velocity_planner): add a parameter for judging over the stop line
  ---------
* feat(autoware_launch): add check_external_emergency_heartbeat option (`#253 <https://github.com/autowarefoundation/autoware_launch/issues/253>`_)
* fix(behavior_path_planner): enable simlunateous executione (`#258 <https://github.com/autowarefoundation/autoware_launch/issues/258>`_)
* chore(ekf_localizer): move parameters to its dedicated yaml file (`#244 <https://github.com/autowarefoundation/autoware_launch/issues/244>`_)
  * chores(ekf_localizer): move parameters to its dedicated yaml file
  * style(pre-commit): autofix
  ---------
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
* fix(planning_launch): disable external lc module in new framework (`#262 <https://github.com/autowarefoundation/autoware_launch/issues/262>`_)
* feat(autoware_launch): add missing stop line parameter (`#252 <https://github.com/autowarefoundation/autoware_launch/issues/252>`_)
* chore(ground_segmentation): add optional param (`#259 <https://github.com/autowarefoundation/autoware_launch/issues/259>`_)
* fix(rviz): fix lane change topic name (`#256 <https://github.com/autowarefoundation/autoware_launch/issues/256>`_)
  * fix(rviz): fix lane change topic name
  * fix(rviz): rename
  ---------
* feat(control_launch): add parameter for aeb (`#232 <https://github.com/autowarefoundation/autoware_launch/issues/232>`_)
* fix(behavior_velocity_planner): disable launch_virtual_traffic_light parameter by default (`#251 <https://github.com/autowarefoundation/autoware_launch/issues/251>`_)
* feat(autoware_launch): enable pose initialization while running (only for sim) (`#243 <https://github.com/autowarefoundation/autoware_launch/issues/243>`_)
  * feat(autoware_launch): enable pose initialization while running (only for sim)
  * style(pre-commit): autofix
  * update localization param
  * both logsim and psim params
  * only one pose_initializer_param_path arg
  * style(pre-commit): autofix
  * use two param files for pose_initializer
  * style(pre-commit): autofix
  * debug
  ---------
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
* feat(lane_change): support param for new framework (include external lc) (`#248 <https://github.com/autowarefoundation/autoware_launch/issues/248>`_)
  * feat(lane_change): support param for new framework (include external lc)
  * rename external lane change
  ---------
* feat(rviz): add path reference marker (`#245 <https://github.com/autowarefoundation/autoware_launch/issues/245>`_)
  * feat(rviz): add path reference (hide as default)
  * fix(rviz): fix typo
  Co-authored-by: Zulfaqar Azmi <93502286+zulfaqar-azmi-t4@users.noreply.github.com>
  * differentiate path color
  ---------
  Co-authored-by: Zulfaqar Azmi <93502286+zulfaqar-azmi-t4@users.noreply.github.com>
  Co-authored-by: Muhammad Zulfaqar Azmi <zulfaqar.azmi@tier4.jp>
* feat(autoware_launch): add option to use akima spline at first (`#247 <https://github.com/autowarefoundation/autoware_launch/issues/247>`_)
  * feat(autoware_launch): enable akima spline for xy only first
  * disable akima spline
  ---------
* fix(autoware_launch): perception mode (`#249 <https://github.com/autowarefoundation/autoware_launch/issues/249>`_)
* fix(pull_over): add params (`#246 <https://github.com/autowarefoundation/autoware_launch/issues/246>`_)
  * fix(pull_over): add params
  * Update autoware_launch/config/planning/scenario_planning/lane_driving/behavior_planning/behavior_path_planner/pull_over/pull_over.param.yaml
  ---------
  Co-authored-by: Kosuke Takeuchi <kosuke.tnp@gmail.com>
* feat(planning_evaluator): delete_launch_planning_evaluator_arg (`#240 <https://github.com/autowarefoundation/autoware_launch/issues/240>`_)
  * delete_launch_planning_evaluator_arg
  * delete_launch_planning_evaluator_arg
  ---------
* feat(avoidance): add config for new framework (`#233 <https://github.com/autowarefoundation/autoware_launch/issues/233>`_)
  * feat(behavior_path_planner): add config for avoidance on new framework
  * fix(behavior_path_planner): fix priority
  * fix(behavior_path_planner): add description of priority param
  ---------
* feat(pull_over): support new framework (`#237 <https://github.com/autowarefoundation/autoware_launch/issues/237>`_)
  feat(behavior_path_planner): add config for pull over on new framework
* feat(lane_change): add config for new framework (`#239 <https://github.com/autowarefoundation/autoware_launch/issues/239>`_)
  * feat(lane_change): add config for new framework
  * rearrange module alphabetically
  ---------
* fix(avoidance): increase road shoulder margin 0.0 -> 0.3 (`#242 <https://github.com/autowarefoundation/autoware_launch/issues/242>`_)
* feat: lengthen safety_check_backward_distance in avoidance module (`#241 <https://github.com/autowarefoundation/autoware_launch/issues/241>`_)
* feat(side_shift): add config for new framework (`#234 <https://github.com/autowarefoundation/autoware_launch/issues/234>`_)
  feat(behavior_path_planner): add config for side shift on new framework
* feat(pull_out): add config for new framework (`#236 <https://github.com/autowarefoundation/autoware_launch/issues/236>`_)
  feat(behavior_path_planner): add config for pull out on new framework
* feat(planning_evaluator): launch planning_evaluator when scenario simulation is running (`#219 <https://github.com/autowarefoundation/autoware_launch/issues/219>`_)
  * feat(planning_evaluator): launch planning_evaluator when scenario simulation is running
  * refactoring
  ---------
* feat(behavior_path_planner): pull over freespace parking (`#221 <https://github.com/autowarefoundation/autoware_launch/issues/221>`_)
  * feat(behavior_path_planner): pull over freespace parking
  * fix typo
  ---------
* fix(behavior_path_planner): change pull over maximum_deceleration (`#229 <https://github.com/autowarefoundation/autoware_launch/issues/229>`_)
* fix(autoware_launch): add walkway visualization (`#231 <https://github.com/autowarefoundation/autoware_launch/issues/231>`_)
* feat(autoware_launch): update obstacle avoidance planner parameter (`#220 <https://github.com/autowarefoundation/autoware_launch/issues/220>`_)
  * update obstacle_avoidance_planner's param
  * update
  ---------
* feat(avoidance): add new param for avoidance target object filtering logic (`#226 <https://github.com/autowarefoundation/autoware_launch/issues/226>`_)
  feat(avoidance): add new param for avoidance
* feat(behavior_path_planner): add param for new planner manager (`#218 <https://github.com/autowarefoundation/autoware_launch/issues/218>`_)
* feat(obstacle_stop_planner): add margin behind goal (`#228 <https://github.com/autowarefoundation/autoware_launch/issues/228>`_)
* fix(autoware_launch): use tier4_sensing_component.launch.xml (`#227 <https://github.com/autowarefoundation/autoware_launch/issues/227>`_)
* feat(mission_planner): add config param file for mission_planner package (`#225 <https://github.com/autowarefoundation/autoware_launch/issues/225>`_)
  * add mission_planner param file
  * style(pre-commit): autofix
  ---------
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
* feat(osbtacle_stop_planner): add new trajectory checker parameter (`#224 <https://github.com/autowarefoundation/autoware_launch/issues/224>`_)
  add new trajectory checker parameter
* fix(obstacle_stop_planner): add lacking param (`#222 <https://github.com/autowarefoundation/autoware_launch/issues/222>`_)
  add param
  Co-authored-by: yamazakiTasuku <tasuku.yamazaki@tier4.jp>
* feat(lane_departure_checker): add optional neighbor lanelets parameters (`#217 <https://github.com/autowarefoundation/autoware_launch/issues/217>`_)
  feat(lane_departure_checker): add optional neighbor lanelets parameters  (`#217 <https://github.com/autowarefoundation/autoware_launch/issues/217>`_)
* feat(vehicle_cmd_gate): enable filter with actual steer in manual mode (`#189 <https://github.com/autowarefoundation/autoware_launch/issues/189>`_)
* chore(run_out): update parameter for mandatory detection area (`#205 <https://github.com/autowarefoundation/autoware_launch/issues/205>`_)
* feat(autoware_launch): update rviz and rtc config for planning (`#213 <https://github.com/autowarefoundation/autoware_launch/issues/213>`_)
* fix(autoware_launch): add missing system file (`#211 <https://github.com/autowarefoundation/autoware_launch/issues/211>`_)
* fix(autoware_launch): unify with tier4 launch (`#210 <https://github.com/autowarefoundation/autoware_launch/issues/210>`_)
  * fix
  * fix
  * fix
  * fix
  * fix
  * fix
  ---------
* feat(system_component_launch): add config param for dummy diag publisher (`#206 <https://github.com/autowarefoundation/autoware_launch/issues/206>`_)
  * add config param for dummy diag publisher
  * fixed dummy_diag_publisher param yaml (add empty diag)
  * launch dummy diag publisher by launch_dummy_diag_publisher param
  ---------
* refactor(autoware_launch): clean up component launch (`#207 <https://github.com/autowarefoundation/autoware_launch/issues/207>`_)
  * refactor(autoware_launch): clean up component launch
  * fix
  * Update autoware_launch/launch/components/tier4_map_component.launch.xml
  Co-authored-by: kminoda <44218668+kminoda@users.noreply.github.com>
  ---------
  Co-authored-by: kminoda <44218668+kminoda@users.noreply.github.com>
* fix: revert "chore: sync param files (`#161 <https://github.com/autowarefoundation/autoware_launch/issues/161>`_)" (`#209 <https://github.com/autowarefoundation/autoware_launch/issues/209>`_)
  This reverts commit 6fe7144c4d8e7909b58c5abbccd59c8c200e0618.
  Co-authored-by: awf-autoware-bot[bot] <94889083+awf-autoware-bot[bot]@users.noreply.github.com>
* fix(autoware_launch): fix the planning param (`#208 <https://github.com/autowarefoundation/autoware_launch/issues/208>`_)
* fix(autoware_launch): add z_axis_filtering params for obstacle_stop_planner node (`#150 <https://github.com/autowarefoundation/autoware_launch/issues/150>`_)
* fix(obstacle_avoidance_planner.param.yaml): add a margin for vehicle to stop before the end of drivable area boundary (`#192 <https://github.com/autowarefoundation/autoware_launch/issues/192>`_)
  * fix: add margin for vehicle stop before the end of drivable area boundary
  Adding ros parameter for vehicle stop margin for obstacle avoidance planner
  * style(pre-commit): autofix
  ---------
  Co-authored-by: Kenji Miyake <31987104+kenji-miyake@users.noreply.github.com>
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
* feat(behavior_path_planner): expand the drivable area based on the vehicle footprint (`#204 <https://github.com/autowarefoundation/autoware_launch/issues/204>`_)
* feat(autoware_launch): add param for configurable lateral_distance (`#140 <https://github.com/autowarefoundation/autoware_launch/issues/140>`_)
  add param for configurable lateral_distance & debug flag
  Co-authored-by: beyza <bnk@leodrive.ai>
* chore: add lanelet2 map config (`#169 <https://github.com/autowarefoundation/autoware_launch/issues/169>`_)
  * add_lanelet2_map_config to autoware_launch config
  * set projector type as default
  ---------
* fix(behavior_velocity_planner): continue collision checking after pass judge (`#203 <https://github.com/autowarefoundation/autoware_launch/issues/203>`_)
  fix(behavior_velocity_planner): revert part of `#2719 <https://github.com/autowarefoundation/autoware_launch/issues/2719>`_
* chore: sync param files (`#161 <https://github.com/autowarefoundation/autoware_launch/issues/161>`_)
  Co-authored-by: takayuki5168 <takayuki5168@users.noreply.github.com>
* feat(autoware_launch): add option to disable path update during avoidance (`#198 <https://github.com/autowarefoundation/autoware_launch/issues/198>`_)
* chore(autoware_launch): minor parameter change for planning and control (`#195 <https://github.com/autowarefoundation/autoware_launch/issues/195>`_)
* chore(autoware_launch): minor fix with trajectory_follower param (`#167 <https://github.com/autowarefoundation/autoware_launch/issues/167>`_)
* fix(autoware_launch): change behavior_velocity parameters (`#179 <https://github.com/autowarefoundation/autoware_launch/issues/179>`_)
* fix(multi_object_tracker): update data association matrix (`#196 <https://github.com/autowarefoundation/autoware_launch/issues/196>`_)
  * fix(multi_object_tracker): update data association matrix
  * Update autoware_launch/config/perception/object_recognition/tracking/multi_object_tracker/data_association_matrix.param.yaml
  also update for other CAR->(other objects)
  Co-authored-by: Yukihiro Saito <yukky.saito@gmail.com>
  ---------
  Co-authored-by: Yukihiro Saito <yukky.saito@gmail.com>
* feat(planning_config): update params to enable 60kmph speed (`#194 <https://github.com/autowarefoundation/autoware_launch/issues/194>`_)
* feat(autoware_launch): add NDT parameters for dynamic_map_loading (`#151 <https://github.com/autowarefoundation/autoware_launch/issues/151>`_)
  * feat(autoware_launch): add NDT parameters for dynamic_map_loading
  * set default param to false
  * set default use_dynamic_map_loading to true
  * fix parameter description
  ---------
* fix(lane_change): update default parameter (`#183 <https://github.com/autowarefoundation/autoware_launch/issues/183>`_)
* fix(autoware_launch): minor change with tier4_control_component (`#186 <https://github.com/autowarefoundation/autoware_launch/issues/186>`_)
* fix(autoware_launch): minor change with tier4_planning_component (`#185 <https://github.com/autowarefoundation/autoware_launch/issues/185>`_)
  * fix(autoware_launch): minor change with tier4_planning_component
  * update
  ---------
* fix(occlusion_spot): occlusion spot parameter disable as default (`#191 <https://github.com/autowarefoundation/autoware_launch/issues/191>`_)
* feat(mpc_lateral_controller): add steering bias removal (`#190 <https://github.com/autowarefoundation/autoware_launch/issues/190>`_)
  * feat(mpc_lateral_controller): add steering bias removal
  * change name
* feat(intersection): add param for stuck stopline overshoot margin (`#188 <https://github.com/autowarefoundation/autoware_launch/issues/188>`_)
* fix(autoware_launch): enable launch_deprecated_api (`#187 <https://github.com/autowarefoundation/autoware_launch/issues/187>`_)
  * fix(autoware_launch): enable launch_deprecated_api
  * Update autoware_launch/launch/components/tier4_autoware_api_component.launch.xml
  Co-authored-by: Takagi, Isamu <43976882+isamu-takagi@users.noreply.github.com>
  Co-authored-by: Takagi, Isamu <43976882+isamu-takagi@users.noreply.github.com>
* feat(control_launch): add min_braking_distance to lane_departure_checker (`#184 <https://github.com/autowarefoundation/autoware_launch/issues/184>`_)
* feat(intersection): improve ego velocity prediction in collision detection (`#181 <https://github.com/autowarefoundation/autoware_launch/issues/181>`_)
* feat(autoware_launch): visualize modified_goal as PoseWithUuidStamped (`#182 <https://github.com/autowarefoundation/autoware_launch/issues/182>`_)
* feat(autoware_launch): add param for considering footprint edges when check the drivable area (`#175 <https://github.com/autowarefoundation/autoware_launch/issues/175>`_)
  * add is_considering_footprint_edges param
  * add description
  Co-authored-by: beyza <bnk@leodrive.ai>
* fix(autoware_launch): add missing simulator param (`#164 <https://github.com/autowarefoundation/autoware_launch/issues/164>`_)
* fix(obstacle_stop_planner): rename param for obstacle stop planner (`#174 <https://github.com/autowarefoundation/autoware_launch/issues/174>`_)
  fix(obstacle_stop_planner): hunting -> chattering
* feat(autoware_lauch): modify launch for planning_validator (`#180 <https://github.com/autowarefoundation/autoware_launch/issues/180>`_)
  * modify launch for planning_validator
  * fix
* feat(autoware_launch): add tier4 autoware api component (`#173 <https://github.com/autowarefoundation/autoware_launch/issues/173>`_)
* fix(autoware_launch): change planning parameters (`#178 <https://github.com/autowarefoundation/autoware_launch/issues/178>`_)
* fix(lane_change): fix default abort param (`#177 <https://github.com/autowarefoundation/autoware_launch/issues/177>`_)
  * fix(lane_change): fix default abort param
  * revert safety time margin
* fix(lane_change): behavior planning common and lane change param (`#158 <https://github.com/autowarefoundation/autoware_launch/issues/158>`_)
  * fix(planning): behavior planning common and lane change param
  * revert forward_path_length
* refactor(autoware_launch/bpp-avoidance): remove redundant parameters (`#170 <https://github.com/autowarefoundation/autoware_launch/issues/170>`_)
* feat(autoware_launch): add fitting_uniform_circle parameter for mpt (`#159 <https://github.com/autowarefoundation/autoware_launch/issues/159>`_)
  * feat(autoware_launch): add new config params for fitting uniform circle
  * reformat
* feat(behavior_path_planner): ignore pull out start near lane end (`#171 <https://github.com/autowarefoundation/autoware_launch/issues/171>`_)
* fix(autoware_launch): fix wrong parameter name (`#172 <https://github.com/autowarefoundation/autoware_launch/issues/172>`_)
  * fix(autoware_launch): fix wrong parameter name
  * fix
* refactor(autoware_launch): organize arguments for planning (`#165 <https://github.com/autowarefoundation/autoware_launch/issues/165>`_)
  * refactor(tier4_planning_launch): organize arguments
  * update
* feat(behavior_path_planner): add option for combining arc pull out paths (`#168 <https://github.com/autowarefoundation/autoware_launch/issues/168>`_)
  * feat(behavior_path_planner): add option for combining arc pull out paths
  * divide_pull_out_path
* feat(behavior_path_planner): ignore pull over goal near lane start (`#166 <https://github.com/autowarefoundation/autoware_launch/issues/166>`_)
  feat(behavior_path_planner) ignore pull over goal near lane start
* feat(autoware_launch): add point cloud container to localization launch (`#108 <https://github.com/autowarefoundation/autoware_launch/issues/108>`_)
* fix(rviz): fix topic name for obstacle_stop_planner debug marker (`#162 <https://github.com/autowarefoundation/autoware_launch/issues/162>`_)
* feat(behavior_path_planner): param to skip some linestring types when expanding the drivable area (`#160 <https://github.com/autowarefoundation/autoware_launch/issues/160>`_)
  [behavior_path_planner] add types to skip in drivable_area_expansion
* feat(autoware_launch): update pure_pursuit parameters (`#154 <https://github.com/autowarefoundation/autoware_launch/issues/154>`_)
* feat(obstacle_stop_planner): add parameter for voxel_grid (`#156 <https://github.com/autowarefoundation/autoware_launch/issues/156>`_)
  * feat(obstacle_stop_planner): add parameter for voxel grid
  * fix: parameter
* feat(avoidance): add new param for yield maneuver in avoidance module (`#146 <https://github.com/autowarefoundation/autoware_launch/issues/146>`_)
  * feat(avoidance): add new param for yield maneuver in avoidance module
  * fix(avoidance): fix param name for readability
  * fix(planning_launch): add missing param
* feat: add speed bump debug markers to autoware.rviz (`#63 <https://github.com/autowarefoundation/autoware_launch/issues/63>`_)
* feat(autoware_launch): add speed bump parameters (`#138 <https://github.com/autowarefoundation/autoware_launch/issues/138>`_)
* fix(autoware-launch): add NDT parameters for de-grounded lidar scan matching (`#155 <https://github.com/autowarefoundation/autoware_launch/issues/155>`_)
  * fix(autoware-launch):add NDT parameters for de-grounded lidar scan matching
  * style(pre-commit): autofix
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
* fix(behavior_velocity_planner): fix blind spot over-detection (`#153 <https://github.com/autowarefoundation/autoware_launch/issues/153>`_)
* fix(planning_launch): ignore unavoidable objects around the goal (`#152 <https://github.com/autowarefoundation/autoware_launch/issues/152>`_)
  * fix(planning_launch): ignore unavoidable objects around the goal
  * Update autoware_launch/config/planning/scenario_planning/lane_driving/behavior_planning/behavior_path_planner/avoidance/avoidance.param.yaml
  Co-authored-by: Kosuke Takeuchi <kosuke.tnp@gmail.com>
* feat(autoware_launch): enable use_individual_control_param (`#148 <https://github.com/autowarefoundation/autoware_launch/issues/148>`_)
  * fix(autoware_launch): add missing param of vehicle_cmd_gate in yaml
  * feat(autoware_launch): enable use_individual_control_param
* fix(autoware_launch): add missing param of vehicle_cmd_gate in yaml (`#149 <https://github.com/autowarefoundation/autoware_launch/issues/149>`_)
* refactor(autoware_launch): organize control yaml (`#147 <https://github.com/autowarefoundation/autoware_launch/issues/147>`_)
* feat: update param path (`#144 <https://github.com/autowarefoundation/autoware_launch/issues/144>`_)
  * feat: update param path
  * fix
  * organize param files
* fix: avoidance parameter file (`#145 <https://github.com/autowarefoundation/autoware_launch/issues/145>`_)
  * fix: avoidance parameter file
  * Update autoware_launch/config/planning/scenario_planning/lane_driving/behavior_planning/behavior_path_planner/avoidance/avoidance.param.yaml
* feat(perception): change detection region (`#141 <https://github.com/autowarefoundation/autoware_launch/issues/141>`_)
* fix(autoware_launch): remove unnecessary `config` declaration (`#137 <https://github.com/autowarefoundation/autoware_launch/issues/137>`_)
  * fix(autoware_launch): remove unnecessary `config` declaration
  * follow alphabetical order
* feat(autoware_launch): move config from autoware.universe for simulator (`#132 <https://github.com/autowarefoundation/autoware_launch/issues/132>`_)
  * feat(autoware_launch): move config to autoware_launch for perception
  * first commit
  * fix(autoware_launch): fix config path of system_error_monitor
  * fix typo again
  * resolve conflict
* fix(autoware_launch): fix config path of system_error_monitor (`#136 <https://github.com/autowarefoundation/autoware_launch/issues/136>`_)
  * fix(autoware_launch): fix config path of system_error_monitor
  * fix typo again
* feat(autoware_launch): move config from autoware.universe for control (`#134 <https://github.com/autowarefoundation/autoware_launch/issues/134>`_)
  * feat(autoware_launch): move config from autoware.universe for control
  * fix
  * update operation_mode_transition_manager.param.yaml
  Co-authored-by: kminoda <koji.minoda@tier4.jp>
* feat(autoware_launch): move config from autoware.universe for map (`#128 <https://github.com/autowarefoundation/autoware_launch/issues/128>`_)
  feat(autoware_launch): move config to autoware_launch for map
* feat(autoware_launch): move config from autoware.universe for localization (`#129 <https://github.com/autowarefoundation/autoware_launch/issues/129>`_)
  * feat(autoware_launch): move config to autoware_launch for localization
  * fix order
* feat(autoware_launch): move config from autoware.universe for perception (`#131 <https://github.com/autowarefoundation/autoware_launch/issues/131>`_)
  feat(autoware_launch): move config to autoware_launch for perception
* feat(autoware_launch): move config from autoware.universe for planning (`#133 <https://github.com/autowarefoundation/autoware_launch/issues/133>`_)
  * feat(autoware_launch): move config from autoware.universe for planning
  * remove unnecessary config
  * add rtc
  * apply universe param change
* feat(autoware_launch): move config from autoware.universe for system (`#130 <https://github.com/autowarefoundation/autoware_launch/issues/130>`_)
  * feat(autoware_launch): move config to autoware_launch for system
  * fix
* feat(autoware_launch): add bound visualization (`#135 <https://github.com/autowarefoundation/autoware_launch/issues/135>`_)
* feat(autoware_launch): visualize multiple candidate paths (`#125 <https://github.com/autowarefoundation/autoware_launch/issues/125>`_)
  * feat(autoware_launch): visualize multiple candidate paths
  * feature(autoware.rviz): fix name space
* fix: change check_external_emergency_heartbeat false (`#119 <https://github.com/autowarefoundation/autoware_launch/issues/119>`_)
  * fix: use_external_emergenc_stop false
  * Update autoware.launch.xml
* fix(autoware_launch): remove web_controller (`#121 <https://github.com/autowarefoundation/autoware_launch/issues/121>`_)
* fix: rename use_external_emergency_stop to check_external_emergency_heartbeat (`#120 <https://github.com/autowarefoundation/autoware_launch/issues/120>`_)
* feat: remove web controller (`#117 <https://github.com/autowarefoundation/autoware_launch/issues/117>`_)
* fix(autoware_launch): add parameter to launch launch_system_monitor (`#113 <https://github.com/autowarefoundation/autoware_launch/issues/113>`_)
* fix(rviz): update obstacle_avoidance_planner virtual_wall topic name (`#115 <https://github.com/autowarefoundation/autoware_launch/issues/115>`_)
* feat: set use_external_emergency_stop to false by default when launching psim (`#109 <https://github.com/autowarefoundation/autoware_launch/issues/109>`_)
* refactor(autoware_launch): change pure_pursuit debug marker topic name (`#110 <https://github.com/autowarefoundation/autoware_launch/issues/110>`_)
  Co-authored-by: Berkay Karaman <berkay@leodrive.ai>
* feat(autoware_launch): add obstacle_collision_checker to launch (`#106 <https://github.com/autowarefoundation/autoware_launch/issues/106>`_)
  * feat(autoware_launch): add obstacle_collision_checker to launch
  * ci(pre-commit): autofix
  * add option to other launch files
  * ci(pre-commit): autofix
  Co-authored-by: Berkay Karaman <berkay@leodrive.ai>
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
* feat(autoware_launch): add autoware_state_panel to autoware.rviz (`#104 <https://github.com/autowarefoundation/autoware_launch/issues/104>`_)
* fix(rviz): enlarge pointcloud size for NDT results (`#103 <https://github.com/autowarefoundation/autoware_launch/issues/103>`_)
  * fix(rviz): enlarge pointcloud size for NDT results
  * change default settings of Initial pointcloud
* feat(autoware_launch): add e2e_simulator.launch.xml (`#98 <https://github.com/autowarefoundation/autoware_launch/issues/98>`_)
  * feat(autoware_launch): add e2e_simulator.launch.xml
  * chore: rename
  * chore: remove unnecessary option
* fix(autoware_launch): launch dummy_localization regardless of scenario_simulation flag (`#95 <https://github.com/autowarefoundation/autoware_launch/issues/95>`_)
* feat: move adapi rviz adaptor  (`#88 <https://github.com/autowarefoundation/autoware_launch/issues/88>`_)
  feat: move adapi rviz adaptor
* chore(autoware_launch): designate vehicle_id to tier4_control_launch (`#74 <https://github.com/autowarefoundation/autoware_launch/issues/74>`_)
  Co-authored-by: Shumpei Wakabayashi <42209144+shmpwk@users.noreply.github.com>
* fix: add adapi dependency (`#86 <https://github.com/autowarefoundation/autoware_launch/issues/86>`_)
* feat(autoware_launch): apply the change of pose initializer (`#72 <https://github.com/autowarefoundation/autoware_launch/issues/72>`_)
  * feat(autoware_launch): apply the change of pose initializer
  * fix: add group for include scope
* feat(autoware_launch): add rviz helper for ad api (`#70 <https://github.com/autowarefoundation/autoware_launch/issues/70>`_)
  * feat(autoware_launch): add rviz helper for ad api
  * feat: change launch file
* feat(rviz): update rviz planning module's debug marker (`#85 <https://github.com/autowarefoundation/autoware_launch/issues/85>`_)
  * feat(rviz): add behavior path debug markers (`#441 <https://github.com/autowarefoundation/autoware_launch/issues/441>`_)
  * feat(rviz): add 2D dummy bus button as default (`#453 <https://github.com/autowarefoundation/autoware_launch/issues/453>`_)
  * feat(rviz): add behavior path debug markers (`#462 <https://github.com/autowarefoundation/autoware_launch/issues/462>`_)
  * feat(rviz): remove old pull over marker
  Co-authored-by: kosuke55 <kosuke.tnp@gmail.com>
* feat: disable visualization of object pose covariance (`#79 <https://github.com/autowarefoundation/autoware_launch/issues/79>`_)
* feat(autoware_launch): add vehicle footprint to autoware.rviz (`#81 <https://github.com/autowarefoundation/autoware_launch/issues/81>`_)
  feat: add vehicle footprint to autoware.rviz
* fix: use sim time option (`#69 <https://github.com/autowarefoundation/autoware_launch/issues/69>`_)
  * fix: use sim time option
  * fix: minimum usesim time option
  * ci(pre-commit): autofix
  * fix: add simulation flag to distinguish
  * ci(pre-commit): autofix
  * Revert "ci(pre-commit): autofix"
  This reverts commit 291770d114baae38c7645b51c2569a66fa6353d3.
  * Revert "Merge commit '7b1424eaeb2b06a6fb2921ff242a3c4e5fef6169' into fix/use_sim_time_option"
  This reverts commit 02fede2df4099d5019b60fddb585e53357e62535, reversing
  changes made to 9aeaa88f39c152333cac658a908f9ab8d7dcd037.
  * Revert "fix: add simulation flag to distinguish"
  This reverts commit 9aeaa88f39c152333cac658a908f9ab8d7dcd037.
  * fix: remove duplicated arg
  * fix: use sim time option
  * ci(pre-commit): autofix
  * fix: ungroup autoware launch
  * Revert "fix: ungroup autoware launch"
  This reverts commit cb3e3aa536ae2aa740cd846c97fe6cf373ba8744.
  * fix: to scoped false
  * chore: better comment
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
* refactor(autoware_launch): use autoware.launch.xml in planning/logging_simulator.launch.xml (`#59 <https://github.com/autowarefoundation/autoware_launch/issues/59>`_)
  * refactor(autoware_launch): use autoware.launch.xml in planning_simulator.launch.xml
  * fix planning_launcher launch and use autoware.launch.xml in logging_simulator.launch.xml
  * fix arguments
  * fix argument order
  * remove init_localization_pose
  * move if tag
  * reorder arguments
  * add description for arguments
* chore(rviz): update rviz config for pull_over (`#58 <https://github.com/autowarefoundation/autoware_launch/issues/58>`_)
* feat: add line_width property to object display (`#56 <https://github.com/autowarefoundation/autoware_launch/issues/56>`_)
* chore: update to universe config (`#52 <https://github.com/autowarefoundation/autoware_launch/issues/52>`_)
  * chore: to universe rviz config
  * feat: replace rostime to date time panel
  * fix : to best effott
  * feat: add manuever topic
  * chore: disable marker by default
  * feat: add diag marker
* fix(mission_planner): disable lane_start_bound in Rviz (`#51 <https://github.com/autowarefoundation/autoware_launch/issues/51>`_)
* feat: add arguments for pointcloud container to planning (`#37 <https://github.com/autowarefoundation/autoware_launch/issues/37>`_)
  * feat: add arguments for pointcloud container to planning
  * add marker for run out module
* feat: disable some namespace for lanelet2 (`#50 <https://github.com/autowarefoundation/autoware_launch/issues/50>`_)
* feat(rviz_plugin): adaptive scaling for display size (`#44 <https://github.com/autowarefoundation/autoware_launch/issues/44>`_)
* refactor: update rviz config for virtual wall marker separation (`#42 <https://github.com/autowarefoundation/autoware_launch/issues/42>`_)
* feat: add autoware_api.launch.xml to launch files in autoware_launch package (`#26 <https://github.com/autowarefoundation/autoware_launch/issues/26>`_)
  * feat: add autoware_api.launch.xml to launch files in autoware_launch package
  * fix(autoware_launch): fix flags for autoware.launch and logging_simulator.launch
* feat(obstacle_avoidance_planner): add obstacle_avoidance_planner wall marker in autoware.rviz (`#28 <https://github.com/autowarefoundation/autoware_launch/issues/28>`_)
* feat: cosmetic change autoware.rviz for perception (`#24 <https://github.com/autowarefoundation/autoware_launch/issues/24>`_)
* feat(autoware_launch): update autoware.rviz not to visualize occlusion spot markers (`#25 <https://github.com/autowarefoundation/autoware_launch/issues/25>`_)
  Co-authored-by: taikitanaka3 <taiki.tanaka@tier4.jp>
* feat(autoware_launch): update autoware.rviz not to visualize obstacle_avoidance_planner markers (`#23 <https://github.com/autowarefoundation/autoware_launch/issues/23>`_)
* fix: incorrect arguments and including launch (`#19 <https://github.com/autowarefoundation/autoware_launch/issues/19>`_)
  * fix: incorrect arguments and including launch
  * add launch_vehicle_interface
  * bug fix
* ci(pre-commit): update pre-commit-hooks-ros (`#18 <https://github.com/autowarefoundation/autoware_launch/issues/18>`_)
  * ci(pre-commit): update pre-commit-hooks-ros
  * ci(pre-commit): autofix
  * Update .pre-commit-config.yaml
  * Update .pre-commit-config.yaml
  * Update .pre-commit-config.yaml
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
* fix: fix launch args (`#6 <https://github.com/autowarefoundation/autoware_launch/issues/6>`_)
* fix: add group to each section (`#5 <https://github.com/autowarefoundation/autoware_launch/issues/5>`_)
  * chore: apply pre-commit
  * fix: add group
* feat: add autoware_launch (`#1 <https://github.com/autowarefoundation/autoware_launch/issues/1>`_)
  * feat: change launch package name (`#186 <https://github.com/autowarefoundation/autoware_launch/issues/186>`_)
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
  * feat(atutoware.rviz): disable selectable for pointcloud visualization (`#402 <https://github.com/autowarefoundation/autoware_launch/issues/402>`_)
  * fix: change the default mode of perception.launch (`#409 <https://github.com/autowarefoundation/autoware_launch/issues/409>`_)
  * fix: change the default mode of perception.launch
  * chore: remove unnecessary comments
  * chore: remove default values
  * rename package name
  * ci(pre-commit): autofix
  * add build_depends.repos
  Co-authored-by: Tomoya Kimura <tomoya.kimura@tier4.jp>
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
  Co-authored-by: tanaka3 <ttatcoder@outlook.jp>
  Co-authored-by: taikitanaka3 <65527974+taikitanaka3@users.noreply.github.com>
  Co-authored-by: Yamato Ando <yamato.ando@gmail.com>
* Contributors: Ahmed Ebrahim, Alexey Panferov, Amadeusz Szymko, Anh Nguyen, Autumn60, Azumi Suzuki, Berkay Karaman, Dmitrii Koldaev, Fumiya Watanabe, Giovanni Muhammad Raditya, Go Sakayori, Hasan Özfidan, Hiroki OTA, Ismet Atabay, Kaan Çolak, Kazunori-Nakajima, Kenji Miyake, Kento Yabuuchi, Kenzo Lobos Tsunekawa, Khalil Selyan, KokiAoki, Kosuke Takeuchi, Kyoichi Sugahara, M. Fatih Cırıt, Makoto Kurihara, Mamoru Sobue, Masaki Baba, Masato Saeki, Maxime CLEMENT, Mehmet Dogru, Mitsuhiro Sakamoto, Motz, Muhammed Yavuz Köseoğlu, Naophis, Phoebe Wu, Ryohsuke Mitsudome, RyuYamamoto, Ryuta Kambe, SakodaShintaro, SaltUhey, Satoshi OTA, Satoshi Tanaka, Shintaro Tomie, Shumpei Wakabayashi, Shunsuke Miura, Taekjin LEE, TaikiYamada4, Takagi, Isamu, Takamasa Horibe, Takayuki Murooka, Takeshi Miura, Tao Zhong, TetsuKawa, Tomohito ANDO, Tomoya Kimura, Vincent Richard, Xuyuan Han, Yamato Ando, Yoshi Ri, Yuki TAKAGI, Yukihiro Saito, Yusuke Muramatsu, Yutaka Shimizu, Yuxuan Liu, Zhe Shen, Zulfaqar Azmi, asana17, awf-autoware-bot[bot], badai nguyen, beginningfan, beyzanurkaya, cyn-liu, danielsanchezaran, ito-san, keisuke, kminoda, melike tanrikulu, mkquda, pawellech1, ryohei sasaki, taikitanaka3, yamazakiTasuku, Łukasz Chojnacki
