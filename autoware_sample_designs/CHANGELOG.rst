^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package autoware_sample_designs
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.52.0 (2026-06-30)
-------------------
* Merge remote-tracking branch 'origin/main' into tmp/bot/bump_version_base
* feat(autoware system designs): autoware system designs psim add planning control (`#1810 <https://github.com/autowarefoundation/autoware_launch/issues/1810>`_)
  * add planning node/modules (WIP)
  fix traffic light msg type
  add control node/modules (WIP)
  fix some msg types, inputs, and connections
  tmp fix
  update
  add visualization component (just rviz node)
  add missing parameters
  fix control parameter setting
  temporary remove the check_external_emergency_heartbeat arg from param
  Remove namespace entries for planning and control modules in AutowareSample.system.yaml
  Update message type for traffic_signals in Tier4PlanningWrapper.node.yaml to use autoware_perception_msgs
  Add new planning and control modules
  - Introduced Control.module, BehaviorPlanning.module, LaneDriving.module, MissionPlanning.module, MotionPlanning.module, Planning.module, and ScenarioPlanning.module.
  - Each module includes instances, subscribers, publishers, and connections to facilitate communication within the system.
  - Added Rviz.node for visualization purposes.
  This update enhances the overall architecture and functionality of the Autoware system design.
  Add control containers to AutowareSample.system.yaml
  - Introduced control_container and control_check_container to encapsulate control-related nodes.
  - Updated Control.module.yaml to remove commented-out control_container instance.
  - This update enhances the modularity and organization of the control components within the Autoware system design.
  Refactor planning modules and add component containers to AutowareSample.system.yaml
  - Removed commented-out container instances from BehaviorPlanning.module.yaml, MissionPlanning.module.yaml, and MotionPlanning.module.yaml for clarity.
  - Introduced new component containers for behavior, mission, and motion planning in AutowareSample.system.yaml to enhance modularity and organization of planning nodes.
  - This update improves the structure and maintainability of the planning components within the Autoware system design.
  Update parameter set files for universe_control and universe_planning
  - Updated autoware_system_design_format to 0.3.0 in both universe_control.parameter_set.yaml and universe_planning.parameter_set.yaml.
  - Changed 'parameters' to 'param_values' and 'parameter_files' to 'param_files' for consistency across parameter definitions.
  - Enhanced clarity by renaming parameter paths in universe_planning.parameter_set.yaml for better readability.
  Fix formatting issues in Rviz.node.yaml and AutowareSample.system.yaml
  - Added a newline at the end of Rviz.node.yaml to adhere to file formatting standards.
  - Removed unnecessary blank lines in AutowareSample.system.yaml for improved readability.
  * Refactor parameter paths in universe_planning.parameter_set.yaml for consistency
  - Unified parameter path keys from 'param_path1', 'param_path2', etc., to a single 'param_path' for clarity and consistency across nodes.
  - Ensured all relevant parameter files are correctly referenced under their respective nodes.
  * Enhance universe_planning.parameter_set.yaml and AutowareSample.system.yaml with additional parameter paths and connections
  - Updated parameter paths in universe_planning.parameter_set.yaml for consistency and clarity.
  - Added new parameter files for latency checking, trajectory checking, and collision checking in the universe planning configuration.
  - Expanded connections in AutowareSample.system.yaml to include localization publishers for kinematic state and acceleration, improving data flow between modules.
  * Refactor Control.module.yaml to update connections and improve clarity
  - Removed the steering status subscriber connection for better organization.
  - Updated the operation mode publisher connection to enhance clarity in the control module's configuration.
  * Refactor planning module configurations to remove virtual traffic light states
  - Eliminated the virtual_traffic_light_states subscriber and publisher from BehaviorPlanning, LaneDriving, MotionPlanning, Planning, and ScenarioPlanning modules for improved clarity and organization.
  - Updated connections accordingly to reflect the removal of these components, enhancing the overall structure of the planning modules.
  * Update planning module and system configurations to use mission planning namespaces
  - Changed publisher names in Planning.module.yaml to include the 'mission_planning' namespace for route and state.
  - Updated connections in AutowareSample.system.yaml to reflect the new publisher names, enhancing clarity and organization in the planning module's configuration.
  * Add planning evaluator node to universe_planning.parameter_set.yaml
  - Introduced a new node configuration for the planning evaluator, including its associated parameter files for enhanced planning capabilities.
  - This update improves the modularity and organization of the planning parameters within the universe planning configuration.
  * Update MissionPlanning and Planning modules to include clear_route server and connections
  - Incremented autoware_system_design_format to 0.3.1 in MissionPlanning.module.yaml and Planning.module.yaml.
  - Added clear_route server to both modules, enhancing route management capabilities.
  - Updated connections in both modules and AutowareSample.system.yaml to integrate clear_route functionality, improving overall system communication.
  - Introduced routing/clear_route client in SampleSensorKitADAPIWrapper.node.yaml for better interaction with the routing service.
  * Enhance Control and Planning modules with new route_state integration
  - Added stop_mode_operator node to Control.module.yaml for improved control logic.
  - Updated subscribers and connections in Control.module.yaml to include route_state, enhancing data flow.
  - Renamed state publisher to route_state in MissionPlanning.module.yaml and Planning.module.yaml for consistency.
  - Updated connections in AutowareSample.system.yaml to reflect new route_state integration, improving overall system communication.
  * Enhance MissionPlanning and Planning modules with new route and server integrations
  - Added new publishers for api_route and mission_planning/route in MissionPlanning.module.yaml and Planning.module.yaml to improve routing capabilities.
  - Introduced set_lanelet_route and set_waypoint_route servers in both modules for enhanced route management.
  - Updated connections in both modules and AutowareSample.system.yaml to reflect the new publishers and servers, improving overall system communication.
  * Add manual lane change handler to MissionPlanning module
  - Introduced manual_lane_change_handler node in MissionPlanning.module.yaml to enhance lane change capabilities.
  - Updated connections in both MissionPlanning.module.yaml and Planning.module.yaml to integrate the new handler, improving overall routing and operational functionality.
  * Add reroute availability publisher to planning modules
  - Introduced reroute_availability publisher in BehaviorPlanning, LaneDriving, and ScenarioPlanning modules to enhance routing capabilities.
  - Updated connections in Planning.module.yaml to integrate reroute availability, improving overall system communication and responsiveness.
  - Ensured consistency across modules by adding modified_goal and reroute_availability connections where applicable.
  * Update Planning and AutowareSample system configurations with new subscriber connections
  - Added new subscriber connections in Planning.module.yaml for route, lanelet map, kinematics, acceleration, operational mode state, and traffic signals to enhance data flow and validation capabilities.
  - Updated AutowareSample.system.yaml to include additional subscriber connections for steering status and traffic signals, improving overall system communication and integration.
  * style(pre-commit): autofix
  * Add operation mode state subscriber connection to AutowareSample system configuration
  - Introduced a new subscriber connection for operation_mode/state in AutowareSample.system.yaml to enhance system communication and integration.
  - This update improves the overall data flow and responsiveness of the system by ensuring that the operation mode state is properly communicated to the control module.
  * Add Rviz node configuration and integration into AutowareSample system
  - Updated Rviz.node.yaml to include arguments for loading the Autoware RViz configuration and image.
  - Integrated Rviz node into AutowareSample.system.yaml, enhancing the system's visualization capabilities by ensuring Rviz is properly connected to the main ECU.
  * Update API paths in AutowareSample system and SampleSensorKitADAPIWrapper configurations
  - Changed the API path for the SampleSensorKitADAPIWrapper from '/api' to '/' to streamline endpoint access.
  - Updated publisher names in SampleSensorKitADAPIWrapper.node.yaml to include 'api/' prefix for consistency across localization, vehicle, planning, and system outputs.
  - Adjusted connections in AutowareSample.system.yaml to reflect the new publisher naming conventions, enhancing overall system integration and communication.
  * Refactor connections in AutowareSample system configuration for clarity and consistency
  * Add additional parameter files for behavior and motion velocity planners in universe planning configuration
  * Add additional parameter files for behavior path planner in universe planning configuration
  * Add connection for fail-safe state between system and control components
  * Refactor planning and control modules to enhance subscriber and publisher interfaces, including updates to object handling and path management.
  * Refactor module connections to improve clarity and consistency in subscriber and publisher mappings across BehaviorPlanning, MotionPlanning, DummyObjectRecognition, and AutowareSample configurations.
  * style(pre-commit): autofix
  * Add occupancy grid map connections for PlanningSimulation mode
  * Enhance control module interfaces by adding emergency command subscribers and publishers, and update connections for improved functionality.
  * Refactor planning module connections to replace 'api_route' with 'mission_planning/route' for improved clarity and consistency in routing.
  * Refactor control module interfaces and connections to standardize external command naming and improve clarity in subscriber and publisher mappings.
  * Enhance connections in AutowareSample system by adding new subscribers for adapi and simulator states to improve data flow and control responsiveness.
  * Fix formatting in Control.module.yaml by ensuring proper newline at the end of the connections section.
  * Refactor Control.module.yaml by removing unused gate_mode publisher and updating connections for external_cmd_converter to enhance data flow.
  * add missing connections to control: sim(imu,vehicle status), api(state)
  * temporary fixes for the diagnostic topics and some control module topics
  * fix gate mode topic
  * cleanup rebase issues
  * Update Control.module.yaml and AutowareSample.system.yaml for version 0.3.1 to 0.3.2, enhancing connections and remapping topics for improved data flow and control responsiveness.
  * Refactor API naming in AutowareSample.system.yaml and SampleSensorKitADAPIWrapper.node.yaml to standardize topic paths and improve clarity in publisher and subscriber mappings.
  * Add remapping for rosbag replay publisher topics to improve data flow in LoggingSimulation mode
  * Add SampleAWSIM node configuration and update connections in AutowareSample.system.yaml for improved data flow and topic remapping
  * Add remapping for PlanningSimulation components to enhance topic routing and data flow
  * Fix remap key to remaps in AutowareSample.system.yaml for consistency across modes
  * Add engage publisher connection and clean up PlanningSimulation remaps for improved data flow
  * Update planning connections and add route publisher for improved data flow
  * bring back temporary fixes
  * Comment out occlusion predictor in traffic light recognition for clarity in node group configuration
  * Refactor Control.module.yaml and add TrajectoryFollower.module.yaml for improved module structure and clarity
  * Add steering_offset_update subscriber and enhance TrajectoryFollower connections for improved data handling
  * Restore performance monitoring links in control diagnostics for enhanced monitoring capabilities
  * Remove unnecessary blank line in TrajectoryFollower.module.yaml for improved readability
  * Update autoware_system_designer to v0.4.0 in workflow and pre-commit configuration for improved linting capabilities
  * Update autoware_system_designer to v0.4.1 in workflow and pre-commit configuration for improved linting capabilities
  ---------
  Co-authored-by: Maxime CLEMENT <maxime.clement@tier4.jp>
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
* feat(perception): update perception pipeline to remove shape estimation (`#1846 <https://github.com/autowarefoundation/autoware_launch/issues/1846>`_)
  * fix(perception): update input channels and ground segmentation parameters for improved tracking and detection
  - Changed associator type from 'bev' to 'polar' in multi-object tracker input channels for better performance.
  - Adjusted ground segmentation parameters: increased margin_max_z from 0.0 to 1.0 and detection_range_z_max from 2.5 to 3.5 to enhance ground detection capabilities.
  * feat(Tier4PlanningWrapper): add data_path parameter for enhanced configuration flexibility
  - Introduced a new parameter 'data_path' with a default value to improve the configuration options for the Tier4PlanningWrapper node.
  - This addition allows for better management of data paths within the planning framework.
  * refactor(perception): remove deprecated tracking modules and streamline object recognition configuration
  - Eliminated unused tracking modules and parameters from ObjectRecognition and ObjectDetection configurations to enhance clarity and reduce complexity.
  - Updated connections in LidarClustering to reflect the removal of obsolete components, ensuring a more efficient data flow.
  refactor(parameter_set): streamline perception parameters by removing unused nodes and updating node paths
  - Removed the unused shape_estimation node from the perception parameter set to enhance clarity.
  - Updated the path for the detected_object_feature_remover node to reflect its new location within the clustering module.
  fix(perception): update pointcloud_map_filter node path for improved object recognition
  * fix(perception): update pointcloud_map_filter node to use voxel_grid_downsample_filter
  * fix(perception): update source URL in input_channels.param.yaml for accuracy
  * fix(perception): update max_z and margin_max_z parameters for improved segmentation accuracy
  * refactor(perception): remove unused parameters from crop_box_filter and streamline sample_system_perception configuration
  * fix(perception): disable detection_by_tracker function in perception launch configuration
  ---------
* refactor(perception): update object recognition modules and parameters to multi-channel merger (`#1833 <https://github.com/autowarefoundation/autoware_launch/issues/1833>`_)
  * refactor(sample_system_perception): streamline parameter configurations by removing unused param_files and adding new param_values for improved clarity and functionality
  * feat(autoware_sample_designs): enhance AutowareSample system configuration by adding new lidar containers and updating node group definitions for improved modularity and clarity
  * feat(lidar): add ring outlier filter node to LidarVelodyneVLS128 module for enhanced point cloud processing
  * fix(robot_state_publisher): add remap targets for robot_description and joint_states to enhance topic clarity and global accessibility
  * refactor(autoware_sample_designs): update lidar container types in AutowareSample system configuration for improved clarity and consistency
  * refactor(autoware_sample_designs): simplify lidar container type in AutowareSample system configuration for enhanced clarity
  * refactor(autoware_sample_designs): correct lidar container types in AutowareSample system configuration for consistency
  * refactor(autoware_sample_designs): update pointcloud_container type and refine node group definitions in AutowareSample system configuration for improved clarity
  * refactor(perception): update object recognition modules and parameters to multi-channel merger
  * style(pre-commit): autofix
  * refactor(perception): update object recognition modules and remove deprecated tracking modules
  - Changed tracking entity from ObjectTrackingMultiChannel.module to ObjectTracking.module.
  - Added centerpoint instance to ObjectDetection.module for improved lidar processing.
  - Introduced new ObjectTracking.module for multi-object tracking capabilities.
  - Removed obsolete ObjectTrackingSingleChannel.module and ObjectTrackingTrackerMergerRadarFusion.module.
  * feat(perception): introduce Perception module and update system configuration
  - Added a new Perception.module.yaml file defining instances for obstacle segmentation, object recognition, and occupancy grid mapping.
  - Updated AutowareSample.system.yaml to integrate the new Perception module, replacing individual perception components with a unified entry.
  - Adjusted connections to route data through the new Perception module for improved modularity and clarity.
  * feat(perception): enhance traffic light recognition integration
  - Added traffic light recognition module to Perception.module.yaml, including new instances and connections for traffic signals.
  - Updated AutowareSample.system.yaml to establish connections for traffic light recognition, integrating it with existing perception components.
  - Adjusted subscriber and publisher configurations to accommodate traffic light data flow.
  * refactor(perception): update object recognition tracking module and connections
  - Changed tracking entity from ObjectTrackingSingleChannel.module to MultiObjectTracker.node for enhanced multi-object tracking capabilities.
  - Updated connections to reflect the new tracking module, ensuring proper data flow between detection and tracking components.
  * style(pre-commit): autofix
  * feat(system): add traffic light node container to AutowareSample system configuration
  - Introduced a new node group for traffic light recognition, including multiple nodes for detection and classification.
  - Enhanced the system's modularity by integrating traffic light components into the existing architecture.
  * feat(system): add lidar centerpoint detection node to AutowareSample system configuration
  - Included the lidar_centerpoint node in the perception object recognition group to enhance detection capabilities.
  - Improved the system's functionality by integrating additional object recognition components.
  * refactor(perception): update publisher names and connections in Perception and AutowareSample system configurations
  - Renamed publishers in Perception.module.yaml for clarity and consistency, aligning with the new object recognition and segmentation modules.
  - Updated connections in AutowareSample.system.yaml to reflect the new publisher names, ensuring proper data flow between perception and planning components.
  * fix(parameter_set): update parameter values to remove quotes for consistency
  - Changed the value format for 'input_frame' and 'logging_file_path' parameters in sample_system_perception.parameter_set.yaml to remove quotes, ensuring consistency in parameter representation.
  * refactor(parameter_set): remove deprecated object association and tracking parameters
  - Eliminated unused parameters related to object association and tracking from sample_system_perception.parameter_set.yaml to streamline configuration.
  - This cleanup enhances clarity and reduces potential confusion in the parameter set.
  ---------
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
* chore: sync files (`#1338 <https://github.com/autowarefoundation/autoware_launch/issues/1338>`_)
  * chore: sync files
  * style(pre-commit): autofix
  ---------
  Co-authored-by: github-actions <github-actions@github.com>
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
* Contributors: Taekjin LEE, awf-autoware-bot[bot], github-actions

0.51.0 (2026-05-01)
-------------------
* Merge remote-tracking branch 'origin/main' into tmp/bot/bump_version_base
* feat(launch): default data_path to ~/autoware_data/ml_models (`#1835 <https://github.com/autowarefoundation/autoware_launch/issues/1835>`_)
  feat(autoware_launch,tier4_perception_launch,tier4_simulator_launch,autoware_sample_designs): default data_path to ~/autoware_data/ml_models
  Roll the `data_path` arg defaults from `$(env HOME)/autoware_data` to
  `$(env HOME)/autoware_data/ml_models` to match the new
  `~/autoware_data/{maps,ml_models,recordings,scenarios,...}` layout
  defined in `autowarefoundation/autoware#7068 <https://github.com/autowarefoundation/autoware/issues/7068>`_.
  Top-level launches and components:
  - autoware_launch/launch/autoware.launch.xml
  - autoware_launch/launch/e2e_simulator.launch.xml
  - autoware_launch/launch/logging_simulator.launch.xml
  - autoware_launch/launch/planning_simulator.launch.xml
  - autoware_launch/launch/components/tier4_perception_component.launch.xml
  - autoware_launch/launch/components/tier4_planning_component.launch.xml
  - tier4_universe_launch/tier4_perception_launch/launch/perception.launch.xml
  - tier4_universe_launch/tier4_perception_launch/launch/object_recognition/detection/detector/camera_bev_detector.launch.xml
  - autoware_sample_designs/design/system/AutowareSample.system.yaml: also
  migrate the `map_path` values for `Runtime` and `E2ESimulation` from
  `$(env HOME)/autoware_map/<map>` to `$(env HOME)/autoware_data/maps[/demos]/<map>`.
  Simulator chain (traffic-light models for planning_simulator):
  - autoware_launch/launch/components/tier4_simulator_component.launch.xml: declare and pass `data_path`
  - autoware_launch/launch/planning_simulator.launch.xml: forward `data_path` to the simulator component include
  - tier4_universe_launch/tier4_simulator_launch/launch/simulator.launch.xml: declare a `data_path` arg and replace the nine hardcoded `$(env HOME)/autoware_data/{tensorrt_yolox,traffic_light_fine_detector,traffic_light_classifier}/...` strings with `$(var data_path)/<package>/...`, matching the perception convention.
  Users on the legacy layout can pin the old root with
  `data_path:=$HOME/autoware_data` (and override `map_path` to legacy
  `$HOME/autoware_map/<map>` if needed).
  Refs: https://github.com/autowarefoundation/autoware/issues/7068
* chore(autoware_sample_designs): add comprehensive README for autoware_sample_designs package (`#1820 <https://github.com/autowarefoundation/autoware_launch/issues/1820>`_)
  * feat(autoware_sample_designs): add comprehensive README for autoware_sample_designs package
  * style(pre-commit): autofix
  * fix(README): update formatting for code blocks and enhance section headings for clarity
  ---------
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
* feat(autoware_sample_designs): bring back autoware system designer implementations (`#1816 <https://github.com/autowarefoundation/autoware_launch/issues/1816>`_)
* revert(autoware_sample_designs): revert autoware system designer systems (`#1751 <https://github.com/autowarefoundation/autoware_launch/issues/1751>`_) (`#1814 <https://github.com/autowarefoundation/autoware_launch/issues/1814>`_)
  This reverts commit d9260eb283ab3bcb1912560ad944954c636ec269.
* feat(autoware_sample_designs): implement autoware system designer systems (`#1751 <https://github.com/autowarefoundation/autoware_launch/issues/1751>`_)
  * feat: add autoware_common_designs and autoware_sample_deployment packages with initial configurations and design modules
  update package name and host ip for awsim
  feat: add VelodyneLidar module and various design configurations for sensing and system components
  feat: add new parameter sets for perception and logging simulation components
  feat: add sample system sensing parameter set and update launch configurations
  feat: update logging simulation parameters and correct parameter set reference in AutowareSample system
  fix: update compute unit for localization component in AutowareSample system configuration
  fix: update connections for localization component in AutowareSample system configuration
  feat: enhance SampleSystemRobotStatePublisher with input/output message types and update AutowareSample system configuration for ADAPI integration
  feat: add sample system global parameter set and reference it in AutowareSample system configuration
  feat: add GNSS and sensor modules for SampleSensorKit, including UbloxGpsDriver and associated configurations
  feat: add new sensor modules and configurations for SampleSensorKit, including VelodyneLidar and IMU decoding
  feat: introduce SampleRosbagReplay node configuration and update AutowareSample system to include rosbag replay functionality
  fix: update namespace for robot_state_publisher in AutowareSample system configuration
  refactor: remove vehicle status outputs from SampleRosbagReplay configuration and clean up AutowareSample system entity references
  feat: add new Lidar configuration files and update parameter set for sensing nodes in AutowareSample system
  feat: add GNSS decoding module and update SampleSensorKit configuration to include GNSS input
  feat: add SampleSensorKitADAPIWrapper configuration and integrate it into AutowareSample system
  refactor: update output names in SampleSensorKitADAPIWrapper configuration to remove 'api/' prefix and adjust AutowareSample system connections accordingly
  refactor: change 'inheritance' to 'base' in Lidar and SensorKit module configurations for consistency
  Update default host IP in lidar launch file to 255.255.255.255 for improved network configuration
  feat: add new perception and sensing modules including ObjectRecognition, TrafficLightRecognition, and various Lidar and radar configurations
  feat: add new Lidar decoding modules and update SampleSensorKit configurations to integrate VLP16 and VLS128 modules
  refactor: update input topic connections in SampleSensorKitLidar module and remove redundant input_topics parameter from sample_system_sensing configuration
  feat: add SampleAWSIM_in and SampleAWSIM_out node configurations with input/output message types and integrate them into AutowareSample system
  refactor: remove deprecated sensing components and integrate traffic light recognition and vehicle velocity converter into E2ESimulation connections
  feat: add TrafficLightRecognitionRefine_1 module and update AutowareSample system to use the new module while modifying compute unit and connections
  fix: update compute unit for control component in AutowareSample system configuration
  feat: add e2e_simulation.parameter_set for end-to-end simulation configuration and update AutowareSample system to reference the new parameter set
  feat: add LidarVelodyneVLP16_awsim and SampleSensorKitAWSIM modules with updated connections in AutowareSample system for enhanced sensing capabilities
  feat: enhance SampleSensorKitAWSIM module by adding pose_ndt input and imu_data output, and update AutowareSample system connections for improved localization and sensing integration
  feat: update e2e_simulation.parameter_set to enable simulation time and reference it in AutowareSample system for improved simulation accuracy
  refactor: remove unused sensing connections from E2ESimulation in AutowareSample system to streamline configuration and improve clarity
  feat: enhance TrafficLightRecognitionRefine_1 module by adding external interfaces and overriding multi_camera_fusion instance, while updating AutowareSample system connections for improved traffic light recognition capabilities
  feat: add system node configuration in e2e_simulation.parameter_set and update AutowareSample system connections for enhanced system monitoring capabilities
  fix: update imu_data output naming in SampleSensorKit and SampleSensorKitAWSIM modules, and adjust connections in AutowareSample system for consistency and clarity
  fix: add additional connection for imu_raw input in SampleSensorKitImu_decode module to improve data flow in AutowareSample system
  fix: add odometry input and update connections in SampleSensorKitAWSIM module to enhance data integration in AutowareSample system
  update configs to pass linter
  fix: update connections in AutowareSample system to streamline data flow between localization and sensing modules
  fix: increase cloud capacity in centerpoint parameters to enhance object detection performance
  fix: update planning simulator launch configuration and modify AutowareSample system to remove unnecessary components and override occupancy grid map settings for improved simulation performance
  feat: introduce DummyObjectRecognition module for enhanced object recognition capabilities in planning simulation, and update AutowareSample system to integrate new perception module and streamline connections
  feat: add localization module and update connections in AutowareSample system to enhance localization capabilities and improve data flow
  feat: add SimplePlanningSimulator module and update connections in AutowareSample system to enhance simulation capabilities and improve data flow
  feat: add vehicle_cmd_converter module and update connections in AutowareSample system to enhance control command processing and improve data flow
  feat: enhance control and simulation data flow in AutowareSample system by adding new connections for object recognition, obstacle segmentation, and vehicle status inputs
  feat: add planning simulation parameter set and integrate it into PlanningSimulation mode in AutowareSample system
  fix: correct connection from vehicle_velocity_converter output in PlanningSimApiLocalization module
  feat: add new modules for object recognition and localization in planning simulation
  * feat: update planning simulation parameters and connections in AutowareSample system to enhance object recognition and data flow
  feat: remove autoware_common_designs package and add new object recognition modules to enhance perception capabilities in AutowareSample system
  feat: add occupancy grid map parameters to planning simulation for improved obstacle detection and processing
  feat: introduce PlanningSimVehicle module and update connections in AutowareSample system to enhance vehicle command processing and door status integration
  feat: update planning simulation parameters to include is_simulation and enable_all_modules_auto_mode for enhanced simulation control
  fix: update connections in AutowareSample system to use wildcard notation for improved flexibility in data routing
  move module files from autoware_universe
  move wrappers from autoware_universe
  feat: add sample system perception parameter set and remove obsolete universe perception parameter sets to streamline configuration
  refactor: update parameter sets in AutowareSample system to use sample_system_perception for consistency and clarity
  refactor: update parameter set in AutowareSample system to use sample_system_perception for improved clarity
  fix: add remap_target for robot_description output in SampleSystemRobotStatePublisher configuration for improved clarity
  feat: add ObjectDetection and ObjectRecognition modules to enhance perception capabilities and remove obsolete merged modules for improved clarity
  feat: introduce TrafficLightRecognition module and remove obsolete merged modules for improved clarity in perception system design
  feat: remove obsolete RadarDriver modules and introduce TrafficLightRecognition module to enhance perception system design
  feat: remove obsolete HesaiLidar and HesaiLidar_decode modules to streamline lidar system design
  feat: add SampleSensorKitADAPIWrapper and SampleSystemSensingWrapper configurations to enhance sensor integration and streamline system design
  * feat: update autoware_system_design_format to v0.2.0 and add package information for various nodes to enhance system clarity and organization
  feat: enhance system design by updating node configurations and improving package organization for better clarity
  refactor: clean up YAML configurations by removing unnecessary whitespace and improving formatting for better readability
  * chore: update autoware_system_design_format from v0.1.0 to 0.1.0 across multiple YAML configurations for consistency and clarity
  build: add autoware_ament_auto_package to CMakeLists.txt
  chore: resolve mistype and exception
  chore: fix pre-commit.ci issue
  chore: standardize package name from 'AWSIM' to 'awsim' in SampleAwsimIn and SampleAwsimOut node configurations for consistency
  * chore: update YAML configurations to standardize 'external_interfaces' to 'inputs' and 'outputs' for consistency across multiple modules
  chore: update autoware_system_design_format from v0.1.0 to v0.3.0 across multiple YAML configurations for consistency and clarity
  chore: update autoware_system_design_format to v0.3.0 and standardize parameter naming across multiple YAML configurations
  chore: update autoware_system_design_format to v0.3.0 and standardize parameter naming conventions across multiple YAML configuration files
  replace conection input/output
  fix service type
  * Standardize YAML configurations by replacing 'inputs' and 'outputs' with 'subscribers' and 'publishers' across multiple perception and sensing modules for consistency.
  Refactor YAML connection definitions to use a consistent multi-line format across various perception and sensing modules, enhancing readability and maintainability.
  Add 'launch' configuration with 'use_container: true' for multiple perception and sensing modules to enable containerized execution.
  Add container name 'pointcloud_container' to sensing modules in AutowareSample.system.yaml for improved containerized execution.
  Remove 'use_container' option from TamagawaImuCanDriver node configuration to streamline YAML settings for improved clarity.
  Update autoware_system_design_format to v0.3.1 and replace 'launch' with 'arguments' for container configuration across multiple perception and sensing modules to enhance clarity and consistency.
  Remove shape estimation node from CameraLidarFusion module and add 'run_convex_hull_conversion' parameter to detected_object_feature_remover for enhanced functionality in perception system.
  Add new node configurations for AutowareGlogComponent, Ros2Container, and Ros2ContainerMultiThread in YAML format to enhance system design and modularity.
  Update AutowareSample.system.yaml to replace 'container_name' with 'container_target' for consistency in container configuration across multiple modules.
  Update AutowareSample.system.yaml to change 'pointcloud_container' references to '/pointcloud_container' and update the entity name for the pointcloud container to 'Ros2ContainerMultiThread.node' for improved clarity and consistency in container configuration.
  * Refactor YAML configurations to remove 'use_container' arguments for clarity and add a new process group for pointcloud container management.
  Add pointcloud_container node group with multi-threaded configuration for improved processing
  Refactor pointcloud_container configuration to use multi-threaded node group for improved performance
  Remove deprecated Ros2Container.node.yaml and Ros2ContainerMultiThread.node.yaml files to streamline container configuration.
  Remove glog_component from pointcloud_container node group to streamline configuration
  Update node paths in parameter set YAML files for consistency with ROS2 structure
  Update node specification in logging_simulation.parameter_set.yaml for clarity
  Update namespaces in AutowareSample.system.yaml for consistency and clarity
  Rename occupancy_grid_map to occupancy_grid_map_node for consistency across module configurations
  Update component paths in AutowareSample.system.yaml for consistency with ROS2 structure
  Rename occupancy_grid_map to occupancy_grid_map_node for consistency in parameter set
  update deployment centerpoint config
  format fix
  * Update autoware_system_designer to v0.3.1 in workflows and pre-commit config
  Add autoware_system_designer repository to build dependencies
  Fix indentation for autoware_system_designer entry in build dependencies
  Fix function call for ament_auto_package in CMakeLists.txt
  Fix entity references in DummyObjectRecognition.module.yaml
  Add node_groups section for pointcloud_container in PlanningSimulation
  Rename object_recognition/detection to object_recognition_detection in DummyObjectRecognition.module.yaml
  Update entity references for LidarVelodyne modules to include online/offline suffixes
  Fix trigger condition in Ros2SocketCanSenderNode configuration
  Remove PointcloudContainer.node.yaml configuration file from the autoware_common_designs deployment.
  Update version to 0.50.0 and add new maintainers for autoware_common_designs and autoware_sample_deployment packages
  merge autoware_common_designs package to autoware_sample_deployment.  update project name in autoware_sample_deployment to autoware_sample_designs.
  rename folder autoware_sample_deployment to autoware_sample_designs
  * Update message type for traffic_signals in Tier4PlanningWrapper.node.yaml to use autoware_perception_msgs for consistency with other modules.
  * Revert "Update message type for traffic_signals in Tier4PlanningWrapper.node.yaml to use autoware_perception_msgs for consistency with other modules."
  This reverts commit aa48a108611a0b7d24337c0f4d1dec0a4776c061.
  * Refactor CMakeLists.txt to remove dummy library and replace it with a custom target for improved build configuration. Delete empty.cpp file as it is no longer needed.
  * Remove pilot-auto provider from workspace.yaml since the awf system does not rely on.
  * Update message type for traffic_signals in Tier4PlanningWrapper.node.yaml to use autoware_perception_msgs for consistency with other modules.
  ---------
* Contributors: Mete Fatih Cırıt, Taekjin LEE, github-actions
