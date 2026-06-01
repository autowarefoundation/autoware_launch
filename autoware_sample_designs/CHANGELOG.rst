^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package autoware_sample_designs
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

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
