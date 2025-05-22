# Autoware Launch Arguments

This document describes the primary launch arguments for the main `autoware.launch.xml` file, which is the typical entry point for launching the Autoware software stack. These arguments allow you to configure various aspects of Autoware to suit your specific map, vehicle, sensor setup, and operational needs.

## How to Use

You can pass these arguments to the `ros2 launch` command. For example:

```bash
ros2 launch autoware_launch autoware.launch.xml map_path:=/path/to/your/map vehicle_model:=your_vehicle sensor_model:=your_sensor_kit use_sim_time:=true
```

---

## I. Essential Parameters

These parameters are fundamental for running Autoware and often need to be specified.

*   **`map_path`**
    *   **Description:** Specifies the absolute path to the directory containing map data, which should include both point cloud map files (e.g., `.pcd`) and Lanelet2 map files (e.g., `.osm`).
    *   **Expected Value:** String (file system path).
    *   **Example:** `/home/user/autoware_maps/my_town`
    *   **Default:** None (must be provided).

*   **`vehicle_model`**
    *   **Description:** Name of the vehicle model to be used. This selects vehicle-specific parameters and configurations (e.g., URDF, dimensions, control settings). Corresponding vehicle description and launch packages should be installed.
    *   **Expected Value:** String.
    *   **Example:** `lexus_rx_450h`, `sample_vehicle`
    *   **Default:** `sample_vehicle`

*   **`sensor_model`**
    *   **Description:** Name of the sensor kit/model to be used. This selects sensor-specific parameters and configurations (e.g., sensor drivers, calibration, TF tree). Corresponding sensor kit description and launch packages should be installed.
    *   **Expected Value:** String.
    *   **Example:** `aip_xx1`, `sample_sensor_kit`
    *   **Default:** `sample_sensor_kit`

*   **`data_path`**
    *   **Description:** Path to a directory where Autoware components can store or load data artifacts (e.g., learned models for perception).
    *   **Expected Value:** String (file system path).
    *   **Default:** `$(env HOME)/autoware_data`

*   **`pointcloud_container_name`**
    *   **Description:** Name of the composable node container used for point cloud processing nodes. Advanced users might change this for specific performance tuning.
    *   **Expected Value:** String.
    *   **Default:** `pointcloud_container`

---

## II. Module Presets

These arguments allow selecting pre-defined sets of parameters for major Autoware subsystems.

*   **`planning_module_preset`**
    *   **Description:** Selects a parameter preset for the planning modules (e.g., behavior planning, path planning). Presets are defined in the configuration files.
    *   **Expected Value:** String (e.g., `default`, `safe_driving`, `performance_oriented`).
    *   **Default:** `default`

*   **`control_module_preset`**
    *   **Description:** Selects a parameter preset for the control modules (e.g., trajectory following, lateral/longitudinal controllers). Presets are defined in the configuration files.
    *   **Expected Value:** String.
    *   **Default:** `default`

---

## III. Module Launch Toggles

These boolean arguments allow you to enable or disable the launch of entire Autoware subsystems. This is useful for running partial stacks or for debugging. All default to `true`.

*   `launch_vehicle`: Launch vehicle-specific components (interface, model).
*   `launch_system`: Launch system-level components (monitoring, diagnostics).
*   `launch_map`: Launch map loading and processing components.
*   `launch_sensing`: Launch sensor drivers and pre-processing.
    *   `launch_sensing_driver`: (Sub-argument) Specifically toggle sensor drivers.
*   `launch_localization`: Launch localization algorithms.
*   `launch_perception`: Launch perception algorithms (object detection, traffic light recognition).
*   `launch_planning`: Launch motion planning algorithms.
*   `launch_control`: Launch vehicle control algorithms.
*   `launch_api`: Launch the Autoware API interface.

---

## IV. Global Configuration

These arguments affect the overall behavior of Autoware.

*   **`use_sim_time`**
    *   **Description:** Set to `true` when running Autoware with a simulator that publishes simulation time on the `/clock` topic. Set to `false` for real-vehicle operation.
    *   **Expected Value:** `true` or `false`.
    *   **Default:** `false`

*   **`is_simulation`**
    *   **Description:** Informs Autoware that it is operating in a simulation environment. Some modules might adjust their behavior based on this flag (e.g., different sensor noise models, relaxed safety constraints).
    *   **Expected Value:** `true` or `false`.
    *   **Default:** `false`

---

## V. Vehicle Specific Configuration

*   **`vehicle_id`**
    *   **Description:** A specific identifier for the vehicle, which might be used for multi-vehicle systems or logging.
    *   **Expected Value:** String.
    *   **Default:** Value of the `VEHICLE_ID` environment variable, or `default` if not set.

*   **`launch_vehicle_interface`**
    *   **Description:** Whether to launch the specific vehicle interface responsible for communication with the vehicle hardware/simulator.
    *   **Expected Value:** `true` or `false`.
    *   **Default:** `true`

---

## VI. Map Specific Configuration

These are used if you need to specify map file names different from the defaults within the `map_path`.

*   **`lanelet2_map_file`**
    *   **Description:** The name of the Lanelet2 map file (usually an OSM file) located within the `map_path`.
    *   **Expected Value:** String (filename).
    *   **Default:** `lanelet2_map.osm`

*   **`pointcloud_map_file`**
    *   **Description:** The name of the point cloud map file (usually a PCD file) located within the `map_path`.
    *   **Expected Value:** String (filename).
    *   **Default:** `pointcloud_map.pcd`

---

## VII. System Specific Configuration

*   **`system_run_mode`**
    *   **Description:** Defines the run mode for system-level components, potentially affecting logging or diagnostic levels.
    *   **Expected Value:** String (e.g., `online`, `offline`, `debug`).
    *   **Default:** `online`

*   **`launch_system_monitor`**
    *   **Description:** Whether to launch the system monitoring tools.
    *   **Expected Value:** `true` or `false`.
    *   **Default:** `true`

*   **`diagnostic_graph_aggregator_graph_path`**
    *   **Description:** Path to the configuration file for the diagnostic graph aggregator, which visualizes system health.
    *   **Expected Value:** String (file system path).
    *   **Default:** Path to `autoware_launch/config/system/diagnostics/autoware-main.yaml`

---

## VIII. Tools and Visualization

*   **`rviz`**
    *   **Description:** Whether to launch RViz, the 3D visualization tool.
    *   **Expected Value:** `true` or `false`.
    *   **Default:** `true`

*   **`rviz_config_name`**
    *   **Description:** Specifies the name of the RViz configuration file to be loaded from the `autoware_launch/rviz/` directory.
    *   **Expected Value:** String (filename, e.g., `autoware.rviz`).
    *   **Default:** `autoware.rviz`

*   **`rviz_config`**
    *   **Description:** Allows specifying the full path to an RViz configuration file, overriding `rviz_config_name`.
    *   **Expected Value:** String (file system path).
    *   **Default:** Derived from `rviz_config_name`.

*   **`rviz_respawn`**
    *   **Description:** If `true`, RViz will automatically restart if it closes unexpectedly.
    *   **Expected Value:** `true` or `false`.
    *   **Default:** `true`

---

## IX. Perception Specific Configuration

*   **`perception_mode`**
    *   **Description:** Selects the primary mode for perception, determining which sensor inputs and fusion strategies are prioritized.
    *   **Expected Value:** String (e.g., `lidar`, `camera_lidar_fusion`, `camera_lidar_radar_fusion`, `radar`).
    *   **Default:** `lidar`

*   **`traffic_light_recognition/use_high_accuracy_detection`**
    *   **Description:** Enables or disables the use of a high-accuracy (potentially more resource-intensive) model for traffic light detection.
    *   **Expected Value:** `true` or `false`.
    *   **Default:** `true`

---

## X. Auto Mode Configuration

*   **`enable_all_modules_auto_mode`**
    *   **Description:** When set to `true`, attempts to start all relevant Autoware modules in autonomous operation mode from the beginning. Use with caution.
    *   **Expected Value:** `true` or `false`.
    *   **Default:** `false`

---

This list covers the most common arguments. Some specific components launched by `autoware.launch.xml` might have their own, more detailed parameters defined in their respective YAML configuration files.
