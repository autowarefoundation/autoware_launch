# autoware_sample_designs

`autoware_sample_designs` is a complete, working example of defining and launching a full Autoware system using [Autoware System Designer](https://github.com/autowarefoundation/autoware_system_designer/blob/main/README.md).

It serves as the reference implementation for how to describe, configure, and deploy an Autoware autonomous driving stack using YAML-based system design — without writing ROS 2 launch files by hand.

## What is Autoware System Designer?

Autoware System Designer is a toolset that lets you describe your entire software system in structured YAML files and automatically generates correct-by-construction ROS 2 launch files, parameter templates, and system visualization diagrams from them.

![Configuration Structure](https://raw.githubusercontent.com/autowarefoundation/autoware_system_designer/main/autoware_system_designer/doc/images/configuration_structure.png)

A system is composed of four entity types:

| Entity | File | Description |
|---|---|---|
| **Node** | `*.node.yaml` | A single ROS 2 node — its topics, parameters, and execution config |
| **Module** | `*.module.yaml` | A reusable group of nodes/sub-modules with wired connections |
| **System** | `*.system.yaml` | The top-level description: components, connections, modes |
| **Parameter Set** | `*.parameter_set.yaml` | Parameter overrides applied to nodes at deployment time |

The build step collects node definitions from across the workspace (resolved via `workspace.yaml`), combines them with the system design files in this package, and generates launch files and a system visualization:

```
each ROS 2 package (core, universe, …)         ┐
  └── design/node/  *.node.yaml                │
                                               ├── autoware_system_designer
this package                                   │
├── design/node/    *.node.yaml  (optional)    │
├── design/module/  *.module.yaml              │
├── design/system/  *.system.yaml              │
└── design/parameter_set/ *.parameter_set.yaml ┘
                                               │
                                               ▼
                                               install/
                                               ├── systems.html
                                               ├── AutowareSample/
                                               │   ├── Runtime.launch.xml
                                               │   ├── LoggingSimulation.launch.xml
                                               │   └── PlanningSimulation.launch.xml
                                               └── ...node launchers...
```

The generated node diagram shows the full system topology including all modules, nodes, and topic connections:

![Node Diagram](https://raw.githubusercontent.com/autowarefoundation/autoware_system_designer/main/autoware_system_designer/doc/images/node_diagram.png)

## Visualizing the System

After building, open the generated HTML page in your browser to interactively explore the full system graph:

```sh
# From your colcon workspace root
firefox install/systems.html
# or
google-chrome install/systems.html
```

The visualization page shows every node, module, topic connection, and operating mode defined in the system. 

## Package Structure

```
autoware_sample_designs/
├── design/
│   ├── node/              # System-specific node definitions (wrapper/optional nodes only)
│   ├── module/            # Reusable subsystem modules (sensing, perception, …)
│   ├── system/
│   │   └── AutowareSample.system.yaml   # Top-level system definition
│   └── parameter_set/     # Parameter overrides per node, per mode
├── config/                # Shared parameter YAML files (owned by this package)
├── launch/                # Hand-written wrapper launch files for legacy integration
├── urdf/                  # Vehicle URDF
├── workspace.yaml         # Provider resolution rules for the system designer
└── CMakeLists.txt
```

The `AutowareSample.system.yaml` defines four operating modes — `Runtime`, `LoggingSimulation`, `PlanningSimulation`, and `E2ESimulation` — each with mode-specific component overrides and connections.

## Relationship with `autoware_launch`

`autoware_sample_designs` is **independent** of `autoware_launch` as a system builder.
The system designer generates its own launch files and manages its own system topology.
`autoware_launch` does **not** control how this system is structured or launched.

The only connection is that some **parameter set files** reference shared parameter YAML files from `autoware_launch` (e.g., tracker, fusion, and filter configs that are common across vehicle variants):

```yaml
# design/parameter_set/sample_system_perception.parameter_set.yaml
parameters:
  - node: /perception/object_recognition/detection/clustering/...
    param_files:
      # shared param file from autoware_launch — values only, no launch dependency
      - param_path: $(find-pkg-share autoware_launch)/config/perception/...
```

This is a runtime parameter file lookup — `autoware_launch` is not involved in building or launching the system. Node-specific parameters that belong to this vehicle configuration live in `config/` inside this package itself.

## Building

In your colcon workspace:

```sh
source /opt/ros/$ROS_DISTRO/setup.bash
colcon build --symlink-install \
    --packages-select autoware_sample_designs \
    --cmake-args -DCMAKE_BUILD_TYPE=Release
```

This invokes `autoware_system_designer_build_deploy()` from `CMakeLists.txt`, which processes `AutowareSample.system.yaml` and generates all launch files under `install/`.

## Launching the System

The exact launch command for each mode is shown in the **Launcher** section of the visualization page (`install/systems.html`).
Use that as the authoritative source — launch file names like `Runtime.launch.xml` are not unique and the same name may be generated by other system design packages in your workspace, so copying a bare `ros2 launch <package> <mode>.launch.xml` from documentation can silently run the wrong system.

In the visualization page, navigate to the deploy system (`AutowareSample`), open the **Launch** section, and copy the command for the desired mode:

```sh
# From your colcon workspace root
firefox install/systems.html
```

## Introducing System Designer to Your Own System

Use `autoware_sample_designs` as the template. The steps to create a new system design package are:

**1. Create a ROS 2 package with the standard layout**

```
my_vehicle_designs/
├── design/
│   ├── node/
│   ├── module/
│   ├── system/
│   │   └── MyVehicle.system.yaml
│   └── parameter_set/
├── config/
├── workspace.yaml
├── CMakeLists.txt
└── package.xml
```

**2. Declare the dependency in `package.xml`**

```xml
<depend>autoware_system_designer</depend>
```

**3. Wire up the build in `CMakeLists.txt`**

```cmake
find_package(autoware_system_designer REQUIRED)

# Generate launch files from MyVehicle.system.yaml
autoware_system_designer_build_deploy(
  ${PROJECT_NAME}
  MyVehicle.system
  PRINT_LEVEL=WARNING
  STRICT=AUTO
)
```

**4. Define your system**

Node definitions (`*.node.yaml`) live in each individual ROS 2 package (under `core/`, `universe/`, etc.) and are resolved automatically via `workspace.yaml` — you do not need to copy them.
Only add files to `design/node/` for nodes that are unique to your system package (e.g., wrapper nodes or optional system-level nodes not defined elsewhere).

Compose these nodes into modules and a top-level system in your `*.system.yaml`. Add `*.parameter_set.yaml` files to override parameters for your specific hardware.

**5. Build and visualize**

```sh
colcon build --packages-select my_vehicle_designs
firefox install/systems.html
```

The generated `install/systems.html` gives you a full interactive diagram of your system before running a single node. Use it to verify connections, inspect parameters, and review mode differences.
