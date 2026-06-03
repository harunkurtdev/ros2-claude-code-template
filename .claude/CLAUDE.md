# ROS 2 + Gazebo Clean Architecture Project — Claude Context

This file is the persistent orientation note Claude reads on every
session in this workspace. Keep it short; deep content lives in
`skills/`, `rules/`, and `agents/`.

## What this template is

A reusable `.claude/` configuration for **two adjacent worlds** that
robotics teams routinely cross between:

1. **ROS 2 / Nav 2** workspaces (Python + C++) that follow **Clean
   Architecture** — colcon, ament, pluginlib.
2. **Gazebo Sim (`gz-sim`)** source trees (C++) that follow the
   **Entity-Component-System** model — cmake / ninja / bazel, plugin
   registration via `GZ_ADD_PLUGIN`.

Both sides ship as **prefix-disambiguated** skills, slash commands and
sub-agents in the same `.claude/` tree. When you start a session, pick
the prefix that matches the project you're touching:

| Concern                          | ROS 2 side                | gz-sim side               |
|----------------------------------|---------------------------|---------------------------|
| Build                            | `/build`                  | `/gz-build`               |
| Test                             | `/test`                   | `/gz-test`                |
| Lint                             | `/lint`                   | `/gz-lint`                |
| Scaffold a unit                  | `/new-package` / `/new-node` | `/gz-new-component` / `/gz-new-system` |
| Changelog                        | `/changelog`              | `/gz-changelog`           |
| Pre-PR review agent              | `ros2-style-reviewer`     | `gz-style-reviewer`       |
| Architectural advisor            | `clean-arch-architect`    | `ecs-architect`           |
| Architecture rule of thumb       | "domain has no ROS deps"  | "data in components, behaviour in systems" |

## Project layout

A package generated through `/new-package` follows:

```
src/<pkg>/<pkg>/
├── domain/           # entities, value objects, ports (no ROS deps)
├── application/      # use cases (depends only on domain)
├── infrastructure/   # rclpy / rclcpp nodes, TF, repositories
└── presentation/     # CLI, launch entrypoints
```

For C++ the same separation lives under `include/<pkg>/<layer>/` and
`src/<layer>/`.

## Where things live

| You need to … | Look at |
|---------------|---------|
| Understand layer boundaries           | `rules/clean_architecture.md` |
| Bootstrap a whole workspace from zero | `commands/new-workspace.md` + `skills/new_ros2_workspace/SKILL.md` |
| Add a new package                     | `commands/new-package.md` + `skills/new_ros2_package/SKILL.md` |
| Add a node                            | `commands/new-node.md` + `skills/ros2_node_creation/SKILL.md` |
| Add a lifecycle node                  | `skills/ros2_lifecycle/SKILL.md` |
| Add a launch file                     | `commands/new-launch.md` + `skills/ros2_launch_config/SKILL.md` |
| Add a Nav 2 plugin                    | `commands/new-nav2-plugin.md` + `skills/new_nav2_plugin/SKILL.md` |
| Pick QoS / topic naming               | `rules/ros2_communication.md` |
| Write tests                           | `rules/testing.md` + `skills/ros2_testing/SKILL.md` |
| Write a ros2_control controller       | `rules/ros2_control_architecture.md` + `skills/ros2_controller_creation/SKILL.md` |
| Look up a ros2_controllers package    | `rules/ros2_controllers_reference.md` (full catalog) |
| Write a ros2_control hardware component / bring up a robot | `skills/ros2_control_hardware_interface/SKILL.md` + `rules/ros2_control_demos.md` |
| Find a runnable ros2_control example   | `rules/ros2_control_demos.md` (17-example map) |
| Write a behavior-tree node (BT.CPP / ROS 2) | `rules/behaviortree_cpp.md` + `rules/behaviortree_ros2.md` + `skills/behaviortree_node_creation/SKILL.md` |
| Interface a Webots robot/sensor with ROS 2 | `rules/webots_ros2_architecture.md` + `skills/webots_ros2_device_plugin/SKILL.md` |
| Bridge a VDA 5050 fleet interface     | `rules/vda5050_protocol.md` + `skills/vda5050_integration/SKILL.md` |
| Look up any VDA 5050 message/field    | `rules/vda5050_messages.md` (complete spec) |
| Generate VDA 5050 code (which format?) | `rules/vda5050_implementation_formats.md` (pydantic / ROS msg / C++ idioms + v2→v3 diffs) |
| Design something — which layer?       | Agent `clean-arch-architect` |
| Review a diff before PR               | Agent `ros2-style-reviewer` |
| Add a new command / skill / agent / rule | `skills/extending_claude_config/SKILL.md` + `/new-command` `/new-skill` `/new-agent` |

## Slash commands

| Command | Purpose |
|---------|---------|
| `/build [pkg]`            | `colcon build` wrapper, `--symlink-install`, `RelWithDebInfo`. |
| `/test [pkg] [filter]`    | `colcon test` + `test-result --all` with a clean summary. |
| `/lint [--all|files]`     | `pre-commit` + ament linters on changed files. |
| `/new-workspace <path> [prefix]` | Bootstrap a complete colcon workspace from zero. |
| `/new-package <name> <py|cpp>` | Scaffold a Clean Architecture package. |
| `/new-node <pkg> <name> <kind> <lang>` | Scaffold a node. |
| `/new-launch <pkg> <name>`      | Scaffold a modular launch file. |
| `/new-nav2-plugin <kind> <Class>` | Scaffold a Nav 2 plugin. |
| `/new-controller <pkg> <Class> <plain\|chainable\|broadcaster>` | Scaffold a ros2_control controller / broadcaster. |
| `/new-hardware <pkg> <Class> <system\|actuator\|sensor>` | Scaffold a ros2_control hardware component + URDF + bringup. |
| `/new-bt-node <pkg> <Class> <kind>` | Scaffold a BehaviorTree.CPP / BehaviorTree.ROS2 leaf node. |
| `/new-vda5050-connector <pkg> [py\|cpp] [fleet\|robot]` | Scaffold a Clean-Architecture VDA 5050 connector. |
| `/new-webots-plugin <pkg> <Class> <cpp\|python> [device]` | Scaffold a webots_ros2 device/robot plugin + URDF + launch. |
| `/new-skill <name>` / `/new-command <name>` / `/new-agent <name>` | Extend this `.claude/` config with a new asset (self-extensibility). |
| `/changelog [base] [pkg]` | Generate a CHANGELOG.rst block from commits. |
| `/gz-build [extra cmake args]` | `cmake -B build -G Ninja -DCMAKE_BUILD_TYPE=RelWithDebInfo` + build. |
| `/gz-test [ctest regex]`  | `ctest --test-dir build` filtered by name. |
| `/gz-lint [--all\|files]` | `pre-commit` over changed-vs-main in a gz-* repo. |
| `/gz-new-component <Name>` | Scaffold an ECS component header. |
| `/gz-new-system <name> [Class]` | Scaffold a full gz-sim system plugin. |
| `/gz-changelog [base]`    | Generate a Changelog.md block from commits. |

`commands/ros2.md` and `commands/nav2.md` are quick references, not
executables — read them when you need the cheatsheet.

## Sub-agents

| Agent | When to use |
|-------|-------------|
| `ros2-style-reviewer` | Before opening a ROS 2 PR — Clean Architecture, lifecycle, QoS, pluginlib, tests, build manifests. |
| `clean-arch-architect`| Before writing ROS 2 code — node vs use case, topic vs service vs action, compose vs split. |
| `gz-style-reviewer`   | Before opening a gz-sim PR — ECS conventions, `GZ_ADD_PLUGIN`, CMake/Bazel parity, Migration.md / Changelog.md drift. |
| `ecs-architect`       | Before writing gz-sim code — where new state lives (component vs PImpl), which system phase, threading. |
| `vda5050-reviewer`    | Before opening a VDA 5050 connector PR — protocol compliance (topics, QoS, header, base/horizon, action state machine, schemas) + bridge Clean Architecture. |
| `ros2-controllers-reviewer` | Before opening a ros2_control controller / broadcaster / hardware-component PR — lifecycle, command/state interface config, `update()`/`read()`/`write()` real-time safety, `generate_parameter_library`, pluginlib export, chainable correctness, URDF bringup, tests. |
| `behaviortree-reviewer` | Before opening a BehaviorTree.CPP / BehaviorTree.ROS2 PR — node base-class choice, non-blocking ticks, ports/blackboard typing, factory/plugin registration, XML v4, ROS 2 wrapper contract. |
| `webots-ros2-reviewer` | Before opening a webots_ros2 PR — plugin `init/step` contract (no `robot.step()`), URDF `<webots>` device/plugin wiring, `Ros2ControlSystem`, `WebotsLauncher`/`WebotsController` launch. |

## Skills index

### ROS 2 core

| Skill                  | Topic |
|------------------------|-------|
| `ros2_node_creation`   | Clean-arch compliant node (Py/C++) |
| `ros2_lifecycle`       | Managed lifecycle nodes |
| `ros2_messaging`       | Pub/Sub, thread-safe buffers, synchronization |
| `ros2_service_action`  | Service/Action server + client wrappers |
| `ros2_launch_config`   | Modular launch files |
| `ros2_transforms`      | TF2 wrappers without leaking `geometry_msgs` into domain |
| `ros2_diagnostics`     | `diagnostic_updater` integration |
| `ros2_bag`             | rosbag2 record / replay |
| `ros2_testing`         | Unit, integration, launch_testing |
| `new_ros2_package`     | Scaffold recipe for `/new-package` |
| `new_ros2_workspace`   | Bootstrap a whole colcon workspace from zero (`/new-workspace`) |

### Nav 2

| Skill                   | Topic |
|-------------------------|-------|
| `nav2_core_interfaces`  | Controller / GlobalPlanner / Smoother / GoalChecker / ProgressChecker / Behavior bases |
| `nav2_controllers`      | DWB (13 critics), MPPI (11 critics), RPP, Graceful, Rotation Shim |
| `nav2_planners`         | NavFn, SMAC (2D / Hybrid-A* / Lattice), Theta*, Route Server |
| `nav2_behavior_tree`    | 82 BT nodes, 15 default trees |
| `nav2_costmap`          | Static, Obstacle, Voxel, Inflation, Range, Denoise + filters |
| `nav2_localization`     | AMCL, Map Server |
| `nav2_servers`          | Lifecycle Manager, Velocity Smoother, Collision Monitor, Docking, Following, Python API |
| `new_nav2_plugin`       | Scaffold recipe for `/new-nav2-plugin` |

### ros2_control

| Skill                       | Topic |
|-----------------------------|-------|
| `ros2_controller_creation`  | Write/extend a ros2_control controller or broadcaster (base class, interfaces, lifecycle, RT-safe `update()`, params, pluginlib) |
| `ros2_control_hardware_interface` | Write a hardware component (System/Actuator/Sensor) + URDF `<ros2_control>` + bringup (from ros2_control_demos) |

### Behavior trees

| Skill                          | Topic |
|--------------------------------|-------|
| `behaviortree_node_creation`   | Write a BT leaf node — plain BehaviorTree.CPP (Sync/Stateful/Condition) or BehaviorTree.ROS2 wrappers (action/service/topic), ports, registration, XML v4 |

### Fleet interface

| Skill                  | Topic |
|------------------------|-------|
| `vda5050_integration`  | Bridge a VDA 5050 v3.0.0 fleet-control interface (MQTT/JSON) onto Nav 2 under Clean Architecture |

### gz-sim / Gazebo

| Skill              | Topic |
|--------------------|-------|
| `gz-build`         | cmake / ninja, colcon, bazel build recipes for gz-sim |
| `gz-test`          | ctest filters, gtest binary directly, headless / xvfb, performance |
| `gz-ecs-overview`  | The Server / SimulationRunner / ECM / System loop in 5 minutes |
| `new-component`    | Component header template (with or without custom serializer) |
| `new-system`       | Full system plugin scaffold — header, source, CMake, plugin registration, integration test |

### Webots simulation

| Skill                       | Topic |
|-----------------------------|-------|
| `webots_ros2_device_plugin` | Interface a Webots robot/sensor with ROS 2 — device plugin (C++/Python), URDF `<webots>` wiring, ros2_control, launch (`/new-webots-plugin`) |

### Meta — extending this template

| Skill                     | Topic |
|---------------------------|-------|
| `extending_claude_config` | Author a new rule / skill / command / agent for this `.claude/` config and index it (`/new-skill`, `/new-command`, `/new-agent`) |

## Rules

| Rule file               | What it constrains |
|-------------------------|--------------------|
| `clean_architecture.md` | Layer dependencies; what is allowed to import what |
| `ros2_general.md`       | Project-wide ROS 2 prescriptions |
| `ros2_nodes.md`         | Node design (lifecycle, callback groups, parameters) |
| `ros2_communication.md` | Topic naming, QoS, custom interfaces |
| `testing.md`            | Unit / integration / launch coverage requirements |
| `nav2_architecture.md`  | Nav 2 system diagram, data flow |
| `nav2_parameters.md`    | Canonical Nav 2 parameter reference |
| `nav2_msgs_reference.md`| Nav 2 msgs / srvs / actions |
| `ros2_control_architecture.md` | ros2_control framework — controller_manager, lifecycle, command/state interfaces, chainable controllers, RT rules |
| `ros2_controllers_reference.md`| ros2_controllers package catalog (all controllers + broadcasters) |
| `ros2_control_demos.md` | ros2_control_demos catalog (17 examples) + hardware-component & bringup learning map |
| `behaviortree_cpp.md`   | BehaviorTree.CPP v4 core — node hierarchy, NodeStatus, factory, ports/blackboard, XML v4, loggers |
| `behaviortree_ros2.md`  | BehaviorTree.ROS2 — action/service/topic wrappers + TreeExecutionServer |
| `webots_ros2_architecture.md` | webots_ros2 — driver, URDF `<webots>` device/plugin mechanism, device plugins, ros2_control bridge, launch, importer |
| `vda5050_protocol.md`   | VDA 5050 v3.0.0 fleet interface — MQTT topics, message overview, action state machine |
| `vda5050_messages.md`   | VDA 5050 v3.0.0 **complete** message spec (all 8 messages, every field) + communication processes |
| `vda5050_implementation_formats.md` | VDA 5050 code-gen format analysis — 3 reference idioms (pydantic / ROS `.msg`+bridge / C++ structs) + v2→v3 differences |
| `robot_specific.md`     | Robot-level overrides — replace per project |

## Conventions worth remembering

* **Domain code does not import `rclpy`, `rclcpp`, or any `*_msgs`
  package.** If it needs ROS, it is not domain.
* **Lifecycle by default** for anything owning a resource (sensor,
  actuator, costmap, hardware bridge).
* **`declare_parameter` for every parameter** — silent
  `get_parameter` on undeclared names is a bug.
* **QoS matches semantics**: sensor data = best-effort, commands =
  reliable, latched config = transient-local.
* **Tests run via `colcon test`**; never `sleep(N)` to synchronize —
  use futures, conditions, or `launch_testing.ReadyToTest`.
* **Pluginlib aliases never change** once a release is cut.

## Common commands

```bash
# Bring the workspace up
colcon build --symlink-install
source install/setup.bash

# Quick check
colcon test --packages-select <pkg>
colcon test-result --all

# Bringup
ros2 launch nav2_bringup tb3_simulation_launch.py
```

See `.claude/commands/ros2.md` and `.claude/commands/nav2.md` for the
full reference card.
