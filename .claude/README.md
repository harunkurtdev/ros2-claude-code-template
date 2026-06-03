# .claude — ROS 2 / Nav 2 + Gazebo Sim Clean Architecture Template

This folder is the persistent configuration that lets Claude Code work
on ROS 2 (especially Nav 2) **and** Gazebo Sim (gz-sim, the ECS C++
side) projects while staying faithful to Clean Architecture principles.
When you start a new project you can copy this template as `.claude/`
and use it directly.

> Two parallel sets run side by side:
> * **ROS 2 / Nav 2** side → colcon, ament, pluginlib, Clean
>   Architecture layers, Python+C++.
> * **gz-sim / Gazebo** side → cmake / ninja / bazel, Entity-Component-
>   System, plugin scaffolding.
>
> Command names are separated by a **prefix**: `/build` (colcon) ⟷ `/gz-build`
> (cmake); `/test` ⟷ `/gz-test`; `ros2-style-reviewer` ⟷
> `gz-style-reviewer`; `clean-arch-architect` ⟷ `ecs-architect`.
> Call the one you mean on purpose; both are loaded and active.

## Quick overview

```
.claude/
├── README.md            # this file (English overview)
├── CLAUDE.md            # persistent project context loaded into Claude
├── settings.json        # permissions / hooks / env variables
├── commands/            # /slash commands (invoked from the terminal)
│   │  ── ROS 2 / Nav 2 side ──
│   ├── build.md             — /build           colcon build wrapper
│   ├── test.md              — /test            colcon test + result
│   ├── lint.md              — /lint            ament_* + pre-commit
│   ├── new-package.md       — /new-package     ROS 2 package scaffold
│   ├── new-node.md          — /new-node        Node scaffold
│   ├── new-launch.md        — /new-launch      Launch file scaffold
│   ├── new-nav2-plugin.md   — /new-nav2-plugin Nav 2 plugin scaffold
│   ├── changelog.md         — /changelog       CHANGELOG.rst for the branch
│   ├── ros2.md              —                  ROS 2 reference card
│   ├── nav2.md              —                  Nav 2 reference card
│   │  ── gz-sim / Gazebo side ──
│   ├── gz-build.md          — /gz-build        cmake + ninja gz-sim build
│   ├── gz-test.md           — /gz-test         ctest / integration tests
│   ├── gz-lint.md           — /gz-lint         pre-commit (gz-sim)
│   ├── gz-new-component.md  — /gz-new-component ECS component scaffold
│   ├── gz-new-system.md     — /gz-new-system   ECS system plugin scaffold
│   └── gz-changelog.md      — /gz-changelog    gz-sim Changelog.md block
├── skills/              # long-form recipes Claude consults
│   │  ── ROS 2 ──
│   ├── ros2_node_creation/        — Clean-arch compliant node
│   ├── ros2_lifecycle/            — Managed lifecycle node
│   ├── ros2_messaging/            — Pub / Sub patterns
│   ├── ros2_service_action/       — Service & Action wrapping
│   ├── ros2_launch_config/        — Modular launch
│   ├── ros2_transforms/           — TF2 management
│   ├── ros2_diagnostics/          — Health monitoring
│   ├── ros2_bag/                  — Rosbag record/replay
│   ├── ros2_testing/              — GTest / pytest / launch_testing
│   ├── new_ros2_package/          — Package scaffold recipe
│   │  ── Nav 2 ──
│   ├── new_nav2_plugin/           — Nav 2 plugin scaffold recipe
│   ├── nav2_core_interfaces/      — Plugin base classes
│   ├── nav2_controllers/          — DWB / MPPI / RPP …
│   ├── nav2_planners/             — NavFn / SMAC / Theta* …
│   ├── nav2_behavior_tree/        — BT nodes
│   ├── nav2_costmap/              — Costmap layers
│   ├── nav2_localization/         — AMCL / Map Server
│   ├── nav2_servers/              — Lifecycle / Velocity / Collision …
│   │  ── gz-sim / Gazebo ──
│   ├── gz-build/                  — cmake / ninja / colcon / bazel recipes
│   ├── gz-test/                   — ctest filters, gtest, headless
│   ├── gz-ecs-overview/           — Entity-Component-System architecture
│   ├── new-component/             — Component header templates
│   └── new-system/                — Full system plugin scaffold
├── rules/               # rules to follow in the project
│   ├── clean_architecture.md      — Layer boundaries
│   ├── ros2_general.md            — General ROS 2 principles
│   ├── ros2_nodes.md              — Node development rules
│   ├── ros2_communication.md      — Topic / QoS / interface rules
│   ├── testing.md                 — Test strategy
│   ├── nav2_architecture.md       — Nav 2 system architecture
│   ├── nav2_parameters.md         — Nav 2 parameter reference
│   ├── nav2_msgs_reference.md     — Nav 2 msg/srv/action list
│   ├── ros2_control_architecture.md — ros2_control framework & lifecycle
│   ├── ros2_controllers_reference.md — ros2_controllers package catalog
│   ├── ros2_control_demos.md      — ros2_control_demos example catalog (17)
│   ├── behaviortree_cpp.md        — BehaviorTree.CPP v4 core reference
│   ├── behaviortree_ros2.md       — BehaviorTree.ROS2 wrappers + execution server
│   ├── webots_ros2_architecture.md — Webots ROS 2 driver, device plugins, ros2_control bridge
│   ├── vda5050_protocol.md        — VDA 5050 v3.0.0 fleet interface overview
│   ├── vda5050_messages.md        — VDA 5050 v3.0.0 complete message spec + processes
│   ├── vda5050_implementation_formats.md — VDA 5050 code-gen format analysis (3 reference idioms)
│   └── robot_specific.md          — Robot-specific settings (override these)
└── agents/              # custom sub-agent definitions
    │  ── ROS 2 / Nav 2 ──
    ├── ros2-style-reviewer.md     — ROS 2 / Clean-arch PR review expert
    ├── clean-arch-architect.md    — Layer / dependency design advisor
    ├── ros2-controllers-reviewer.md — ros2_control controller PR reviewer
    ├── behaviortree-reviewer.md   — BehaviorTree.CPP / ROS2 PR reviewer
    ├── webots-ros2-reviewer.md    — webots_ros2 plugin / launch PR reviewer
    │  ── gz-sim / Gazebo ──
    ├── gz-style-reviewer.md       — gz-sim style / PR review expert
    ├── ecs-architect.md           — ECS design advisor
    │  ── VDA 5050 fleet interface ──
    └── vda5050-reviewer.md        — VDA 5050 protocol-compliance PR reviewer
```

## How to use

### Slash commands
Invoked directly from the terminal:

```
# Start from zero
/new-workspace ~/my_robot_ws acme   # bootstrap a complete colcon workspace

# ROS 2 / Nav 2
/build                    # whole workspace (colcon)
/build my_pkg             # a single package only
/test                     # colcon test + result
/lint --all               # all files
/new-package my_pkg python
/new-node my_pkg my_node lifecycle cpp
/new-nav2-plugin controller MyController
/new-controller my_pkg MyController chainable
/new-hardware my_pkg MyRobotHardware system
/new-bt-node my_pkg MyAction ros-action
/new-vda5050-connector my_connector python robot
/new-webots-plugin my_pkg MyLidar cpp Lidar
/changelog

# Extend this template itself (self-extensible)
/new-skill   my_topic_helper
/new-command new-my-thing
/new-agent   my-domain-reviewer

# gz-sim / Gazebo
/gz-build                 # cmake -B build && ninja
/gz-test                  # ctest --test-dir build
/gz-lint                  # pre-commit (gz-sim)
/gz-new-component Foo
/gz-new-system fancy_drive FancyDrive
/gz-changelog
```

### Skills
When Claude takes on a task it automatically consults the relevant
`SKILL.md`. To run a skill explicitly, you can give the file path
directly:

```
apply all the steps in .claude/skills/ros2_lifecycle/SKILL.md
```

### Sub-agents
Invoked through the `Agent` tool:

ROS 2 / Nav 2 side:
* `subagent_type: "ros2-style-reviewer"` — strictly audits the diff
  for clean-arch and ROS 2 rule compliance, gives line-anchored
  feedback.
* `subagent_type: "clean-arch-architect"` — produces optioned
  recommendations on which layer a new feature should live in and how.

gz-sim / Gazebo side:
* `subagent_type: "gz-style-reviewer"` — audits gz-sim style, plugin
  registration, and CMake/Bazel consistency.
* `subagent_type: "ecs-architect"` — produces optioned recommendations
  on which Component new state/behaviour belongs to or which System
  phase it belongs in.

### Personal overrides

For personal permission/env settings you don't want to commit to the
repo, create a `.claude/settings.local.json` file (it has been added to
.gitignore automatically).

## When adding new content

* A new skill: `.claude/skills/<kebab_or_snake>/SKILL.md` —
  `name` and `description` in the frontmatter are **required**.
* A new slash command: `.claude/commands/<name>.md` —
  `description` in the frontmatter, optional `argument-hint` and
  `allowed-tools`. Parameters are taken via `$ARGUMENTS` in the body.
* A new sub-agent: `.claude/agents/<name>.md` — `name`,
  `description`, optional `tools` and `model`.
* Update the index in this README with every addition.

## Philosophy

1. **Invert the dependency.** `domain/` does not know about ROS 2; the
   ROS 2 context stays in the `infrastructure/` layer.
2. **Recipe first, code second.** Commands are small helpers; thanks to
   skills and rules the code Claude produces is consistent.
3. **Explicit commands > magical automatic behaviour.** Automatic
   behaviours are expressed via `settings.json` hooks; we don't produce
   hidden side effects.
4. **Single source.** Every piece of information lives in one place;
   instead of duplication, link with the `[[rule_file]]` format.
