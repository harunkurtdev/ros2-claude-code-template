---
description: Scaffold a new Nav 2 plugin — controller, planner, behavior, smoother, goal-checker, progress-checker, costmap-layer, or BT node.
argument-hint: "<plugin_kind> <ClassName> [package_name]"
allowed-tools: ["Bash", "Read", "Write", "Edit"]
---

Scaffold a new Nav 2 plugin.

Argument handling (`$ARGUMENTS`):
1. `<plugin_kind>` — one of:
   `controller`, `planner`, `behavior`, `smoother`,
   `goal_checker`, `progress_checker`, `costmap_layer`, `bt_node`.
2. `<ClassName>` — CamelCase C++ class name.
3. `<package_name>` — optional; defaults to a new package
   `nav2_<snake_class>` under `src/`.

If `<plugin_kind>` or `<ClassName>` is missing, ask the user.

Process:
1. Read `.claude/skills/new-nav2-plugin/SKILL.md` end to end.
2. Read the matching upstream interface skill:
   * `controller` / `planner` / `smoother` / `goal_checker` /
     `progress_checker` / `behavior` → `nav2-core-interfaces`.
   * `costmap_layer` → `nav2-costmap`.
   * `bt_node` → `nav2-behavior-tree`.
3. Create (or reuse) the host package (`ament_cmake`). If creating,
   follow the `new-package` command.
4. Generate:
   * Header in `include/<pkg>/<snake_class>.hpp` inheriting the right
     `nav2_core::*` (or `nav2_costmap_2d::Layer`, or
     `BT::ActionNodeBase` …) base class.
   * Source in `src/<snake_class>.cpp` with all pure-virtuals
     implemented as TODO stubs that compile.
   * `plugins.xml` (or extend the existing one) declaring the new
     plugin class and `<library path="<pkg>">`.
   * Update `CMakeLists.txt` to:
     - Build the shared library.
     - `pluginlib_export_plugin_description_file(nav2_core plugins.xml)`
       (or the matching package for the plugin kind).
   * Update `package.xml` with `pluginlib` + the matching
     `<depend>nav2_core</depend>` etc.
5. Add a YAML stub at `config/<snake_class>.yaml` showing parameter
   declarations.
6. Add an integration test that loads the plugin via `pluginlib` and
   calls its main entrypoint.
7. `pre-commit run --files <touched files>`.
8. Print: files created and how to point Nav 2 at the plugin (the YAML
   key under `controller_server` / `planner_server` / etc.).

Refer to `.claude/rules/nav2_parameters.md` for canonical parameter
names and defaults.
