---
description: Bootstrap a complete ROS 2 colcon workspace from scratch — dir layout, .gitignore, README, this .claude/ config, and seed interfaces/core/bringup packages.
argument-hint: "<workspace_path> [project_prefix]"
allowed-tools: ["Bash", "Read", "Write", "Edit"]
---

Create a brand-new, ready-to-build ROS 2 workspace.

Argument handling (`$ARGUMENTS`):
1. `<workspace_path>` — where to create it (e.g. `~/my_robot_ws`)
   (required).
2. `[project_prefix]` — package name prefix (e.g. `acme`). If missing,
   derive from the workspace folder name or ask.

If anything is unclear (path already exists, prefix), ask before acting.

Process:
1. Read the `new-ros2-workspace` skill end to end.
2. Verify `<workspace_path>` does not already contain a `src/`.
3. Create the layout: `<ws>/src`, `.gitignore` (build/ install/ log/),
   top-level `README.md` with the build/source/test commands; `git init`
   (ask first).
4. Copy this template's `.claude/` into the new workspace root so the
   slash commands work there (skip if creating inside the template repo).
5. Seed packages in dependency order, delegating to the
   `new-ros2-package` skill / `/new-package`:
   * `<prefix>_interfaces` (ament_cmake) — only if custom msgs are needed.
   * `<prefix>_core` (Clean Architecture, language per user).
   * `<prefix>_bringup` (ament_python) — top-level launch/config.
6. Print:
   * the tree that was created,
   * `cd <ws> && colcon build --symlink-install && source install/setup.bash`,
   * the follow-up commands for the project's domain
     (`/new-vda5050-connector`, `/new-controller`, `/new-bt-node`, …).

Do **not** run `colcon build` automatically — let the user review first.
Confirm workspace name/path and project prefix before scaffolding.
