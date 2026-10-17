---
description: Scaffold a Node inside an existing package — standard, lifecycle, or component, in Python or C++.
argument-hint: "<package> <node_name> <standard|lifecycle|component> <python|cpp>"
allowed-tools: ["Bash", "Read", "Write", "Edit"]
---

Add a new ROS 2 node to an existing package.

Argument handling (`$ARGUMENTS`):
1. `<package>` — existing package under `src/` (required).
2. `<node_name>` — `snake_case` for Python files / `CamelCase` for C++ class.
3. `<kind>` — one of `standard`, `lifecycle`, `component`.
4. `<lang>` — `python` or `cpp`.

If anything is missing, ask the user before scaffolding.

Process:
1. Verify the package exists and matches the requested language
   (look at the build type in `package.xml`).
2. Read the relevant skill:
   * `ros2-node-creation` for `standard`.
   * `ros2-lifecycle` for `lifecycle`.
3. Place files following Clean Architecture:
   * **Infrastructure** layer hosts the ROS 2 adapter (the actual Node).
   * **Application** layer hosts the use-case the node orchestrates.
   * **Domain** stays ROS-free.
4. Wire entry points:
   * Python: `setup.py` `entry_points={'console_scripts': [...]}` and
     `setup.cfg` if needed.
   * C++: add the executable / component target to `CMakeLists.txt`
     with `rclcpp_components_register_nodes(...)` for component nodes.
5. Add a parameters YAML stub at `config/<node_name>.yaml` and a minimal
   launch fragment at `launch/<node_name>.launch.py` that loads it.
6. Add a smoke test (`test/test_<node_name>.py` or
   `test/<NodeName>_test.cpp`) that just constructs the node and shuts
   it down — guards against import errors.
7. `pre-commit run --files <touched files>`.
8. Print: files created, the launch command to try it
   (`ros2 launch <package> <node_name>.launch.py`).

Never just dump a Node subclass into `__main__` — it must follow the
Clean Architecture layout enforced by `.claude/rules/clean_architecture.md`.
