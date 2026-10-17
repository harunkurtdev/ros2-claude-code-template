---
description: Scaffold a modular launch file (Python or XML) inside an existing package, following the ros2-launch-config skill.
argument-hint: "<package> <launch_name> [python|xml]"
allowed-tools: ["Bash", "Read", "Write", "Edit"]
---

Add a new launch file to an existing package.

Argument handling (`$ARGUMENTS`):
1. `<package>` — required, must exist under `src/`.
2. `<launch_name>` — without `.launch.py` / `.launch.xml` suffix.
3. `<format>` — optional, defaults to `python`.

Process:
1. Read the `ros2-launch-config` skill end to end.
2. Place the file at `src/<package>/launch/<launch_name>.launch.<ext>`.
3. Use the skill's modular template:
   * `LaunchDescription` returned from `generate_launch_description()`.
   * Declared `LaunchArgument`s for every tunable parameter.
   * `IncludeLaunchDescription` for any sub-launch composition.
   * `ParameterFile` / `ParameterValue` rather than inline dicts.
   * `LifecycleNode` instead of `Node` if the underlying node is
     lifecycle-managed.
4. Add a stub parameter YAML at `config/<launch_name>.yaml` and load
   it via the launch arg.
5. Update `setup.py` / `CMakeLists.txt` so `launch/` and `config/` are
   installed.
6. Update `package.xml` `<exec_depend>` to include every package the
   launch references.
7. `pre-commit run --files <touched files>`.
8. Print: files created and the test command
   (`ros2 launch <package> <launch_name>.launch.<ext>`).
