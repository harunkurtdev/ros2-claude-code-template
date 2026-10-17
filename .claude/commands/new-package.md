---
description: Scaffold a new ROS 2 package (ament_python or ament_cmake) following Clean Architecture layout — domain / application / infrastructure / presentation.
argument-hint: "<package_name> <python|cpp> [description]"
allowed-tools: ["Bash", "Read", "Write", "Edit"]
---

Scaffold a new package under `src/` of the current ROS 2 workspace.

Argument handling (`$ARGUMENTS`):
* First token → `package_name` (required, must match `[a-z][a-z0-9_]*`).
* Second token → `python` or `cpp` (required).
* Remaining tokens → free-form description for `package.xml`.

If `$ARGUMENTS` is empty or malformed, ask the user before doing anything.

Process:
1. Read the `new-ros2-package` skill end to end.
2. Verify `src/<package_name>/` does not already exist.
3. Run `ros2 pkg create` with the right build type:
   * Python: `ros2 pkg create --build-type ament_python <name>
     --license Apache-2.0 --maintainer-name "$(git config user.name)"
     --maintainer-email "$(git config user.email)"`
   * C++:    `ros2 pkg create --build-type ament_cmake  <name>
     --license Apache-2.0 --maintainer-name "$(git config user.name)"
     --maintainer-email "$(git config user.email)"`
4. **Restructure into Clean Architecture layout** following the
   `new-ros2-package` skill's templates:
   * Python:
     ```
     src/<name>/<name>/
       domain/        (entities, value objects, ports)
       application/   (use cases, application services)
       infrastructure/ (ROS 2 nodes, repositories)
       presentation/  (CLI, launch entrypoints)
     ```
   * C++:
     ```
     src/<name>/
       include/<name>/{domain,application,infrastructure,presentation}/
       src/{domain,application,infrastructure,presentation}/
     ```
5. Add `launch/`, `config/`, `test/` directories with a placeholder
   `.gitkeep` and a minimal smoke test.
6. Update `package.xml` with the description from `$ARGUMENTS` and add
   `<test_depend>ament_lint_auto</test_depend>` + the default linters.
7. Update `setup.py` / `CMakeLists.txt` to install the launch/config
   directories.
8. Run `pre-commit run --files <every file you touched>` (or
   `ament_*` if pre-commit is not yet configured).
9. Print a summary:
   * Path to the new package
   * Build command (`/build <name>` or `colcon build --packages-select <name>`)
   * Files created

Do **not** run `colcon build` automatically — the user may want to
review first.
