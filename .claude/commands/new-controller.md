---
description: Scaffold a ros2_control controller or broadcaster — base class, command/state interface config, RT-safe update(), generate_parameter_library, pluginlib export, and a test.
argument-hint: "<package> <ClassName> <plain|chainable|broadcaster> [py|cpp]"
allowed-tools: ["Bash", "Read", "Write", "Edit"]
---

Scaffold a ros2_control controller/broadcaster inside an existing package.

Argument handling (`$ARGUMENTS`):
1. `<package>` — existing package under `src/` (required).
2. `<ClassName>` — CamelCase controller class (required).
3. `<kind>` — `plain` (`controller_interface::ControllerInterface`),
   `chainable` (`ChainableControllerInterface`), or `broadcaster`
   (read-only, `NONE` command interfaces).
4. `[lang]` — `cpp` (default; ros2_control controllers are C++).

If anything is missing, ask before scaffolding.

Process:
1. Read the skill `ros2-controller-creation` and the rule
   `ros2_control_architecture.md`; pick the closest existing controller
   in `~/nav2_ws/src/ros2_controllers/` to mirror
   (`forward_command_controller` for plain, `diff_drive_controller` for
   chainable, `imu_sensor_broadcaster` for broadcaster).
2. Create:
   * `include/<package>/<snake_class>.hpp` — the class deriving from the
     chosen base, overriding `on_init`, `command_interface_configuration`,
     `state_interface_configuration`, the lifecycle callbacks, and the
     right update method (`update()` for plain; `update_reference_from_subscribers`
     + `update_and_write_commands` for chainable). Broadcasters return
     `NONE` for command config.
   * `src/<snake_class>.cpp` — implementation + `PLUGINLIB_EXPORT_CLASS`
     with the correct base class.
   * `src/<snake_class>_parameters.yaml` — `generate_parameter_library`
     input with sensible `validation` / `read_only`.
   * `<package>_plugin.xml` (or extend an existing one) with the right
     `base_class_type`.
   * `test/test_<snake_class>.cpp` — gtest that drives
     `on_configure → on_activate`, calls the update method with mock
     loaned interfaces, and asserts command values.
3. Wire `CMakeLists.txt`: `generate_parameter_library(...)`,
   `add_library`, link `controller_interface::controller_interface` +
   the params lib + `pluginlib::pluginlib`, and
   `pluginlib_export_plugin_description_file(controller_interface <xml>)`.
4. Add deps to `package.xml` (`controller_interface`, `pluginlib`,
   `generate_parameter_library`, `realtime_tools`, message pkgs).
5. Remind: nothing RT-unsafe in the update method (no alloc/locks/throw/
   unthrottled logging; use `RealtimePublisher`/`RealtimeThreadSafeBox`).
6. `pre-commit run --files <touched files>`.
7. Print: files created, the plugin alias, and a `controllers.yaml`
   snippet plus `ros2 run controller_manager spawner <name>`.

Never put parameter handling outside `generate_parameter_library`, and
never rename the plugin alias of an existing controller.
