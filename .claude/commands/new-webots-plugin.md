---
description: Scaffold a webots_ros2 device/robot plugin (C++ PluginInterface or Python init/step) + URDF <webots> declaration, optional ros2_control wiring, and a launch file.
argument-hint: "<package> <ClassName> <cpp|python> [device_type]"
allowed-tools: ["Bash", "Read", "Write", "Edit"]
---

Scaffold a webots_ros2 plugin and its bringup inside an existing package.

Argument handling (`$ARGUMENTS`):
1. `<package>` — existing package under `src/` (required).
2. `<ClassName>` — plugin class name (required).
3. `<lang>` — `cpp` (`PluginInterface`, pluginlib) or `python`
   (`init`/`step` class, no build step).
4. `[device_type]` — the Webots device class it wraps (e.g. `Lidar`,
   `DistanceSensor`, `Camera`). If the sensor is standard, first suggest
   the **no-code `<device>`** path instead.

If anything is missing, ask before scaffolding.

Process:
1. Read the skill `webots_ros2_device_plugin` and the rule
   `webots_ros2_architecture.md`. If a built-in plugin already covers the
   device, recommend just declaring a `<device>` in the URDF and stop.
2. For a **custom plugin**, create:
   * cpp → `include/<package>/<Class>.hpp` + `src/<Class>.cpp` deriving
     from `webots_ros2_driver::Ros2SensorPlugin` / `PluginInterface`
     (`init()` gets the device handle + creates pubs/subs; `step()` reads
     the device and publishes — never call `robot.step()`); a
     `<package>_plugin.xml` (`base_class_type=webots_ros2_driver::PluginInterface`)
     and `pluginlib_export_plugin_description_file(webots_ros2_driver …)`.
   * python → `<package>/<package>/<snake>.py` with a class exposing
     `init(self, webots_node, properties)` + `step(self)`; no pluginlib.
3. Add the URDF wiring: a `<device reference="…" type="…"><ros>…</ros>`
   entry and/or a `<plugin type="…"/>` in the `<webots>` block. For motors,
   add the `webots_ros2_control::Ros2Control` plugin + a `<ros2_control>`
   block and a `ros2_control.yml`.
4. Provide/extend a launch file mirroring `webots_ros2_epuck`
   (`WebotsLauncher` + `robot_state_publisher` + `WebotsController` +
   controller spawners).
5. Wire `CMakeLists.txt`/`package.xml` (cpp: dep `webots_ros2_driver`,
   `pluginlib`, `sensor_msgs`; python: `setup.py` data files).
6. `pre-commit run --files <touched files>`.
7. Print: files created, the URDF snippet, and
   `ros2 launch <package> robot_launch.py`.

Never call `robot.step()` in a plugin; `reference`/`type` must match the
Webots device; prefer the no-code `<device>` path when a built-in covers it.
