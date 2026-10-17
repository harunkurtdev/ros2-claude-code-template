---
description: Scaffold a ros2_control hardware component (System/Actuator/Sensor) + URDF <ros2_control> tag + bringup (controllers.yaml + launch), with pluginlib export and a test.
argument-hint: "<package> <ClassName> <system|actuator|sensor>"
allowed-tools: ["Bash", "Read", "Write", "Edit"]
---

Scaffold a ros2_control hardware component and its bringup.

Argument handling (`$ARGUMENTS`):
1. `<package>` — existing package under `src/` (required).
2. `<ClassName>` — CamelCase hardware class (required).
3. `<kind>` — `system` (whole robot, `SystemInterface`),
   `actuator` (`ActuatorInterface`), or `sensor` (read-only,
   `SensorInterface`).

If anything is missing, ask before scaffolding.

Process:
1. Read the skill `ros2-control-hardware-interface` and the rules
   `ros2_control_architecture.md` + `ros2_control_demos.md`; pick the
   closest example in `~/nav2_ws/src/ros2_control_demos/` to mirror
   (`example_1` system, `example_6` actuator, `example_5` sensor).
2. Create the plugin:
   * `hardware/include/<package>/<snake_class>.hpp` — class deriving from
     `hardware_interface::{System,Actuator,Sensor}Interface`, overriding
     `on_init(HardwareComponentInterfaceParams)`, `on_configure`,
     `on_activate`, `on_deactivate`, and `read()` / `write()`
     (sensors: `read()` only, no command interfaces).
   * `hardware/<snake_class>.cpp` — implementation + `PLUGINLIB_EXPORT_CLASS`
     with the right `hardware_interface::*Interface` base.
   * `<package>.xml` — pluginlib description, `base_class_type` matching.
3. Create the bringup + description:
   * `description/ros2_control/<robot>.ros2_control.xacro` — the
     `<ros2_control>` tag with `<hardware><plugin>` (== the alias) and the
     joints' `<command_interface>` / `<state_interface>`.
   * `bringup/config/<robot>_controllers.yaml` — `controller_manager`
     with `update_rate` + `joint_state_broadcaster` + one command
     controller.
   * `bringup/launch/<robot>.launch.py` — robot_state_publisher +
     `ros2_control_node` + spawners (broadcaster first).
   * `test/test_<snake_class>.cpp` — load the URDF and check the
     interfaces export.
4. Wire `CMakeLists.txt` / `package.xml`: deps `hardware_interface`,
   `pluginlib`, `rclcpp_lifecycle`;
   `pluginlib_export_plugin_description_file(hardware_interface <xml>)`;
   install `description/` and `bringup/`.
5. Remind: `read()`/`write()` are the RT hot path — open device handles in
   `on_activate`, not in the loop; no alloc/locks/blocking I/O there.
6. `pre-commit run --files <touched files>`.
7. Print: files created and
   `ros2 launch <package> <robot>.launch.py` +
   `ros2 control list_hardware_interfaces`.

The URDF `<plugin>` string must equal the pluginlib alias and
`PLUGINLIB_EXPORT_CLASS`; a `sensor` component must not export command
interfaces.
