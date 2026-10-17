# ros2_control_demos — Example Catalog & Learning Map

The official, runnable example suite for ros2_control. Source:
`~/nav2_ws/src/ros2_control_demos/` (branch `master`). These demos teach
the **hardware-component** side of ros2_control (writing
`hardware_interface` plugins) and **bringup** (URDF `<ros2_control>` tags,
`controllers.yaml`, launch files) — the complement to the controller side
in `ros2_controllers_reference.md`.

- Upstream: <https://github.com/ros-controls/ros2_control_demos>
- Rendered docs: <https://control.ros.org> → "Demos".
- Each example: `example_N/doc/userdoc.rst` (authoritative walk-through).
- Shared robot meshes/URDF: `ros2_control_demo_description/`.

## What the demos add over `ros2_controllers`

`ros2_controllers` = the controller plugins. `ros2_control_demos` = how to
**stand up a whole robot**: a hardware component that exports interfaces, a
URDF that wires joints to that component, a `controllers.yaml`, and a
launch file that starts `robot_state_publisher` + `controller_manager` +
spawners. Use them as copy-paste templates for new hardware.

## Example index (what each one teaches)

| # | Robot | Teaches |
|---|-------|---------|
| 1 | RRBot | The baseline: a `SystemInterface` with position command + position state; forward & JTC bringup. **Start here.** |
| 2 | DiffBot | A mobile base `SystemInterface` (velocity) driven by `diff_drive_controller`. |
| 3 | RRBot multi-interface | One joint exporting **multiple** command/state interfaces (position+velocity+acceleration); switching command modes. |
| 4 | Industrial robot, integrated sensor | A `SystemInterface` that also exports **sensor** state interfaces (FTS inside the system). |
| 5 | Industrial robot, external sensor | A standalone **`SensorInterface`** component (force-torque) separate from the actuator system. |
| 6 | Modular robot | Per-actuator **`ActuatorInterface`** components — separate comms to each actuator. |
| 7 | 6-DOF robot | Full tutorial bringing a realistic 6-DOF arm up with JTC. |
| 8 | Transmissions | Using `transmission_interface` between joint and actuator space. |
| 9 | Gazebo (Classic) | Simulation hardware via the Gazebo `ros2_control` plugin instead of a custom component. |
| 10 | GPIO interfaces | A component exporting `gpio/*` command/state interfaces, driven by `gpio_controllers`. |
| 11 | CarlikeBot | Ackermann steering: 2 steering + 2 drive, with a steering controller. |
| 12 | **Controller chaining** | Wiring a chainable controller's reference interfaces to another controller (the chaining demo). |
| 13 | Multi-robot + lifecycle | Multiple hardware components with per-component **lifecycle management** (activate/deactivate independently). |
| 14 | Modular, no state feedback | Actuators that don't report state + a separate sensor providing the position feedback. |
| 15 | Multiple controller managers | Running more than one `controller_manager` (e.g. per subsystem). |
| 16 | DiffBot + chained controllers | DiffBot bring-up using chained controllers. |
| 17 | RRBot + diagnostics | A hardware component that publishes `diagnostic_msgs` via the diagnostics interface. |

## Hardware component types (the three base classes)

| Base class | Exports | Use for |
|------------|---------|---------|
| `hardware_interface::SystemInterface` | command **and** state interfaces for a whole robot | most robots (examples 1–4, 7, 17) |
| `hardware_interface::ActuatorInterface` | one actuator's command + state | per-joint modular hardware (example 6, 14) |
| `hardware_interface::SensorInterface` | **state interfaces only** | standalone sensors (example 5) |

See `skills/ros2-control-hardware-interface/SKILL.md` to write one.

## Anatomy of a demo package

```
example_N/
├── hardware/                      # the hardware_interface plugin (System/Actuator/Sensor)
│   ├── include/<pkg>/<robot>.hpp
│   └── <robot>.cpp                # + PLUGINLIB_EXPORT_CLASS(... hardware_interface::SystemInterface)
├── description/
│   ├── ros2_control/<robot>.ros2_control.xacro   # <ros2_control> tag: plugin + joints + interfaces
│   ├── urdf/  <robot>.urdf.xacro
│   └── rviz/, launch/view_robot.launch.py
├── bringup/
│   ├── config/<robot>_controllers.yaml           # controller_manager + controllers
│   └── launch/<robot>.launch.py                  # rsp + controller_manager + spawners
├── <pkg>.xml                      # pluginlib: base_class_type=hardware_interface::SystemInterface
├── CMakeLists.txt / package.xml
└── doc/userdoc.rst
```

## Running a demo

```bash
# visualize the URDF only
ros2 launch ros2_control_demo_example_1 view_robot.launch.py

# full bring-up (robot_state_publisher + controller_manager + spawners)
ros2 launch ros2_control_demo_example_1 rrbot.launch.py

# introspect
ros2 control list_hardware_interfaces
ros2 control list_controllers

# drive it
ros2 launch ros2_control_demo_example_1 test_forward_position_controller.launch.py
ros2 launch ros2_control_demo_example_1 test_joint_trajectory_controller.launch.py
```

For behaviour and per-step explanation always defer to the example's
`doc/userdoc.rst`.
