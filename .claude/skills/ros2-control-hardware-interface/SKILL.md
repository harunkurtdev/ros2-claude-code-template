---
name: ros2-control-hardware-interface
description: Scaffold a ros2_control hardware component (SystemInterface / ActuatorInterface / SensorInterface) and its bringup — the plugin (on_init/on_configure/on_activate, RT-safe read()/write()), URDF <ros2_control> wiring, controllers.yaml + launch, pluginlib export. Trigger when the user asks to integrate hardware or bring up a robot under ros2_control.
---

# Writing a ros2_control Hardware Component + Bringup

How to bring up a real (or simulated) robot under ros2_control: write a
`hardware_interface` plugin, wire it in the URDF, and launch it. Mirrors
the patterns in `~/nav2_ws/src/ros2_control_demos/`.

- Framework + interfaces + lifecycle: `rules/ros2_control_architecture.md`.
- Example catalog (copy the closest one): `rules/ros2_control_demos.md`.
- Controller side (the plugin that *commands* this hardware):
  `skills/ros2-controller-creation/SKILL.md`.
- Reference implementations to mirror:
  - whole robot → `example_1` (RRBot `SystemInterface`)
  - mobile base → `example_2` (DiffBot)
  - standalone sensor → `example_5` (`SensorInterface`)
  - per-actuator modular → `example_6` (`ActuatorInterface`)
  - GPIO → `example_10`, diagnostics → `example_17`

## First decision: which hardware base class?

| You are integrating… | Base class | Exports |
|----------------------|-----------|---------|
| A whole robot (joints + maybe sensors) | `hardware_interface::SystemInterface` | command + state interfaces |
| A single actuator | `hardware_interface::ActuatorInterface` | command + state for one actuator |
| A read-only sensor | `hardware_interface::SensorInterface` | **state interfaces only** |

A hardware component is a **pluginlib plugin** loaded by the
`controller_manager` (via the URDF), not a node. It runs in the RT loop:
`read() → controllers' update() → write()` every cycle.

## 1. The hardware plugin

`hardware/include/<pkg>/<robot>.hpp`:

```cpp
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "rclcpp_lifecycle/state.hpp"

namespace my_robot
{
class MyRobotHardware : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(MyRobotHardware)

  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareComponentInterfaceParams & params) override;
  hardware_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State &) override;
  hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State &) override;
  hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State &) override;

  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;
  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  double cfg_param_;   // read from URDF <param> in on_init
};
}  // namespace my_robot
```

`hardware/<robot>.cpp` essentials:

```cpp
CallbackReturn MyRobotHardware::on_init(const HardwareComponentInterfaceParams & params)
{
  if (SystemInterface::on_init(params) != CallbackReturn::SUCCESS)
    return CallbackReturn::ERROR;
  // joints + their interfaces come from the URDF (get_hardware_info());
  // read custom params: info_.hardware_parameters["my_param"]
  return CallbackReturn::SUCCESS;
}

CallbackReturn MyRobotHardware::on_activate(const rclcpp_lifecycle::State &)
{
  for (const auto & [name, descr] : joint_state_interfaces_)  set_state(name, 0.0);
  for (const auto & [name, descr] : joint_command_interfaces_) set_command(name, get_state(name));
  return CallbackReturn::SUCCESS;   // open device handles HERE
}

hardware_interface::return_type MyRobotHardware::read(const rclcpp::Time &, const rclcpp::Duration &)
{
  // RT-safe: poll the device, push values into state interfaces
  set_state("joint1/position", read_encoder());
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type MyRobotHardware::write(const rclcpp::Time &, const rclcpp::Duration &)
{
  // RT-safe: take command-interface values, send them to the device
  send_to_motor(get_command("joint1/position"));
  return hardware_interface::return_type::OK;
}
```

Modern hardware_interface auto-creates the interface handles from the
URDF, so you use `get_state/set_state/get_command/set_command(name)`
helpers rather than hand-managing `std::vector<double>`. (Older demos
override `export_state_interfaces()` / `export_command_interfaces()` —
both patterns exist; prefer the URDF-driven one.)

`read()` / `write()` are the **RT hot path** — same rules as a
controller's `update()`: no allocation, no locks, no throw, no blocking
I/O, no unthrottled logging. Do device setup in `on_configure`/`on_activate`.

## 2. Plugin export

`<pkg>.xml`:

```xml
<library path="my_robot">
  <class name="my_robot/MyRobotHardware" type="my_robot::MyRobotHardware"
         base_class_type="hardware_interface::SystemInterface">
    <description>My robot hardware interface.</description>
  </class>
</library>
```

```cpp
#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(my_robot::MyRobotHardware, hardware_interface::SystemInterface)
```

```cmake
pluginlib_export_plugin_description_file(hardware_interface my_robot.xml)
```

`package.xml` depends: `hardware_interface`, `pluginlib`,
`rclcpp_lifecycle` (+ device libs).

## 3. URDF `<ros2_control>` tag (wires joints → your plugin)

`description/ros2_control/<robot>.ros2_control.xacro`:

```xml
<ros2_control name="${name}" type="system">
  <hardware>
    <plugin>my_robot/MyRobotHardware</plugin>
    <param name="my_param">0.5</param>          <!-- read in on_init -->
  </hardware>
  <joint name="${prefix}joint1">
    <command_interface name="position">
      <param name="min">-1</param>
      <param name="max">1</param>
    </command_interface>
    <state_interface name="position"/>
  </joint>
</ros2_control>
```

The `<plugin>` string must equal the pluginlib `name` alias above. The
`<command_interface>` / `<state_interface>` names here are exactly what
controllers claim (`joint1/position`).

## 4. Bringup

`bringup/config/<robot>_controllers.yaml`:

```yaml
controller_manager:
  ros__parameters:
    update_rate: 100  # Hz — the read/update/write loop frequency
    joint_state_broadcaster:
      type: joint_state_broadcaster/JointStateBroadcaster
    forward_position_controller:
      type: forward_command_controller/ForwardCommandController

forward_position_controller:
  ros__parameters:
    joints: [joint1, joint2]
    interface_name: position
```

`bringup/launch/<robot>.launch.py` (the standard shape): start
`robot_state_publisher` with the xacro-expanded URDF, start
`controller_manager` (`ros2_control_node`) with the controllers.yaml +
`robot_description`, then `spawner` each controller
(`joint_state_broadcaster` first, then your command controller). Copy
`example_1/bringup/launch/rrbot.launch.py` and adapt names.

## 5. Verify

```bash
ros2 launch my_robot_bringup my_robot.launch.py
ros2 control list_hardware_interfaces     # your command/state ifaces appear, claimed/available
ros2 control list_controllers             # active/inactive
```

## Common pitfalls

- **Non-RT-safe `read()`/`write()`** — allocation, blocking device I/O
  without a timeout, locks, unthrottled logging. Open sockets/serial in
  `on_activate`, not in the loop.
- **Plugin string mismatch** — the URDF `<plugin>` must match the `.xml`
  `name` alias and `PLUGINLIB_EXPORT_CLASS`; `base_class_type` must be the
  right `hardware_interface::*Interface`.
- **A `SensorInterface` exporting command interfaces** — sensors are
  read-only (state interfaces only).
- **Spawning the command controller before `joint_state_broadcaster`** —
  broadcaster first.
- **`update_rate` mismatch** between controller_manager and a controller's
  own rate, or a write() that ignores `period`.
- **Treating the component as a node** — no `rclcpp::spin`; the
  controller_manager ticks it.
