---
name: ros2-controller-creation
description: Scaffold or extend a ros2_control controller or broadcaster (ControllerInterface / ChainableControllerInterface) — base-class choice, command/state interface configuration, lifecycle, real-time-safe update(), generate_parameter_library, pluginlib export, tests. Trigger when the user asks to write a ros2_control controller or broadcaster.
---

# Writing a ros2_control Controller / Broadcaster

How to create or extend a controller in the **ros2_control** framework,
following the conventions used across `~/nav2_ws/src/ros2_controllers/`.

- Framework + lifecycle + RT rules: `rules/ros2_control_architecture.md`.
- Existing controller catalog (copy the closest one): `rules/ros2_controllers_reference.md`.
- Reference implementations to mirror:
  - simplest controller → `forward_command_controller/`
  - feedback + odometry + chaining → `diff_drive_controller/`
  - read-only sensor → `imu_sensor_broadcaster/`
  - PID / pure chaining → `pid_controller/`, `chained_filter_controller/`

## First decision: which base class?

| You are building… | Base class | Override |
|-------------------|-----------|----------|
| A controller that just writes commands | `ControllerInterface` | `update()` |
| A controller others can chain into / that exposes a reference | `ChainableControllerInterface` | `update_reference_from_subscribers()` + `update_and_write_commands()` |
| A read-only sensor/state publisher | `ControllerInterface` (or `Chainable` to re-export state) with **NONE** command interfaces | `update()` (read state → publish) |

A controller is a **pluginlib plugin**, not a node. It runs inside
`controller_manager`. It never opens hardware directly — only through the
command/state interfaces the manager loans it.

## Minimal package skeleton

```
my_controller/
├── include/my_controller/my_controller.hpp
├── src/my_controller.cpp
├── src/my_controller_parameters.yaml
├── my_controller_plugin.xml
├── CMakeLists.txt
├── package.xml
├── doc/userdoc.rst
└── test/test_my_controller.cpp
```

### 1. Header — the class

```cpp
#include "controller_interface/controller_interface.hpp"   // or chainable_controller_interface.hpp
#include "my_controller/my_controller_parameters.hpp"      // generated
#include "realtime_tools/realtime_publisher.hpp"
#include "realtime_tools/realtime_thread_safe_box.hpp"

namespace my_controller
{
class MyController : public controller_interface::ControllerInterface
{
public:
  controller_interface::CallbackReturn on_init() override;
  controller_interface::InterfaceConfiguration command_interface_configuration() const override;
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;
  controller_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State &) override;
  controller_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State &) override;
  controller_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State &) override;
  controller_interface::return_type update(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

protected:
  std::shared_ptr<ParamListener> param_listener_;
  Params params_;
};
}  // namespace my_controller
```

### 2. Interface configuration

```cpp
controller_interface::InterfaceConfiguration
MyController::command_interface_configuration() const
{
  return { controller_interface::interface_configuration_type::INDIVIDUAL,
           { params_.joint + "/velocity" } };          // names you will write
}
controller_interface::InterfaceConfiguration
MyController::state_interface_configuration() const
{
  return { controller_interface::interface_configuration_type::INDIVIDUAL,
           { params_.joint + "/position", params_.joint + "/velocity" } };
}
// A broadcaster returns type NONE for command_interface_configuration().
```

### 3. Lifecycle + the hot path

```cpp
controller_interface::CallbackReturn MyController::on_init()
{
  param_listener_ = std::make_shared<ParamListener>(get_node());
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn MyController::on_configure(const rclcpp_lifecycle::State &)
{
  params_ = param_listener_->get_params();
  // create subscribers / RealtimePublisher / preallocate messages HERE
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn MyController::on_activate(const rclcpp_lifecycle::State &)
{
  // cache references into command_interfaces_ / state_interfaces_ by index here
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::return_type MyController::update(
  const rclcpp::Time &, const rclcpp::Duration &)
{
  // RT-safe ONLY: no new/malloc, no locks, no throw, no unthrottled logging
  const double fb = state_interfaces_[0].get_optional().value_or(0.0);
  command_interfaces_[0].set_value(compute(fb));
  return controller_interface::return_type::OK;
}
```

### 4. Parameters (`generate_parameter_library`)

`src/my_controller_parameters.yaml`:

```yaml
my_controller:
  joint: {
    type: string,
    default_value: "",
    description: "Joint whose interfaces this controller claims",
    read_only: true,
    validation: { not_empty<>: [] }
  }
  gain: {
    type: double, default_value: 1.0,
    validation: { gt<>: [0.0] }
  }
```

Never `declare_parameter` by hand in a controller — use the generated
`ParamListener`/`Params`. Re-call `get_params()` in `update()` only if you
need live updates (and guard with `param_listener_->is_old(params_)`).

### 5. Plugin export + CMake + macro

`my_controller_plugin.xml`:

```xml
<library path="my_controller">
  <class name="my_controller/MyController" type="my_controller::MyController"
         base_class_type="controller_interface::ControllerInterface">
    <description>What it does.</description>
  </class>
</library>
```

`CMakeLists.txt` essentials:

```cmake
find_package(generate_parameter_library REQUIRED)
find_package(controller_interface REQUIRED)
find_package(pluginlib REQUIRED)

generate_parameter_library(my_controller_parameters src/my_controller_parameters.yaml)

add_library(my_controller SHARED src/my_controller.cpp)
target_link_libraries(my_controller PUBLIC
  controller_interface::controller_interface
  my_controller_parameters
  pluginlib::pluginlib)

pluginlib_export_plugin_description_file(controller_interface my_controller_plugin.xml)
```

Bottom of `src/my_controller.cpp`:

```cpp
#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(my_controller::MyController, controller_interface::ControllerInterface)
```

`package.xml` depends: `controller_interface`, `pluginlib`,
`generate_parameter_library`, `realtime_tools`, `rclcpp_lifecycle`, plus
your message packages.

## Testing

Mirror an existing `test/`. Controllers are tested by constructing them
directly, assigning **mock** loaned interfaces, driving the lifecycle
(`configure → activate`), calling `update()`, and asserting on the
command-interface values / published messages — no real hardware. Use
`controller_manager`'s test fixtures (`ControllerInterfaceBaseTest` style)
where helpful. Don't `sleep()` to synchronize.

## Common pitfalls

- **Work in `update()` that isn't RT-safe** — allocation, locks, `throw`,
  `RCLCPP_INFO` every cycle, `publisher_->publish()` instead of
  `RealtimePublisher`. This causes deadline misses.
- **Claiming interfaces in `on_configure`** — interfaces are valid only
  from `on_activate`; cache handle references there.
- **Hand-rolled `declare_parameter`** instead of `generate_parameter_library`.
- **Renaming the plugin alias** (`<pkg>/<Class>`) after release — it is
  user-facing API in `controller_manager` YAML.
- **A broadcaster claiming command interfaces** — broadcasters return
  `NONE` for command config and must not actuate.
- **Treating the controller as a node** — no standalone `rclcpp::spin`; it
  is ticked by the controller_manager.
