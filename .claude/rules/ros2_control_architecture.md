# ros2_control / ros2_controllers Architecture Reference

`ros2_controllers` is the official controller suite for the
**ros2_control** framework. Source: `~/nav2_ws/src/ros2_controllers/`
(branch `master`, ~**6.7.0**, ROS 2 Rolling/Jazzy-Kilted line).

- **Upstream:** <https://github.com/ros-controls/ros2_controllers>
- **Framework docs:** <https://control.ros.org>
- **Sibling framework repo** (`controller_manager`, `controller_interface`,
  `hardware_interface`): the `ros2_control` repo — not vendored here, but
  every controller in this repo links `controller_interface::controller_interface`.
- **Runnable examples / hardware side:** `~/nav2_ws/src/ros2_control_demos/`
  — see `ros2_control_demos.md` (catalog) and
  `skills/ros2-control-hardware-interface/SKILL.md` (write a hardware plugin).

> A controller in ros2_control is **not** a standalone node. It is a
> **pluginlib plugin** loaded into the `controller_manager` process and
> ticked synchronously in the real-time `update()` loop. It talks to
> hardware only through **command/state interfaces**, never by opening its
> own hardware handles.

---

## 1. The big picture

```
        ┌──────────────────────── controller_manager (the RT host) ────────────────────────┐
        │  reads /robot_description, loads controllers as plugins, runs the update loop      │
        │                                                                                    │
 ros2   │   ControllerInterfaceBase  ◄── plugin ── DiffDriveController, JointTrajectory…     │
 topics │        │  update()                                                                 │
 srvs   │        ▼                                                                           │
 actions│   LoanedCommandInterface / LoanedStateInterface   (claimed per cycle)             │
        └────────────────────────────────┬───────────────────────────────────────────────-─┘
                                          │ read state / write command
                          ┌───────────────▼───────────────┐
                          │     hardware_interface         │  (SystemInterface / Actuator / Sensor)
                          │  exports command + state ifaces │
                          └───────────────┬────────────────┘
                                          ▼
                                     real / sim hardware
```

The `controller_manager`:
1. loads each controller via **pluginlib** (`controller_interface`),
2. calls its lifecycle transitions,
3. asks each active controller which interfaces it needs
   (`command_interface_configuration()` / `state_interface_configuration()`),
4. **claims** those interfaces from `hardware_interface` and hands them
   to the controller as `LoanedCommandInterface` / `LoanedStateInterface`,
5. ticks `read() → update() → write()` every control period.

---

## 2. Command vs. state interfaces

The hardware-abstraction unit is a **named interface string**
`"<joint_or_sensor>/<interface_type>"`:

| Kind | Direction | Examples |
|------|-----------|----------|
| **command interface** | controller → hardware | `wheel_left/velocity`, `joint1/position`, `gpio1/digital_output` |
| **state interface**   | hardware → controller | `wheel_left/velocity`, `joint1/position`, `imu/orientation.x` |

Standard interface types: `position`, `velocity`, `acceleration`,
`effort` (`hardware_interface::HW_IF_*`). Sensors export composite ones
via `semantic_components` (e.g. `IMUSensor`, `ForceTorqueSensor`).

A controller declares what it needs with:

```cpp
controller_interface::InterfaceConfiguration command_interface_configuration() const override;
controller_interface::InterfaceConfiguration state_interface_configuration() const override;
// type ∈ { ALL, INDIVIDUAL, NONE };  names = explicit list when INDIVIDUAL
```

- A **controller** typically claims command interfaces (`INDIVIDUAL`) and
  the matching state interfaces for feedback.
- A **broadcaster** claims **`NONE`** command interfaces and only reads
  state interfaces — it never actuates, it just publishes to ROS.

---

## 3. Controller lifecycle (LifecycleNode under the hood)

Every controller is a managed (lifecycle) node. Override only what you need:

| Method | When | Notes |
|--------|------|-------|
| `on_init()` | once, right after load | declare the `generate_parameter_library` `ParamListener`; return `ERROR` on bad setup. **No hardware yet.** |
| `on_configure(state)` | inactive | read params, create publishers/subscribers/services, allocate buffers. **No interfaces claimed yet.** |
| `on_activate(state)` | active | grab `LoanedCommandInterface`/`LoanedStateInterface` references; start commanding. |
| `on_deactivate(state)` | inactive | release interface handles; stop commanding. |
| `on_cleanup(state)` | unconfigured | free resources created in configure. |
| `on_error(state)` | error | recover or fail. |
| `update(time, period)` | **every RT cycle** | the hot path — see §5. |

Return type for transitions is
`controller_interface::CallbackReturn { SUCCESS, FAILURE, ERROR }`;
`update()` returns `controller_interface::return_type { OK, ERROR }`.

---

## 4. Plain vs. chainable controllers

**Plain** — `controller_interface::ControllerInterface` (12 pkgs here):

```cpp
controller_interface::return_type update(
  const rclcpp::Time & time, const rclcpp::Duration & period) override;
```

**Chainable** — `controller_interface::ChainableControllerInterface`
(9 pkgs). A chainable controller can expose **reference interfaces** so
another controller upstream can write into it (controller chaining), and
splits `update()` in two:

```cpp
// runs when NOT in chained mode — pull the reference from ROS subscribers
return_type update_reference_from_subscribers(time, period) override;
// always runs — convert the reference into hardware commands
return_type update_and_write_commands(time, period) override;

bool on_set_chained_mode(bool chained_mode) override;
std::vector<hardware_interface::CommandInterface> on_export_reference_interfaces() override; // controllers
std::vector<hardware_interface::StateInterface>   on_export_state_interfaces() override;     // broadcasters
```

When chained, the upstream controller writes the reference interface
directly and `update_reference_from_subscribers()` is skipped.

---

## 5. Real-time rules for `update()`

`update()` runs in the controller_manager's RT thread. It **must not**:

- allocate (`new`, `std::vector` growth, `std::string` ops), lock an
  unbounded mutex, throw, log unthrottled, or do any blocking I/O.

Use the project's RT tooling instead:

- **`realtime_tools::RealtimePublisher<T>`** — lock-free publish from
  `update()`; never call a plain `publisher->publish()` in the hot path.
- **`realtime_tools::RealtimeThreadSafeBox<T>`** (and the older
  `RealtimeBuffer`) — exchange data between the subscriber callback
  (non-RT) and `update()` (RT) without blocking.
- Cache message objects and interface index lookups in `on_configure` /
  `on_activate`, not in `update()`.

---

## 6. Parameters — `generate_parameter_library`

Controllers do **not** hand-roll `declare_parameter`. They ship a
`src/<pkg>_parameters.yaml`, and CMake generates a typed header:

```cmake
generate_parameter_library(<pkg>_parameters src/<pkg>_parameters.yaml)
```

```yaml
diff_drive_controller:
  wheel_radius: {
    type: double,
    default_value: 0.0,
    description: "Radius of a wheel…",
    read_only: true,                 # only settable before configure
    validation: { gt<>: [0.0] }      # built-in + custom validators
  }
```

```cpp
#include "diff_drive_controller/diff_drive_controller_parameters.hpp"
param_listener_ = std::make_shared<ParamListener>(get_node());
params_ = param_listener_->get_params();   // refresh in update() if dynamic
```

---

## 7. Plugin registration

Each package exports its controller as a `controller_interface` plugin:

```xml
<!-- diff_drive_plugin.xml -->
<library path="diff_drive_controller">
  <class name="diff_drive_controller/DiffDriveController"
         type="diff_drive_controller::DiffDriveController"
         base_class_type="controller_interface::ChainableControllerInterface">
    <description>…</description>
  </class>
</library>
```

```cmake
pluginlib_export_plugin_description_file(controller_interface diff_drive_plugin.xml)
```

```cpp
// bottom of the .cpp
#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  diff_drive_controller::DiffDriveController,
  controller_interface::ControllerInterface)   // or ChainableControllerInterface
```

The `name` alias (`<pkg>/<Class>`) is what users put under
`controller_manager`'s `ros__parameters.<name>.type` — **it is API; do
not rename it once released.**

---

## 8. Package layout convention

```
<pkg>/
├── include/<pkg>/<pkg>.hpp            # controller class
├── src/<pkg>.cpp                      # implementation + PLUGINLIB_EXPORT_CLASS
├── src/<pkg>_parameters.yaml         # generate_parameter_library input
├── <pkg>_plugin.xml                  # pluginlib description
├── CMakeLists.txt                    # generate_parameter_library + export
├── package.xml                       # depends controller_interface, pluginlib, …
├── doc/userdoc.rst                   # control.ros.org docs
└── test/                             # gtest + controller_manager test fixtures
```

---

## 9. Controllers vs. broadcasters (mental model)

| | Controller | Broadcaster |
|--|-----------|-------------|
| Command interfaces | claims & writes | **NONE** |
| State interfaces | reads (feedback) | reads (the data to publish) |
| ROS input | topic/action/service → reference | none |
| ROS output | status/odometry (optional) | the sensor/state message |
| Examples | diff_drive, JTC, pid, forward_command | imu/pose/range/joint_state broadcasters |

See `ros2_controllers_reference.md` for the full per-package catalog.
