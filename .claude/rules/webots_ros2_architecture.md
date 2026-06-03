# webots_ros2 — Architecture Reference

The official ROS 2 interface for the **Webots** simulator. Source:
`~/nav2_ws/src/webots_ros2/` (~**2025.0.1**). A simulation alternative to
gz-sim: instead of an ECS, a Webots robot is driven by a **`webots_ros2_driver`
node** that maps Webots *devices* to ROS 2 topics via URDF-declared
plugins.

- Upstream + docs: <https://github.com/cyberbotics/webots_ros2>,
  <https://docs.ros.org/en/rolling/p/webots_ros2/>
- Mostly **C++** for the driver/control (performance), **Python** for the
  importer and the robot demos.

> Mental model: Webots simulates physics and devices. A robot's URDF
> carries a `<webots>` block that says "this Webots device → that ROS 2
> topic" and "load these plugins". The `webots_controller` process runs a
> `WebotsNode` that, each Webots timestep, ticks every plugin's `step()`
> to read sensors / write actuators over ROS 2.

---

## 1. Package map

| Package | Build | Purpose |
|---------|-------|---------|
| `webots_ros2_driver` | ament_cmake (C++/Py) | **Core** — `WebotsNode`, `Driver.cpp`, device plugins, `Ros2Supervisor`, `WebotsLauncher`/`WebotsController` launch helpers |
| `webots_ros2_control` | ament_cmake (C++) | `ros2_control` `SystemInterface` bridging Webots motors/sensors to `controller_manager` |
| `webots_ros2_importer` | ament_python | URDF / Xacro → Webots **PROTO** converter (`urdf2proto`, `xacro2proto`) |
| `webots_ros2_msgs` | ament_cmake | Webots msgs/srvs (`FloatStamped`, `CameraRecognitionObject(s)`, `SpawnNodeFromString`, …) |
| `webots_ros2_epuck` / `_turtlebot` / `_mavic` / `_tesla` / `_tiago` / `_universal_robot` / `_crazyflie` / `_husarion` | ament_python | Runnable robot demos — copy these for launch + URDF + ros2_control wiring |
| `webots_ros2_tests` | ament_python | integration tests |

---

## 2. How a Webots robot becomes a ROS 2 node

```
WebotsLauncher(world=…, ros2_supervisor=True)   # starts Webots + the .wbt world
        │
WebotsController(robot_name=…, parameters=[{robot_description: …urdf}])
        │  → runs the `webots-controller` wrapper → Driver.cpp
        ▼
WebotsNode (rclcpp::Node)  ── loads plugins from the URDF <webots> block
        │  every Webots timestep:
        ▼
   plugin.step()  ×N   ── read Webots device → publish ROS msg / write actuator
```

- **`WebotsLauncher`** (`webots_ros2_driver/webots_launcher.py`) starts the
  simulator with a `.wbt` world; `ros2_supervisor=True` adds a supervisor
  robot for dynamic URDF spawning (`Ros2SupervisorLauncher`).
- **`WebotsController`** (`webots_ros2_driver/webots_controller.py`) wraps
  `ExecuteProcess`, connects the driver to a named robot
  (`--robot-name`), and supports `respawn=True` for world resets.

---

## 3. The URDF `<webots>` mechanism

A robot's URDF carries Webots wiring inside a `<webots>` element:

```xml
<robot name="MyRobot">
  <webots>
    <!-- map a Webots device (by its Webots name) to a ROS 2 topic -->
    <device reference="lidar" type="Lidar">
      <ros>
        <topicName>/scan</topicName>
        <enabled>true</enabled>
        <alwaysOn>true</alwaysOn>
      </ros>
    </device>

    <!-- load a dynamic plugin (C++ or Python) -->
    <plugin type="webots_ros2_control::Ros2Control" />
    <plugin type="my_pkg.plugins.MyPlugin" />   <!-- python: module.path.Class -->
  </webots>

  <!-- standard ros2_control block (consumed by Ros2ControlSystem) -->
  <ros2_control name="WebotsControl" type="system">
    <hardware><plugin>webots_ros2_control::Ros2ControlSystem</plugin></hardware>
    <joint name="left wheel motor">
      <state_interface name="position"/>
      <command_interface name="velocity"/>
    </joint>
  </ros2_control>
</robot>
```

- `<device reference="…" type="…">` — `reference` matches the Webots
  device name; `type` is the Webots class (`Lidar`, `Camera`,
  `DistanceSensor`, `Motor`, `InertialUnit`, …).
- `<ros>` configures the topic and sensor flags (`enabled`, `alwaysOn`).
- `<plugin type="…">` — a C++ pluginlib class (`ns::Class`) or a Python
  class (`module.path.Class`).

---

## 4. Device plugin interface

The driver ticks every plugin once per timestep. Two flavours:

**C++** — `webots_ros2_driver::PluginInterface`
(`include/webots_ros2_driver/PluginInterface.hpp`):

```cpp
class PluginInterface {
public:
  // called once before spinning; params come from the URDF <plugin>/<device>
  virtual void init(WebotsNode *node,
                    std::unordered_map<std::string, std::string> &parameters) = 0;
  // called every Webots timestep — do NOT call robot.step() yourself
  virtual void step() = 0;
};
```

Sensor plugins derive from `Ros2SensorPlugin`. Registered via pluginlib
(`webots_ros2_*.xml`, `base_class_type="webots_ros2_driver::PluginInterface"`).
Built-ins: `Ros2Lidar`, `Ros2Camera`, `Ros2RangeFinder`, `Ros2IMU`,
`Ros2GPS`, `Ros2DistanceSensor`, `Ros2LED`, `Ros2Compass`,
`Ros2Receiver`/`Ros2Emitter`, …

**Python** — a class the driver calls into:

```python
class MyPlugin:
    def init(self, webots_node, properties):
        self.robot = webots_node.robot          # Webots Driver/Supervisor handle
        self.dev = self.robot.getDevice('my_sensor')
        # create rclpy pubs/subs via webots_node…
    def step(self):
        ...                                      # read device, publish
```

### Device → standard message map

| Webots device | ROS 2 message | plugin |
|---------------|---------------|--------|
| Camera | `sensor_msgs/Image` | `Ros2Camera` |
| Lidar | `sensor_msgs/LaserScan` + `PointCloud2` | `Ros2Lidar` |
| RangeFinder | `sensor_msgs/Image` (depth) | `Ros2RangeFinder` |
| DistanceSensor | `sensor_msgs/Range` | `Ros2DistanceSensor` |
| InertialUnit+Gyro+Accel (fused) | `sensor_msgs/Imu` | `Ros2IMU` |
| GPS | `sensor_msgs/NavSatFix` | `Ros2GPS` |
| Compass | `geometry_msgs/Vector3Stamped` | `Ros2Compass` |
| LED | `std_msgs/UInt32` | `Ros2LED` |

---

## 5. ros2_control bridge (`webots_ros2_control`)

`webots_ros2_control::Ros2ControlSystem` is a
`hardware_interface::SystemInterface` that maps URDF `<ros2_control>`
joints to Webots motors + position sensors:

```cpp
class Ros2ControlSystem : public Ros2ControlSystemInterface {
  void init(webots_ros2_driver::WebotsNode *node,
            const hardware_interface::HardwareInfo &info) override;   // parse <joint>s
  std::vector<hardware_interface::StateInterface>   export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;
  hardware_interface::return_type read (const rclcpp::Time&, const rclcpp::Duration&) override;
  hardware_interface::return_type write(const rclcpp::Time&, const rclcpp::Duration&) override;
};
```

It is loaded by the driver via the `<plugin>webots_ros2_control::Ros2Control</plugin>`
entry in `<webots>`, and `controller_manager` then spawns the normal
controllers (`diff_drive_controller`, `joint_state_broadcaster`, …) — so
the whole `ros2_controllers` suite (see `ros2_control_architecture.md`)
works against Webots. The robot's controllers come from a `ros2_control.yml`
passed to `WebotsController`.

---

## 6. Launch idiom (e.g. `webots_ros2_epuck`)

```python
from webots_ros2_driver.webots_launcher import WebotsLauncher
from webots_ros2_driver.webots_controller import WebotsController

webots = WebotsLauncher(world=PathJoinSubstitution([pkg, 'worlds', world]),
                        ros2_supervisor=True)
robot = WebotsController(
    robot_name='e-puck',
    parameters=[{'robot_description': urdf_path, 'use_sim_time': True,
                 'set_robot_state_publisher': True}, ros2_control_params],
    remappings=[('/diffdrive_controller/cmd_vel', '/cmd_vel')],
    respawn=True)
# + robot_state_publisher + controller_manager spawners (joint_state_broadcaster first)
return LaunchDescription([webots, robot_state_publisher, robot, *spawners])
```

---

## 7. Importer (`webots_ros2_importer`)

Converts robot descriptions into Webots PROTO so a URDF robot can live in
a `.wbt` world:

```bash
ros2 run webots_ros2_importer urdf2proto --input robot.urdf --output robot.proto
```

Python API: `webots_ros2_importer.urdf2webots.importer.convertUrdfFile(...)`.
Options: `--box-collision`, `--normal`, `--tool-slot`, `--init-pos`, …
(xacro is expanded first).

---

## 8. Where webots fits vs the rest of this template

- **vs gz-sim** (`gz-ecs-overview`): both are simulators. gz-sim uses an
  ECS + system plugins; webots_ros2 uses URDF-declared device plugins and
  a per-robot driver node. Pick per project.
- **with ros2_control** (`ros2_control_architecture.md`): `Ros2ControlSystem`
  is the Webots hardware component, so `/new-controller` controllers and
  `joint_state_broadcaster` run unchanged.
- **with Nav 2**: webots demos (turtlebot, tiago) provide odom + `/scan`
  + TF, so Nav 2 bringup works on top.

To write a device plugin, see skill `webots_ros2_device_plugin`.
