---
name: webots_ros2_device_plugin
description: Write a webots_ros2 device/robot plugin (C++ PluginInterface or Python init/step class), declare it + Webots devices in the URDF <webots> block, wire ros2_control, and build a launch file. Trigger when the user asks to interface a Webots robot/sensor with ROS 2 or write a webots_ros2 plugin.
---

# Writing a webots_ros2 device plugin + bringup

How to expose a Webots robot/sensor to ROS 2: write a driver plugin,
declare it in the URDF `<webots>` block, optionally bridge motors through
ros2_control, and launch it.

- Architecture + device→message map + URDF mechanism:
  `rules/webots_ros2_architecture.md`.
- Controller side (what commands the joints): `ros2_control_architecture.md`
  + skill `ros2_controller_creation`.
- Copy from the real demos in `~/nav2_ws/src/webots_ros2/`:
  `webots_ros2_epuck` (sensors + diff-drive ros2_control),
  `webots_ros2_turtlebot` (Nav 2-ready), and the built-in plugins under
  `webots_ros2_driver/.../plugins/` (`Ros2Lidar`, `Ros2Camera`, …).

## First decision

| You need to… | Do this |
|--------------|---------|
| Publish a **standard** sensor (lidar/camera/IMU/GPS/range/…) | just declare a `<device>` — a built-in plugin already handles it; **no code** |
| Custom sensor logic / fuse / non-standard device | write a **plugin** (C++ `PluginInterface` or Python `init/step`) |
| Drive **motors** with controllers | use `webots_ros2_control::Ros2ControlSystem` + a `<ros2_control>` block, then spawn controllers |

Prefer the no-code `<device>` path; only write a plugin when behaviour is
custom.

## A. Declare devices in the URDF `<webots>` block (no code)

```xml
<robot name="MyRobot">
  <webots>
    <device reference="lidar" type="Lidar">
      <ros>
        <topicName>/scan</topicName>
        <enabled>true</enabled>
        <alwaysOn>true</alwaysOn>
        <frameName>lidar_link</frameName>
      </ros>
    </device>
    <device reference="camera" type="Camera">
      <ros><topicName>/camera/image_raw</topicName></ros>
    </device>
  </webots>
</robot>
```

`reference` must equal the Webots device name; `type` is the Webots class.

## B. Custom C++ plugin

```cpp
// my_pkg/include/my_pkg/MySensor.hpp
#include "webots_ros2_driver/plugins/Ros2SensorPlugin.hpp"   // or PluginInterface.hpp
#include <webots/distance_sensor.h>

namespace my_pkg {
class MySensor : public webots_ros2_driver::Ros2SensorPlugin {
public:
  void init(webots_ros2_driver::WebotsNode *node,
            std::unordered_map<std::string, std::string> &params) override {
    Ros2SensorPlugin::init(node, params);                 // parses <ros> (topic/frame/qos)
    mDevice = wb_robot_get_device(params["device_name"].c_str());
    wb_distance_sensor_enable(mDevice, node->robot()->getBasicTimeStep());
    mPub = node->create_publisher<sensor_msgs::msg::Range>(mTopicName, rclcpp::SensorDataQoS());
  }
  void step() override {                                   // every Webots timestep
    if (!preStep()) return;                                // honours enabled/alwaysOn
    sensor_msgs::msg::Range m; m.range = wb_distance_sensor_get_value(mDevice);
    mPub->publish(m);
  }
private:
  WbDeviceTag mDevice;
  rclcpp::Publisher<sensor_msgs::msg::Range>::SharedPtr mPub;
};
}  // namespace my_pkg
```

Register with pluginlib + the URDF:

```xml
<!-- my_pkg/my_plugin.xml -->
<library path="my_pkg">
  <class type="my_pkg::MySensor" base_class_type="webots_ros2_driver::PluginInterface">
    <description>Custom distance sensor.</description>
  </class>
</library>
```
```cmake
pluginlib_export_plugin_description_file(webots_ros2_driver my_plugin.xml)
```
```xml
<webots><plugin type="my_pkg::MySensor"/></webots>
```

## C. Custom Python plugin (no build step)

```python
# my_pkg/my_pkg/my_plugin.py
import rclpy
from std_msgs.msg import Float32

class MyPlugin:
    def init(self, webots_node, properties):
        self.robot = webots_node.robot                     # Webots Driver handle
        self.sensor = self.robot.getDevice('my_sensor')
        self.sensor.enable(int(self.robot.getBasicTimeStep()))
        self.node = rclpy.create_node('my_plugin')
        self.pub = self.node.create_publisher(Float32, properties.get('topic', '/my_sensor'), 10)
    def step(self):
        rclpy.spin_once(self.node, timeout_sec=0)
        self.pub.publish(Float32(data=float(self.sensor.getValue())))
```
```xml
<webots>
  <plugin type="my_pkg.my_plugin.MyPlugin">
    <topic>/my_sensor</topic>
  </plugin>
</webots>
```

Do **not** call `robot.step()` inside `step()` — the driver does it.

## D. Motors via ros2_control

```xml
<webots><plugin type="webots_ros2_control::Ros2Control"/></webots>
<ros2_control name="WebotsControl" type="system">
  <hardware><plugin>webots_ros2_control::Ros2ControlSystem</plugin></hardware>
  <joint name="left wheel motor">
    <state_interface name="position"/><command_interface name="velocity"/>
  </joint>
  <joint name="right wheel motor">
    <state_interface name="position"/><command_interface name="velocity"/>
  </joint>
</ros2_control>
```
Then spawn `joint_state_broadcaster` (first) and e.g. `diff_drive_controller`
from a `ros2_control.yml`.

## E. Launch

Copy `webots_ros2_epuck/launch/robot_launch.py`: `WebotsLauncher(world=…,
ros2_supervisor=True)` + `robot_state_publisher` + `WebotsController(
robot_name=…, parameters=[{robot_description: urdf}, ros2_control_params],
respawn=True)` + the controller spawners.

## Common pitfalls

- **Calling `robot.step()`** in a plugin `step()` — the driver owns the
  loop; just read/write devices.
- **`reference` ≠ the Webots device name**, or wrong `type` — the device
  won't be found.
- Writing a plugin when a **built-in `<device>`** already covers the
  sensor — declare, don't code.
- `base_class_type` must be `webots_ros2_driver::PluginInterface`; the
  pluginlib export must target `webots_ros2_driver`.
- Forgetting `use_sim_time: true` everywhere, or spawning the command
  controller before `joint_state_broadcaster`.
- Python plugin: enable the device with the **basic time step** and pump
  rclpy with `spin_once(timeout_sec=0)` (non-blocking).
