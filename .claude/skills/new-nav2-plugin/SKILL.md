---
name: new-nav2-plugin
description: Scaffold a new Nav 2 plugin (controller, planner, behavior, smoother, goal-checker, progress-checker, costmap layer, or BT node). Wires pluginlib registration, parameter declaration on the lifecycle node, and a minimal integration test. Trigger when the user asks to write or extend a Nav 2 plugin.
---

# Scaffolding a Nav 2 plugin

Nav 2 exposes every behaviour as a `pluginlib`-loaded class behind a
small set of `nav2_core::*` (or `nav2_costmap_2d::Layer`, or
`BT::ActionNodeBase`) interfaces. The recipe is the same shape for
every plugin kind — only the base class and the host server change.

## Decide first

| Plugin kind         | Base class                                  | Loaded by                  | Plugin description file |
|---------------------|---------------------------------------------|----------------------------|-------------------------|
| Controller          | `nav2_core::Controller`                     | `controller_server`        | `nav2_core_plugins.xml` |
| Global planner      | `nav2_core::GlobalPlanner`                  | `planner_server`           | `nav2_core_plugins.xml` |
| Smoother            | `nav2_core::Smoother`                       | `smoother_server`          | `nav2_core_plugins.xml` |
| Goal checker        | `nav2_core::GoalChecker`                    | `controller_server`        | `nav2_core_plugins.xml` |
| Progress checker    | `nav2_core::ProgressChecker`                | `controller_server`        | `nav2_core_plugins.xml` |
| Behavior            | `nav2_core::Behavior`                       | `behavior_server`          | `nav2_core_plugins.xml` |
| Costmap layer       | `nav2_costmap_2d::Layer` / `CostmapLayer`   | `local_/global_costmap`    | `nav2_costmap_2d_plugins.xml` |
| BT action / cond.   | `BT::SyncActionNode` / `BT::ConditionNode`  | `bt_navigator`             | `nav2_tree_nodes.xml`   |

See `.claude/skills/nav2-core-interfaces/SKILL.md` for the full method
list of each base.

## Step 1 — header

For a controller (other plugin kinds are analogous):

```cpp
// include/<pkg>/<snake_class>.hpp
#ifndef <PKG>__<SNAKE_CLASS>_HPP_
#define <PKG>__<SNAKE_CLASS>_HPP_

#include <memory>
#include <string>

#include <nav2_core/controller.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>

namespace <pkg>
{

class <ClassName> : public nav2_core::Controller
{
public:
  <ClassName>() = default;
  ~<ClassName>() override = default;

  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    std::string name,
    std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;

  void cleanup()    override;
  void activate()   override;
  void deactivate() override;

  geometry_msgs::msg::TwistStamped computeVelocityCommands(
    const geometry_msgs::msg::PoseStamped & pose,
    const geometry_msgs::msg::Twist & velocity,
    nav2_core::GoalChecker * goal_checker) override;

  void setPlan(const nav_msgs::msg::Path & path) override;
  void setSpeedLimit(const double & speed_limit, const bool & percentage) override;

private:
  rclcpp_lifecycle::LifecycleNode::WeakPtr node_;
  std::string plugin_name_;
  std::shared_ptr<tf2_ros::Buffer> tf_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;

  // Declared params
  double desired_linear_vel_{0.0};
};

}  // namespace <pkg>

#endif
```

## Step 2 — source

```cpp
// src/<snake_class>.cpp
#include "<pkg>/<snake_class>.hpp"

#include <pluginlib/class_list_macros.hpp>

namespace <pkg>
{

void <ClassName>::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  std::string name,
  std::shared_ptr<tf2_ros::Buffer> tf,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  node_         = parent;
  plugin_name_  = name;
  tf_           = tf;
  costmap_ros_  = costmap_ros;

  auto node = node_.lock();
  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name_ + ".desired_linear_vel",
    rclcpp::ParameterValue(0.5));
  node->get_parameter(plugin_name_ + ".desired_linear_vel",
                      desired_linear_vel_);
}

void <ClassName>::cleanup()    {}
void <ClassName>::activate()   {}
void <ClassName>::deactivate() {}

geometry_msgs::msg::TwistStamped
<ClassName>::computeVelocityCommands(
  const geometry_msgs::msg::PoseStamped & /*pose*/,
  const geometry_msgs::msg::Twist & /*velocity*/,
  nav2_core::GoalChecker * /*goal_checker*/)
{
  geometry_msgs::msg::TwistStamped cmd_vel;
  cmd_vel.header.stamp = node_.lock()->now();
  cmd_vel.twist.linear.x = desired_linear_vel_;
  return cmd_vel;
}

void <ClassName>::setPlan(const nav_msgs::msg::Path & /*path*/) {}
void <ClassName>::setSpeedLimit(const double & /*speed_limit*/,
                                const bool   & /*percentage*/) {}

}  // namespace <pkg>

PLUGINLIB_EXPORT_CLASS(<pkg>::<ClassName>, nav2_core::Controller)
```

Critical points:

* **Always** use `nav2_util::declare_parameter_if_not_declared` —
  Nav 2 calls `configure` more than once across lifecycle transitions.
* Parameters are keyed under `<plugin_name>.<param>` because users set
  them under the plugin alias they pick in YAML, not under the plugin
  class name.
* `PLUGINLIB_EXPORT_CLASS` last argument must match the loader's
  expected base class (`nav2_core::Controller`,
  `nav2_costmap_2d::Layer`, …).

## Step 3 — pluginlib manifest (`plugins.xml`)

```xml
<library path="<pkg>">
  <class
    type="<pkg>::<ClassName>"
    base_class_type="nav2_core::Controller">
    <description>
      One-line description of what this controller does.
    </description>
  </class>
</library>
```

## Step 4 — `CMakeLists.txt`

```cmake
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(pluginlib REQUIRED)
find_package(nav2_core REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(nav2_util REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)

add_library(${PROJECT_NAME} SHARED src/<snake_class>.cpp)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

ament_target_dependencies(${PROJECT_NAME}
  rclcpp rclcpp_lifecycle pluginlib nav2_core nav2_costmap_2d
  nav2_util tf2_ros geometry_msgs nav_msgs)

pluginlib_export_plugin_description_file(nav2_core plugins.xml)

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

install(DIRECTORY include/ DESTINATION include)
install(FILES plugins.xml DESTINATION share/${PROJECT_NAME})

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(rclcpp rclcpp_lifecycle pluginlib nav2_core)

ament_package()
```

For costmap layers replace `nav2_core` with `nav2_costmap_2d` in the
`pluginlib_export_plugin_description_file` call.

## Step 5 — `package.xml`

```xml
<depend>rclcpp</depend>
<depend>rclcpp_lifecycle</depend>
<depend>pluginlib</depend>
<depend>nav2_core</depend>
<depend>nav2_costmap_2d</depend>
<depend>nav2_util</depend>
<depend>tf2_ros</depend>
<depend>geometry_msgs</depend>
<depend>nav_msgs</depend>
```

## Step 6 — YAML stub (`config/<snake_class>.yaml`)

```yaml
controller_server:
  ros__parameters:
    controller_plugins: ["FollowPath"]
    FollowPath:
      plugin: "<pkg>::<ClassName>"
      desired_linear_vel: 0.5
```

The key `FollowPath` is the **alias** users will set in their nav2
params; the `plugin:` line is the fully-qualified class name.

## Step 7 — integration test

```cpp
// test/integration/test_load_plugin.cpp
#include <gtest/gtest.h>
#include <pluginlib/class_loader.hpp>
#include <nav2_core/controller.hpp>

TEST(LoadPlugin, IsConstructible)
{
  pluginlib::ClassLoader<nav2_core::Controller> loader(
    "nav2_core", "nav2_core::Controller");
  auto plugin = loader.createSharedInstance("<pkg>::<ClassName>");
  EXPECT_NE(plugin, nullptr);
}
```

## Step 8 — verify

```bash
/build <pkg>
/test  <pkg>
```

Then point a Nav 2 stack at the new plugin via the YAML and
`ros2 launch nav2_bringup tb3_simulation_launch.py`.

## Common mistakes

* Forgetting `PLUGINLIB_EXPORT_CLASS` — the loader will silently fail
  to find the class.
* Declaring parameters under the class name instead of `plugin_name_`.
* Hard-coding the plugin alias in YAML examples — keep them as
  placeholders.
* Linking `nav2_core` without `nav2_util` — `nav2_util::declare_*` is
  in a separate target.
* Costmap layer that allocates inside `updateCosts()` — that runs at
  costmap rate and must be hot-path-safe.
