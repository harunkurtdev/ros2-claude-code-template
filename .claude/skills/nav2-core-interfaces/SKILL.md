---
name: nav2-core-interfaces
description: Reference for the nav2_core plugin base classes — Controller, GlobalPlanner, Smoother, GoalChecker, ProgressChecker, Behavior — and their exception classes. Trigger when the user implements or reasons about a custom Nav 2 plugin interface.
---

# Nav2 Core Plugin Interfaces

Source: `~/nav2_ws/src/navigation2/nav2_core/include/nav2_core/`

## Plugin Base Classes

All Nav2 plugins implement these abstract interfaces. Infrastructure layer implements them; domain/application layers depend on them.

### 1. Controller (Local Planner)

**Header:** `controller.hpp`

```cpp
class Controller {
public:
  virtual void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr &,
    std::string name, std::shared_ptr<tf2_ros::Buffer>,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS>) = 0;
  virtual void cleanup() = 0;
  virtual void activate() = 0;
  virtual void deactivate() = 0;
  virtual void newPathReceived() {};
  virtual geometry_msgs::msg::TwistStamped computeVelocityCommands(
    const geometry_msgs::msg::PoseStamped & pose,
    const geometry_msgs::msg::Twist & velocity,
    nav2_core::GoalChecker * goal_checker) = 0;
  virtual void cancel() {};
  virtual void setSpeedLimit(const double & speed_limit, const bool & percentage) = 0;
  virtual void reset() {};
};
```

**Implementations:** DWB, MPPI, RegulatedPurePursuit, Graceful, RotationShim

### 2. GlobalPlanner

**Header:** `global_planner.hpp`

```cpp
class GlobalPlanner {
public:
  virtual void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr &,
    std::string name, std::shared_ptr<tf2_ros::Buffer>,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS>) = 0;
  virtual void cleanup() = 0;
  virtual void activate() = 0;
  virtual void deactivate() = 0;
  virtual nav_msgs::msg::Path createPlan(
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal) = 0;
};
```

**Implementations:** NavFn, SmacPlanner2D, SmacPlannerHybrid, SmacPlannerLattice, ThetaStar

### 3. Smoother

**Header:** `smoother.hpp`

```cpp
class Smoother {
public:
  virtual void configure(...) = 0;
  virtual void cleanup() = 0;
  virtual void activate() = 0;
  virtual void deactivate() = 0;
  virtual bool smooth(
    nav_msgs::msg::Path & path,
    const nav2_costmap_2d::Costmap2D & costmap) = 0;
};
```

**Implementations:** SimpleSmoother, SavitzkyGolaySmoother, ConstrainedSmoother

### 4. GoalChecker

**Header:** `goal_checker.hpp`

```cpp
class GoalChecker {
public:
  virtual void initialize(...) = 0;
  virtual void reset() = 0;
  virtual bool isGoalReached(
    const geometry_msgs::msg::Pose & query_pose,
    const geometry_msgs::msg::Pose & goal_pose,
    const geometry_msgs::msg::Twist & velocity) = 0;
  virtual bool getTolerances(
    geometry_msgs::msg::Pose & pose_tolerance,
    geometry_msgs::msg::Twist & vel_tolerance) = 0;
};
```

### 5. ProgressChecker

**Header:** `progress_checker.hpp`

```cpp
class ProgressChecker {
public:
  virtual void initialize(...) = 0;
  virtual bool check(const geometry_msgs::msg::PoseStamped & current_pose) = 0;
  virtual void reset() = 0;
};
```

### 6. PathHandler

**Header:** `path_handler.hpp`

```cpp
class PathHandler {
public:
  virtual void initialize(...) = 0;
  virtual void setPlan(const nav_msgs::msg::Path & path) = 0;
  virtual nav_msgs::msg::Path findPlanSegment(...) = 0;
  virtual nav_msgs::msg::Path transformLocalPlan(...) = 0;
  virtual geometry_msgs::msg::PoseStamped getTransformedGoal(...) = 0;
};
```

### 7. Behavior

**Header:** `behavior.hpp`

```cpp
enum class CostmapInfoType { NONE, LOCAL, GLOBAL, BOTH };

class Behavior {
public:
  virtual void configure(...) = 0;
  virtual void cleanup() = 0;
  virtual void activate() = 0;
  virtual void deactivate() = 0;
  virtual CostmapInfoType getResourceInfo() = 0;
};
```

**Implementations:** Spin, BackUp, DriveOnHeading, Wait, AssistedTeleop

### 8. WaypointTaskExecutor

**Header:** `waypoint_task_executor.hpp`

```cpp
class WaypointTaskExecutor {
public:
  virtual void initialize(...) = 0;
  virtual bool processAtWaypoint(
    const geometry_msgs::msg::PoseStamped & current_pose,
    const int & current_waypoint_index) = 0;
};
```

### 9. BehaviorTreeNavigator

**Header:** `behavior_tree_navigator.hpp`

Template class for BT-based navigation action plugins. Contains:
- `FeedbackUtils` struct (robot_frame, global_frame, transform_tolerance, tf)
- `NavigatorMuxer` (ensures single active navigator)
- `BehaviorTreeNavigator<ActionT>` (template with BT lifecycle)

## Exception Classes

### Controller Exceptions (`controller_exceptions.hpp`)
- `ControllerException`, `InvalidController`, `ControllerTFError`
- `FailedToMakeProgress`, `PatienceExceeded`, `InvalidPath`
- `NoValidControl`, `ControllerTimedOut`

### Planner Exceptions (`planner_exceptions.hpp`)
- `PlannerException`, `InvalidPlanner`, `StartOccupied`, `GoalOccupied`
- `StartOutsideMapBounds`, `GoalOutsideMapBounds`
- `NoValidPathCouldBeFound`, `PlannerTimedOut`, `PlannerTFError`
- `NoViapointsGiven`, `PlannerCancelled`

### Smoother Exceptions (`smoother_exceptions.hpp`)
- `SmootherException`, `InvalidSmoother`, `InvalidPath`
- `SmootherTimedOut`, `SmoothedPathInCollision`, `FailedToSmoothPath`

### Route Exceptions (`route_exceptions.hpp`)
- `RouteException`, `OperationFailed`, `NoValidRouteCouldBeFound`
- `TimedOut`, `RouteTFError`, `NoValidGraph`
- `IndeterminantNodesOnGraph`, `InvalidEdgeScorerUse`

## Creating a Custom Plugin

### Custom Controller Example (C++)

```cpp
#include "nav2_core/controller.hpp"

namespace my_controller {

class MyController : public nav2_core::Controller {
public:
  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    std::string name,
    std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override
  {
    // Initialize from parameters
  }

  geometry_msgs::msg::TwistStamped computeVelocityCommands(
    const geometry_msgs::msg::PoseStamped & pose,
    const geometry_msgs::msg::Twist & velocity,
    nav2_core::GoalChecker * goal_checker) override
  {
    // Compute and return velocity command
  }

  void setSpeedLimit(const double & speed_limit, const bool & percentage) override {}
  void cleanup() override {}
  void activate() override {}
  void deactivate() override {}
};

}  // namespace

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(my_controller::MyController, nav2_core::Controller)
```

### Plugin XML Registration

```xml
<library path="my_controller">
  <class type="my_controller::MyController" base_class_type="nav2_core::Controller">
    <description>My custom controller</description>
  </class>
</library>
```
