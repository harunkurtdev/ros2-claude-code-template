---
name: ros2-service-action
description: Use when implementing ROS2 Humble services and actions with clear domain/application/infrastructure boundaries.
---

# ROS2 Service & Action Skill
## Authoritative Sources

- ROS2 Humble docs: `https://docs.ros.org/en/humble/index.html`
- Codex Skills docs: `https://developers.openai.com/codex/skills`
- ROS2 tutorials (nodes, launch, services/actions, tf2, testing, rosbag2): `https://docs.ros.org/en/humble/Tutorials.html`
- Codex skill conventions and examples: `https://github.com/openai/skills`

Use ROS2 Humble docs as source of truth for APIs/behavior and `openai/skills` for skill structure and trigger wording.

This skill provides a guide for implementing ROS2 Services and Actions adhering to Clean Architecture principles.

## Service Implementation

### 1. Service Definition (.srv)

```
# srv/SetRobotMode.srv
# Request
string mode           # "idle", "active", "emergency"
bool force_change     # Force mode change

---

# Response
bool success
string message
string previous_mode
```

### 2. Infrastructure Layer - Service Server (Python)

Implement Python service/action adapters from Humble tutorials and convert request/response messages at infrastructure boundaries.

### 3. Infrastructure Layer - Service Server (C++)

```cpp
// infrastructure/ros2/services/robot_mode_service.hpp
#pragma once

#include <rclcpp/rclcpp.hpp>
#include "robot_interfaces/srv/set_robot_mode.hpp"
#include "domain/use_cases/set_robot_mode.hpp"

namespace infrastructure::ros2::services {

class RobotModeServiceNode : public rclcpp::Node {
public:
    explicit RobotModeServiceNode(
        std::shared_ptr<domain::use_cases::SetRobotModeUseCase> use_case,
        const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

private:
    void handle_set_mode(
        const std::shared_ptr<robot_interfaces::srv::SetRobotMode::Request> request,
        std::shared_ptr<robot_interfaces::srv::SetRobotMode::Response> response);

    std::shared_ptr<domain::use_cases::SetRobotModeUseCase> use_case_;
    rclcpp::Service<robot_interfaces::srv::SetRobotMode>::SharedPtr service_;
};

} // namespace

// infrastructure/ros2/services/robot_mode_service.cpp
#include "infrastructure/ros2/services/robot_mode_service.hpp"

namespace infrastructure::ros2::services {

RobotModeServiceNode::RobotModeServiceNode(
    std::shared_ptr<domain::use_cases::SetRobotModeUseCase> use_case,
    const rclcpp::NodeOptions& options)
    : Node("robot_mode_service", options), use_case_(use_case) {

    using namespace std::placeholders;
    service_ = this->create_service<robot_interfaces::srv::SetRobotMode>(
        "/robot/set_mode",
        std::bind(&RobotModeServiceNode::handle_set_mode, this, _1, _2)
    );
}

void RobotModeServiceNode::handle_set_mode(
    const std::shared_ptr<robot_interfaces::srv::SetRobotMode::Request> request,
    std::shared_ptr<robot_interfaces::srv::SetRobotMode::Response> response) {

    // Conversion from message to domain object would go here
    // ...

    // Execute use case
    // auto result = use_case_->execute(...);

    // Map result to response
    // response->success = result.success;
}

} // namespace
```

### 4. Service Client (C++)

```cpp
// infrastructure/ros2/clients/robot_mode_client.hpp
#pragma once
#include <rclcpp/rclcpp.hpp>
#include "robot_interfaces/srv/set_robot_mode.hpp"

namespace infrastructure::ros2::clients {

class RobotModeClient {
public:
    explicit RobotModeClient(rclcpp::Node::SharedPtr node);

    std::future<std::shared_ptr<robot_interfaces::srv::SetRobotMode::Response>>
    set_mode_async(const std::string& mode, bool force = false);

private:
    rclcpp::Node::SharedPtr node_;
    rclcpp::Client<robot_interfaces::srv::SetRobotMode>::SharedPtr client_;
};

} // namespace
```

## Action Implementation

### 1. Action Definition (.action)

```
# action/NavigateToPoint.action
# Goal
geometry_msgs/Point target_point
float32 max_velocity
bool avoid_obstacles

---

# Result
bool success
string message
float32 total_distance
float32 total_time

---

# Feedback
geometry_msgs/Point current_position
float32 distance_remaining
float32 estimated_time
string status
```

### 2. Action Server (C++)

```cpp
// infrastructure/ros2/actions/navigation_action_server.hpp
#pragma once

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include "robot_interfaces/action/navigate_to_point.hpp"
#include "domain/use_cases/navigate_to_point.hpp"

namespace infrastructure::ros2::actions {

class NavigationActionServer : public rclcpp::Node {
public:
    using NavigateToPoint = robot_interfaces::action::NavigateToPoint;
    using GoalHandle = rclcpp_action::ServerGoalHandle<NavigateToPoint>;

    explicit NavigationActionServer(
        std::shared_ptr<domain::use_cases::NavigateToPointUseCase> use_case,
        const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

private:
    rclcpp_action::GoalResponse handle_goal(
        const rclcpp_action::GoalUUID& uuid,
        std::shared_ptr<const NavigateToPoint::Goal> goal);

    rclcpp_action::CancelResponse handle_cancel(
        const std::shared_ptr<GoalHandle> goal_handle);

    void handle_accepted(const std::shared_ptr<GoalHandle> goal_handle);
    void execute(const std::shared_ptr<GoalHandle> goal_handle);

    std::shared_ptr<domain::use_cases::NavigateToPointUseCase> use_case_;
    rclcpp_action::Server<NavigateToPoint>::SharedPtr action_server_;
};

} // namespace
```

### 3. Action Client (C++)

```cpp
// infrastructure/ros2/clients/navigation_client.hpp
#pragma once

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include "robot_interfaces/action/navigate_to_point.hpp"

namespace infrastructure::ros2::clients {

class NavigationClient {
public:
    using NavigateToPoint = robot_interfaces::action::NavigateToPoint;
    using GoalHandle = rclcpp_action::ClientGoalHandle<NavigateToPoint>;

    explicit NavigationClient(rclcpp::Node::SharedPtr node);

    std::shared_future<std::shared_ptr<GoalHandle>> navigate_to(
        double x, double y, double z,
        std::function<void(const NavigateToPoint::Feedback&)> feedback_cb);

private:
    rclcpp::Node::SharedPtr node_;
    rclcpp_action::Client<NavigateToPoint>::SharedPtr client_;
};

} // namespace
```
