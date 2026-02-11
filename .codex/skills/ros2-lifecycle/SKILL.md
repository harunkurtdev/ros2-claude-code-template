---
name: ros2-lifecycle
description: Use when implementing managed lifecycle nodes, transitions, and lifecycle clients in ROS2 Humble.
---

# ROS2 Lifecycle Nodes Skill
## Authoritative Sources

- ROS2 Humble docs: `https://docs.ros.org/en/humble/index.html`
- Codex Skills docs: `https://developers.openai.com/codex/skills`
- ROS2 tutorials (nodes, launch, services/actions, tf2, testing, rosbag2): `https://docs.ros.org/en/humble/Tutorials.html`
- Codex skill conventions and examples: `https://github.com/openai/skills`

Use ROS2 Humble docs as source of truth for APIs/behavior and `openai/skills` for skill structure and trigger wording.

## Lifecycle States

```
              +---------+
              | UNKNOWN |
              +----+----+
                   |
              create()
                   v
           +------+------+
           | UNCONFIGURED|<----+
           +------+------+     |
                  |            |
            configure()   cleanup()
                  v            |
           +------+------+     |
           |  INACTIVE  |------+
           +------+------+
                  |
            activate()
                  v
           +------+------+
           |   ACTIVE   |
           +------+------+
                  |
           deactivate()
                  v
           +------+------+
           |  INACTIVE  |
           +-------------+
```

## Python Implementation

Implement Python lifecycle nodes following the Humble `rclpy.lifecycle` tutorial patterns, then keep transition/business logic in application/domain layers.

## C++ Implementation

### Lifecycle Node Template

```cpp
// infrastructure/ros2/nodes/managed_node.hpp
#pragma once

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <rclcpp_lifecycle/lifecycle_publisher.hpp>
#include <std_msgs/msg/string.hpp>

namespace infrastructure::ros2::nodes {

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

class ManagedNode : public rclcpp_lifecycle::LifecycleNode {
public:
    explicit ManagedNode(const std::string& node_name, bool intra_process_comms = false)
        : rclcpp_lifecycle::LifecycleNode(node_name,
            rclcpp::NodeOptions().use_intra_process_comms(intra_process_comms)) {}

    CallbackReturn on_configure(const rclcpp_lifecycle::State &) override {
        RCLCPP_INFO(get_logger(), "Configuring...");
        pub_ = this->create_publisher<std_msgs::msg::String>("topic", 10);
        return CallbackReturn::SUCCESS;
    }

    CallbackReturn on_activate(const rclcpp_lifecycle::State &) override {
        RCLCPP_INFO(get_logger(), "Activating...");
        pub_->on_activate();
        // Start timers, etc.
        return CallbackReturn::SUCCESS;
    }

    CallbackReturn on_deactivate(const rclcpp_lifecycle::State &) override {
        RCLCPP_INFO(get_logger(), "Deactivating...");
        pub_->on_deactivate();
        return CallbackReturn::SUCCESS;
    }

    CallbackReturn on_cleanup(const rclcpp_lifecycle::State &) override {
        RCLCPP_INFO(get_logger(), "Cleaning up...");
        pub_.reset();
        return CallbackReturn::SUCCESS;
    }

    CallbackReturn on_shutdown(const rclcpp_lifecycle::State &) override {
        RCLCPP_INFO(get_logger(), "Shutting down...");
        return CallbackReturn::SUCCESS;
    }

private:
    rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::String>::SharedPtr pub_;
};

} // namespace
```

### Lifecycle Client (C++)

Allows controlling the state of another node using the `lifeycle_msgs` services.

```cpp
// infrastructure/ros2/lifecycle/lifecycle_client.hpp
#include <rclcpp/rclcpp.hpp>
#include <lifecycle_msgs/srv/change_state.hpp>
#include <lifecycle_msgs/srv/get_state.hpp>

class LifecycleClient {
public:
    LifecycleClient(rclcpp::Node::SharedPtr node, const std::string& target_node) {
        client_ = node->create_client<lifecycle_msgs::srv::ChangeState>(
            target_node + "/change_state");
    }

    bool change_state(std::uint8_t transition_id) {
        auto request = std::make_shared<lifecycle_msgs::srv::ChangeState::Request>();
        request->transition.id = transition_id;

        // ... async call logic ...
        return true;
    }

private:
    rclcpp::Client<lifecycle_msgs::srv::ChangeState>::SharedPtr client_;
};
```

## Best Practices

- **Resource Management**: Allocate resources (memory, connections) in `on_configure` and release them in `on_cleanup`.
- **Activity**: Only publish data or perform main logic when in the **ACTIVE** state.
- **Error Handling**: Use `on_error` callback to handle failures during transitions.
- **Launch Integration**: Use `LifecycleNode` in launch files to automatically manage states.
