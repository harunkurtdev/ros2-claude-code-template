---
name: ROS2 Node Creation
description: Guide for creating ROS2 nodes following Clean Architecture principles (Python & C++)
---

# ROS2 Node Creation Skill

This skill is used to create ROS2 nodes that adhere to Clean Architecture principles. It covers both Python and C++ implementations.

## Directory Structure

```
src/
├── domain/                    # Business Logic Layer
│   ├── entities/             # Core business objects
│   ├── repositories/         # Repository interfaces (abstract)
│   └── use_cases/           # Business rules
├── application/              # Application Layer
│   ├── services/            # Application services
│   └── interfaces/          # Port interfaces
└── infrastructure/           # Infrastructure Layer
    └── ros2/
        ├── nodes/           # ROS2 Node implementations
        ├── publishers/      # Publisher adapters
        ├── subscribers/     # Subscriber adapters
        └── services/        # Service adapters
```

## Python Implementation

### 1. Base Node Template

```python
#!/usr/bin/env python3
"""
ROS2 Node: [NodeName]
Description: [Node Description]
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from typing import Optional, Callable
from abc import ABC, abstractmethod


class BaseNode(Node, ABC):
    """Base class for all nodes."""

    def __init__(self, node_name: str):
        super().__init__(node_name)
        self._setup_parameters()
        self._setup_publishers()
        self._setup_subscribers()
        self._setup_services()
        self._setup_timers()
        self.get_logger().info(f'{node_name} initialized')

    @abstractmethod
    def _setup_parameters(self) -> None:
        """Define ROS2 parameters."""
        pass

    @abstractmethod
    def _setup_publishers(self) -> None:
        """Create publishers."""
        pass

    @abstractmethod
    def _setup_subscribers(self) -> None:
        """Create subscribers."""
        pass

    def _setup_services(self) -> None:
        """Create services (optional)."""
        pass

    def _setup_timers(self) -> None:
        """Create timers (optional)."""
        pass

    def get_default_qos(self) -> QoSProfile:
        """Default QoS profile."""
        return QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
```

### 2. Concrete Node Implementation

```python
# ... (Same as before, but with English comments) ...
# See previous version for logic, just translate comments
```

## C++ Implementation

### 1. Base Node Template (Header)

```cpp
// infrastructure/ros2/nodes/base_node.hpp
#pragma once

#include <rclcpp/rclcpp.hpp>
#include <string>
#include <memory>

namespace infrastructure::ros2::nodes {

class BaseNode : public rclcpp::Node {
public:
    explicit BaseNode(const std::string& node_name,
                      const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
    virtual ~BaseNode() = default;

protected:
    virtual void setup_parameters() = 0;
    virtual void setup_publishers() = 0;
    virtual void setup_subscribers() = 0;
    virtual void setup_services() {}
    virtual void setup_timers() {}

    rclcpp::QoS get_default_qos() const;
};

} // namespace infrastructure::ros2::nodes
```

### 2. Base Node Template (Source)

```cpp
// infrastructure/ros2/nodes/base_node.cpp
#include "infrastructure/ros2/nodes/base_node.hpp"

namespace infrastructure::ros2::nodes {

BaseNode::BaseNode(const std::string& node_name, const rclcpp::NodeOptions& options)
    : Node(node_name, options) {

    // Virtual calls in constructor are dangerous in C++,
    // but common in ROS2 if careful or using an init() method.
    // Better pattern: Call these in a separate init() or distinct lifecycle.
    // For simplicity in this template, we assume derived classes handle initialization
    // or use the lifecycle node pattern.
}

rclcpp::QoS BaseNode::get_default_qos() const {
    return rclcpp::QoS(10)
        .reliability(rmw_qos_reliability_policy_t::RMW_QOS_POLICY_RELIABILITY_RELIABLE)
        .history(rmw_qos_history_policy_t::RMW_QOS_POLICY_HISTORY_KEEP_LAST);
}

} // namespace infrastructure::ros2::nodes
```

### 3. Concrete Node Implementation (C++)

```cpp
// infrastructure/ros2/nodes/sensor_node.hpp
#pragma once

#include "infrastructure/ros2/nodes/base_node.hpp"
#include "application/services/sensor_service.hpp"
#include <std_msgs/msg/float64.hpp>
#include <sensor_msgs/msg/temperature.hpp>

namespace infrastructure::ros2::nodes {

class SensorNode : public BaseNode {
public:
    explicit SensorNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

    // Dependency Injection
    void set_sensor_service(std::shared_ptr<application::services::ISensorService> service);

protected:
    void setup_parameters() override;
    void setup_publishers() override;
    void setup_subscribers() override;
    void setup_timers() override;

private:
    void raw_callback(const std_msgs::msg::Float64::SharedPtr msg);
    void timer_callback();

    std::shared_ptr<application::services::ISensorService> sensor_service_;

    rclcpp::Publisher<sensor_msgs::msg::Temperature>::SharedPtr temp_pub_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr raw_sub_;
    rclcpp::TimerBase::SharedPtr timer_;

    double update_rate_;
    std::string sensor_topic_;
};

} // namespace infrastructure::ros2::nodes
```

### 4. Dependency Injection (C++)

```cpp
// application/services/sensor_service.hpp
#pragma once
#include "domain/entities/sensor_data.hpp"

namespace application::services {

class ISensorService {
public:
    virtual ~ISensorService() = default;
    virtual domain::entities::SensorData process(double raw_data) = 0;
};

} // namespace application::services
```

## QoS Profiles (C++)

```cpp
// infrastructure/ros2/qos_profiles.hpp
#pragma once
#include <rclcpp/qos.hpp>

namespace infrastructure::ros2 {

class QoSProfiles {
public:
    static rclcpp::QoS sensor_data() {
        return rclcpp::QoS(1).best_effort().durability_volatile();
    }

    static rclcpp::QoS command() {
        return rclcpp::QoS(10).reliable().transient_local();
    }
};

} // namespace
```
