---
name: ROS2 Messaging Patterns
description: ROS2 messaging patterns and best practices with Clean Architecture (Python & C++)
---

# ROS2 Messaging Patterns Skill

This skill covers ROS2 messaging patterns adhering to Clean Architecture principles, protecting the domain layer from ROS2 dependencies.

## Publisher/Subscriber Patterns

### 1. Domain-Driven Publisher (Python)

See previous Python example.

### 2. Domain-Driven Publisher (C++)

```cpp
// infrastructure/ros2/publishers/state_publisher.hpp
#pragma once
#include <rclcpp/rclcpp.hpp>
#include "domain/entities/robot_state.hpp"
#include "robot_interfaces/msg/robot_state.hpp"

namespace infrastructure::ros2::publishers {

template<typename T>
class IStatePublisher {
public:
    virtual void publish(const T& state) = 0;
    virtual ~IStatePublisher() = default;
};

class RobotStatePublisher : public IStatePublisher<domain::entities::RobotState> {
public:
    RobotStatePublisher(rclcpp::Node::SharedPtr node, const std::string& topic, const rclcpp::QoS& qos)
        : node_(node) {
        publisher_ = node_->create_publisher<robot_interfaces::msg::RobotState>(topic, qos);
    }

    void publish(const domain::entities::RobotState& state) override {
        robot_interfaces::msg::RobotState msg;
        msg.mode = static_cast<int>(state.mode);
        // ... mapping ...
        publisher_->publish(msg);
    }

private:
    rclcpp::Node::SharedPtr node_;
    rclcpp::Publisher<robot_interfaces::msg::RobotState>::SharedPtr publisher_;
};

} // namespace
```

### 3. Generic Subscriber Handler (C++)

```cpp
// infrastructure/ros2/subscribers/base_subscriber.hpp
#pragma once
#include <rclcpp/rclcpp.hpp>
#include <functional>

namespace infrastructure::ros2::subscribers {

template<typename MsgT, typename EntityT>
class BaseSubscriber {
public:
    BaseSubscriber(rclcpp::Node::SharedPtr node,
                   const std::string& topic,
                   const rclcpp::QoS& qos,
                   std::function<void(const EntityT&)> callback)
        : callback_(callback) {

        subscription_ = node->create_subscription<MsgT>(
            topic, qos,
            [this](const typename MsgT::SharedPtr msg) {
                this->handle_message(msg);
            }
        );
    }

    virtual ~BaseSubscriber() = default;

protected:
    virtual EntityT convert_to_entity(const typename MsgT::SharedPtr msg) = 0;

private:
    void handle_message(const typename MsgT::SharedPtr msg) {
        auto entity = convert_to_entity(msg);
        callback_(entity);
    }

    rclcpp::Subscription<MsgT>::SharedPtr subscription_;
    std::function<void(const EntityT&)> callback_;
};

} // namespace
```

## Synchronization (C++)

Using `message_filters` in C++.

```cpp
// infrastructure/ros2/sync/time_sync.hpp
#pragma once
#include <rclcpp/rclcpp.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>

namespace infrastructure::ros2::sync {

class CameraSync {
public:
    CameraSync(rclcpp::Node::SharedPtr node) {
        image_sub_.subscribe(node, "image_raw");
        info_sub_.subscribe(node, "camera_info");

        sync_ = std::make_shared<Sync>(MySyncPolicy(10), image_sub_, info_sub_);
        sync_->registerCallback(&CameraSync::callback, this);
    }

private:
    void callback(const sensor_msgs::msg::Image::ConstSharedPtr& image,
                  const sensor_msgs::msg::CameraInfo::ConstSharedPtr& info) {
        // Handle synchronized messages
    }

    message_filters::Subscriber<sensor_msgs::msg::Image> image_sub_;
    message_filters::Subscriber<sensor_msgs::msg::CameraInfo> info_sub_;

    typedef message_filters::sync_policies::ApproximateTime<
        sensor_msgs::msg::Image, sensor_msgs::msg::CameraInfo> MySyncPolicy;
    typedef message_filters::Synchronizer<MySyncPolicy> Sync;
    std::shared_ptr<Sync> sync_;
};

} // namespace
```

## Event Bus using ROS2 (C++)

```cpp
// infrastructure/ros2/events/event_bus.hpp
#pragma once
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <nlohmann/json.hpp> // External dependency

class EventBus {
public:
    EventBus(rclcpp::Node::SharedPtr node) {
        pub_ = node->create_publisher<std_msgs::msg::String>("/events/all", 10);
        // ... subscriber setup ...
    }

    void emit(const std::string& type, const nlohmann::json& payload) {
        std_msgs::msg::String msg;
        nlohmann::json data;
        data["type"] = type;
        data["payload"] = payload;
        msg.data = data.dump();
        pub_->publish(msg);
    }

private:
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_;
};
```

## Best Practices

- **Isolation**: Keep domain entities independent of ROS2 message types.
- **Conversion**: Perform message-to-entity conversion in the infrastructure layer.
- **Thread Safety**: Use `std::mutex` for shared resources in C++ callbacks.
- **QoS**: Select appropriate QoS profiles (Reliable vs Best Effort).
