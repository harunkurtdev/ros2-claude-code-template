---
name: ROS2 Transforms (TF2)
description: ROS2 TF2 and Transform management with Clean Architecture (Python & C++)
---

# ROS2 Transforms (TF2) Skill

This skill demonstrates how to use the ROS2 TF2 (Transform Library) while adhering to Clean Architecture principles. The Domain layer should be protected from direct TF2 dependencies by using Repository or Service patterns.

## Domain Layer

The domain layer should not depend on TF2 or `geometry_msgs`.

```python
# domain/entities/pose.py
from dataclasses import dataclass

@dataclass
class Pose:
    position: tuple  # (x, y, z)
    orientation: tuple  # (x, y, z, w)
    frame_id: str
    timestamp: float
```

```cpp
// domain/entities/pose.hpp
#pragma once
#include <string>

namespace domain::entities {

struct Point3D { double x, y, z; };
struct Quaternion { double x, y, z, w; };

struct Pose {
    Point3D position;
    Quaternion orientation;
    std::string frame_id;
    double timestamp;
};

} // namespace
```

## Infrastructure Layer

TF2 implementation resides here.

### TF2 Wrapper Service (Python)

See previous Python example.

### TF2 Wrapper Service (C++)

```cpp
// infrastructure/ros2/services/tf_service.hpp
#pragma once
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include "domain/interfaces/transform_service.hpp"

namespace infrastructure::ros2::services {

class TFService : public domain::interfaces::ITransformService {
public:
    explicit TFService(rclcpp::Node::SharedPtr node);

    std::optional<domain::entities::Pose> get_transform(
        const std::string& target_frame,
        const std::string& source_frame) override;

private:
    rclcpp::Node::SharedPtr node_;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
};

} // namespace

// infrastructure/ros2/services/tf_service.cpp
#include "infrastructure/ros2/services/tf_service.hpp"

namespace infrastructure::ros2::services {

TFService::TFService(rclcpp::Node::SharedPtr node) : node_(node) {
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
}

std::optional<domain::entities::Pose> TFService::get_transform(
    const std::string& target_frame,
    const std::string& source_frame) {

    try {
        geometry_msgs::msg::TransformStamped t = tf_buffer_->lookupTransform(
            target_frame, source_frame, tf2::TimePointZero);

        return domain::entities::Pose{
            {t.transform.translation.x, t.transform.translation.y, t.transform.translation.z},
            {t.transform.rotation.x, t.transform.rotation.y, t.transform.rotation.z, t.transform.rotation.w},
            t.header.frame_id,
            rclcpp::Time(t.header.stamp).seconds()
        };
    } catch (const tf2::TransformException & ex) {
        RCLCPP_WARN(node_->get_logger(), "Could not transform %s to %s: %s",
            source_frame.c_str(), target_frame.c_str(), ex.what());
        return std::nullopt;
    }
}

} // namespace
```

### Static Transform Broadcaster (C++)

```cpp
// infrastructure/ros2/helpers/tf_publisher.hpp
#include <tf2_ros/static_transform_broadcaster.h>

class TFPublisher {
public:
    TFPublisher(rclcpp::Node::SharedPtr node) : node_(node) {
        static_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(node);
    }

    void publish_static(const std::string& parent, const std::string& child,
                        const std::array<double, 3>& trans,
                        const std::array<double, 4>& rot) {
        geometry_msgs::msg::TransformStamped t;
        t.header.stamp = node_->get_clock()->now();
        t.header.frame_id = parent;
        t.child_frame_id = child;
        t.transform.translation.x = trans[0];
        // ...
        static_broadcaster_->sendTransform(t);
    }

private:
    rclcpp::Node::SharedPtr node_;
    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_broadcaster_;
};
```

## Best Practices

1. **Isolate Dependencies**: Domain Use Cases should never import `tf2_ros`.
2. **Exception Handling**: Always catch `tf2::TransformException`.
3. **Buffer Cache Time**: Default is usually 10s.
4. **Time Sync**: Use precise timestamps (not `Time(0)`) for data synchronization when possible.
