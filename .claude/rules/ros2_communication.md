---
description: ROS2 Communication Standards (Topics, QoS, Messages)
---

# ROS2 Communication Standards

This rule file defines communication standards in ROS2.

## Topic Naming Conventions

```
# Format: /<namespace>/<category>/<specific>
# Examples:

# Sensor topics
/robot/sensors/lidar/scan
/robot/sensors/camera/image_raw
/robot/sensors/imu/data

# Control topics
/robot/control/cmd_vel
/robot/control/joint_commands

# State topics
/robot/state/odometry
/robot/state/battery
/robot/state/joint_states

# Navigation topics
/robot/navigation/goal
/robot/navigation/path
/robot/navigation/costmap

# Perception topics
/robot/perception/detections
/robot/perception/markers
```

### Naming Rules

| Rule                           | Example                        |
| ------------------------------ | ------------------------------ |
| Use lowercase                  | `/robot/cmd_vel`               |
| Use underscores for multi-word | `/joint_states`                |
| Avoid abbreviations            | `/camera/image` not `/cam/img` |
| Use namespaces                 | `/robot1/cmd_vel`              |
| Be descriptive                 | `/laser_scan` not `/ls`        |

## QoS Profiles

### Python Example

```python
from rclpy.qos import (
    QoSProfile,
    QoSReliabilityPolicy,
    QoSDurabilityPolicy,
    QoSHistoryPolicy,
    QoSLivelinessPolicy
)

# Sensor Data QoS - High frequency, best effort
SENSOR_QOS = QoSProfile(
    reliability=QoSReliabilityPolicy.BEST_EFFORT,
    durability=QoSDurabilityPolicy.VOLATILE,
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=5
)

# Control Commands QoS - Reliable delivery
CONTROL_QOS = QoSProfile(
    reliability=QoSReliabilityPolicy.RELIABLE,
    durability=QoSDurabilityPolicy.VOLATILE,
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=10
)

# State/Configuration QoS - Latched topics
STATE_QOS = QoSProfile(
    reliability=QoSReliabilityPolicy.RELIABLE,
    durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=1
)
```

### C++ Example

```cpp
#include <rclcpp/qos.hpp>

// Sensor Data QoS
static const rclcpp::QoS SENSOR_QOS = rclcpp::QoS(5)
    .reliability(rmw_qos_reliability_policy_t::RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT)
    .durability(rmw_qos_durability_policy_t::RMW_QOS_POLICY_DURABILITY_VOLATILE);

// Control Commands QoS
static const rclcpp::QoS CONTROL_QOS = rclcpp::QoS(10)
    .reliability(rmw_qos_reliability_policy_t::RMW_QOS_POLICY_RELIABILITY_RELIABLE)
    .durability(rmw_qos_durability_policy_t::RMW_QOS_POLICY_DURABILITY_VOLATILE);

// State/Configuration QoS
static const rclcpp::QoS STATE_QOS = rclcpp::QoS(1)
    .reliability(rmw_qos_reliability_policy_t::RMW_QOS_POLICY_RELIABILITY_RELIABLE)
    .durability(rmw_qos_durability_policy_t::RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL);
```

### QoS Selection Guide

| Data Type          | Reliability | Durability      | Depth | Example           |
| ------------------ | ----------- | --------------- | ----- | ----------------- |
| Sensor (high freq) | BEST_EFFORT | VOLATILE        | 1-5   | LiDAR, Camera     |
| Commands           | RELIABLE    | VOLATILE        | 10    | cmd_vel           |
| State/Config       | RELIABLE    | TRANSIENT_LOCAL | 1     | robot_description |
| Transforms         | RELIABLE    | VOLATILE        | 100   | tf                |
| Map                | RELIABLE    | TRANSIENT_LOCAL | 1     | occupancy_grid    |

## Custom Message Definition

```
# File: msg/RobotState.msg
# Header for timestamp and frame
std_msgs/Header header

# Position
geometry_msgs/Point position

# Orientation (quaternion)
geometry_msgs/Quaternion orientation

# Velocity
geometry_msgs/Twist velocity

# Status enum
uint8 STATUS_IDLE = 0
uint8 STATUS_MOVING = 1
uint8 STATUS_ERROR = 2
uint8 status

# Battery percentage (0-100)
float32 battery_level

# Array of sensor readings
float64[] sensor_readings
```

## Custom Service Definition

```
# File: srv/SetRobotMode.srv
# Request
---
# Mode to set
uint8 MODE_MANUAL = 0
uint8 MODE_AUTO = 1
uint8 MODE_EMERGENCY = 2
uint8 mode

# Optional parameters
string[] parameters
---
# Response
bool success
string message
float64 transition_time
```

## Custom Action Definition

```
# File: action/NavigateToGoal.action
# Goal
---
geometry_msgs/PoseStamped target_pose
float32 max_velocity
bool allow_replanning
---
# Result
bool success
string message
float64 total_time
float64 total_distance
---
# Feedback
geometry_msgs/PoseStamped current_pose
float32 distance_remaining
float32 estimated_time_remaining
uint8 recovery_count
```

## Message Package Structure

```
robot_interfaces/
├── CMakeLists.txt
├── package.xml
├── msg/
│   ├── RobotState.msg
├── srv/
│   ├── SetRobotMode.srv
└── action/
    └── NavigateToGoal.action
```

### CMakeLists.txt for Message Package

```cmake
cmake_minimum_required(VERSION 3.8)
project(robot_interfaces)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(action_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/RobotState.msg"
  "srv/SetRobotMode.srv"
  "action/NavigateToGoal.action"
  DEPENDENCIES std_msgs geometry_msgs action_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
```

### package.xml for Message Package

```xml
<?xml version="1.0"?>
<package format="3">
  <name>robot_interfaces</name>
  <version>0.1.0</version>
  <description>Robot interface definitions</description>
  <maintainer email="dev@example.com">Developer</maintainer>
  <license>Apache-2.0</license>

  <buildtool_depend>ament_cmake</buildtool_depend>
  <buildtool_depend>rosidl_default_generators</buildtool_depend>

  <depend>std_msgs</depend>
  <depend>geometry_msgs</depend>
  <depend>action_msgs</depend>

  <exec_depend>rosidl_default_runtime</exec_depend>

  <member_of_group>rosidl_interface_packages</member_of_group>

  <export>
    <build_type>ament_cmake</build_type>
  </export>
</package>
```

## TF2 Transform Best Practices

### Python Example

```python
import tf2_ros
from geometry_msgs.msg import TransformStamped

class TransformPublisher:
    """Best practices for TF2 transforms."""

    def __init__(self, node):
        self._node = node

        # Static transform broadcaster (for fixed transforms)
        self._static_broadcaster = tf2_ros.StaticTransformBroadcaster(node)

        # Dynamic transform broadcaster
        self._dynamic_broadcaster = tf2_ros.TransformBroadcaster(node)
```

### C++ Example

```cpp
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>

class TransformPublisher {
public:
    TransformPublisher(rclcpp::Node::SharedPtr node)
        : node_(node) {
        static_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(node);
        dynamic_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(node);
    }
    // ...
private:
    rclcpp::Node::SharedPtr node_;
    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_broadcaster_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> dynamic_broadcaster_;
};
```
