---
description: ROS2 Testing Standards
---

# ROS2 Testing Standards
## Authoritative Sources

- ROS2 Humble main docs: `https://docs.ros.org/en/humble/index.html`
- Package development guide: `https://docs.ros.org/en/humble/How-To-Guides/Developing-a-ROS-2-Package.html`
- Tutorials index: `https://docs.ros.org/en/humble/Tutorials.html`
- Skill authoring patterns: `https://github.com/openai/skills`

When this file conflicts with external references, prefer ROS2 Humble docs for ROS behavior and `openai/skills` for skill structure.

## Test Structure

```
package_name/test/
├── unit/           # Pure Python/C++ tests
├── integration/    # ROS2 node tests
└── e2e/           # Full system/Launch tests
```

## Unit Tests

### Python (pytest)

```python
import pytest
from domain.entities.robot import Robot

class TestRobot:
    def test_creation(self):
        robot = Robot(name="Bot", max_velocity=1.0)
        assert robot.name == "Bot"

    def test_invalid_velocity(self):
        with pytest.raises(ValueError):
            Robot(name="Bot", max_velocity=-1.0)
```

### C++ (GTest)

```cpp
#include <gtest/gtest.h>
#include "domain/entities/robot.hpp"

TEST(RobotTest, Creation) {
    domain::entities::Robot robot;
    robot.name = "Bot";
    EXPECT_EQ(robot.name, "Bot");
}
```

## Integration Tests

### Python (rclpy)

```python
import pytest
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

@pytest.fixture(scope='module')
def ros2_context():
    rclpy.init()
    yield
    rclpy.shutdown()

@pytest.fixture
def test_node(ros2_context):
    node = Node('test_node')
    yield node
    node.destroy_node()

def test_publisher(test_node):
    received = []
    sub = test_node.create_subscription(
        String, 'test', lambda m: received.append(m), 10
    )
    pub = test_node.create_publisher(String, 'test', 10)

    msg = String(data='Hello')
    pub.publish(msg)

    rclpy.spin_once(test_node, timeout_sec=1.0)
    assert len(received) >= 1
```

### C++ (GTest + rclcpp)

```cpp
#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>

class NodeIntegrationTest : public ::testing::Test {
protected:
    void SetUp() override {
        rclcpp::init(0, nullptr);
        node_ = std::make_shared<rclcpp::Node>("test_node");
    }

    void TearDown() override {
        rclcpp::shutdown();
    }

    rclcpp::Node::SharedPtr node_;
};

TEST_F(NodeIntegrationTest, SimpleSpin) {
    auto start = node_->get_clock()->now();
    rclcpp::spin_some(node_);
    // Assert logic...
}
```

## Launch Tests (E2E)

```python
import launch_testing
from launch import LaunchDescription
from launch_ros.actions import Node

@launch_testing.markers.keep_alive
def generate_test_description():
    return LaunchDescription([
        Node(package='pkg', executable='node'),
        launch_testing.actions.ReadyToTest()
    ])
```

## Test Commands

```bash
# All tests
colcon test

# Specific package
colcon test --packages-select my_pkg

# With coverage (Python)
pytest --cov=my_pkg --cov-report=html
```
