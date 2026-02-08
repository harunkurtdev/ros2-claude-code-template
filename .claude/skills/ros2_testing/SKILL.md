---
name: ROS2 Testing
description: ROS2 test strategies and patterns with Clean Architecture (Python & C++)
---

# ROS2 Testing Skill

This skill provides test strategies for ROS2 applications adhering to Clean Architecture principles, covering Unit, Integration, and E2E tests in both Python and C++.

## Test Pyramid

```
        /\
       /  \  E2E Tests (Launch Tests / System Tests)
      /----\
     /      \  Integration Tests (Node / Component Tests)
    /--------\
   /          \  Unit Tests (Domain / Application Logic)
  /--------------\
```

## Directory Structure

```
tests/
├── unit/
│   ├── domain/
│   └── application/
├── integration/
│   └── ros2/
└── e2e/
    └── launch_tests/
```

## Python Testing

### 1. Unit Tests (Domain Layer)

See the previous version for Python Unit Test examples. They remain valid as domain logic is pure Python.

### 2. Integration Tests (Node Test)

```python
# tests/integration/ros2/nodes/test_sensor_node.py
import pytest
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64

@pytest.fixture(scope='module')
def ros_context():
    rclpy.init()
    yield
    rclpy.shutdown()

@pytest.fixture
def test_node(ros_context):
    node = Node('test_helper')
    yield node
    node.destroy_node()

def test_sensor_integration(test_node):
    # Verify node behavior by subscribing/publishing
    pass
```

## C++ Testing (GTest)

### 1. Unit Tests (Domain Layer)

```cpp
// tests/unit/domain/use_cases/test_robot_controller.cpp
#include <gtest/gtest.h>
#include <gmock/gmock.h>
#include "domain/use_cases/robot_controller.hpp"
#include "domain/entities/robot_state.hpp"

using namespace domain;
using ::testing::Return;

class MockRobotRepository : public repositories::IRobotRepository {
public:
    MOCK_METHOD(entities::RobotState, get_state, (), (override));
    MOCK_METHOD(void, set_mode, (entities::RobotMode), (override));
};

TEST(RobotControllerTest, StartFromIdle) {
    auto mock_repo = std::make_shared<MockRobotRepository>();
    use_cases::RobotControllerUseCase use_case(mock_repo);

    EXPECT_CALL(*mock_repo, get_state())
        .WillOnce(Return(entities::RobotState{entities::RobotMode::IDLE}));

    EXPECT_CALL(*mock_repo, set_mode(entities::RobotMode::ACTIVE));

    auto result = use_case.start();
    EXPECT_TRUE(result.success);
}
```

### 2. Integration Tests (Node/Component)

```cpp
// tests/integration/ros2/test_sensor_node.cpp
#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include "infrastructure/ros2/nodes/sensor_node.hpp"

class SensorNodeTest : public ::testing::Test {
protected:
    void SetUp() override {
        rclcpp::init(0, nullptr);
        node_ = std::make_shared<infrastructure::ros2::nodes::SensorNode>();
    }

    void TearDown() override {
        rclcpp::shutdown();
    }

    std::shared_ptr<infrastructure::ros2::nodes::SensorNode> node_;
};

TEST_F(SensorNodeTest, Initialization) {
    EXPECT_STREQ(node_->get_name(), "sensor_node");
}
```

### 3. Launch Tests (Python with GTest)

You can run GTest executables from launch files to perform system-level tests.

```python
# tests/e2e/launch_tests/system_test.launch.py
from launch import LaunchDescription
from launch_ros.actions import Node
from launch_testing.actions import ReadyToTest

def generate_launch_description():
    # Launch system under test
    app_node = Node(package='my_robot', executable='main_node')

    # Launch GTest runner
    test_runner = Node(
        package='my_robot',
        executable='system_integration_test',
        output='screen'
    )

    return LaunchDescription([
        app_node,
        test_runner,
        ReadyToTest()
    ])
```

## Best Practices

- **Mocking**: Use `unittest.mock` for Python and `gmock` for C++.
- **Separation**: Keep domain logic usage in unit tests strictly separate from ROS2 dependencies.
- **Fixtures**: Use `pytest.fixture` and GTest `SetUp/TearDown` to manage ROS2 context (`rclpy.init/shutdown`).
- **Async Testing**: For C++ integration tests, use `rclcpp::spin_some` or `wait_for_future` to handle async operations.
