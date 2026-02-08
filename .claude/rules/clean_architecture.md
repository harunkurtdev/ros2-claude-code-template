---
description: Clean Architecture Principles for ROS2 Projects
---

# Clean Architecture Rules

This rule file defines how to apply Clean Architecture principles in ROS2 projects.

## Layer Dependency Rules

```
┌─────────────────────────────────────────────────────────┐
│                  PRESENTATION LAYER                      │
│          (CLI, GUI, API - User Interfaces)              │
└─────────────────────────┬───────────────────────────────┘
                          │ depends on
┌─────────────────────────▼───────────────────────────────┐
│                  APPLICATION LAYER                       │
│            (Use Cases, Application Services)             │
└─────────────────────────┬───────────────────────────────┘
                          │ depends on
┌─────────────────────────▼───────────────────────────────┐
│                    DOMAIN LAYER                          │
│     (Entities, Value Objects, Domain Interfaces)         │
└─────────────────────────▲───────────────────────────────┘
                          │ implements
┌─────────────────────────┴───────────────────────────────┐
│                 INFRASTRUCTURE LAYER                     │
│        (ROS2 Adapters, Hardware, Persistence)            │
└─────────────────────────────────────────────────────────┘
```

**CRITICAL RULES:**

1. The Domain layer must NOT depend on any outer layer.
2. The Application layer can only depend on the Domain layer.
3. The Infrastructure layer implements Domain interfaces.
4. The Presentation layer can only depend on the Application layer.

## Domain Layer

### Entities

#### Python Example

```python
# ✅ CORRECT: Pure Python, no external dependencies
from dataclasses import dataclass
from abc import ABC, abstractmethod
from uuid import UUID, uuid4

@dataclass
class Entity(ABC):
    """Base entity with unique identifier."""
    id: UUID = None

    def __post_init__(self):
        if self.id is None:
            self.id = uuid4()

    def __eq__(self, other):
        if not isinstance(other, Entity):
            return False
        return self.id == other.id

    def __hash__(self):
        return hash(self.id)

@dataclass
class Robot(Entity):
    """Robot domain entity."""
    name: str
    max_velocity: float
    max_acceleration: float
```

#### C++ Example

```cpp
// ✅ CORRECT: Pure C++, no ROS2 dependencies
#pragma once
#include <string>
#include <memory>

namespace domain::entities {

struct Robot {
    std::string id;
    std::string name;
    double max_velocity;
    double max_acceleration;

    bool operator==(const Robot& other) const {
        return id == other.id;
    }
};

} // namespace
```

### Value Objects

#### Python Example

```python
# ✅ CORRECT: Immutable, no identity
from dataclasses import dataclass

@dataclass(frozen=True)
class Position:
    """Position value object."""
    x: float
    y: float
    z: float = 0.0

    def distance_to(self, other: 'Position') -> float:
        return ((self.x - other.x)**2 +
                (self.y - other.y)**2 +
                (self.z - other.z)**2) ** 0.5
```

#### C++ Example

```cpp
// ✅ CORRECT: Immutable struct (const members or accessors)
#pragma once
#include <cmath>

namespace domain::values {

struct Position {
    const double x;
    const double y;
    const double z;

    Position(double x, double y, double z = 0.0) : x(x), y(y), z(z) {}

    double distance_to(const Position& other) const {
        return std::sqrt(std::pow(x - other.x, 2) +
                         std::pow(y - other.y, 2) +
                         std::pow(z - other.z, 2));
    }
};

} // namespace
```

### Domain Interfaces (Ports)

#### Python Example

```python
# ✅ CORRECT: Abstract interface in domain layer
from abc import ABC, abstractmethod
from typing import Optional, List

class RobotRepository(ABC):
    """Port for robot data access."""

    @abstractmethod
    def get_by_id(self, robot_id: UUID) -> Optional[Robot]:
        pass

    @abstractmethod
    def save(self, robot: Robot) -> Robot:
        pass
```

#### C++ Example

```cpp
// ✅ CORRECT: Abstract base class (interface)
#pragma once
#include "domain/entities/robot.hpp"
#include <optional>
#include <vector>

namespace domain::interfaces {

class IRobotRepository {
public:
    virtual ~IRobotRepository() = default;
    virtual std::optional<entities::Robot> get_by_id(const std::string& id) = 0;
    virtual void save(const entities::Robot& robot) = 0;
};

} // namespace
```

## Application Layer

### Use Cases

#### Python Example

```python
# ✅ CORRECT: Business logic orchestration
class MoveRobotUseCase:
    def __init__(self, motion_controller: MotionController):
        self._motion_controller = motion_controller

    def execute(self, target: Position) -> bool:
        return self._motion_controller.move_to(target)
```

#### C++ Example

```cpp
// ✅ CORRECT: Business logic orchestration
#pragma once
#include "domain/interfaces/motion_controller.hpp"

namespace application::use_cases {

class MoveRobotUseCase {
public:
    explicit MoveRobotUseCase(std::shared_ptr<domain::interfaces::IMotionController> controller)
        : controller_(controller) {}

    bool execute(const domain::values::Position& target) {
        return controller_->move_to(target);
    }

private:
    std::shared_ptr<domain::interfaces::IMotionController> controller_;
};

} // namespace
```

## Infrastructure Layer

### ROS2 Adapters

#### Python Example

```python
# ✅ CORRECT: Implements domain interface
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class ROS2MotionController(MotionController):
    """ROS2 adapter implementing MotionController port."""
    # ... implementation ...
```

#### C++ Example

```cpp
// ✅ CORRECT: Implements domain interface
#include "domain/interfaces/motion_controller.hpp"
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>

namespace infrastructure::ros2::adapters {

class ROS2MotionController : public domain::interfaces::IMotionController {
public:
    explicit ROS2MotionController(rclcpp::Node::SharedPtr node) : node_(node) {
        pub_ = node_->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
    }

    bool move_to(const domain::values::Position& target) override {
        // Implementation...
        return true;
    }

private:
    rclcpp::Node::SharedPtr node_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_;
};

} // namespace
```

## Anti-Patterns to Avoid

### Python

```python
# ❌ INCORRECT: Domain importing ROS2
from rclpy.node import Node  # Domain should NOT import ROS2!

class Robot(Entity):
    def __init__(self, node: Node):  # WRONG!
        self.node = node

# ❌ INCORRECT: Business logic in infrastructure
class ROS2MotionController:
    def move_to(self, position):
        # WRONG: Business rules should be in domain/application
        if self.battery_level < 20:
            return False
```

### C++

```cpp
// ❌ INCORRECT: Domain header including ROS2
#include <rclcpp/rclcpp.hpp> // WRONG in Domain!

namespace domain::entities {
    class Robot {
        rclcpp::Node::SharedPtr node_; // WRONG!
    };
}
```
