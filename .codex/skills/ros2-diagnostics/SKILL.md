---
name: ros2-diagnostics
description: Use when adding diagnostic_updater health checks, status reporting, and frequency monitoring in ROS2 Humble.
---

# ROS2 Diagnostics Skill
## Authoritative Sources

- ROS2 Humble docs: `https://docs.ros.org/en/humble/index.html`
- Codex Skills docs: `https://developers.openai.com/codex/skills`
- ROS2 tutorials (nodes, launch, services/actions, tf2, testing, rosbag2): `https://docs.ros.org/en/humble/Tutorials.html`
- Codex skill conventions and examples: `https://github.com/openai/skills`

Use ROS2 Humble docs as source of truth for APIs/behavior and `openai/skills` for skill structure and trigger wording.

This skill demonstrates how to integrate standard ROS2 diagnostics tools for health monitoring within a Clean Architecture.

## Domain Layer

```python
# domain/entities/health.py
class HealthLevel(Enum):
    OK = 0
    WARN = 1
    ERROR = 2
    STALE = 3

@dataclass
class ComponentHealth:
    name: str
    level: HealthLevel
    message: str
    values: dict
```

## Infrastructure Layer (Python)

Use `diagnostic_updater` patterns from ROS2 Humble documentation and keep health semantics mapped through domain entities first.

## Infrastructure Layer (C++)

Using `diagnostic_updater` package in C++.

```cpp
// infrastructure/ros2/diagnostics/diagnostics_manager.hpp
#pragma once
#include <rclcpp/rclcpp.hpp>
#include <diagnostic_updater/diagnostic_updater.hpp>
#include "domain/interfaces/diagnostics_port.hpp"

namespace infrastructure::ros2::diagnostics {

class DiagnosticsManager {
public:
    explicit DiagnosticsManager(rclcpp::Node::SharedPtr node)
        : node_(node), updater_(node) {
        updater_.setHardwareID(node->get_name());
    }

    void register_monitor(const std::string& name,
                          std::function<void(diagnostic_updater::DiagnosticStatusWrapper&)> callback) {
        updater_.add(name, callback);
    }

    // Example callback wrapper for domain entities
    void check_component(diagnostic_updater::DiagnosticStatusWrapper& stat) {
        // Retrieve health from domain service
        // auto health = domain_service_->get_health();

        // stat.summary(health.level, health.message);
        // stat.add("temp", health.value);
    }

private:
    rclcpp::Node::SharedPtr node_;
    diagnostic_updater::Updater updater_;
};

} // namespace
```

### Frequency Status Example (C++)

```cpp
// infrastructure/ros2/diagnostics/frequency_monitor.hpp
#include <diagnostic_updater/publisher.hpp> // For TopicDiagnostic

class FrequencyMonitor {
public:
    FrequencyMonitor(diagnostic_updater::Updater& updater,
                     const std::string& topic_name,
                     double min_freq, double max_freq) {

        diagnostic_updater::FrequencyStatusParam freq_param(&min_freq, &max_freq, 0.1, 10);

        monitor_ = std::make_unique<diagnostic_updater::HeaderlessTopicDiagnostic>(
            topic_name, updater, freq_param);
    }

    void tick() {
        monitor_->tick();
    }

private:
    std::unique_ptr<diagnostic_updater::HeaderlessTopicDiagnostic> monitor_;
};
```

## Use Case Integration

```cpp
// application/services/motor_controller.cpp
void MotorController::check_temp(diagnostic_updater::DiagnosticStatusWrapper& stat) {
    double temp = read_temp();
    if (temp > 80.0) {
        stat.summary(diagnostic_msgs::msg::DiagnosticStatus::ERROR, "Overheating");
    } else {
        stat.summary(diagnostic_msgs::msg::DiagnosticStatus::OK, "Normal");
    }
    stat.add("temp", temp);
}
```

## Best Practices

1. **Hardware ID**: Always set a unique Hardware ID in the updater.
2. **Frequency Monitoring**: Use `TopicDiagnostic` to monitor publication rates.
3. **Meaningful Messages**: Provide descriptive error messages.
4. **Standard Levels**: Stick to OK, WARN, ERROR, STALE semantics.
