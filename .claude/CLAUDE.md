# ROS2 Clean Architecture Project

This project is set up with a comprehensive set of **Claude Skills** designed to facilitate ROS2 development following **Clean Architecture** principles.

## Available Skills

The following skills are available in `.claude/skills` and can be used to guide development:

| Skill Name              | Description                               | Key Components                                                                        |
| ----------------------- | ----------------------------------------- | ------------------------------------------------------------------------------------- |
| **ros2_node_creation**  | Create Clean Architecture compliant Nodes | `BaseNode` template, Dependency Injection, QoS profiles (Python & C++)                |
| **ros2_launch_config**  | Modular Launch files                      | Composition, `IncludeLaunchDescription`, Parameter management, C++ executable support |
| **ros2_service_action** | Services and Actions                      | Server/Client wrappers, Domain Use Case integration (Python & C++)                    |
| **ros2_messaging**      | Pub/Sub Patterns                          | Domain-driven publishers, Generic subscribers, Thread-safe buffers, Synchronization   |
| **ros2_testing**        | Testing Strategy                          | Unit (Domain), Integration (Node), E2E (Launch), GTest/GMock support                  |
| **ros2_lifecycle**      | Managed Nodes                             | Lifecycle Node templates, State transition management, Lifecycle Clients              |
| **ros2_transforms**     | TF2 Management                            | TF2 Wrappers avoiding domain dependency on `geometry_msgs`                            |
| **ros2_diagnostics**    | Health Monitoring                         | `diagnostic_updater` integration, Health entities, Frequency monitoring               |
| **ros2_bag**            | Data Recording                            | Programmatic bag recording and replay utilities (rosbag2)                             |

## Project Structure

The project follows a strict separation of concerns:

- **src/domain/**: Pure business logic, entities, and use cases. No ROS2 dependencies.
- **src/application/**: Application services and interfaces. Orchestrates logic.
- **src/infrastructure/**: ROS2 specific implementations (Nodes, Publishers, Subscribers).

## Getting Started

To use a skill, reference the skill file (e.g., `.claude/skills/ros2_node_creation/SKILL.md`) for templates and best practices.

## Common Commands

### Build & Test

- **Build**: `colcon build --symlink-install`
- **Build Package**: `colcon build --packages-select <package_name>`
- **Test**: `colcon test`
- **Source**: `source install/setup.bash`

### Introspection

- **List Nodes**: `ros2 node list`
- **List Topics**: `ros2 topic list`
- **Echo Topic**: `ros2 topic echo <topic_name>`
- **List Services**: `ros2 service list`
- **Call Service**: `ros2 service call <service_name> <service_type> <args>`
- **List Parameters**: `ros2 param list`

### Debugging

- **Doctor**: `ros2 doctor`
- **Graph**: `rqt_graph`
- **Monitor**: `rqt_topic`
