# ROS2 Clean Architecture Project

This project is set up with a comprehensive set of **Codex Skills** designed to facilitate ROS2 development following **Clean Architecture** principles.

## Authoritative References

- ROS2 Humble docs: `https://docs.ros.org/en/humble/index.html`
- Codex Skills docs: `https://developers.openai.com/codex/skills`
- Codex Rules docs: `https://developers.openai.com/codex/rules`
- OpenAI skills repository: `https://github.com/openai/skills`
- For ROS2 APIs and behavior, prioritize Humble documentation.
- For skill layout and conventions, follow Codex Skills docs first, then align with `openai/skills`.
- For command approval policies, follow Codex Rules docs and maintain `.codex/rules/default.rules`.

## Available Skills

The following skills are available in `.codex/skills` and can be used to guide development:

| Skill Name              | Description                               | Key Components                                                                        |
| ----------------------- | ----------------------------------------- | ------------------------------------------------------------------------------------- |
| **ros2-node-creation**  | Create Clean Architecture compliant nodes | `BaseNode` template, dependency injection, QoS profiles (Python & C++)               |
| **ros2-launch-config**  | Modular launch files                      | Composition, `IncludeLaunchDescription`, parameter management, C++ executable support |
| **ros2-service-action** | Services and actions                      | Server/client wrappers, domain use case integration (Python & C++)                   |
| **ros2-messaging**      | Pub/sub patterns                          | Domain-driven publishers, generic subscribers, thread-safe buffers, synchronization   |
| **ros2-testing**        | Testing strategy                          | Unit (domain), integration (node), E2E (launch), GTest/GMock support                 |
| **ros2-lifecycle**      | Managed nodes                             | Lifecycle node templates, state transition management, lifecycle clients              |
| **ros2-transforms**     | TF2 management                            | TF2 wrappers avoiding domain dependency on `geometry_msgs`                            |
| **ros2-diagnostics**    | Health monitoring                         | `diagnostic_updater` integration, health entities, frequency monitoring               |
| **ros2-bag**            | Data recording                            | Programmatic bag recording and replay utilities (`rosbag2`)                           |
| **ros2-dockerfile**     | Docker build patterns                     | ROS2 Humble base images, rosdep/colcon layers, multi-stage runtime images            |

## Project Structure

The project follows a strict separation of concerns:

- **src/domain/**: Pure business logic, entities, and use cases. No ROS2 dependencies.
- **src/application/**: Application services and interfaces. Orchestrates logic.
- **src/infrastructure/**: ROS2 specific implementations (Nodes, Publishers, Subscribers).

## Getting Started

To use a skill, reference the skill file (e.g., `.codex/skills/ros2-node-creation/SKILL.md`) for templates and best practices.
For containerization, use `.codex/skills/ros2-dockerfile/SKILL.md` with `.codex/rules/ros2_dockerfile.md`.

## Common Commands

For a comprehensive list of ROS2 commands, build instructions, and debugging tools, please refer to:

- **[ROS2 Commands Reference](.codex/commands/ros2.md)**: `colcon`, `ros2`, `rqt`, etc.

### Quick Reference

- **Build**: `colcon build --symlink-install`
- **Test**: `colcon test`
- **Source**: `source install/setup.bash`
