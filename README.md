# ROS2 Clean Architecture Environment

## Project Purpose

This project establishes a standardized, robust, and maintainable environment for developing **ROS2 (Robot Operating System 2)** applications. It is designed to strictly adhere to **Clean Architecture** principles, ensuring that business logic is decoupled from the underlying ROS2 framework.

The primary goal is to provide a comprehensive set of rules, templates, and guidelines that enable developers to build scalable robotic software with consistent patterns in both **Python** and **C++**.

## Key Features

### 1. Clean Architecture Implementation

The project enforces a clear separation of concerns:

- **Domain Layer**: Core business logic and entities (Framework-agnostic).
- **Application Layer**: Use cases and application-specific logic.
- **Infrastructure Layer**: ROS2 adapters, hardware interfaces, and repositories.
- **Presentation Layer**: CLI tools, GUIs, and external APIs.

### 2. Bilingual Support (Python & C++)

Recognizing the dual nature of the ROS2 ecosystem, this environment provides equal support for both languages:

- **Standardized Nodes**: Templates for standard Nodes, Lifecycle Nodes, and Managed Nodes.
- **Communication patterns**: Consistent implementation of Publishers, Subscribers, Services, and Actions.
- **Build Systems**: Best practices for `setup.py` (Python) and `CMakeLists.txt` (C++).

### 3. Comprehensive Rule Set

The `.claude/rules` directory contains detailed guidelines for:

- **Architecture**: Defining layer boundaries and dependency rules.
- **Node Development**: Patterns for creating robust and testable nodes.
- **Communication**: Standards for Topic naming, QoS profiles, and custom interfaces.
- **Testing**: Strategies for Unit (GTest/pytest), Integration, and Launch testing.

### 4. Developer Skills & Templates

A library of "Skills" (`.claude/skills`) provides ready-to-use templates and explanations for:

- **Node Creation & Lifecycle Management**
- **messaging Patterns (Pub/Sub, Services, Actions)**
- **Launch Configuration & Parameters**
- **TF2 Transforms & Diagnostics**
- **Bag Recording & Replay**

## Getting Started

1.  **Review the Rules**: Check the `.claude/rules/` directory to understand the architectural standards.
2.  **Use the Skills**: Refer to `.claude/skills/` for implementation examples and templates.
3.  **Run Tests**: Use `colcon test` and `pytest` to verify your implementations.

## License

This project is open-source and available under the Apache 2.0 License.
