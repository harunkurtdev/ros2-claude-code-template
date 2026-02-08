---
name: ROS2 Launch & Configuration
description: Clean Architecture compatible ROS2 launch files and parameter management (Python)
---

# ROS2 Launch & Configuration Skill

This skill provides a guide for creating modular and reusable ROS2 launch files, which are written in Python for both C++ and Python nodes.

## Launch File Structure

```
packages/
└── robot_core/
    └── launch/
        ├── robot_launch.py          # Main launch file
        ├── sensors_launch.py        # Sensor subsystem
        ├── navigation_launch.py     # Navigation subsystem
        └── includes/
            ├── common.py            # Common functions
            └── defaults.py          # Default values
```

## Basic Launch File Template

```python
#!/usr/bin/env python3
"""
Launch file: robot_launch.py
Description: Main robot system launch file
"""

import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    GroupAction,
    SetEnvironmentVariable,
    LogInfo
)
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node, SetParameter
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Generate launch description."""

    # Package paths
    pkg_robot_core = get_package_share_directory('robot_core')

    # Launch arguments
    declared_arguments = [
        DeclareLaunchArgument(
            'robot_name',
            default_value='robot_1',
            description='Robot namespace'
        ),
        DeclareLaunchArgument(
            'use_sim',
            default_value='false',
            description='Use simulation mode'
        ),
        DeclareLaunchArgument(
            'config_file',
            default_value=os.path.join(pkg_robot_core, 'config', 'params.yaml'),
            description='Path to parameter file'
        ),
    ]

    # Configurations
    robot_name = LaunchConfiguration('robot_name')
    use_sim = LaunchConfiguration('use_sim')
    config_file = LaunchConfiguration('config_file')

    # Environment setup
    env_setup = [
        SetEnvironmentVariable('RCUTILS_COLORIZED_OUTPUT', '1'),
    ]

    # Global parameters
    global_params = SetParameter(name='use_sim_time', value=use_sim)

    # C++ Node Example
    cpp_node = Node(
        package='robot_cpp_pkg',
        executable='robot_controller_cpp', # Name of the C++ executable in CMakeLists.txt
        name='controller',
        namespace=robot_name,
        parameters=[config_file],
        output='screen'
    )

    # Python Node Example
    py_node = Node(
        package='robot_py_pkg',
        executable='sensor_node.py', # Name of the script or entry point
        name='sensors',
        namespace=robot_name,
        parameters=[config_file],
        output='screen',
        condition=UnlessCondition(use_sim)
    )

    return LaunchDescription(
        declared_arguments +
        env_setup +
        [global_params, cpp_node, py_node]
    )
```

## Parameter File Structure (YAML)

```yaml
# config/params.yaml
/**:
  ros__parameters:
    # Global parameters
    use_sim_time: false
    log_level: "info"

robot_state:
  ros__parameters:
    # Robot state node parameters
    update_rate: 100.0
    frame_id: "base_link"

    # Nested parameters
    position_filter:
      type: "kalman"
      process_noise: 0.01

navigation:
  ros__parameters:
    max_velocity: 1.5
    planner:
      type: "astar"
```

## Loading Parameters in C++

```cpp
// Within a ROS2 Node
void load_parameters() {
    this->declare_parameter("update_rate", 10.0);
    this->declare_parameter("position_filter.type", "default");

    double rate = this->get_parameter("update_rate").as_double();
    std::string filter_type = this->get_parameter("position_filter.type").as_string();

    RCLCPP_INFO(this->get_logger(), "Rate: %f, Filter: %s", rate, filter_type.c_str());
}
```

## Lifecycle Node Launch integration

```python
from launch_ros.actions import LifecycleNode
from launch_ros.events.lifecycle import ChangeState
from lifecycle_msgs.msg import Transition
from launch.actions import EmitEvent, RegisterEventHandler
from launch.event_handlers import OnProcessStart

def generate_launch_description():
    # ...
    driver_node = LifecycleNode(
        package='robot_drivers',
        executable='lidar_driver',
        name='lidar',
        namespace='',
        output='screen'
    )

    # Auto-configure on start
    configure_event = RegisterEventHandler(
        OnProcessStart(
            target_action=driver_node,
            on_start=[
                EmitEvent(event=ChangeState(
                    lifecycle_node_matcher=lambda n: n == driver_node,
                    transition_id=Transition.TRANSITION_CONFIGURE,
                )),
            ]
        )
    )

    return LaunchDescription([driver_node, configure_event])
```
