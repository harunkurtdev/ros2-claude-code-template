---
description: ROS2 General Conventions and Best Practices
---

# ROS2 General Conventions

This rule file defines general conventions and best practices for ROS2 projects.

## Package Naming

```
# Format: <organization>_<component>_<type>
# Examples:
robot_navigation_core
robot_perception_msgs
robot_control_interfaces
```

- Package names must be in **snake_case**.
- Use only lowercase letters and underscores.
- Names should be short and descriptive.

## File Structure

```
package_name/
├── package.xml              # Package manifest
├── setup.py                 # Python package setup
├── setup.cfg                # Setup configuration (Python)
├── CMakeLists.txt           # Build configuration (C++)
├── resource/
│   └── package_name         # Marker file
├── package_name/            # Python source
│   ├── __init__.py
│   └── node_file.py
├── src/                     # C++ source
│   └── node_file.cpp
├── include/                 # C++ headers
│   └── package_name/
│       └── node_file.hpp
├── launch/
│   └── node_launch.py
├── config/
│   └── params.yaml
└── test/
    └── test_node.py
```

## CMakeLists.txt (C++ Packages)

```cmake
cmake_minimum_required(VERSION 3.8)
project(package_name)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

# Add targets here
add_executable(my_node src/node_file.cpp)
target_include_directories(my_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(my_node rclcpp)

install(TARGETS my_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY include/
  DESTINATION include)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
```

## Python Setup Best Practices

```python
from setuptools import find_packages, setup

package_name = 'package_name'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch',
            ['launch/node_launch.py']),
        ('share/' + package_name + '/config',
            ['config/params.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your.email@example.com',
    description='Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'node_name = package_name.node_file:main',
        ],
    },
)
```

## Launch Files

```python
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # Declare arguments
    use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time'
    )

    # Define nodes
    my_node = Node(
        package='package_name',
        executable='node_name',
        name='node_name',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time')
        }],
        output='screen'
    )

    return LaunchDescription([
        use_sim_time,
        my_node,
    ])
```

## Parameter Files (YAML)

```yaml
/**:
  ros__parameters:
    # Node-specific parameters
    update_rate: 10.0

    # Nested parameters
    sensor:
      frame_id: "base_link"
      range_min: 0.1
      range_max: 10.0
```

## Logging Best Practices

```python
# Use appropriate log levels
self.get_logger().debug('Detailed debug info')
self.get_logger().info('Normal operation info')
self.get_logger().warn('Warning message')
self.get_logger().error('Error message')
self.get_logger().fatal('Fatal error')

# Use throttled logging for high-frequency messages
self.get_logger().info('Status update', throttle_duration_sec=1.0)

# Use once logging for initialization
self.get_logger().info('Node initialized', once=True)
```

## Workspace Organization

```
ros2_ws/
├── src/
│   ├── robot_core/          # Core packages
│   ├── robot_interfaces/    # Message/service definitions
│   ├── robot_bringup/       # Launch and config
│   └── third_party/         # External dependencies
├── build/                   # Build directory
├── install/                 # Install directory
└── log/                     # Log directory
```
