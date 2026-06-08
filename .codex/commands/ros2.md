# ROS2 Common Commands

## Build System (colcon)

| Action              | Command                                     | Description                                                              |
| ------------------- | ------------------------------------------- | ------------------------------------------------------------------------ |
| **Build Workspace** | `colcon build --symlink-install`            | Build all packages with symlinks (allows python changes without rebuild) |
| **Build Package**   | `colcon build --packages-select <pkg_name>` | Build a specific package only                                            |
| **Build Up To**     | `colcon build --packages-up-to <pkg_name>`  | Build a package and its dependencies                                     |
| **Clean Build**     | `rm -rf build/ install/ log/`               | Remove build artifacts (use with caution)                                |
| **Test**            | `colcon test`                               | Run tests for all packages                                               |
| **Test Output**     | `colcon test-result --all`                  | Show test results                                                        |

## Environment

| Action             | Command                     | Description                  |
| ------------------ | --------------------------- | ---------------------------- | ------------------------------- |
| **Source Overlay** | `source install/setup.bash` | Source the workspace overlay |
| **Check Env**      | `printenv                   | grep ROS`                    | Check ROS environment variables |

## Introspection (ros2)

### Nodes

| Command                      | Description                                      |
| ---------------------------- | ------------------------------------------------ |
| `ros2 node list`             | List all running nodes                           |
| `ros2 node info <node_name>` | Show subscribers, publishers, services of a node |

### Topics

| Command                                | Description                                      |
| -------------------------------------- | ------------------------------------------------ |
| `ros2 topic list`                      | List all active topics                           |
| `ros2 topic echo <topic>`              | Print messages from a topic to screen            |
| `ros2 topic info <topic>`              | Show message type and publisher/subscriber count |
| `ros2 topic pub <topic> <type> <args>` | Publish a message manually                       |
| `ros2 topic hz <topic>`                | Check publishing frequency                       |
| `ros2 topic bw <topic>`                | Check bandwidth usage                            |

### Services

| Command                                     | Description             |
| ------------------------------------------- | ----------------------- |
| `ros2 service list`                         | List available services |
| `ros2 service call <service> <type> <args>` | Call a service manually |
| `ros2 service type <service>`               | Show service type       |

### Actions

| Command                                        | Description            |
| ---------------------------------------------- | ---------------------- |
| `ros2 action list`                             | List available actions |
| `ros2 action send_goal <action> <type> <goal>` | Send an action goal    |

### Parameters

| Command                                 | Description                           |
| --------------------------------------- | ------------------------------------- |
| `ros2 param list`                       | List all parameters                   |
| `ros2 param get <node> <param>`         | Get a parameter value                 |
| `ros2 param set <node> <param> <value>` | Set a parameter value dynamically     |
| `ros2 param dump <node>`                | Dump all parameters of a node to YAML |

## Debugging Implementation

| Tool            | Command                          | Description                               |
| --------------- | -------------------------------- | ----------------------------------------- |
| **RQT Graph**   | `rqt_graph`                      | Visualize node and topic connections      |
| **RQT Console** | `rqt_console`                    | View log messages with filtering          |
| **TF Tree**     | `ros2 run tf2_tools view_frames` | Generate PDF of TF tree                   |
| **Doctor**      | `ros2 doctor`                    | Check for system issues/misconfigurations |
