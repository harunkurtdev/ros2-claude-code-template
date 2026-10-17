---
description: Scaffold a Behavior Tree leaf node — plain BehaviorTree.CPP (sync/stateful/condition) or a BehaviorTree.ROS2 wrapper (action/service/topic) — with ports, registration, and an XML usage snippet.
argument-hint: "<package> <ClassName> <sync|stateful|condition|ros-action|ros-service|ros-topic-pub|ros-topic-sub>"
allowed-tools: ["Bash", "Read", "Write", "Edit"]
---

Scaffold a behavior-tree node inside an existing package.

Argument handling (`$ARGUMENTS`):
1. `<package>` — existing package under `src/` (required).
2. `<ClassName>` — CamelCase node class (required).
3. `<kind>` — one of:
   * `sync` → `BT::SyncActionNode` (finishes in one tick)
   * `stateful` → `BT::StatefulActionNode` (async; onStart/onRunning/onHalted)
   * `condition` → `BT::ConditionNode`
   * `ros-action` → `BT::RosActionNode<ActionT>`
   * `ros-service` → `BT::RosServiceNode<ServiceT>`
   * `ros-topic-pub` → `BT::RosTopicPubNode<MsgT>`
   * `ros-topic-sub` → `BT::RosTopicSubNode<MsgT>`

For the `ros-*` kinds, also ask for the interface type (e.g.
`btcpp_ros2_interfaces/action/Sleep`). For Nav 2-specific BT nodes, use
`/new-nav2-plugin bt_node` instead.

If anything is missing, ask before scaffolding.

Process:
1. Read the skill `behaviortree-node-creation` and the rules
   `behaviortree_cpp.md` (plain) / `behaviortree_ros2.md` (ros-*). Mirror
   the closest sample (`~/nav2_ws/src/BehaviorTree.CPP/sample_nodes/` or
   `~/nav2_ws/src/BehaviorTree.ROS2/btcpp_ros2_samples/src/`).
2. Create:
   * `include/<package>/<snake_class>.hpp` — the class deriving from the
     chosen base, with `static providedPorts()` (use `providedBasicPorts`
     for ros-* wrappers) and the required overrides:
     - sync/condition: `tick()`
     - stateful: `onStart()`, `onRunning()`, `onHalted()`
     - ros-action: `setGoal()`, `onResultReceived()`, `onFeedback()`,
       `onFailure()`, `onHalt()`
     - ros-service: `setRequest()`, `onResponseReceived()`, `onFailure()`
     - ros-topic-pub/sub: `setMessage()` / message handler
   * `src/<snake_class>.cpp` — implementation (non-blocking ticks!).
3. Registration:
   * plain → `BT_REGISTER_NODES(factory){ factory.registerNodeType<Class>("Name"); }`
   * ros-* → `CreateRosNodePlugin(Class, "Name")` (uses `RosNodeParams`).
   * Or register directly in the executor with `registerNodeType<Class>(...)`.
4. `CMakeLists.txt` / `package.xml`: dep `behaviortree_cpp` (+
   `behaviortree_ros2` and the message package for ros-*); build the
   plugin as a shared lib if registering via plugin.
5. Add an XML usage snippet (`BTCPP_format="4"`) and, where useful, a
   gtest that ticks the node.
6. `pre-commit run --files <touched files>`.
7. Print: files created and the XML tag + how to register it.

Never block inside `tick()`/`onRunning()`; return `RUNNING` and resume on
the next tick. `onFeedback()` must not return `IDLE`.
