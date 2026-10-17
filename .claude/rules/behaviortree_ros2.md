# BehaviorTree.ROS2 — ROS 2 Integration Reference

The official bridge that exposes **ROS 2 actions / services / topics as
BehaviorTree.CPP leaf nodes**. Source: `~/nav2_ws/src/BehaviorTree.ROS2/`
(~**v0.3.0**, `humble`). Builds on BehaviorTree.CPP (see
`behaviortree_cpp.md`).

- Upstream: <https://github.com/BehaviorTree/BehaviorTree.ROS2>
- Distinct from `nav2_behavior_tree` (Nav 2-specific nodes): this is a
  **generic** wrapper for any ROS 2 interface.

## Packages

| Package | Purpose |
|---------|---------|
| `behaviortree_ros2` | the wrapper base classes + `TreeExecutionServer` |
| `btcpp_ros2_interfaces` | `ExecuteTree.action`, `Sleep.action`, `NodeStatus.msg`, `GetTrees.srv` |
| `btcpp_ros2_samples` | runnable examples (sleep action, set_bool service, executor) |

## 1. The four wrapper base classes

All are templates that inherit from `BT::ActionNodeBase` and bridge the
**synchronous BT tick** to **asynchronous ROS 2**: a pending call returns
`RUNNING` across ticks, then `SUCCESS`/`FAILURE` when it resolves.

| Wrapper | ROS 2 primitive | Override |
|---------|-----------------|----------|
| `RosActionNode<ActionT>` | action client | `setGoal()`, `onResultReceived()`, `onFeedback()`, `onFailure()`, `onHalt()` |
| `RosServiceNode<ServiceT>` | service client | `setRequest()`, `onResponseReceived()`, `onFailure()` |
| `RosTopicPubNode<MsgT>` | publisher | `setMessage()` |
| `RosTopicSubNode<MsgT>` | subscriber | message handling / `onTick()` |

```cpp
class SleepAction : public RosActionNode<btcpp_ros2_interfaces::action::Sleep>
{
public:
  SleepAction(const std::string& name, const NodeConfig& conf, const RosNodeParams& params)
    : RosActionNode<...::Sleep>(name, conf, params) {}

  static BT::PortsList providedPorts() {
    return providedBasicPorts({ InputPort<unsigned>("msec") });   // BT ports + the ROS port
  }
  bool setGoal(Goal& goal) override;                              // BT input → action goal
  BT::NodeStatus onResultReceived(const WrappedResult& wr) override;  // result → status
  BT::NodeStatus onFeedback(const std::shared_ptr<const Feedback> fb) override;
  BT::NodeStatus onFailure(ActionNodeErrorCode error) override;
  void onHalt() override;                                         // cancel the goal
};
```

`providedBasicPorts({...})` merges your ports with the wrapper's own port
(used to override the action/service/topic name from XML).

### Error codes

- Action: `SERVER_UNREACHABLE`, `SEND_GOAL_TIMEOUT`,
  `GOAL_REJECTED_BY_SERVER`, `ACTION_ABORTED`, `ACTION_CANCELLED`,
  `INVALID_GOAL`.
- Service: `SERVICE_UNREACHABLE`, `SERVICE_TIMEOUT`, `INVALID_REQUEST`.

## 2. RosNodeParams (wiring)

```cpp
struct RosNodeParams {
  std::weak_ptr<rclcpp::Node> nh;
  std::string default_port_value;     // action server / service / topic name (context-dependent)
  std::chrono::milliseconds server_timeout = 1000ms;
  std::chrono::milliseconds wait_for_server_timeout = 500ms;
};
```

The node handle and timeouts are injected at registration; the
`default_port_value` is the ROS name (overridable via the XML port).

## 3. Registration (plugin macros)

```cpp
// register into a factory directly, with the shared RosNodeParams:
RegisterRosNode(factory, "libsleep_action.so", params);

// inside the node's library (loaded as a plugin):
CreateRosNodePlugin(SleepAction, "SleepAction");
// or, for several nodes:
BT_REGISTER_ROS_NODES(factory, params) {
  factory.registerNodeType<SleepAction>("SleepAction", params);
}
```

Plugin symbol: `BT_RegisterRosNodeFromPlugin`. Unlike plain BT.CPP
plugins, ROS plugins receive `RosNodeParams` so they can capture the node
handle.

## 4. TreeExecutionServer (run a tree over a ROS 2 action)

A ready-made server that executes a behavior tree on request via the
`ExecuteTree` action. Configured with `generate_parameter_library`
(`bt_executor_parameters.yaml`).

`ExecuteTree.action`:
```
# Goal
string target_tree      # which tree to run
string payload          # optional, user-defined
---
# Result
NodeStatus node_status
string return_message
---
# Feedback
string message
```

Customize by subclassing and overriding the virtual hooks:

| Hook | When |
|------|------|
| `registerNodesIntoFactory(factory)` | register your nodes before tree creation |
| `onGoalReceived(tree_name, payload)` | accept/reject a goal |
| `onTreeCreated(tree)` | after the tree is built |
| `onLoopAfterTick(status)` | each loop after `tickOnce()` — can override the result |
| `onLoopFeedback()` | produce action feedback each loop |
| `onTreeExecutionCompleted(status, cancelled)` | after execution ends |

This is the idiomatic way to expose "run BT X" to the rest of a ROS 2
system, analogous to how Nav 2's `bt_navigator` hosts navigation trees.

## 5. Logging

`RosBtLogger` (`bt_ros_logger.hpp`) routes BT transitions to the ROS 2
logger; combine with BT.CPP's `Groot2Publisher` for live visualization.

## 6. When to use what

| Need | Use |
|------|-----|
| Call a ROS 2 action/service/topic from a tree | the matching wrapper above |
| Run a whole tree as a ROS 2 action service | `TreeExecutionServer` |
| Navigation-specific BT nodes | `nav2_behavior_tree` (skill `nav2-behavior-tree`) |
| A pure-logic node (no ROS) | plain BT.CPP `SyncActionNode`/`StatefulActionNode` (`behaviortree_cpp.md`) |

See skill `behaviortree-node-creation` for the full how-to.
