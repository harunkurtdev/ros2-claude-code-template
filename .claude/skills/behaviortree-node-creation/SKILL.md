---
name: behaviortree-node-creation
description: Scaffold a Behavior Tree leaf node — plain BehaviorTree.CPP (SyncActionNode / StatefulActionNode / ConditionNode) or a BehaviorTree.ROS2 wrapper (RosActionNode / RosServiceNode / RosTopicPubNode / RosTopicSubNode) — with ports, factory/plugin registration, and XML v4 usage. Trigger when the user asks to write a behavior-tree node (not Nav 2-specific).
---

# Writing a Behavior Tree Node (BehaviorTree.CPP + ROS 2)

How to write, register, and use a custom behavior-tree leaf node — both
plain BehaviorTree.CPP nodes and the ROS 2 wrappers from
BehaviorTree.ROS2.

- Core library reference: `rules/behaviortree_cpp.md`.
- ROS 2 wrappers reference: `rules/behaviortree_ros2.md`.
- Copy from the real samples:
  - plain → `~/nav2_ws/src/BehaviorTree.CPP/sample_nodes/`
    (`dummy_nodes`, `movebase_node`)
  - ROS 2 → `~/nav2_ws/src/BehaviorTree.ROS2/btcpp_ros2_samples/src/`
    (`sleep_action`, `set_bool_node`)
- Nav 2-specific BT nodes: use skill `nav2-behavior-tree` instead.

## First decision: which base class?

| Your node… | Base class | Library |
|------------|-----------|---------|
| does quick work, finishes in one tick | `BT::SyncActionNode` | BT.CPP |
| is async / long-running (no ROS) | `BT::StatefulActionNode` | BT.CPP |
| is a boolean check | `BT::ConditionNode` | BT.CPP |
| calls a ROS 2 **action** | `BT::RosActionNode<ActionT>` | BT.ROS2 |
| calls a ROS 2 **service** | `BT::RosServiceNode<ServiceT>` | BT.ROS2 |
| **publishes** to a topic | `BT::RosTopicPubNode<MsgT>` | BT.ROS2 |
| **subscribes** to a topic | `BT::RosTopicSubNode<MsgT>` | BT.ROS2 |

Golden rule: **never block in a tick**. Sync nodes return immediately;
async/ROS nodes return `RUNNING` and resume on the next tick.

## A. Plain BT.CPP node

```cpp
#include "behaviortree_cpp/bt_factory.h"

class SaySomething : public BT::SyncActionNode {
public:
  SaySomething(const std::string& name, const BT::NodeConfig& config)
    : BT::SyncActionNode(name, config) {}

  static BT::PortsList providedPorts() {                 // omit if no ports
    return { BT::InputPort<std::string>("message") };
  }
  BT::NodeStatus tick() override {
    auto msg = getInput<std::string>("message");
    if (!msg) throw BT::RuntimeError("missing 'message': ", msg.error());
    std::cout << msg.value() << "\n";
    return BT::NodeStatus::SUCCESS;
  }
};
```

Async work → `StatefulActionNode`:

```cpp
class MoveBase : public BT::StatefulActionNode {
public:
  static BT::PortsList providedPorts() { return { BT::InputPort<Pose2D>("goal") }; }
  BT::NodeStatus onStart() override;     // kick off; return RUNNING (or SUCCESS if instant)
  BT::NodeStatus onRunning() override;   // poll; return RUNNING until done, then SUCCESS/FAILURE
  void onHalted() override;              // cancellation cleanup
};
```

## B. ROS 2 wrapper node (action example)

```cpp
#include "behaviortree_ros2/bt_action_node.hpp"
#include "btcpp_ros2_interfaces/action/sleep.hpp"
using namespace BT;

class SleepAction : public RosActionNode<btcpp_ros2_interfaces::action::Sleep> {
public:
  SleepAction(const std::string& name, const NodeConfig& conf, const RosNodeParams& params)
    : RosActionNode<btcpp_ros2_interfaces::action::Sleep>(name, conf, params) {}

  static PortsList providedPorts() {
    return providedBasicPorts({ InputPort<unsigned>("msec") });   // + the action-name port
  }
  bool setGoal(Goal& goal) override {                 // BT input → goal; false ⇒ INVALID_GOAL
    goal.msec_timeout = getInput<unsigned>("msec").value();
    return true;
  }
  NodeStatus onResultReceived(const WrappedResult& wr) override {
    return wr.result->done ? NodeStatus::SUCCESS : NodeStatus::FAILURE;
  }
  NodeStatus onFeedback(const std::shared_ptr<const Feedback> fb) override {
    return NodeStatus::RUNNING;   // must NOT return IDLE
  }
  NodeStatus onFailure(ActionNodeErrorCode error) override {
    RCLCPP_ERROR(logger(), "Sleep failed: %d", error);
    return NodeStatus::FAILURE;
  }
};
```

Service nodes override `setRequest()` + `onResponseReceived()`; pub nodes
override `setMessage()`; sub nodes handle the received message.

## Registration

Plain BT.CPP:
```cpp
factory.registerNodeType<SaySomething>("SaySomething");
// free function → factory.registerSimpleAction("CheckBattery", std::bind(CheckBattery));
// plugin: BT_REGISTER_NODES(factory){ factory.registerNodeType<MyNode>("MyNode"); }
//         → factory.registerFromPlugin("libmy_nodes.so");
```

ROS 2 (needs `RosNodeParams` to carry the node handle / timeouts):
```cpp
BT::RosNodeParams params;
params.nh = node;                       // std::shared_ptr<rclcpp::Node>
params.default_port_value = "sleep_action";   // default action/service/topic name
factory.registerNodeType<SleepAction>("SleepAction", params);
// or as a plugin:  CreateRosNodePlugin(SleepAction, "SleepAction");
//                  RegisterRosNode(factory, "libsleep_action.so", params);
```

## Use it in a tree (XML v4)

```xml
<root BTCPP_format="4">
  <BehaviorTree ID="MainTree">
    <Sequence>
      <SaySomething message="starting"/>
      <SleepAction msec="2000"/>
    </Sequence>
  </BehaviorTree>
</root>
```

```cpp
factory.registerBehaviorTreeFromFile("main.xml");
auto tree = factory.createTree("MainTree");
tree.tickWhileRunning();   // ROS nodes: spin the node in parallel / use the executor
```

To expose a whole tree as a ROS 2 action, subclass
`BT::TreeExecutionServer` and override `registerNodesIntoFactory()` (see
`behaviortree_ros2.md`).

## Ports & blackboard

- `getInput<T>("name")` returns `BT::Expected<T>` — always check it.
- `setOutput("name", value)` writes to the blackboard.
- `{key}` in XML binds a port to a blackboard entry.
- For custom port types, specialize `BT::convertFromString<T>(StringView)`.
- Reserved port names: `name`, `ID`, anything starting with `_`.

## Common pitfalls

- **Blocking in `tick()` / `onRunning()`** — defeats the tree; return
  `RUNNING` and resume next tick.
- **`onFeedback` returning `IDLE`** — not allowed (throws); return
  `RUNNING` (or SUCCESS/FAILURE to stop early).
- **Forgetting `providedPorts()`** when the XML uses ports, or a name
  mismatch between `providedPorts` and the XML attribute.
- **Registering a ROS wrapper without `RosNodeParams`** — it needs the
  node handle; use the `(factory, params)` overload / `RegisterRosNode`.
- **Mixing v3 and v4 XML** — set `BTCPP_format="4"`; migrate with
  `convert_v3_to_v4.py`.
- **Type mismatch on ports** — once an entry is written strongly-typed,
  its type locks; a `std::string` output can convert, others can't.
- Use BT.CPP nodes for pure logic; reach for the ROS 2 wrappers only when
  the node actually talks to a ROS 2 interface.
