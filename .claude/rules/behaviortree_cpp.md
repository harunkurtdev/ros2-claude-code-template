# BehaviorTree.CPP v4 — Core Reference

The C++ behavior-tree library that Nav 2 and BehaviorTree.ROS2 build on.
Source: `~/nav2_ws/src/BehaviorTree.CPP/` (~**v4.9.0**, namespace `BT::`,
library `behaviortree_cpp`). The ROS 2 integration is a separate layer —
see `behaviortree_ros2.md`.

- Upstream + docs: <https://www.behaviortree.dev>
- The repo ships its own `CLAUDE.md` at its root — read it for build /
  lint / test specifics. C++17, CMake 3.16.3+, Google style, 90-col.

> A behavior tree is a **tick-driven** tree: the root is ticked at some
> frequency; each tick propagates down and every node returns a
> `NodeStatus`. Leaf nodes do the work; control/decorator nodes shape the
> flow. Data flows through a typed **Blackboard** via **ports**.

---

## 1. Node hierarchy

All nodes inherit from `TreeNode` (`include/behaviortree_cpp/tree_node.h`):

```
TreeNode
├── LeafNode
│   ├── ActionNode      — does work (Sync / Stateful / coroutine)
│   └── ConditionNode   — returns SUCCESS/FAILURE only, no RUNNING
├── ControlNode         — multiple children, defines flow
└── DecoratorNode       — exactly one child, modifies it
```

| Category | Built-ins |
|----------|-----------|
| **Control** | `SequenceNode`, `SequenceWithMemory`, `FallbackNode`, `ParallelNode`, `ParallelAllNode`, `ReactiveSequence`, `ReactiveFallback`, `IfThenElseNode`, `WhileDoElseNode`, `SwitchNode` |
| **Decorator** | `InverterNode`, `RetryNode`, `RepeatNode`, `TimeoutNode`, `DelayNode`, `ForceSuccess`, `ForceFailure`, `KeepRunningUntilFailure`, `RunOnce`, `SubtreeNode` |
| **Action (built-in)** | `AlwaysSuccess`, `AlwaysFailure`, `SetBlackboard`, `Sleep`, `Script`, `UnsetBlackboard` |

## 2. NodeStatus

```
IDLE      — not yet ticked (or reset)
RUNNING   — asynchronous work in progress (returned across ticks)
SUCCESS   — completed OK
FAILURE   — completed with failure
SKIPPED   — skipped via a precondition (v4 feature)
```

## 3. The three ways to write a leaf node

| Base class | Use when | Override |
|------------|----------|----------|
| `SyncActionNode` | work completes within one tick | `tick()` → SUCCESS/FAILURE |
| `StatefulActionNode` | asynchronous / long-running | `onStart()`, `onRunning()` (return RUNNING until done), `onHalted()` |
| `ConditionNode` | a boolean check | `tick()` → SUCCESS/FAILURE (never RUNNING) |

```cpp
class SaySomething : public BT::SyncActionNode {
public:
  SaySomething(const std::string& name, const BT::NodeConfig& config)
    : BT::SyncActionNode(name, config) {}

  static BT::PortsList providedPorts() {           // MANDATORY if it has ports
    return { BT::InputPort<std::string>("message") };
  }
  BT::NodeStatus tick() override {
    auto msg = getInput<std::string>("message");
    std::cout << msg.value() << std::endl;
    return BT::NodeStatus::SUCCESS;
  }
};
```

A `StatefulActionNode` **must not block** in `onRunning()` — return
`RUNNING` and let the next tick resume; implement `onHalted()` for
cancellation.

## 4. Factory, registration & XML

`BehaviorTreeFactory` (`bt_factory.h`) registers node types and builds
trees from XML.

```cpp
BT::BehaviorTreeFactory factory;
factory.registerNodeType<SaySomething>("SaySomething");      // by name = the XML tag
factory.registerSimpleAction("CheckBattery", std::bind(CheckBattery));  // free function
factory.registerBehaviorTreeFromFile("my_tree.xml");
auto tree = factory.createTree("MainTree");
tree.tickWhileRunning();      // or tickOnce() in your own loop
```

Plugins (shared libs loaded at runtime):

```cpp
BT_REGISTER_NODES(factory) { factory.registerNodeType<MyNode>("MyNode"); }
// → build a .so, load via factory.registerFromPlugin("libmy_nodes.so")
```

XML v4 shape:

```xml
<root BTCPP_format="4">
  <BehaviorTree ID="MainTree">
    <Sequence>
      <CheckBattery/>
      <SaySomething message="hello"/>
      <SaySomething message="{greeting}"/>   <!-- {key} = blackboard entry -->
    </Sequence>
  </BehaviorTree>
</root>
```

`convert_v3_to_v4.py` migrates v3 trees. v4 added `SKIPPED`,
pre/post-condition scripts, and the scripting language.

## 5. Blackboard & ports (typed dataflow)

The **Blackboard** is shared typed key-value storage. Ports connect a
node's input/output to a blackboard entry referenced as `{key}` in XML.

- `InputPort<T>("name")`, `OutputPort<T>("name")`, `BidirectionalPort<T>`.
- Read with `getInput<T>("name")` (returns `Expected<T>`), write with
  `setOutput("name", value)`.
- Port-connection rules (see `docs/PORT_CONNECTION_RULES.md`):
  - same-typed ports always connect;
  - generic ports (`BT::Any`, `AnyTypeAllowed`) accept anything;
  - a `std::string` output is a "universal donor" — converted to the
    consumer type via `convertFromString<T>` (specialize this for custom
    types);
  - type locks after the first strongly-typed write;
  - reserved names: `name`, `ID`, and any starting with `_`.

## 6. Scripting & pre/post conditions (v4)

Nodes accept embedded script expressions, e.g.
`_skipIf="battery_low"`, `_successIf`, `_failureIf`, `_post`, plus the
`<Script code="x := 5"/>` action. Backed by the lexy-based parser in
`scripting/`.

## 7. Loggers / observability

`loggers/`: `Groot2Publisher` (live visualization + recording over ZeroMQ,
`BTCPP_GROOT_INTERFACE`), `FileLogger2`, `SqliteLogger`,
`StdCoutLogger`, `MinitraceLogger`. Attach to a `Tree` to trace ticks.

## 8. Source / include layout

```
include/behaviortree_cpp/
├── *.h            # tree_node, blackboard, bt_factory, basic_types, behavior_tree
├── controls/  decorators/  actions/   # node category headers
├── loggers/   scripting/   utils/  contrib/
src/  ... (mirrors include) + loggers/
3rdparty/          # vendored: TinyXML2, cppzmq, flatbuffers, lexy, minicoro, minitrace
sample_nodes/      # DummyNodes, CrossDoor, MoveBase examples (copy these)
```

## 9. Relationship to Nav 2 and ROS 2

- **Nav 2** uses BehaviorTree.CPP directly; its 82 BT nodes live in
  `nav2_behavior_tree` (see skill `nav2-behavior-tree`). Those are Nav 2
  specific.
- **BehaviorTree.ROS2** wraps ROS 2 actions/services/topics as BT leaf
  nodes generically — see `behaviortree_ros2.md` and the skill
  `behaviortree-node-creation`.
- BT.CPP itself has **no ROS dependency**; ROS is auto-detected
  (`ament_cmake`) only for packaging.
