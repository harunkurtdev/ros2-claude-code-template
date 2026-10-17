---
name: vda5050-integration
description: Bridge a VDA 5050 v3.0.0 fleet-control interface (MQTT/JSON) onto Nav 2 under Clean Architecture — domain entities, MQTT/Nav 2 adapters behind ports, order→NavigateThroughPoses mapping, state aggregation, action handlers. Trigger when the user asks to build or extend a VDA 5050 connector / fleet bridge.
---

# VDA 5050 ↔ ROS 2 / Nav 2 Integration

How to bridge a **VDA 5050 v3.0.0** fleet-control interface (MQTT/JSON)
onto a Nav 2 stack **without breaking Clean Architecture**.

- Protocol overview: `rules/vda5050_protocol.md`.
- Complete message/field spec + processes: `rules/vda5050_messages.md`.
- **Code-generation format analysis** (3 proven idioms — pydantic /
  ROS `.msg`+bridge / C++ structs): `rules/vda5050_implementation_formats.md`.
  Real reference repos in `~/nav2_ws/src/`: `isaac_mission_dispatch`
  (fleet/pydantic), `isaac_ros_cloud_control` (robot/ROS 2),
  `vda5050_core` (C++ core).
- Authoritative spec + JSON schemas: `~/nav2_ws/src/VDA5050/`
  (`VDA5050_EN.md`, `json_schemas/*.schema`).
- **Reference implementation** (working, Clean-Architecture):
  `~/nav2_ws/src/vda5050_connector/`. Study it before writing a new one.

## The one rule that drives the design

VDA 5050 is an **outer-world protocol** — MQTT topics, JSON payloads,
schema versions. By the layer rules in `rules/clean_architecture.md`:

> The protocol does **not** belong in the domain. Domain entities model
> *orders, nodes, edges, actions, state* as plain Python/C++ — no MQTT,
> no `json`, no `*_msgs`. MQTT and Nav 2 are **infrastructure adapters**
> behind domain ports.

```
PRESENTATION   launch entrypoint, CLI, RViz
     │
APPLICATION    OrderHandler · StateAssembler · InstantActionHandler
     │            (graph traversal, action queue, state aggregation)
DOMAIN         entities: Order/Node/Edge/Action/State/Connection/Factsheet
     ▲            ports:  MqttClientInterface · NavigationAdapterInterface
     │ implements
INFRASTRUCTURE MqttBridge (paho) · Nav2Adapter (rclpy + NavigateToPose)
```

The connector mirrors this exactly:

```
vda5050_connector/
├── domain/
│   ├── entities/        order.py state.py action.py connection.py
│   │                    factsheet.py visualization.py header.py graph.py
│   └── interfaces/      mqtt_client.py  navigation_adapter.py   ← PORTS
├── application/         order_handler.py  state_assembler.py
│                        action_handler.py
└── infrastructure/      mqtt_bridge.py  nav2_adapter.py
                         fleet_adapter.py  vda5050_node.py        ← composition root
```

## Layer responsibilities

### Domain — entities + ports (no ROS, no MQTT, no JSON)

Model each message as a dataclass tree. Keep field names aligned with
the JSON schema so (de)serialization is mechanical, but **do not** put
`json.loads` here — that is an adapter concern.

```python
# domain/entities/order.py  (excerpt of the real connector)
@dataclass
class NodePosition:
    x: float = 0.0
    y: float = 0.0
    theta: float = 0.0
    map_id: str = ''

@dataclass
class OrderNode:
    node_id: str = ''
    sequence_id: int = 0
    released: bool = False
    node_position: Optional[NodePosition] = None
    actions: List[Action] = field(default_factory=list)

@dataclass
class Order:
    order_id: str = ''
    order_update_id: int = 0
    nodes: List[OrderNode] = field(default_factory=list)
    edges: List[OrderEdge] = field(default_factory=list)
```

Ports (abstract) live in `domain/interfaces/`:

```python
class NavigationAdapterInterface(ABC):
    @abstractmethod
    def navigate_to(self, node: OrderNode, on_done) -> None: ...
    @abstractmethod
    def cancel(self) -> None: ...
    @abstractmethod
    def get_position(self) -> AGVPosition: ...

class MqttClientInterface(ABC):
    @abstractmethod
    def publish(self, topic, payload, qos=0, retain=False) -> None: ...
    @abstractmethod
    def subscribe(self, topic, callback, qos=0) -> None: ...
    @abstractmethod
    def set_last_will(self, topic, payload, qos=1, retain=True) -> None: ...
```

### Application — protocol logic, depends only on domain

- **`OrderHandler`** — validates an incoming `Order`, enforces
  base/horizon, runs the node/edge graph in `sequenceId` order,
  resolves missing `nodePosition` from a `NavigationGraph` (VDA 5050
  allows omitting position when both MC and robot know the node),
  tracks `nodeStates`/`edgeStates`, and calls `NavigationAdapterInterface`.
  Handles order **updates / stitching** (same `orderId`, next
  `orderUpdateId`, first node = `lastNodeId`).
- **`StateAssembler`** — aggregates position, velocity, battery, errors,
  `actionStates`, `nodeStates`/`edgeStates` into a `State` entity for
  periodic publish.
- **`InstantActionHandler`** — `cancelOrder`, `startPause`/`stopPause`,
  `stateRequest`, `factsheetRequest`, `pick`/`drop`, etc.; drives the
  `actionStatus` state machine (`WAITING → … → FINISHED/FAILED/RETRIABLE`).

### Infrastructure — adapters implementing the ports

- **`MqttBridge`** (paho-mqtt) — topic `vda5050/v3/<mfr>/<serial>/<topic>`,
  QoS 0 for everything except `connection` (QoS 1), MQTT **last will** on
  `connection` = `CONNECTION_BROKEN`. (De)serializes JSON ↔ domain
  entities here, and **validates against `json_schemas/*.schema`**.
- **`Nav2Adapter`** (`rclpy`) — maps `navigate_to(node)` →
  `NavigateToPose` action; reads telemetry from running topics
  (`/amcl_pose` → position, `/odom` → velocity, `/battery_state`,
  `/scan` → safety, `/diagnostics` → errors/info).
- **`vda5050_node.py`** — composition root: instantiates adapters,
  injects ports into the application services, runs the publish timers.

## VDA 5050 → Nav 2 mapping cheat-sheet

| VDA 5050 | Nav 2 / ROS 2 |
|----------|----------------|
| `order` node (released base) | `NavigateToPose` goal (per node) or `NavigateThroughPoses` (whole base) |
| `nodePosition {x,y,theta,mapId}` | `geometry_msgs/PoseStamped` in `map` frame |
| `edge.maximumSpeed` | controller `setSpeedLimit` / velocity-smoother cap |
| `edge.trajectory` (NURBS) | sample → `nav_msgs/Path` (or let the planner replan) |
| `cancelOrder` instant action | cancel the Nav 2 action goal |
| `startPause`/`stopPause` | cancel/resume goal or velocity smoother gate |
| `pick`/`drop`/`finePositioning` | custom behavior / action server (your hardware) |
| `state.mobileRobotPosition` | `/amcl_pose` (`PoseWithCovarianceStamped`) |
| `state.velocity` | `/odom` (`nav_msgs/Odometry`) |
| `state.powerSupply` | `/battery_state` (`sensor_msgs/BatteryState`) |
| `state.errors` / `information` | `/diagnostics` (`DiagnosticArray`) |
| `state.driving` / `paused` | derived from goal status / pause flag |
| `state.safetyState` | E-stop topic + `/scan` proximity |
| `initializePosition` instant action | publish `/initialpose` to AMCL |

## QoS alignment

VDA 5050 prescribes MQTT QoS, which maps onto ROS 2 QoS at the bridge
edge (see `rules/ros2_communication.md`):

| Data | MQTT QoS | ROS 2 QoS |
|------|----------|-----------|
| `order` / `instantActions` (commands) | 0 | RELIABLE, depth 10 |
| `state` / `visualization` (telemetry) | 0 | BEST_EFFORT, depth 5 |
| `connection` | 1 | RELIABLE, TRANSIENT_LOCAL |

## Checklist for a new connector

1. Define domain entities mirroring the 8 message schemas — **no** MQTT
   / JSON / `*_msgs` imports.
2. Define `MqttClientInterface` + `NavigationAdapterInterface` ports.
3. Put graph traversal, order stitching, action queue and the
   `actionStatus` state machine in **application** services.
4. Implement adapters in **infrastructure**; validate every inbound and
   outbound JSON against `json_schemas/*.schema`.
5. Set the `connection` last will to `CONNECTION_BROKEN`; publish
   `ONLINE` on connect.
6. Publish `state` on a timer (default ~1 Hz) **and** on every event
   (node reached, action status change); `visualization` faster (~10 Hz).
7. Send `factsheet` on connect and on `factsheetRequest`.
8. Test: unit-test entities + handlers without ROS; integration-test the
   adapters with `launch_testing` + a local MQTT broker (see
   `rules/testing.md`).

## Common pitfalls

- **Domain importing `paho`, `json`, or `rclpy`** → it's not domain. Move
  it to an adapter behind a port.
- **Mutating the base** — base nodes/edges (`released=true`) are
  committed; only extend the horizon or send an order update.
- **Order update first node ≠ `lastNodeId`** → must be rejected with an
  `error` carrying the `orderId`/`orderUpdateId`.
- **Forgetting `headerId` per-topic increment** or non-UTC timestamps.
- **Reporting `loads: []` when the load state is unknown** — omit the
  field entirely instead (empty array means *confirmed unloaded*).
- **Schema drift** — always re-validate against the v3.0.0 schemas in
  `~/nav2_ws/src/VDA5050/json_schemas/`; the VDA PDF is authoritative if
  they ever differ.
