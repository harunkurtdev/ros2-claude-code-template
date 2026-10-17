# VDA 5050 — Implementation Format Analysis (for code generation)

When generating VDA 5050 code, **match one of the three proven idioms
below** instead of inventing a representation. Each is taken from a real
reference implementation cloned next to this workspace. Pick by language
and role, then follow that idiom's field-naming, enum, serialization and
MQTT conventions exactly.

The wire format is always **VDA 5050 JSON over MQTT** (see
`vda5050_protocol.md` / `vda5050_messages.md`). What differs between
implementations is how the *in-memory model* maps to that JSON.

## Reference implementations

| Repo (in `~/nav2_ws/src/`) | Role | Lang | Spec ver | Model format |
|----------------------------|------|------|----------|--------------|
| `isaac_mission_dispatch` | fleet / master-control | Python | v2.x | **pydantic** BaseModel, camelCase fields |
| `isaac_ros_cloud_control` | robot / AGV (ROS 2) | C++ + Py | v2.x | **ROS 2 `.msg`** (snake_case) + MQTT bridge that case-converts |
| `vda5050_core` (ros-industrial) | protocol-agnostic core (both sides) | C++ | v1.3.x | **plain structs** + `nlohmann::json` `to_json/from_json` |

> ⚠️ **Version caveat:** all three are **v2.x / v1.x**. This template's
> spec rules (`vda5050_messages.md`) target **v3.0.0**. Reuse their
> *structure and idioms*, but use **v3.0.0 enum spellings & fields** when
> generating (see the "v2 → v3" table at the bottom). Don't copy
> `CONNECTIONBROKEN`/`TEACHIN` verbatim.

---

## Format A — Python / pydantic (fleet side)

*Source: `isaac_mission_dispatch/.../mission/vda5050_types/vda5050_types.py`.*

Fields are **camelCase = the JSON keys**, so serialization is just
`.json()` — no alias layer.

```python
import enum, pydantic

class VDA5050ActionStatus(str, enum.Enum):     # str-Enum → JSON value == name
    WAITING = "WAITING"; INITIALIZING = "INITIALIZING"; RUNNING = "RUNNING"
    PAUSED = "PAUSED"; FINISHED = "FINISHED"; FAILED = "FAILED"

class VDA5050Order(pydantic.BaseModel):
    headerId: int = 0
    timestamp: str = ""
    version: str = "3.0.0"          # ← use 3.0.0, not the repo's 2.0.0
    manufacturer: str = ""
    serialNumber: str = ""
    orderId: str
    orderUpdateId: int
    nodes: list[VDA5050Node]
    edges: list[VDA5050Edge]

    @pydantic.validator("edges")     # graph invariants live on the model
    def _edge_count(cls, edges, values):
        if len(edges) != max(len(values.get("nodes", [])) - 1, 0):
            raise ValueError("edges must be exactly nodes-1")
        return edges
```

- Serialize: `order.json()` → camelCase JSON. Deserialize:
  `VDA5050State(**json.loads(payload))` (pydantic validates on construct).
- Enums: `class X(str, enum.Enum)` so the value *is* the JSON string; add
  helper props (`@property done`) on the enum.
- Transport: `paho.mqtt.client`; topic
  `f"{prefix}/{robot}/{topic}"`; `headerId` incremented per send;
  `timestamp = datetime.now(timezone.utc).isoformat()`.
- **Generate this when** writing fleet/dispatch logic or any pure-Python
  VDA 5050 service. Also matches the template's `vda5050_connector`
  domain-entity style (dataclasses are fine too — keep camelCase mapping
  in the adapter).

---

## Format B — ROS 2 `.msg` + MQTT bridge (robot side)

*Source: `isaac_ros_cloud_control/vda5050_msgs` + `isaac_ros_mqtt_bridge`.*

ROS message fields are **snake_case** (ROS convention); a bridge node
converts between snake_case and JSON camelCase at the MQTT edge.

```
# vda5050_msgs/msg/Order.msg
int32 header_id            # ↔ JSON headerId
string timestamp
string version
string manufacturer
string serial_number       # ↔ JSON serialNumber
string order_id            # ↔ JSON orderId
uint32 order_update_id
vda5050_msgs/Node[] nodes
vda5050_msgs/Edge[] edges
```

Bridge conversion (Python, `paho-mqtt` + `rosbridge_library`):

```python
# MQTT(JSON,camelCase) → ROS(snake_case)
d = json.loads(payload)
d = convert_dict_keys(d, "camel_to_snake")          # orderId → order_id
ros_msg = ros_loader.get_message_instance("vda5050_msgs/Order")
message_conversion.populate_instance(d, ros_msg)

# ROS(snake_case) → MQTT(JSON, dromedary camelCase)
d = message_conversion.extract_values(ros_msg)
d = convert_dict_keys(d, "snake_to_dromedary")      # order_id → orderId
mqtt.publish(f"{prefix}/state", json.dumps(d))
```

- `convert_camel_to_snake`: `re.sub(r'(?<!^)(?=[A-Z])','_',s).lower()`;
  `snake_to_dromedary` for the reverse (first letter lowercase).
- MQTT topic: `f"{interface}/{major}/{manufacturer}/{serial}/{topic}"`,
  **QoS 1**, last-will on `…/connection` with state `CONNECTION_BROKEN`.
- Actions dispatched via **pluginlib**: a `Vda5050ActionHandlerBase`
  (`Initialize/Execute/Cancel/Pause/Resume`) with concrete plugins
  (docking, pick/place…) calling Nav 2 actions; status reported via
  `client_node_->UpdateActionState(action, RUNNING|FINISHED|FAILED)`.
- **Generate this when** the robot side must expose VDA 5050 as ROS 2
  messages and integrate with Nav 2 — the same shape as the template's
  `vda5050-integration` skill (this is its closest real-world sibling).

---

## Format C — C++ structs + `nlohmann::json` (protocol core)

*Source: `vda5050_core/include/vda5050_core/types` + `json_utils`.*

Plain structs, snake_case members, `std::optional<T>` for optional fields;
free `to_json/from_json` map snake_case ↔ camelCase by hand.

```cpp
struct Action {
  std::string action_type;
  std::string action_id;
  BlockingType blocking_type;                       // enum class
  std::optional<std::string> action_description;
  std::optional<std::vector<ActionParameter>> action_parameters;
};

void to_json(nlohmann::json& j, const Action& a) {
  j["actionType"] = a.action_type;                  // snake → camel here
  j["actionId"]   = a.action_id;
  j["blockingType"] = blocking_type_traits<BlockingType>::to_string(a.blocking_type);
  if (a.action_description) j["actionDescription"] = *a.action_description;
  if (a.action_parameters)  j["actionParameters"]  = *a.action_parameters;
}
void from_json(const nlohmann::json& j, Action& a) {
  a.action_type = j.at("actionType").get<std::string>();
  a.action_id   = j.at("actionId").get<std::string>();
  a.blocking_type = blocking_type_traits<BlockingType>::from_string(
                      j.at("blockingType").get<std::string>());
  if (j.contains("actionDescription")) a.action_description = j.at("actionDescription");
}
```

- Enums: `enum class` + a `<name>_traits` struct with
  `to_string`/`from_string` (throw on invalid) — no magic numbers on the wire.
- `Header` carries `std::chrono::time_point<system_clock>` serialized to
  the ISO-8601 string via a `timestamp_traits`.
- Optional fields: only write when present (`if (opt)`), only read when
  `j.contains(...)`.
- Transport: `PahoMqttCpp`; build is `ament_cmake` with header-only
  `INTERFACE` targets for `types`/`json_utils`. Includes an event-driven
  execution engine + order-graph validator + master/agv fleet side.
- **Generate this when** writing a performant, framework-light C++ VDA
  5050 library/core not tied to ROS messages.

---

## Cross-cutting rules (all formats)

- **One field-name mapping rule:** wire JSON is **camelCase**; in-memory
  is camelCase (Python/pydantic) or snake_case (ROS `.msg`, C++). If
  snake_case, convert *only at the serialization boundary*, never leak
  camelCase into ROS/C++ identifiers or snake_case onto the wire.
- **Header on every message:** `headerId` (uint32, per-topic +1 each
  send), `timestamp` (ISO-8601 **UTC**, `…Z`), `version`, `manufacturer`,
  `serialNumber`.
- **Enums serialize to their exact spec string** — model them so the
  string is authoritative (str-Enum / enum-class+traits), never integers.
- **MQTT topic:** `interfaceName/majorVersion/manufacturer/serialNumber/topic`
  (e.g. `uagv/v2/…` in the v2 repos → `vda5050/v3/…` for v3). `connection`
  is QoS 1 with a `CONNECTION_BROKEN` last will; rest QoS 0.
- **Validation belongs on the model / boundary** (pydantic validators,
  C++ order-graph validator, schema check) — not in business logic.
- Always validate against `~/nav2_ws/src/VDA5050/json_schemas/*.schema`.

## v2 → v3 enum/field differences (use the v3 spelling when generating)

The reference repos are v2.x; **v3.0.0 differs** — do not copy these verbatim:

| Concept | v2.x in the repos | **v3.0.0 (use this)** |
|---------|-------------------|-----------------------|
| `header.version` | `"2.0.0"` / `"2.1.0"` | `"3.0.0"` |
| `connectionState` | `CONNECTIONBROKEN` (3 values) | `CONNECTION_BROKEN` **+ `HIBERNATING`** |
| `operatingMode` | `TEACHIN` | `TEACH_IN` |
| `action.blockingType` | `NONE/SOFT/HARD` | adds **`SINGLE`** |
| `actionStatus` | 6 values | adds **`RETRIABLE`** (+ `retry`/`skipRetry`) |
| Order graph | nodes/edges | adds **corridors, zones, request/response** |

For the authoritative v3.0.0 field lists and enums, always defer to
`vda5050_messages.md`.
