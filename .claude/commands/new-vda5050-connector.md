---
description: Scaffold a Clean-Architecture VDA 5050 v3.0.0 connector (MQTT/JSON ↔ Nav 2) — domain entities, ports, application handlers, MQTT + Nav 2 adapters — in the chosen code format.
argument-hint: "<package> [python|cpp] [fleet|robot]"
allowed-tools: ["Bash", "Read", "Write", "Edit"]
---

Scaffold a VDA 5050 connector / fleet bridge package.

Argument handling (`$ARGUMENTS`):
1. `<package>` — new package name (required), created under `src/`.
2. `[lang]` — `python` (pydantic idiom) or `cpp` (struct + nlohmann
   idiom). Default `python`.
3. `[side]` — `robot` (AGV side, default) or `fleet` (master control).

If anything is missing, ask before scaffolding.

Process:
1. Read the skill `vda5050-integration`, the format guide
   `vda5050_implementation_formats.md`, and the spec
   `vda5050_messages.md`. Pick the matching reference impl to mirror:
   * python + fleet → `~/nav2_ws/src/isaac_mission_dispatch`
   * python/cpp + robot (ROS 2) → `~/nav2_ws/src/isaac_ros_cloud_control`
     and the template's `~/nav2_ws/src/vda5050_connector`
   * cpp core → `~/nav2_ws/src/vda5050_core`
2. Scaffold the Clean Architecture layout (see `clean_architecture.md`):
   * **domain/entities** — message models for `order`, `state`,
     `action`, `connection`, `factsheet` (pydantic camelCase, or C++
     struct + snake_case). **No MQTT / no `rclpy` / no `*_msgs`.**
   * **domain/interfaces** — ports: `MqttClientInterface`,
     `NavigationAdapterInterface`.
   * **application** — `OrderHandler` (graph traversal, base/horizon,
     stitching), `StateAssembler`, `InstantActionHandler` (actionStatus
     state machine).
   * **infrastructure** — `MqttBridge` (paho / PahoMqttCpp; topic
     `vda5050/v3/<mfr>/<serial>/<topic>`, QoS 0 except `connection` QoS 1
     + `CONNECTION_BROKEN` last will; JSON validated against
     `~/nav2_ws/src/VDA5050/json_schemas/`), `Nav2Adapter`
     (order→`NavigateThroughPoses`, telemetry from `/amcl_pose`,`/odom`,
     `/battery_state`), composition-root node.
3. Use **v3.0.0 enum spellings** (`CONNECTION_BROKEN`, `TEACH_IN`,
   `SINGLE`, `HIBERNATING`, `RETRIABLE`) — not the v2 spellings in the
   reference repos (see the v2→v3 table in
   `vda5050_implementation_formats.md`). `version = "3.0.0"`.
4. Wire build files (`setup.py`/`package.xml` for Python;
   `CMakeLists.txt`/`package.xml` for C++) and a launch + params file.
5. Add tests: domain entities + handlers without ROS/broker; one
   round-trip (order JSON → Nav 2 goal; telemetry → state JSON validated
   against the schema).
6. `pre-commit run --files <touched files>`.
7. Print: files created, the layer map, and how to run it against a local
   MQTT broker.

Keep the protocol out of the domain (it is an infrastructure concern),
never mutate a released base, and increment `headerId` per topic.
