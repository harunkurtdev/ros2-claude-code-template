# VDA 5050 Protocol Reference (Version 3.0.0)

Interface for the communication between mobile robots and a fleet
control, jointly published by the **VDA** and **VDMA**.

> This file is the **conceptual overview / quick reference**. For the
> exhaustive field-by-field spec of every message and every
> communication process, see **`vda5050_messages.md`**.

- **Authoritative source:** the PDF on the VDA website is the legally
  valid version — <https://www.vda.de/vda-5050> (this reference tracks
  *VDA5050-V3.0.0-2025-03*).
- **Machine-readable spec:** `~/nav2_ws/src/VDA5050/` (git tag
  `3.0.0`, branch `main`):
  - `VDA5050_EN.md` — full prose specification.
  - `json_schemas/*.schema` — JSON Schema for every message
    (`order`, `instantActions`, `state`, `visualization`,
    `connection`, `factsheet`, `zoneSet`, `responses`).
  - `assets/` — UML / process diagrams.
- **Reference ROS 2 implementation:** `~/nav2_ws/src/vda5050_connector/`
  — a Clean-Architecture bridge that maps VDA 5050 onto Nav 2. See
  skill `vda5050-integration`.

> Semantic versioning: `[Major].[Minor].[Patch]`. Major (x.0.0) =
> breaking (new mandatory fields); Minor (3.x.0) = backward-compatible
> features; Patch (3.0.x) = corrections. This document is **3.0.0**.

---

## 1. Roles & transport

| Term | Meaning |
|------|---------|
| **Fleet control** (master control / MC) | Central system that issues orders and instant actions, consumes state. |
| **Mobile robot** (AGV / AMR) | Executes orders, publishes state, factsheet, visualization. |

- **Transport:** MQTT (**3.1.1 minimum**) carrying **JSON** payloads.
- IEEE-754 specials (`NaN`, `Infinity`) are **not** allowed. Booleans
  are `true`/`false` — never magic numbers or `'TRUE'`.
- A maximum message length is not defined by the spec; it is bounded by
  MQTT and by the limits the robot advertises in its `factsheet`
  (`protocolLimits`).

### MQTT QoS per topic

| QoS | Topics |
|-----|--------|
| **0** (best effort) | `order`, `instantActions`, `state`, `factsheet`, `zoneSet`, `responses`, `visualization` |
| **1** (at least once) | `connection` |

The MQTT **last will** is set on the `connection` topic with
`connectionState = "CONNECTION_BROKEN"` so the broker reports
ungraceful disconnects.

---

## 2. Topic structure

Suggested topic levels for a local broker (cloud brokers may have to
adapt this):

```
interfaceName/majorVersion/manufacturer/serialNumber/topic
# example:
vda5050/v3/KIT/0001/order
```

| Level | Notes |
|-------|-------|
| `interfaceName` | Interface name, e.g. `vda5050`. |
| `majorVersion`  | Major version, prefixed with `v` (e.g. `v3`). |
| `manufacturer`  | Robot manufacturer. |
| `serialNumber`  | Unique robot serial. Allowed chars: `A-Z a-z 0-9 _ . : -`. |
| `topic`         | One of the topics below. |

`/` (topic separator) and the MQTT wildcards `+`, `#`, `$` must **not**
appear inside any level.

### 2.1 Topics for communication

| Topic | Published by | Subscribed by | Impl. | Schema |
|-------|--------------|---------------|-------|--------|
| `order`         | fleet control | mobile robot | mandatory | `order.schema` |
| `instantActions`| fleet control | mobile robot | mandatory | `instantActions.schema` |
| `state`         | mobile robot  | fleet control | mandatory | `state.schema` |
| `visualization` | mobile robot  | visualization systems | optional | `visualization.schema` |
| `connection`    | broker / robot| fleet control | mandatory | `connection.schema` |
| `factsheet`     | mobile robot  | fleet control | mandatory | `factsheet.schema` |
| `zoneSet`       | fleet control | mobile robot | optional | `zoneSet.schema` |
| `responses`     | fleet control | mobile robot | optional | `responses.schema` |

---

## 3. Protocol header

Every JSON message begins with this header (the header itself is **not**
a nested object — these are top-level fields of each message):

| Field | Type | Description |
|-------|------|-------------|
| `headerId` | uint32 | Per-topic counter, incremented by 1 on every *sent* message. |
| `timestamp` | string | ISO 8601 UTC, `YYYY-MM-DDTHH:mm:ss.fffZ`. |
| `version` | string | Protocol `[Major].[Minor].[Patch]` (e.g. `3.0.0`). |
| `manufacturer` | string | Robot manufacturer. |
| `serialNumber` | string | Robot serial number. |

---

## 4. `order` (fleet → robot)

An order is a **graph** of nodes (waypoints) and edges (connections)
traversed in `sequenceId` order. Shared `sequenceId` across nodes and
edges defines the traversal sequence; an order always starts and ends on
a node, alternating node → edge → node.

| Field | Type | Notes |
|-------|------|-------|
| *header fields* | | see §3 |
| `orderId` | string | Identifies messages belonging to the same order. |
| `orderUpdateId` | uint32 | Unique per `orderId`, starts at 0; increments on update. |
| `orderDescription` | string (opt) | Human-readable, **never** for logic. |
| `nodes` | array<node> | Nodes to traverse. |
| `edges` | array<edge> | Edges to traverse. |

### Base vs. horizon

`released` partitions the graph:

- `released = true`  → **base**: committed, guaranteed-to-execute part.
  The base **cannot be changed**; the fleet must assume it is already
  executed.
- `released = false` → **horizon**: forecast that may still change; the
  robot does not commit to it.

State flags `newBaseRequest` and the `base request` mechanism (§6.6.3 in
the spec) let the robot ask for the base to be extended before it runs
out of committed path.

### node

| Field | Type | Notes |
|-------|------|-------|
| `nodeId` | string | May repeat within an order — disambiguated by `sequenceId`. |
| `sequenceId` | uint32 | Shared node/edge ordering. |
| `nodeDescriptor` | string (opt) | Human-readable. |
| `released` | bool | base / horizon. |
| `nodePosition` | object (opt) | Omit for line-guided robots. |
| `actions` | array<action> | Actions on the node (empty if none). |

**nodePosition:** `x` `y` (float64, m), `theta` (rad, `[-π,π]`),
`mapId` (string), optional `allowedDeviationXY` (ellipse `a`,`b`,`theta`)
and `allowedDeviationTheta` (`[0,π]`). `0.0` deviation = match as
precisely as technically possible.

### edge (directional, start node → end node)

| Field | Type | Notes |
|-------|------|-------|
| `edgeId` | string | May repeat; disambiguated by `sequenceId`. |
| `sequenceId` | uint32 | Shared node/edge ordering. |
| `released` | bool | base / horizon. |
| `startNodeId` / `endNodeId` | string | Connected nodes. |
| `maximumSpeed` | float64 (opt, m/s) | Speed cap on the edge. |
| `maximumMobileRobotHeight` / `minimumLoadHandlingDeviceHeight` | float64 (opt, m) | |
| `orientation` | float64 (opt, rad) | Interpreted via `orientationType`. |
| `orientationType` | enum (opt) | `GLOBAL` (omni only) \| `TANGENTIAL` (default; 0 = forward, π = backward). |
| `direction` | string (opt) | Junction direction for line-guided robots (`"left"`, `"580 Hz"`, …). |
| `reachOrientationBeforeEntering` | bool (opt) | Omni only; default `false`. |
| `maximumRotationSpeed` | float64 (opt, rad/s) | |
| `trajectory` | object (opt) | NURBS (`degree`, `knotVector`, `controlPoints`). |
| `length` | float64 (opt, m) | Used by line-guided robots to slow before a stop. |
| `corridor` | object (opt) | Deviation bounds for obstacle avoidance (v3 feature). |
| `actions` | array<action> | Active only while traversing the edge. |

---

## 5. Actions

### action object

| Field | Type | Notes |
|-------|------|-------|
| `actionType` | string | Function identifier (see predefined actions). |
| `actionId` | string | Unique; maps to `actionState` in state. Use UUIDs. |
| `actionDescriptor` | string (opt) | Human-readable. |
| `blockingType` | enum | `NONE` \| `SINGLE` \| `SOFT` \| `HARD`. |
| `actionParameters` | array<{key,value}> (opt) | Action-specific params. |
| `retriable` | bool (opt) | `true` → may enter `RETRIABLE` on failure. Default `false`. |

### blockingType semantics

| Type | Driving | Other actions |
|------|---------|---------------|
| `NONE`   | allowed | allowed |
| `SINGLE` | allowed | not allowed |
| `SOFT`   | not allowed | allowed |
| `HARD`   | not allowed | not allowed (sole action) |

Actions with `NONE`/`SOFT` may run in parallel; a `SINGLE`/`HARD` action
waits for all collected parallel actions to be `FINISHED`/`FAILED`
first. `SOFT`/`HARD` stop automatic driving until they clear.

### 5.1 Action status state machine

`actionStatus` enum and transitions (state `actionStates[]` /
`instantActionStates[]` / `zoneActionStates[]`):

```
WAITING → INITIALIZING → RUNNING → FINISHED
                          │  ↑
                          ↓  │ (retry)
                        PAUSED
                          │
                          ↓
                       FAILED ⇄ RETRIABLE   (retry → RUNNING, skipRetry → FAILED)
```

| Status | Meaning |
|--------|---------|
| `WAITING` | Received, not yet triggered (node not released / queued). |
| `INITIALIZING` | Preparing (may be skipped if instant). |
| `RUNNING` | In progress. |
| `PAUSED` | Suspended (e.g. `startPause`, safety field). |
| `FINISHED` | Completed; `resultDescription` may be set. |
| `FAILED` | Failed; if `retriable=false` this is terminal. |
| `RETRIABLE` | Failed but awaiting `retry` / `skipRetry` instant action. |

### 5.2 Predefined actions (§6.2.3)

Every robot **must** support `cancelOrder`, `startPause`, `stopPause`.
Custom actions are allowed when none of these map.

| actionType | Key parameters | Scope (instant/node/edge/zone) | Notes |
|------------|----------------|--------------------------------|-------|
| `startPause` / `stopPause` | – | instant | Pause/resume; linked state `paused`. |
| `startCharging` / `stopCharging` | – | instant, node | Linked state `powerSupply.charging`. |
| `startHibernation` / `stopHibernation` | `wakeUpTime` (opt) | instant | Enters/exits `HIBERNATING` connection state. |
| `shutdown` | – | instant | Coordinated disconnect; requires idle. |
| `initializePosition` | `x`,`y`,`theta`,`mapId`,`lastNodeId` | instant | Reset localization pose. |
| `stateRequest` | – | instant | Force a fresh `state` message. |
| `factsheetRequest` | – | instant | Force a `factsheet` message. |
| `logReport` | `reason` | instant | Generate & store a log. |
| `cancelOrder` | `orderId` (opt) | instant | Stop ASAP / next node; → idle. |
| `pick` / `drop` | `lhd`,`stationType`,`stationName`,`loadType`,`loadId`,`height`,`depth`,`side` | node, edge | Load handling; updates `loads`. |
| `detectObject` | `objectType` (opt) | node, edge, zone | |
| `finePositioning` | `stationType`,`stationName` (opt) | node, edge, zone | Exact positioning on a target. |
| `waitForTrigger` | `triggerType` [string] (`FLEET_CONTROL`/`LOCAL`/custom) | node, zone | Fleet handles timeout. |
| `retry` / `skipRetry` | `actionId` | instant | Re-run / abandon a `RETRIABLE` action. |
| `downloadMap` / `enableMap` / `deleteMap` | mapId/mapVersion | instant | Map lifecycle (§6.3). |
| `downloadZoneSet` / `enableZoneSet` / `deleteZoneSet` | zoneSetId | instant | Zone-set lifecycle (§6.4). |
| `clearInstantActions` / `clearZoneActions` | – | instant | Purge `FINISHED`/`FAILED` from the respective array. |
| `updateCertificate` | `service` (`MQTT`), `keyDownloadLink`, `certificateDownloadLink`, `certificateAuthorityDownloadLink` (opt) | instant | TLS cert rotation. |

---

## 6. `instantActions` (fleet → robot)

Header fields + `actions` array. Same `action` object as in orders, but
executed immediately regardless of order/graph progress. Reported in
`state.instantActionStates[]` until `clearInstantActions`.

---

## 7. `state` (robot → fleet)

The primary telemetry message. Key fields:

| Field | Type | Notes |
|-------|------|-------|
| *header fields* | | see §3 |
| `maps` | array<map> | `{mapId, mapVersion, mapStatus(ENABLED/DISABLED), mapDescriptor?}`. |
| `zoneSets` | array<zoneSet> | `{zoneSetId, mapId, zoneSetStatus}`. |
| `orderId` / `orderUpdateId` | string / uint32 | Current or last order; `""`/`0` if none. |
| `lastNodeId` / `lastNodeSequenceId` | string / uint32 | Last reached / current node. |
| `nodeStates` / `edgeStates` | arrays | Remaining graph to traverse (empty when idle). |
| `plannedPath` / `intermediatePath` | object (opt) | NURBS path / ETA waypoints. |
| `mobileRobotPosition` | object (opt) | `x,y,theta,mapId`; omit only for non-localizing robots. |
| `velocity` | object (opt) | Robot-frame velocity. |
| `loads` | array<load> (opt) | Omit entirely if unknown; empty array = unloaded. |
| `driving` | bool | Driving (manual or automatic). |
| `paused` | bool (opt) | Paused via button or instant action. |
| `newBaseRequest` | bool (opt) | Robot is near end of base; fleet should extend it. |
| `zoneRequests` / `edgeRequests` | arrays (opt) | Active interactive-zone / corridor requests (v3). |
| `distanceSinceLastNode` | float64 (opt, m) | Line-guided robots. |
| `actionStates` | array<actionState> | All actions of the current order. |
| `instantActionStates` | array<actionState> | Until `clearInstantActions`; may raise `INSTANT_ACTION_STATES_FULL`. |
| `zoneActionStates` | array<actionState> (opt) | Required if action zones supported. |
| `powerSupply` | object | Battery/charge info (`charging`, `batteryCharge`, …). |
| `operatingMode` | enum | see below. |
| `errors` | array<error> | Active errors; empty = none. |
| `information` | array<info> (opt) | Debug/visualization only — never for fleet logic. |
| `safetyState` | object | E-stop / field-violation status. |

`actionState`: `{actionId, actionType?, actionDescriptor?, actionStatus,
resultDescription?}`.

### operatingMode enum

`STARTUP` | `AUTOMATIC` | `SEMIAUTOMATIC` | `INTERVENED` | `MANUAL` |
`SERVICE` | `TEACH_IN`.

---

## 8. `connection` (broker/robot → fleet)

Header fields + `connectionState`:

| Value | Meaning |
|-------|---------|
| `ONLINE` | Robot–broker connection active. |
| `OFFLINE` | Coordinated disconnect. |
| `HIBERNATING` | Low-power; state messages paused, broker connection kept. |
| `CONNECTION_BROKEN` | Unexpected loss (delivered via MQTT last will). |

---

## 9. `factsheet` (robot → fleet)

Capability advertisement sent on connect / on `factsheetRequest`. Notable
blocks: `typeSpecification`, `physicalParameters`,
`protocolLimits` (`maximumMessageLength`, `maximumTopicSerialLength`,
`maximumTopicElementLength`, array/string length caps),
`protocolFeatures` (supported `agvActions` with their parameters),
`agvGeometry`, `loadSpecification`, `localizationParameters`. Full schema
in `factsheet.schema`.

---

## 10. `visualization`, `zoneSet`, `responses`

- **`visualization`** — high-frequency `mobileRobotPosition` + `velocity`
  for monitoring UIs (optional, no order logic).
- **`zoneSet`** — fleet → robot transfer of named zone sets (keepout,
  speed, interactive zones); v3 introduces interactive / corridor zones.
- **`responses`** — fleet → robot replies to in-state requests
  (request/response mechanism, §6.9), e.g. corridor / zone-access
  grants.

---

## 11. Order lifecycle (summary)

```
fleet: order(orderId, orderUpdateId=0, base+horizon)
   ├─ robot accepts → state.nodeStates/edgeStates populated
   ├─ robot traverses base, runs node/edge actions, streams state
   ├─ fleet: order(orderId, orderUpdateId+1)  ← stitch / extend base
   │     (new graph must start with the robot's lastNode as first node)
   └─ cancelOrder (instant) → robot stops ASAP/next node → idle
```

- **Order update / stitching:** an update keeps `orderId`, increments
  `orderUpdateId`, and its first node = the robot's `lastNodeId`
  (the "decision node"). The robot rejects updates whose first node does
  not match.
- **Rejection:** invalid orders/updates are reported via `errors[]`
  carrying the offending `orderId`/`orderUpdateId`.

For full prose (corridors, zones, maps, request/response) read
`~/nav2_ws/src/VDA5050/VDA5050_EN.md` §6, and validate any message
against the corresponding `json_schemas/*.schema`.
