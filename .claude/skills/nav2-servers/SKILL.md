---
name: nav2-servers
description: Reference for the Nav 2 server nodes — Lifecycle Manager, Velocity Smoother, Collision Monitor, Smoother Server, Waypoint Follower, Docking and Following servers, the Simple Commander Python API, and the Loopback Simulator. Trigger when the user configures, launches, or scripts the Nav 2 stack.
---

# Nav2 Server Nodes

Source: `~/nav2_ws/src/navigation2/`

## Lifecycle Manager

**Package:** `nav2_lifecycle_manager`
**Purpose:** Orchestrate lifecycle transitions of all Nav2 nodes

### Architecture
Manages state machine: `UNCONFIGURED → INACTIVE → ACTIVE → FINALIZED`
Uses bond connections for health monitoring. Auto-reconnects on failure.

### Key Parameters
```yaml
lifecycle_manager:
  ros__parameters:
    autostart: true
    bond_timeout: 4.0
    bond_heartbeat_period: 0.5
    service_timeout: 5.0
    attempt_respawn_reconnection: true
    node_names:
      - "map_server"
      - "amcl"
      - "controller_server"
      - "planner_server"
      - "smoother_server"
      - "behavior_server"
      - "bt_navigator"
      - "waypoint_follower"
      - "velocity_smoother"
      - "collision_monitor"
```

### Service
- `manage_nodes` (nav2_msgs/ManageLifecycleNodes)
  - Commands: STARTUP(0), PAUSE(1), RESUME(2), RESET(3), SHUTDOWN(4), CONFIGURE(5), CLEANUP(6)

---

## Velocity Smoother

**Package:** `nav2_velocity_smoother`
**Purpose:** Apply acceleration/deceleration limits to velocity commands

### Architecture
- Subscribes to raw `cmd_vel` (from controller)
- Publishes smoothed `cmd_vel_smoothed`
- Enforces kinematic limits on linear and angular velocities

### Key Parameters
```yaml
velocity_smoother:
  ros__parameters:
    smoothing_frequency: 20.0
    scale_velocities: false
    feedback: "OPEN_LOOP"           # or "CLOSED_LOOP" (uses odom)
    max_velocity: [0.26, 0.0, 1.0]   # [vx, vy, wz]
    min_velocity: [-0.26, 0.0, -1.0]
    max_accel: [2.5, 0.0, 3.2]
    max_decel: [-2.5, 0.0, -3.2]
    deadband_velocity: [0.0, 0.0, 0.0]
    velocity_timeout: 1.0
    odom_topic: "odom"
    odom_duration: 0.1
```

---

## Collision Monitor

**Package:** `nav2_collision_monitor`
**Purpose:** Real-time collision avoidance between controller and hardware

### Architecture
Intercepts `cmd_vel` → applies safety actions → outputs safe `cmd_vel`.
Monitors configurable polygons/circles around robot.

**Action Types:**
| Type | Behavior |
|------|----------|
| `stop` | Zero velocity on detection |
| `slowdown` | Reduce velocity by factor |
| `limit` | Cap max velocity |
| `approach` | Slow proportionally to distance |

**Polygon Sources:**
- `Polygon` - Static polygon shape
- `Circle` - Circular zone
- `VelocityPolygon` - Dynamic polygon based on current velocity

**Data Sources:**
- `Scan` - LaserScan
- `PointCloud` - PointCloud2
- `Range` - Single range sensor
- `Costmap` - Costmap data

### Key Parameters
```yaml
collision_monitor:
  ros__parameters:
    cmd_vel_in_topic: "cmd_vel_smoothed"
    cmd_vel_out_topic: "cmd_vel"
    transform_tolerance: 0.5
    source_timeout: 5.0
    stop_pub_timeout: 2.0
    polygons: ["FootprintApproach"]
    FootprintApproach:
      type: "polygon"
      action_type: "approach"
      footprint_topic: "/local_costmap/published_footprint"
      time_before_collision: 2.0
      simulation_time_step: 0.02
      max_points: 5
      enabled: true
    observation_sources: ["scan"]
    scan:
      type: "scan"
      topic: "/scan"
```

### Collision Detector
Separate node (`CollisionDetector`) that only detects, doesn't modify commands. Publishes `CollisionDetectorState`.

---

## Smoother Server

**Package:** `nav2_smoother`
**Action:** `SmoothPath`
**Purpose:** Host path smoother plugins

### Smoother Plugins
| Plugin | Package | Algorithm |
|--------|---------|-----------|
| `SimpleSmoother` | `nav2_smoother` | Basic iterative smoothing |
| `SavitzkyGolaySmoother` | `nav2_smoother` | Savitzky-Golay filter |
| `ConstrainedSmoother` | `nav2_constrained_smoother` | Convex optimization with constraints |

```yaml
smoother_server:
  ros__parameters:
    smoother_plugins: ["simple_smoother"]
    simple_smoother:
      plugin: "nav2_smoother::SimpleSmoother"
      tolerance: 1.0e-10
      max_its: 1000
      do_refinement: true
```

### Constrained Smoother
```yaml
    constrained_smoother:
      plugin: "nav2_constrained_smoother::ConstrainedSmoother"
      tolerance: 1.0e-10
      max_its: 1000
      enforce_path_inversion: true  # For lattice planner
      do_refinement: true
      w_smooth: 0.3
      w_data: 0.2
```

---

## Waypoint Follower

**Package:** `nav2_waypoint_follower`
**Actions:** `FollowWaypoints`, `FollowGPSWaypoints`
**Purpose:** Sequential navigation through ordered waypoints

### Architecture
Uses `NavigateToPose` action internally for each waypoint. Supports plugin-based task execution at each waypoint.

### Key Parameters
```yaml
waypoint_follower:
  ros__parameters:
    loop_rate: 20
    stop_on_failure: true
    waypoint_task_executor_plugin: "wait_at_waypoint"
    wait_at_waypoint:
      plugin: "nav2_waypoint_follower::WaitAtWaypoint"
      enabled: true
      waypoint_pause_duration: 200  # ms
```

---

## Docking Server

**Package:** `nav2_docking`
**Actions:** `DockRobot`, `UndockRobot`
**Purpose:** Autonomous charging dock approach/departure

### Architecture
- Navigates to staging pose → detects dock → controlled approach → verify charging
- Plugin-based dock types (SimpleChargingDock, SimpleNonChargingDock)
- Internal controller for precise dock approach
- Pose filtering for dock detection smoothing

### Key Parameters
```yaml
docking_server:
  ros__parameters:
    dock_plugins: ["simple_charging_dock"]
    simple_charging_dock:
      plugin: "opennav_docking::SimpleChargingDock"
      docking_threshold: 0.05
      staging_x_offset: -0.7
      use_external_detection_pose: true
    docks: ["home_dock"]
    home_dock:
      type: "simple_charging_dock"
      frame: "map"
      dock_pose: [0.0, 0.0, 0.0]
    controller:
      use_collision_detection: true
      k_phi: 3.0
      k_delta: 2.0
      v_linear_min: 0.15
      v_linear_max: 0.25
    filter_coef: 0.1
    detection_timeout: 10.0
    max_retries: 3
```

---

## Following Server

**Package:** `nav2_following`
**Action:** `FollowObject`
**Purpose:** Track and follow dynamic targets

### Architecture
Similar to docking but for moving targets. Subscribes to target pose topic, applies filtering, controls approach.

```yaml
following_server:
  ros__parameters:
    fixed_frame: "odom"
    base_frame: "base_link"
    filter_coef: 0.1
    detection_timeout: 5.0
    odom_topic: "/odom"
    controller:
      use_collision_detection: false
      k_phi: 3.0
      k_delta: 2.0
```

---

## Simple Commander (Python API)

**Package:** `nav2_simple_commander`
**Class:** `BasicNavigator`
**Purpose:** High-level Python API for Nav2

### Key Methods
```python
from nav2_simple_commander.robot_navigator import BasicNavigator

navigator = BasicNavigator()

# Initialization
navigator.setInitialPose(initial_pose)
navigator.waitUntilNav2Active()

# Navigation
navigator.goToPose(goal_pose)
navigator.goThroughPoses(poses)
navigator.followWaypoints(waypoints)
navigator.followPath(path)

# Recovery
navigator.spin(spin_dist=1.57)
navigator.backup(backup_dist=0.15, backup_speed=0.025)

# Docking
navigator.dockRobot(dock_id="home_dock")
navigator.undockRobot(dock_type="simple_charging_dock")

# Planning only
path = navigator.getPath(start, goal, planner_id="GridBased")
smoothed = navigator.smoothPath(path, smoother_id="simple_smoother")

# Status
navigator.isTaskComplete()      # Returns True when done
result = navigator.getResult()  # TaskResult enum
feedback = navigator.getFeedback()

# Costmap
navigator.clearAllCostmaps()
navigator.clearLocalCostmap()
navigator.clearGlobalCostmap()

# Lifecycle
navigator.lifecycleStartup()
navigator.lifecycleShutdown()
```

### TaskResult Enum
```python
class TaskResult(Enum):
    UNKNOWN = 0
    SUCCEEDED = 1
    CANCELED = 2
    FAILED = 3
```

---

## Loopback Simulator

**Package:** `nav2_loopback_sim`
**Purpose:** Lightweight sim without physics (for testing Nav2)

Accepts `cmd_vel`, publishes:
- Odometry on `/odom`
- TF: `odom` → `base_link`
- LaserScan from occupancy grid (optional)

```yaml
loopback_simulator:
  ros__parameters:
    update_duration: 0.02
    base_frame_id: "base_link"
    odom_frame_id: "odom"
    map_frame_id: "map"
    scan_frame_id: "base_scan"
    publish_map_odom_tf: true
    publish_clock: true
```

---

## Data Flow Summary

```
cmd_vel flow:
  Controller Server → Velocity Smoother → Collision Monitor → Hardware

Localization flow:
  /scan + /map → AMCL → map→odom TF

Planning flow:
  Goal → BT Navigator → Planner Server → Smoother Server → Controller Server

Costmap flow:
  /scan + /map → Costmap Layers → Combined Costmap → Planner/Controller
```
