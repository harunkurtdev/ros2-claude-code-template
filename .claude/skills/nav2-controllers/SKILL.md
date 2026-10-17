---
name: nav2-controllers
description: Reference for the Nav 2 Controller Server and its controller plugins — DWB and its critics, MPPI and its critics, Regulated Pure Pursuit, Graceful, and Rotation Shim — with parameters and a comparison matrix. Trigger when the user selects, configures, or tunes a Nav 2 local controller.
---

# Nav2 Controller Plugins

Source: `~/nav2_ws/src/navigation2/`

## Controller Server

**Package:** `nav2_controller`
**Action:** `FollowPath` (nav2_msgs::action::FollowPath)
**Plugin Base:** `nav2_core::Controller`

Hosts controller plugins, progress checkers, goal checkers, and path handlers. Runs at `controller_frequency` (default 20Hz).

---

## 1. DWB Controller (Dynamic Window Based)

**Package:** `nav2_dwb_controller`
**Plugin:** `dwb_core::DWBLocalPlanner`
**Best for:** General purpose differential-drive robots

### Architecture
`DWBLocalPlanner` → `StandardTrajectoryGenerator` + `TrajectoryCritics`

Samples velocity space, generates trajectory candidates, scores with critics, picks best.

### 13 Critic Plugins

| Critic | Purpose |
|--------|---------|
| `PathDist` | Distance from trajectory to global path |
| `GoalDist` | Distance from trajectory endpoint to goal |
| `PathAlign` | Alignment of trajectory heading with path direction |
| `GoalAlign` | Alignment with goal orientation |
| `BaseObstacle` | Obstacle avoidance using robot center |
| `FootprintObstacle` | Obstacle avoidance using full footprint |
| `ObstacleFootprint` | Alternative footprint obstacle scorer |
| `RotateToGoal` | Penalize non-rotation when near goal |
| `Oscillation` | Penalize oscillating commands |
| `PreferForward` | Prefer forward motion |
| `Twirling` | Penalize unnecessary rotation |
| `MapGrid` | Grid-based cost scoring (base for Path/GoalDist) |
| `StandardTrajectoryGenerator` | Velocity sample generation |

### Key Parameters

```yaml
FollowPath:
  plugin: "dwb_core::DWBLocalPlanner"
  # Velocity limits
  min_vel_x: 0.0
  max_vel_x: 0.26
  min_vel_y: 0.0
  max_vel_y: 0.0
  max_vel_theta: 1.0
  min_speed_xy: 0.0
  max_speed_xy: 0.26
  # Acceleration limits
  acc_lim_x: 2.5
  acc_lim_y: 0.0
  acc_lim_theta: 3.2
  decel_lim_x: -2.5
  decel_lim_y: 0.0
  decel_lim_theta: -3.2
  # Sampling
  vx_samples: 20
  vy_samples: 5
  vtheta_samples: 20
  sim_time: 1.7
  linear_granularity: 0.05
  angular_granularity: 0.025
  # Critics
  critics: ["RotateToGoal","Oscillation","BaseObstacle","GoalAlign","PathAlign","PathDist","GoalDist"]
```

---

## 2. MPPI Controller (Model Predictive Path Integral)

**Package:** `nav2_mppi_controller`
**Plugin:** `nav2_mppi_controller::MPPIController`
**Best for:** Complex environments, smooth paths, high-performance

### Architecture
`MPPIController` → `Optimizer` + `CriticManager` + `MotionModel`

Stochastic optimal control: samples thousands of trajectories, scores with critics, selects optimal.

### Motion Models
- `DiffDrive` - Differential drive
- `Omnidirectional` - Holonomic
- `Ackermann` - Car-like

### 11 Critic Plugins

| Critic | Purpose |
|--------|---------|
| `ConstraintCritic` | Enforce kinematic constraints |
| `CostCritic` | Costmap-based scoring |
| `GoalCritic` | Distance to goal |
| `GoalAngleCritic` | Orientation at goal |
| `PathAlignCritic` | Alignment with global path |
| `PathAngleCritic` | Heading angle relative to path |
| `PathFollowCritic` | Distance from path |
| `ObstaclesCritic` | Obstacle avoidance |
| `PreferForwardCritic` | Forward motion preference |
| `TwirlingCritic` | Rotation minimization |
| `VelocityDeadbandCritic` | Avoid deadband velocities |

### Key Parameters

```yaml
FollowPath:
  plugin: "nav2_mppi_controller::MPPIController"
  time_steps: 56
  model_dt: 0.05
  batch_size: 2000
  ax_max: 3.0
  ax_min: -3.0
  ay_max: 3.0
  az_max: 3.5
  vx_std: 0.2
  vy_std: 0.2
  wz_std: 0.4
  vx_max: 0.5
  vx_min: -0.35
  vy_max: 0.5
  wz_max: 1.9
  iteration_count: 1
  temperature: 0.3
  gamma: 0.015
  motion_model: "DiffDrive"
  prune_distance: 1.7
  enforce_path_inversion: false
  critics: [...]
  # Per-critic weights
  PathAlignCritic:
    enabled: true
    cost_weight: 14.0
    threshold_to_consider: 0.5
  GoalCritic:
    enabled: true
    cost_weight: 5.0
    threshold_to_consider: 1.4
```

---

## 3. Regulated Pure Pursuit (RPP)

**Package:** `nav2_regulated_pure_pursuit_controller`
**Plugin:** `nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController`
**Best for:** Basic path tracking, low computational cost

### Architecture
`RegulatedPurePursuitController` → `CollisionChecker` + `RegulationFunctions`

Pure pursuit with velocity regulation based on curvature, obstacles, and goal proximity.

### Key Features
- Velocity-scaled or fixed lookahead distance
- Curvature-based speed regulation
- Cost-regulated linear velocity
- Approach velocity scaling near goal
- Collision detection with footprint
- Rotation to path/goal heading

### Key Parameters (30+)

```yaml
FollowPath:
  plugin: "nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController"
  desired_linear_vel: 0.5
  lookahead_dist: 0.6
  min_lookahead_dist: 0.3
  max_lookahead_dist: 0.9
  lookahead_time: 1.5
  rotate_to_heading_angular_vel: 1.8
  use_velocity_scaled_lookahead_dist: false
  min_approach_linear_velocity: 0.05
  approach_velocity_scaling_dist: 1.0
  use_collision_detection: true
  max_allowed_time_to_collision_up_to_carrot: 1.0
  use_regulated_linear_velocity_scaling: true
  use_cost_regulated_linear_velocity_scaling: false
  regulated_linear_scaling_min_radius: 0.9
  regulated_linear_scaling_min_speed: 0.25
  allow_reversing: false
  max_angular_accel: 3.2
  rotate_to_heading_min_angle: 0.785
```

---

## 4. Graceful Controller

**Package:** `nav2_graceful_controller`
**Plugin:** `nav2_graceful_controller::GracefulController`
**Best for:** Tight spaces, smooth continuous motions

### Architecture
`GracefulController` → `EgoPolarCoords` + `SmoothControlLaw`

Uses ego-polar coordinate system for mathematically guaranteed smooth convergence.

### Key Parameters

```yaml
FollowPath:
  plugin: "nav2_graceful_controller::GracefulController"
  # Control law gains
  k_phi: 2.0
  k_delta: 1.0
  beta: 0.4
  lambda: 2.0
  # Velocity limits
  v_linear_min: 0.1
  v_linear_max: 0.5
  v_angular_max: 1.0
  # Behavior
  slowdown_radius: 1.5
  initial_rotation: true
  final_rotation: true
  allow_backward: false
```

---

## 5. Rotation Shim Controller

**Package:** `nav2_rotation_shim_controller`
**Plugin:** `nav2_rotation_shim_controller::RotationShimController`
**Best for:** Wrapper to add in-place rotation before any controller

### Architecture
Middleware that wraps a primary controller. Handles initial rotation alignment before delegating forward motion.

State Machine: Alignment Check → Rotation → Forward (primary controller) → Optional final rotation

### Key Parameters

```yaml
FollowPath:
  plugin: "nav2_rotation_shim_controller::RotationShimController"
  primary_controller: "dwb_core::DWBLocalPlanner"  # or any other
  forward_sampling_distance: 0.5
  angular_dist_threshold: 0.785  # ~45 degrees
  rotate_to_heading_angular_vel: 1.8
  max_angular_accel: 3.2
  rotate_to_goal_heading: false
```

---

## Comparison Matrix

| Feature | DWB | MPPI | RPP | Graceful | Rotation Shim |
|---------|-----|------|-----|----------|---------------|
| **Approach** | Sampling+Scoring | Stochastic Optimal | Pure Pursuit | Ego-Polar Law | Middleware |
| **Complexity** | Medium | High | Low | Medium | Low |
| **Computation** | Fast | High (GPU-like) | Very Fast | Fast | Very Fast |
| **Smoothness** | Good | Excellent | Good | Excellent | Depends |
| **Critics** | 13 plugins | 11 plugins | Integrated | Integrated | N/A |
| **Motion Models** | 1 | 3 (Diff/Omni/Ack) | 1 | 1 | Delegates |
| **Omnidirectional** | Yes | Yes | No | No | No |
| **Reversing** | Limited | Yes | Optional | Optional | No |
