---
name: nav2-planners
description: Reference for the Nav 2 Planner Server and its global planner plugins — NavFn, the SMAC suite (2D, Hybrid-A*, Lattice), Theta*, and the graph-based Route Server — with a comparison table. Trigger when the user selects, configures, or tunes a Nav 2 global planner.
---

# Nav2 Planner Plugins

Source: `~/nav2_ws/src/navigation2/`

## Planner Server

**Package:** `nav2_planner`
**Actions:** `ComputePathToPose`, `ComputePathThroughPoses`
**Plugin Base:** `nav2_core::GlobalPlanner`
**Service:** `is_path_valid` - Validate paths against current costmap

---

## 1. NavFn Planner

**Package:** `nav2_navfn_planner`
**Plugin:** `nav2_navfn_planner::NavfnPlanner`
**Algorithm:** Dijkstra / A* navigation function
**Best for:** General purpose, circular robots

### How It Works
1. Builds navigation potential field from goal (Dijkstra or A*)
2. Extracts path by gradient descent from start
3. Smooths approach near goal to remove discretization artifacts

### Key Parameters

```yaml
GridBased:
  plugin: "nav2_navfn_planner::NavfnPlanner"
  tolerance: 0.5              # Goal relaxation (meters) when obstructed
  use_astar: true             # A* (true) vs Dijkstra (false)
  allow_unknown: true         # Plan through unknown space
  use_final_approach_orientation: false  # Use goal orientation
```

### Cost Mapping
- `COST_NEUTRAL = 50`, `COST_FACTOR = 0.8`
- Lethal: 254, Unknown: 255, Max passable: 252
- `POT_HIGH = 1.0e10` (unassigned potential)

---

## 2. SMAC Planner Suite

**Package:** `nav2_smac_planner`
**Three plugins** with shared templated A* core.

### 2a. SmacPlanner2D - Grid A*

**Plugin:** `nav2_smac_planner::SmacPlanner2D`
**Algorithm:** 2D A* (8-connected or 4-connected)
**Best for:** Circular robots, open spaces

```yaml
GridBased:
  plugin: "nav2_smac_planner::SmacPlanner2D"
  tolerance: 0.125
  allow_unknown: true
  max_iterations: 1000000
  max_on_approach_iterations: 1000
  max_planning_time: 5.0
  cost_travel_multiplier: 2.0
  downsample_costmap: false
  smooth_path: true
```

### 2b. SmacPlannerHybrid - Hybrid-A* (SE2)

**Plugin:** `nav2_smac_planner::SmacPlannerHybrid`
**Algorithm:** Hybrid-A* with SE2 search (x, y, theta)
**Best for:** Ackermann, car-like, legged, non-circular robots

**Motion Models:**
- `DUBIN` - Forward only (symmetric)
- `REEDS_SHEPP` - Forward + reverse

**Key Features:**
- Kinematically feasible paths
- Analytic expansion on goal approach
- Multi-resolution search (costmap downsampling)
- Obstacle heuristic with dynamic programming
- Angle quantization (default 72 bins)

```yaml
GridBased:
  plugin: "nav2_smac_planner::SmacPlannerHybrid"
  tolerance: 0.25
  allow_unknown: true
  max_iterations: 1000000
  max_on_approach_iterations: 1000
  max_planning_time: 5.0
  angle_quantization_bins: 72
  minimum_turning_radius: 0.40
  motion_model_for_search: "REEDS_SHEPP"
  cost_travel_multiplier: 2.0
  # Penalties
  reverse_penalty: 2.0          # Reeds-Shepp only
  change_penalty: 0.0           # Direction change
  non_straight_penalty: 1.2     # Non-straight motion
  cost_penalty: 2.0             # Obstacle proximity
  retrospective_penalty: 0.015  # Prefer later maneuvers
  # Analytic expansion
  analytic_expansion_ratio: 3.5
  analytic_expansion_max_length: 3.0
  analytic_expansion_max_cost: 200
  # Performance
  cache_obstacle_heuristic: true  # 40x speedup between replans
  downsample_costmap: false
  smooth_path: true
  debug_visualizations: false
```

### 2c. SmacPlannerLattice - State Lattice

**Plugin:** `nav2_smac_planner::SmacPlannerLattice`
**Algorithm:** State lattice with configurable motion primitives
**Best for:** Arbitrary shaped robots, full drivetrain capabilities

**Provided Control Sets:**
- Ackermann, Legged, Differential, Omnidirectional

```yaml
GridBased:
  plugin: "nav2_smac_planner::SmacPlannerLattice"
  tolerance: 0.25
  allow_unknown: true
  max_iterations: 1000000
  max_planning_time: 5.0
  lattice_filepath: ""  # Path to lattice primitives file
  rotation_penalty: 5.0
  # Same penalty/expansion params as Hybrid
```

### Shared Smoother Parameters (all SMAC)

```yaml
  smoother:
    max_iterations: 1000
    w_smooth: 0.3
    w_data: 0.2
    tolerance: 1.0e-10
    do_refinement: true
```

### Performance Benchmarks (75m path)
- NavFn: 146ms (with path artifacts)
- Smac 2D: 243ms
- Smac Hybrid-A*: 144ms (2-20ms on small maps)
- Smac Lattice: 113ms

---

## 3. Theta* Planner

**Package:** `nav2_theta_star_planner`
**Plugin:** `nav2_theta_star_planner::ThetaStarPlanner`
**Algorithm:** Lazy Theta* P (any-angle planning)
**Best for:** Smooth paths, smaller robots

### How It Works
A* with line-of-sight (LOS) checks. When parent can see neighbor directly, connects them bypassing intermediate nodes. Produces naturally smooth paths without post-processing.

### Cost Function
```
g(neigh) = g(curr) + w_euc_cost * euclidean_dist +
           w_traversal_cost * (costmap_cost / LETHAL)^2
h(neigh) = w_heuristic_cost * euclidean_dist(neigh, goal)
f = g + h
```
When LOS succeeds: `g(neigh) = g(parent)` instead of `g(curr)`.

### Key Parameters

```yaml
GridBased:
  plugin: "nav2_theta_star_planner::ThetaStarPlanner"
  how_many_corners: 8          # 4 or 8 connected
  w_euc_cost: 1.0              # Path length weight (tautness)
  w_traversal_cost: 2.0        # Costmap traversal weight
  allow_unknown: true
  terminal_checking_interval: 5000
  use_final_approach_orientation: false
```

### Tuning Tips
- Increase `w_traversal_cost` → paths center in free space (more expansions)
- Increase `w_euc_cost` → taut, straight paths
- Use gentle inflation (cost_scaling_factor: 10.0) for best results
- ~46ms average for 87.5m path

---

## 4. Route Server (Graph-Based)

**Package:** `nav2_route`
**Actions:** `ComputeRoute`, `ComputeAndTrackRoute`
**Service:** `set_route_graph` (swap graphs at runtime)

### Architecture
Not a planner plugin - standalone server for pre-defined navigation graphs.

**Components:**
- `RoutePlanner` - Dijkstra's on graph with KD-tree nearest-neighbor
- `RouteTracker` - Real-time route execution monitoring
- `EdgeScorer` plugins - Cost functions for edges
- `RouteOperation` plugins - Actions at nodes/edges (doors, speed limits)
- `GraphFileLoader` - GeoJSON graph format (default)

### Use Cases
- Structured environments (warehouses, hospitals)
- Multi-floor navigation
- Semantic routing (speed zones, one-way lanes)
- Long-distance routing with free-space planner for first/last mile

### Key Parameters

```yaml
route_server:
  ros__parameters:
    base_frame: "base_link"
    route_frame: "map"
    path_density: 0.05              # Points per meter in dense path
    max_planning_time: 5.0
    smooth_corners: true
    smoothing_radius: 0.5
    graph_filepath: "path/to/graph.geojson"
    graph_file_loader: "nav2_route::GeoJsonGraphFileLoader"
    edge_cost_functions: ["distance_scorer"]
    operations: ["adjust_speed_limit"]
    radius_to_achieve_node: 0.5
    enable_nn_search: true
```

### Performance
- 100 nodes: ~0.003ms
- 10K nodes: ~0.23ms
- 90K nodes: ~3.75ms
- 1M nodes: ~44ms

---

## Comparison Table

| Feature | NavFn | Smac 2D | Smac Hybrid | Smac Lattice | Theta* | Route |
|---------|-------|---------|-------------|--------------|--------|-------|
| **Algorithm** | Dijkstra/A* | Grid A* | Hybrid-A* SE2 | State Lattice | A*+LOS | Graph Dijkstra |
| **Robot Type** | Circular | Circular | Non-circular | Any shape | Circular | Any |
| **Kinematic** | No | No | Yes | Yes | No | N/A |
| **Reverse** | N/A | N/A | Reeds-Shepp | Yes | N/A | N/A |
| **Speed** | ~146ms | ~243ms | ~144ms | ~113ms | ~46ms | <<1ms |
| **Path Quality** | Good* | Better | Best | Best | Good (smooth) | Pre-defined |
| **Smoothing** | Auto | Yes | Yes | Limited | Auto | Optional |
| **Multi-res** | No | Yes | Yes | Yes | No | N/A |

*NavFn can have path discontinuity artifacts
