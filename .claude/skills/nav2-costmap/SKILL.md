---
name: nav2-costmap
description: Reference for Nav 2 Costmap2D — layered costmap architecture, cost values, the Static, Obstacle, Voxel, Inflation, Range, and Denoise layers, costmap filters, and typical configurations. Trigger when the user configures or debugs local/global costmaps or writes a costmap layer.
---

# Nav2 Costmap2D

Source: `~/nav2_ws/src/navigation2/nav2_costmap_2d/`

## Architecture

```
Costmap2DROS (LifecycleNode)
  └── LayeredCostmap
        ├── Plugin Layers (combined into master costmap)
        │   ├── StaticLayer
        │   ├── ObstacleLayer / VoxelLayer
        │   ├── InflationLayer
        │   ├── RangeSensorLayer
        │   └── DenoiseLayer
        └── Filter Layers (post-processing)
            ├── KeepoutFilter
            ├── SpeedFilter
            └── BinaryFilter
```

### Update Flow
1. `updateBounds()` on each layer → defines region to update
2. `updateCosts()` on each layer → applies changes to master grid
3. Combination method determines merge: `Overwrite`, `Max`, `MaxWithoutUnknownOverwrite`

## Cost Values

```cpp
// nav2_costmap_2d/cost_values.hpp
FREE_SPACE = 0
// 1-252: Cost gradient (higher = more expensive)
MAX_NON_OBSTACLE = 252
INSCRIBED_INFLATED_OBSTACLE = 253
LETHAL_OBSTACLE = 254
NO_INFORMATION = 255  // Unknown
```

## Core Classes

### Costmap2D
Main 2D grid. Key methods:
- `getCost(mx, my)` / `setCost(mx, my, cost)` - Cell access
- `worldToMap(wx, wy, mx, my)` / `mapToWorld(mx, my, wx, wy)` - Coordinate conversion
- `getSizeInCellsX()`, `getSizeInCellsY()`, `getResolution()`
- `setConvexPolygonCost(polygon, cost)` - Fill polygon with cost
- `updateOrigin(new_ox, new_oy)` - For rolling window

### Costmap2DROS (ROS Wrapper)
Lifecycle node managing layers, footprint, TF. Key features:
- Dynamic plugin loading via pluginlib
- Footprint publishing and updates
- Costmap publishing for visualization
- Clear costmap services

### Layer (Plugin Interface)
```cpp
class Layer {
  virtual void onInitialize() = 0;
  virtual void updateBounds(robot_x, robot_y, robot_yaw,
    min_x, min_y, max_x, max_y) = 0;
  virtual void updateCosts(master_grid, min_x, min_y, max_x, max_y) = 0;
  virtual void reset() = 0;
  virtual bool isClearable() = 0;
  virtual void activate() = 0;
  virtual void deactivate() = 0;
};
```

### FootprintCollisionChecker
```cpp
template<typename CostmapT>
class FootprintCollisionChecker {
  double footprintCost(const Footprint & footprint);
  double footprintCostAtPose(x, y, theta, footprint);
  double lineCost(x0, x1, y0, y1);
  double pointCost(x, y);
};
```

---

## Layer Plugins

### 1. StaticLayer

**Plugin:** `nav2_costmap_2d::StaticLayer`
**Purpose:** Load occupancy grid from map server (SLAM output)
**Clearable:** No

```yaml
static_layer:
  plugin: "nav2_costmap_2d::StaticLayer"
  map_subscribe_transient_local: true
  map_topic: "/map"
  subscribe_to_updates: false
  track_unknown_space: true
  use_maximum: false
  lethal_threshold: 100
  unknown_cost_value: -1
  trinary_costmap: true
```

### 2. ObstacleLayer

**Plugin:** `nav2_costmap_2d::ObstacleLayer`
**Purpose:** Process LaserScan and PointCloud2 for dynamic obstacles
**Clearable:** Yes

Maintains observation buffers for:
- **Marking:** Insert obstacles where sensor detects them
- **Clearing:** Raytrace free space between sensor and obstacle

```yaml
obstacle_layer:
  plugin: "nav2_costmap_2d::ObstacleLayer"
  enabled: true
  observation_sources: scan pointcloud
  scan:
    topic: /scan
    sensor_frame: ""
    observation_persistence: 0.0
    expected_update_rate: 0.0
    data_type: LaserScan
    min_obstacle_height: 0.0
    max_obstacle_height: 2.0
    inf_is_valid: false
    marking: true
    clearing: true
    obstacle_max_range: 2.5
    obstacle_min_range: 0.0
    raytrace_max_range: 3.0
    raytrace_min_range: 0.0
  pointcloud:
    topic: /points
    data_type: PointCloud2
    # Same params as scan...
```

### 3. VoxelLayer

**Plugin:** `nav2_costmap_2d::VoxelLayer`
**Purpose:** 3D obstacle representation (extends ObstacleLayer)
**Clearable:** Yes

Uses `nav2_voxel_grid::VoxelGrid` for 3D tracking. Projects to 2D costmap.

```yaml
voxel_layer:
  plugin: "nav2_costmap_2d::VoxelLayer"
  enabled: true
  # All ObstacleLayer params plus:
  z_voxels: 16          # Vertical resolution
  z_resolution: 0.05    # Voxel height (meters)
  origin_z: 0.0
  mark_threshold: 0     # Min voxels to mark
  unknown_threshold: 15  # Voxels for unknown
  publish_voxel_map: true
```

### 4. InflationLayer

**Plugin:** `nav2_costmap_2d::InflationLayer`
**Purpose:** Expand obstacle costs with exponential decay
**Clearable:** No

Cost function: exponential falloff from lethal obstacles.

```yaml
inflation_layer:
  plugin: "nav2_costmap_2d::InflationLayer"
  cost_scaling_factor: 3.0    # Exponential decay rate
  inflation_radius: 0.55      # Max inflation distance (meters)
  inflate_unknown: false
  inflate_around_unknown: false
```

**Cost formula:**
```
cost = INSCRIBED (253) if distance <= inscribed_radius
cost = exp(-cost_scaling_factor * (distance - inscribed_radius)) * (252 - 1) + 1
```

Higher `cost_scaling_factor` = steeper decay (costs drop faster from obstacles).

### 5. RangeSensorLayer

**Plugin:** `nav2_costmap_2d::RangeSensorLayer`
**Purpose:** IR/Sonar/range sensor integration
**Clearable:** Yes

```yaml
range_layer:
  plugin: "nav2_costmap_2d::RangeSensorLayer"
  topics: ["/sonar0", "/sonar1"]
  input_sensor_type: ALL  # or VARIABLE, FIXED
```

### 6. DenoiseLayer

**Plugin:** `nav2_costmap_2d::DenoiseLayer`
**Purpose:** Filter noise (standalone obstacle pixels or small groups)
**Clearable:** No

```yaml
denoise_layer:
  plugin: "nav2_costmap_2d::DenoiseLayer"
  enabled: true
  minimal_group_size: 2
```

---

## Costmap Filters

Applied as post-processing on combined costmap.

### KeepoutFilter
Enforces no-go zones from filter mask.

```yaml
keepout_filter:
  plugin: "nav2_costmap_2d::KeepoutFilter"
  enabled: true
  filter_info_topic: "/costmap_filter_info"
```

### SpeedFilter
Speed limitations by area from filter mask.

```yaml
speed_filter:
  plugin: "nav2_costmap_2d::SpeedFilter"
  enabled: true
  filter_info_topic: "/costmap_filter_info"
  speed_limit_topic: "/speed_limit"
```

**CostmapFilterInfo message types:**
- `type: 0` → Keepout zone
- `type: 1` → Speed limit (percentage)
- `type: 2` → Speed limit (m/s)

---

## Typical Configurations

### Local Costmap (Rolling Window)

```yaml
local_costmap:
  local_costmap:
    ros__parameters:
      update_frequency: 5.0
      publish_frequency: 2.0
      global_frame: odom
      robot_base_frame: base_link
      rolling_window: true
      width: 3
      height: 3
      resolution: 0.05
      footprint: "[[0.15,0.15],[0.15,-0.15],[-0.15,-0.15],[-0.15,0.15]]"
      # OR: robot_radius: 0.22  (for circular robots)
      plugins: ["voxel_layer", "inflation_layer"]
```

### Global Costmap (Static)

```yaml
global_costmap:
  global_costmap:
    ros__parameters:
      update_frequency: 1.0
      publish_frequency: 1.0
      global_frame: map
      robot_base_frame: base_link
      robot_radius: 0.22
      resolution: 0.05
      track_unknown_space: true
      plugins: ["static_layer", "obstacle_layer", "inflation_layer"]
```

---

## Utility Classes

### ObservationBuffer
Buffers sensor data with TF transforms. Stores point clouds with origin for marking/clearing.

### LineIterator (`nav2_util`)
Bresenham line algorithm for raytracing (clearing free space).

### FootprintCollisionChecker
Collision detection using oriented robot footprint against costmap.

### CostmapDownsampler (`nav2_smac_planner`)
Multi-resolution costmap support for planners.
