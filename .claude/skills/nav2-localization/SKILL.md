---
name: nav2-localization
description: Reference for Nav 2 localization and maps — AMCL, Map Server, Map Saver, the Costmap Filter Info Server, the Vector Object Server, and the TF frame tree. Trigger when the user configures localization, loads or saves maps, or debugs map/odom/base_link transforms.
---

# Nav2 Localization & Map Server

Source: `~/nav2_ws/src/navigation2/nav2_amcl/` and `nav2_map_server/`

## AMCL (Adaptive Monte Carlo Localization)

**Package:** `nav2_amcl`
**Node:** `amcl`
**Purpose:** Particle filter-based localization on known map

### Architecture
- Subscribes to `/scan` (LaserScan) and `/map` (OccupancyGrid)
- Publishes `amcl_pose` (PoseWithCovarianceStamped) and `particlecloud`
- Broadcasts TF: `map` → `odom`
- Updates when robot moves beyond thresholds (`update_min_d`, `update_min_a`)

### Motion Models
| Model | Class | Robot Type |
|-------|-------|------------|
| `nav2_amcl::DifferentialMotionModel` | Default | Differential drive |
| `nav2_amcl::OmniMotionModel` | Omnidirectional | Holonomic robots |

### Sensor Models
| Model | Description |
|-------|-------------|
| `likelihood_field` | Fast, uses endpoint only (recommended) |
| `beam` | Full beam model with miss/hit/random/max |

### Key Parameters

```yaml
amcl:
  ros__parameters:
    # Robot model
    robot_model_type: "nav2_amcl::DifferentialMotionModel"

    # Motion noise (alpha1-5)
    alpha1: 0.2   # Rotation from rotation
    alpha2: 0.2   # Rotation from translation
    alpha3: 0.2   # Translation from translation
    alpha4: 0.2   # Translation from rotation
    alpha5: 0.2   # Translation noise (omni only)

    # Particle filter
    max_particles: 2000
    min_particles: 500
    pf_err: 0.05
    pf_z: 0.99
    resample_interval: 1
    recovery_alpha_slow: 0.0    # Slow average weight (0=disabled)
    recovery_alpha_fast: 0.0    # Fast average weight (0=disabled)

    # Laser sensor
    laser_model_type: "likelihood_field"
    max_beams: 60
    laser_max_range: 100.0
    laser_min_range: -1.0
    z_hit: 0.5
    z_rand: 0.5
    z_short: 0.05   # beam model
    z_max: 0.05      # beam model
    sigma_hit: 0.2

    # Update control
    update_min_d: 0.25    # Min translation before update (m)
    update_min_a: 0.2     # Min rotation before update (rad)

    # Transform
    global_frame_id: "map"
    base_frame_id: "base_footprint"
    odom_frame_id: "odom"
    transform_tolerance: 1.0
    tf_broadcast: true

    # Initial pose
    set_initial_pose: false
    initial_pose:
      x: 0.0
      y: 0.0
      z: 0.0
      yaw: 0.0
    always_reset_initial_pose: false
    first_map_only: false
```

### Services
- `reinitialize_global_localization` - Spread particles uniformly
- `request_nomotion_update` - Force update without motion

### Tuning Tips
- More particles → better accuracy, higher CPU
- Lower `update_min_d/a` → more frequent updates
- `likelihood_field` is faster than `beam` model
- Set `recovery_alpha_slow/fast` > 0 for kidnapped robot recovery
- Use `set_initial_pose: true` for known start position

---

## Map Server

**Package:** `nav2_map_server`
**Node:** `map_server`
**Purpose:** Serve static occupancy grid maps

### Key Parameters

```yaml
map_server:
  ros__parameters:
    yaml_filename: "map.yaml"
    topic_name: "map"
    frame_id: "map"
```

### Map YAML Format

```yaml
image: map.pgm
resolution: 0.05          # meters/pixel
origin: [-10.0, -10.0, 0.0]  # [x, y, yaw]
negate: 0
occupied_thresh: 0.65
free_thresh: 0.196
```

### Services
- `load_map` (nav2_msgs/LoadMap) - Load new map file
- `save_map` (nav2_msgs/SaveMap) - Save current map

---

## Map Saver

**Node:** `map_saver_server`
**Purpose:** Save maps from SLAM or other sources

```yaml
map_saver:
  ros__parameters:
    save_map_timeout: 5.0
    free_thresh_default: 0.25
    occupied_thresh_default: 0.65
    map_subscribe_transient_local: true
```

---

## Costmap Filter Info Server

**Node:** `costmap_filter_info_server`
**Purpose:** Serve filter metadata for keepout/speed zones

```yaml
costmap_filter_info_server:
  ros__parameters:
    type: 0                    # 0=keepout, 1=speed%, 2=speed_m/s
    filter_info_topic: "/costmap_filter_info"
    mask_topic: "/filter_mask"
    base: 0.0
    multiplier: 1.0
```

---

## Vector Object Server

**Node:** `vector_object_server`
**Purpose:** Dynamic polygon/circle obstacles

### Services
- `add_shapes` (nav2_msgs/AddShapes) - Add polygons/circles
- `remove_shapes` (nav2_msgs/RemoveShapes) - Remove by UUID
- `get_shapes` (nav2_msgs/GetShapes) - Query shapes

---

## TF Frame Tree

```
map (global fixed frame)
 └── odom (odometry frame, AMCL broadcasts map→odom)
      └── base_footprint / base_link
           ├── lidar_link / laser_frame
           ├── camera_link
           ├── imu_link
           └── wheel links
```

AMCL corrects odometry drift by adjusting the `map` → `odom` transform.
