---
name: nav2-behavior-tree
description: Reference for the Nav 2 BT Navigator — the 82 built-in behavior tree nodes, the 15 default BT XML trees, standard tree structure, and recovery behavior plugins. Trigger when the user designs, edits, or debugs a Nav 2 behavior tree or recovery behavior.
---

# Nav2 Behavior Tree Nodes & Behaviors

Source: `~/nav2_ws/src/navigation2/nav2_behavior_tree/` and `nav2_bt_navigator/` and `nav2_behaviors/`

## BT Navigator

**Package:** `nav2_bt_navigator`
**Actions:** `NavigateToPose`, `NavigateThroughPoses`

Orchestrates planning, control, and recovery via Behavior Tree XML files. Loads BT node plugins dynamically.

### Navigator Plugins
- `NavigateToPoseNavigator` - Single goal navigation
- `NavigateThroughPosesNavigator` - Multi-goal navigation

---

## BT Node Inventory (82 total)

### Action Nodes (47)

#### Navigation
| Node | Purpose | Key Ports |
|------|---------|-----------|
| `NavigateToPose` | Navigate to single goal | goal, behavior_tree, error_code_id |
| `NavigateThroughPoses` | Navigate through goals | goals, behavior_tree, error_code_id |
| `ComputePathToPose` | Plan to single goal | goal, start, path, planner_id, error_code_id |
| `ComputePathThroughPoses` | Plan through goals | goals, start, path, planner_id, error_code_id |
| `FollowPath` | Execute path following | path, controller_id, goal_checker_id, error_code_id |
| `SmoothPath` | Smooth a path | unsmoothed_path, smoothed_path, smoother_id, error_code_id |
| `ComputeRoute` | Graph-based route plan | goal, start, route, path, error_code_id |
| `ComputeAndTrackRoute` | Route plan + tracking | goal, start, error_code_id |

#### Path Operations
| Node | Purpose | Key Ports |
|------|---------|-----------|
| `TruncatePath` | Cut path before goal | input_path, output_path, distance |
| `TruncatePathLocal` | Cut path in local frame | input_path, output_path, distance_forward, distance_backward |
| `ConcatenatePaths` | Join two paths | path1, path2, output_path |
| `GetPoseFromPath` | Extract pose at index | path, index, pose |

#### Recovery Behaviors
| Node | Purpose | Key Ports |
|------|---------|-----------|
| `Spin` | Rotate in place | spin_dist (rad), time_allowance, error_code_id |
| `BackUp` | Reverse motion | backup_dist, backup_speed, time_allowance, error_code_id |
| `DriveOnHeading` | Move on heading | dist_to_travel, speed, time_allowance, error_code_id |
| `Wait` | Pause execution | wait_duration, error_code_id |
| `AssistedTeleop` | Collision-safe teleop | time_allowance, error_code_id |
| `FollowObject` | Follow tracked object | pose_topic, tracked_frame, error_code_id |
| `DockRobot` | Dock at station | dock_id, dock_type, error_code_id |
| `UndockRobot` | Undock from station | dock_type, error_code_id |

#### Goal Management
| Node | Purpose | Key Ports |
|------|---------|-----------|
| `AppendGoalPoseToGoals` | Add goal to vector | goal, goals |
| `RemovePassedGoals` | Remove reached goals | input_goals, output_goals, radius |
| `RemoveInCollisionGoals` | Filter collision goals | input_goals, output_goals |
| `GetNextFewGoals` | Get subset of goals | input_goals, output_goals, number |
| `ExtractRouteNodesAsGoals` | Route nodes → goals | route, goals |
| `GetCurrentPose` | Get robot's current pose | current_pose |

#### Costmap Operations
| Node | Purpose | Key Ports |
|------|---------|-----------|
| `ClearEntireCostmap` | Clear all costmap | service_name ("global" or "local") |
| `ClearCostmapExceptRegion` | Clear except area | service_name, reset_distance |
| `ClearCostmapAroundRobot` | Clear near robot | service_name, reset_distance |
| `ClearCostmapAroundPose` | Clear near pose | service_name, pose, reset_distance |

#### Plugin Selectors
| Node | Purpose | Key Ports |
|------|---------|-----------|
| `PlannerSelector` | Choose planner | topic, selected_planner |
| `ControllerSelector` | Choose controller | topic, selected_controller |
| `SmootherSelector` | Choose smoother | topic, selected_smoother |
| `GoalCheckerSelector` | Choose goal checker | topic, selected_goal_checker |
| `ProgressCheckerSelector` | Choose progress checker | topic, selected_progress_checker |
| `PathHandlerSelector` | Choose path handler | topic, selected_path_handler |

#### Cancel Actions
| Node | Purpose |
|------|---------|
| `CancelControl` | Cancel FollowPath |
| `CancelBackUp` | Cancel BackUp |
| `CancelSpin` | Cancel Spin |
| `CancelDriveOnHeading` | Cancel DriveOnHeading |
| `CancelWait` | Cancel Wait |
| `CancelAssistedTeleop` | Cancel AssistedTeleop |
| `CancelFollowObject` | Cancel FollowObject |
| `CancelComputeAndTrackRoute` | Cancel route tracking |

#### Services
| Node | Purpose |
|------|---------|
| `ReinitializeGlobalLocalization` | Reset AMCL |
| `ToggleCollisionMonitor` | Enable/disable collision monitor |

---

### Condition Nodes (22)

| Node | Purpose | Key Ports |
|------|---------|-----------|
| `GoalReached` | Goal position reached? | goal, global_frame, robot_base_frame |
| `GoalUpdated` | New goal received? | - |
| `GlobalUpdatedGoal` | Globally updated goal? | - |
| `IsGoalNearby` | Goal near in path? | path, goal, distance_threshold |
| `ArePosesNear` | Two poses close? | pose1, pose2, tolerance |
| `IsPathValid` | Path collision-free? | path, server_timeout |
| `IsWithinPathTrackingBounds` | Robot on path? | path, bounds |
| `DistanceTraveled` | Moved far enough? | distance, global_frame |
| `IsStuck` | Robot stuck? | - |
| `IsStopped` | Robot stopped? | velocity_threshold |
| `IsPoseOccupied` | Pose in obstacle? | pose |
| `InitialPoseReceived` | Initial pose set? | - |
| `TransformAvailable` | TF transform exists? | child, parent |
| `TimeExpired` | Time elapsed? | seconds |
| `PathExpiringTimer` | Path update timer | seconds, path |
| `IsBatteryLow` | Battery low? | min_battery, battery_topic, is_voltage |
| `IsBatteryCharging` | Charging? | battery_topic |
| `WouldAPlannerRecoveryHelp` | Planner recovery needed? | error_code |
| `WouldAControllerRecoveryHelp` | Controller recovery? | error_code |
| `WouldASmootherRecoveryHelp` | Smoother recovery? | error_code |
| `WouldARouteRecoveryHelp` | Route recovery? | error_code |
| `AreErrorCodesPresent` | Match error codes | error_code, error_codes_to_check |

---

### Control Nodes (6)

| Node | Purpose | Key Ports |
|------|---------|-----------|
| `PipelineSequence` | Sequential with parallel setup | - |
| `PersistentSequence` | Resumable sequence | - |
| `NonblockingSequence` | Non-blocking sequential | - |
| `RecoveryNode` | Primary + recovery pattern | number_of_retries |
| `RoundRobin` | Cycle through children | wrap_around (bool) |
| `PauseResumeController` | Pause/resume via service | pause_service, resume_service |

---

### Decorator Nodes (7)

| Node | Purpose | Key Ports |
|------|---------|-----------|
| `RateController` | Tick at frequency | hz (default 10.0) |
| `SpeedController` | Rate by robot speed | min_rate, max_rate, min_speed, max_speed |
| `DistanceController` | Tick after distance | distance, global_frame |
| `GoalUpdater` | Listen for goal updates | input_goal → output_goal |
| `GoalUpdatedController` | Tick on goal update | goal/goals |
| `PathLongerOnApproach` | Detect path lengthening | path, prox_len, length_factor |
| `SingleTrigger` | Execute once only | - |

---

## Default BT XML Files (15)

Location: `~/nav2_ws/src/navigation2/nav2_bt_navigator/behavior_trees/`

| File | Description |
|------|-------------|
| `navigate_to_pose_w_replanning_and_recovery.xml` | **Standard** - 1Hz replanning + recovery |
| `navigate_to_pose_w_replanning_goal_patience_and_recovery.xml` | With goal patience |
| `navigate_to_pose_w_bounds_check.xml` | Path tracking bounds |
| `navigate_w_replanning_distance.xml` | Replan by distance traveled |
| `navigate_w_replanning_speed.xml` | Replan by speed changes |
| `navigate_w_replanning_time.xml` | Replan at fixed intervals |
| `navigate_w_replanning_only_if_goal_is_updated.xml` | Replan only on new goal |
| `navigate_w_replanning_only_if_path_becomes_invalid.xml` | Replan on invalid path |
| `navigate_w_recovery_and_replanning_only_if_path_becomes_invalid.xml` | Combined |
| `navigate_through_poses_w_replanning_and_recovery.xml` | Multi-pose with recovery |
| `navigate_on_route_graph_w_recovery.xml` | Route graph navigation |
| `navigate_w_routing_global_planning_and_control_w_recovery.xml` | Full routing integration |
| `nav_to_pose_with_consistent_replanning_and_if_path_becomes_invalid.xml` | Consistent replanning |
| `follow_point.xml` | Point following |
| `odometry_calibration.xml` | Odom calibration |

## Standard BT Structure

```xml
<root BTCPP_format="4" main_tree_to_execute="MainTree">
  <BehaviorTree ID="MainTree">
    <RecoveryNode number_of_retries="6">
      <!-- Primary: Plan + Follow -->
      <PipelineSequence>
        <!-- Selector nodes for plugins -->
        <ControllerSelector .../>
        <PlannerSelector .../>
        <GoalCheckerSelector .../>
        <ProgressCheckerSelector .../>
        <PathHandlerSelector .../>
        <SmootherSelector .../>

        <!-- Replanning at 1Hz -->
        <RateController hz="1.0">
          <RecoveryNode>
            <ComputePathToPose goal="{goal}" path="{path}" planner_id="{selected_planner}"/>
            <ClearEntireCostmap name="ClearGlobalCostmap" service_name="/global_costmap/clear_entirely_global_costmap"/>
          </RecoveryNode>
        </RateController>

        <!-- Path following with recovery -->
        <RecoveryNode>
          <FollowPath path="{path}" controller_id="{selected_controller}" goal_checker_id="{selected_goal_checker}"/>
          <ClearEntireCostmap name="ClearLocalCostmap" service_name="/local_costmap/clear_entirely_local_costmap"/>
        </RecoveryNode>
      </PipelineSequence>

      <!-- Recovery: Spin, Wait, BackUp rotation -->
      <Sequence>
        <WouldAControllerRecoveryHelp error_code="{error_code}"/>
        <RoundRobin>
          <Spin spin_dist="1.57"/>
          <Wait wait_duration="5.0"/>
          <BackUp backup_dist="0.30" backup_speed="0.15"/>
        </RoundRobin>
      </Sequence>
    </RecoveryNode>
  </BehaviorTree>
</root>
```

---

## Recovery Behavior Plugins

**Package:** `nav2_behaviors`
**Server:** `BehaviorServer` (hosts plugins via pluginlib)

| Behavior | Action | Purpose | Key Params |
|----------|--------|---------|------------|
| `Spin` | nav2_msgs/Spin | Rotate in place | rotational vel/accel limits, simulate_ahead_time |
| `BackUp` | nav2_msgs/BackUp | Reverse | distance, speed, acceleration/deceleration limits |
| `DriveOnHeading` | nav2_msgs/DriveOnHeading | Move forward/backward | distance, speed, accel limits, min speed (0.10) |
| `Wait` | nav2_msgs/Wait | Pause | duration |
| `AssistedTeleop` | nav2_msgs/AssistedTeleop | Safe teleop | projection_time, simulation_time_step |

All behaviors:
- Use `CostmapInfoType::LOCAL` for collision checking
- Support `disable_collision_checks` flag
- Have `time_allowance` for timeout
- Return error codes and elapsed time
