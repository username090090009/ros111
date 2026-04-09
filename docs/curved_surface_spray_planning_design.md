# Curved-Surface Spray Trajectory Planning — Technical Design Document

## 1. Design Goals

Replace the original "simple planar snake spray" approach with **curved-surface spray trajectory planning** based on STL meshes of real car doors.

### 1.1 Deprecated Modules

| Module | File | Description |
|--------|------|-------------|
| Planar snake spray executor | `door_spray_executor.cpp` | Generates fixed-X 2D snake paths, no curved surface support |

### 1.2 New Modules

| Module | File | Description |
|--------|------|-------------|
| STL mesh loader | `stl_mesh_loader.h/cpp` | Parses binary STL, stores triangles and normals |
| Curved-surface spray planner | `surface_spray_planner.h/cpp` | Generates curved-surface spray paths from STL mesh |
| Curved-surface spray node | `curved_surface_spray_node.cpp` | ROS node integrating planning, IK pre-check, and MoveIt execution |

## 2. Algorithm Pipeline

```
┌──────────────────────────────────────────────────────────────┐
│                      Startup Flow                            │
│                                                              │
│  1. Load configuration from ROS parameters                   │
│  2. Read car door STL file                                   │
│  3. Apply coordinate transform (mesh offset + door pose)     │
│  4. Execute curved-surface slicing trajectory planning        │
│  5. IK pre-check: validate reachability per waypoint          │
│  6. Split each band into reachable subsegments                │
│  7. Publish RViz visualization                                │
│  8. Segment-wise MoveIt Cartesian path execution              │
│  9. Output diagnostics summary                                │
└──────────────────────────────────────────────────────────────┘
```

### 2.1 STL Mesh Loading

- Supports binary STL format (80-byte header + triangle count + triangle data)
- Configurable scale factor (SolidWorks exports in mm, use 0.001 to convert to meters)
- Supports rigid-body transform (rotation + translation) for model-to-world conversion

### 2.2 Horizontal Slicing

For each slice height `z`:
1. Iterate all triangles
2. Check each edge for intersection with the `z` plane
3. Collect intersection points, take midpoint of each triangle's intersection line
4. Record the corresponding face normal

### 2.3 Path Point Generation

For each slice's sample point list:
1. Sort by sweep direction (Y or X axis)
2. Compute cumulative arc length, interpolate at `sample_step` intervals
3. Offset along normal by `standoff_distance` to get TCP position
4. Construct orientation quaternion from normal (X axis points toward surface)
5. Store both world-frame and work-frame representations

### 2.4 Snake Path

Even-numbered bands scan forward, odd-numbered bands scan backward, reducing inter-band travel distance.

### 2.5 IK Pre-check (New)

For each waypoint:
1. Attempt IK with the strict normal-aligned orientation
2. If IK fails, try rotations around the spray axis (orientation relaxation)
3. Mark each waypoint as reachable or unreachable
4. Compute per-band statistics

### 2.6 Reachable Segment Extraction (New)

Within each band:
1. Identify contiguous sequences of reachable waypoints
2. Each contiguous sequence becomes a `ReachableSegment`
3. Segments shorter than `min_segment_size` are skipped

### 2.7 Segment-wise Execution (New)

For each reachable segment:
1. If needed, move to segment start via joint-space planning
2. Compute Cartesian path for the segment
3. If Cartesian fraction is too low, retry with relaxed collision
4. Execute the trajectory
5. Report success/failure and Cartesian fraction

## 3. Coordinate Systems and Transforms

### 3.1 Frame Hierarchy

```
world (fixed)
  ├── base_link (robot arm base)
  │     └── link1 → link2 → ... → link6
  │           └── spray_gun_mount_link
  │                 ├── spray_gun_link
  │                 │     └── spray_tcp_link (spray tool center point)
  │                 └── rgbd_camera_link
  │
  └── door_link (car door)
        ├── Joint pose: xyz=(0.90, 0, 0.50) rpy=(0, 0, pi/2)
        ├── Mesh offset: xyz=(-2.035, 0.669, -0.925)
        ├── door_mount_frame (coincides with door_link)
        └── door_work_frame (coincides with door_link)
```

### 3.2 Work Frame Design

- `door_mount_frame`: Represents the door mounting position. In real deployment, determined by calibration.
- `door_work_frame`: Reference frame for spray path representation. Coincides with `door_link` origin.

**Advantages:**
- Spray paths stored in `door_work_frame` are portable across different door placements
- Enables offline path planning + online frame calibration workflow
- Separates geometry processing from runtime execution reference

### 3.3 STL Coordinate Transform Chain

```
STL raw coordinates (mm)
  → × scale (0.001) → Scaled coordinates (m)
  → + mesh_offset → Centered at door_link origin
  → × door_transform → World frame
```

## 4. ROS Interface

### 4.1 Node: curved_surface_spray_node

**Published topics:**
- `spray_waypoint_markers` (visualization_msgs/MarkerArray) — Waypoint visualization
- `spray_pose_array` (geometry_msgs/PoseArray) — Waypoint pose array

**Parameters:**
See `curved_spray_demo.launch` for the complete parameter list.

### 4.2 Key Parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| `band_spacing` | 0.03 m | Horizontal band spacing |
| `sample_step` | 0.02 m | In-band path point spacing |
| `standoff_distance` | 0.18 m | Spray gun standoff from surface |
| `orientation_tolerance` | 0.15 rad | Max orientation relaxation |
| `ik_check_timeout` | 0.05 s | IK solver timeout for pre-check |
| `allow_spray_door_collision` | true | Allow spray gun / door collision |
| `min_segment_size` | 3 | Minimum waypoints per executable segment |

## 5. Timeout Root Cause Analysis

### 5.1 Original Problem

Planning consistently timed out even with visually correct door placement.

### 5.2 Root Causes Identified

1. **IK solver timeout too short (5ms)**: The KDL kinematics solver was configured with only 5ms timeout (`kinematics_solver_timeout: 0.005`), which is far too short for complex poses near workspace boundaries. Fixed to 50ms.

2. **No IK pre-check**: All waypoints were sent directly to `computeCartesianPath`, which internally attempts IK for each point. Unreachable points caused the path computation to either fail or return very low coverage fractions.

3. **Entire band execution**: A single unreachable point in a 30-point band would cause the entire band's Cartesian path to fail, wasting planning time.

4. **Over-constrained orientation**: Strict surface-normal alignment allowed no wrist flexibility, causing IK failures for poses near joint limits.

5. **Excessive collision detection**: The door collision mesh rejected valid near-surface spray poses because the spray gun body was considered colliding.

### 5.3 Fixes Applied

1. Increased IK solver timeout from 0.005s to 0.05s in `kinematics.yaml`
2. Added per-waypoint IK pre-check before any Cartesian execution
3. Implemented reachable segment splitting within each band
4. Added orientation relaxation (rotation around spray axis)
5. Added collision allowance between spray gun links and door_link
6. Added comprehensive diagnostics for debugging remaining issues

## 6. Car Door Pose Fix Record

### 6.1 Original Problem

The car door STL was exported from SolidWorks with origin at the vehicle coordinate system origin (not at the mesh geometric center), causing severe display offset in RViz:

- Mesh geometric center is approximately (2.035, -0.669, 0.925) m from STL origin
- Original URDF had no rotation (`rpy="0 0 0"`), so the door did not face the robot
- Z offset was 0, placing the door center below ground level

### 6.2 Solution Applied

1. **door.xacro**: Added visual/collision `<origin>` offset `xyz="-2.035 0.669 -0.925"`
2. **arm_description.xacro**: Set `world_to_door_joint` to `xyz="0.90 0 0.50" rpy="0 0 1.5708"`
3. **curved_surface_spray_node**: Planner parameters synchronized with the same offset and pose
4. **Work frames added**: `door_mount_frame` and `door_work_frame` for clean frame-based planning
