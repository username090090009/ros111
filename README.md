# Car Door Curved-Surface Spray Trajectory Planning System

A 6-axis robotic arm car door curved-surface spray trajectory planning system based on ROS1 / MoveIt1.

## Project Overview

This project implements **curved-surface spray trajectory planning** targeting real car door STL meshes, replacing the original simple planar snake spray approach. The system loads the car door STL model, extracts surface geometry, generates spray path points with surface normals, performs IK reachability pre-checks, splits paths into executable segments, and executes them via MoveIt Cartesian Path planning.

### Core Features

- **STL mesh curved-surface planning**: Directly loads car door STL files, generates spray paths along the curved surface
- **Automatic surface normal computation**: Each spray path point orientation aligns to the local surface normal
- **Horizontal band snake scanning**: Slices along the Z axis from bottom to top, alternating direction to form snake patterns
- **IK pre-check and reachability analysis**: Validates every waypoint before execution, reports reachability statistics
- **Reachable segment splitting**: Automatically splits bands into continuous reachable subsegments
- **Segment-wise Cartesian execution**: Executes each reachable segment independently instead of entire bands
- **Orientation relaxation**: When strict normal alignment causes IK failure, tries rotations around the spray axis
- **Collision handling**: Allows spray gun / door near-surface contact to avoid false collision rejections
- **Configurable parameters**: Band spacing, sample step, standoff distance, orientation tolerance, etc.
- **RViz visualization**: Path points and surface normal arrows published as RViz Markers
- **Work frame support**: Waypoints can be represented relative to `door_work_frame` for portability
- **Comprehensive diagnostics**: IK success ratio, first failed waypoint, per-band statistics, Cartesian fraction per segment

## System Architecture

```
arm_description/                    # Robot + spray gun + car door URDF/xacro models
├── urdf/
│   ├── arm_description.xacro       # Main xacro: arm + spray gun + door joints + work frames
│   └── door.xacro                  # Car door STL mesh definition
├── meshes/
│   ├── car_door_visual.stl         # Car door visual mesh (SolidWorks export, mm units)
│   ├── car_door_collision.stl      # Car door collision mesh
│   └── *.STL                       # Robot arm link meshes
└── launch/

arm_perception/                     # Perception and trajectory planning
├── include/arm_perception/
│   ├── stl_mesh_loader.h           # Binary STL mesh loader
│   └── surface_spray_planner.h     # Curved-surface spray trajectory planner
├── src/
│   ├── stl_mesh_loader.cpp         # STL loading implementation
│   ├── surface_spray_planner.cpp   # Curved-surface planning implementation
│   ├── curved_surface_spray_node.cpp   # [Main] Curved-surface spray ROS node
│   ├── door_spray_executor.cpp     # [Legacy] Planar snake spray (superseded)
│   ├── door_moveit_executor.cpp    # [Legacy] Single-point MoveIt executor
│   └── door_cloud_listener.cpp     # Point cloud perception node

arm_description_moveit_config/      # MoveIt configuration
├── launch/
│   ├── curved_spray_demo.launch    # [Recommended] Complete curved spray demo
│   ├── spray_demo.launch           # Legacy planar spray demo (superseded)
│   └── ...
└── config/
    ├── kinematics.yaml             # IK solver config (KDL, timeout=0.05s)
    └── ...
```

## Quick Start

### Prerequisites

- ROS Melodic / Noetic
- MoveIt 1
- Gazebo 9 / 11
- Eigen3
- PCL

### Build

```bash
cd ~/catkin_ws
catkin_make
# or
catkin build
source devel/setup.bash
```

### Run Curved-Surface Spray Demo

```bash
# Recommended: one-command full demo (Gazebo + MoveIt + curved spray)
roslaunch arm_description_moveit_config curved_spray_demo.launch
```

### Parameter Tuning

Adjust spray behavior via launch file arguments:

```bash
# Denser bands (2cm spacing) + larger standoff
roslaunch arm_description_moveit_config curved_spray_demo.launch \
  band_spacing:=0.02 \
  standoff_distance:=0.25

# Plan and visualize only, do not execute
roslaunch arm_description_moveit_config curved_spray_demo.launch \
  auto_execute:=false

# Flip normals (when spray surface faces the wrong way)
roslaunch arm_description_moveit_config curved_spray_demo.launch \
  flip_normals:=true

# Increase orientation relaxation for better IK success rate
roslaunch arm_description_moveit_config curved_spray_demo.launch \
  orientation_tolerance:=0.3

# Disable spray gun / door collision allowance
roslaunch arm_description_moveit_config curved_spray_demo.launch \
  allow_spray_door_collision:=false
```

## Coordinate Frames

### Frame Hierarchy

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
        ├── door_mount_frame (coincides with door_link origin)
        └── door_work_frame  (coincides with door_link origin)
```

### `door_work_frame` Design

The `door_work_frame` is defined as a fixed frame coinciding with `door_link` origin. It serves as the reference for representing spray path waypoints in a door-relative coordinate system.

**Purpose:**
- In simulation, `door_work_frame` is computed from the URDF joint parameters.
- In real-robot deployment, `door_work_frame` will be determined by calibration (e.g., vision-based localization).
- Spray paths stored relative to `door_work_frame` can be directly reused across different door placements.

### STL Coordinate Transform Chain

```
STL raw coordinates (mm)
  → × scale (0.001) → Scaled coordinates (m)
  → + mesh_offset → Centered at door_link origin
  → × door_transform → World frame
```

## Car Door Pose Configuration

### Fixed Issues

The original car door STL had the following problems:
1. **STL mesh origin not at geometric center**: SolidWorks exported the STL with origin at the vehicle coordinate system, approximately 2m from the door's geometric center.
2. **Door not rotated**: The door plane was not facing the robot arm.
3. **Z position at zero**: The door was on the ground, mostly outside the robot workspace.

### Solution

**door.xacro mesh centering offset:**

Car door STL bounding box (after scaling, in meters):
- X: 1.429 ~ 2.642 (width ≈ 1.213m)
- Y: -0.858 ~ -0.480 (thickness ≈ 0.379m)
- Z: 0.367 ~ 1.483 (height ≈ 1.116m)
- Geometric center: (2.035, -0.669, 0.925)

Setting `visual/collision origin xyz="-2.035 0.669 -0.925"` centers the mesh at `door_link` origin.

**arm_description.xacro joint pose:**

```xml
<joint name="world_to_door_joint" type="fixed">
  <origin xyz="0.90 0.00 0.50" rpy="0 0 1.5708"/>
</joint>
```

- `X=0.90m`: Door center 0.9m from robot base
- `Z=0.50m`: Door center height 0.5m, placing the spray region in the robot workspace
- `yaw=pi/2`: Rotate 90° so the door surface faces the robot

> Note: If the door outer surface faces away from the robot after loading, change yaw to `-1.5708`,
> or set `flip_normals:=true` in the launch file.

## Planning Algorithm

### Curved-Surface Spray Path Generation Pipeline

1. **Load STL mesh**: Read binary STL file, extract triangles and normals
2. **Coordinate transform**: Transform mesh from model frame to robot base frame (consistent with URDF)
3. **Horizontal slicing**: Slice mesh along Z axis at `band_spacing` intervals
4. **Contour sampling**: For each slice, find triangle-plane intersection lines, sort by sweep direction, sample at equal intervals
5. **Normal offset**: Offset each sample point along the local surface normal by `standoff_distance` to get spray TCP position
6. **Orientation generation**: Construct spray gun orientation so TCP X axis points toward the surface (opposite of normal)
7. **Snake path**: Adjacent bands alternate sweep direction forming S-shaped paths
8. **IK pre-check**: Validate IK for every waypoint, with orientation relaxation if needed
9. **Reachable segment extraction**: Split each band into continuous reachable subsegments
10. **Segment-wise execution**: Plan and execute Cartesian paths per segment

### Planning Timeout Root Cause and Fix

The previous version experienced planning timeouts due to several issues:

1. **IK solver timeout too short** (0.005s → fixed to 0.05s): The KDL IK solver had only 5ms to find a solution, which is insufficient for many poses near workspace boundaries. Increased to 50ms.

2. **No IK pre-check**: The code attempted Cartesian path execution on all waypoints without knowing which ones were reachable, leading to the planner spending time on impossible poses.

3. **No reachable segment splitting**: A single unreachable waypoint in a band would cause the entire band's Cartesian path to fail or have very low coverage fraction.

4. **Over-constrained orientation**: Strict normal alignment with no tolerance caused IK failures for poses near joint limits. Added configurable orientation relaxation.

5. **Door collision too restrictive**: The door collision mesh rejected valid near-surface spray poses. Added collision allowance between spray gun links and door_link.

### Core Parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| `band_spacing` | 0.03 m | Horizontal band spacing |
| `sample_step` | 0.02 m | In-band path point spacing |
| `standoff_distance` | 0.18 m | Spray gun standoff from surface |
| `boundary_margin` | 0.02 m | Boundary trimming margin |
| `eef_step` | 0.01 m | Cartesian path interpolation step |
| `min_cartesian_fraction` | 0.70 | Minimum Cartesian path coverage |
| `orientation_tolerance` | 0.15 rad | Max orientation relaxation |
| `ik_check_timeout` | 0.05 s | IK solver timeout for pre-check |
| `relaxation_attempts` | 6 | Number of orientation relaxation attempts |
| `min_segment_size` | 3 | Minimum waypoints per executable segment |
| `allow_spray_door_collision` | true | Allow spray gun / door collision |

## Diagnostics

The node outputs detailed diagnostic information during execution:

```
========================================
IK Pre-check Diagnostics Summary:
  Total waypoints: 450
  Reachable:   380 / 450 (84.4%)
  Unreachable: 70 / 450 (15.6%)
  First unreachable: band 2, waypoint 15
  Total executable segments: 22
========================================
```

For each band:
```
  Band 0: 30 waypoints, 28 reachable (93.3%), 2 segments
  Band 1: 32 waypoints, 25 reachable (78.1%), 3 segments
```

For each segment execution:
```
  Executing segment: band 0, seg 0, 15 waypoints (idx 0-14)...
    Cartesian fraction: 95.2% (15 waypoints)
    Segment execution succeeded.
```

### Troubleshooting

| Symptom | Likely Cause | Solution |
|---------|-------------|----------|
| 0% reachable waypoints | Door outside workspace | Adjust door_x/y/z closer to robot |
| Low IK success rate | Standoff too large | Reduce standoff_distance |
| Low IK success rate | Orientation too strict | Increase orientation_tolerance |
| Low Cartesian fraction | Collision rejection | Set allow_spray_door_collision=true |
| All segments skipped | min_segment_size too high | Reduce min_segment_size |
| Planning timeout | IK solver timeout too low | Increase kinematics_solver_timeout in kinematics.yaml |

## Future Work

- [ ] Multi-region segmented spraying (auto-partition by reachability)
- [ ] Point cloud perception for automatic door localization (reuse door_cloud_listener)
- [ ] Velocity/acceleration optimization (constant-speed spray constraint)
- [ ] Real robot deployment and collision verification
- [ ] Custom spray region ROI selection
- [ ] Boundary-aware stripe generation (replace pure Z-slicing)

## License

This project is for research and educational purposes only.
