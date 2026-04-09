/**
 * @file curved_surface_spray_node.cpp
 * @brief Curved-surface spray trajectory planning and execution ROS node
 *
 * 替代旧的 door_spray_executor（简单平面蛇形喷涂），
 * 实现基于 STL 网格的真实车门曲面喷涂轨迹规划。
 *
 * 主要改进（相比上一版本）：
 *   - IK 预检查：在执行前逐点验证可达性
 *   - 可达段分割：将每个条带按 IK 连续性分割为子段
 *   - 段级 Cartesian 执行：每个可达子段独立规划和执行
 *   - 姿态松弛：IK 失败时尝试绕喷涂轴旋转
 *   - 碰撞处理：允许喷枪与车门近表面接触
 *   - 诊断输出：详细的可达性和执行统计信息
 *   - 工作坐标系：支持 door_work_frame 路径表示
 *
 * ROS parameters (via launch file or command line):
 *   ~stl_filepath          : Car door STL file path
 *   ~mesh_scale            : STL scale factor (default 0.001)
 *   ~door_x/y/z            : Door center position in world frame (matches URDF)
 *   ~door_roll/pitch/yaw   : Door orientation
 *   ~mesh_offset_x/y/z     : STL mesh centering offset
 *   ~band_spacing          : Band spacing (m)
 *   ~sample_step           : In-band sample step (m)
 *   ~standoff_distance     : Spray gun standoff from surface (m)
 *   ~boundary_margin       : Boundary trimming margin
 *   ~flip_normals          : Whether to flip normals
 *   ~planning_group        : MoveIt planning group name
 *   ~end_effector_link     : End effector link name
 *   ~eef_step              : Cartesian path interpolation step
 *   ~jump_threshold        : Cartesian path jump threshold
 *   ~min_cartesian_fraction: Minimum Cartesian path coverage
 *   ~auto_execute          : Whether to execute after planning
 *   ~visualize             : Whether to publish RViz markers
 *   ~orientation_tolerance : Max angular deviation for orientation relaxation (rad)
 *   ~ik_check_timeout      : IK solver timeout for pre-check (s)
 *   ~relaxation_attempts   : Number of orientation relaxation attempts
 *   ~min_segment_size      : Minimum waypoints per executable segment
 *   ~allow_spray_door_collision : Allow collision between spray gun and door
 */

#include <ros/ros.h>
#include <ros/package.h>

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseArray.h>
#include <visualization_msgs/MarkerArray.h>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit_msgs/RobotTrajectory.h>
#include <moveit_msgs/PlanningScene.h>
#include <moveit_msgs/AllowedCollisionMatrix.h>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "arm_perception/stl_mesh_loader.h"
#include "arm_perception/surface_spray_planner.h"

/// 可达子段：一组连续可达的路径点
struct ReachableSegment
{
  int band_index;                          ///< 所属条带
  int start_idx;                           ///< 在条带内的起始索引
  int end_idx;                             ///< 在条带内的结束索引（不含）
  std::vector<geometry_msgs::Pose> poses;  ///< 段内的路径点
};

/**
 * @class CurvedSurfaceSprayNode
 * @brief Curved-surface spray main node
 *
 * 曲面喷涂主节点：集成规划、IK预检查、段级执行和诊断
 */
class CurvedSurfaceSprayNode
{
public:
  CurvedSurfaceSprayNode()
    : planning_group_(ros::NodeHandle("~").param<std::string>("planning_group", "arm")),
      move_group_(planning_group_)
  {
    ros::NodeHandle pnh("~");
    pnh.param<std::string>("end_effector_link", end_effector_link_, "spray_tcp_link");
    pnh.param<double>("planning_time", planning_time_, 15.0);
    pnh.param<int>("max_attempts", max_attempts_, 20);
    pnh.param<double>("position_tolerance", position_tolerance_, 0.02);
    pnh.param<double>("orientation_tolerance_goal", goal_orientation_tolerance_, 0.30);

    // ---- Cartesian path parameters ----
    pnh.param<double>("eef_step", eef_step_, 0.01);
    pnh.param<double>("jump_threshold", jump_threshold_, 0.0);
    pnh.param<double>("min_cartesian_fraction", min_cartesian_fraction_, 0.70);

    // ---- Execution and visualization ----
    pnh.param<bool>("auto_execute", auto_execute_, true);
    pnh.param<bool>("visualize", visualize_, true);

    // ---- IK pre-check and orientation relaxation ----
    pnh.param<double>("orientation_tolerance", orientation_tolerance_, 0.15);
    pnh.param<double>("ik_check_timeout", ik_check_timeout_, 0.05);
    pnh.param<int>("relaxation_attempts", relaxation_attempts_, 6);
    pnh.param<int>("min_segment_size", min_segment_size_, 3);

    // ---- Collision handling ----
    pnh.param<bool>("allow_spray_door_collision", allow_spray_door_collision_, true);

    // ---- STL mesh parameters ----
    std::string pkg_path = ros::package::getPath("arm_description");
    if (pkg_path.empty())
    {
      ROS_WARN("Cannot find arm_description package path. "
               "Make sure the workspace is properly built and sourced.");
    }
    std::string default_stl = pkg_path + "/meshes/car_door_visual.stl";
    pnh.param<std::string>("stl_filepath", stl_filepath_, default_stl);
    pnh.param<double>("mesh_scale", mesh_scale_, 0.001);

    // ---- Door pose (must match URDF world_to_door_joint) ----
    pnh.param<double>("door_x", door_x_, 0.90);
    pnh.param<double>("door_y", door_y_, 0.00);
    pnh.param<double>("door_z", door_z_, 0.50);
    pnh.param<double>("door_roll", door_roll_, 0.0);
    pnh.param<double>("door_pitch", door_pitch_, 0.0);
    pnh.param<double>("door_yaw", door_yaw_, 1.5708);

    // ---- Mesh centering offset (must match door.xacro) ----
    pnh.param<double>("mesh_offset_x", mesh_offset_x_, -2.035);
    pnh.param<double>("mesh_offset_y", mesh_offset_y_, 0.669);
    pnh.param<double>("mesh_offset_z", mesh_offset_z_, -0.925);

    // ---- Spray planning parameters ----
    pnh.param<double>("band_spacing", band_spacing_, 0.03);
    pnh.param<double>("sample_step", sample_step_, 0.02);
    pnh.param<double>("standoff_distance", standoff_distance_, 0.18);
    pnh.param<double>("boundary_margin", boundary_margin_, 0.02);
    pnh.param<bool>("flip_normals", flip_normals_, false);

    // ---- Configure MoveIt ----
    move_group_.setPlanningTime(planning_time_);
    move_group_.setNumPlanningAttempts(max_attempts_);
    move_group_.setGoalPositionTolerance(position_tolerance_);
    move_group_.setGoalOrientationTolerance(goal_orientation_tolerance_);
    move_group_.setEndEffectorLink(end_effector_link_);
    move_group_.allowReplanning(true);

    // ---- Publishers ----
    if (visualize_)
    {
      waypoint_marker_pub_ = nh_.advertise<visualization_msgs::MarkerArray>(
          "spray_waypoint_markers", 1, true);
      pose_array_pub_ = nh_.advertise<geometry_msgs::PoseArray>(
          "spray_pose_array", 1, true);
    }

    // ---- Collision handling publisher ----
    planning_scene_pub_ = nh_.advertise<moveit_msgs::PlanningScene>(
        "planning_scene", 1);

    ROS_INFO("========================================");
    ROS_INFO("Curved-surface spray planning node started");
    ROS_INFO("========================================");
    ROS_INFO("Planning group: %s", planning_group_.c_str());
    ROS_INFO("End effector link: %s", end_effector_link_.c_str());
    ROS_INFO("STL file: %s", stl_filepath_.c_str());
    ROS_INFO("Door pose: xyz=(%.3f, %.3f, %.3f) rpy=(%.3f, %.3f, %.3f)",
             door_x_, door_y_, door_z_, door_roll_, door_pitch_, door_yaw_);
    ROS_INFO("Mesh offset: (%.3f, %.3f, %.3f)",
             mesh_offset_x_, mesh_offset_y_, mesh_offset_z_);
    ROS_INFO("Band spacing: %.3f m, sample step: %.3f m, standoff: %.3f m",
             band_spacing_, sample_step_, standoff_distance_);
    ROS_INFO("IK check timeout: %.3f s, orientation tolerance: %.3f rad",
             ik_check_timeout_, orientation_tolerance_);
    ROS_INFO("Min segment size: %d, relaxation attempts: %d",
             min_segment_size_, relaxation_attempts_);
  }

  /**
   * @brief 执行完整的曲面喷涂流程
   * @return 成功返回 true
   */
  bool run()
  {
    // 1. 构造规划参数
    arm_perception::SprayPlannerParams params;
    params.stl_filepath = stl_filepath_;
    params.mesh_scale = mesh_scale_;
    params.mesh_offset = Eigen::Vector3d(mesh_offset_x_, mesh_offset_y_, mesh_offset_z_);
    params.band_spacing = band_spacing_;
    params.sample_step = sample_step_;
    params.standoff_distance = standoff_distance_;
    params.boundary_margin = boundary_margin_;
    params.flip_normals = flip_normals_;
    params.orientation_tolerance = orientation_tolerance_;

    // 构造车门位姿变换矩阵
    Eigen::Isometry3d door_transform = Eigen::Isometry3d::Identity();
    Eigen::AngleAxisd roll_rot(door_roll_, Eigen::Vector3d::UnitX());
    Eigen::AngleAxisd pitch_rot(door_pitch_, Eigen::Vector3d::UnitY());
    Eigen::AngleAxisd yaw_rot(door_yaw_, Eigen::Vector3d::UnitZ());
    door_transform.linear() = (yaw_rot * pitch_rot * roll_rot).toRotationMatrix();
    door_transform.translation() = Eigen::Vector3d(door_x_, door_y_, door_z_);
    params.door_transform = door_transform;

    // 2. 初始化规划器
    arm_perception::SurfaceSprayPlanner planner;
    ROS_INFO("Loading STL mesh and initializing planner...");

    if (!planner.init(params))
    {
      ROS_ERROR("Planner initialization failed! Check STL file path: %s",
                stl_filepath_.c_str());
      return false;
    }

    ROS_INFO("Mesh loaded: %zu triangles, total area %.4f m^2",
             planner.mesh().numTriangles(),
             planner.mesh().totalArea());
    ROS_INFO("Mesh bounding box (world frame): "
             "min=(%.3f, %.3f, %.3f) max=(%.3f, %.3f, %.3f)",
             planner.mesh().boundsMin().x(),
             planner.mesh().boundsMin().y(),
             planner.mesh().boundsMin().z(),
             planner.mesh().boundsMax().x(),
             planner.mesh().boundsMax().y(),
             planner.mesh().boundsMax().z());

    // 3. 执行轨迹规划
    ROS_INFO("Running curved-surface spray trajectory planning...");
    if (!planner.plan())
    {
      ROS_ERROR("Curved-surface spray trajectory planning failed!");
      return false;
    }

    ROS_INFO("Planning complete: %d bands, %zu total waypoints",
             planner.numBands(),
             planner.numWaypoints());

    // 4. 可视化
    if (visualize_)
    {
      publishVisualization(planner);
    }

    // 5. 设置碰撞许可（允许喷枪和车门近表面接触）
    if (allow_spray_door_collision_)
    {
      setupCollisionAllowance();
    }

    // 6. IK 预检查 + 可达段分割
    ROS_INFO("========================================");
    ROS_INFO("Starting IK pre-check and reachability analysis...");
    ROS_INFO("========================================");

    const auto& bands = planner.bands();
    std::vector<std::vector<ReachableSegment>> all_segments;
    int total_reachable = 0;
    int total_unreachable = 0;
    int total_segments = 0;
    int first_unreachable_band = -1;
    int first_unreachable_idx = -1;

    for (const auto& band : bands)
    {
      // 对每个条带进行 IK 预检查
      std::vector<bool> ik_results;
      std::vector<geometry_msgs::Pose> band_poses;
      checkBandIK(band, ik_results, band_poses);

      int band_reachable = 0;
      for (bool ok : ik_results)
      {
        if (ok) ++band_reachable;
      }
      int band_unreachable = static_cast<int>(ik_results.size()) - band_reachable;
      total_reachable += band_reachable;
      total_unreachable += band_unreachable;

      // 记录第一个不可达点
      if (first_unreachable_band < 0 && band_unreachable > 0)
      {
        first_unreachable_band = band.band_index;
        for (size_t i = 0; i < ik_results.size(); ++i)
        {
          if (!ik_results[i])
          {
            first_unreachable_idx = static_cast<int>(i);
            break;
          }
        }
      }

      // 分割可达段
      std::vector<ReachableSegment> segments =
          splitIntoReachableSegments(band.band_index, band_poses, ik_results);
      total_segments += static_cast<int>(segments.size());

      ROS_INFO("  Band %d: %zu waypoints, %d reachable (%.1f%%), %zu segments",
               band.band_index,
               ik_results.size(),
               band_reachable,
               ik_results.empty() ? 0.0 :
                   100.0 * band_reachable / ik_results.size(),
               segments.size());

      all_segments.push_back(segments);
    }

    // 打印诊断摘要
    int total_waypoints = total_reachable + total_unreachable;
    ROS_INFO("========================================");
    ROS_INFO("IK Pre-check Diagnostics Summary:");
    ROS_INFO("  Total waypoints: %d", total_waypoints);
    ROS_INFO("  Reachable:   %d / %d (%.1f%%)",
             total_reachable, total_waypoints,
             total_waypoints > 0 ?
                 100.0 * total_reachable / total_waypoints : 0.0);
    ROS_INFO("  Unreachable: %d / %d (%.1f%%)",
             total_unreachable, total_waypoints,
             total_waypoints > 0 ?
                 100.0 * total_unreachable / total_waypoints : 0.0);
    if (first_unreachable_band >= 0)
    {
      ROS_INFO("  First unreachable: band %d, waypoint %d",
               first_unreachable_band, first_unreachable_idx);
    }
    ROS_INFO("  Total executable segments: %d", total_segments);
    ROS_INFO("========================================");

    if (total_reachable == 0)
    {
      ROS_ERROR("No reachable waypoints found! Possible causes:");
      ROS_ERROR("  1. Door is outside robot workspace (check door_x/y/z)");
      ROS_ERROR("  2. Standoff distance too large (current: %.3f m)", standoff_distance_);
      ROS_ERROR("  3. Orientation too restrictive (try increasing orientation_tolerance)");
      ROS_ERROR("  4. Collision between spray gun and door (check allow_spray_door_collision)");
      return false;
    }

    if (!auto_execute_)
    {
      ROS_INFO("auto_execute=false, visualization only, not executing trajectory.");
      return true;
    }

    // 7. 段级执行
    ROS_INFO("========================================");
    ROS_INFO("Starting segment-wise trajectory execution...");
    ROS_INFO("========================================");

    int executed_segments = 0;
    int failed_segments = 0;
    int skipped_segments = 0;
    bool need_recovery_move = false;

    for (size_t bi = 0; bi < all_segments.size(); ++bi)
    {
      const auto& segments = all_segments[bi];
      for (size_t si = 0; si < segments.size(); ++si)
      {
        const auto& seg = segments[si];

        if (static_cast<int>(seg.poses.size()) < min_segment_size_)
        {
          ROS_INFO("  Skipping segment (band %d, seg %zu): "
                   "too few waypoints (%zu < %d)",
                   seg.band_index, si, seg.poses.size(), min_segment_size_);
          ++skipped_segments;
          need_recovery_move = true;
          continue;
        }

        // 如果需要恢复移动（前一段失败或跳过），先用关节空间规划移到段起点
        if (need_recovery_move || (si == 0 && bi > 0))
        {
          ROS_INFO("  Moving to segment start (band %d, seg %zu)...",
                   seg.band_index, si);
          if (!moveToPose(seg.poses.front(), "segment_start"))
          {
            ROS_WARN("  Cannot reach segment start, skipping segment.");
            ++failed_segments;
            need_recovery_move = true;
            continue;
          }
          ros::Duration(0.3).sleep();
          need_recovery_move = false;
        }
        else if (si == 0 && bi == 0)
        {
          // 第一段：先移动到起始点
          ROS_INFO("  Moving to spray start point...");
          if (!moveToPose(seg.poses.front(), "spray_start"))
          {
            ROS_WARN("  Cannot reach spray start point, aborting.");
            return false;
          }
          ros::Duration(0.5).sleep();
        }

        // 执行 Cartesian Path
        ROS_INFO("  Executing segment: band %d, seg %zu, "
                 "%zu waypoints (idx %d-%d)...",
                 seg.band_index, si, seg.poses.size(),
                 seg.start_idx, seg.end_idx - 1);

        if (executeCartesianSegment(seg))
        {
          ++executed_segments;
          need_recovery_move = false;
        }
        else
        {
          ++failed_segments;
          need_recovery_move = true;
        }

        ros::Duration(0.2).sleep();
      }
    }

    // 执行总结
    ROS_INFO("========================================");
    ROS_INFO("Spray Execution Summary:");
    ROS_INFO("  Segments executed: %d", executed_segments);
    ROS_INFO("  Segments failed:   %d", failed_segments);
    ROS_INFO("  Segments skipped:  %d", skipped_segments);
    ROS_INFO("========================================");

    return (executed_segments > 0);
  }

private:
  // ==========================================================================
  // IK 预检查：检查条带内每个路径点的可达性
  // ==========================================================================

  /**
   * @brief 对条带路径点进行 IK 预检查
   * @param band 喷涂条带
   * @param ik_results [输出] 每个点的 IK 结果（true=可达）
   * @param poses [输出] 每个点的 ROS Pose（可能经过松弛调整）
   *
   * 对每个路径点：
   *   1. 首先尝试原始姿态的 IK
   *   2. 如果失败，尝试绕喷涂轴旋转的松弛姿态
   *   3. 如果仍然失败，标记为不可达
   */
  void checkBandIK(const arm_perception::SprayBand& band,
                   std::vector<bool>& ik_results,
                   std::vector<geometry_msgs::Pose>& poses)
  {
    ik_results.clear();
    poses.clear();
    ik_results.reserve(band.waypoints.size());
    poses.reserve(band.waypoints.size());

    // 获取当前机器人状态用于 IK 计算
    moveit::core::RobotStatePtr kinematic_state =
        move_group_.getCurrentState(5.0);
    if (!kinematic_state)
    {
      ROS_WARN("  Failed to get current robot state for IK check.");
      // 标记所有点为不可达
      for (size_t i = 0; i < band.waypoints.size(); ++i)
      {
        ik_results.push_back(false);
        poses.push_back(waypointToPose(band.waypoints[i]));
      }
      return;
    }

    const moveit::core::JointModelGroup* jmg =
        kinematic_state->getJointModelGroup(planning_group_);
    if (!jmg)
    {
      ROS_WARN("  Joint model group '%s' not found.", planning_group_.c_str());
      for (size_t i = 0; i < band.waypoints.size(); ++i)
      {
        ik_results.push_back(false);
        poses.push_back(waypointToPose(band.waypoints[i]));
      }
      return;
    }

    for (size_t i = 0; i < band.waypoints.size(); ++i)
    {
      const auto& wp = band.waypoints[i];
      geometry_msgs::Pose pose = waypointToPose(wp);
      bool found_ik = false;

      // 尝试原始姿态
      found_ik = kinematic_state->setFromIK(
          jmg, pose, end_effector_link_, ik_check_timeout_);

      // 如果原始姿态失败，尝试绕喷涂轴旋转的松弛姿态
      if (!found_ik && relaxation_attempts_ > 0 && orientation_tolerance_ > 1e-6)
      {
        for (int a = 1; a <= relaxation_attempts_ && !found_ik; ++a)
        {
          double fraction = static_cast<double>(a) / relaxation_attempts_;
          double angle = orientation_tolerance_ * fraction;

          // 尝试正方向旋转
          Eigen::Quaterniond relaxed_q =
              arm_perception::SurfaceSprayPlanner::rotateAroundSprayAxis(
                  wp.orientation, wp.surface_normal, angle);
          geometry_msgs::Pose relaxed_pose = pose;
          relaxed_pose.orientation.x = relaxed_q.x();
          relaxed_pose.orientation.y = relaxed_q.y();
          relaxed_pose.orientation.z = relaxed_q.z();
          relaxed_pose.orientation.w = relaxed_q.w();

          found_ik = kinematic_state->setFromIK(
              jmg, relaxed_pose, end_effector_link_, ik_check_timeout_);
          if (found_ik)
          {
            pose = relaxed_pose;
            break;
          }

          // 尝试负方向旋转
          relaxed_q = arm_perception::SurfaceSprayPlanner::rotateAroundSprayAxis(
              wp.orientation, wp.surface_normal, -angle);
          relaxed_pose.orientation.x = relaxed_q.x();
          relaxed_pose.orientation.y = relaxed_q.y();
          relaxed_pose.orientation.z = relaxed_q.z();
          relaxed_pose.orientation.w = relaxed_q.w();

          found_ik = kinematic_state->setFromIK(
              jmg, relaxed_pose, end_effector_link_, ik_check_timeout_);
          if (found_ik)
          {
            pose = relaxed_pose;
            break;
          }
        }
      }

      ik_results.push_back(found_ik);
      poses.push_back(pose);
    }
  }

  // ==========================================================================
  // 可达段分割：将条带按 IK 连续性分割为子段
  // ==========================================================================

  /**
   * @brief 将条带路径点按 IK 可达性分割为连续子段
   *
   * 连续可达的路径点组成一个 ReachableSegment。
   * 不可达的点作为分割边界。
   */
  std::vector<ReachableSegment> splitIntoReachableSegments(
      int band_index,
      const std::vector<geometry_msgs::Pose>& poses,
      const std::vector<bool>& ik_results) const
  {
    std::vector<ReachableSegment> segments;

    if (poses.empty())
    {
      return segments;
    }

    ReachableSegment current_seg;
    current_seg.band_index = band_index;
    current_seg.start_idx = -1;
    bool in_segment = false;

    for (size_t i = 0; i < poses.size(); ++i)
    {
      if (ik_results[i])
      {
        if (!in_segment)
        {
          // 开始新段
          current_seg.start_idx = static_cast<int>(i);
          current_seg.poses.clear();
          in_segment = true;
        }
        current_seg.poses.push_back(poses[i]);
        current_seg.end_idx = static_cast<int>(i) + 1;
      }
      else
      {
        if (in_segment)
        {
          // 结束当前段
          segments.push_back(current_seg);
          in_segment = false;
        }
      }
    }

    // 结束最后一个段
    if (in_segment)
    {
      segments.push_back(current_seg);
    }

    return segments;
  }

  // ==========================================================================
  // 碰撞许可设置：允许喷枪与车门的近表面碰撞
  // ==========================================================================

  /**
   * @brief 设置碰撞许可矩阵，允许喷枪与车门接触
   *
   * 喷涂时喷枪必须靠近车门表面，如果碰撞检测过于严格，
   * 很多有效的喷涂姿态会被错误拒绝。
   */
  void setupCollisionAllowance()
  {
    ROS_INFO("Setting up collision allowance for spray gun <-> door...");

    moveit_msgs::PlanningScene planning_scene_msg;
    planning_scene_msg.is_diff = true;

    // 允许 spray_gun_link 和 door_link 之间的碰撞
    moveit_msgs::AllowedCollisionMatrix& acm =
        planning_scene_msg.allowed_collision_matrix;

    acm.entry_names.push_back("door_link");
    acm.entry_names.push_back("spray_gun_link");
    acm.entry_names.push_back("spray_gun_mount_link");
    acm.entry_names.push_back("spray_tcp_link");

    // 创建 NxN 的 ACM 条目
    // entry_names 顺序: [0]=door_link, [1]=spray_gun_link,
    //   [2]=spray_gun_mount_link, [3]=spray_tcp_link
    // 允许 door_link (index 0) 与所有喷枪组件 (index 1,2,3) 的碰撞
    for (size_t i = 0; i < acm.entry_names.size(); ++i)
    {
      moveit_msgs::AllowedCollisionEntry entry;
      for (size_t j = 0; j < acm.entry_names.size(); ++j)
      {
        // 允许 door_link 与所有喷枪链接的碰撞
        bool allow = (i == 0 || j == 0) && (i != j);
        entry.enabled.push_back(allow);
      }
      acm.entry_values.push_back(entry);
    }

    // 等待 planning_scene 订阅者
    ros::Duration(0.5).sleep();
    planning_scene_pub_.publish(planning_scene_msg);
    ros::Duration(0.5).sleep();

    ROS_INFO("Collision allowance set: spray_gun_link <-> door_link allowed.");
  }

  // ==========================================================================
  // 移动到指定姿态（关节空间规划）
  // ==========================================================================

  /**
   * @brief 使用 MoveIt 关节空间规划移动到指定位姿
   */
  bool moveToPose(const geometry_msgs::Pose& target_pose,
                  const std::string& name)
  {
    move_group_.clearPoseTargets();
    move_group_.setStartStateToCurrentState();
    move_group_.setPoseTarget(target_pose, end_effector_link_);

    ROS_INFO("  Planning to [%s]: pos=(%.3f, %.3f, %.3f)",
             name.c_str(),
             target_pose.position.x,
             target_pose.position.y,
             target_pose.position.z);

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    auto result = move_group_.plan(plan);

    if (result == moveit::planning_interface::MoveItErrorCode::SUCCESS)
    {
      ROS_INFO("  [%s] Planning succeeded, executing...", name.c_str());
      auto exec_result = move_group_.execute(plan);
      if (exec_result == moveit::planning_interface::MoveItErrorCode::SUCCESS)
      {
        ROS_INFO("  [%s] Execution succeeded.", name.c_str());
        return true;
      }
      else
      {
        ROS_WARN("  [%s] Execution failed (error code: %d).",
                 name.c_str(), exec_result.val);
        return false;
      }
    }
    else
    {
      ROS_WARN("  [%s] Planning failed (error code: %d). "
               "Possible causes: unreachable pose, collision, or timeout.",
               name.c_str(), result.val);
      return false;
    }
  }

  // ==========================================================================
  // 段级 Cartesian 执行
  // ==========================================================================

  /**
   * @brief 使用 MoveIt Cartesian Path 执行一个可达子段
   *
   * 相比旧版的条带级执行，段级执行更精细：
   * 每个段已经过 IK 预检查，所有点都可达。
   */
  bool executeCartesianSegment(const ReachableSegment& segment)
  {
    if (segment.poses.empty())
    {
      return false;
    }

    moveit_msgs::RobotTrajectory trajectory;
    const bool avoid_collisions = !allow_spray_door_collision_;

    move_group_.setStartStateToCurrentState();

    double fraction = move_group_.computeCartesianPath(
        segment.poses, eef_step_, jump_threshold_,
        trajectory, avoid_collisions);

    ROS_INFO("    Cartesian fraction: %.1f%% (%zu waypoints)",
             fraction * 100.0, segment.poses.size());

    if (fraction < min_cartesian_fraction_)
    {
      ROS_WARN("    Cartesian fraction too low (%.1f%% < %.1f%%). "
               "Trying with relaxed collision...",
               fraction * 100.0, min_cartesian_fraction_ * 100.0);

      // 尝试不碰撞检查的 Cartesian 路径（作为后备）
      fraction = move_group_.computeCartesianPath(
          segment.poses, eef_step_, jump_threshold_,
          trajectory, false);  // avoid_collisions = false

      ROS_INFO("    Relaxed Cartesian fraction: %.1f%%", fraction * 100.0);

      if (fraction < min_cartesian_fraction_)
      {
        ROS_WARN("    Segment execution abandoned: fraction still too low.");
        ROS_WARN("    Diagnosis: likely cause is IK discontinuity "
                 "or joint limit at boundary.");
        return false;
      }
    }

    moveit::planning_interface::MoveGroupInterface::Plan cartesian_plan;
    cartesian_plan.trajectory_ = trajectory;

    auto exec_result = move_group_.execute(cartesian_plan);

    if (exec_result == moveit::planning_interface::MoveItErrorCode::SUCCESS)
    {
      ROS_INFO("    Segment execution succeeded.");
      return true;
    }
    else
    {
      ROS_WARN("    Segment execution failed (error code: %d).",
               exec_result.val);
      return false;
    }
  }

  // ==========================================================================
  // 辅助函数
  // ==========================================================================

  /// 将 SprayWaypoint 转换为 geometry_msgs::Pose
  static geometry_msgs::Pose waypointToPose(
      const arm_perception::SprayWaypoint& wp)
  {
    geometry_msgs::Pose pose;
    pose.position.x = wp.position.x();
    pose.position.y = wp.position.y();
    pose.position.z = wp.position.z();
    pose.orientation.x = wp.orientation.x();
    pose.orientation.y = wp.orientation.y();
    pose.orientation.z = wp.orientation.z();
    pose.orientation.w = wp.orientation.w();
    return pose;
  }

  // ==========================================================================
  // RViz 可视化
  // ==========================================================================

  /**
   * @brief 发布喷涂路径点的 RViz 可视化
   */
  void publishVisualization(
      const arm_perception::SurfaceSprayPlanner& planner)
  {
    const auto& waypoints = planner.waypoints();

    // 发布 MarkerArray：路径点 + 法向量箭头
    visualization_msgs::MarkerArray marker_array;

    // 清除旧 marker
    visualization_msgs::Marker delete_marker;
    delete_marker.action = visualization_msgs::Marker::DELETEALL;
    marker_array.markers.push_back(delete_marker);

    // 路径点球体
    for (size_t i = 0; i < waypoints.size(); ++i)
    {
      visualization_msgs::Marker marker;
      marker.header.frame_id = "world";
      marker.header.stamp = ros::Time::now();
      marker.ns = "spray_waypoints";
      marker.id = static_cast<int>(i);
      marker.type = visualization_msgs::Marker::SPHERE;
      marker.action = visualization_msgs::Marker::ADD;

      marker.pose.position.x = waypoints[i].position.x();
      marker.pose.position.y = waypoints[i].position.y();
      marker.pose.position.z = waypoints[i].position.z();
      marker.pose.orientation.w = 1.0;

      marker.scale.x = 0.008;
      marker.scale.y = 0.008;
      marker.scale.z = 0.008;

      // 按条带索引着色（交替红蓝）
      if (waypoints[i].band_index % 2 == 0)
      {
        marker.color.r = 1.0;
        marker.color.g = 0.2;
        marker.color.b = 0.2;
      }
      else
      {
        marker.color.r = 0.2;
        marker.color.g = 0.2;
        marker.color.b = 1.0;
      }
      marker.color.a = 0.9;

      marker.lifetime = ros::Duration(0);
      marker_array.markers.push_back(marker);
    }

    // 法向量箭头（每隔若干个点画一个）
    int arrow_step = std::max(1, static_cast<int>(waypoints.size() / 50));
    for (size_t i = 0; i < waypoints.size(); i += arrow_step)
    {
      visualization_msgs::Marker arrow;
      arrow.header.frame_id = "world";
      arrow.header.stamp = ros::Time::now();
      arrow.ns = "spray_normals";
      arrow.id = static_cast<int>(i);
      arrow.type = visualization_msgs::Marker::ARROW;
      arrow.action = visualization_msgs::Marker::ADD;

      geometry_msgs::Point p_start, p_end;
      p_start.x = waypoints[i].surface_point.x();
      p_start.y = waypoints[i].surface_point.y();
      p_start.z = waypoints[i].surface_point.z();

      double arrow_len = 0.05;
      p_end.x = p_start.x + waypoints[i].surface_normal.x() * arrow_len;
      p_end.y = p_start.y + waypoints[i].surface_normal.y() * arrow_len;
      p_end.z = p_start.z + waypoints[i].surface_normal.z() * arrow_len;

      arrow.points.push_back(p_start);
      arrow.points.push_back(p_end);

      arrow.scale.x = 0.004;
      arrow.scale.y = 0.008;
      arrow.scale.z = 0.008;

      arrow.color.r = 0.0;
      arrow.color.g = 1.0;
      arrow.color.b = 0.0;
      arrow.color.a = 0.8;

      arrow.lifetime = ros::Duration(0);
      marker_array.markers.push_back(arrow);
    }

    waypoint_marker_pub_.publish(marker_array);

    // 发布 PoseArray（MoveIt/RViz 可直接显示）
    geometry_msgs::PoseArray pose_array;
    pose_array.header.frame_id = "world";
    pose_array.header.stamp = ros::Time::now();

    for (const auto& wp : waypoints)
    {
      geometry_msgs::Pose pose;
      pose.position.x = wp.position.x();
      pose.position.y = wp.position.y();
      pose.position.z = wp.position.z();
      pose.orientation.x = wp.orientation.x();
      pose.orientation.y = wp.orientation.y();
      pose.orientation.z = wp.orientation.z();
      pose.orientation.w = wp.orientation.w();
      pose_array.poses.push_back(pose);
    }

    pose_array_pub_.publish(pose_array);

    ROS_INFO("Published %zu waypoint RViz markers.", waypoints.size());
  }

  // ---- Member variables ----
  ros::NodeHandle nh_;

  // MoveIt 参数（planning_group_ 必须在 move_group_ 之前声明，因为初始化列表中使用）
  std::string planning_group_;
  moveit::planning_interface::MoveGroupInterface move_group_;

  std::string end_effector_link_;
  double planning_time_;
  int max_attempts_;
  double position_tolerance_;
  double goal_orientation_tolerance_;

  // Cartesian 路径参数
  double eef_step_;
  double jump_threshold_;
  double min_cartesian_fraction_;

  // 执行参数
  bool auto_execute_;
  bool visualize_;

  // IK 预检查参数
  double orientation_tolerance_;
  double ik_check_timeout_;
  int relaxation_attempts_;
  int min_segment_size_;

  // 碰撞参数
  bool allow_spray_door_collision_;

  // STL 参数
  std::string stl_filepath_;
  double mesh_scale_;

  // 车门位姿
  double door_x_, door_y_, door_z_;
  double door_roll_, door_pitch_, door_yaw_;

  // 网格偏移
  double mesh_offset_x_, mesh_offset_y_, mesh_offset_z_;

  // 喷涂参数
  double band_spacing_;
  double sample_step_;
  double standoff_distance_;
  double boundary_margin_;
  bool flip_normals_;

  // 发布者
  ros::Publisher waypoint_marker_pub_;
  ros::Publisher pose_array_pub_;
  ros::Publisher planning_scene_pub_;
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "curved_surface_spray_node");
  ros::AsyncSpinner spinner(2);
  spinner.start();

  CurvedSurfaceSprayNode node;

  // 等待 MoveIt 就绪
  ros::Duration(2.0).sleep();

  bool success = node.run();
  if (success)
  {
    ROS_INFO("Curved-surface spray task completed successfully.");
  }
  else
  {
    ROS_ERROR("Curved-surface spray task failed.");
  }

  ros::waitForShutdown();
  return 0;
}
