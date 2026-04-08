/**
 * @file curved_surface_spray_node.cpp
 * @brief 曲面喷涂轨迹规划与执行 ROS 节点
 *
 * 替代旧的 door_spray_executor（简单平面蛇形喷涂），
 * 实现基于 STL 网格的真实车门曲面喷涂轨迹规划。
 *
 * 工作流程：
 *   1. 从参数服务器读取车门 STL 路径、缩放、位姿等参数
 *   2. 调用 SurfaceSprayPlanner 在曲面上生成喷涂路径点
 *   3. 将路径点发布为 RViz 可视化 Marker（可选）
 *   4. 使用 MoveIt Cartesian Path 执行喷涂轨迹
 *
 * ROS 参数（通过 launch 文件或命令行设置）：
 *   ~stl_filepath        : 车门 STL 文件路径
 *   ~mesh_scale          : STL 缩放因子（默认 0.001）
 *   ~door_x/y/z          : 车门中心在世界坐标系中的位置（与 URDF 一致）
 *   ~door_roll/pitch/yaw : 车门朝向
 *   ~mesh_offset_x/y/z   : STL 网格居中偏移
 *   ~band_spacing         : 条带间距
 *   ~sample_step          : 条带内采样步长
 *   ~standoff_distance    : 喷枪到表面的距离
 *   ~boundary_margin      : 边界裁剪余量
 *   ~flip_normals         : 是否翻转法向量
 *   ~planning_group       : MoveIt 规划组名称
 *   ~end_effector_link    : 末端执行器 link 名称
 *   ~eef_step             : Cartesian 路径插值步长
 *   ~jump_threshold        : Cartesian 路径跳跃阈值
 *   ~min_cartesian_fraction: 最小 Cartesian 路径覆盖率
 *   ~auto_execute         : 规划后是否自动执行
 *   ~visualize            : 是否发布可视化 Marker
 */

#include <ros/ros.h>
#include <ros/package.h>

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseArray.h>
#include <visualization_msgs/MarkerArray.h>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/RobotTrajectory.h>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "arm_perception/stl_mesh_loader.h"
#include "arm_perception/surface_spray_planner.h"

/**
 * @class CurvedSurfaceSprayNode
 * @brief 曲面喷涂主节点
 */
class CurvedSurfaceSprayNode
{
public:
  CurvedSurfaceSprayNode()
    : move_group_("arm")
  {
    ros::NodeHandle pnh("~");

    // ---- MoveIt 参数 ----
    pnh.param<std::string>("planning_group", planning_group_, "arm");
    pnh.param<std::string>("end_effector_link", end_effector_link_, "spray_tcp_link");
    pnh.param<double>("planning_time", planning_time_, 15.0);
    pnh.param<int>("max_attempts", max_attempts_, 20);
    pnh.param<double>("position_tolerance", position_tolerance_, 0.02);
    pnh.param<double>("orientation_tolerance", orientation_tolerance_, 0.30);

    // ---- Cartesian 路径参数 ----
    pnh.param<double>("eef_step", eef_step_, 0.01);
    pnh.param<double>("jump_threshold", jump_threshold_, 0.0);
    pnh.param<double>("min_cartesian_fraction", min_cartesian_fraction_, 0.70);

    // ---- 执行与可视化 ----
    pnh.param<bool>("auto_execute", auto_execute_, true);
    pnh.param<bool>("visualize", visualize_, true);

    // ---- STL 网格参数 ----
    std::string default_stl = ros::package::getPath("arm_description")
                            + "/meshes/car_door_visual.stl";
    pnh.param<std::string>("stl_filepath", stl_filepath_, default_stl);
    pnh.param<double>("mesh_scale", mesh_scale_, 0.001);

    // ---- 车门位姿（需与 URDF world_to_door_joint 一致）----
    pnh.param<double>("door_x", door_x_, 0.90);
    pnh.param<double>("door_y", door_y_, 0.00);
    pnh.param<double>("door_z", door_z_, 0.50);
    pnh.param<double>("door_roll", door_roll_, 0.0);
    pnh.param<double>("door_pitch", door_pitch_, 0.0);
    pnh.param<double>("door_yaw", door_yaw_, 1.5708);

    // ---- 网格居中偏移（需与 door.xacro 中的值一致）----
    pnh.param<double>("mesh_offset_x", mesh_offset_x_, -2.035);
    pnh.param<double>("mesh_offset_y", mesh_offset_y_, 0.669);
    pnh.param<double>("mesh_offset_z", mesh_offset_z_, -0.925);

    // ---- 喷涂规划参数 ----
    pnh.param<double>("band_spacing", band_spacing_, 0.03);
    pnh.param<double>("sample_step", sample_step_, 0.02);
    pnh.param<double>("standoff_distance", standoff_distance_, 0.18);
    pnh.param<double>("boundary_margin", boundary_margin_, 0.02);
    pnh.param<bool>("flip_normals", flip_normals_, false);

    // ---- 配置 MoveIt ----
    move_group_.setPlanningTime(planning_time_);
    move_group_.setNumPlanningAttempts(max_attempts_);
    move_group_.setGoalPositionTolerance(position_tolerance_);
    move_group_.setGoalOrientationTolerance(orientation_tolerance_);
    move_group_.setEndEffectorLink(end_effector_link_);
    move_group_.allowReplanning(true);

    // ---- 发布者 ----
    if (visualize_)
    {
      waypoint_marker_pub_ = nh_.advertise<visualization_msgs::MarkerArray>(
          "spray_waypoint_markers", 1, true);
      pose_array_pub_ = nh_.advertise<geometry_msgs::PoseArray>(
          "spray_pose_array", 1, true);
    }

    ROS_INFO("======================================");
    ROS_INFO("曲面喷涂轨迹规划节点已启动");
    ROS_INFO("======================================");
    ROS_INFO("规划组: %s", planning_group_.c_str());
    ROS_INFO("末端link: %s", end_effector_link_.c_str());
    ROS_INFO("STL 文件: %s", stl_filepath_.c_str());
    ROS_INFO("车门位姿: xyz=(%.3f, %.3f, %.3f) rpy=(%.3f, %.3f, %.3f)",
             door_x_, door_y_, door_z_, door_roll_, door_pitch_, door_yaw_);
    ROS_INFO("网格偏移: (%.3f, %.3f, %.3f)",
             mesh_offset_x_, mesh_offset_y_, mesh_offset_z_);
    ROS_INFO("条带间距: %.3f m, 采样步长: %.3f m, 离面距离: %.3f m",
             band_spacing_, sample_step_, standoff_distance_);
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
    ROS_INFO("正在加载 STL 网格并初始化规划器...");

    if (!planner.init(params))
    {
      ROS_ERROR("规划器初始化失败！请检查 STL 文件路径: %s", stl_filepath_.c_str());
      return false;
    }

    ROS_INFO("网格加载成功：%zu 个三角面片，总面积 %.4f m²",
             planner.mesh().numTriangles(),
             planner.mesh().totalArea());
    ROS_INFO("网格包围盒（世界坐标系）: min=(%.3f, %.3f, %.3f) max=(%.3f, %.3f, %.3f)",
             planner.mesh().boundsMin().x(),
             planner.mesh().boundsMin().y(),
             planner.mesh().boundsMin().z(),
             planner.mesh().boundsMax().x(),
             planner.mesh().boundsMax().y(),
             planner.mesh().boundsMax().z());

    // 3. 执行轨迹规划
    ROS_INFO("正在进行曲面喷涂轨迹规划...");
    if (!planner.plan())
    {
      ROS_ERROR("曲面喷涂轨迹规划失败！");
      return false;
    }

    ROS_INFO("轨迹规划完成：%d 个条带，%zu 个路径点",
             planner.numBands(),
             planner.numWaypoints());

    // 4. 可视化
    if (visualize_)
    {
      publishVisualization(planner);
    }

    // 5. 转换为 MoveIt Pose 并执行
    std::vector<geometry_msgs::Pose> spray_poses = planner.toRosPoses();

    if (spray_poses.empty())
    {
      ROS_WARN("没有生成可用的喷涂路径点。");
      return false;
    }

    if (!auto_execute_)
    {
      ROS_INFO("auto_execute=false，仅发布可视化，不执行轨迹。");
      return true;
    }

    // 6. 先移动到第一个喷涂路径点
    ROS_INFO("正在移动到喷涂起始点...");
    if (!moveToPose(spray_poses.front(), "喷涂起始点"))
    {
      ROS_WARN("无法到达喷涂起始点，终止。");
      return false;
    }
    ros::Duration(0.5).sleep();

    // 7. 按条带分段执行 Cartesian Path
    ROS_INFO("开始执行曲面喷涂轨迹...");
    const auto& waypoints = planner.waypoints();

    int current_band = -1;
    std::vector<geometry_msgs::Pose> band_poses;

    for (size_t i = 0; i < waypoints.size(); ++i)
    {
      if (waypoints[i].band_index != current_band)
      {
        // 执行上一个条带
        if (!band_poses.empty())
        {
          ROS_INFO("执行条带 %d：%zu 个路径点", current_band,
                   band_poses.size());
          if (!executeCartesianBand(band_poses, current_band))
          {
            ROS_WARN("条带 %d 执行失败，跳过继续下一条带", current_band);
          }
          ros::Duration(0.2).sleep();
        }

        current_band = waypoints[i].band_index;
        band_poses.clear();
      }

      geometry_msgs::Pose pose;
      pose.position.x = waypoints[i].position.x();
      pose.position.y = waypoints[i].position.y();
      pose.position.z = waypoints[i].position.z();
      pose.orientation.x = waypoints[i].orientation.x();
      pose.orientation.y = waypoints[i].orientation.y();
      pose.orientation.z = waypoints[i].orientation.z();
      pose.orientation.w = waypoints[i].orientation.w();
      band_poses.push_back(pose);
    }

    // 执行最后一个条带
    if (!band_poses.empty())
    {
      ROS_INFO("执行条带 %d：%zu 个路径点", current_band,
               band_poses.size());
      if (!executeCartesianBand(band_poses, current_band))
      {
        ROS_WARN("条带 %d 执行失败", current_band);
      }
    }

    ROS_INFO("======================================");
    ROS_INFO("曲面喷涂轨迹执行完毕");
    ROS_INFO("======================================");
    return true;
  }

private:
  /**
   * @brief 使用 MoveIt 关节空间规划移动到指定位姿
   */
  bool moveToPose(const geometry_msgs::Pose& target_pose, const std::string& name)
  {
    move_group_.clearPoseTargets();
    move_group_.setStartStateToCurrentState();
    move_group_.setPoseTarget(target_pose, end_effector_link_);

    ROS_INFO("规划到 [%s]: pos=(%.3f, %.3f, %.3f)",
             name.c_str(),
             target_pose.position.x,
             target_pose.position.y,
             target_pose.position.z);

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    auto result = move_group_.plan(plan);

    if (result == moveit::planning_interface::MoveItErrorCode::SUCCESS)
    {
      ROS_INFO("[%s] 规划成功，开始执行...", name.c_str());
      auto exec_result = move_group_.execute(plan);
      if (exec_result == moveit::planning_interface::MoveItErrorCode::SUCCESS)
      {
        ROS_INFO("[%s] 执行成功。", name.c_str());
        return true;
      }
      else
      {
        ROS_WARN("[%s] 执行失败。", name.c_str());
        return false;
      }
    }
    else
    {
      ROS_WARN("[%s] 规划失败。", name.c_str());
      return false;
    }
  }

  /**
   * @brief 使用 MoveIt Cartesian Path 执行一个条带的路径
   */
  bool executeCartesianBand(const std::vector<geometry_msgs::Pose>& band_poses,
                            int band_index)
  {
    if (band_poses.empty())
    {
      return false;
    }

    moveit_msgs::RobotTrajectory trajectory;
    const bool avoid_collisions = true;

    move_group_.setStartStateToCurrentState();

    double fraction = move_group_.computeCartesianPath(
        band_poses, eef_step_, jump_threshold_, trajectory, avoid_collisions);

    ROS_INFO("条带 %d Cartesian 路径覆盖率: %.1f%%", band_index, fraction * 100.0);

    if (fraction < min_cartesian_fraction_)
    {
      ROS_WARN("条带 %d 覆盖率不足 (%.1f%% < %.1f%%)，跳过",
               band_index, fraction * 100.0, min_cartesian_fraction_ * 100.0);
      return false;
    }

    moveit::planning_interface::MoveGroupInterface::Plan cartesian_plan;
    cartesian_plan.trajectory_ = trajectory;

    auto exec_result = move_group_.execute(cartesian_plan);

    if (exec_result == moveit::planning_interface::MoveItErrorCode::SUCCESS)
    {
      ROS_INFO("条带 %d 执行成功。", band_index);
      return true;
    }
    else
    {
      ROS_WARN("条带 %d 执行失败。", band_index);
      return false;
    }
  }

  /**
   * @brief 发布喷涂路径点的 RViz 可视化
   */
  void publishVisualization(const arm_perception::SurfaceSprayPlanner& planner)
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

    ROS_INFO("已发布 %zu 个路径点的 RViz 可视化 Marker", waypoints.size());
  }

  // ---- 成员变量 ----
  ros::NodeHandle nh_;
  moveit::planning_interface::MoveGroupInterface move_group_;

  // MoveIt 参数
  std::string planning_group_;
  std::string end_effector_link_;
  double planning_time_;
  int max_attempts_;
  double position_tolerance_;
  double orientation_tolerance_;

  // Cartesian 路径参数
  double eef_step_;
  double jump_threshold_;
  double min_cartesian_fraction_;

  // 执行参数
  bool auto_execute_;
  bool visualize_;

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
    ROS_INFO("曲面喷涂任务完成。");
  }
  else
  {
    ROS_ERROR("曲面喷涂任务失败。");
  }

  ros::waitForShutdown();
  return 0;
}
