/**
 * @file surface_spray_planner.cpp
 * @brief 曲面喷涂轨迹规划器实现
 *
 * 规划流程：
 *   1. 加载 STL 网格并应用车门位姿变换（与 URDF 一致）
 *   2. 沿 Z 轴从下到上将网格切分为水平条带
 *   3. 在每个条带的 Z 高度处，对三角面片求交线并采样
 *   4. 按扫描方向排序采样点，构建蛇形路径
 *   5. 每个路径点沿表面法向量偏移 standoff 距离，作为喷枪 TCP 位置
 */

#include "arm_perception/surface_spray_planner.h"

#include <algorithm>
#include <cmath>
#include <map>

namespace arm_perception
{

// ============================================================================
// 初始化：加载网格、应用变换
// ============================================================================

bool SurfaceSprayPlanner::init(const SprayPlannerParams& params)
{
  params_ = params;
  waypoints_.clear();
  num_bands_ = 0;

  // 1. 加载 STL
  if (!mesh_.load(params_.stl_filepath, params_.mesh_scale))
  {
    return false;
  }

  // 2. 构造完整变换：先应用网格居中偏移，再应用车门关节位姿
  //    T_world_mesh = T_world_door * T_door_mesh
  //    其中 T_door_mesh 仅为平移（mesh offset）
  Eigen::Isometry3d mesh_offset_transform = Eigen::Isometry3d::Identity();
  mesh_offset_transform.translation() = params_.mesh_offset;

  Eigen::Isometry3d full_transform = params_.door_transform * mesh_offset_transform;

  mesh_.applyTransform(full_transform);

  // 3. 可选：翻转法向量
  if (params_.flip_normals)
  {
    for (auto& tri : mesh_.trianglesMutable())
    {
      tri.normal = -tri.normal;
    }
  }

  return true;
}

// ============================================================================
// 网格切片：在指定 Z 高度处求三角网格的交线
// ============================================================================

void SurfaceSprayPlanner::sliceMeshAtHeight(
    double z_height,
    std::vector<Eigen::Vector3d>& points,
    std::vector<Eigen::Vector3d>& normals) const
{
  points.clear();
  normals.clear();

  for (const auto& tri : mesh_.triangles())
  {
    // 找到与 z = z_height 平面相交的边
    // 对三角形的三条边分别检查
    std::vector<Eigen::Vector3d> intersections;

    for (int e = 0; e < 3; ++e)
    {
      const Eigen::Vector3d& p0 = tri.vertices[e];
      const Eigen::Vector3d& p1 = tri.vertices[(e + 1) % 3];

      double z0 = p0.z();
      double z1 = p1.z();

      // 检查边是否跨越 z_height（严格跨越，排除端点恰好在平面上的情况）
      if ((z0 - z_height) * (z1 - z_height) < 0.0)
      {
        // 线性插值求交点
        double t = (z_height - z0) / (z1 - z0);
        Eigen::Vector3d intersection = p0 + t * (p1 - p0);
        intersections.push_back(intersection);
      }
    }

    // 如果没有严格交叉的边，检查顶点是否恰好在平面上
    if (intersections.empty())
    {
      for (int v = 0; v < 3; ++v)
      {
        if (std::abs(tri.vertices[v].z() - z_height) < 1e-6)
        {
          intersections.push_back(tri.vertices[v]);
        }
      }
      // 去重：顶点在平面上时可能被多次添加
      if (intersections.size() > 2)
      {
        intersections.resize(2);
      }
    }

    // 一个三角面与水平面最多交于两个点（形成线段）
    if (intersections.size() >= 2)
    {
      // 取线段中点作为采样点
      Eigen::Vector3d mid = (intersections[0] + intersections[1]) * 0.5;
      points.push_back(mid);
      normals.push_back(tri.normal);
    }
  }
}

// ============================================================================
// 从法向量构造喷枪姿态
// ============================================================================

Eigen::Quaterniond SurfaceSprayPlanner::buildOrientationFromNormal(
    const Eigen::Vector3d& normal)
{
  // 喷枪 TCP X 轴指向车门表面 = 法向量的反方向
  Eigen::Vector3d x_axis = -normal.normalized();

  // 选择世界 Z 轴（竖直方向）作为参考上方向
  Eigen::Vector3d world_up(0.0, 0.0, 1.0);

  // 如果 X 轴与世界 Z 近乎平行，换用 Y 轴作为参考
  if (std::abs(x_axis.dot(world_up)) > 0.95)
  {
    world_up = Eigen::Vector3d(0.0, 1.0, 0.0);
  }

  // 构造正交坐标系 (x_axis, y_axis, z_axis)
  Eigen::Vector3d y_axis = world_up.cross(x_axis).normalized();
  Eigen::Vector3d z_axis = x_axis.cross(y_axis).normalized();

  // 构造旋转矩阵（列向量为 x, y, z）
  Eigen::Matrix3d rot;
  rot.col(0) = x_axis;
  rot.col(1) = y_axis;
  rot.col(2) = z_axis;

  return Eigen::Quaterniond(rot).normalized();
}

// ============================================================================
// 执行轨迹规划
// ============================================================================

bool SurfaceSprayPlanner::plan()
{
  waypoints_.clear();
  num_bands_ = 0;

  if (mesh_.numTriangles() == 0)
  {
    return false;
  }

  // 获取网格在 Z 方向的范围（已变换到世界坐标系）
  double z_min = mesh_.boundsMin().z() + params_.boundary_margin;
  double z_max = mesh_.boundsMax().z() - params_.boundary_margin;

  if (z_min >= z_max)
  {
    return false;
  }

  // 确定扫描轴（在水平面内选择跨度最大的方向）
  int sweep_axis = params_.sweep_axis;
  if (sweep_axis < 0)
  {
    double x_span = mesh_.boundsMax().x() - mesh_.boundsMin().x();
    double y_span = mesh_.boundsMax().y() - mesh_.boundsMin().y();
    sweep_axis = (y_span >= x_span) ? 1 : 0;
  }

  // 从下到上逐层切片
  int band_index = 0;
  for (double z = z_min; z <= z_max; z += params_.band_spacing)
  {
    std::vector<Eigen::Vector3d> slice_points;
    std::vector<Eigen::Vector3d> slice_normals;

    sliceMeshAtHeight(z, slice_points, slice_normals);

    if (slice_points.empty())
    {
      continue;
    }

    // 按扫描轴方向排序
    std::vector<size_t> indices(slice_points.size());
    for (size_t i = 0; i < indices.size(); ++i)
    {
      indices[i] = i;
    }

    std::sort(indices.begin(), indices.end(),
              [&](size_t a, size_t b) {
                return slice_points[a](sweep_axis) < slice_points[b](sweep_axis);
              });

    // 蛇形路径：偶数条带正序，奇数条带反序
    if (band_index % 2 != 0)
    {
      std::reverse(indices.begin(), indices.end());
    }

    // 沿排序后的点列表进行等距采样
    // 先把排序后的点放入临时容器
    std::vector<Eigen::Vector3d> sorted_points(indices.size());
    std::vector<Eigen::Vector3d> sorted_normals(indices.size());
    for (size_t i = 0; i < indices.size(); ++i)
    {
      sorted_points[i] = slice_points[indices[i]];
      sorted_normals[i] = slice_normals[indices[i]];
    }

    // 等距采样
    if (sorted_points.size() >= 2)
    {
      // 计算切片总长度
      double total_length = 0.0;
      std::vector<double> cumulative_length(sorted_points.size(), 0.0);
      for (size_t i = 1; i < sorted_points.size(); ++i)
      {
        total_length += (sorted_points[i] - sorted_points[i - 1]).norm();
        cumulative_length[i] = total_length;
      }

      if (total_length < 1e-6)
      {
        continue;
      }

      // 按步长采样
      double current_distance = 0.0;
      size_t seg_idx = 0;

      while (current_distance <= total_length)
      {
        // 找到当前距离对应的线段
        while (seg_idx + 1 < sorted_points.size() &&
               cumulative_length[seg_idx + 1] < current_distance)
        {
          ++seg_idx;
        }

        if (seg_idx + 1 >= sorted_points.size())
        {
          break;
        }

        // 在线段上插值
        double seg_start = cumulative_length[seg_idx];
        double seg_end = cumulative_length[seg_idx + 1];
        double seg_len = seg_end - seg_start;
        double t = (seg_len > 1e-9) ? (current_distance - seg_start) / seg_len : 0.0;

        Eigen::Vector3d pt = sorted_points[seg_idx] * (1.0 - t)
                           + sorted_points[seg_idx + 1] * t;
        Eigen::Vector3d nm = (sorted_normals[seg_idx] * (1.0 - t)
                           + sorted_normals[seg_idx + 1] * t).normalized();

        // 构造喷涂路径点
        SprayWaypoint wp;
        wp.surface_point = pt;
        wp.surface_normal = nm;
        wp.position = pt + nm * params_.standoff_distance;
        wp.orientation = buildOrientationFromNormal(nm);
        wp.band_index = band_index;

        waypoints_.push_back(wp);

        current_distance += params_.sample_step;
      }
    }
    else if (sorted_points.size() == 1)
    {
      // 只有一个交点，直接加入
      SprayWaypoint wp;
      wp.surface_point = sorted_points[0];
      wp.surface_normal = sorted_normals[0];
      wp.position = sorted_points[0] + sorted_normals[0] * params_.standoff_distance;
      wp.orientation = buildOrientationFromNormal(sorted_normals[0]);
      wp.band_index = band_index;
      waypoints_.push_back(wp);
    }

    ++band_index;
  }

  num_bands_ = band_index;
  return !waypoints_.empty();
}

// ============================================================================
// 转换为 ROS Pose 数组
// ============================================================================

std::vector<geometry_msgs::Pose> SurfaceSprayPlanner::toRosPoses() const
{
  std::vector<geometry_msgs::Pose> poses;
  poses.reserve(waypoints_.size());

  for (const auto& wp : waypoints_)
  {
    geometry_msgs::Pose pose;
    pose.position.x = wp.position.x();
    pose.position.y = wp.position.y();
    pose.position.z = wp.position.z();
    pose.orientation.x = wp.orientation.x();
    pose.orientation.y = wp.orientation.y();
    pose.orientation.z = wp.orientation.z();
    pose.orientation.w = wp.orientation.w();
    poses.push_back(pose);
  }

  return poses;
}

}  // namespace arm_perception
