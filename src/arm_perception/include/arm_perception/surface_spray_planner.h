/**
 * @file surface_spray_planner.h
 * @brief 曲面喷涂轨迹规划器
 *
 * 基于 STL 三角网格，在车门曲面上生成喷涂轨迹路径点（waypoints）。
 * 规划流程：
 *   1. 将 STL 网格变换到机械臂基座坐标系
 *   2. 沿指定方向（默认 Z 轴，即竖直方向）将网格切分为若干水平条带
 *   3. 在每个条带内，沿条带方向采样喷涂路径点
 *   4. 每个路径点包含：
 *      - 位置：从网格表面沿法向量偏移 standoff 距离
 *      - 姿态：喷枪 TCP 的 X 轴指向车门表面（即沿法向量反方向）
 *   5. 相邻条带交替方向，形成蛇形（S 形）路径
 */

#ifndef ARM_PERCEPTION_SURFACE_SPRAY_PLANNER_H
#define ARM_PERCEPTION_SURFACE_SPRAY_PLANNER_H

#include <string>
#include <vector>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <geometry_msgs/Pose.h>

#include "arm_perception/stl_mesh_loader.h"

namespace arm_perception
{

/// 喷涂规划参数
struct SprayPlannerParams
{
  /// STL 文件路径
  std::string stl_filepath;

  /// STL 缩放因子（mm->m 时为 0.001）
  double mesh_scale = 0.001;

  /// 车门在世界坐标系中的位姿（与 URDF 中 world_to_door_joint 一致）
  Eigen::Isometry3d door_transform = Eigen::Isometry3d::Identity();

  /// 网格居中偏移（对应 door.xacro 中的 mesh offset，单位 m）
  Eigen::Vector3d mesh_offset = Eigen::Vector3d(-2.035, 0.669, -0.925);

  /// 喷涂条带间距（条带之间的步进距离，单位 m）
  double band_spacing = 0.03;

  /// 条带内采样步长（沿条带方向的路径点间距，单位 m）
  double sample_step = 0.02;

  /// 喷枪与车门表面的法向偏移距离（standoff，单位 m）
  double standoff_distance = 0.18;

  /// 喷涂区域裁剪 — 相对于网格包围盒的边界收缩（单位 m）
  /// 避免在网格边缘处产生不稳定的路径点
  double boundary_margin = 0.02;

  /// 切片方向：0=X, 1=Y, 2=Z（默认沿 Z 切片，即水平条带从下到上）
  int slice_axis = 2;

  /// 条带内扫描方向：在垂直于 slice_axis 的平面内，沿哪个轴扫描
  /// -1 表示自动选择（选跨度最大的轴）
  int sweep_axis = -1;

  /// 法向量翻转：true 时将所有法向量取反
  /// 当喷涂面朝向与默认法向量方向相反时使用
  bool flip_normals = false;
};

/// 单个喷涂路径点
struct SprayWaypoint
{
  Eigen::Vector3d position;     ///< 喷枪 TCP 位置（base_link 坐标系）
  Eigen::Quaterniond orientation; ///< 喷枪 TCP 姿态
  Eigen::Vector3d surface_point; ///< 对应的车门表面点（用于可视化/调试）
  Eigen::Vector3d surface_normal; ///< 对应的表面法向量
  int band_index;                ///< 所属条带索引
};

/**
 * @class SurfaceSprayPlanner
 * @brief 基于 STL 网格的曲面喷涂轨迹规划器
 */
class SurfaceSprayPlanner
{
public:
  SurfaceSprayPlanner() = default;

  /**
   * @brief 初始化规划器：加载 STL 网格并变换到世界坐标系
   * @param params 喷涂规划参数
   * @return 初始化成功返回 true
   */
  bool init(const SprayPlannerParams& params);

  /**
   * @brief 执行曲面喷涂轨迹规划
   * @return 规划成功返回 true
   *
   * 调用后通过 waypoints() 获取规划结果。
   */
  bool plan();

  /// 获取规划生成的路径点列表
  const std::vector<SprayWaypoint>& waypoints() const { return waypoints_; }

  /// 将路径点转换为 MoveIt 可用的 geometry_msgs::Pose 数组
  std::vector<geometry_msgs::Pose> toRosPoses() const;

  /// 获取条带数量
  int numBands() const { return num_bands_; }

  /// 获取路径点总数
  size_t numWaypoints() const { return waypoints_.size(); }

  /// 获取已加载的网格
  const StlMeshLoader& mesh() const { return mesh_; }

private:
  /**
   * @brief 在指定 Z 高度处对网格进行水平切片，获取交线上的采样点及其法向量
   * @param z_height 切片 Z 坐标
   * @param points [输出] 切片上的采样点
   * @param normals [输出] 各采样点对应的表面法向量
   */
  void sliceMeshAtHeight(double z_height,
                         std::vector<Eigen::Vector3d>& points,
                         std::vector<Eigen::Vector3d>& normals) const;

  /**
   * @brief 根据表面法向量构造喷枪姿态四元数
   *
   * 喷枪 TCP 的 X 轴指向车门表面（法向量反方向），
   * Z 轴尽量朝上（世界 Z 方向）。
   */
  static Eigen::Quaterniond buildOrientationFromNormal(const Eigen::Vector3d& normal);

  SprayPlannerParams params_;
  StlMeshLoader mesh_;
  std::vector<SprayWaypoint> waypoints_;
  int num_bands_ = 0;
};

}  // namespace arm_perception

#endif  // ARM_PERCEPTION_SURFACE_SPRAY_PLANNER_H
