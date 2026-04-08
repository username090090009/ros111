/**
 * @file stl_mesh_loader.h
 * @brief 二进制 STL 网格加载器
 *
 * 从二进制 STL 文件中读取三角面片，提取顶点坐标和面法向量。
 * 支持指定缩放因子（例如 SolidWorks 导出的 mm 单位 STL 需乘 0.001 转换为 m）。
 */

#ifndef ARM_PERCEPTION_STL_MESH_LOADER_H
#define ARM_PERCEPTION_STL_MESH_LOADER_H

#include <string>
#include <vector>
#include <array>
#include <Eigen/Core>
#include <Eigen/Geometry>

namespace arm_perception
{

/// 三角面片结构：包含三个顶点和一个面法向量
struct Triangle
{
  Eigen::Vector3d vertices[3];  ///< 三角形的三个顶点（缩放后，单位：m）
  Eigen::Vector3d normal;       ///< 面法向量（归一化）

  /// 计算三角形面积
  double area() const
  {
    Eigen::Vector3d e1 = vertices[1] - vertices[0];
    Eigen::Vector3d e2 = vertices[2] - vertices[0];
    return 0.5 * e1.cross(e2).norm();
  }

  /// 计算三角形重心
  Eigen::Vector3d centroid() const
  {
    return (vertices[0] + vertices[1] + vertices[2]) / 3.0;
  }
};

/**
 * @class StlMeshLoader
 * @brief 加载二进制 STL 文件并存储三角面片数据
 *
 * 使用方法：
 *   StlMeshLoader loader;
 *   if (loader.load("/path/to/mesh.stl", 0.001)) {
 *     const auto& triangles = loader.triangles();
 *     // 处理三角面片...
 *   }
 */
class StlMeshLoader
{
public:
  StlMeshLoader() = default;

  /**
   * @brief 加载 STL 文件
   * @param filepath STL 文件的绝对路径
   * @param scale 缩放因子（默认 1.0，mm 转 m 时使用 0.001）
   * @return 加载成功返回 true
   */
  bool load(const std::string& filepath, double scale = 1.0);

  /**
   * @brief 对已加载的网格应用刚体变换（旋转+平移）
   * @param transform 4x4 齐次变换矩阵
   *
   * 用于将 STL 网格从模型坐标系变换到机械臂基座坐标系。
   * 调用后 triangles() 返回的数据将处于新坐标系中。
   */
  void applyTransform(const Eigen::Isometry3d& transform);

  /// 获取所有三角面片
  const std::vector<Triangle>& triangles() const { return triangles_; }

  /// 获取网格包围盒最小角点
  const Eigen::Vector3d& boundsMin() const { return bounds_min_; }

  /// 获取网格包围盒最大角点
  const Eigen::Vector3d& boundsMax() const { return bounds_max_; }

  /// 获取网格几何中心
  Eigen::Vector3d center() const { return (bounds_min_ + bounds_max_) * 0.5; }

  /// 获取三角面片数量
  size_t numTriangles() const { return triangles_.size(); }

  /// 获取网格总表面积
  double totalArea() const;

private:
  /// 重新计算包围盒
  void updateBounds();

  std::vector<Triangle> triangles_;
  Eigen::Vector3d bounds_min_;
  Eigen::Vector3d bounds_max_;
};

}  // namespace arm_perception

#endif  // ARM_PERCEPTION_STL_MESH_LOADER_H
