/**
 * @file stl_mesh_loader.cpp
 * @brief 二进制 STL 网格加载器实现
 *
 * 二进制 STL 格式：
 *   80 字节 header
 *   4 字节 uint32 三角面片数量
 *   每个三角面片 50 字节：
 *     12 字节 (3 x float32) 法向量
 *     36 字节 (3 x 3 x float32) 三个顶点
 *     2 字节 属性
 */

#include "arm_perception/stl_mesh_loader.h"

#include <fstream>
#include <cstring>
#include <cstdint>
#include <limits>
#include <cmath>

namespace arm_perception
{

bool StlMeshLoader::load(const std::string& filepath, double scale)
{
  std::ifstream file(filepath, std::ios::binary);
  if (!file.is_open())
  {
    return false;
  }

  // 跳过 80 字节 header
  char header[80];
  file.read(header, 80);
  if (!file.good())
  {
    return false;
  }

  // 读取三角面片数量
  uint32_t num_triangles = 0;
  file.read(reinterpret_cast<char*>(&num_triangles), sizeof(uint32_t));
  if (!file.good() || num_triangles == 0)
  {
    return false;
  }

  triangles_.clear();
  triangles_.reserve(num_triangles);

  for (uint32_t i = 0; i < num_triangles; ++i)
  {
    // 读取法向量 (3 x float32)
    float nx, ny, nz;
    file.read(reinterpret_cast<char*>(&nx), sizeof(float));
    file.read(reinterpret_cast<char*>(&ny), sizeof(float));
    file.read(reinterpret_cast<char*>(&nz), sizeof(float));

    Triangle tri;

    // 读取三个顶点 (各 3 x float32)
    for (int v = 0; v < 3; ++v)
    {
      float vx, vy, vz;
      file.read(reinterpret_cast<char*>(&vx), sizeof(float));
      file.read(reinterpret_cast<char*>(&vy), sizeof(float));
      file.read(reinterpret_cast<char*>(&vz), sizeof(float));
      tri.vertices[v] = Eigen::Vector3d(vx * scale, vy * scale, vz * scale);
    }

    // 跳过 2 字节属性
    uint16_t attr;
    file.read(reinterpret_cast<char*>(&attr), sizeof(uint16_t));

    if (!file.good())
    {
      triangles_.clear();
      return false;
    }

    // 使用文件中的法向量（已缩放后不影响方向，但重新归一化）
    Eigen::Vector3d file_normal(nx, ny, nz);
    double norm = file_normal.norm();
    if (norm > 1e-8)
    {
      tri.normal = file_normal.normalized();
    }
    else
    {
      // 如果文件中法向量为零，从顶点叉积重新计算
      Eigen::Vector3d e1 = tri.vertices[1] - tri.vertices[0];
      Eigen::Vector3d e2 = tri.vertices[2] - tri.vertices[0];
      Eigen::Vector3d computed_normal = e1.cross(e2);
      double cn = computed_normal.norm();
      tri.normal = (cn > 1e-12) ? (computed_normal / cn) : Eigen::Vector3d::UnitZ();
    }

    triangles_.push_back(tri);
  }

  updateBounds();
  return true;
}

void StlMeshLoader::applyTransform(const Eigen::Isometry3d& transform)
{
  Eigen::Matrix3d rotation = transform.rotation();

  for (auto& tri : triangles_)
  {
    for (int v = 0; v < 3; ++v)
    {
      tri.vertices[v] = transform * tri.vertices[v];
    }
    // 法向量只旋转，不平移
    tri.normal = rotation * tri.normal;
    tri.normal.normalize();
  }

  updateBounds();
}

double StlMeshLoader::totalArea() const
{
  double area = 0.0;
  for (const auto& tri : triangles_)
  {
    area += tri.area();
  }
  return area;
}

void StlMeshLoader::updateBounds()
{
  if (triangles_.empty())
  {
    bounds_min_.setZero();
    bounds_max_.setZero();
    return;
  }

  bounds_min_.setConstant(std::numeric_limits<double>::max());
  bounds_max_.setConstant(std::numeric_limits<double>::lowest());

  for (const auto& tri : triangles_)
  {
    for (int v = 0; v < 3; ++v)
    {
      bounds_min_ = bounds_min_.cwiseMin(tri.vertices[v]);
      bounds_max_ = bounds_max_.cwiseMax(tri.vertices[v]);
    }
  }
}

}  // namespace arm_perception
