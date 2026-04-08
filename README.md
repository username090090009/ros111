# 车门曲面喷涂轨迹规划系统

基于 ROS1 / MoveIt1 的 6 轴机械臂车门曲面喷涂轨迹规划系统。

## 项目概述

本项目实现了面向真实整车门 STL 网格的**曲面喷涂轨迹规划**，替代了原有的简单平面蛇形喷涂方案。系统直接从车门 STL 模型中提取表面几何信息，在曲面上生成带法向量的喷涂路径点，并通过 MoveIt Cartesian Path 执行喷涂轨迹。

### 核心特性

- **STL 网格曲面轨迹规划**：直接加载车门 STL 文件，沿曲面生成喷涂路径
- **自动法向量计算**：每个喷涂路径点的姿态自动对齐到局部表面法向量
- **水平条带蛇形扫描**：沿 Z 轴从下到上分层切片，交替方向形成蛇形路径
- **可调参数**：条带间距、采样步长、离面距离等均可通过 ROS 参数调节
- **RViz 可视化**：路径点和法向量箭头实时发布为 RViz Marker
- **保留已有资产**：复用现有 ROS1/MoveIt1 工作单元和喷枪建模

## 系统架构

```
arm_description/          # 机械臂 + 喷枪 + 车门 URDF/xacro 模型
├── urdf/
│   ├── arm_description.xacro   # 主 xacro：机械臂 + 喷枪 + 车门关节
│   └── door.xacro              # 车门 STL 网格定义
├── meshes/
│   ├── car_door_visual.stl     # 车门可视网格（SolidWorks 导出，mm 单位）
│   ├── car_door_collision.stl  # 车门碰撞网格
│   └── *.STL                   # 机械臂各 link 网格
└── launch/

arm_perception/           # 感知与轨迹规划
├── include/arm_perception/
│   ├── stl_mesh_loader.h       # STL 二进制网格加载器
│   └── surface_spray_planner.h # 曲面喷涂轨迹规划器
├── src/
│   ├── stl_mesh_loader.cpp             # STL 加载实现
│   ├── surface_spray_planner.cpp       # 曲面规划实现
│   ├── curved_surface_spray_node.cpp   # 【新主节点】曲面喷涂 ROS 节点
│   ├── door_spray_executor.cpp         # 【旧】平面蛇形喷涂（已废弃）
│   ├── door_moveit_executor.cpp        # 【旧】单点 MoveIt 执行器
│   └── door_cloud_listener.cpp         # 点云感知节点

arm_description_moveit_config/  # MoveIt 配置
├── launch/
│   ├── curved_spray_demo.launch  # 【推荐】曲面喷涂完整演示
│   ├── spray_demo.launch         # 旧平面喷涂演示（已废弃）
│   └── ...
└── config/
```

## 快速开始

### 环境要求

- ROS Melodic / Noetic
- MoveIt 1
- Gazebo 9 / 11
- Eigen3
- PCL

### 编译

```bash
cd ~/catkin_ws
catkin_make
# 或
catkin build
source devel/setup.bash
```

### 运行曲面喷涂演示

```bash
# 推荐：一键启动完整演示（Gazebo + MoveIt + 曲面喷涂）
roslaunch arm_description_moveit_config curved_spray_demo.launch
```

### 参数调节

通过 launch 文件参数或命令行参数调节喷涂行为：

```bash
# 加密条带（间距 2cm）+ 增大离面距离
roslaunch arm_description_moveit_config curved_spray_demo.launch \
  band_spacing:=0.02 \
  standoff_distance:=0.25

# 仅规划和可视化，不执行
roslaunch arm_description_moveit_config curved_spray_demo.launch \
  auto_execute:=false

# 翻转法向量（当喷涂面朝向不对时使用）
roslaunch arm_description_moveit_config curved_spray_demo.launch \
  flip_normals:=true
```

## 车门位姿配置说明

### 已修复的问题

原始配置中车门 STL 存在以下问题：
1. **STL 网格原点不在几何中心**：SolidWorks 导出的 STL 原点在整车坐标系中，距车门实际几何中心约 2m
2. **车门未旋转**：车门平面未正对机械臂
3. **Z 方向位置为零**：车门放在地面上，大部分区域不在机械臂可达空间内

### 修复方案

**door.xacro 中的网格居中偏移：**

车门 STL 网格包围盒（缩放后，单位 m）：
- X: 1.429 ~ 2.642 (宽度 ≈ 1.213m)
- Y: -0.858 ~ -0.480 (厚度 ≈ 0.379m)
- Z: 0.367 ~ 1.483 (高度 ≈ 1.116m)
- 几何中心: (2.035, -0.669, 0.925)

通过设置 `visual/collision origin xyz="-2.035 0.669 -0.925"`，将网格居中到 door_link 坐标原点。

**arm_description.xacro 中的关节位姿：**

```xml
<joint name="world_to_door_joint" type="fixed">
  <origin xyz="0.90 0.00 0.50" rpy="0 0 1.5708"/>
</joint>
```

- `X=0.90m`：车门中心距机械臂基座 0.9m（用户设定值）
- `Z=0.50m`：车门中心高度 0.5m，使喷涂区域处于机械臂可达空间
- `yaw=π/2`：旋转 90°，使车门表面正对机械臂

> 注：如果加载后发现车门外表面朝向不对（背面朝向机械臂），可将 yaw 改为 `-1.5708`，
> 或在 launch 文件中设置 `flip_normals:=true`。

## 轨迹规划算法

### 曲面喷涂路径生成流程

1. **加载 STL 网格**：读取二进制 STL 文件，提取三角面片和法向量
2. **坐标变换**：将网格从模型坐标系变换到机械臂基座坐标系（与 URDF 一致）
3. **水平切片**：沿 Z 轴以 `band_spacing` 间隔对网格做水平截面
4. **轮廓采样**：在每个截面上，找到三角面片与水平面的交线，按扫描方向排序后等距采样
5. **法向偏移**：每个采样点沿局部表面法向量偏移 `standoff_distance`，作为喷枪 TCP 位置
6. **姿态生成**：构造喷枪姿态，使 TCP X 轴指向车门表面（法向量反方向）
7. **蛇形路径**：相邻条带交替扫描方向，形成 S 形路径

### 核心参数

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `band_spacing` | 0.03 m | 水平条带间距 |
| `sample_step` | 0.02 m | 条带内路径点间距 |
| `standoff_distance` | 0.18 m | 喷枪到表面距离 |
| `boundary_margin` | 0.02 m | 边界裁剪余量 |
| `eef_step` | 0.01 m | Cartesian 路径插值步长 |
| `min_cartesian_fraction` | 0.70 | 最小路径覆盖率 |

## 后续规划

- [ ] 支持多区域分段喷涂（按可达性自动分区）
- [ ] 集成点云感知自动定位车门（复用 door_cloud_listener）
- [ ] 速度/加速度优化（匀速喷涂约束）
- [ ] 真机部署与碰撞检测验证
- [ ] 支持自定义喷涂区域 ROI 选择

## 许可证

本项目仅供学习研究使用。
