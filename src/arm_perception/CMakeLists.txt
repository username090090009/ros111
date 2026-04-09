cmake_minimum_required(VERSION 3.0.2)
project(arm_perception)

## C++14 standard (required by Eigen)
add_compile_options(-std=c++14)

## Find dependencies
find_package(catkin REQUIRED COMPONENTS
  pcl_conversions
  pcl_ros
  roscpp
  roslib
  sensor_msgs
  std_msgs
  visualization_msgs
  geometry_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  moveit_core
  moveit_ros_planning_interface
  moveit_msgs
)

## 查找 Eigen3
find_package(Eigen3 REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES stl_mesh_loader surface_spray_planner
  CATKIN_DEPENDS
    pcl_conversions pcl_ros roscpp roslib
    sensor_msgs std_msgs visualization_msgs geometry_msgs
    tf2 tf2_ros tf2_geometry_msgs
    moveit_core moveit_ros_planning_interface moveit_msgs
)

## Header include paths
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

# ===========================================================================
# Library: STL mesh loader
# ===========================================================================
add_library(stl_mesh_loader
  src/stl_mesh_loader.cpp
)
target_link_libraries(stl_mesh_loader
  ${catkin_LIBRARIES}
)
add_dependencies(stl_mesh_loader
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

# ===========================================================================
# Library: Curved-surface spray trajectory planner
# ===========================================================================
add_library(surface_spray_planner
  src/surface_spray_planner.cpp
)
target_link_libraries(surface_spray_planner
  stl_mesh_loader
  ${catkin_LIBRARIES}
)
add_dependencies(surface_spray_planner
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

# ===========================================================================
# Node: Curved-surface spray planning and execution (main flow)
# ===========================================================================
add_executable(curved_surface_spray_node
  src/curved_surface_spray_node.cpp
)
target_link_libraries(curved_surface_spray_node
  surface_spray_planner
  stl_mesh_loader
  ${catkin_LIBRARIES}
)
add_dependencies(curved_surface_spray_node
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

# ===========================================================================
# Legacy nodes (preserved but no longer the main flow)
# ===========================================================================

# Point cloud perception node
add_executable(door_cloud_listener src/door_cloud_listener.cpp)
target_link_libraries(door_cloud_listener ${catkin_LIBRARIES})
add_dependencies(door_cloud_listener ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

# Legacy single-pose MoveIt executor
add_executable(door_moveit_executor src/door_moveit_executor.cpp)
target_link_libraries(door_moveit_executor ${catkin_LIBRARIES})
add_dependencies(door_moveit_executor ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

# Legacy planar snake spray executor (superseded by curved_surface_spray_node)
add_executable(door_spray_executor src/door_spray_executor.cpp)
target_link_libraries(door_spray_executor ${catkin_LIBRARIES})
add_dependencies(door_spray_executor ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
