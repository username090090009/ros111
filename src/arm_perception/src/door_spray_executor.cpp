#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
#include <vector>
#include <string>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/MoveItErrorCodes.h>
#include <moveit_msgs/RobotTrajectory.h>

class DoorSprayExecutor
{
public:
    DoorSprayExecutor()
      : move_group_("arm"),
        target_received_(false),
        executed_once_(false),
        execution_in_progress_(false)
    {
        ros::NodeHandle pnh("~");

        pnh.param<std::string>("planning_group", planning_group_, "arm");
        pnh.param<std::string>("end_effector_link", end_effector_link_, "spray_tcp_link");

        pnh.param<double>("position_tolerance", position_tolerance_, 0.02);
        pnh.param<double>("orientation_tolerance", orientation_tolerance_, 0.30);
        pnh.param<double>("planning_time", planning_time_, 15.0);
        pnh.param<int>("max_attempts", max_attempts_, 20);

        // 喷涂参数：主方向从上往下
        pnh.param<double>("spray_width", spray_width_, 0.15);     // 左右短摆
        pnh.param<double>("spray_height", spray_height_, 0.20);   // 上下主方向
        pnh.param<double>("line_spacing", line_spacing_, 0.02);   // 每行向下步进

        // Cartesian 路径参数
        pnh.param<double>("eef_step", eef_step_, 0.01);
        pnh.param<double>("jump_threshold", jump_threshold_, 0.0);
        pnh.param<double>("min_cartesian_fraction", min_cartesian_fraction_, 0.85);

        move_group_.setPlanningTime(planning_time_);
        move_group_.setNumPlanningAttempts(max_attempts_);
        move_group_.setGoalPositionTolerance(position_tolerance_);
        move_group_.setGoalOrientationTolerance(orientation_tolerance_);
        move_group_.setEndEffectorLink(end_effector_link_);
        move_group_.allowReplanning(true);

        sub_ = nh_.subscribe("/door_approach_pose_base", 1, &DoorSprayExecutor::targetCallback, this);

        ROS_INFO("door_spray_executor started.");
        ROS_INFO("Planning group: %s", planning_group_.c_str());
        ROS_INFO("End effector link: %s", end_effector_link_.c_str());
        ROS_INFO("Cartesian spray mode enabled.");
        ROS_INFO("Spray width: %.3f, height: %.3f, spacing: %.3f",
                 spray_width_, spray_height_, line_spacing_);
        ROS_INFO("eef_step: %.3f, jump_threshold: %.3f, min_fraction: %.2f",
                 eef_step_, jump_threshold_, min_cartesian_fraction_);
        ROS_INFO("Waiting for /door_approach_pose_base ...");
    }

private:
    void targetCallback(const geometry_msgs::PoseStampedConstPtr& msg)
    {
        latest_target_ = *msg;
        target_received_ = true;

        ROS_INFO("Received spray center pose in frame [%s]: pos=(%.3f, %.3f, %.3f)",
                latest_target_.header.frame_id.c_str(),
                latest_target_.pose.position.x,
                latest_target_.pose.position.y,
                latest_target_.pose.position.z);

        if (executed_once_)
        {
            ROS_INFO_THROTTLE(2.0, "Spray task already completed successfully. Ignoring repeated updates.");
            return;
        }

        if (execution_in_progress_)
        {
            ROS_INFO_THROTTLE(1.0, "Spray execution already in progress.");
            return;
        }

        execution_in_progress_ = true;
        bool ok = executeSprayPath();
        execution_in_progress_ = false;

        if (ok)
        {
            executed_once_ = true;
            ROS_INFO("Spray task finished successfully.");
        }
        else
        {
            ROS_WARN("Spray task failed. Will retry when next target update arrives.");
        }
    }

    std::vector<geometry_msgs::Pose> generateVerticalSnakeWaypoints(const geometry_msgs::PoseStamped& center_pose)
    {
        std::vector<geometry_msgs::Pose> waypoints;

        double center_x = center_pose.pose.position.x;
        double center_y = center_pose.pose.position.y;
        double center_z = center_pose.pose.position.z;
        geometry_msgs::Quaternion fixed_orientation;
        fixed_orientation.x=0.0;
        fixed_orientation.y=0.0;
        fixed_orientation.z=0.0;
        fixed_orientation.w=1.0;

        int num_lines = static_cast<int>(spray_height_ / line_spacing_) + 1;

        double y_left  = center_y - spray_width_ / 2.0;
        double y_right = center_y + spray_width_ / 2.0;
        double z_top   = center_z + spray_height_ / 2.0;

        for (int i = 0; i < num_lines; ++i)
        {
            double z = z_top - i * line_spacing_;

            geometry_msgs::Pose p1, p2;
            p1.position.x = center_x;
            p1.position.z = z;
            p1.orientation = fixed_orientation;

            p2.position.x = center_x;
            p2.position.z = z;
            p2.orientation = fixed_orientation;

            if (i % 2 == 0)
            {
                p1.position.y = y_left;
                p2.position.y = y_right;
            }
            else
            {
                p1.position.y = y_right;
                p2.position.y = y_left;
            }

            waypoints.push_back(p1);
            waypoints.push_back(p2);
        }

        return waypoints;
    }

    bool moveToPose(const geometry_msgs::Pose& target_pose, const std::string& name)
    {
        move_group_.clearPoseTargets();
        move_group_.setStartStateToCurrentState();
        move_group_.setPoseReferenceFrame(latest_target_.header.frame_id);
        move_group_.setPoseTarget(target_pose, end_effector_link_);

        ROS_INFO("Planning to %s: pos=(%.3f, %.3f, %.3f)",
                 name.c_str(),
                 target_pose.position.x,
                 target_pose.position.y,
                 target_pose.position.z);

        moveit::planning_interface::MoveGroupInterface::Plan plan;
        moveit::planning_interface::MoveItErrorCode success = move_group_.plan(plan);

        if (success == moveit::planning_interface::MoveItErrorCode::SUCCESS)
        {
            ROS_INFO("%s planning succeeded. Executing...", name.c_str());

            moveit::planning_interface::MoveItErrorCode exec_result = move_group_.execute(plan);

            if (exec_result == moveit::planning_interface::MoveItErrorCode::SUCCESS)
            {
                ROS_INFO("%s execution succeeded.", name.c_str());
                return true;
            }
            else
            {
                ROS_WARN("%s execution failed.", name.c_str());
                return false;
            }
        }
        else
        {
            ROS_WARN("%s planning failed.", name.c_str());
            return false;
        }
    }

    bool executeCartesianSpray(const std::vector<geometry_msgs::Pose>& spray_waypoints)
    {
        if (spray_waypoints.empty())
        {
            ROS_WARN("No spray waypoints generated.");
            return false;
        }

        moveit_msgs::RobotTrajectory trajectory;
        const bool avoid_collisions = true;

        // Update start state to current robot state before Cartesian planning
        move_group_.setStartStateToCurrentState();

        ROS_INFO("Computing Cartesian path for %zu spray waypoints...", spray_waypoints.size());

        double fraction = move_group_.computeCartesianPath(
            spray_waypoints,
            eef_step_,
            jump_threshold_,
            trajectory,
            avoid_collisions);

        ROS_INFO("Cartesian path fraction: %.3f", fraction);

        if (fraction < min_cartesian_fraction_)
        {
            ROS_WARN("Cartesian path fraction too low (%.3f < %.3f). Abort execution.",
                     fraction, min_cartesian_fraction_);
            return false;
        }

        moveit::planning_interface::MoveGroupInterface::Plan cartesian_plan;
        cartesian_plan.trajectory_ = trajectory;

        ROS_INFO("Executing Cartesian spray trajectory...");
        moveit::planning_interface::MoveItErrorCode exec_result = move_group_.execute(cartesian_plan);

        if (exec_result == moveit::planning_interface::MoveItErrorCode::SUCCESS)
        {
            ROS_INFO("Cartesian spray execution succeeded.");
            return true;
        }
        else
        {
            ROS_WARN("Cartesian spray execution failed.");
            return false;
        }
    }

    bool executeSprayPath()
    {
        

        if (!target_received_)
        {
            ROS_WARN("No target received yet.");
            return false;
        }

        // 1. 先到喷涂中心/起始接近位姿
        geometry_msgs::Pose center_pose = latest_target_.pose;
        ROS_INFO("Spray target: pos=(%.3f, %.3f, %.3f), quat=(%.3f, %.3f, %.3f, %.3f)",
                 center_pose.position.x, center_pose.position.y, center_pose.position.z,
                 center_pose.orientation.x, center_pose.orientation.y,
                 center_pose.orientation.z, center_pose.orientation.w);

        // center_pose.position.x-=0.10;

        if (!moveToPose(center_pose, "spray_center"))
        {
            ROS_WARN("Cannot reach spray center with pose constraint. Stop.");
            return false;
        }

        ros::Duration(0.5).sleep();

        // 2. 生成从上往下的蛇形路径
        std::vector<geometry_msgs::Pose> spray_waypoints =
            generateVerticalSnakeWaypoints(latest_target_);

        ROS_INFO("Generated %zu vertical snake spray waypoints.", spray_waypoints.size());

        // 3. 先移动到第一个喷涂点（普通规划）
        if (!spray_waypoints.empty())
        {
            if (!moveToPose(spray_waypoints.front(), "spray_start"))
            {
                ROS_WARN("Cannot reach spray start pose. Stop.");
                return false;
            }
            ros::Duration(0.5).sleep();
        }

        // 4. 从喷涂起始点开始执行 Cartesian Path
        bool ok = executeCartesianSpray(spray_waypoints);
        if (!ok)
        {
            ROS_WARN("Cartesian spray path execution failed.");
            return false;
        }

        ROS_INFO("Vertical snake Cartesian spray path execution completed.");
        return true;
    }

    ros::NodeHandle nh_;
    ros::Subscriber sub_;

    moveit::planning_interface::MoveGroupInterface move_group_;
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface_;

    geometry_msgs::PoseStamped latest_target_;
    bool target_received_;
    bool executed_once_;
    bool execution_in_progress_;

    std::string planning_group_;
    std::string end_effector_link_;

    double position_tolerance_;
    double orientation_tolerance_;
    double planning_time_;
    int max_attempts_;

    double spray_width_;
    double spray_height_;
    double line_spacing_;

    double eef_step_;
    double jump_threshold_;
    double min_cartesian_fraction_;
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "door_spray_executor");
    ros::AsyncSpinner spinner(2);
    spinner.start();

    DoorSprayExecutor executor;
    ros::waitForShutdown();
    return 0;
}
