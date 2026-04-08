#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/MoveItErrorCodes.h>

class DoorMoveItExecutor
{
public:
    DoorMoveItExecutor()
      : move_group_("arm"),
        target_received_(false),
        executed_once_(false)
    {
        ros::NodeHandle pnh("~");

        pnh.param<std::string>("planning_group", planning_group_, "arm");
        pnh.param<std::string>("end_effector_link", end_effector_link_, "spray_tcp_link");
        pnh.param<double>("position_tolerance", position_tolerance_, 0.02);
        pnh.param<double>("orientation_tolerance", orientation_tolerance_, 0.2);
        pnh.param<double>("planning_time", planning_time_, 8.0);
        pnh.param<int>("max_attempts", max_attempts_, 10);
        pnh.param<bool>("auto_execute", auto_execute_, true);
        pnh.param<bool>("position_only", position_only_, true);

        move_group_.setPlanningTime(planning_time_);
        move_group_.setNumPlanningAttempts(max_attempts_);
        move_group_.setGoalPositionTolerance(position_tolerance_);
        move_group_.setGoalOrientationTolerance(orientation_tolerance_);
        move_group_.setEndEffectorLink(end_effector_link_);
        move_group_.allowReplanning(true);

        sub_ = nh_.subscribe("/door_approach_pose_base", 1, &DoorMoveItExecutor::targetCallback, this);

        ROS_INFO("door_moveit_executor started.");
        ROS_INFO("Planning group: %s", planning_group_.c_str());
        ROS_INFO("End effector link: %s", end_effector_link_.c_str());
        ROS_INFO("Position-only mode: %s", position_only_ ? "true" : "false");
        ROS_INFO("Waiting for /door_approach_pose_base ...");
    }

private:
    void targetCallback(const geometry_msgs::PoseStampedConstPtr& msg)
    {
        latest_target_ = *msg;
        target_received_ = true;

        ROS_INFO("Received target pose in frame [%s]: pos=(%.3f, %.3f, %.3f)",
                 latest_target_.header.frame_id.c_str(),
                 latest_target_.pose.position.x,
                 latest_target_.pose.position.y,
                 latest_target_.pose.position.z);

        if (!executed_once_)
        {
            executed_once_ = true;
            planAndExecute();
        }
        else
        {
            ROS_INFO_THROTTLE(2.0, "Target already processed once. Ignoring repeated updates.");
        }
    }

    void planAndExecute()
    {
        if (!target_received_)
        {
            ROS_WARN("No target received yet.");
            return;
        }

        move_group_.clearPoseTargets();
        move_group_.setStartStateToCurrentState();
        move_group_.setPoseReferenceFrame(latest_target_.header.frame_id);

        if (position_only_)
        {
            geometry_msgs::Point target_point = latest_target_.pose.position;
            move_group_.setPositionTarget(
                target_point.x,
                target_point.y,
                target_point.z,
                end_effector_link_);

            ROS_INFO("Planning in POSITION-ONLY mode to (%.3f, %.3f, %.3f)",
                     target_point.x, target_point.y, target_point.z);
        }
        else
        {
            move_group_.setPoseTarget(latest_target_.pose, end_effector_link_);
            ROS_INFO("Planning in FULL-POSE mode.");
        }

        moveit::planning_interface::MoveGroupInterface::Plan plan;
        moveit::planning_interface::MoveItErrorCode success = move_group_.plan(plan);

        if (success == moveit::planning_interface::MoveItErrorCode::SUCCESS)
        {
            ROS_INFO("Planning succeeded.");

            if (auto_execute_)
            {
                ROS_INFO("Executing plan...");
                moveit::planning_interface::MoveItErrorCode exec_result = move_group_.execute(plan);

                if (exec_result == moveit::planning_interface::MoveItErrorCode::SUCCESS)
                {
                    ROS_INFO("Execution succeeded.");
                }
                else
                {
                    ROS_WARN("Execution failed.");
                }
            }
        }
        else
        {
            ROS_WARN("Planning failed.");

            if (position_only_)
            {
                ROS_WARN("Even position-only planning failed. Likely target is outside reachable workspace.");
            }
            else
            {
                ROS_WARN("Full-pose planning failed. Try position-only mode first.");
            }
        }

        move_group_.clearPoseTargets();
    }

    ros::NodeHandle nh_;
    ros::Subscriber sub_;

    moveit::planning_interface::MoveGroupInterface move_group_;
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface_;

    geometry_msgs::PoseStamped latest_target_;
    bool target_received_;
    bool executed_once_;

    std::string planning_group_;
    std::string end_effector_link_;
    double position_tolerance_;
    double orientation_tolerance_;
    double planning_time_;
    int max_attempts_;
    bool auto_execute_;
    bool position_only_;
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "door_moveit_executor");
    ros::AsyncSpinner spinner(2);
    spinner.start();

    DoorMoveItExecutor executor;
    ros::waitForShutdown();
    return 0;
}
