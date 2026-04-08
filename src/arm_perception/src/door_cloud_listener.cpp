#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/PoseStamped.h>

#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Vector3.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <pcl/filters/filter.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/extract_indices.h>

#include <pcl/ModelCoefficients.h>
#include <pcl/PointIndices.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/common/centroid.h>

class DoorCloudListener
{
public:

    DoorCloudListener()
  : tf_listener_(tf_buffer_)
    {
        ros::NodeHandle pnh("~");
        pnh.param<double>("approach_offset", approach_offset_, 0.15);
        pnh.param<double>("max_reach_radius", max_reach_radius_, 0.85);

        sub_ = nh_.subscribe("/camera/depth/points", 1, &DoorCloudListener::cloudCallback, this);

        center_pub_ = nh_.advertise<visualization_msgs::Marker>("door_plane_center_marker", 1);
        normal_pub_ = nh_.advertise<visualization_msgs::Marker>("door_plane_normal_marker", 1);

        pose_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("door_target_pose", 1);
        pose_base_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("door_target_pose_base", 1);
        approach_pose_base_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("door_approach_pose_base", 1);

        ROS_INFO("door_cloud_listener started, waiting for /camera/depth/points ...");
        ROS_INFO("approach_offset: %.3f", approach_offset_);
        ROS_INFO("max_reach_radius: %.3f", max_reach_radius_);
    }
    

private:
    void publishCenterMarker(const pcl::PointXYZ& center, const std::string& frame_id)
    {
        visualization_msgs::Marker marker;
        marker.header.frame_id = frame_id;
        marker.header.stamp = ros::Time::now();
        marker.ns = "door_plane_center";
        marker.id = 0;
        marker.type = visualization_msgs::Marker::SPHERE;
        marker.action = visualization_msgs::Marker::ADD;

        marker.pose.position.x = center.x;
        marker.pose.position.y = center.y;
        marker.pose.position.z = center.z;
        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;
        marker.pose.orientation.w = 1.0;

        marker.scale.x = 0.03;
        marker.scale.y = 0.03;
        marker.scale.z = 0.03;

        marker.color.r = 1.0;
        marker.color.g = 0.0;
        marker.color.b = 0.0;
        marker.color.a = 1.0;

        marker.lifetime = ros::Duration(0.2);
        center_pub_.publish(marker);
    }

    void publishNormalMarker(const pcl::PointXYZ& center,
                             const pcl::ModelCoefficients::Ptr& coefficients,
                             const std::string& frame_id)
    {
        visualization_msgs::Marker marker;
        marker.header.frame_id = frame_id;
        marker.header.stamp = ros::Time::now();
        marker.ns = "door_plane_normal";
        marker.id = 1;
        marker.type = visualization_msgs::Marker::ARROW;
        marker.action = visualization_msgs::Marker::ADD;

        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;
        marker.pose.orientation.w = 1.0;

        geometry_msgs::Point p_start, p_end;
        p_start.x = center.x;
        p_start.y = center.y;
        p_start.z = center.z;

        double scale = 0.10;
        p_end.x = center.x + coefficients->values[0] * scale;
        p_end.y = center.y + coefficients->values[1] * scale;
        p_end.z = center.z + coefficients->values[2] * scale;

        marker.points.push_back(p_start);
        marker.points.push_back(p_end);

        marker.scale.x = 0.01;
        marker.scale.y = 0.02;
        marker.scale.z = 0.02;

        marker.color.r = 0.0;
        marker.color.g = 1.0;
        marker.color.b = 0.0;
        marker.color.a = 1.0;

        marker.lifetime = ros::Duration(0.2);
        normal_pub_.publish(marker);
    }

    geometry_msgs::PoseStamped makeCameraPose(const pcl::PointXYZ& center,
                                              const std::string& frame_id)
    {
        geometry_msgs::PoseStamped pose_msg;
        pose_msg.header.frame_id = frame_id;
        pose_msg.header.stamp = ros::Time::now();

        pose_msg.pose.position.x = center.x;
        pose_msg.pose.position.y = center.y;
        pose_msg.pose.position.z = center.z;

        pose_msg.pose.orientation.x = 0.0;
        pose_msg.pose.orientation.y = 0.0;
        pose_msg.pose.orientation.z = 0.0;
        pose_msg.pose.orientation.w = 1.0;

        return pose_msg;
    }

    geometry_msgs::PoseStamped makeCameraApproachPose(const pcl::PointXYZ& center,
                                                      const pcl::ModelCoefficients::Ptr& coefficients,
                                                      const std::string& frame_id)
    {
        geometry_msgs::PoseStamped pose_msg;
        pose_msg.header.frame_id = frame_id;
        pose_msg.header.stamp = ros::Time::now();

        double nx = coefficients->values[0];
        double ny = coefficients->values[1];
        double nz = coefficients->values[2];

        double norm = std::sqrt(nx * nx + ny * ny + nz * nz);
        if (norm < 1e-6)
        {
            nx = 0.0;
            ny = 0.0;
            nz = 1.0;
            norm = 1.0;
        }
        nx /= norm;
        ny /= norm;
        nz /= norm;

        // 沿法向量反方向退后 0.15m
        double offset = 0.15;
        pose_msg.pose.position.x = center.x - nx * offset;
        pose_msg.pose.position.y = center.y - ny * offset;
        pose_msg.pose.position.z = center.z - nz * offset;

        pose_msg.pose.orientation.x = 0.0;
        pose_msg.pose.orientation.y = 0.0;
        pose_msg.pose.orientation.z = 0.0;
        pose_msg.pose.orientation.w = 1.0;

        return pose_msg;
    }

    bool transformPose(const std::string& target_frame,
                       const geometry_msgs::PoseStamped& input_pose,
                       geometry_msgs::PoseStamped& output_pose)
    {
        try
        {
            geometry_msgs::TransformStamped transform =
                tf_buffer_.lookupTransform(target_frame,
                                           input_pose.header.frame_id,
                                           ros::Time(0),
                                           ros::Duration(0.2));

            tf2::doTransform(input_pose, output_pose, transform);
            output_pose.header.stamp = ros::Time::now();
            return true;
        }
        catch (tf2::TransformException& ex)
        {
            ROS_WARN_THROTTLE(1.0, "TF transform to %s failed: %s",
                              target_frame.c_str(), ex.what());
            return false;
        }
    }

    geometry_msgs::Quaternion buildQuaternionFromNormal(double nx, double ny, double nz)
    {
        // 让工具 x 轴指向“车门法向量的反方向”，即喷枪朝向车门
        tf2::Vector3 x_axis(-nx, -ny, -nz);
        if (x_axis.length() < 1e-6)
        {
            x_axis = tf2::Vector3(1.0, 0.0, 0.0);
        }
        x_axis.normalize();

        // 选一个参考“上方向”
        tf2::Vector3 world_up(0.0, 0.0, 1.0);

        // 如果 x_axis 和 world_up 太接近，换一个参考轴
        if (std::fabs(x_axis.dot(world_up)) > 0.95)
        {
            world_up = tf2::Vector3(0.0, 1.0, 0.0);
        }

        // 构造正交基
        tf2::Vector3 y_axis = world_up.cross(x_axis);
        y_axis.normalize();

        tf2::Vector3 z_axis = x_axis.cross(y_axis);
        z_axis.normalize();

        // 列向量分别为 x, y, z
        tf2::Matrix3x3 rot(
            x_axis.x(), y_axis.x(), z_axis.x(),
            x_axis.y(), y_axis.y(), z_axis.y(),
            x_axis.z(), y_axis.z(), z_axis.z());

        tf2::Quaternion q;
        rot.getRotation(q);
        q.normalize();

        geometry_msgs::Quaternion q_msg;
        q_msg.x = q.x();
        q_msg.y = q.y();
        q_msg.z = q.z();
        q_msg.w = q.w();
        return q_msg;
    }

    void cloudCallback(const sensor_msgs::PointCloud2ConstPtr& msg)
    {
        // 1. PointCloud2 -> PCL
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromROSMsg(*msg, *cloud);

        // 2. Remove NaN
        std::vector<int> indices;
        pcl::removeNaNFromPointCloud(*cloud, *cloud, indices);
        size_t valid_points = cloud->points.size();

        // 3. ROI filtering
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_z(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PassThrough<pcl::PointXYZ> pass_z;
        pass_z.setInputCloud(cloud);
        pass_z.setFilterFieldName("z");
        pass_z.setFilterLimits(0.1, 1.5);
        pass_z.filter(*cloud_z);

        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_x(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PassThrough<pcl::PointXYZ> pass_x;
        pass_x.setInputCloud(cloud_z);
        pass_x.setFilterFieldName("x");
        pass_x.setFilterLimits(-0.6, 0.6);
        pass_x.filter(*cloud_x);

        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_roi(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PassThrough<pcl::PointXYZ> pass_y;
        pass_y.setInputCloud(cloud_x);
        pass_y.setFilterFieldName("y");
        pass_y.setFilterLimits(-0.6, 0.6);
        pass_y.filter(*cloud_roi);

        if (cloud_roi->points.empty())
        {
            ROS_WARN_THROTTLE(1.0, "ROI cloud is empty after filtering.");
            return;
        }

        // 4. Plane segmentation
        pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
        pcl::PointIndices::Ptr inliers(new pcl::PointIndices);

        pcl::SACSegmentation<pcl::PointXYZ> seg;
        seg.setOptimizeCoefficients(true);
        seg.setModelType(pcl::SACMODEL_PLANE);
        seg.setMethodType(pcl::SAC_RANSAC);
        seg.setDistanceThreshold(0.01);
        seg.setInputCloud(cloud_roi);
        seg.segment(*inliers, *coefficients);

        if (inliers->indices.empty())
        {
            ROS_WARN_THROTTLE(1.0, "RANSAC found no plane.");
            return;
        }

        // 5. Extract plane cloud
        pcl::PointCloud<pcl::PointXYZ>::Ptr plane_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::ExtractIndices<pcl::PointXYZ> extract;
        extract.setInputCloud(cloud_roi);
        extract.setIndices(inliers);
        extract.setNegative(false);
        extract.filter(*plane_cloud);

        if (plane_cloud->points.empty())
        {
            ROS_WARN_THROTTLE(1.0, "Extracted plane cloud is empty.");
            return;
        }

        // 6. Compute centroid
        Eigen::Vector4f centroid;
        pcl::compute3DCentroid(*plane_cloud, centroid);

        pcl::PointXYZ center;
        center.x = centroid[0];
        center.y = centroid[1];
        center.z = centroid[2];

        double nx = coefficients->values[0];
        double ny = coefficients->values[1];
        double nz = coefficients->values[2];
        double norm = std::sqrt(nx * nx + ny * ny + nz * nz);
        if (norm < 1e-6) norm = 1.0;
        nx /= norm;
        ny /= norm;
        nz /= norm;

        double dot_to_camera = -(nx * center.x + ny * center.y + nz * center.z);
        if (dot_to_camera < 0)
        {
            nx = -nx;
            ny = -ny;
            nz = -nz;
            coefficients->values[0] = nx;
            coefficients->values[1] = ny;
            coefficients->values[2] = nz;
            ROS_INFO_ONCE("Flipped plane normal to face camera.");
        }

        // 7. Publish markers in camera frame
        publishCenterMarker(center, msg->header.frame_id);
        publishNormalMarker(center, coefficients, msg->header.frame_id);

        // 8. Publish target pose in camera frame
        geometry_msgs::PoseStamped pose_camera = makeCameraPose(center, msg->header.frame_id);
        pose_pub_.publish(pose_camera);

        // 9. Transform target pose to base_link
        geometry_msgs::PoseStamped pose_base;
        if (transformPose("base_link", pose_camera, pose_base))
        {
            pose_base_pub_.publish(pose_base);
        }

                // 10. 在 camera 坐标系中构造法向量末端点，用于变换到 base_link
        geometry_msgs::PoseStamped normal_tip_camera;
        normal_tip_camera.header.frame_id = msg->header.frame_id;
        normal_tip_camera.header.stamp = ros::Time::now();
        normal_tip_camera.pose.position.x = center.x + nx * 0.1;
        normal_tip_camera.pose.position.y = center.y + ny * 0.1;
        normal_tip_camera.pose.position.z = center.z + nz * 0.1;
        normal_tip_camera.pose.orientation.x = 0.0;
        normal_tip_camera.pose.orientation.y = 0.0;
        normal_tip_camera.pose.orientation.z = 0.0;
        normal_tip_camera.pose.orientation.w = 1.0;

        // 11. 将法向量末端点变换到 base_link
        geometry_msgs::PoseStamped normal_tip_base;
        if (transformPose("base_link", normal_tip_camera, normal_tip_base))
        {
            // 门中心（base_link）
            double cx = pose_base.pose.position.x;
            double cy = pose_base.pose.position.y;
            double cz = pose_base.pose.position.z;

            // 门法向量（base_link）
            double nbx = normal_tip_base.pose.position.x - pose_base.pose.position.x;
            double nby = normal_tip_base.pose.position.y - pose_base.pose.position.y;
            double nbz = normal_tip_base.pose.position.z - pose_base.pose.position.z;

            double nnorm = std::sqrt(nbx * nbx + nby * nby + nbz * nbz);
            if (nnorm < 1e-6)
            {
                ROS_WARN_THROTTLE(1.0, "Normal in base frame is too small.");
                return;
            }
            nbx /= nnorm;
            nby /= nnorm;
            nbz /= nnorm;

            // 12. 自动选择“朝机械臂这一侧”的法向量
            // base_link 原点就在机械臂基座附近，因此从门中心指向原点的方向就是“朝机械臂”
            double to_robot_x = -cx;
            double to_robot_y = -cy;
            double to_robot_z = -cz;

            double dot = nbx * to_robot_x + nby * to_robot_y + nbz * to_robot_z;

            // 如果当前法向量不是朝机械臂这一侧，就翻转
            if (dot < 0.0)
            {
                nbx = -nbx;
                nby = -nby;
                nbz = -nbz;
            }

            // 13. 在 base_link 下生成接近位姿
            geometry_msgs::PoseStamped approach_pose_base;
            approach_pose_base.header.frame_id = "base_link";
            approach_pose_base.header.stamp = ros::Time::now();

            // 喷涂距离：门中心沿“朝机械臂”这一侧的法向量偏移 offset
            double offset = approach_offset_;
            approach_pose_base.pose.position.x = cx + nbx * offset;
            approach_pose_base.pose.position.y = cy + nby * offset;
            approach_pose_base.pose.position.z = cz + nbz * offset;

            // 用朝机械臂一侧的法向量构造姿态
            // buildQuaternionFromNormal 内部会让工具 +X 指向门面
            approach_pose_base.pose.orientation = buildQuaternionFromNormal(nbx, nby, nbz);

            approach_pose_base_pub_.publish(approach_pose_base);

            ROS_INFO_THROTTLE(
                1.0,
                "approach_pose_base: pos=(%.3f, %.3f, %.3f), quat=(%.3f, %.3f, %.3f, %.3f)",
                approach_pose_base.pose.position.x,
                approach_pose_base.pose.position.y,
                approach_pose_base.pose.position.z,
                approach_pose_base.pose.orientation.x,
                approach_pose_base.pose.orientation.y,
                approach_pose_base.pose.orientation.z,
                approach_pose_base.pose.orientation.w);
        }

        ROS_INFO_THROTTLE(
            1.0,
            "frame=%s total=%zu valid=%zu roi=%zu plane_inliers=%zu center_cam=(%.3f, %.3f, %.3f) normal_cam=(%.4f, %.4f, %.4f)",
            msg->header.frame_id.c_str(),
            static_cast<size_t>(msg->width) * static_cast<size_t>(msg->height),
            valid_points,
            cloud_roi->points.size(),
            plane_cloud->points.size(),
            center.x, center.y, center.z,
            nx, ny, nz);
    }

    ros::NodeHandle nh_;
    ros::Subscriber sub_;
    ros::Publisher center_pub_;
    ros::Publisher normal_pub_;
    ros::Publisher pose_pub_;
    ros::Publisher pose_base_pub_;
    ros::Publisher approach_pose_base_pub_;

    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;

    double approach_offset_;

    double max_reach_radius_;
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "door_cloud_listener");
    DoorCloudListener listener;
    ros::spin();
    return 0;
}
