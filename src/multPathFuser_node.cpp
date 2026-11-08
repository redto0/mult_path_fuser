#include "mult_path_fuser/multPathFuser_node.hpp"

// For _1
using namespace std::placeholders;

mult_path_fuser::mult_path_fuser(const rclcpp::NodeOptions& options) : Node("mult_path_fuser", options) {
    // Parameters
    // float x = this->declare_parameter<float>("foo", -10.0);

    target_frame = this->declare_parameter<std::string>("target_frame", "rear_axle");

    this->path_preferred_subscription = this->create_subscription<nav_msgs::msg::Path>(
        "/path_preferred", rclcpp::QoS(5),
        std::bind(&mult_path_fuser::path_preferred_sub, this, std::placeholders::_1));

    this->path_secondary_subscription = this->create_subscription<nav_msgs::msg::Path>(
        "/path_secondary", rclcpp::QoS(5),
        std::bind(&mult_path_fuser::path_secondary_sub, this, std::placeholders::_1));

    this->combined_path_pub = this->create_publisher<nav_msgs::msg::Path>("/path", 5);

    // tf
    tf_buffer = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    transform_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer);
}

// Store the latest secondary path
void mult_path_fuser::path_secondary_sub(const nav_msgs::msg::Path::SharedPtr msg) {
    // store the lastest secondary path
    // note this only works because the secondary path is published at a lower rate than the other path!
    std::lock_guard<std::mutex> lock(secondary_path_mtx);
    path_secondary = *msg;
}
// Helper to compute 2D distance between poses
double pose_distance(const geometry_msgs::msg::PoseStamped& a, const geometry_msgs::msg::PoseStamped& b) {
    double dx = a.pose.position.x - b.pose.position.x;
    double dy = a.pose.position.y - b.pose.position.y;
    return std::sqrt(dx * dx + dy * dy);
}
// Fuse paths: preferred path wins, append extra secondary points if preferred is shorter
void mult_path_fuser::path_preferred_sub(const nav_msgs::msg::Path::SharedPtr msg) {
    transform_path(*msg, target_frame);
    {
        std::lock_guard<std::mutex> preferred_lock(preferred_path_mtx);
        path_preferred = *msg;  // Start with preferred path
    }
    transform_path(path_preferred, target_frame);

    // Transform secondary path into preferred frame
    nav_msgs::msg::Path secondary_transformed;
    {
        std::lock_guard<std::mutex> secondary_lock(secondary_path_mtx);
        secondary_transformed = path_secondary;
    }
    transform_path(secondary_transformed, target_frame);

    // Start fused path with preferred
    nav_msgs::msg::Path fused_path = path_preferred;

    // Distance threshold in meters
    const double dist_thresh = 0.5;

    if (!secondary_transformed.poses.empty()) {
        // Start from last preferred point
        geometry_msgs::msg::PoseStamped last_preferred_pose = fused_path.poses.back();

        for (const auto& secondary_pose : secondary_transformed.poses) {
            if (pose_distance(last_preferred_pose, secondary_pose) > dist_thresh) {
                fused_path.poses.push_back(secondary_pose);
                last_preferred_pose = secondary_pose;  // Update last point
            }
        }
    }

    // Update preferred path with fused version for next iteration
    path_preferred = fused_path;

    combined_path_pub->publish(fused_path);
}
