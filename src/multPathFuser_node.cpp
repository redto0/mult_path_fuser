#include "mult_path_fuser/multPathFuser_node.hpp"

// For _1
using namespace std::placeholders;

mult_path_fuser::mult_path_fuser(const rclcpp::NodeOptions& options) : Node("mult_path_fuser", options) {
    // Parameters
    // float x = this->declare_parameter<float>("foo", -10.0);

    target_frame = this->declare_parameter<std::string>("target_frame", "rear_axle");
    odom_frame = this->declare_parameter<std::string>("odom_frame", "odom");

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

// Store the latest secondary path and re-fuse
void mult_path_fuser::path_secondary_sub(const nav_msgs::msg::Path::SharedPtr msg) {
    {
        std::lock_guard<std::mutex> lock(secondary_path_mtx);
        path_secondary = *msg;
    }
    fuse_and_publish();
}
// Helper to compute 2D distance between poses
double pose_distance(const geometry_msgs::msg::PoseStamped& a, const geometry_msgs::msg::PoseStamped& b) {
    double dx = a.pose.position.x - b.pose.position.x;
    double dy = a.pose.position.y - b.pose.position.y;
    return std::sqrt(dx * dx + dy * dy);
}

// Store the latest preferred path and re-fuse
void mult_path_fuser::path_preferred_sub(const nav_msgs::msg::Path::SharedPtr msg) {
    {
        std::lock_guard<std::mutex> preferred_lock(preferred_path_mtx);
        path_preferred = *msg;
    }
    fuse_and_publish();
}

// Fuse paths: preferred path wins, append extra secondary points if preferred is shorter.
// Both inputs are treated as async/possibly-stale: each is independently bridged through
// odom_frame to "now" before fusion, so neither side has to be the one that just arrived.
void mult_path_fuser::fuse_and_publish() {
    const rclcpp::Time now = this->get_clock()->now();

    nav_msgs::msg::Path preferred_transformed;
    {
        std::lock_guard<std::mutex> preferred_lock(preferred_path_mtx);
        preferred_transformed = path_preferred;
    }
    transform_path(preferred_transformed, target_frame, now, odom_frame);

    nav_msgs::msg::Path secondary_transformed;
    {
        std::lock_guard<std::mutex> secondary_lock(secondary_path_mtx);
        secondary_transformed = path_secondary;
    }
    transform_path(secondary_transformed, target_frame, now, odom_frame);

    // Start fused path with preferred
    nav_msgs::msg::Path fused_path = preferred_transformed;

    // Distance threshold in meters
    const double dist_thresh = 0.5;

    if (!secondary_transformed.poses.empty()) {
        geometry_msgs::msg::PoseStamped last_preferred_pose;
        bool have_last_pose = !fused_path.poses.empty();
        if (have_last_pose) {
            last_preferred_pose = fused_path.poses.back();
        }

        for (const auto& secondary_pose : secondary_transformed.poses) {
            if (!have_last_pose || pose_distance(last_preferred_pose, secondary_pose) > dist_thresh) {
                fused_path.poses.push_back(secondary_pose);
                last_preferred_pose = secondary_pose;
                have_last_pose = true;
            }
        }
    }

    fused_path.header.frame_id = target_frame;
    fused_path.header.stamp = now;

    combined_path_pub->publish(fused_path);
}
